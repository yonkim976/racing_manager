"""Short-horizon lateral trajectory lattice for live race driving AI."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, hypot
from time import perf_counter

from simulation.collision import (
    BodyPose,
    oriented_body_separation_m,
)
from simulation.track_physics import DRIVING_LINE_RACING, TrackPhysicsProfile
from simulation.track_surface import TrackSurfaceProfile
from simulation.vehicle_physics import (
    FOLLOWING_CONTROL_REACTION_SECONDS,
    FOLLOWING_PREDICTIVE_DECELERATION_MPS2,
    LongitudinalVehiclePhysics,
    VehiclePhysicsModifiers,
)
from simulation.vehicle_dynamics import MAX_ACCELERATION_MPS2, MAX_BRAKING_MPS2

# Owned here so RaceEngine mixins can share the cadence without importing the
# race_engine facade.  1 Hz leaves nine tactical opportunities between routine
# path rebuilds at the 10 Hz tactical decision rate.
LOCAL_TRAJECTORY_PLAN_INTERVAL_SECONDS = 1.00


@dataclass(frozen=True)
class LocalTrajectoryPlannerConfig:
    """Bounded first-stage lattice and prediction settings."""

    candidate_count: int = 5
    lateral_spacing_m: float = 0.75
    horizon_seconds: float = 3.0
    sample_interval_seconds: float = 0.10
    lateral_transition_seconds: float = 1.20
    control_lookahead_seconds: float = 0.45
    maximum_control_lookahead_seconds: float = 0.95
    edge_margin_m: float = 0.35
    traffic_candidate_count: int = 7
    minimum_traffic_clearance_m: float = 0.50
    minimum_path_hold_seconds: float = 0.80
    switch_cost_improvement_threshold: float = 0.05

    def __post_init__(self) -> None:
        if self.candidate_count not in {5, 7, 9}:
            raise ValueError("local trajectory lattice must contain 5, 7, or 9 candidates")
        if self.lateral_spacing_m <= 0.0:
            raise ValueError("local trajectory lateral spacing must be positive")
        if self.horizon_seconds < 2.0 or self.horizon_seconds > 4.0:
            raise ValueError("local trajectory horizon must be between 2 and 4 seconds")
        if self.sample_interval_seconds <= 0.0:
            raise ValueError("local trajectory sample interval must be positive")
        if self.lateral_transition_seconds <= 0.0:
            raise ValueError("local trajectory transition time must be positive")
        if self.control_lookahead_seconds <= 0.0:
            raise ValueError("local trajectory control lookahead must be positive")
        if self.maximum_control_lookahead_seconds < self.control_lookahead_seconds:
            raise ValueError("maximum control lookahead cannot be below its minimum")
        if self.traffic_candidate_count not in {5, 7, 9}:
            raise ValueError("traffic trajectory lattice must contain 5, 7, or 9 candidates")
        if self.minimum_path_hold_seconds < 0.0:
            raise ValueError("minimum local path hold time cannot be negative")
        if self.switch_cost_improvement_threshold < 0.0:
            raise ValueError("local path switch threshold cannot be negative")


@dataclass(frozen=True)
class LocalTrajectoryPlannerWeights:
    """Pace-mode cost weights; hard body and boundary safety stays unweighted."""

    forward_progress: float = 1.0
    extra_path_distance: float = 1.0
    lateral_deviation: float = 1.0
    surface: float = 1.0
    tire_surface: float = 1.0
    traffic: float = 1.0

    def __post_init__(self) -> None:
        if any(
            weight <= 0.0
            for weight in (
                self.forward_progress,
                self.extra_path_distance,
                self.lateral_deviation,
                self.surface,
                self.tire_surface,
                self.traffic,
            )
        ):
            raise ValueError("local trajectory weights must be positive")


@dataclass(frozen=True)
class NearbyVehiclePredictionInput:
    driver_id: int
    total_progress: float
    speed_mps: float
    acceleration_mps2: float
    lateral_offset_m: float
    lateral_speed_mps: float
    target_lateral_offset_m: float
    heading_offset_rad: float
    body_width_m: float
    body_length_m: float
    corner_role: str | None = None
    corner_turn_direction: int = 0
    corner_pair_separation_m: float = 0.0


@dataclass(frozen=True)
class LocalTrajectoryPlanningRequest:
    total_progress: float
    speed_mps: float
    acceleration_mps2: float
    lateral_offset_m: float
    lateral_speed_mps: float
    body_width_m: float
    body_length_m: float
    modifiers: VehiclePhysicsModifiers
    line_name: str = DRIVING_LINE_RACING
    nearby_vehicles: tuple[NearbyVehiclePredictionInput, ...] = ()
    following_driver_id: int | None = None
    following_desired_gap_m: float = 0.0
    following_minimum_gap_m: float = 0.0
    tire_wear: float = 0.0
    tire_lateral_grip: float = 1.0
    tire_braking_grip: float = 1.0
    tire_traction_grip: float = 1.0
    previous_selected_candidate_id: str | None = None
    previous_selected_age_seconds: float = 0.0
    candidate_lateral_biases_m: tuple[float, ...] | None = None
    corner_role: str | None = None
    corner_turn_direction: int = 0
    corner_pair_separation_m: float = 0.0
    weights: LocalTrajectoryPlannerWeights = field(
        default_factory=LocalTrajectoryPlannerWeights
    )


@dataclass(frozen=True)
class LocalTrajectorySample:
    time_seconds: float
    total_progress: float
    speed_mps: float
    acceleration_mps2: float
    target_speed_mps: float
    throttle: float
    brake: float
    lateral_offset_m: float
    lateral_speed_mps: float
    heading_offset_rad: float
    body_pose: BodyPose


@dataclass(frozen=True)
class PredictedOccupancySample:
    time_seconds: float
    body_pose: BodyPose


@dataclass(frozen=True)
class PredictedVehicleOccupancy:
    driver_id: int
    samples: tuple[PredictedOccupancySample, ...]


@dataclass(frozen=True)
class LocalTrajectoryCandidate:
    candidate_id: str
    lateral_bias_m: float
    longitudinal_samples: tuple[_LongitudinalSample, ...]
    lateral_offsets_m: tuple[float, ...]
    lateral_speeds_mps: tuple[float, ...]
    heading_offsets_rad: tuple[float, ...]
    body_width_m: float
    body_length_m: float
    track_length_m: float
    forward_distance_m: float
    path_distance_m: float
    surface_cost_seconds: float
    tire_surface_cost_seconds: float
    body_boundary_violations: int
    predicted_collision_count: int
    first_collision_time_seconds: float | None
    minimum_opponent_clearance_m: float
    conflicting_driver_ids: tuple[int, ...]
    traffic_cost: float
    objective_cost: float
    control_lookahead_seconds: float
    control_target_lateral_offset_m: float
    control_target_lateral_speed_mps: float

    @property
    def viable(self) -> bool:
        return (
            self.body_boundary_violations == 0
            and self.predicted_collision_count == 0
        )

    def body_pose_at(self, index: int) -> BodyPose:
        prediction = self.longitudinal_samples[index]
        return BodyPose(
            longitudinal_m=prediction.total_progress * self.track_length_m,
            lateral_m=self.lateral_offsets_m[index],
            heading_rad=self.heading_offsets_rad[index],
            length_m=self.body_length_m,
            width_m=self.body_width_m,
            longitudinal_speed_mps=prediction.speed_mps,
            lateral_speed_mps=self.lateral_speeds_mps[index],
        )

    @property
    def samples(self) -> tuple[LocalTrajectorySample, ...]:
        """Materialize detailed body occupancy only when a consumer needs it."""
        return tuple(
            LocalTrajectorySample(
                time_seconds=prediction.time_seconds,
                total_progress=prediction.total_progress,
                speed_mps=prediction.speed_mps,
                acceleration_mps2=prediction.acceleration_mps2,
                target_speed_mps=prediction.target_speed_mps,
                throttle=prediction.throttle,
                brake=prediction.brake,
                lateral_offset_m=lateral_offset_m,
                lateral_speed_mps=lateral_speed_mps,
                heading_offset_rad=heading_offset_rad,
                body_pose=self.body_pose_at(index),
            )
            for index, (
                prediction,
                lateral_offset_m,
                lateral_speed_mps,
                heading_offset_rad,
            ) in enumerate(zip(
                self.longitudinal_samples,
                self.lateral_offsets_m,
                self.lateral_speeds_mps,
                self.heading_offsets_rad,
            ))
        )


@dataclass(frozen=True)
class LocalTrajectoryPlan:
    candidates: tuple[LocalTrajectoryCandidate, ...]
    opponent_occupancies: tuple[PredictedVehicleOccupancy, ...]
    selected_candidate_id: str
    selection_reason: str
    horizon_seconds: float
    generation_duration_ms: float = field(compare=False)

    @property
    def selected(self) -> LocalTrajectoryCandidate:
        return next(
            candidate
            for candidate in self.candidates
            if candidate.candidate_id == self.selected_candidate_id
        )


@dataclass(frozen=True)
class _LongitudinalSample:
    time_seconds: float
    total_progress: float
    speed_mps: float
    acceleration_mps2: float
    target_speed_mps: float
    throttle: float
    brake: float


@dataclass(frozen=True)
class _TrajectoryBounds:
    rear_minimum_m: float
    rear_maximum_m: float
    front_minimum_m: float
    front_maximum_m: float
    white_minimum_m: float
    white_maximum_m: float


class LocalTrajectoryPlanner:
    """Generate and score a deterministic short-horizon lateral lattice."""

    def __init__(
        self,
        track_profile: TrackPhysicsProfile,
        surface_profile: TrackSurfaceProfile,
        physics_by_line: dict[str, LongitudinalVehiclePhysics],
        *,
        track_length_m: float,
        config: LocalTrajectoryPlannerConfig | None = None,
    ) -> None:
        self.track_profile = track_profile
        self.surface_profile = surface_profile
        self.physics_by_line = physics_by_line
        self.track_length_m = max(1.0, float(track_length_m))
        self.config = config or LocalTrajectoryPlannerConfig()
        self._boundary_cache: dict[tuple[int, int, int], _TrajectoryBounds] = {}

    def _trajectory_bounds(
        self,
        total_progress: float,
        body_width_m: float,
        body_length_m: float,
    ) -> _TrajectoryBounds:
        distance_bin_m = round((total_progress % 1.0) * self.track_length_m)
        key = (
            distance_bin_m,
            round(body_width_m * 1000.0),
            round(body_length_m * 1000.0),
        )
        cached = self._boundary_cache.get(key)
        if cached is not None:
            return cached
        half_length_progress = body_length_m / 2.0 / self.track_length_m
        rear_minimum_m, rear_maximum_m = (
            self.surface_profile.trajectory_body_lateral_bounds(
                total_progress - half_length_progress,
                body_width_m=body_width_m,
                edge_margin_m=self.config.edge_margin_m,
            )
        )
        front_minimum_m, front_maximum_m = (
            self.surface_profile.trajectory_body_lateral_bounds(
                total_progress + half_length_progress,
                body_width_m=body_width_m,
                edge_margin_m=self.config.edge_margin_m,
            )
        )
        track_sample = self.surface_profile.track_profile.at_progress(total_progress)
        result = _TrajectoryBounds(
            rear_minimum_m=rear_minimum_m,
            rear_maximum_m=rear_maximum_m,
            front_minimum_m=front_minimum_m,
            front_maximum_m=front_maximum_m,
            white_minimum_m=(-track_sample.right_width_m + body_width_m / 2.0),
            white_maximum_m=(track_sample.left_width_m - body_width_m / 2.0),
        )
        if len(self._boundary_cache) >= 4096:
            self._boundary_cache.clear()
        self._boundary_cache[key] = result
        return result

    def lateral_biases_m(self, candidate_count: int | None = None) -> tuple[float, ...]:
        count = candidate_count or self.config.candidate_count
        half_count = count // 2
        return tuple(
            index * self.config.lateral_spacing_m
            for index in range(-half_count, half_count + 1)
        )

    @staticmethod
    def _candidate_id(bias_m: float, spacing_m: float) -> str:
        if abs(bias_m) <= 1e-9:
            return "center"
        rank = max(1, round(abs(bias_m) / spacing_m))
        return f"{'left' if bias_m > 0.0 else 'right'}_{rank}"

    def _predict_longitudinal(
        self,
        request: LocalTrajectoryPlanningRequest,
    ) -> tuple[_LongitudinalSample, ...]:
        line_name = (
            request.line_name
            if request.line_name in self.physics_by_line
            else DRIVING_LINE_RACING
        )
        physics = self.physics_by_line[line_name]
        distance_m = self.track_profile.line_distance_at_total_progress(
            line_name,
            request.total_progress,
        )
        speed_mps = max(0.0, request.speed_mps)
        following_vehicle = next(
            (
                nearby
                for nearby in request.nearby_vehicles
                if nearby.driver_id == request.following_driver_id
            ),
            None,
        )
        following_initial_distance_m = (
            self.track_profile.line_distance_at_total_progress(
                line_name,
                following_vehicle.total_progress,
            )
            if following_vehicle is not None
            else 0.0
        )
        following_acceleration_mps2 = (
            max(-20.0, min(10.0, following_vehicle.acceleration_mps2))
            if following_vehicle is not None
            else 0.0
        )
        samples = [
            _LongitudinalSample(
                time_seconds=0.0,
                total_progress=request.total_progress,
                speed_mps=speed_mps,
                acceleration_mps2=request.acceleration_mps2,
                target_speed_mps=physics.target_speed_mps(
                    distance_m,
                    request.modifiers,
                ),
                throttle=0.0,
                brake=0.0,
            )
        ]
        elapsed_seconds = 0.0
        step_index = 0
        target_speed_mps = samples[0].target_speed_mps
        while elapsed_seconds < self.config.horizon_seconds - 1e-9:
            step_seconds = min(
                self.config.sample_interval_seconds,
                self.config.horizon_seconds - elapsed_seconds,
            )
            if step_index > 0 and step_index % 2 == 0:
                target_speed_mps = physics.target_speed_mps(
                    distance_m,
                    request.modifiers,
                )
            next_elapsed_seconds = elapsed_seconds + step_seconds
            leader_next_distance_m: float | None = None
            if following_vehicle is not None:
                acceleration_time = min(0.75, elapsed_seconds)
                leader_speed_mps = max(
                    0.0,
                    following_vehicle.speed_mps
                    + following_acceleration_mps2 * acceleration_time,
                )
                leader_distance_m = (
                    following_initial_distance_m
                    + following_vehicle.speed_mps * elapsed_seconds
                    + 0.5
                    * following_acceleration_mps2
                    * acceleration_time**2
                    + following_acceleration_mps2
                    * acceleration_time
                    * max(0.0, elapsed_seconds - acceleration_time)
                )
                gap_m = leader_distance_m - distance_m
                minimum_gap_m = max(0.0, request.following_minimum_gap_m)
                usable_gap_m = max(0.0, gap_m - minimum_gap_m)
                reaction_distance_m = max(
                    0.0,
                    speed_mps - leader_speed_mps,
                ) * FOLLOWING_CONTROL_REACTION_SECONDS
                braking_gap_m = max(0.0, usable_gap_m - reaction_distance_m)
                kinematic_safe_speed_mps = (
                    leader_speed_mps**2
                    + 2.0
                    * FOLLOWING_PREDICTIVE_DECELERATION_MPS2
                    * braking_gap_m
                ) ** 0.5
                target_speed_mps = min(
                    target_speed_mps,
                    kinematic_safe_speed_mps,
                )
                if gap_m < request.following_desired_gap_m:
                    target_speed_mps = min(
                        target_speed_mps,
                        max(
                            0.0,
                            leader_speed_mps
                            + 0.85
                            * (gap_m - request.following_desired_gap_m),
                        ),
                    )
                next_acceleration_time = min(0.75, next_elapsed_seconds)
                leader_next_distance_m = (
                    following_initial_distance_m
                    + following_vehicle.speed_mps * next_elapsed_seconds
                    + 0.5
                    * following_acceleration_mps2
                    * next_acceleration_time**2
                    + following_acceleration_mps2
                    * next_acceleration_time
                    * max(0.0, next_elapsed_seconds - next_acceleration_time)
                )
            speed_error_mps = target_speed_mps - speed_mps
            if speed_error_mps >= 0.0:
                acceleration_mps2 = min(
                    MAX_ACCELERATION_MPS2
                    * max(0.2, request.modifiers.power)
                    * max(0.2, request.modifiers.traction),
                    speed_error_mps / max(0.20, step_seconds * 3.0),
                )
                throttle = min(
                    1.0,
                    acceleration_mps2 / max(1e-9, MAX_ACCELERATION_MPS2),
                )
                brake = 0.0
            else:
                braking_limit_mps2 = (
                    MAX_BRAKING_MPS2
                    * max(0.2, request.modifiers.braking)
                )
                acceleration_mps2 = max(
                    -braking_limit_mps2,
                    speed_error_mps / max(0.15, step_seconds * 2.0),
                )
                throttle = 0.0
                brake = min(
                    1.0,
                    -acceleration_mps2 / max(1e-9, braking_limit_mps2),
                )
            next_speed_mps = max(
                0.0,
                speed_mps + acceleration_mps2 * step_seconds,
            )
            next_distance_m = distance_m + (
                speed_mps + next_speed_mps
            ) * 0.5 * step_seconds
            if leader_next_distance_m is not None:
                maximum_distance_m = (
                    leader_next_distance_m
                    - max(0.0, request.following_minimum_gap_m)
                )
                if next_distance_m > maximum_distance_m:
                    next_distance_m = max(distance_m, maximum_distance_m)
                    next_speed_mps = max(
                        0.0,
                        2.0 * (next_distance_m - distance_m) / step_seconds
                        - speed_mps,
                    )
                    acceleration_mps2 = (
                        next_speed_mps - speed_mps
                    ) / step_seconds
                    throttle = 0.0
                    brake = min(
                        1.0,
                        max(0.0, -acceleration_mps2)
                        / max(1e-9, MAX_BRAKING_MPS2),
                    )
            distance_m = next_distance_m
            elapsed_seconds += step_seconds
            step_index += 1
            speed_mps = next_speed_mps
            samples.append(
                _LongitudinalSample(
                    time_seconds=elapsed_seconds,
                    total_progress=self.track_profile.total_progress_at_line_distance(
                        line_name,
                        distance_m,
                    ),
                    speed_mps=speed_mps,
                    acceleration_mps2=acceleration_mps2,
                    target_speed_mps=target_speed_mps,
                    throttle=throttle,
                    brake=brake,
                )
            )
        return tuple(samples)

    def _predict_nearby_occupancies(
        self,
        request: LocalTrajectoryPlanningRequest,
        longitudinal: tuple[_LongitudinalSample, ...],
    ) -> tuple[PredictedVehicleOccupancy, ...]:
        occupancies: list[PredictedVehicleOccupancy] = []
        for nearby in request.nearby_vehicles:
            acceleration_mps2 = max(
                -20.0,
                min(10.0, nearby.acceleration_mps2),
            )
            previous_longitudinal_m = nearby.total_progress * self.track_length_m
            previous_lateral_m = nearby.lateral_offset_m
            corner_direction = (
                1 if nearby.corner_turn_direction >= 0 else -1
            )
            initial_racing_offset_m = self.track_profile.line_offset_at_progress(
                DRIVING_LINE_RACING,
                nearby.total_progress,
            )
            tactical_offset_from_line_m = (
                nearby.target_lateral_offset_m - initial_racing_offset_m
            )
            samples: list[PredictedOccupancySample] = []
            for index, ego_sample in enumerate(longitudinal):
                time_seconds = ego_sample.time_seconds
                acceleration_time = min(0.75, time_seconds)
                speed_mps = max(
                    0.0,
                    nearby.speed_mps + acceleration_mps2 * acceleration_time,
                )
                distance_m = (
                    nearby.speed_mps * time_seconds
                    + 0.5 * acceleration_mps2 * acceleration_time**2
                    + acceleration_mps2
                    * acceleration_time
                    * max(0.0, time_seconds - acceleration_time)
                )
                longitudinal_m = (
                    nearby.total_progress * self.track_length_m + distance_m
                )
                predicted_progress = longitudinal_m / self.track_length_m
                # Track offsets are Frenet coordinates, so a driver holding a
                # line must follow that line's changing lateral coordinate.
                # Holding the absolute offset made opponents appear to drift
                # across curved/reference-line data and hid real closing-path
                # conflicts after higher-fidelity track profiles were loaded.
                target_lateral_offset_m = (
                    self.track_profile.line_offset_at_progress(
                        DRIVING_LINE_RACING,
                        predicted_progress,
                    )
                    + tactical_offset_from_line_m
                )
                if nearby.corner_role in {"inside", "outside"}:
                    role_direction = (
                        corner_direction
                        if nearby.corner_role == "inside"
                        else -corner_direction
                    )
                    racing_offset_m = self.track_profile.line_offset_at_progress(
                        DRIVING_LINE_RACING,
                        predicted_progress,
                    )
                    minimum_m, maximum_m = (
                        self.surface_profile.trajectory_body_lateral_bounds(
                            predicted_progress,
                            body_width_m=nearby.body_width_m,
                            edge_margin_m=self.config.edge_margin_m,
                        )
                    )
                    half_separation_m = max(
                        0.0,
                        nearby.corner_pair_separation_m / 2.0,
                    )
                    pair_midpoint_m = max(
                        minimum_m + half_separation_m,
                        min(
                            maximum_m - half_separation_m,
                            racing_offset_m,
                        ),
                    )
                    target_lateral_offset_m = (
                        pair_midpoint_m + role_direction * half_separation_m
                    )
                transition_ratio = min(
                    1.0,
                    time_seconds / self.config.lateral_transition_seconds,
                )
                smooth_ratio = transition_ratio * transition_ratio * (
                    3.0 - 2.0 * transition_ratio
                )
                lateral_m = (
                    nearby.lateral_offset_m
                    + (
                        target_lateral_offset_m
                        - nearby.lateral_offset_m
                    )
                    * smooth_ratio
                    + nearby.lateral_speed_mps
                    * time_seconds
                    * (1.0 - transition_ratio)
                )
                if index == 0:
                    heading_rad = nearby.heading_offset_rad
                    lateral_speed_mps = nearby.lateral_speed_mps
                else:
                    step_seconds = max(
                        1e-9,
                        time_seconds - longitudinal[index - 1].time_seconds,
                    )
                    lateral_speed_mps = (
                        lateral_m - previous_lateral_m
                    ) / step_seconds
                    heading_rad = atan2(
                        lateral_m - previous_lateral_m,
                        max(1e-6, longitudinal_m - previous_longitudinal_m),
                    )
                samples.append(
                    PredictedOccupancySample(
                        time_seconds=time_seconds,
                        body_pose=BodyPose(
                            longitudinal_m=longitudinal_m,
                            lateral_m=lateral_m,
                            heading_rad=heading_rad,
                            length_m=nearby.body_length_m,
                            width_m=nearby.body_width_m,
                            longitudinal_speed_mps=speed_mps,
                            lateral_speed_mps=lateral_speed_mps,
                        ),
                    )
                )
                previous_longitudinal_m = longitudinal_m
                previous_lateral_m = lateral_m
            occupancies.append(
                PredictedVehicleOccupancy(
                    driver_id=nearby.driver_id,
                    samples=tuple(samples),
                )
            )
        return tuple(occupancies)

    def _candidate(
        self,
        request: LocalTrajectoryPlanningRequest,
        longitudinal: tuple[_LongitudinalSample, ...],
        boundaries: tuple[_TrajectoryBounds, ...],
        base_lateral_offsets_m: tuple[float, ...],
        transition_ratios: tuple[float, ...],
        opponent_occupancies: tuple[PredictedVehicleOccupancy, ...],
        lateral_bias_m: float,
        bias_directions: tuple[int, ...],
        candidate_id: str | None = None,
    ) -> LocalTrajectoryCandidate:
        lateral_offsets_m: list[float] = []
        lateral_speeds_mps: list[float] = []
        heading_offsets_rad: list[float] = []
        previous_lateral_m = request.lateral_offset_m
        previous_longitudinal_m = request.total_progress * self.track_length_m
        path_distance_m = 0.0
        for index, prediction in enumerate(longitudinal):
            smooth_ratio = transition_ratios[index]
            base_lateral_m = base_lateral_offsets_m[index]
            desired_lateral_m = (
                base_lateral_m + lateral_bias_m * bias_directions[index]
            )
            lateral_m = request.lateral_offset_m + (
                desired_lateral_m - request.lateral_offset_m
            ) * smooth_ratio
            longitudinal_m = prediction.total_progress * self.track_length_m
            delta_longitudinal_m = longitudinal_m - previous_longitudinal_m
            delta_lateral_m = lateral_m - previous_lateral_m
            if index == 0:
                lateral_speed_mps = request.lateral_speed_mps
                heading_rad = atan2(
                    request.lateral_speed_mps,
                    max(1.0, request.speed_mps),
                )
            else:
                step_seconds = max(
                    1e-9,
                    prediction.time_seconds - longitudinal[index - 1].time_seconds,
                )
                lateral_speed_mps = delta_lateral_m / step_seconds
                heading_rad = atan2(
                    delta_lateral_m,
                    max(1e-6, delta_longitudinal_m),
                )
                path_distance_m += hypot(
                    max(0.0, delta_longitudinal_m),
                    delta_lateral_m,
                )
            lateral_offsets_m.append(lateral_m)
            lateral_speeds_mps.append(lateral_speed_mps)
            heading_offsets_rad.append(heading_rad)
            previous_longitudinal_m = longitudinal_m
            previous_lateral_m = lateral_m

        boundary_violations = 0
        uses_non_track_surface = False
        for lateral_m, heading_rad, bounds in zip(
            lateral_offsets_m,
            heading_offsets_rad,
            boundaries,
        ):
            rear_lateral_m = (
                lateral_m - request.body_length_m / 2.0 * heading_rad
            )
            front_lateral_m = (
                lateral_m + request.body_length_m / 2.0 * heading_rad
            )
            if (
                rear_lateral_m < bounds.rear_minimum_m
                or rear_lateral_m > bounds.rear_maximum_m
            ):
                boundary_violations += 1
            if (
                front_lateral_m < bounds.front_minimum_m
                or front_lateral_m > bounds.front_maximum_m
            ):
                boundary_violations += 1
            uses_non_track_surface = uses_non_track_surface or (
                lateral_m < bounds.white_minimum_m
                or lateral_m > bounds.white_maximum_m
            )

        surface_cost_seconds = 0.0
        tire_surface_cost_seconds = 0.0
        if uses_non_track_surface or boundary_violations > 0:
            assessment_indices = list(range(len(longitudinal)))
            if boundary_violations > 0:
                assessment_indices = list(range(0, len(longitudinal), 4))
                if assessment_indices[-1] != len(longitudinal) - 1:
                    assessment_indices.append(len(longitudinal) - 1)
            assessment = self.surface_profile.assess_trajectory(
                [longitudinal[index].total_progress for index in assessment_indices],
                [lateral_offsets_m[index] for index in assessment_indices],
                track_length_m=self.track_length_m,
                body_width_m=request.body_width_m,
                body_length_m=request.body_length_m,
                edge_margin_m=self.config.edge_margin_m,
                heading_offsets_rad=[
                    heading_offsets_rad[index] for index in assessment_indices
                ],
            )
            boundary_violations = max(
                boundary_violations,
                assessment.body_boundary_violations,
            )
            surface_cost_seconds = assessment.cost_seconds
            tire_wear = max(0.0, min(1.0, request.tire_wear))
            tire_grip_deficit = sum(
                max(0.0, 1.0 - max(0.0, grip_factor))
                for grip_factor in (
                    request.tire_lateral_grip,
                    request.tire_braking_grip,
                    request.tire_traction_grip,
                )
            ) / 3.0
            tire_risk = tire_wear * 1.20 + tire_grip_deficit * 4.0
            contact_count = max(1, len(longitudinal) * 4)
            unstable_surface_exposure = (
                assessment.low_kerb_contacts * 0.04
                + assessment.high_kerb_contacts * 0.40
                + assessment.runoff_contacts * 0.60
                + assessment.grass_contacts * 4.0
                + assessment.gravel_contacts * 5.0
            ) / contact_count
            tire_surface_cost_seconds = tire_risk * (
                surface_cost_seconds + unstable_surface_exposure
            )

        conflicting_driver_ids: list[int] = []
        first_collision_time_seconds: float | None = None
        minimum_opponent_clearance_m = float("inf")
        for occupancy in opponent_occupancies:
            opponent_conflict = False
            initial_opponent_pose = occupancy.samples[0].body_pose
            initial_ego_longitudinal_m = (
                request.total_progress * self.track_length_m
            )
            opponent_started_safely_behind_same_corridor = (
                initial_opponent_pose.longitudinal_m
                + 0.5 * (request.body_length_m + initial_opponent_pose.length_m)
                + self.config.minimum_traffic_clearance_m
                < initial_ego_longitudinal_m
                and abs(
                    initial_opponent_pose.lateral_m
                    - request.lateral_offset_m
                )
                <= 0.5
                * (request.body_width_m + initial_opponent_pose.width_m)
                + self.config.minimum_traffic_clearance_m
            )
            for index, opponent_sample in enumerate(occupancy.samples):
                prediction = longitudinal[index]
                ego_pose = BodyPose(
                    longitudinal_m=(
                        prediction.total_progress * self.track_length_m
                    ),
                    lateral_m=lateral_offsets_m[index],
                    heading_rad=heading_offsets_rad[index],
                    length_m=request.body_length_m,
                    width_m=request.body_width_m,
                    longitudinal_speed_mps=prediction.speed_mps,
                    lateral_speed_mps=lateral_speeds_mps[index],
                )
                opponent_pose = opponent_sample.body_pose
                center_distance_m = hypot(
                    opponent_pose.longitudinal_m - ego_pose.longitudinal_m,
                    opponent_pose.lateral_m - ego_pose.lateral_m,
                )
                bounding_separation_m = center_distance_m - (
                    hypot(ego_pose.length_m, ego_pose.width_m)
                    + hypot(opponent_pose.length_m, opponent_pose.width_m)
                ) / 2.0
                # Circumscribed-circle separation is a conservative lower
                # bound.  Once even that bound is above the clearance target,
                # the exact four-axis SAT result cannot affect collision or
                # candidate cost and can be skipped safely.
                separation_m = (
                    bounding_separation_m
                    if bounding_separation_m > self.config.minimum_traffic_clearance_m
                    else oriented_body_separation_m(ego_pose, opponent_pose)
                )
                minimum_opponent_clearance_m = min(
                    minimum_opponent_clearance_m,
                    separation_m,
                )
                # SAT separation is zero exactly when the oriented rectangles
                # touch or overlap.  Re-running the full SAT overlap solver here
                # doubled geometry work for every candidate/sample pair, even
                # though the planner only needs a conflict boolean.
                if separation_m > 1e-7:
                    continue
                if (
                    opponent_started_safely_behind_same_corridor
                    and abs(lateral_bias_m) <= 1e-9
                ):
                    # A car holding its corridor is not responsible for a
                    # predicted rear-end impact from a faster follower. The
                    # follower's longitudinal controller owns that closing
                    # gap. A car changing corridor remains responsible for
                    # checking that follower, otherwise a pull-out can cut
                    # directly across a faster car's nose.
                    continue
                opponent_conflict = True
                collision_time = opponent_sample.time_seconds
                if (
                    first_collision_time_seconds is None
                    or collision_time < first_collision_time_seconds
                ):
                    first_collision_time_seconds = collision_time
            if opponent_conflict:
                conflicting_driver_ids.append(occupancy.driver_id)

        predicted_collision_count = len(conflicting_driver_ids)
        traffic_cost = predicted_collision_count * 1_000.0
        if first_collision_time_seconds is not None:
            traffic_cost += (
                self.config.horizon_seconds - first_collision_time_seconds
            ) * 50.0
        if minimum_opponent_clearance_m != float("inf"):
            traffic_cost += max(
                0.0,
                self.config.minimum_traffic_clearance_m
                - minimum_opponent_clearance_m,
            ) * 8.0
        forward_distance_m = max(
            0.0,
            (
                longitudinal[-1].total_progress
                - longitudinal[0].total_progress
            )
            * self.track_length_m,
        )
        extra_path_distance_m = max(0.0, path_distance_m - forward_distance_m)
        average_speed_mps = max(
            1.0,
            sum(sample.speed_mps for sample in longitudinal) / len(longitudinal),
        )
        objective_cost = (
            -forward_distance_m / 100.0 * request.weights.forward_progress
            + extra_path_distance_m
            / average_speed_mps
            * request.weights.extra_path_distance
            + abs(lateral_bias_m) * 0.08 * request.weights.lateral_deviation
            + surface_cost_seconds * request.weights.surface
            + tire_surface_cost_seconds * request.weights.tire_surface
            + boundary_violations * 1_000.0
            + traffic_cost * request.weights.traffic
        )
        control_lookahead_seconds = min(
            self.config.maximum_control_lookahead_seconds,
            self.config.control_lookahead_seconds
            + max(0.0, request.speed_mps) / 140.0,
        )
        control_sample_index = min(
            range(len(longitudinal)),
            key=lambda index: abs(
                longitudinal[index].time_seconds
                - control_lookahead_seconds
            ),
        )
        return LocalTrajectoryCandidate(
            candidate_id=(
                candidate_id
                or self._candidate_id(
                    lateral_bias_m * bias_directions[0],
                    self.config.lateral_spacing_m,
                )
            ),
            lateral_bias_m=lateral_bias_m,
            longitudinal_samples=longitudinal,
            lateral_offsets_m=tuple(lateral_offsets_m),
            lateral_speeds_mps=tuple(lateral_speeds_mps),
            heading_offsets_rad=tuple(heading_offsets_rad),
            body_width_m=request.body_width_m,
            body_length_m=request.body_length_m,
            track_length_m=self.track_length_m,
            forward_distance_m=forward_distance_m,
            path_distance_m=path_distance_m,
            surface_cost_seconds=surface_cost_seconds,
            tire_surface_cost_seconds=tire_surface_cost_seconds,
            body_boundary_violations=boundary_violations,
            predicted_collision_count=predicted_collision_count,
            first_collision_time_seconds=first_collision_time_seconds,
            minimum_opponent_clearance_m=minimum_opponent_clearance_m,
            conflicting_driver_ids=tuple(conflicting_driver_ids),
            traffic_cost=traffic_cost,
            objective_cost=objective_cost,
            control_lookahead_seconds=control_lookahead_seconds,
            control_target_lateral_offset_m=lateral_offsets_m[
                control_sample_index
            ],
            control_target_lateral_speed_mps=lateral_speeds_mps[
                control_sample_index
            ],
        )

    def plan(
        self,
        request: LocalTrajectoryPlanningRequest,
    ) -> LocalTrajectoryPlan:
        started = perf_counter()
        longitudinal = self._predict_longitudinal(request)
        def boundary_at(sample: _LongitudinalSample) -> _TrajectoryBounds:
            return self._trajectory_bounds(
                sample.total_progress,
                request.body_width_m,
                request.body_length_m,
            )

        anchor_indices = set(range(0, len(longitudinal), 2))
        anchor_indices.add(len(longitudinal) - 1)
        boundary_anchors = {
            index: boundary_at(longitudinal[index])
            for index in sorted(anchor_indices)
        }
        boundaries = []
        for index in range(len(longitudinal)):
            if index in boundary_anchors:
                boundaries.append(boundary_anchors[index])
                continue
            previous = boundary_anchors[index - 1]
            following = boundary_anchors[index + 1]
            boundaries.append(
                _TrajectoryBounds(
                    rear_minimum_m=max(
                        previous.rear_minimum_m,
                        following.rear_minimum_m,
                    ),
                    rear_maximum_m=min(
                        previous.rear_maximum_m,
                        following.rear_maximum_m,
                    ),
                    front_minimum_m=max(
                        previous.front_minimum_m,
                        following.front_minimum_m,
                    ),
                    front_maximum_m=min(
                        previous.front_maximum_m,
                        following.front_maximum_m,
                    ),
                    white_minimum_m=max(
                        previous.white_minimum_m,
                        following.white_minimum_m,
                    ),
                    white_maximum_m=min(
                        previous.white_maximum_m,
                        following.white_maximum_m,
                    ),
                )
            )
        frozen_boundaries = tuple(boundaries)
        base_lateral_offsets_m = tuple(
            self.track_profile.line_offset_at_progress(
                request.line_name,
                sample.total_progress,
            )
            for sample in longitudinal
        )
        if request.corner_role not in {None, "inside", "outside"}:
            raise ValueError("corner role must be inside, outside, or None")
        if request.corner_role is not None:
            turn_direction = (
                1 if request.corner_turn_direction >= 0 else -1
            )
            role_direction = (
                turn_direction
                if request.corner_role == "inside"
                else -turn_direction
            )
            half_separation_m = max(
                0.0,
                request.corner_pair_separation_m / 2.0,
            )
            corner_offsets_m: list[float] = []
            for racing_offset_m, bounds in zip(
                base_lateral_offsets_m,
                frozen_boundaries,
            ):
                minimum_m = max(
                    bounds.rear_minimum_m,
                    bounds.front_minimum_m,
                )
                maximum_m = min(
                    bounds.rear_maximum_m,
                    bounds.front_maximum_m,
                )
                pair_midpoint_m = max(
                    minimum_m + half_separation_m,
                    min(
                        maximum_m - half_separation_m,
                        racing_offset_m,
                    ),
                )
                corner_offsets_m.append(
                    pair_midpoint_m + role_direction * half_separation_m
                )
            base_lateral_offsets_m = tuple(corner_offsets_m)
        bias_directions: tuple[int, ...]
        if request.corner_role is None:
            bias_directions = tuple(1 for _ in longitudinal)
        else:
            # Corner candidates shift a dynamically allocated two-car corridor
            # rather than adding another fixed inside/outside line template.
            bias_directions = tuple(1 for _ in longitudinal)
        transition_ratios = tuple(
            (
                min(
                    1.0,
                    sample.time_seconds
                    / self.config.lateral_transition_seconds,
                )
                ** 2
            )
            * (
                3.0
                - 2.0
                * min(
                    1.0,
                    sample.time_seconds
                    / self.config.lateral_transition_seconds,
                )
            )
            for sample in longitudinal
        )
        opponent_occupancies = self._predict_nearby_occupancies(
            request,
            longitudinal,
        )
        candidate_count = (
            max(
                self.config.candidate_count,
                self.config.traffic_candidate_count,
            )
            if opponent_occupancies
            else self.config.candidate_count
        )
        candidate_biases_m = (
            request.candidate_lateral_biases_m
            if request.candidate_lateral_biases_m is not None
            else self.lateral_biases_m(candidate_count)
        )
        if not candidate_biases_m:
            raise ValueError("local trajectory requires at least one lateral candidate")
        candidates = tuple(
            self._candidate(
                request,
                longitudinal,
                frozen_boundaries,
                base_lateral_offsets_m,
                transition_ratios,
                opponent_occupancies,
                lateral_bias_m,
                bias_directions,
                (
                    f"{request.corner_role}_{index}"
                    if request.corner_role is not None
                    else None
                ),
            )
            for index, lateral_bias_m in enumerate(candidate_biases_m)
        )
        viable = [candidate for candidate in candidates if candidate.viable]
        selected = min(
            viable or list(candidates),
            key=lambda candidate: (
                candidate.objective_cost,
                abs(candidate.lateral_bias_m),
                candidate.candidate_id,
            ),
        )
        selection_reason = "lowest_cost"
        previous = next(
            (
                candidate
                for candidate in candidates
                if candidate.candidate_id
                == request.previous_selected_candidate_id
            ),
            None,
        )
        previous_is_traffic_safe = bool(
            previous is not None
            and previous.viable
            and (
                not opponent_occupancies
                or previous.minimum_opponent_clearance_m
                >= self.config.minimum_traffic_clearance_m
            )
        )
        if previous_is_traffic_safe and previous is not None:
            if previous.candidate_id == selected.candidate_id:
                selection_reason = "continued"
            elif (
                request.previous_selected_age_seconds
                < self.config.minimum_path_hold_seconds
            ):
                selected = previous
                selection_reason = "minimum_hold"
            elif (
                previous.objective_cost - selected.objective_cost
                < self.config.switch_cost_improvement_threshold
            ):
                selected = previous
                selection_reason = "hysteresis"
        return LocalTrajectoryPlan(
            candidates=candidates,
            opponent_occupancies=opponent_occupancies,
            selected_candidate_id=selected.candidate_id,
            selection_reason=selection_reason,
            horizon_seconds=self.config.horizon_seconds,
            generation_duration_ms=(perf_counter() - started) * 1000.0,
        )
