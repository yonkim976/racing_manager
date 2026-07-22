"""Fixed-step longitudinal vehicle physics for the optional Physics V2 engine."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import sqrt

from simulation.speed_profile import SpeedProfile
from simulation.vehicle_dynamics import (
    DynamicBicycleState,
    MAX_ACCELERATION_MPS2,
    MAX_BRAKING_MPS2,
    MAX_SPEED_MPS as PHYSICS_MAX_SPEED_MPS,
    aero_coefficients,
    advance_dynamic_bicycle,
    drive_force_limits_n,
    force_envelope,
    lateral_speed_limit_mps,
    maximum_brake_force_n,
    maximum_braking_deceleration_mps2,
)

PHYSICS_STEP_SECONDS = 0.02  # 50 Hz
PHYSICS_MIN_SPEED_MPS = 18.0
PHYSICS_LATERAL_ACCELERATION_MPS2 = 4.5
PHYSICS_HANDLING_LATERAL_ACCELERATION_MPS2 = 7.0
PHYSICS_MAX_LATERAL_SPEED_MPS = 4.0
CONTROLLER_LOOKAHEAD_M = 450.0
CONTROLLER_SAMPLE_DISTANCE_M = 50.0
CONTROLLER_CACHE_DISTANCE_M = 10.0
TELEMETRY_BRAKING_CURVATURE_RESERVE = 0.25
TELEMETRY_SPEED_CEILING_BRAKING_THRESHOLD = 0.5
RUN_WIDE_LATERAL_GRIP_THRESHOLD = 1.50
LOCKUP_SLIP_RATIO_THRESHOLD = 0.05
TRACTION_LOSS_SLIP_RATIO_THRESHOLD = 0.05
AXLE_SLIP_CLASSIFICATION_THRESHOLD_RAD = 0.16
AXLE_SLIP_BALANCE_MARGIN_RAD = 0.025
_CONTROLLER_SCALE_CACHE: dict[tuple[float, ...], float] = {}


@dataclass(frozen=True)
class VehiclePhysicsModifiers:
    """Dimensionless modifiers supplied by car, driver, tires and race control."""

    power: float = 1.0
    grip: float = 1.0
    pace: float = 1.0
    speed_limit_factor: float = 1.0
    maximum_speed_mps: float | None = None
    braking: float = 1.0
    traction: float = 1.0
    mass_kg: float = 768.0
    effective_power_kw: float = 750.0
    drivetrain_efficiency: float = 0.965
    corner_drag_area_m2: float = 1.10
    straight_drag_area_m2: float = 0.85
    corner_downforce_area_m2: float = 5.20
    straight_downforce_area_m2: float = 3.30
    brake_force_n: float = 44000.0
    mechanical_grip: float = 1.0
    front_aero_share: float = 0.455
    traction_factor: float = 1.0
    drag_multiplier: float = 1.0
    downforce_multiplier: float = 1.0
    predictive_downforce_multiplier: float = 1.0
    surface_drag_deceleration_mps2: float = 0.0
    brake_modulation_error: float = 0.0
    throttle_modulation_error: float = 0.0
    wheelbase_m: float = 3.4
    yaw_inertia_kgm2: float = 1700.0


@dataclass(frozen=True)
class VehicleFollowingConstraint:
    """Predicted motion of a same-line car that must not be intersected."""

    leader_distance_m: float
    leader_speed_mps: float
    leader_end_distance_m: float
    leader_end_speed_mps: float
    desired_gap_m: float
    minimum_gap_m: float


@dataclass(frozen=True)
class VehiclePhysicsResult:
    distance_m: float
    speed_mps: float
    acceleration_mps2: float
    target_speed_mps: float
    throttle: float
    brake: float
    lateral_offset_m: float
    lateral_speed_mps: float
    lateral_acceleration_mps2: float
    grip_utilization: float
    handling_state: str
    slip_angle_rad: float
    drive_force_n: float
    yaw_rate_rad_s: float
    steering_angle_rad: float
    front_slip_angle_rad: float
    rear_slip_angle_rad: float
    wheel_lock_ratio: float
    traction_slip_ratio: float
    tire_slide_energy_j: float


class LongitudinalVehiclePhysics:
    """Advance a point-mass race car along a closed one-dimensional track."""

    def __init__(
        self,
        profile: SpeedProfile | None,
        track_length_m: float,
        reference_lap_time: float,
        *,
        planner_braking_utilization: float = 1.0,
        controller_sample_distance_m: float = CONTROLLER_SAMPLE_DISTANCE_M,
        controller_speed_scale_floor: float = 0.60,
        telemetry_speed_reference_weight: float = 0.0,
        telemetry_braking_curvature_threshold: float = 0.5,
        telemetry_max_braking_utilization: float = 1.0,
        telemetry_braking_speed_reserve: float = 1.0,
        braking_longitudinal_grip_factor: float = 1.0,
        brake_control_error_fraction: float = 0.1,
    ) -> None:
        self.profile = profile
        self.track_length_m = max(1.0, float(track_length_m))
        self.reference_lap_time = max(1.0, float(reference_lap_time))
        self.planner_braking_utilization = max(
            0.3,
            min(1.0, float(planner_braking_utilization)),
        )
        self.controller_sample_distance_m = max(
            5.0,
            min(50.0, float(controller_sample_distance_m)),
        )
        self.controller_speed_scale_floor = max(
            0.60,
            min(1.10, float(controller_speed_scale_floor)),
        )
        self.telemetry_speed_reference_weight = max(
            0.0,
            min(1.0, float(telemetry_speed_reference_weight)),
        )
        self.telemetry_braking_curvature_threshold = max(
            0.0,
            min(1.0, float(telemetry_braking_curvature_threshold)),
        )
        self.telemetry_max_braking_utilization = max(
            self.planner_braking_utilization,
            min(1.0, float(telemetry_max_braking_utilization)),
        )
        self.telemetry_braking_speed_reserve = max(
            0.80,
            min(1.0, float(telemetry_braking_speed_reserve)),
        )
        self.braking_longitudinal_grip_factor = max(
            1.0,
            min(1.5, float(braking_longitudinal_grip_factor)),
        )
        self.brake_control_error_fraction = max(
            0.03,
            min(0.2, float(brake_control_error_fraction)),
        )
        self._predictive_speed_cache: dict[tuple[float, ...], float] = {}
        self._predictive_track_sample_cache: dict[
            int,
            tuple[float, float, float],
        ] = {}
        scale_key = self._controller_scale_key()
        cached_scale = _CONTROLLER_SCALE_CACHE.get(scale_key)
        if cached_scale is None:
            self.controller_speed_scale = 1.0
            predicted_lap_time = self._estimate_predictive_lap_time()
            cached_scale = max(
                self.controller_speed_scale_floor,
                min(1.10, predicted_lap_time / self.reference_lap_time),
            )
            _CONTROLLER_SCALE_CACHE[scale_key] = cached_scale
        self.controller_speed_scale = cached_scale

    def _controller_scale_key(self) -> tuple[float, ...]:
        if self.profile is None:
            return (
                round(self.track_length_m, 3),
                round(self.reference_lap_time, 3),
                round(self.planner_braking_utilization, 3),
                round(self.controller_sample_distance_m, 3),
                round(self.controller_speed_scale_floor, 3),
                round(self.telemetry_speed_reference_weight, 3),
                round(TELEMETRY_BRAKING_CURVATURE_RESERVE, 3),
                round(TELEMETRY_SPEED_CEILING_BRAKING_THRESHOLD, 3),
                round(self.telemetry_braking_speed_reserve, 3),
                round(self.telemetry_max_braking_utilization, 3),
                round(self.braking_longitudinal_grip_factor, 3),
                round(self.brake_control_error_fraction, 3),
                0.0,
                0.0,
            )
        return (
            round(self.track_length_m, 3),
            round(self.reference_lap_time, 3),
            round(self.planner_braking_utilization, 3),
            round(self.controller_sample_distance_m, 3),
            round(self.controller_speed_scale_floor, 3),
            round(self.telemetry_speed_reference_weight, 3),
            round(TELEMETRY_BRAKING_CURVATURE_RESERVE, 3),
            round(TELEMETRY_SPEED_CEILING_BRAKING_THRESHOLD, 3),
            round(self.telemetry_braking_speed_reserve, 3),
            round(self.telemetry_max_braking_utilization, 3),
            round(self.braking_longitudinal_grip_factor, 3),
            round(self.brake_control_error_fraction, 3),
            float(len(self.profile.progress)),
            round(sum(self.profile.curvatures_1pm), 6),
            round(sum(self.profile.raw_speeds_mps), 3),
            round(sum(self.profile.braking_fractions), 3),
        )

    def _estimate_predictive_lap_time(self) -> float:
        """Calibrate a circuit globally without consuming its legacy speed targets."""
        if self.profile is None or not self.profile.progress:
            return self.reference_lap_time
        modifiers = VehiclePhysicsModifiers()
        lap_time = 0.0
        count = len(self.profile.progress)
        for index, start in enumerate(self.profile.progress):
            end = self.profile.progress[(index + 1) % count]
            if index == count - 1:
                end += 1.0
            segment_distance_m = max(0.0, end - start) * self.track_length_m
            target_speed_mps = self._predictive_target_speed_mps(
                start * self.track_length_m,
                modifiers,
            )
            lap_time += segment_distance_m / max(
                PHYSICS_MIN_SPEED_MPS,
                target_speed_mps,
            )
        return max(1.0, lap_time)

    def target_speed_mps(
        self,
        distance_m: float,
        modifiers: VehiclePhysicsModifiers,
    ) -> float:
        target = self._predictive_target_speed_mps(distance_m, modifiers)
        target *= self.controller_speed_scale
        driver_utilization = max(
            0.92,
            min(1.005, 0.980 + (modifiers.pace - 1.0) * 0.60),
        )
        target *= driver_utilization
        target *= max(0.2, min(1.50, modifiers.speed_limit_factor))
        if modifiers.maximum_speed_mps is not None:
            target = min(target, max(PHYSICS_MIN_SPEED_MPS, modifiers.maximum_speed_mps))
        return max(PHYSICS_MIN_SPEED_MPS, min(PHYSICS_MAX_SPEED_MPS, target))

    def _aero_coefficients(
        self,
        distance_m: float,
        modifiers: VehiclePhysicsModifiers,
    ) -> tuple[float, float]:
        return aero_coefficients(self._curvature_1pm(distance_m), modifiers)

    def _raw_curvature_1pm(self, distance_m: float) -> float:
        if self.profile is None:
            return 0.0
        progress = (distance_m / self.track_length_m) % 1.0
        return self.profile.curvature_at_progress(progress)

    def _curvature_1pm(self, distance_m: float) -> float:
        curvature = self._raw_curvature_1pm(distance_m)
        if self.telemetry_speed_reference_weight <= 0.0 or curvature <= 1e-9:
            return curvature

        # The measured envelope also calibrates the curvature consumed by the
        # force model.  Calibrating only the controller target lets a car aim
        # for a real-world speed while the handling model still believes that
        # speed exceeds available lateral force, producing artificial
        # understeer.  v_limit is proportional to sqrt(1 / curvature), so the
        # squared speed ratio is the matching empirical curvature correction.
        assert self.profile is not None
        progress = (distance_m / self.track_length_m) % 1.0
        measured_speed = self.profile.raw_speed_at_progress(progress)
        # A low measured speed in a braking zone is not evidence of tighter
        # geometry.  Feeding it into both curvature calibration and the
        # backward braking pass consumes lateral grip twice and makes the
        # controller brake unrealistically early.  Let braking telemetry tune
        # deceleration while non-braking samples remain curvature evidence.
        braking_fraction = self.profile.braking_fraction_at_progress(progress)
        neutral_limit = lateral_speed_limit_mps(
            curvature,
            VehiclePhysicsModifiers(),
        )
        calibration_speed = measured_speed
        if braking_fraction >= self.telemetry_braking_curvature_threshold:
            # In a braking sample, only use telemetry to *relax* a centerline
            # curvature that would make the measured speed impossible.  Keep
            # a braking reserve so the same tyre envelope can still provide
            # longitudinal deceleration; never tighten geometry merely because
            # the driver is slowing for the next apex.
            calibration_speed = max(
                neutral_limit,
                measured_speed
                * (
                    1.0
                    + TELEMETRY_BRAKING_CURVATURE_RESERVE
                    * braking_fraction
                ),
            )
        speed_ratio = max(
            0.15,
            min(3.00, calibration_speed / max(1.0, neutral_limit)),
        )
        curvature_scale = 1.0 + self.telemetry_speed_reference_weight * (
            speed_ratio * speed_ratio - 1.0
        )
        return curvature / max(0.25, curvature_scale)

    def _lateral_speed_limit_mps(
        self,
        distance_m: float,
        modifiers: VehiclePhysicsModifiers,
    ) -> float:
        """Return the steady-state speed where tyre lateral force is exhausted."""
        return lateral_speed_limit_mps(
            self._curvature_1pm(distance_m),
            modifiers,
        )

    def _maximum_braking_deceleration_mps2(
        self,
        distance_m: float,
        speed_mps: float,
        modifiers: VehiclePhysicsModifiers,
    ) -> float:
        return maximum_braking_deceleration_mps2(
            self._curvature_1pm(distance_m),
            speed_mps,
            modifiers,
        )

    def _predictive_track_sample(
        self,
        distance_m: float,
    ) -> tuple[float, float, float]:
        """Return cached geometry/telemetry for a 10 m planning sample.

        Every predictive speed query overlaps most of the previous query's
        lookahead window.  Geometry and source telemetry do not depend on the
        car state, so recomputing their bisect/interpolation work for each
        fuel and tyre modifier was the dominant 20-car CPU cost.
        """
        distance_bin = int(
            (distance_m % self.track_length_m) / CONTROLLER_CACHE_DISTANCE_M
        )
        cached = self._predictive_track_sample_cache.get(distance_bin)
        if cached is not None:
            return cached
        sample_distance_m = distance_bin * CONTROLLER_CACHE_DISTANCE_M
        curvature = self._curvature_1pm(sample_distance_m)
        if self.profile is None:
            result = (curvature, PHYSICS_MAX_SPEED_MPS, 0.0)
        else:
            progress = sample_distance_m / self.track_length_m
            result = (
                curvature,
                self.profile.raw_speed_at_progress(progress),
                self.profile.braking_fraction_at_progress(progress),
            )
        self._predictive_track_sample_cache[distance_bin] = result
        return result

    def _predictive_target_speed_mps(
        self,
        distance_m: float,
        modifiers: VehiclePhysicsModifiers,
    ) -> float:
        """Look ahead and work backwards from future physical corner limits."""
        if self.profile is None:
            return min(
                PHYSICS_MAX_SPEED_MPS,
                self.track_length_m / self.reference_lap_time,
            )

        predictive_modifiers = replace(
            modifiers,
            drag_multiplier=1.0,
            downforce_multiplier=modifiers.predictive_downforce_multiplier,
        )
        normalized_distance_m = distance_m % self.track_length_m
        distance_bin = int(
            normalized_distance_m / CONTROLLER_CACHE_DISTANCE_M
        )
        cache_key = (
            float(distance_bin),
            round(predictive_modifiers.mass_kg, 1),
            round(predictive_modifiers.grip, 2),
            round(predictive_modifiers.mechanical_grip, 2),
            round(predictive_modifiers.braking, 2),
            round(predictive_modifiers.brake_force_n, -2),
            round(predictive_modifiers.corner_drag_area_m2, 2),
            round(predictive_modifiers.straight_drag_area_m2, 2),
            round(predictive_modifiers.corner_downforce_area_m2, 2),
            round(predictive_modifiers.straight_downforce_area_m2, 2),
            round(predictive_modifiers.downforce_multiplier, 2),
        )
        cached = self._predictive_speed_cache.get(cache_key)
        if cached is not None:
            return cached

        sample_count = int(
            CONTROLLER_LOOKAHEAD_M / self.controller_sample_distance_m
        )
        start_distance_m = distance_bin * CONTROLLER_CACHE_DISTANCE_M
        distances = [
            start_distance_m + index * self.controller_sample_distance_m
            for index in range(sample_count + 1)
        ]
        speed_limits = []
        track_samples = [
            self._predictive_track_sample(sample_distance)
            for sample_distance in distances
        ]
        neutral_modifiers = VehiclePhysicsModifiers()
        for curvature, measured_speed, measured_braking_fraction in track_samples:
            physical_limit = lateral_speed_limit_mps(
                curvature,
                predictive_modifiers,
            )
            if self.telemetry_speed_reference_weight > 0.0:
                neutral_limit = lateral_speed_limit_mps(
                    curvature,
                    neutral_modifiers,
                )
                calibration_ratio = max(
                    0.50,
                    min(2.00, measured_speed / max(1.0, neutral_limit)),
                )
                physical_limit *= 1.0 + self.telemetry_speed_reference_weight * (
                    calibration_ratio - 1.0
                )
                if (
                    measured_braking_fraction
                    >= TELEMETRY_SPEED_CEILING_BRAKING_THRESHOLD
                ):
                    # In an observed braking zone the measured speed is an
                    # actual vehicle-state constraint, not curvature evidence.
                    # Use it as a weighted ceiling so the backwards braking
                    # pass starts early enough to reach the real entry speed.
                    telemetry_ceiling = physical_limit + (
                        measured_speed - physical_limit
                    ) * self.telemetry_speed_reference_weight
                    physical_limit = min(
                        physical_limit,
                        max(
                            PHYSICS_MIN_SPEED_MPS,
                            telemetry_ceiling * self.telemetry_braking_speed_reserve,
                        ),
                    )
            speed_limits.append(physical_limit)
        for index in range(sample_count - 1, -1, -1):
            measured_braking_fraction = max(
                track_samples[index][2],
                track_samples[index + 1][2],
            )
            # Braking telemetry says *where* the real driver was braking; it
            # does not prove that every tyre/brake step used 100% of the
            # theoretical force envelope.  Cap the inferred utilization so
            # the planner preserves stopping-distance margin at Bahrain T1.
            braking_utilization = self.planner_braking_utilization + (
                self.telemetry_speed_reference_weight
                * measured_braking_fraction
                * (
                    self.telemetry_max_braking_utilization
                    - self.planner_braking_utilization
                )
            )
            braking_deceleration = maximum_braking_deceleration_mps2(
                track_samples[index][0],
                speed_limits[index],
                predictive_modifiers,
            ) * braking_utilization
            braking_limit = sqrt(
                speed_limits[index + 1] ** 2
                + 2.0
                * braking_deceleration
                * self.controller_sample_distance_m
            )
            speed_limits[index] = min(speed_limits[index], braking_limit)
        self._predictive_speed_cache[cache_key] = speed_limits[0]
        return speed_limits[0]

    def advance(
        self,
        *,
        distance_m: float,
        speed_mps: float,
        delta_seconds: float,
        modifiers: VehiclePhysicsModifiers,
        following: VehicleFollowingConstraint | None = None,
        lateral_offset_m: float = 0.0,
        lateral_speed_mps: float = 0.0,
        target_lateral_offset_m: float = 0.0,
        target_lateral_speed_mps: float = 0.0,
        reference_lateral_offset_m: float | None = None,
        reference_lateral_speed_mps: float = 0.0,
        maximum_lateral_speed_mps: float = PHYSICS_MAX_LATERAL_SPEED_MPS,
        minimum_lateral_offset_m: float = -5.0,
        maximum_lateral_offset_m: float = 5.0,
        nominal_minimum_lateral_offset_m: float | None = None,
        nominal_maximum_lateral_offset_m: float | None = None,
        heading_error_rad: float = 0.0,
        yaw_rate_rad_s: float = 0.0,
    ) -> VehiclePhysicsResult:
        """Advance using an integer number of exact 50 Hz physics steps."""
        elapsed_seconds = max(0.0, float(delta_seconds))
        step_count = int(elapsed_seconds / PHYSICS_STEP_SECONDS + 1e-9)
        if abs(step_count * PHYSICS_STEP_SECONDS - elapsed_seconds) > 1e-9:
            raise ValueError(
                "vehicle physics accepts only exact 0.02-second step multiples; "
                "use FixedStepAccumulator at the caller boundary"
            )
        distance = float(distance_m)
        speed = max(0.0, float(speed_mps))
        acceleration = 0.0
        target = (
            self.target_speed_mps(distance, modifiers)
            if step_count <= 0
            else 0.0
        )
        throttle = 0.0
        brake = 0.0
        elapsed = 0.0
        lateral_offset = float(lateral_offset_m)
        lateral_speed = float(lateral_speed_mps)
        lateral_speed_limit = max(
            0.5,
            min(
                PHYSICS_MAX_LATERAL_SPEED_MPS,
                float(maximum_lateral_speed_mps),
            ),
        )
        target_lateral_speed = max(
            -lateral_speed_limit,
            min(lateral_speed_limit, float(target_lateral_speed_mps)),
        )
        reference_lateral_offset = (
            float(reference_lateral_offset_m)
            if reference_lateral_offset_m is not None
            else 0.0
        )
        reference_lateral_speed = max(
            -lateral_speed_limit,
            min(
                lateral_speed_limit,
                float(reference_lateral_speed_mps),
            ),
        )
        lateral_acceleration = 0.0
        applied_drive_force_n = 0.0
        grip_utilization = 0.0
        handling_state = "stable"
        slip_angle_rad = float(heading_error_rad)
        yaw_rate = float(yaw_rate_rad_s)
        steering_angle_rad = 0.0
        front_slip_angle_rad = 0.0
        rear_slip_angle_rad = 0.0
        wheel_lock_ratio = 0.0
        traction_slip_ratio = 0.0
        tire_slide_energy_j = 0.0

        for _ in range(step_count):
            step = PHYSICS_STEP_SECONDS
            target = self.target_speed_mps(distance, modifiers)
            outside_nominal_track = bool(
                nominal_minimum_lateral_offset_m is not None
                and nominal_maximum_lateral_offset_m is not None
                and not (
                    nominal_minimum_lateral_offset_m
                    <= lateral_offset
                    <= nominal_maximum_lateral_offset_m
                )
            )
            if outside_nominal_track:
                # A real driver lifts while recovering across the white line;
                # continuing to chase the normal exit-speed target compounds
                # wheelspin and can keep an otherwise clean car off track.
                target = min(target, max(PHYSICS_MIN_SPEED_MPS, speed - 3.0))
            leader_distance = None
            if following is not None:
                elapsed_ratio = min(1.0, elapsed / max(delta_seconds, 1e-9))
                leader_distance = following.leader_distance_m + (
                    following.leader_end_distance_m - following.leader_distance_m
                ) * elapsed_ratio
                leader_speed = following.leader_speed_mps + (
                    following.leader_end_speed_mps - following.leader_speed_mps
                ) * elapsed_ratio
                gap = leader_distance - distance
                if gap < following.desired_gap_m:
                    following_target = (
                        leader_speed
                        + 0.85 * (gap - following.desired_gap_m)
                    )
                    target = min(target, max(0.0, following_target))
                if gap <= following.minimum_gap_m:
                    target = min(target, max(0.0, leader_speed - 2.0))

            difference = target - speed
            curvature_1pm = self._curvature_1pm(distance)
            envelope = force_envelope(
                curvature_1pm,
                speed,
                modifiers,
            )
            mass_kg = envelope.mass_kg
            drag_force_n = envelope.drag_force_n
            rolling_resistance_n = envelope.rolling_resistance_n
            maximum_tire_force_n = envelope.maximum_tire_force_n
            required_lateral_force_n = envelope.required_lateral_force_n
            lateral_grip_utilization = envelope.lateral_grip_utilization
            longitudinal_force_n = 0.0
            rear_traction_overload = False
            if difference >= 0:
                throttle = min(1.0, difference / max(1.0, target * 0.12))
                brake = 0.0
                (
                    power_limited_force_n,
                    traction_limited_force_n,
                    drive_force_n,
                ) = drive_force_limits_n(
                    speed,
                    envelope,
                    modifiers,
                )
                raw_drive_request_n = power_limited_force_n * throttle
                # A clean standard lap should not visibly break traction at
                # every apex. The small overdrive represents the driver's
                # throttle modulation error and grows only for the very
                # highest pace requests; true low-grip/attack situations can
                # still exceed the rear-axle envelope.
                traction_overdrive = min(
                    0.35,
                    0.06
                    + max(0.0, modifiers.pace - 1.03) * 1.25
                    + max(0.0, modifiers.throttle_modulation_error),
                )
                controlled_drive_limit_n = traction_limited_force_n * (
                    0.98
                    + (
                        traction_overdrive * lateral_grip_utilization
                        if throttle > 0.75 and lateral_grip_utilization > 0.70
                        else 0.0
                    )
                )
                requested_drive_force_n = min(
                    raw_drive_request_n,
                    controlled_drive_limit_n,
                )
                rear_traction_overload = (
                    throttle > 0.45
                    and lateral_grip_utilization > 0.72
                    and requested_drive_force_n > traction_limited_force_n * 1.05
                )
                step_traction_slip_ratio = min(
                    2.0,
                    max(
                    0.0,
                    (requested_drive_force_n - traction_limited_force_n)
                    / max(1.0, traction_limited_force_n),
                    ),
                )
                traction_slip_ratio = max(
                    traction_slip_ratio,
                    step_traction_slip_ratio,
                )
                post_peak_traction_force_n = traction_limited_force_n * (
                    1.0 - min(0.25, step_traction_slip_ratio * 0.18)
                )
                longitudinal_force_n = min(
                    requested_drive_force_n,
                    post_peak_traction_force_n,
                )
                tire_slide_energy_j += max(
                    0.0,
                    requested_drive_force_n - longitudinal_force_n,
                ) * speed * step
                applied_drive_force_n = longitudinal_force_n
                acceleration = max(
                    -MAX_BRAKING_MPS2,
                    min(
                        MAX_ACCELERATION_MPS2,
                        (
                            longitudinal_force_n
                            - drag_force_n
                            - rolling_resistance_n
                        )
                        / mass_kg,
                    ),
                )
            else:
                throttle = 0.0
                brake = min(
                    1.0,
                    -difference
                    / max(1.0, speed * self.brake_control_error_fraction),
                )
                available_brake_force_n = (
                    envelope.available_longitudinal_force_n
                    * modifiers.braking
                    * self.braking_longitudinal_grip_factor
                )
                raw_brake_request_n = modifiers.brake_force_n * brake
                # Normal braking is threshold-controlled.  Lockup appears
                # when a near-maximum command overlaps high lateral load,
                # where the available longitudinal share changes rapidly.
                # Normal threshold braking stays just below a reportable
                # lockup. An emergency speed-limit request deliberately has
                # less modulation reserve and may cross the combined limit.
                brake_overdrive = min(
                    0.35,
                    (0.10 if modifiers.speed_limit_factor < 0.60 else 0.06)
                    + max(0.0, modifiers.brake_modulation_error),
                )
                controlled_brake_limit_n = available_brake_force_n * (
                    0.98
                    + (
                        brake_overdrive * lateral_grip_utilization
                        if brake > 0.85 and lateral_grip_utilization > 0.80
                        else 0.0
                    )
                )
                requested_brake_force_n = min(
                    raw_brake_request_n,
                    controlled_brake_limit_n,
                )
                step_wheel_lock_ratio = min(
                    2.0,
                    max(
                    0.0,
                    (requested_brake_force_n - available_brake_force_n)
                    / max(1.0, available_brake_force_n),
                    ),
                )
                wheel_lock_ratio = max(wheel_lock_ratio, step_wheel_lock_ratio)
                post_peak_brake_force_n = available_brake_force_n * (
                    1.0 - min(0.18, step_wheel_lock_ratio * 0.14)
                )
                longitudinal_force_n = min(
                    requested_brake_force_n,
                    post_peak_brake_force_n,
                )
                tire_slide_energy_j += max(
                    0.0,
                    requested_brake_force_n - longitudinal_force_n,
                ) * speed * step
                applied_drive_force_n = 0.0
                acceleration = max(
                    -MAX_BRAKING_MPS2,
                    -(
                        longitudinal_force_n
                        + drag_force_n
                        + rolling_resistance_n
                    )
                    / mass_kg,
                )

            acceleration = max(
                -MAX_BRAKING_MPS2,
                min(
                    MAX_ACCELERATION_MPS2,
                    acceleration
                    - max(0.0, modifiers.surface_drag_deceleration_mps2),
                ),
            )

            normalized_longitudinal_force_n = (
                longitudinal_force_n / self.braking_longitudinal_grip_factor
                if brake > 0.0
                else longitudinal_force_n
            )
            grip_utilization = (
                sqrt(
                    required_lateral_force_n * required_lateral_force_n
                    + normalized_longitudinal_force_n
                    * normalized_longitudinal_force_n
                )
                / maximum_tire_force_n
                if maximum_tire_force_n > 1e-9
                else 0.0
            )

            if lateral_grip_utilization > RUN_WIDE_LATERAL_GRIP_THRESHOLD:
                handling_state = "run_wide"
            elif lateral_grip_utilization > 1.0:
                handling_state = "understeer"
            elif rear_traction_overload:
                handling_state = "oversteer"
            elif wheel_lock_ratio > LOCKUP_SLIP_RATIO_THRESHOLD:
                handling_state = "lockup"
            elif traction_slip_ratio > TRACTION_LOSS_SLIP_RATIO_THRESHOLD:
                handling_state = "wheelspin"
            else:
                handling_state = "stable"

            signed_curvature = (
                self.profile.signed_curvature_at_progress(
                    (distance / self.track_length_m) % 1.0
                )
                if self.profile is not None
                else 0.0
            )
            turn_direction = (
                1.0 if signed_curvature > 0.0
                else -1.0 if signed_curvature < 0.0
                else 0.0
            )
            grip_excess = max(
                0.0,
                max(grip_utilization, lateral_grip_utilization) - 1.0,
            )
            if handling_state != "stable":
                acceleration = max(
                    -MAX_BRAKING_MPS2,
                    acceleration - min(8.0, 1.0 + grip_excess * 18.0),
                )

            next_speed = max(0.0, speed + acceleration * step)
            next_distance = distance + 0.5 * (speed + next_speed) * step
            if following is not None and leader_distance is not None:
                next_elapsed_ratio = min(
                    1.0,
                    (elapsed + step) / max(delta_seconds, 1e-9),
                )
                leader_next_distance = following.leader_distance_m + (
                    following.leader_end_distance_m - following.leader_distance_m
                ) * next_elapsed_ratio
                maximum_distance = leader_next_distance - following.minimum_gap_m
                if next_distance > maximum_distance:
                    next_distance = maximum_distance
                    next_speed = min(next_speed, following.leader_end_speed_mps)
                    acceleration = min(0.0, (next_speed - speed) / step)
                    throttle = 0.0
                    brake = 1.0
            distance = next_distance
            speed = next_speed

            available_lateral_force_n = sqrt(
                max(
                    0.0,
                    maximum_tire_force_n * maximum_tire_force_n
                    - normalized_longitudinal_force_n
                    * normalized_longitudinal_force_n,
                )
            )
            bicycle = advance_dynamic_bicycle(
                DynamicBicycleState(
                    lateral_offset_m=(
                        lateral_offset - reference_lateral_offset
                    ),
                    lateral_speed_mps=(
                        lateral_speed - reference_lateral_speed
                    ),
                    heading_error_rad=slip_angle_rad,
                    yaw_rate_rad_s=yaw_rate,
                ),
                speed_mps=max(0.5, speed),
                curvature_1pm=signed_curvature,
                target_lateral_offset_m=(
                    target_lateral_offset_m - reference_lateral_offset
                ),
                target_lateral_speed_mps=(
                    target_lateral_speed - reference_lateral_speed
                ),
                delta_seconds=step,
                mass_kg=mass_kg,
                wheelbase_m=modifiers.wheelbase_m,
                yaw_inertia_kgm2=modifiers.yaw_inertia_kgm2,
                maximum_tire_force_n=available_lateral_force_n,
                front_force_share=modifiers.front_aero_share,
                grip_factor=modifiers.grip * modifiers.mechanical_grip,
                # Tyre-force saturation in the bicycle model already creates
                # the physical run-wide motion.  The former point-mass model's
                # extra scripted drift would count the same loss twice.
                external_lateral_acceleration_mps2=0.0,
            )
            lateral_acceleration = bicycle.lateral_acceleration_mps2
            next_reference_lateral_offset = (
                reference_lateral_offset + reference_lateral_speed * step
            )
            next_lateral_speed = max(
                -lateral_speed_limit,
                min(
                    lateral_speed_limit,
                    bicycle.lateral_speed_mps + reference_lateral_speed,
                ),
            )
            next_lateral_offset = (
                bicycle.lateral_offset_m + next_reference_lateral_offset
            )
            reference_lateral_offset = next_reference_lateral_offset
            slip_angle_rad = bicycle.heading_error_rad
            yaw_rate = bicycle.yaw_rate_rad_s
            steering_angle_rad = bicycle.steering_angle_rad
            front_slip_angle_rad = bicycle.front_slip_angle_rad
            rear_slip_angle_rad = bicycle.rear_slip_angle_rad
            absolute_front_slip = abs(front_slip_angle_rad)
            absolute_rear_slip = abs(rear_slip_angle_rad)
            if handling_state == "stable":
                if (
                    grip_utilization >= 0.80
                    and absolute_rear_slip
                    >= AXLE_SLIP_CLASSIFICATION_THRESHOLD_RAD
                    and absolute_rear_slip
                    > absolute_front_slip + AXLE_SLIP_BALANCE_MARGIN_RAD
                ):
                    handling_state = "oversteer"
                elif (
                    grip_utilization >= 0.80
                    and absolute_front_slip
                    >= AXLE_SLIP_CLASSIFICATION_THRESHOLD_RAD
                    and absolute_front_slip
                    > absolute_rear_slip + AXLE_SLIP_BALANCE_MARGIN_RAD
                ):
                    handling_state = "understeer"
            active_minimum_lateral_offset_m = minimum_lateral_offset_m
            active_maximum_lateral_offset_m = maximum_lateral_offset_m
            inside_nominal_bounds = (
                nominal_minimum_lateral_offset_m is not None
                and nominal_maximum_lateral_offset_m is not None
                and nominal_minimum_lateral_offset_m
                <= lateral_offset
                <= nominal_maximum_lateral_offset_m
            )
            if inside_nominal_bounds:
                active_minimum_lateral_offset_m = max(
                    minimum_lateral_offset_m,
                    nominal_minimum_lateral_offset_m,
                )
                active_maximum_lateral_offset_m = min(
                    maximum_lateral_offset_m,
                    nominal_maximum_lateral_offset_m,
                )
            elif (
                nominal_minimum_lateral_offset_m is not None
                and nominal_maximum_lateral_offset_m is not None
            ):
                # Width profiles can narrow under the car between frames.
                # Use the previewed body-safe envelope and cancel only the
                # outward component; forced-wide and avoidance states omit
                # nominal bounds at the caller and remain physically free.
                handling_state = "recovering"
                if lateral_offset < nominal_minimum_lateral_offset_m:
                    next_lateral_offset = max(
                        lateral_offset,
                        next_lateral_offset,
                    )
                    next_lateral_speed = max(0.0, next_lateral_speed)
                elif lateral_offset > nominal_maximum_lateral_offset_m:
                    next_lateral_offset = min(
                        lateral_offset,
                        next_lateral_offset,
                    )
                    next_lateral_speed = min(0.0, next_lateral_speed)
            if next_lateral_offset < active_minimum_lateral_offset_m:
                next_lateral_offset = active_minimum_lateral_offset_m
                next_lateral_speed = max(0.0, next_lateral_speed)
            elif next_lateral_offset > active_maximum_lateral_offset_m:
                next_lateral_offset = active_maximum_lateral_offset_m
                next_lateral_speed = min(0.0, next_lateral_speed)
            lateral_offset = next_lateral_offset
            lateral_speed = next_lateral_speed

            elapsed += step

        return VehiclePhysicsResult(
            distance_m=distance,
            speed_mps=speed,
            acceleration_mps2=acceleration,
            target_speed_mps=target,
            throttle=throttle,
            brake=brake,
            lateral_offset_m=lateral_offset,
            lateral_speed_mps=lateral_speed,
            lateral_acceleration_mps2=lateral_acceleration,
            grip_utilization=grip_utilization,
            handling_state=handling_state,
            slip_angle_rad=slip_angle_rad,
            drive_force_n=applied_drive_force_n,
            yaw_rate_rad_s=yaw_rate,
            steering_angle_rad=steering_angle_rad,
            front_slip_angle_rad=front_slip_angle_rad,
            rear_slip_angle_rad=rear_slip_angle_rad,
            wheel_lock_ratio=wheel_lock_ratio,
            traction_slip_ratio=traction_slip_ratio,
            tire_slide_energy_j=tire_slide_energy_j,
        )
