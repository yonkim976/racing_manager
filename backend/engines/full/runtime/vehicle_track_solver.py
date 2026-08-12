"""FULL vehicle-specific racing-line optimization and profile cache."""

from __future__ import annotations

from dataclasses import replace
from math import hypot

from models.schemas import Circuit
from simulation.global_trajectory_optimizer import (
    GlobalTrajectoryEvaluation,
    GlobalTrajectoryOptimizationRequest,
    GlobalTrajectoryOptimizationResult,
    GlobalTrajectoryOptimizer,
    GlobalTrajectoryOptimizerConfig,
)
from simulation.track_contracts import (
    DRIVING_LINE_DEFENSIVE,
    DRIVING_LINE_INSIDE,
    DRIVING_LINE_OUTSIDE,
    TrackPhysicsSample,
)
from simulation.vehicle_dimensions import (
    PHYSICAL_CAR_LENGTH_M,
    PHYSICAL_CAR_WIDTH_M,
)
from simulation.track_physics import (
    LIVE_TRAJECTORY_CENTER_STRIDE,
    LIVE_TRAJECTORY_CORNER_CURVATURE_THRESHOLD_1PM,
    LIVE_TRAJECTORY_OPTIMIZATION_STEPS_M,
    LIVE_TRAJECTORY_SPEED_PASS_COUNT,
    RACING_LINE_MAX_LATERAL_SLOPE,
    RACING_LINE_OPTIMIZATION_STEPS_M,
    RACING_LINE_REFERENCE_OFFSET_WEIGHT,
    RACING_LINE_TRANSITION_RADIUS_M,
    TACTICAL_LINE_MIN_LAP_TIME_PENALTY_SECONDS,
    TRACK_EDGE_MARGIN_M,
    VEHICLE_TRACK_PHYSICS_CACHE_SIZE,
    TrackPhysicsProfile,
    _LegacyGlobalTrajectoryCostModel,
    _alternative_line_offsets,
    _build_racing_line_samples,
    _centerline_normals,
    _centerline_progress,
    _closed_points,
    _line_transitions_within_limit,
    _local_metric_coordinate_frame,
    _metric_widths,
    _offset_path_points,
    _path_physics,
    _racing_line_objective,
    build_track_physics_profile,
)

from .track_surface import TrackSurfaceProfile
from .trajectory_physics import (
    TRAJECTORY_SPEED_PASS_COUNT,
    TireTrajectorySpec,
    VehicleTrajectorySpec,
    build_trajectory_speed_profile,
)


_VEHICLE_TRACK_PHYSICS_CACHE: dict[
    tuple[object, ...],
    TrackPhysicsProfile,
] = {}


def clear_vehicle_track_physics_cache() -> None:
    """Release large vehicle-specific profiles owned by finished sessions."""
    _VEHICLE_TRACK_PHYSICS_CACHE.clear()


def vehicle_track_physics_cache_count() -> int:
    return len(_VEHICLE_TRACK_PHYSICS_CACHE)


class PhysicalGlobalTrajectoryCostModel(_LegacyGlobalTrajectoryCostModel):
    """Whole-lap cost model driven by nominal car and tyre force limits."""

    def __init__(
        self,
        points: list[tuple[float, float]],
        widths: list[tuple[float, float]],
        track_length_m: float,
        reference_offset_weight: float,
        vehicle: VehicleTrajectorySpec,
        tire: TireTrajectorySpec,
        *,
        max_lateral_slope: float = RACING_LINE_MAX_LATERAL_SLOPE,
        braking_utilization: float = 1.0,
        speed_pass_count: int = 4,
        surface_profile: TrackSurfaceProfile | None = None,
        trajectory_progress: list[float] | tuple[float, ...] | None = None,
    ) -> None:
        super().__init__(
            points,
            widths,
            track_length_m,
            reference_offset_weight,
            max_lateral_slope,
        )
        self.vehicle = vehicle
        self.tire = tire
        self.braking_utilization = braking_utilization
        self.speed_pass_count = max(1, speed_pass_count)
        self.surface_profile = surface_profile
        self.trajectory_progress = tuple(trajectory_progress or ())

    def evaluate(
        self,
        offsets_m,
        reference_offsets_m,
    ) -> GlobalTrajectoryEvaluation | None:
        offsets = list(offsets_m)
        if not _line_transitions_within_limit(
            offsets,
            self.center_segment_lengths_m,
            max_lateral_slope=self.max_lateral_slope,
        ):
            return None
        path_points = _offset_path_points(
            self.points,
            self.normals,
            offsets,
            self.units_per_meter,
        )
        segment_lengths, curvatures, _ = _path_physics(
            path_points,
            self.meters_per_unit,
        )
        speed_profile = build_trajectory_speed_profile(
            segment_lengths,
            curvatures,
            self.vehicle,
            self.tire,
            braking_utilization=self.braking_utilization,
            pass_count=self.speed_pass_count,
        )
        surface_cost_seconds = 0.0
        if self.surface_profile is not None:
            progresses = self.trajectory_progress or tuple(
                index / len(offsets)
                for index in range(len(offsets))
            )
            assessment = self.surface_profile.assess_trajectory(
                progresses,
                offsets,
                track_length_m=sum(self.center_segment_lengths_m),
                body_width_m=PHYSICAL_CAR_WIDTH_M,
                body_length_m=PHYSICAL_CAR_LENGTH_M,
                edge_margin_m=TRACK_EDGE_MARGIN_M,
            )
            if assessment.body_boundary_violations > 0:
                return None
            surface_cost_seconds = assessment.cost_seconds
        objective = _racing_line_objective(
            segment_lengths,
            curvatures,
            speed_profile.lap_time_seconds,
            offsets,
            list(reference_offsets_m),
            self.reference_offset_weight,
        ) + surface_cost_seconds
        return GlobalTrajectoryEvaluation(
            offsets_m=tuple(offsets),
            path_points=tuple(path_points),
            segment_lengths_m=tuple(segment_lengths),
            curvatures_1pm=tuple(curvatures),
            lap_time_seconds=speed_profile.lap_time_seconds,
            objective_cost=objective,
            target_speeds_mps=speed_profile.target_speeds_mps,
            brake_utilization=speed_profile.brake_utilization,
            throttle_utilization=speed_profile.throttle_utilization,
        )


def optimize_vehicle_trajectory(
    circuit: Circuit,
    vehicle: VehicleTrajectorySpec,
    tire: TireTrajectorySpec,
    *,
    braking_utilization: float = 1.0,
    optimizer_config: GlobalTrajectoryOptimizerConfig | None = None,
    search_speed_pass_count: int = 4,
) -> GlobalTrajectoryOptimizationResult:
    """Build a vehicle-specific nominal trajectory without changing race state."""
    base_profile = build_track_physics_profile(circuit)
    points = _closed_points(circuit)
    surface_profile = TrackSurfaceProfile.for_circuit(circuit, base_profile)
    optimization_widths = [
        surface_profile.trajectory_optimization_widths(sample.progress)
        for sample in base_profile.samples
    ]
    initial_offsets = [
        sample.racing_line_offset_m
        for sample in base_profile.samples
    ]
    calibration = circuit.physics_calibration
    reference_offset_weight = (
        calibration.racing_line_reference_weight
        if calibration
        else RACING_LINE_REFERENCE_OFFSET_WEIGHT
    )
    max_lateral_slope = (
        calibration.racing_line_max_lateral_slope
        if calibration
        else RACING_LINE_MAX_LATERAL_SLOPE
    )
    cost_model = PhysicalGlobalTrajectoryCostModel(
        points,
        optimization_widths,
        max(1.0, float(circuit.track_length_m)),
        reference_offset_weight,
        vehicle,
        tire,
        max_lateral_slope=max_lateral_slope,
        braking_utilization=braking_utilization,
        speed_pass_count=search_speed_pass_count,
        surface_profile=surface_profile,
        trajectory_progress=base_profile.progress,
    )
    optimizer = GlobalTrajectoryOptimizer(
        optimizer_config
        or GlobalTrajectoryOptimizerConfig(
            optimization_steps_m=RACING_LINE_OPTIMIZATION_STEPS_M,
            transition_radius_m=RACING_LINE_TRANSITION_RADIUS_M,
        )
    )
    result = optimizer.optimize(
        GlobalTrajectoryOptimizationRequest(
            initial_offsets_m=tuple(initial_offsets),
            track_length_m=max(1.0, float(circuit.track_length_m)),
        ),
        cost_model,
    )
    final_model = PhysicalGlobalTrajectoryCostModel(
        points,
        optimization_widths,
        max(1.0, float(circuit.track_length_m)),
        reference_offset_weight,
        vehicle,
        tire,
        max_lateral_slope=max_lateral_slope,
        braking_utilization=braking_utilization,
        speed_pass_count=TRAJECTORY_SPEED_PASS_COUNT,
        surface_profile=surface_profile,
        trajectory_progress=base_profile.progress,
    )
    reference_offsets = final_model.prepare_initial_offsets(initial_offsets)
    final_evaluation = final_model.evaluate(
        result.evaluation.offsets_m,
        reference_offsets,
    )
    if final_evaluation is None:
        raise ValueError("physical optimizer produced an invalid final trajectory")
    return replace(
        result,
        evaluation=final_evaluation,
        diagnostics=replace(
            result.diagnostics,
            final_objective_cost=final_evaluation.objective_cost,
        ),
    )


def _adaptive_live_trajectory_center_indices(
    base_profile: TrackPhysicsProfile,
) -> tuple[int, ...]:
    """Return sparse straight samples plus entry/apex/exit per corner."""
    curvatures = [
        abs(sample.curvature_1pm)
        for sample in base_profile.racing_line_samples
    ]
    count = len(curvatures)
    if count == 0:
        return ()

    selected = set(range(0, count, LIVE_TRAJECTORY_CENTER_STRIDE))
    active = [
        index
        for index, curvature in enumerate(curvatures)
        if curvature >= LIVE_TRAJECTORY_CORNER_CURVATURE_THRESHOLD_1PM
    ]
    if not active:
        return tuple(sorted(selected))

    runs: list[list[int]] = []
    current = [active[0]]
    for index in active[1:]:
        if index == current[-1] + 1:
            current.append(index)
        else:
            runs.append(current)
            current = [index]
    runs.append(current)
    if len(runs) > 1 and runs[0][0] == 0 and runs[-1][-1] == count - 1:
        runs[0] = runs[-1] + runs[0]
        runs.pop()

    for run in runs:
        apex = max(run, key=lambda index: curvatures[index])
        selected.update((run[0], apex, run[-1]))

    return tuple(sorted(selected))


def build_vehicle_track_physics_profile(
    circuit: Circuit,
    vehicle: VehicleTrajectorySpec,
    tire: TireTrajectorySpec,
) -> TrackPhysicsProfile:
    """Return a cached, live-budget vehicle-specific set of driving lines."""
    points = _closed_points(circuit)
    _, center_progress = _centerline_progress(points)
    widths = _metric_widths(circuit, center_progress)
    coordinate_frame = _local_metric_coordinate_frame(
        points,
        circuit.track_length_m,
    )
    calibration_key = (
        circuit.physics_calibration.model_dump_json()
        if circuit.physics_calibration is not None
        else ""
    )
    base_profile = build_track_physics_profile(circuit)
    adaptive_center_indices = _adaptive_live_trajectory_center_indices(base_profile)
    cache_key: tuple[object, ...] = (
        round(max(1.0, float(circuit.track_length_m)), 6),
        tuple(points),
        tuple(widths),
        calibration_key,
        vehicle,
        tire,
        LIVE_TRAJECTORY_OPTIMIZATION_STEPS_M,
        LIVE_TRAJECTORY_CENTER_STRIDE,
        LIVE_TRAJECTORY_SPEED_PASS_COUNT,
        adaptive_center_indices,
    )
    cached = _VEHICLE_TRACK_PHYSICS_CACHE.get(cache_key)
    if cached is not None:
        _VEHICLE_TRACK_PHYSICS_CACHE.pop(cache_key)
        _VEHICLE_TRACK_PHYSICS_CACHE[cache_key] = cached
        return cached

    optimization_result = optimize_vehicle_trajectory(
        circuit,
        vehicle,
        tire,
        optimizer_config=GlobalTrajectoryOptimizerConfig(
            optimization_steps_m=LIVE_TRAJECTORY_OPTIMIZATION_STEPS_M,
            transition_radius_m=RACING_LINE_TRANSITION_RADIUS_M,
            center_stride=LIVE_TRAJECTORY_CENTER_STRIDE,
            center_indices=adaptive_center_indices,
        ),
        search_speed_pass_count=LIVE_TRAJECTORY_SPEED_PASS_COUNT,
    )
    optimized = optimization_result.evaluation
    optimized_offsets = list(optimized.offsets_m)
    racing_points = list(optimized.path_points)
    racing_lengths = list(optimized.segment_lengths_m)
    racing_curvatures = list(optimized.curvatures_1pm)
    racing_line_samples = _build_racing_line_samples(
        base_profile.progress,
        optimized_offsets,
        racing_points,
        racing_lengths,
        racing_curvatures,
    )

    source_lengths = [
        hypot(
            points[(index + 1) % len(points)][0] - point[0],
            points[(index + 1) % len(points)][1] - point[1],
        )
        for index, point in enumerate(points)
    ]
    meters_per_unit = max(1.0, float(circuit.track_length_m)) / max(
        1e-9,
        sum(source_lengths),
    )
    units_per_meter = 1.0 / meters_per_unit
    center_segment_lengths_m = [length * meters_per_unit for length in source_lengths]
    normals = _centerline_normals(points)
    turns = [sample.turn_signal for sample in base_profile.samples]
    driving_line_samples = {}
    predicted_line_lap_times: dict[str, float] = {}
    for line_name in (
        DRIVING_LINE_INSIDE,
        DRIVING_LINE_OUTSIDE,
        DRIVING_LINE_DEFENSIVE,
    ):
        line_offsets = _alternative_line_offsets(
            optimized_offsets,
            widths,
            turns,
            max(1.0, float(circuit.track_length_m)),
            center_segment_lengths_m,
            line_name,
        )
        line_points = _offset_path_points(points, normals, line_offsets, units_per_meter)
        line_lengths, line_curvatures, _ = _path_physics(line_points, meters_per_unit)
        line_speed_profile = build_trajectory_speed_profile(
            line_lengths,
            line_curvatures,
            vehicle,
            tire,
        )
        predicted_line_lap_times[line_name] = max(
            line_speed_profile.lap_time_seconds,
            optimized.lap_time_seconds
            + TACTICAL_LINE_MIN_LAP_TIME_PENALTY_SECONDS[line_name],
        )
        driving_line_samples[line_name] = _build_racing_line_samples(
            base_profile.progress,
            line_offsets,
            line_points,
            line_lengths,
            line_curvatures,
        )

    profile = TrackPhysicsProfile(
        [
            TrackPhysicsSample(
                progress=sample.progress,
                left_width_m=sample.left_width_m,
                right_width_m=sample.right_width_m,
                racing_line_offset_m=optimized_offsets[index],
                turn_signal=sample.turn_signal,
            )
            for index, sample in enumerate(base_profile.samples)
        ],
        racing_line_samples,
        optimized.lap_time_seconds,
        driving_line_samples,
        predicted_line_lap_times,
        optimization_result.diagnostics,
        coordinate_frame,
    )
    _VEHICLE_TRACK_PHYSICS_CACHE[cache_key] = profile
    if len(_VEHICLE_TRACK_PHYSICS_CACHE) > VEHICLE_TRACK_PHYSICS_CACHE_SIZE:
        del _VEHICLE_TRACK_PHYSICS_CACHE[next(iter(_VEHICLE_TRACK_PHYSICS_CACHE))]
    return profile
