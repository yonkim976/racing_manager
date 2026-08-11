"""Metric track-width and automatically generated racing-line profiles."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass, replace
from math import atan2, floor, hypot, sqrt

from models.schemas import Circuit
from simulation.global_trajectory_optimizer import (
    GlobalTrajectoryEvaluation,
    GlobalTrajectoryOptimizationDiagnostics,
    GlobalTrajectoryOptimizationRequest,
    GlobalTrajectoryOptimizationResult,
    GlobalTrajectoryOptimizer,
    GlobalTrajectoryOptimizerConfig,
)
from simulation.track_geometry import normalize_points
from simulation.track_contracts import (
    DRIVING_LINE_DEFENSIVE,
    DRIVING_LINE_INSIDE,
    DRIVING_LINE_OUTSIDE,
    DRIVING_LINE_RACING,
    LocalMetricCoordinateFrame,
    TrackPhysicsSample,
)
from simulation.track_surface import (
    TRAJECTORY_LOW_KERB_ALLOWANCE_M,
    TrackSurfaceProfile,
)
from simulation.vehicle_dimensions import (
    PHYSICAL_CAR_LENGTH_M,
    PHYSICAL_CAR_WIDTH_M,
)
from simulation.trajectory_physics import (
    TRAJECTORY_SPEED_PASS_COUNT,
    TireTrajectorySpec,
    VehicleTrajectorySpec,
    build_trajectory_speed_profile,
)

DEFAULT_TRACK_WIDTH_M = 12.0
TRACK_EDGE_MARGIN_M = 0.35
RACING_LINE_KERB_ALLOWANCE_M = TRAJECTORY_LOW_KERB_ALLOWANCE_M
RACING_LINE_OPTIMIZATION_STEPS_M = (1.25, 0.6, 0.3)
RACING_LINE_TRANSITION_RADIUS_M = 220.0
RACING_LINE_MAX_LATERAL_SLOPE = 0.05
TACTICAL_LINE_MAX_LATERAL_SLOPE = 0.07
TACTICAL_LINE_MIN_LAP_TIME_PENALTY_SECONDS = {
    "inside": 0.18,
    "outside": 0.28,
    "defensive_line": 0.12,
}
RACING_LINE_CURVATURE_RATE_WEIGHT = 5_000.0
RACING_LINE_REFERENCE_OFFSET_WEIGHT = 0.0
RACING_LINE_INITIAL_SMOOTHING_PASSES = 3
RACING_LINE_ACCELERATION_MPS2 = 12.0
RACING_LINE_BRAKING_MPS2 = 34.0
RACING_LINE_LATERAL_ACCEL_MPS2 = 34.0
RACING_LINE_MIN_SPEED_MPS = 28.0
RACING_LINE_MAX_SPEED_MPS = 96.0
TRACK_PHYSICS_CACHE_SIZE = 32
VEHICLE_TRACK_PHYSICS_CACHE_SIZE = 128
LIVE_TRAJECTORY_OPTIMIZATION_STEPS_M = (0.6, 0.3)
LIVE_TRAJECTORY_CENTER_STRIDE = 12
LIVE_TRAJECTORY_SPEED_PASS_COUNT = 1
LIVE_TRAJECTORY_CORNER_CURVATURE_THRESHOLD_1PM = 0.004


@dataclass(frozen=True)
class RacingLinePathSample:
    """One point on the generated racing line in circuit-coordinate space."""

    center_progress: float
    path_progress: float
    path_distance_m: float
    x: float
    y: float
    lateral_offset_m: float
    curvature_1pm: float


class TrackPhysicsProfile:
    def __init__(
        self,
        samples: list[TrackPhysicsSample],
        racing_line_samples: list[RacingLinePathSample] | None = None,
        predicted_racing_lap_time: float = 0.0,
        driving_line_samples: dict[str, list[RacingLinePathSample]] | None = None,
        predicted_line_lap_times: dict[str, float] | None = None,
        optimization_diagnostics: GlobalTrajectoryOptimizationDiagnostics | None = None,
        coordinate_frame: LocalMetricCoordinateFrame | None = None,
    ) -> None:
        self.samples = samples
        self.progress = [sample.progress for sample in samples]
        self.racing_line_samples = racing_line_samples or []
        self.predicted_racing_lap_time = max(0.0, predicted_racing_lap_time)
        self.driving_line_samples = {
            DRIVING_LINE_RACING: self.racing_line_samples,
            **(driving_line_samples or {}),
        }
        self.predicted_line_lap_times = {
            DRIVING_LINE_RACING: self.predicted_racing_lap_time,
            **(predicted_line_lap_times or {}),
        }
        self.optimization_diagnostics = optimization_diagnostics
        self.coordinate_frame = coordinate_frame
        self._driving_line_lengths_m = {
            name: self._calculate_line_length_m(line_samples)
            for name, line_samples in self.driving_line_samples.items()
        }

    @property
    def racing_line_coords(self) -> list[list[float]]:
        return self.coords_for_line(DRIVING_LINE_RACING)

    @property
    def driving_line_coords(self) -> dict[str, list[list[float]]]:
        return {
            name: self.coords_for_line(name)
            for name in self.driving_line_samples
        }

    @property
    def driving_line_lengths_m(self) -> dict[str, float]:
        return {
            name: self.length_for_line(name)
            for name in self.driving_line_samples
        }

    def coords_for_line(self, name: str) -> list[list[float]]:
        coords = [
            [sample.x, sample.y]
            for sample in self.driving_line_samples.get(name, [])
        ]
        if coords:
            coords.append(coords[0][:])
        return coords

    def line_offset_at_progress(self, name: str, progress: float) -> float:
        samples = self.driving_line_samples.get(name) or self.racing_line_samples
        if not samples:
            return self.at_progress(progress).racing_line_offset_m
        index, next_index, ratio = self._line_indices_at_center_progress(progress)
        return (
            samples[index].lateral_offset_m
            + (samples[next_index].lateral_offset_m - samples[index].lateral_offset_m)
            * ratio
        )

    def line_pose_at_progress(
        self,
        name: str,
        progress: float,
    ) -> tuple[float, float, float]:
        """Return interpolated x/y and tangent heading for a driving line."""
        samples = self.driving_line_samples.get(name) or self.racing_line_samples
        if not samples:
            return 0.0, 0.0, 0.0
        index, next_index, ratio = self._line_indices_at_center_progress(progress)
        previous = samples[(index - 1) % len(samples)]
        current = samples[index]
        following = samples[next_index]
        after = samples[(next_index + 1) % len(samples)]

        # A piecewise chord position with a per-segment heading rotates the
        # lateral-offset normal abruptly at every source sample.  Cars away
        # from the reference line can therefore move several metres in one
        # 20 ms frame despite continuous progress.  A circular Catmull-Rom
        # span keeps both the path and its tangent continuous across samples.
        ratio2 = ratio * ratio
        ratio3 = ratio2 * ratio

        def interpolate_axis(p0: float, p1: float, p2: float, p3: float) -> float:
            return 0.5 * (
                2.0 * p1
                + (-p0 + p2) * ratio
                + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * ratio2
                + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * ratio3
            )

        def tangent_axis(p0: float, p1: float, p2: float, p3: float) -> float:
            return 0.5 * (
                (-p0 + p2)
                + 2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * ratio
                + 3.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * ratio2
            )

        x = interpolate_axis(previous.x, current.x, following.x, after.x)
        y = interpolate_axis(previous.y, current.y, following.y, after.y)
        dx = tangent_axis(previous.x, current.x, following.x, after.x)
        dy = tangent_axis(previous.y, current.y, following.y, after.y)
        if abs(dx) + abs(dy) <= 1e-9:
            dx = following.x - current.x
            dy = following.y - current.y
        return (
            x,
            y,
            atan2(dy, dx),
        )

    def line_pose_at_progress_m(
        self,
        name: str,
        progress: float,
    ) -> tuple[float, float, float]:
        """Return a driving-line pose in the local metre coordinate frame."""
        x_render, y_render, heading = self.line_pose_at_progress(name, progress)
        if self.coordinate_frame is None:
            raise RuntimeError("track profile has no local metric coordinate frame")
        x_m, y_m = self.coordinate_frame.to_local_m(x_render, y_render)
        return x_m, y_m, heading

    def line_distance_at_total_progress(self, name: str, total_progress: float) -> float:
        samples = self.driving_line_samples.get(name) or self.racing_line_samples
        line_length = self.length_for_line(name)
        if not samples or line_length <= 0.0:
            return total_progress
        lap = floor(total_progress)
        fraction = total_progress - lap
        index, next_index, ratio = self._line_indices_at_center_progress(fraction)
        start_distance = samples[index].path_distance_m
        end_distance = (
            line_length
            if next_index == 0
            else samples[next_index].path_distance_m
        )
        return lap * line_length + start_distance + (end_distance - start_distance) * ratio

    def total_progress_at_line_distance(self, name: str, distance_m: float) -> float:
        samples = self.driving_line_samples.get(name) or self.racing_line_samples
        line_length = self.length_for_line(name)
        if not samples or line_length <= 0.0:
            return distance_m
        lap = floor(distance_m / line_length)
        lap_distance = distance_m - lap * line_length
        distances = [sample.path_distance_m for sample in samples]
        index = bisect_right(distances, lap_distance) - 1
        if index < 0:
            index = len(samples) - 1
        next_index = (index + 1) % len(samples)
        start_distance = samples[index].path_distance_m
        end_distance = line_length if next_index == 0 else samples[next_index].path_distance_m
        start_progress = samples[index].center_progress
        end_progress = 1.0 if next_index == 0 else samples[next_index].center_progress
        ratio = min(
            1.0,
            max(0.0, (lap_distance - start_distance) / max(1e-9, end_distance - start_distance)),
        )
        return lap + start_progress + (end_progress - start_progress) * ratio

    def _line_indices_at_center_progress(self, progress: float) -> tuple[int, int, float]:
        if not self.progress:
            return 0, 0, 0.0
        normalized = progress % 1.0
        index = bisect_right(self.progress, normalized) - 1
        if index < 0:
            index = len(self.progress) - 1
        next_index = (index + 1) % len(self.progress)
        start = self.progress[index]
        end = 1.0 if next_index == 0 else self.progress[next_index]
        target = normalized if normalized >= start else normalized + 1.0
        if next_index == 0 and target < start:
            target += 1.0
        ratio = min(1.0, max(0.0, (target - start) / max(1e-9, end - start)))
        return index, next_index, ratio

    @property
    def racing_line_length_m(self) -> float:
        return self.length_for_line(DRIVING_LINE_RACING)

    def length_for_line(self, name: str) -> float:
        line_samples = self.driving_line_samples.get(name, [])
        if not line_samples:
            return 0.0
        return self._driving_line_lengths_m.get(name, 0.0)

    def _calculate_line_length_m(
        self,
        line_samples: list[RacingLinePathSample],
    ) -> float:
        if not line_samples:
            return 0.0
        last = line_samples[-1]
        first = line_samples[0]
        return (
            last.path_distance_m
            + hypot(last.x - first.x, last.y - first.y)
            * self._meters_per_unit(line_samples)
        )

    def _meters_per_unit(self, line_samples: list[RacingLinePathSample]) -> float:
        if len(line_samples) < 2:
            return 0.0
        coordinate_length = sum(
            hypot(
                line_samples[(index + 1) % len(line_samples)].x - sample.x,
                line_samples[(index + 1) % len(line_samples)].y - sample.y,
            )
            for index, sample in enumerate(line_samples)
        )
        path_distance = line_samples[-1].path_distance_m
        closing_coordinate = hypot(
            line_samples[-1].x - line_samples[0].x,
            line_samples[-1].y - line_samples[0].y,
        )
        open_coordinate = max(0.0, coordinate_length - closing_coordinate)
        return path_distance / open_coordinate if open_coordinate > 1e-9 else 0.0

    def _sample_values(self, progress: float) -> tuple[TrackPhysicsSample, TrackPhysicsSample, float]:
        if not self.samples:
            fallback = TrackPhysicsSample(0.0, 6.0, 6.0, 0.0, 0.0)
            return fallback, fallback, 0.0
        normalized = progress % 1.0
        index = bisect_right(self.progress, normalized) - 1
        if index < 0:
            index = len(self.samples) - 1
        next_index = (index + 1) % len(self.samples)
        start_progress = self.progress[index]
        end_progress = self.progress[next_index]
        if next_index == 0:
            end_progress += 1.0
        target = normalized if normalized >= start_progress else normalized + 1.0
        ratio = (target - start_progress) / max(1e-9, end_progress - start_progress)
        return self.samples[index], self.samples[next_index], min(1.0, max(0.0, ratio))

    def at_progress(self, progress: float) -> TrackPhysicsSample:
        first, second, ratio = self._sample_values(progress)
        interpolate = lambda a, b: a + (b - a) * ratio
        return TrackPhysicsSample(
            progress=progress % 1.0,
            left_width_m=interpolate(first.left_width_m, second.left_width_m),
            right_width_m=interpolate(first.right_width_m, second.right_width_m),
            racing_line_offset_m=interpolate(
                first.racing_line_offset_m,
                second.racing_line_offset_m,
            ),
            turn_signal=interpolate(first.turn_signal, second.turn_signal),
        )


_TRACK_PHYSICS_CACHE: dict[tuple[object, ...], TrackPhysicsProfile] = {}
_VEHICLE_TRACK_PHYSICS_CACHE: dict[
    tuple[object, ...],
    TrackPhysicsProfile,
] = {}


def clear_track_physics_caches() -> None:
    """Clear all profile caches for independent validation builds/tests."""
    _TRACK_PHYSICS_CACHE.clear()
    clear_vehicle_track_physics_cache()


def clear_vehicle_track_physics_cache() -> None:
    """Release the large vehicle-specific profiles owned by finished sessions."""
    _VEHICLE_TRACK_PHYSICS_CACHE.clear()


def track_physics_cache_counts() -> dict[str, int]:
    """Expose cache sizes without serializing any track profile data."""
    return {
        "common_track_physics_cache_entries": len(_TRACK_PHYSICS_CACHE),
        "vehicle_track_physics_cache_entries": len(_VEHICLE_TRACK_PHYSICS_CACHE),
    }


def _local_metric_coordinate_frame(
    points: list[tuple[float, float]],
    track_length_m: float,
) -> LocalMetricCoordinateFrame:
    render_length = sum(
        hypot(
            points[(index + 1) % len(points)][0] - point[0],
            points[(index + 1) % len(points)][1] - point[1],
        )
        for index, point in enumerate(points)
    )
    return LocalMetricCoordinateFrame(
        origin_x_render=points[0][0],
        origin_y_render=points[0][1],
        meters_per_render_unit=max(1.0, float(track_length_m)) / max(1e-9, render_length),
        track_length_m=max(1.0, float(track_length_m)),
    )


def _closed_points(circuit: Circuit) -> list[tuple[float, float]]:
    points = normalize_points(circuit.track_coords)
    if len(points) > 1 and hypot(
        points[0][0] - points[-1][0],
        points[0][1] - points[-1][1],
    ) <= 1e-6:
        points = points[:-1]
    return points


def _centerline_progress(
    points: list[tuple[float, float]],
) -> tuple[list[float], list[float]]:
    lengths = [
        hypot(
            points[(index + 1) % len(points)][0] - point[0],
            points[(index + 1) % len(points)][1] - point[1],
        )
        for index, point in enumerate(points)
    ]
    total_length = max(1e-9, sum(lengths))
    progress: list[float] = []
    cumulative = 0.0
    for length in lengths:
        progress.append(cumulative / total_length)
        cumulative += length
    return lengths, progress


def _metric_widths(
    circuit: Circuit,
    progress: list[float],
) -> list[tuple[float, float]]:
    count = len(progress)
    fallback_half = max(4.0, float(circuit.track_width_m or DEFAULT_TRACK_WIDTH_M) / 2.0)
    if circuit.track_width_profile:
        source = sorted(circuit.track_width_profile, key=lambda item: item.progress)
        source_progress = [item.progress for item in source]
        widths: list[tuple[float, float]] = []
        for target in progress:
            next_index = bisect_right(source_progress, target) % len(source)
            previous_index = (next_index - 1) % len(source)
            previous = source[previous_index]
            following = source[next_index]
            start = previous.progress
            end = following.progress
            adjusted_target = target
            if next_index == 0:
                end += 1.0
                if adjusted_target < start:
                    adjusted_target += 1.0
            ratio = (adjusted_target - start) / max(1e-9, end - start)
            widths.append(
                (
                    max(
                        4.0,
                        previous.left_width_m
                        + (following.left_width_m - previous.left_width_m) * ratio,
                    ),
                    max(
                        4.0,
                        previous.right_width_m
                        + (following.right_width_m - previous.right_width_m) * ratio,
                    ),
                )
            )
        return widths
    source = list(circuit.metric.centerline) if circuit.metric else []
    if not source:
        return [(fallback_half, fallback_half) for _ in range(count)]

    widths: list[tuple[float, float]] = []
    for index in range(count):
        source_index = min(len(source) - 1, round(index * len(source) / max(1, count)))
        point = source[source_index]
        left = float(point.w_tr_left_m) if point.w_tr_left_m is not None else fallback_half
        right = float(point.w_tr_right_m) if point.w_tr_right_m is not None else fallback_half
        widths.append((max(4.0, left), max(4.0, right)))
    return widths


def _measured_racing_line_offsets(
    circuit: Circuit,
    progress: list[float],
    widths: list[tuple[float, float]],
) -> list[float] | None:
    """Interpolate a circular measured reference onto centerline samples."""
    calibration = circuit.physics_calibration
    if calibration is None or not calibration.racing_line_reference:
        return None
    references = sorted(
        calibration.racing_line_reference,
        key=lambda sample: sample.progress,
    )
    reference_progress = [sample.progress for sample in references]
    offsets: list[float] = []
    for sample_progress, width in zip(progress, widths):
        next_index = bisect_right(reference_progress, sample_progress)
        previous = references[(next_index - 1) % len(references)]
        following = references[next_index % len(references)]
        previous_progress = previous.progress
        following_progress = following.progress
        target_progress = sample_progress
        if next_index == 0:
            previous_progress -= 1.0
        if next_index == len(references):
            following_progress += 1.0
        if target_progress < previous_progress:
            target_progress += 1.0
        span = max(1e-9, following_progress - previous_progress)
        ratio = max(0.0, min(1.0, (target_progress - previous_progress) / span))
        offset = previous.lateral_offset_m + (
            following.lateral_offset_m - previous.lateral_offset_m
        ) * ratio
        offsets.append(_clamp_offset(offset, width))
    return offsets


def _signed_turn_signal(points: list[tuple[float, float]], index: int) -> float:
    count = len(points)
    previous = points[(index - 1) % count]
    current = points[index]
    following = points[(index + 1) % count]
    incoming = (current[0] - previous[0], current[1] - previous[1])
    outgoing = (following[0] - current[0], following[1] - current[1])
    in_length = max(1e-9, hypot(*incoming))
    out_length = max(1e-9, hypot(*outgoing))
    incoming = (incoming[0] / in_length, incoming[1] / in_length)
    outgoing = (outgoing[0] / out_length, outgoing[1] / out_length)
    cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
    dot = max(-1.0, min(1.0, incoming[0] * outgoing[0] + incoming[1] * outgoing[1]))
    return max(-1.0, min(1.0, atan2(cross, dot) / 0.22))


def _centerline_normals(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    normals: list[tuple[float, float]] = []
    count = len(points)
    for index in range(count):
        previous = points[(index - 1) % count]
        following = points[(index + 1) % count]
        tangent_x = following[0] - previous[0]
        tangent_y = following[1] - previous[1]
        length = max(1e-9, hypot(tangent_x, tangent_y))
        normals.append((-tangent_y / length, tangent_x / length))
    return normals


def _offset_path_points(
    points: list[tuple[float, float]],
    normals: list[tuple[float, float]],
    offsets_m: list[float],
    units_per_meter: float,
) -> list[tuple[float, float]]:
    return [
        (
            point[0] + normal[0] * offset * units_per_meter,
            point[1] + normal[1] * offset * units_per_meter,
        )
        for point, normal, offset in zip(points, normals, offsets_m)
    ]


def _path_physics(
    path_points: list[tuple[float, float]],
    meters_per_unit: float,
) -> tuple[list[float], list[float], float]:
    """Return segment lengths, signed curvature and predicted lap time."""
    count = len(path_points)
    if count < 3:
        return [], [], 0.0
    segment_lengths = [
        hypot(
            path_points[(index + 1) % count][0] - point[0],
            path_points[(index + 1) % count][1] - point[1],
        )
        * meters_per_unit
        for index, point in enumerate(path_points)
    ]
    curvatures: list[float] = []
    for index, point in enumerate(path_points):
        previous = path_points[(index - 1) % count]
        following = path_points[(index + 1) % count]
        incoming = (point[0] - previous[0], point[1] - previous[1])
        outgoing = (following[0] - point[0], following[1] - point[1])
        incoming_length = max(1e-9, hypot(*incoming))
        outgoing_length = max(1e-9, hypot(*outgoing))
        incoming_unit = (incoming[0] / incoming_length, incoming[1] / incoming_length)
        outgoing_unit = (outgoing[0] / outgoing_length, outgoing[1] / outgoing_length)
        cross = incoming_unit[0] * outgoing_unit[1] - incoming_unit[1] * outgoing_unit[0]
        dot = max(-1.0, min(1.0, incoming_unit[0] * outgoing_unit[0] + incoming_unit[1] * outgoing_unit[1]))
        arc_length_m = max(
            1e-6,
            0.5 * (segment_lengths[(index - 1) % count] + segment_lengths[index]),
        )
        curvatures.append(atan2(cross, dot) / arc_length_m)

    speed_limits = [
        RACING_LINE_MAX_SPEED_MPS
        if abs(curvature) <= 1e-5
        else min(
            RACING_LINE_MAX_SPEED_MPS,
            max(
                RACING_LINE_MIN_SPEED_MPS,
                sqrt(RACING_LINE_LATERAL_ACCEL_MPS2 / abs(curvature)),
            ),
        )
        for curvature in curvatures
    ]
    for _ in range(4):
        for index in range(count - 1, -1, -1):
            next_index = (index + 1) % count
            speed_limits[index] = min(
                speed_limits[index],
                sqrt(
                    speed_limits[next_index] ** 2
                    + 2.0 * RACING_LINE_BRAKING_MPS2 * segment_lengths[index]
                ),
            )
        for index in range(count):
            next_index = (index + 1) % count
            speed_limits[next_index] = min(
                speed_limits[next_index],
                sqrt(
                    speed_limits[index] ** 2
                    + 2.0 * RACING_LINE_ACCELERATION_MPS2 * segment_lengths[index]
                ),
            )
    lap_time = sum(
        2.0 * segment_lengths[index]
        / max(1.0, speed_limits[index] + speed_limits[(index + 1) % count])
        for index in range(count)
    )
    return segment_lengths, curvatures, lap_time


def _clamp_offset(offset: float, widths: tuple[float, float]) -> float:
    left_width, right_width = widths
    positive_limit = left_width - PHYSICAL_CAR_WIDTH_M / 2.0 - TRACK_EDGE_MARGIN_M
    negative_limit = right_width - PHYSICAL_CAR_WIDTH_M / 2.0 - TRACK_EDGE_MARGIN_M
    return max(-negative_limit, min(positive_limit, offset))


def _project_offset_transition_limits(
    offsets: list[float],
    widths: list[tuple[float, float]],
    segment_lengths_m: list[float],
    *,
    max_lateral_slope: float = RACING_LINE_MAX_LATERAL_SLOPE,
) -> list[float]:
    """Keep lateral movement continuous enough for a driveable steering path.

    The limit is expressed as metres of lateral travel per metre along the
    centerline. Repeated forward/backward projection makes the constraint
    continuous across the start/finish join without favouring either direction.
    """
    projected = [
        _clamp_offset(offset, width)
        for offset, width in zip(offsets, widths)
    ]
    count = len(projected)
    if count < 2:
        return projected

    for _ in range(max(10, count)):
        for index in range(count):
            next_index = (index + 1) % count
            maximum_change = max_lateral_slope * max(0.0, segment_lengths_m[index])
            projected[next_index] = _clamp_offset(
                max(
                    projected[index] - maximum_change,
                    min(projected[index] + maximum_change, projected[next_index]),
                ),
                widths[next_index],
            )
        for index in range(count - 1, -1, -1):
            next_index = (index + 1) % count
            maximum_change = max_lateral_slope * max(0.0, segment_lengths_m[index])
            projected[index] = _clamp_offset(
                max(
                    projected[next_index] - maximum_change,
                    min(projected[next_index] + maximum_change, projected[index]),
                ),
                widths[index],
            )
        if _line_transitions_within_limit(
            projected,
            segment_lengths_m,
            max_lateral_slope=max_lateral_slope,
        ):
            break
    return projected


def _line_transitions_within_limit(
    offsets: list[float],
    segment_lengths_m: list[float],
    *,
    max_lateral_slope: float = RACING_LINE_MAX_LATERAL_SLOPE,
) -> bool:
    return all(
        abs(offsets[(index + 1) % len(offsets)] - offset)
        <= max_lateral_slope * max(0.0, segment_lengths_m[index]) + 1e-9
        for index, offset in enumerate(offsets)
    )


def _racing_line_objective(
    segment_lengths: list[float],
    curvatures: list[float],
    lap_time: float,
    offsets: list[float] | None = None,
    reference_offsets: list[float] | None = None,
    reference_offset_weight: float = RACING_LINE_REFERENCE_OFFSET_WEIGHT,
) -> float:
    """Balance minimum lap time with a smooth, steerable curvature profile."""
    if not segment_lengths or not curvatures:
        return lap_time
    curvature_rate_energy = sum(
        (
            curvatures[(index + 1) % len(curvatures)] - curvature
        ) ** 2
        / max(1.0, segment_lengths[index])
        for index, curvature in enumerate(curvatures)
    )
    reference_error = 0.0
    if (
        offsets is not None
        and reference_offsets is not None
        and len(offsets) == len(reference_offsets)
        and offsets
    ):
        reference_error = sum(
            (offset - reference) ** 2
            for offset, reference in zip(offsets, reference_offsets)
        ) / len(offsets)
    return (
        lap_time
        + RACING_LINE_CURVATURE_RATE_WEIGHT * curvature_rate_energy
        + reference_offset_weight * reference_error
    )


class _LegacyGlobalTrajectoryCostModel:
    """Adapter preserving the original curvature-only trajectory cost."""

    def __init__(
        self,
        points: list[tuple[float, float]],
        widths: list[tuple[float, float]],
        track_length_m: float,
        reference_offset_weight: float,
        max_lateral_slope: float = RACING_LINE_MAX_LATERAL_SLOPE,
    ) -> None:
        self.points = points
        self.widths = widths
        self.reference_offset_weight = reference_offset_weight
        self.max_lateral_slope = max(
            0.01,
            min(RACING_LINE_MAX_LATERAL_SLOPE, float(max_lateral_slope)),
        )
        source_length = sum(
            hypot(
                points[(index + 1) % len(points)][0] - point[0],
                points[(index + 1) % len(points)][1] - point[1],
            )
            for index, point in enumerate(points)
        )
        self.meters_per_unit = track_length_m / max(1e-9, source_length)
        self.units_per_meter = 1.0 / self.meters_per_unit
        self.center_segment_lengths_m = [
            hypot(
                points[(index + 1) % len(points)][0] - point[0],
                points[(index + 1) % len(points)][1] - point[1],
            )
            * self.meters_per_unit
            for index, point in enumerate(points)
        ]
        self.normals = _centerline_normals(points)

    def prepare_initial_offsets(
        self,
        offsets_m,
    ) -> tuple[float, ...]:
        return tuple(
            _project_offset_transition_limits(
                list(offsets_m),
                self.widths,
                self.center_segment_lengths_m,
                max_lateral_slope=self.max_lateral_slope,
            )
        )

    def prepare_candidate_offsets(
        self,
        offsets_m,
    ) -> tuple[float, ...]:
        return tuple(
            _clamp_offset(offset, width)
            for offset, width in zip(offsets_m, self.widths)
        )

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
        segment_lengths, curvatures, lap_time = _path_physics(
            path_points,
            self.meters_per_unit,
        )
        objective = _racing_line_objective(
            segment_lengths,
            curvatures,
            lap_time,
            offsets,
            list(reference_offsets_m),
            self.reference_offset_weight,
        )
        return GlobalTrajectoryEvaluation(
            offsets_m=tuple(offsets),
            path_points=tuple(path_points),
            segment_lengths_m=tuple(segment_lengths),
            curvatures_1pm=tuple(curvatures),
            lap_time_seconds=lap_time,
            objective_cost=objective,
        )


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


def _optimize_global_trajectory(
    points: list[tuple[float, float]],
    widths: list[tuple[float, float]],
    initial_offsets: list[float],
    track_length_m: float,
    *,
    reference_offset_weight: float = RACING_LINE_REFERENCE_OFFSET_WEIGHT,
    max_lateral_slope: float = RACING_LINE_MAX_LATERAL_SLOPE,
) -> GlobalTrajectoryOptimizationResult:
    request = GlobalTrajectoryOptimizationRequest(
        initial_offsets_m=tuple(initial_offsets),
        track_length_m=track_length_m,
    )
    cost_model = _LegacyGlobalTrajectoryCostModel(
        points,
        widths,
        track_length_m,
        reference_offset_weight,
        max_lateral_slope,
    )
    optimizer = GlobalTrajectoryOptimizer(
        GlobalTrajectoryOptimizerConfig(
            optimization_steps_m=RACING_LINE_OPTIMIZATION_STEPS_M,
            transition_radius_m=RACING_LINE_TRANSITION_RADIUS_M,
        )
    )
    return optimizer.optimize(request, cost_model)


def optimize_vehicle_trajectory(
    circuit: Circuit,
    vehicle: VehicleTrajectorySpec,
    tire: TireTrajectorySpec,
    *,
    braking_utilization: float = 1.0,
    optimizer_config: GlobalTrajectoryOptimizerConfig | None = None,
    search_speed_pass_count: int = 4,
) -> GlobalTrajectoryOptimizationResult:
    """Build a vehicle-specific nominal trajectory without changing live race state."""
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
    """Return sparse straight samples plus entry/apex/exit samples per corner.

    The previous fixed stride could skip a corner apex entirely.  This keeps
    the cold-build budget bounded while guaranteeing that each meaningful
    curvature region can independently adjust its approach, apex and exit.
    """
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
    adaptive_center_indices = _adaptive_live_trajectory_center_indices(
        base_profile,
    )
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
    center_segment_lengths_m = [
        length * meters_per_unit
        for length in source_lengths
    ]
    normals = _centerline_normals(points)
    turns = [sample.turn_signal for sample in base_profile.samples]
    driving_line_samples: dict[str, list[RacingLinePathSample]] = {}
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
        line_points = _offset_path_points(
            points,
            normals,
            line_offsets,
            units_per_meter,
        )
        line_lengths, line_curvatures, _ = _path_physics(
            line_points,
            meters_per_unit,
        )
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
        del _VEHICLE_TRACK_PHYSICS_CACHE[
            next(iter(_VEHICLE_TRACK_PHYSICS_CACHE))
        ]
    return profile


def _optimize_racing_line_offsets(
    points: list[tuple[float, float]],
    widths: list[tuple[float, float]],
    initial_offsets: list[float],
    track_length_m: float,
    *,
    reference_offset_weight: float = RACING_LINE_REFERENCE_OFFSET_WEIGHT,
    max_lateral_slope: float = RACING_LINE_MAX_LATERAL_SLOPE,
) -> tuple[list[float], list[tuple[float, float]], list[float], list[float], float]:
    """Compatibility wrapper around the explicit global optimizer interface."""
    evaluation = _optimize_global_trajectory(
        points,
        widths,
        initial_offsets,
        track_length_m,
        reference_offset_weight=reference_offset_weight,
        max_lateral_slope=max_lateral_slope,
    ).evaluation
    return (
        list(evaluation.offsets_m),
        list(evaluation.path_points),
        list(evaluation.segment_lengths_m),
        list(evaluation.curvatures_1pm),
        evaluation.lap_time_seconds,
    )


def _alternative_line_offsets(
    base_offsets: list[float],
    widths: list[tuple[float, float]],
    turns: list[float],
    track_length_m: float,
    segment_lengths_m: list[float],
    line_name: str,
) -> list[float]:
    """Generate a smooth corner-local line that rejoins the racing line."""
    count = len(base_offsets)
    if count == 0:
        return []
    average_spacing_m = track_length_m / count
    if line_name == DRIVING_LINE_DEFENSIVE:
        approach_steps = max(2, round(260.0 / average_spacing_m))
        exit_steps = max(2, round(100.0 / average_spacing_m))
        edge_fraction = 0.52
    else:
        approach_steps = max(2, round(180.0 / average_spacing_m))
        exit_steps = max(2, round(180.0 / average_spacing_m))
        edge_fraction = 0.94 if line_name == DRIVING_LINE_INSIDE else 0.72

    corner_indices = [
        index for index, turn in enumerate(turns)
        if abs(turn) >= 0.12
    ]
    offsets = base_offsets[:]
    for index in range(count):
        strongest_score = 0.0
        selected_turn = 0.0
        for corner_index in corner_indices:
            delta = (corner_index - index) % count
            if delta > count / 2:
                delta -= count
            if delta >= 0:
                if delta > approach_steps:
                    continue
                proximity = 1.0 - delta / (approach_steps + 1)
            else:
                if -delta > exit_steps:
                    continue
                proximity = 1.0 - (-delta) / (exit_steps + 1)
            score = abs(turns[corner_index]) * proximity
            if score > strongest_score:
                strongest_score = score
                selected_turn = turns[corner_index]

        if strongest_score <= 0.0 or selected_turn == 0.0:
            continue
        turn_direction = 1.0 if selected_turn > 0.0 else -1.0
        if line_name == DRIVING_LINE_OUTSIDE:
            turn_direction *= -1.0
        left_width, right_width = widths[index]
        available_width = (
            left_width - PHYSICAL_CAR_WIDTH_M / 2.0 - TRACK_EDGE_MARGIN_M
            if turn_direction > 0.0
            else right_width - PHYSICAL_CAR_WIDTH_M / 2.0 - TRACK_EDGE_MARGIN_M
        )
        target = turn_direction * max(0.0, available_width) * edge_fraction
        blend = min(1.0, strongest_score)
        offsets[index] = _clamp_offset(
            base_offsets[index] * (1.0 - blend) + target * blend,
            widths[index],
        )

    # Smooth joins back onto the racing line without erasing the corner choice.
    for _ in range(2):
        offsets = [
            _clamp_offset(
                0.2 * offsets[(index - 1) % count]
                + 0.6 * offsets[index]
                + 0.2 * offsets[(index + 1) % count],
                widths[index],
            )
            for index in range(count)
        ]
    return _project_offset_transition_limits(
        offsets,
        widths,
        segment_lengths_m,
        max_lateral_slope=TACTICAL_LINE_MAX_LATERAL_SLOPE,
    )


def _build_racing_line_samples(
    progress: list[float],
    offsets: list[float],
    path_points: list[tuple[float, float]],
    segment_lengths: list[float],
    curvatures: list[float],
) -> list[RacingLinePathSample]:
    total_length = max(1e-9, sum(segment_lengths))
    cumulative = 0.0
    samples: list[RacingLinePathSample] = []
    for index, point in enumerate(path_points):
        samples.append(RacingLinePathSample(
            center_progress=progress[index],
            path_progress=cumulative / total_length,
            path_distance_m=cumulative,
            x=round(point[0], 6),
            y=round(point[1], 6),
            lateral_offset_m=offsets[index],
            curvature_1pm=curvatures[index],
        ))
        cumulative += segment_lengths[index]
    return samples


def build_track_physics_profile(circuit: Circuit) -> TrackPhysicsProfile:
    points = _closed_points(circuit)
    if len(points) < 3:
        return TrackPhysicsProfile([])

    lengths, progress = _centerline_progress(points)
    widths = _metric_widths(circuit, progress)
    coordinate_frame = _local_metric_coordinate_frame(
        points,
        circuit.track_length_m,
    )
    calibration_key = (
        circuit.physics_calibration.model_dump_json()
        if circuit.physics_calibration is not None
        else ""
    )
    cache_key: tuple[object, ...] = (
        round(max(1.0, float(circuit.track_length_m)), 6),
        tuple(points),
        tuple(widths),
        calibration_key,
    )
    cached = _TRACK_PHYSICS_CACHE.get(cache_key)
    if cached is not None:
        # Reinsertion keeps the plain dict ordered as a small LRU cache.
        _TRACK_PHYSICS_CACHE.pop(cache_key)
        _TRACK_PHYSICS_CACHE[cache_key] = cached
        return cached

    turns = [_signed_turn_signal(points, index) for index in range(len(points))]
    optimization_widths = [
        (
            left_width + (RACING_LINE_KERB_ALLOWANCE_M if abs(turn) >= 0.08 else 0.0),
            right_width + (RACING_LINE_KERB_ALLOWANCE_M if abs(turn) >= 0.08 else 0.0),
        )
        for (left_width, right_width), turn in zip(widths, turns)
    ]
    count = len(points)
    raw_offsets: list[float] = []
    for index, turn in enumerate(turns):
        entry_turn = turns[(index + 2) % count]
        exit_turn = turns[(index - 2) % count]
        line_signal = 0.82 * turn - 0.42 * entry_turn - 0.34 * exit_turn
        left_width, right_width = widths[index]
        positive_limit = left_width - PHYSICAL_CAR_WIDTH_M / 2.0 - TRACK_EDGE_MARGIN_M
        negative_limit = right_width - PHYSICAL_CAR_WIDTH_M / 2.0 - TRACK_EDGE_MARGIN_M
        desired = line_signal * min(positive_limit, negative_limit)
        raw_offsets.append(max(-negative_limit, min(positive_limit, desired)))

    measured_offsets = _measured_racing_line_offsets(
        circuit,
        progress,
        optimization_widths,
    )
    if measured_offsets is not None:
        raw_offsets = measured_offsets

    smoothed = raw_offsets
    calibration = circuit.physics_calibration
    smoothing_passes = (
        calibration.racing_line_initial_smoothing_passes
        if calibration
        else RACING_LINE_INITIAL_SMOOTHING_PASSES
    )
    max_lateral_slope = (
        calibration.racing_line_max_lateral_slope
        if calibration
        else RACING_LINE_MAX_LATERAL_SLOPE
    )
    for _ in range(smoothing_passes):
        smoothed = [
            0.25 * smoothed[(index - 1) % count]
            + 0.5 * smoothed[index]
            + 0.25 * smoothed[(index + 1) % count]
            for index in range(count)
        ]

    optimization_result = _optimize_global_trajectory(
        points,
        optimization_widths,
        smoothed,
        max(1.0, float(circuit.track_length_m)),
        reference_offset_weight=(
            calibration.racing_line_reference_weight
            if calibration
            else RACING_LINE_REFERENCE_OFFSET_WEIGHT
        ),
        max_lateral_slope=max_lateral_slope,
    )
    optimized_trajectory = optimization_result.evaluation
    optimized_offsets = list(optimized_trajectory.offsets_m)
    racing_points = list(optimized_trajectory.path_points)
    racing_lengths = list(optimized_trajectory.segment_lengths_m)
    racing_curvatures = list(optimized_trajectory.curvatures_1pm)
    predicted_lap_time = optimized_trajectory.lap_time_seconds
    racing_line_samples = _build_racing_line_samples(
        progress,
        optimized_offsets,
        racing_points,
        racing_lengths,
        racing_curvatures,
    )

    source_length = sum(lengths)
    meters_per_unit = max(1.0, float(circuit.track_length_m)) / max(1e-9, source_length)
    units_per_meter = 1.0 / meters_per_unit
    center_segment_lengths_m = [length * meters_per_unit for length in lengths]
    normals = _centerline_normals(points)
    driving_line_samples: dict[str, list[RacingLinePathSample]] = {}
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
        line_points = _offset_path_points(
            points,
            normals,
            line_offsets,
            units_per_meter,
        )
        line_lengths, line_curvatures, line_lap_time = _path_physics(
            line_points,
            meters_per_unit,
        )
        line_lap_time = max(
            line_lap_time,
            predicted_lap_time
            + TACTICAL_LINE_MIN_LAP_TIME_PENALTY_SECONDS[line_name],
        )
        driving_line_samples[line_name] = _build_racing_line_samples(
            progress,
            line_offsets,
            line_points,
            line_lengths,
            line_curvatures,
        )
        predicted_line_lap_times[line_name] = line_lap_time

    profile = TrackPhysicsProfile(
        [
            TrackPhysicsSample(
                progress=progress[index],
                left_width_m=widths[index][0],
                right_width_m=widths[index][1],
                racing_line_offset_m=optimized_offsets[index],
                turn_signal=turns[index],
            )
            for index in range(count)
        ],
        racing_line_samples,
        predicted_lap_time,
        driving_line_samples,
        predicted_line_lap_times,
        optimization_result.diagnostics,
        coordinate_frame,
    )
    _TRACK_PHYSICS_CACHE[cache_key] = profile
    if len(_TRACK_PHYSICS_CACHE) > TRACK_PHYSICS_CACHE_SIZE:
        del _TRACK_PHYSICS_CACHE[next(iter(_TRACK_PHYSICS_CACHE))]
    return profile
