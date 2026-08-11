"""Immutable display geometry compiled from the shared FULL track profile."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, ceil, cos, hypot, sin
from typing import Any, Iterable

from models.schemas import Circuit
from simulation.start_grid_geometry import GridDisplaySlot, build_grid_slots
from simulation.track_physics import (
    DRIVING_LINE_RACING,
    LocalMetricCoordinateFrame,
    TrackPhysicsProfile,
    build_track_physics_profile,
)
from simulation.vehicle_dimensions import (
    PHYSICAL_CAR_LENGTH_M,
    PHYSICAL_CAR_WIDTH_M,
    PHYSICAL_CAR_WHEELBASE_M,
)


PROGRESS_EPSILON = 1e-9
PIT_ROUTE_ANCHOR_MERGE_DISTANCE_M = 5.0
PIT_ROUTE_TANGENT_LEAD_M = 8.0
PIT_EXIT_LANE_SAMPLE_SPACING_M = 4.0


Point = tuple[float, float]


def _coord_pair(point: Any) -> Point:
    if hasattr(point, "x") and hasattr(point, "y"):
        return float(point.x), float(point.y)
    return float(point[0]), float(point[1])


def _render_coords(points: Iterable[Point]) -> tuple[tuple[float, float], ...]:
    # Keep the compiled circuit's existing float precision.  RaceInfo has
    # historically exposed these coordinates without presentation rounding,
    # and the shared contract must preserve the FULL anchor values exactly.
    return tuple((float(x), float(y)) for x, y in points)


def _progress_distance(start: float, target: float) -> float:
    return (target - start) % 1.0


def _pit_progress(circuit: Circuit, name: str) -> float | None:
    pit_lane = circuit.pit_lane
    if pit_lane is None:
        return None
    value = getattr(pit_lane, name, None)
    return None if value is None else float(value) % 1.0


def build_pit_route_points_m(
    circuit: Circuit,
    track_profile: TrackPhysicsProfile,
) -> tuple[Point, ...]:
    """Build the authoritative pit route in the profile's local metre frame."""

    entry = _pit_progress(circuit, "entry_progress")
    exit_ = _pit_progress(circuit, "exit_progress")
    coordinate_frame = track_profile.coordinate_frame
    if (
        entry is None
        or exit_ is None
        or coordinate_frame is None
        or len(circuit.pit_lane_coords) < 2
    ):
        return ()

    entry_x, entry_y, entry_heading = track_profile.line_pose_at_progress_m(
        DRIVING_LINE_RACING,
        entry,
    )
    exit_x, exit_y, exit_heading = track_profile.line_pose_at_progress_m(
        DRIVING_LINE_RACING,
        exit_,
    )
    interior_points = [
        coordinate_frame.to_local_m(*_coord_pair(point))
        for point in circuit.pit_lane_coords
    ]
    while interior_points and hypot(
        interior_points[0][0] - entry_x,
        interior_points[0][1] - entry_y,
    ) < PIT_ROUTE_ANCHOR_MERGE_DISTANCE_M:
        interior_points.pop(0)
    while interior_points and hypot(
        interior_points[-1][0] - exit_x,
        interior_points[-1][1] - exit_y,
    ) < PIT_ROUTE_ANCHOR_MERGE_DISTANCE_M:
        interior_points.pop()

    points: list[Point] = [(entry_x, entry_y)]
    if interior_points:
        entry_distance = hypot(
            interior_points[0][0] - entry_x,
            interior_points[0][1] - entry_y,
        )
        entry_lead = min(PIT_ROUTE_TANGENT_LEAD_M, entry_distance * 0.4)
        points.append(
            (
                entry_x + cos(entry_heading) * entry_lead,
                entry_y + sin(entry_heading) * entry_lead,
            )
        )
    points.extend(interior_points)
    if interior_points:
        exit_distance = hypot(
            interior_points[-1][0] - exit_x,
            interior_points[-1][1] - exit_y,
        )
        exit_lead = min(PIT_ROUTE_TANGENT_LEAD_M, exit_distance * 0.4)
        points.append(
            (
                exit_x - cos(exit_heading) * exit_lead,
                exit_y - sin(exit_heading) * exit_lead,
            )
        )
    points.append((exit_x, exit_y))

    deduplicated = [points[0]]
    for point in points[1:]:
        if hypot(point[0] - deduplicated[-1][0], point[1] - deduplicated[-1][1]) > 1e-9:
            deduplicated.append(point)
    return tuple(deduplicated)


def _pit_lane_pose_at_progress_m(
    circuit: Circuit,
    track_profile: TrackPhysicsProfile,
    lane_progress: float,
) -> tuple[float, float, float]:
    points = build_pit_route_points_m(circuit, track_profile)
    entry = _pit_progress(circuit, "entry_progress")
    exit_ = _pit_progress(circuit, "exit_progress")
    if not points or entry is None or exit_ is None:
        return track_profile.line_pose_at_progress_m(DRIVING_LINE_RACING, entry or 0.0)

    clamped = min(1.0, max(0.0, lane_progress))
    if clamped <= 0.0:
        return track_profile.line_pose_at_progress_m(DRIVING_LINE_RACING, entry)
    if clamped >= 1.0:
        return track_profile.line_pose_at_progress_m(DRIVING_LINE_RACING, exit_)

    segment_lengths = [
        hypot(next_point[0] - point[0], next_point[1] - point[1])
        for point, next_point in zip(points, points[1:])
    ]
    total_length = sum(segment_lengths)
    if total_length <= 1e-9:
        return points[0][0], points[0][1], 0.0
    target_distance = clamped * total_length
    traversed = 0.0
    for index, segment_length in enumerate(segment_lengths):
        if traversed + segment_length >= target_distance:
            ratio = (target_distance - traversed) / max(segment_length, 1e-9)
            point = points[index]
            next_point = points[index + 1]
            return (
                point[0] + (next_point[0] - point[0]) * ratio,
                point[1] + (next_point[1] - point[1]) * ratio,
                atan2(next_point[1] - point[1], next_point[0] - point[0]),
            )
        traversed += segment_length
    point = points[-2]
    next_point = points[-1]
    return next_point[0], next_point[1], atan2(
        next_point[1] - point[1],
        next_point[0] - point[0],
    )


def _pit_exit_lane_start_track_progress(
    circuit: Circuit,
    track_profile: TrackPhysicsProfile,
) -> float | None:
    pit_lane = circuit.pit_lane
    entry = _pit_progress(circuit, "entry_progress")
    exit_ = _pit_progress(circuit, "exit_progress")
    if pit_lane is None or entry is None or exit_ is None:
        return None
    pit_x, pit_y, _ = _pit_lane_pose_at_progress_m(
        circuit,
        track_profile,
        pit_lane.side_rejoin_progress,
    )
    progress = (
        entry + _progress_distance(entry, exit_) * pit_lane.side_rejoin_progress
    ) % 1.0
    line_length_m = max(1.0, track_profile.length_for_line(DRIVING_LINE_RACING))
    for _ in range(4):
        line_x, line_y, line_heading = track_profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            progress,
        )
        tangent_delta_m = (
            (pit_x - line_x) * cos(line_heading)
            + (pit_y - line_y) * sin(line_heading)
        )
        progress = (progress + tangent_delta_m / line_length_m) % 1.0
    return progress


def build_pit_exit_lane_points_m(
    circuit: Circuit,
    track_profile: TrackPhysicsProfile,
) -> tuple[Point, ...]:
    """Build the dedicated post-pit side lane in local metre coordinates."""

    pit_lane = circuit.pit_lane
    if pit_lane is None or pit_lane.exit_lane_rejoin_progress is None:
        return ()
    start_progress = _pit_exit_lane_start_track_progress(circuit, track_profile)
    if start_progress is None:
        return ()
    rejoin_progress = pit_lane.exit_lane_rejoin_progress % 1.0
    progress_range = _progress_distance(start_progress, rejoin_progress)
    if progress_range <= PROGRESS_EPSILON:
        return ()

    start_x, start_y, _ = _pit_lane_pose_at_progress_m(
        circuit,
        track_profile,
        pit_lane.side_rejoin_progress,
    )
    line_x, line_y, line_heading = track_profile.line_pose_at_progress_m(
        DRIVING_LINE_RACING,
        start_progress,
    )
    start_lateral_delta_m = (
        (start_x - line_x) * -sin(line_heading)
        + (start_y - line_y) * cos(line_heading)
    )
    sample_count = max(
        12,
        ceil(progress_range * track_profile.coordinate_frame.track_length_m / PIT_EXIT_LANE_SAMPLE_SPACING_M),
    )
    merge_start = pit_lane.exit_lane_merge_start
    points: list[Point] = []
    for index in range(sample_count + 1):
        route_progress = index / sample_count
        track_progress = (start_progress + progress_range * route_progress) % 1.0
        x, y, heading = track_profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            track_progress,
        )
        if route_progress <= merge_start:
            lateral_scale = 1.0
        else:
            merge_ratio = (route_progress - merge_start) / max(
                PROGRESS_EPSILON,
                1.0 - merge_start,
            )
            merge_ratio = min(1.0, max(0.0, merge_ratio))
            smooth = merge_ratio * merge_ratio * (3.0 - 2.0 * merge_ratio)
            lateral_scale = 1.0 - smooth
        lateral_delta_m = start_lateral_delta_m * lateral_scale
        points.append(
            (
                x - sin(heading) * lateral_delta_m,
                y + cos(heading) * lateral_delta_m,
            )
        )
    points[0] = (start_x, start_y)
    return tuple(points)


def _to_render_coords(
    points_m: Iterable[Point],
    coordinate_frame: LocalMetricCoordinateFrame,
) -> tuple[tuple[float, float], ...]:
    return _render_coords(coordinate_frame.from_local_m(x_m, y_m) for x_m, y_m in points_m)


@dataclass(frozen=True, slots=True)
class TrackDisplayGeometry:
    """Immutable shared render/metric geometry for one compiled circuit.

    Track paths remain in ``*_render`` coordinate space for canvas consumers;
    pose samplers return local metric metres through the retained compiled
    ``TrackPhysicsProfile``.  The profile is intentionally presentation-only
    and is excluded from serialization and result hashes.
    """

    circuit_id: int | str
    coordinate_frame: LocalMetricCoordinateFrame
    track_length_m: float
    track_width_m: float
    car_width_m: float
    car_length_m: float
    wheelbase_m: float
    track_coords: tuple[tuple[float, float], ...]
    racing_line_coords: tuple[tuple[float, float], ...]
    racing_line_profile: tuple[tuple[float, float], ...]
    track_width_profile: tuple[tuple[float, float, float], ...]
    racing_line_length_m: float
    predicted_racing_lap_time: float
    driving_line_coords: tuple[tuple[str, tuple[tuple[float, float], ...]], ...]
    driving_line_lengths_m: tuple[tuple[str, float], ...]
    predicted_line_lap_times: tuple[tuple[str, float], ...]
    grid_slots: tuple[GridDisplaySlot, ...]
    pit_lane_coords: tuple[tuple[float, float], ...]
    pit_exit_lane_coords: tuple[tuple[float, float], ...]
    pit_wall_coords: tuple[tuple[float, float], ...]
    start_finish_index: int
    pit_box_offset: float
    pit_lane_width_m: float
    pit_speed_limit_kph: float
    pit_entry_progress: float
    pit_exit_progress: float
    pit_side_entry_progress: float
    pit_speed_limit_start: float
    pit_box_progress: float
    pit_speed_limit_end: float
    pit_side_rejoin_progress: float
    _compiled_profile: TrackPhysicsProfile = field(repr=False, compare=False)

    @property
    def world_origin_x_render(self) -> float:
        return self.coordinate_frame.origin_x_render

    @property
    def world_origin_y_render(self) -> float:
        return self.coordinate_frame.origin_y_render

    @property
    def world_meters_per_render_unit(self) -> float:
        return self.coordinate_frame.meters_per_render_unit

    @property
    def compiled_profile(self) -> TrackPhysicsProfile:
        """Return the compiled profile used by the public pose sampler."""
        return self._compiled_profile

    def line_pose_at_progress_m(
        self,
        line_name: str,
        progress: float,
    ) -> tuple[float, float, float]:
        return self._compiled_profile.line_pose_at_progress_m(line_name, progress)

    def to_race_info_geometry(self) -> dict[str, Any]:
        """Serialize only the shared geometry fields for RaceInfo consumers."""
        return {
            "track_length_m": self.track_length_m,
            "world_origin_x_render": self.world_origin_x_render,
            "world_origin_y_render": self.world_origin_y_render,
            "world_meters_per_render_unit": self.world_meters_per_render_unit,
            "track_width_m": self.track_width_m,
            "car_width_m": self.car_width_m,
            "car_length_m": self.car_length_m,
            "wheelbase_m": self.wheelbase_m,
            "grid_slots": [slot.to_dict() for slot in self.grid_slots],
            "racing_line_profile": [list(sample) for sample in self.racing_line_profile],
            "track_width_profile": [list(sample) for sample in self.track_width_profile],
            "racing_line_coords": [list(point) for point in self.racing_line_coords],
            "racing_line_length_m": self.racing_line_length_m,
            "predicted_racing_lap_time": self.predicted_racing_lap_time,
            "driving_line_coords": {
                name: [list(point) for point in coords]
                for name, coords in self.driving_line_coords
            },
            "driving_line_lengths_m": dict(self.driving_line_lengths_m),
            "predicted_line_lap_times": dict(self.predicted_line_lap_times),
            "track_coords": [list(point) for point in self.track_coords],
            "start_finish_index": self.start_finish_index,
            "pit_lane_coords": [list(point) for point in self.pit_lane_coords],
            "pit_exit_lane_coords": [list(point) for point in self.pit_exit_lane_coords],
            "pit_wall_coords": [list(point) for point in self.pit_wall_coords],
            "pit_box_offset": self.pit_box_offset,
            "pit_lane_width_m": self.pit_lane_width_m,
            "pit_speed_limit_kph": self.pit_speed_limit_kph,
            "pit_entry_progress": self.pit_entry_progress,
            "pit_exit_progress": self.pit_exit_progress,
            "pit_side_entry_progress": self.pit_side_entry_progress,
            "pit_speed_limit_start": self.pit_speed_limit_start,
            "pit_box_progress": self.pit_box_progress,
            "pit_speed_limit_end": self.pit_speed_limit_end,
            "pit_side_rejoin_progress": self.pit_side_rejoin_progress,
        }


def build_track_display_geometry(
    circuit: Circuit,
    *,
    track_profile: TrackPhysicsProfile | None = None,
    grid_driver_ids: Iterable[int | str] = (),
    start_sequence_enabled: bool = True,
) -> TrackDisplayGeometry:
    """Compile one immutable geometry DTO from the public circuit/profile contract."""

    profile = track_profile or build_track_physics_profile(circuit)
    coordinate_frame = profile.coordinate_frame
    if coordinate_frame is None:
        raise ValueError(f"circuit {circuit.id} has no local metric coordinate frame")
    track_coords = _render_coords(_coord_pair(point) for point in circuit.track_coords)
    pit_lane_coords = _to_render_coords(
        build_pit_route_points_m(circuit, profile),
        coordinate_frame,
    )
    if not pit_lane_coords:
        pit_lane_coords = _render_coords(_coord_pair(point) for point in circuit.pit_lane_coords)
    pit_exit_lane_coords = _to_render_coords(
        build_pit_exit_lane_points_m(circuit, profile),
        coordinate_frame,
    )
    pit_lane = circuit.pit_lane
    driving_line_coords = tuple(
        (
            name,
            _render_coords(coords),
        )
        for name, coords in sorted(profile.driving_line_coords.items(), key=lambda item: item[0])
    )
    return TrackDisplayGeometry(
        circuit_id=circuit.id,
        coordinate_frame=coordinate_frame,
        track_length_m=profile.coordinate_frame.track_length_m,
        track_width_m=float(circuit.track_width_m),
        car_width_m=PHYSICAL_CAR_WIDTH_M,
        car_length_m=PHYSICAL_CAR_LENGTH_M,
        wheelbase_m=PHYSICAL_CAR_WHEELBASE_M,
        track_coords=track_coords,
        racing_line_coords=_render_coords(profile.racing_line_coords),
        racing_line_profile=tuple(
            (
                round(sample.progress, 9),
                round(sample.racing_line_offset_m, 9),
            )
            for sample in profile.samples
        ),
        track_width_profile=tuple(
            (
                round(sample.progress, 9),
                round(sample.left_width_m, 9),
                round(sample.right_width_m, 9),
            )
            for sample in profile.samples
        ),
        racing_line_length_m=profile.racing_line_length_m,
        predicted_racing_lap_time=profile.predicted_racing_lap_time,
        driving_line_coords=driving_line_coords,
        driving_line_lengths_m=tuple(sorted(profile.driving_line_lengths_m.items())),
        predicted_line_lap_times=tuple(sorted(profile.predicted_line_lap_times.items())),
        grid_slots=build_grid_slots(
            grid_driver_ids,
            profile.coordinate_frame.track_length_m,
            start_sequence_enabled=start_sequence_enabled,
            track_profile=profile,
        ),
        pit_lane_coords=pit_lane_coords,
        pit_exit_lane_coords=pit_exit_lane_coords,
        pit_wall_coords=_render_coords(_coord_pair(point) for point in circuit.pit_wall_coords),
        start_finish_index=int(circuit.start_finish_index),
        pit_box_offset=float(pit_lane.box_offset if pit_lane else 11.0),
        pit_lane_width_m=float(pit_lane.lane_width_m if pit_lane else 4.0),
        pit_speed_limit_kph=float(pit_lane.speed_limit_kph if pit_lane else 80.0),
        pit_entry_progress=float(pit_lane.entry_progress if pit_lane and pit_lane.entry_progress is not None else 0.88),
        pit_exit_progress=float(pit_lane.exit_progress if pit_lane and pit_lane.exit_progress is not None else 0.10),
        pit_side_entry_progress=float(pit_lane.side_entry_progress if pit_lane else 0.02),
        pit_speed_limit_start=float(pit_lane.speed_limit_start if pit_lane else 0.12),
        pit_box_progress=float(pit_lane.box_progress if pit_lane else 0.50),
        pit_speed_limit_end=float(pit_lane.speed_limit_end if pit_lane else 0.88),
        pit_side_rejoin_progress=float(pit_lane.side_rejoin_progress if pit_lane else 0.94),
        _compiled_profile=profile,
    )
