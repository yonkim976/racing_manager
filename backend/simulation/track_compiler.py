"""Compile editable circuit layout segments into sampled track coordinates."""

from __future__ import annotations

from bisect import bisect_right
from math import acos, ceil, hypot, isclose

from models.schemas import Circuit, PitLaneConfig, TrackBoundaries, TrackLayoutSegment, TrackLayoutSegmentType, TrackPoint
from simulation.geo_projection import compile_geo_circuit, compile_metric_circuit
from simulation.track_geometry import (
    calculate_curvature,
    calculate_normal,
    calculate_tangent,
    normalize_points,
    resample_polyline_by_distance,
)

Point = tuple[float, float]

TRACK_POINT_SPACING = 16.0
PIT_LANE_POINT_SPACING = 14.0
MIN_RAW_SEGMENT_SAMPLES = 10


def _point(values) -> Point:
    if isinstance(values, dict):
        return float(values["x"]), float(values["y"])
    if hasattr(values, "x") and hasattr(values, "y"):
        return float(values.x), float(values.y)
    return float(values[0]), float(values[1])


def _distance(a: Point, b: Point) -> float:
    return hypot(b[0] - a[0], b[1] - a[1])


def _lerp(a: Point, b: Point, t: float) -> Point:
    return a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t


def _cubic_bezier(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
    inverse = 1.0 - t
    return (
        inverse**3 * p0[0]
        + 3 * inverse**2 * t * p1[0]
        + 3 * inverse * t**2 * p2[0]
        + t**3 * p3[0],
        inverse**3 * p0[1]
        + 3 * inverse**2 * t * p1[1]
        + 3 * inverse * t**2 * p2[1]
        + t**3 * p3[1],
    )


def _catmull_rom(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
    t2 = t * t
    t3 = t2 * t
    return (
        0.5
        * (
            (2 * p1[0])
            + (-p0[0] + p2[0]) * t
            + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
            + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3
        ),
        0.5
        * (
            (2 * p1[1])
            + (-p0[1] + p2[1]) * t
            + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
            + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3
        ),
    )


def _editor_control_points(points) -> list[Point]:
    return [_point(point) for point in points or []]


def _segment_sample_count(segment: TrackLayoutSegment) -> int:
    if segment.samples is not None:
        return segment.samples

    start = _point(segment.start)
    end = _point(segment.end)
    base_distance = _distance(start, end)
    if segment.type == TrackLayoutSegmentType.BEZIER and segment.cp1 and segment.cp2:
        base_distance += _distance(start, _point(segment.cp1))
        base_distance += _distance(_point(segment.cp1), _point(segment.cp2))
        base_distance += _distance(_point(segment.cp2), end)

    return max(MIN_RAW_SEGMENT_SAMPLES, ceil(base_distance / 14.0))


def _sample_segment(segment: TrackLayoutSegment) -> list[Point]:
    start = _point(segment.start)
    end = _point(segment.end)
    samples = _segment_sample_count(segment)

    if segment.type == TrackLayoutSegmentType.STRAIGHT:
        return [_lerp(start, end, i / samples) for i in range(samples + 1)]

    if segment.type == TrackLayoutSegmentType.BEZIER:
        if not segment.cp1 or not segment.cp2:
            raise ValueError("Bezier layout segment requires cp1 and cp2")
        cp1 = _point(segment.cp1)
        cp2 = _point(segment.cp2)
        return [_cubic_bezier(start, cp1, cp2, end, i / samples) for i in range(samples + 1)]

    raise ValueError(f"Unsupported layout segment type: {segment.type}")


def _raw_points_from_segments(segments: list[TrackLayoutSegment]) -> list[Point]:
    raw_points: list[Point] = []
    for segment in segments:
        samples = _sample_segment(segment)
        if raw_points and _distance(raw_points[-1], samples[0]) < 0.001:
            raw_points.extend(samples[1:])
        else:
            raw_points.extend(samples)
    return raw_points


def _cumulative_lengths(points: list[Point]) -> list[float]:
    cumulative = [0.0]
    for index in range(len(points) - 1):
        cumulative.append(cumulative[-1] + _distance(points[index], points[index + 1]))
    return cumulative


def _point_at_distance(points: list[Point], cumulative: list[float], target: float) -> Point:
    if target <= 0:
        return points[0]
    if target >= cumulative[-1]:
        return points[-1]

    index = max(0, bisect_right(cumulative, target) - 1)
    segment_length = cumulative[index + 1] - cumulative[index]
    if segment_length <= 0:
        return points[index]

    t = (target - cumulative[index]) / segment_length
    return _lerp(points[index], points[index + 1], t)


def _point_and_tangent_at_distance(
    points: list[Point],
    cumulative: list[float],
    target: float,
) -> tuple[Point, Point]:
    if target <= 0:
        index = 0
    elif target >= cumulative[-1]:
        index = len(points) - 2
    else:
        index = max(0, bisect_right(cumulative, target) - 1)

    index = min(len(points) - 2, index)
    segment_length = cumulative[index + 1] - cumulative[index]
    if segment_length <= 0:
        return points[index], (1.0, 0.0)

    t = (target - cumulative[index]) / segment_length
    point = _lerp(points[index], points[index + 1], min(1.0, max(0.0, t)))
    return point, (
        (points[index + 1][0] - points[index][0]) / segment_length,
        (points[index + 1][1] - points[index][1]) / segment_length,
    )


def _point_and_tangent_at_progress(coords: list[list[float]], progress: float) -> tuple[Point, Point]:
    points = [_point(coord) for coord in coords]
    cumulative = _cumulative_lengths(points)
    total_length = cumulative[-1]
    target = ((progress % 1.0) * total_length) if total_length > 0 else 0.0
    return _point_and_tangent_at_distance(points, cumulative, target)


def _offset_from_tangent(point: Point, tangent: Point, offset: float) -> Point:
    return point[0] - tangent[1] * offset, point[1] + tangent[0] * offset


def _progress_delta(start: float, end: float) -> float:
    delta = (end - start) % 1.0
    if isclose(delta, 0.0, abs_tol=1e-9):
        return 0.0
    return delta


def _resample_points(points: list[Point], spacing: float, closed: bool) -> list[list[float]]:
    if len(points) < 2:
        return [[x, y] for x, y in points]

    prepared = points[:]
    if closed and not _same_point(prepared[0], prepared[-1]):
        prepared.append(prepared[0])

    cumulative = _cumulative_lengths(prepared)
    total_length = cumulative[-1]
    if total_length <= 0:
        return [[prepared[0][0], prepared[0][1]]]

    steps = max(1, ceil(total_length / spacing))
    if closed:
        sampled = [
            _point_at_distance(prepared, cumulative, total_length * index / steps)
            for index in range(steps)
        ]
        sampled.append(sampled[0])
    else:
        sampled = [
            _point_at_distance(prepared, cumulative, total_length * index / steps)
            for index in range(steps + 1)
        ]

    return [[round(x, 3), round(y, 3)] for x, y in sampled]


def _same_point(a: Point, b: Point) -> bool:
    return isclose(a[0], b[0], abs_tol=0.001) and isclose(a[1], b[1], abs_tol=0.001)


def compile_layout_segments(
    segments: list[TrackLayoutSegment],
    *,
    closed: bool = True,
    spacing: float = TRACK_POINT_SPACING,
) -> list[list[float]]:
    """Return nearly even-distance sampled coordinates for layout segments."""
    if not segments:
        return []

    raw_points = _raw_points_from_segments(segments)
    return _resample_points(raw_points, spacing=spacing, closed=closed)


def sample_catmull_rom_closed(points, spacing: float = TRACK_POINT_SPACING) -> list[list[float]]:
    """Sample closed Catmull-Rom control points into legacy ``[x, y]`` coords."""
    controls = _editor_control_points(points)
    if len(controls) < 4:
        return []

    raw: list[Point] = []
    count = len(controls)
    for index in range(count):
        p0 = controls[(index - 1) % count]
        p1 = controls[index]
        p2 = controls[(index + 1) % count]
        p3 = controls[(index + 2) % count]
        segment_distance = _distance(p1, p2)
        samples = max(MIN_RAW_SEGMENT_SAMPLES, ceil(segment_distance / 10.0))
        for step in range(samples):
            raw.append(_catmull_rom(p0, p1, p2, p3, step / samples))

    raw.append(raw[0])
    return resample_polyline_by_distance(raw, spacing=spacing, closed=True)


def sample_catmull_rom_open(points, spacing: float = PIT_LANE_POINT_SPACING) -> list[list[float]]:
    """Sample open Catmull-Rom control points into legacy ``[x, y]`` coords."""
    controls = _editor_control_points(points)
    if len(controls) < 2:
        return []
    if len(controls) == 2:
        return resample_polyline_by_distance(controls, spacing=spacing, closed=False)

    raw: list[Point] = []
    count = len(controls)
    for index in range(count - 1):
        p0 = controls[max(0, index - 1)]
        p1 = controls[index]
        p2 = controls[index + 1]
        p3 = controls[min(count - 1, index + 2)]
        segment_distance = _distance(p1, p2)
        samples = max(MIN_RAW_SEGMENT_SAMPLES, ceil(segment_distance / 10.0))
        for step in range(samples):
            raw.append(_catmull_rom(p0, p1, p2, p3, step / samples))
    raw.append(controls[-1])
    return resample_polyline_by_distance(raw, spacing=spacing, closed=False)


def calculate_track_metrics(track_coords) -> list[TrackPoint]:
    """Build per-point centerline metadata while preserving legacy coords."""
    points = normalize_points(track_coords)
    if not points:
        return []

    cumulative = [0.0]
    for index in range(len(points) - 1):
        cumulative.append(cumulative[-1] + _distance(points[index], points[index + 1]))
    total = cumulative[-1] or 1.0

    metrics: list[TrackPoint] = []
    for index, point in enumerate(points):
        tangent = calculate_tangent(points, index)
        normal = calculate_normal(points, index)
        metrics.append(
            TrackPoint(
                x=round(point[0], 3),
                y=round(point[1], 3),
                s=round(cumulative[index], 3),
                progress=round(min(1.0, cumulative[index] / total), 6),
                tangent_x=round(tangent[0], 6),
                tangent_y=round(tangent[1], 6),
                normal_x=round(normal[0], 6),
                normal_y=round(normal[1], 6),
                curvature=round(calculate_curvature(points, index), 6),
            )
        )
    return metrics


def map_progress_to_track_index(progress: float, track_coords) -> int:
    return _progress_to_track_index(track_coords, progress)


def map_landmarks_to_indices(circuit: Circuit) -> Circuit:
    """Resolve landmark progress values against compiled coordinates."""
    if not circuit.track_coords:
        return circuit
    for landmark in circuit.landmarks:
        if landmark.progress is not None:
            landmark.track_index = map_progress_to_track_index(landmark.progress, circuit.track_coords)
    return circuit


def generate_track_boundaries(track_coords, track_width: float) -> TrackBoundaries:
    """Generate simple left/right boundary preview coordinates."""
    points = normalize_points(track_coords)
    if not points:
        return TrackBoundaries()

    half_width = max(0.0, track_width) / 2.0
    left: list[list[float]] = []
    right: list[list[float]] = []
    for index, point in enumerate(points):
        normal = calculate_normal(points, index)
        left.append([
            round(point[0] + normal[0] * half_width, 3),
            round(point[1] + normal[1] * half_width, 3),
        ])
        right.append([
            round(point[0] - normal[0] * half_width, 3),
            round(point[1] - normal[1] * half_width, 3),
        ])
    return TrackBoundaries(left=left, right=right)


def compile_editor_centerline(circuit: Circuit) -> list[list[float]]:
    """Compile editor centerline control points when present."""
    editor = circuit.editor
    if not editor or len(editor.centerlineControlPoints) < 4:
        return []
    return sample_catmull_rom_closed(
        editor.centerlineControlPoints,
        spacing=editor.sampleSpacing,
    )


def _progress_to_track_index(coords: list[list[float]], progress: float) -> int:
    if len(coords) < 2:
        return 0

    points = [_point(coord) for coord in coords]
    cumulative = _cumulative_lengths(points)
    total_length = cumulative[-1]
    target = (progress % 1.0) * total_length
    return min(len(coords) - 2, max(0, bisect_right(cumulative, target) - 1))


def _compile_offset_pit_lane(
    coords: list[list[float]],
    config: PitLaneConfig,
) -> tuple[list[list[float]], list[list[float]]]:
    if (
        len(coords) < 2
        or config.entry_progress is None
        or config.exit_progress is None
    ):
        return [], []

    entry = config.entry_progress % 1.0
    exit_ = config.exit_progress % 1.0
    lap_delta = _progress_delta(entry, exit_)
    if lap_delta <= 0:
        return [], []

    entry_blend = min(config.entry_blend, lap_delta / 3)
    exit_blend = min(config.exit_blend, lap_delta / 3)
    offset_delta = max(0.0, lap_delta - entry_blend - exit_blend)

    lane_raw: list[Point] = [_point_and_tangent_at_progress(coords, entry)[0]]
    wall_raw: list[Point] = []
    for index in range(config.samples):
        t = index / max(1, config.samples - 1)
        progress = (entry + entry_blend + offset_delta * t) % 1.0
        point, tangent = _point_and_tangent_at_progress(coords, progress)
        lane_raw.append(_offset_from_tangent(point, tangent, config.lane_offset))
        if config.wall_offset is not None:
            wall_raw.append(_offset_from_tangent(point, tangent, config.wall_offset))
    lane_raw.append(_point_and_tangent_at_progress(coords, exit_)[0])

    return (
        _resample_points(lane_raw, spacing=PIT_LANE_POINT_SPACING, closed=False),
        _resample_points(wall_raw, spacing=PIT_LANE_POINT_SPACING, closed=False) if wall_raw else [],
    )


def _offset_open_path(coords: list[list[float]], offset: float) -> list[list[float]]:
    if len(coords) < 2:
        return []

    points = [_point(coord) for coord in coords]
    offset_points: list[Point] = []
    for index, point in enumerate(points):
        previous_point = points[max(0, index - 1)]
        next_point = points[min(len(points) - 1, index + 1)]
        dx = next_point[0] - previous_point[0]
        dy = next_point[1] - previous_point[1]
        length = max(0.001, hypot(dx, dy))
        offset_points.append((
            point[0] + (-dy / length) * offset,
            point[1] + (dx / length) * offset,
        ))

    return [[round(x, 3), round(y, 3)] for x, y in offset_points]


def _compile_explicit_pit_wall(
    pit_lane_coords: list[list[float]],
    config: PitLaneConfig | None,
) -> list[list[float]]:
    if not config or config.wall_offset is None or len(pit_lane_coords) < 2:
        return []

    offset = config.wall_offset - config.lane_offset
    if isclose(offset, 0.0, abs_tol=0.001):
        offset = -7.0
    return _offset_open_path(pit_lane_coords, offset)


def _anchor_explicit_pit_lane(
    pit_lane_coords: list[list[float]],
    track_coords: list[list[float]],
    config: PitLaneConfig | None,
) -> list[list[float]]:
    if (
        len(pit_lane_coords) < 2
        or len(track_coords) < 2
        or config is None
        or config.entry_progress is None
        or config.exit_progress is None
    ):
        return pit_lane_coords

    entry = _point_and_tangent_at_progress(track_coords, config.entry_progress)[0]
    exit_ = _point_and_tangent_at_progress(track_coords, config.exit_progress)[0]
    raw_points = [entry, *[_point(coord) for coord in pit_lane_coords], exit_]
    deduped: list[Point] = []
    for point in raw_points:
        if not deduped or not _same_point(deduped[-1], point):
            deduped.append(point)

    return _resample_points(deduped, spacing=PIT_LANE_POINT_SPACING, closed=False)


def _nearest_progress_on_path(coords: list[list[float]], x: float, y: float) -> float:
    """Project a point onto the nearest track segment and return lap progress."""
    if len(coords) < 2:
        return 0.0

    points = [_point(coord) for coord in coords]
    cumulative = _cumulative_lengths(points)
    total_length = cumulative[-1]
    if total_length <= 0:
        return 0.0

    best_distance_squared = float("inf")
    best_distance_along = 0.0
    for index, (start, end) in enumerate(zip(points, points[1:])):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 0:
            continue

        t = min(1.0, max(0.0, ((x - start[0]) * dx + (y - start[1]) * dy) / length_squared))
        projected_x = start[0] + dx * t
        projected_y = start[1] + dy * t
        distance_squared = (x - projected_x) ** 2 + (y - projected_y) ** 2
        if distance_squared < best_distance_squared:
            best_distance_squared = distance_squared
            segment_length = cumulative[index + 1] - cumulative[index]
            best_distance_along = cumulative[index] + segment_length * t

    return min(1.0, max(0.0, best_distance_along / total_length))


def compile_pit_lane(circuit: Circuit) -> tuple[list[list[float]], list[list[float]]]:
    """Compile pit lane coordinates from editor, explicit segments, or offsets."""
    if (circuit.geo or circuit.metric) and circuit.pit_lane_coords:
        pit_lane_coords = [
            [round(x, 3), round(y, 3)]
            for x, y in normalize_points(circuit.pit_lane_coords)
        ]
        pit_lane_coords = _anchor_explicit_pit_lane(
            pit_lane_coords,
            circuit.track_coords,
            circuit.pit_lane,
        )
        return pit_lane_coords, _compile_explicit_pit_wall(pit_lane_coords, circuit.pit_lane)

    editor = circuit.editor
    if editor and len(editor.pitLaneControlPoints) >= 2:
        pit_lane_coords = sample_catmull_rom_open(
            editor.pitLaneControlPoints,
            spacing=max(2.0, editor.sampleSpacing * 0.875),
        )
        if circuit.track_coords and pit_lane_coords:
            pit_config = circuit.pit_lane or PitLaneConfig()
            pit_config.entry_progress = _nearest_progress_on_path(
                circuit.track_coords,
                pit_lane_coords[0][0],
                pit_lane_coords[0][1],
            )
            pit_config.exit_progress = _nearest_progress_on_path(
                circuit.track_coords,
                pit_lane_coords[-1][0],
                pit_lane_coords[-1][1],
            )
            circuit.pit_lane = pit_config
            pit_lane_coords = _anchor_explicit_pit_lane(
                pit_lane_coords,
                circuit.track_coords,
                circuit.pit_lane,
            )
        return pit_lane_coords, _compile_explicit_pit_wall(pit_lane_coords, circuit.pit_lane)

    if circuit.pit_lane_segments:
        pit_lane_coords = compile_layout_segments(
            circuit.pit_lane_segments,
            closed=False,
            spacing=PIT_LANE_POINT_SPACING,
        )
        pit_lane_coords = _anchor_explicit_pit_lane(
            pit_lane_coords,
            circuit.track_coords,
            circuit.pit_lane,
        )
        return pit_lane_coords, _compile_explicit_pit_wall(pit_lane_coords, circuit.pit_lane)

    if circuit.pit_lane_coords:
        pit_lane_coords = [
            [round(x, 3), round(y, 3)]
            for x, y in normalize_points(circuit.pit_lane_coords)
        ]
        pit_lane_coords = _anchor_explicit_pit_lane(
            pit_lane_coords,
            circuit.track_coords,
            circuit.pit_lane,
        )
        return pit_lane_coords, _compile_explicit_pit_wall(pit_lane_coords, circuit.pit_lane)

    if circuit.pit_lane:
        return _compile_offset_pit_lane(
            circuit.track_coords,
            circuit.pit_lane,
        )

    return [], []


def compile_circuit_layout(circuit: Circuit) -> Circuit:
    """Compile source layout data while preserving the public Circuit contract."""
    compiled = circuit.model_copy(deep=True)

    source_result = compile_metric_circuit(compiled) or compile_geo_circuit(compiled)
    if source_result:
        compiled.track_coords = source_result.track_coords
        compiled.start_finish_index = 0
        if source_result.pit_lane_coords:
            compiled.pit_lane_coords = source_result.pit_lane_coords
            pit_config = compiled.pit_lane or PitLaneConfig()
            pit_config.entry_progress = _nearest_progress_on_path(
                compiled.track_coords,
                source_result.pit_lane_coords[0][0],
                source_result.pit_lane_coords[0][1],
            )
            pit_config.exit_progress = _nearest_progress_on_path(
                compiled.track_coords,
                source_result.pit_lane_coords[-1][0],
                source_result.pit_lane_coords[-1][1],
            )
            compiled.pit_lane = pit_config
    else:
        editor_track_coords = compile_editor_centerline(compiled)
        if editor_track_coords:
            compiled.track_coords = editor_track_coords
        elif compiled.layout_segments:
            compiled.track_coords = compile_layout_segments(
                compiled.layout_segments,
                closed=True,
                spacing=compiled.editor.sampleSpacing if compiled.editor else TRACK_POINT_SPACING,
            )
            compiled.start_finish_index = 0
        elif compiled.track_coords:
            compiled.track_coords = [
                [round(x, 3), round(y, 3)]
                for x, y in normalize_points(compiled.track_coords)
            ]

    if compiled.track_coords and compiled.start_finish_index >= len(compiled.track_coords) - 1:
        compiled.start_finish_index = max(0, len(compiled.track_coords) - 2)

    pit_lane_coords, pit_wall_coords = compile_pit_lane(compiled)
    if pit_lane_coords:
        compiled.pit_lane_coords = pit_lane_coords
    elif compiled.pit_lane_coords:
        compiled.pit_lane_coords = [
            [round(x, 3), round(y, 3)]
            for x, y in normalize_points(compiled.pit_lane_coords)
        ]
    if pit_wall_coords:
        compiled.pit_wall_coords = pit_wall_coords

    map_landmarks_to_indices(compiled)
    compiled.track_points = calculate_track_metrics(compiled.track_coords)
    if source_result and source_result.track_boundaries:
        compiled.track_boundaries = source_result.track_boundaries
    elif compiled.editor and compiled.track_coords:
        compiled.track_boundaries = generate_track_boundaries(
            compiled.track_coords,
            compiled.editor.trackWidth,
        )
    elif (compiled.geo or compiled.metric) and compiled.track_coords:
        compiled.track_boundaries = generate_track_boundaries(
            compiled.track_coords,
            12.0,
        )

    return compiled
