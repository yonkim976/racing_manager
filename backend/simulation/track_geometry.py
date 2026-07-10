"""Geometry helpers for circuit validation and segment lookup."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from math import acos, ceil, hypot, isclose

from models.schemas import Circuit, TrackSegment, TrackSegmentType

Point = tuple[float, float]

MIN_TRACK_POINTS = 4
MAX_SEGMENT_LENGTH = 220.0
MIN_SEGMENT_LENGTH = 8.0
MAX_START_FINISH_CLOSURE = 2.0
MIN_DRS_LENGTH = 0.03
MAX_DRS_LENGTH = 0.35
MIN_COMPILED_TRACK_POINTS = 50
MAX_CURVATURE_WARNING = 0.075
MIN_TRACK_X = -10.0
MAX_TRACK_X = 820.0
MIN_TRACK_Y = -10.0
MAX_TRACK_Y = 620.0


@dataclass(frozen=True)
class TrackBounds:
    min_x: float
    min_y: float
    max_x: float
    max_y: float


def _coord_to_point(coord) -> Point:
    """Return an ``(x, y)`` point from legacy arrays or metadata objects."""
    if isinstance(coord, dict):
        return float(coord["x"]), float(coord["y"])
    if hasattr(coord, "x") and hasattr(coord, "y"):
        return float(coord.x), float(coord.y)
    return float(coord[0]), float(coord[1])


def normalize_points(coords) -> list[Point]:
    return [_coord_to_point(coord) for coord in coords]


def distance(a: Point, b: Point) -> float:
    return hypot(b[0] - a[0], b[1] - a[1])


def segment_lengths(coords: list[list[float]]) -> list[float]:
    points = normalize_points(coords)
    return [distance(points[i], points[i + 1]) for i in range(len(points) - 1)]


def polyline_length(points, closed: bool = True) -> float:
    """Return path length for point arrays or track point objects."""
    normalized = normalize_points(points)
    if len(normalized) < 2:
        return 0.0

    total = sum(distance(normalized[i], normalized[i + 1]) for i in range(len(normalized) - 1))
    if closed and distance(normalized[0], normalized[-1]) > 1e-6:
        total += distance(normalized[-1], normalized[0])
    return total


def track_length(coords: list[list[float]]) -> float:
    return sum(segment_lengths(coords))


def track_bounds(coords: list[list[float]]) -> TrackBounds:
    points = normalize_points(coords)
    return TrackBounds(
        min_x=min(point[0] for point in points),
        min_y=min(point[1] for point in points),
        max_x=max(point[0] for point in points),
        max_y=max(point[1] for point in points),
    )


def _prepared_points(points, closed: bool) -> list[Point]:
    prepared = normalize_points(points)
    if closed and len(prepared) > 1 and distance(prepared[0], prepared[-1]) > 1e-6:
        prepared.append(prepared[0])
    return prepared


def _cumulative_lengths(points: list[Point]) -> list[float]:
    cumulative = [0.0]
    for index in range(len(points) - 1):
        cumulative.append(cumulative[-1] + distance(points[index], points[index + 1]))
    return cumulative


def _lerp(a: Point, b: Point, t: float) -> Point:
    return a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t


def _point_at_distance(points: list[Point], cumulative: list[float], target: float) -> Point:
    if not points:
        return 0.0, 0.0
    if len(points) == 1 or target <= 0:
        return points[0]
    if target >= cumulative[-1]:
        return points[-1]

    index = max(0, bisect_right(cumulative, target) - 1)
    index = min(index, len(points) - 2)
    segment_length = cumulative[index + 1] - cumulative[index]
    if segment_length <= 0:
        return points[index]
    return _lerp(points[index], points[index + 1], (target - cumulative[index]) / segment_length)


def interpolate_polyline_by_progress(points, progress: float) -> Point:
    """Interpolate a point on a closed path by normalized progress."""
    prepared = _prepared_points(points, closed=True)
    if len(prepared) < 2:
        return prepared[0] if prepared else (0.0, 0.0)

    cumulative = _cumulative_lengths(prepared)
    total = cumulative[-1]
    if total <= 0:
        return prepared[0]
    target = (progress % 1.0) * total
    return _point_at_distance(prepared, cumulative, target)


def nearest_point_index(points, x: float, y: float) -> int:
    """Return index of the closest point in a path."""
    normalized = normalize_points(points)
    if not normalized:
        return 0
    target = (float(x), float(y))
    return min(
        range(len(normalized)),
        key=lambda index: distance(normalized[index], target),
    )


def calculate_tangent(points, index: int) -> Point:
    """Return a unit tangent at a path index."""
    normalized = normalize_points(points)
    if len(normalized) < 2:
        return 1.0, 0.0
    count = len(normalized)
    index = max(0, min(count - 1, index))
    previous_point = normalized[index - 1] if index > 0 else normalized[0]
    next_point = normalized[index + 1] if index < count - 1 else normalized[-1]
    if distance(previous_point, next_point) <= 1e-9 and count > 2:
        previous_point = normalized[(index - 1) % count]
        next_point = normalized[(index + 1) % count]
    dx = next_point[0] - previous_point[0]
    dy = next_point[1] - previous_point[1]
    length = max(1e-9, hypot(dx, dy))
    return dx / length, dy / length


def calculate_normal(points, index: int) -> Point:
    tangent = calculate_tangent(points, index)
    return -tangent[1], tangent[0]


def calculate_curvature(points, index: int) -> float:
    """Approximate curvature from neighboring segment angle change."""
    normalized = normalize_points(points)
    if len(normalized) < 3:
        return 0.0

    count = len(normalized)
    closed = distance(normalized[0], normalized[-1]) <= 1e-6
    if closed and index == count - 1:
        index = 0

    previous_index = (index - 1) % (count - 1 if closed else count)
    next_index = (index + 1) % (count - 1 if closed else count)
    if not closed:
        if index <= 0 or index >= count - 1:
            return 0.0
        previous_index = index - 1
        next_index = index + 1

    previous_point = normalized[previous_index]
    point = normalized[index]
    next_point = normalized[next_index]
    in_length = distance(previous_point, point)
    out_length = distance(point, next_point)
    if in_length <= 1e-9 or out_length <= 1e-9:
        return 0.0

    incoming = ((point[0] - previous_point[0]) / in_length, (point[1] - previous_point[1]) / in_length)
    outgoing = ((next_point[0] - point[0]) / out_length, (next_point[1] - point[1]) / out_length)
    dot = max(-1.0, min(1.0, incoming[0] * outgoing[0] + incoming[1] * outgoing[1]))
    return acos(dot) / max(1e-9, 0.5 * (in_length + out_length))


def _orientation(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    return (
        _orientation(a, b, c) * _orientation(a, b, d) < 0
        and _orientation(c, d, a) * _orientation(c, d, b) < 0
    )


def self_intersections(coords: list[list[float]]) -> list[tuple[int, int]]:
    points = normalize_points(coords)
    intersections: list[tuple[int, int]] = []

    for i in range(len(points) - 1):
        for j in range(i + 1, len(points) - 1):
            if abs(i - j) <= 1:
                continue
            if i == 0 and j == len(points) - 2:
                continue
            if segments_intersect(points[i], points[i + 1], points[j], points[j + 1]):
                intersections.append((i, j))

    return intersections


def detect_self_intersection(points) -> list[tuple[int, int]]:
    return self_intersections(points)


def resample_polyline_by_distance(points, spacing: float, closed: bool = True) -> list[list[float]]:
    """Resample a path at nearly uniform distance intervals."""
    prepared = _prepared_points(points, closed=closed)
    if len(prepared) < 2:
        return [[round(x, 3), round(y, 3)] for x, y in prepared]

    cumulative = _cumulative_lengths(prepared)
    total = cumulative[-1]
    if total <= 0:
        return [[round(prepared[0][0], 3), round(prepared[0][1], 3)]]

    steps = max(1, ceil(total / max(0.001, spacing)))
    final_index = steps - 1 if closed else steps
    sampled = [
        _point_at_distance(prepared, cumulative, total * index / steps)
        for index in range(final_index + 1)
    ]
    if closed:
        sampled.append(sampled[0])

    return [[round(x, 3), round(y, 3)] for x, y in sampled]


def progress_range_length(start: float, end: float) -> float:
    if start <= end:
        return end - start
    return (1.0 - start) + end


def segment_at_progress(circuit: Circuit, progress: float) -> TrackSegment | None:
    """Return the configured driving segment for a normalized progress value."""
    if not circuit.segments:
        return None

    normalized = progress % 1.0
    for segment in circuit.segments:
        if segment.start <= normalized < segment.end:
            return segment
        if isclose(segment.end, 1.0) and isclose(normalized, 1.0):
            return segment
    return circuit.segments[-1] if isclose(normalized, 1.0) else None


def validate_circuit_geometry(circuit: Circuit) -> list[str]:
    """Return human-readable validation errors for a circuit definition."""
    return validate_circuit_geometry_detailed(circuit)["errors"]


def validate_circuit_geometry_detailed(circuit: Circuit) -> dict[str, list[str]]:
    """Return validation errors and non-blocking warnings for a circuit."""
    errors: list[str] = []
    warnings: list[str] = []
    points = normalize_points(circuit.track_coords)

    if len(points) < MIN_TRACK_POINTS:
        return {
            "errors": [f"{circuit.name}: track_coords must contain at least {MIN_TRACK_POINTS} points"],
            "warnings": warnings,
        }

    editor = getattr(circuit, "editor", None)
    centerline_points = getattr(editor, "centerlineControlPoints", []) if editor else []
    if centerline_points and len(centerline_points) < 4:
        errors.append(f"{circuit.name}: editor centerline requires at least 4 control points")

    if len(points) < MIN_COMPILED_TRACK_POINTS:
        errors.append(
            f"{circuit.name}: compiled track_coords must contain at least {MIN_COMPILED_TRACK_POINTS} points"
        )

    closure = distance(points[0], points[-1])
    if closure > MAX_START_FINISH_CLOSURE:
        errors.append(f"{circuit.name}: track is not closed, first/last distance={closure:.1f}")

    if not 0 <= circuit.start_finish_index < len(points) - 1:
        errors.append(f"{circuit.name}: start_finish_index is outside track segment range")

    intersections = self_intersections(circuit.track_coords)
    if intersections:
        errors.append(f"{circuit.name}: track self-intersections={intersections}")

    curvature_hotspots = [
        (index, calculate_curvature(points, index))
        for index in range(1, max(1, len(points) - 1))
    ]
    sharp_hotspots = [
        (index, curvature)
        for index, curvature in curvature_hotspots
        if curvature > MAX_CURVATURE_WARNING
    ]
    if sharp_hotspots:
        index, curvature = max(sharp_hotspots, key=lambda item: item[1])
        warnings.append(f"{circuit.name}: sharp curvature near point {index} ({curvature:.3f})")

    lengths = segment_lengths(circuit.track_coords)
    for index, length in enumerate(lengths):
        if length < MIN_SEGMENT_LENGTH:
            errors.append(f"{circuit.name}: segment {index} is too short ({length:.1f})")
        if length > MAX_SEGMENT_LENGTH:
            errors.append(f"{circuit.name}: segment {index} is too long ({length:.1f})")

    bounds = track_bounds(circuit.track_coords)
    if (
        bounds.min_x < MIN_TRACK_X
        or bounds.max_x > MAX_TRACK_X
        or bounds.min_y < MIN_TRACK_Y
        or bounds.max_y > MAX_TRACK_Y
    ):
        errors.append(f"{circuit.name}: track bounds are outside the supported canvas range")

    for index, landmark in enumerate(circuit.landmarks):
        if landmark.progress is not None and not 0.0 <= landmark.progress <= 1.0:
            errors.append(
                f"{circuit.name}: landmark {index} ({landmark.label}) has invalid progress"
            )
        if landmark.track_index >= len(points) - 1:
            errors.append(
                f"{circuit.name}: landmark {index} ({landmark.label}) has invalid track_index"
            )

    if circuit.pit_lane is not None:
        if circuit.pit_lane.entry_progress is not None:
            entry_index = int((circuit.pit_lane.entry_progress % 1.0) * max(1, len(points) - 1))
            if not 0 <= entry_index < len(points) - 1:
                errors.append(f"{circuit.name}: pit entry index is outside track range")
        if circuit.pit_lane.exit_progress is not None:
            exit_index = int((circuit.pit_lane.exit_progress % 1.0) * max(1, len(points) - 1))
            if not 0 <= exit_index < len(points) - 1:
                errors.append(f"{circuit.name}: pit exit index is outside track range")

    if circuit.pit_lane_coords:
        pit_points = normalize_points(circuit.pit_lane_coords)
        if len(pit_points) < 2:
            errors.append(f"{circuit.name}: pit lane must contain at least 2 points")
        elif distance(pit_points[0], pit_points[-1]) <= 1.0:
            warnings.append(f"{circuit.name}: pit lane appears closed; expected an open path")

    for index, zone in enumerate(circuit.drs_zones):
        length = progress_range_length(zone.start, zone.end)
        if length < MIN_DRS_LENGTH:
            errors.append(f"{circuit.name}: DRS zone {index} is too short ({length:.3f})")
        if length > MAX_DRS_LENGTH:
            errors.append(f"{circuit.name}: DRS zone {index} is too long ({length:.3f})")
        if not _drs_zone_is_inside_straight_segment(circuit, zone.start, zone.end):
            warnings.append(f"{circuit.name}: DRS zone {index} should be inside a straight segment")

    errors.extend(_validate_track_segments(circuit))
    return {"errors": errors, "warnings": warnings}


def _drs_zone_is_inside_straight_segment(circuit: Circuit, start: float, end: float) -> bool:
    if not circuit.segments:
        return False

    normalized_start = start % 1.0
    normalized_end = end % 1.0
    if normalized_start <= normalized_end:
        return _progress_range_is_inside_straight_segment(
            circuit,
            normalized_start,
            normalized_end,
        )

    return _progress_range_is_inside_straight_segment(
        circuit,
        normalized_start,
        1.0,
    ) and _progress_range_is_inside_straight_segment(
        circuit,
        0.0,
        normalized_end,
    )


def _progress_range_is_inside_straight_segment(circuit: Circuit, start: float, end: float) -> bool:
    if start >= end:
        return False

    tolerance = 0.001
    for segment in circuit.segments:
        if segment.type != TrackSegmentType.STRAIGHT:
            continue
        if start + tolerance >= segment.start and end - tolerance <= segment.end:
            return True
    return False


def _validate_track_segments(circuit: Circuit) -> list[str]:
    errors: list[str] = []
    if not circuit.segments:
        return [f"{circuit.name}: segments are required for driving model support"]

    if not isclose(circuit.segments[0].start, 0.0, abs_tol=0.001):
        errors.append(f"{circuit.name}: first segment must start at 0.0")
    if not isclose(circuit.segments[-1].end, 1.0, abs_tol=0.001):
        errors.append(f"{circuit.name}: last segment must end at 1.0")

    allowed_types = {segment_type.value for segment_type in TrackSegmentType}
    for index, segment in enumerate(circuit.segments):
        if segment.type.value not in allowed_types:
            errors.append(f"{circuit.name}: segment {index} has unsupported type {segment.type}")
        if segment.start >= segment.end:
            errors.append(f"{circuit.name}: segment {index} start must be less than end")
        if progress_range_length(segment.start, segment.end) < 0.03:
            errors.append(f"{circuit.name}: segment {index} is too short")
        if index > 0:
            previous = circuit.segments[index - 1]
            if not isclose(previous.end, segment.start, abs_tol=0.001):
                errors.append(
                    f"{circuit.name}: segments {index - 1} and {index} are not contiguous"
                )

    return errors
