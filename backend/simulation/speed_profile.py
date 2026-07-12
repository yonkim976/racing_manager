"""Curvature-based target speed profiles for circuit driving."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from math import acos, hypot, sqrt

from models.schemas import Circuit

Point = tuple[float, float]

PROFILE_MIN_SPEED_MPS = 28.0
PROFILE_MAX_SPEED_MPS = 96.0
PROFILE_LATERAL_ACCEL_MPS2 = 34.0
CURVATURE_EPSILON = 1e-5
PROFILE_PASS_COUNT = 6


@dataclass(frozen=True)
class SpeedProfile:
    """Curvature-derived physical speed limits around a lap."""

    progress: list[float]
    raw_speeds_mps: list[float]

    def raw_speed_at_progress(self, progress: float) -> float:
        """Interpolate the un-normalized physical speed limit around the lap."""
        return self._value_at_progress(self.raw_speeds_mps, progress, PROFILE_MAX_SPEED_MPS)

    def _value_at_progress(
        self,
        values: list[float],
        progress: float,
        fallback: float,
    ) -> float:
        if not self.progress or not values:
            return fallback
        if len(self.progress) == 1:
            return values[0]

        normalized = progress % 1.0
        index = bisect_right(self.progress, normalized) - 1
        if index < 0:
            index = len(self.progress) - 1

        next_index = (index + 1) % len(self.progress)
        start = self.progress[index]
        end = self.progress[next_index]
        if next_index == 0:
            end += 1.0
        target = normalized if normalized >= start else normalized + 1.0
        span = max(1e-9, end - start)
        ratio = min(1.0, max(0.0, (target - start) / span))
        return values[index] + (values[next_index] - values[index]) * ratio


def _point(values) -> Point:
    if isinstance(values, dict):
        return float(values["x"]), float(values["y"])
    if hasattr(values, "x") and hasattr(values, "y"):
        return float(values.x), float(values.y)
    return float(values[0]), float(values[1])


def _distance(a: Point, b: Point) -> float:
    return hypot(b[0] - a[0], b[1] - a[1])


def _dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _unit_vector(a: Point, b: Point) -> Point | None:
    length = _distance(a, b)
    if length <= 1e-9:
        return None
    return (b[0] - a[0]) / length, (b[1] - a[1]) / length


def _prepare_closed_points(coords: list[list[float]]) -> list[Point]:
    points = [_point(coord) for coord in coords]
    if len(points) > 1 and _distance(points[0], points[-1]) <= 1e-6:
        points = points[:-1]
    return points


def _segment_lengths_m(points: list[Point], track_length_m: float) -> tuple[list[float], float]:
    pixel_lengths = [
        _distance(points[index], points[(index + 1) % len(points)])
        for index in range(len(points))
    ]
    pixel_total = sum(pixel_lengths)
    if pixel_total <= 0:
        return [0.0 for _ in points], 0.0

    meters_per_pixel = track_length_m / pixel_total
    return [length * meters_per_pixel for length in pixel_lengths], pixel_total


def _curvature_at(points: list[Point], segment_lengths_m: list[float], index: int) -> float:
    previous_index = (index - 1) % len(points)
    next_index = (index + 1) % len(points)
    incoming = _unit_vector(points[previous_index], points[index])
    outgoing = _unit_vector(points[index], points[next_index])
    if incoming is None or outgoing is None:
        return 0.0

    clamped_dot = max(-1.0, min(1.0, _dot(incoming, outgoing)))
    turn_angle = acos(clamped_dot)
    arc_length = max(
        1e-6,
        0.5 * (segment_lengths_m[previous_index] + segment_lengths_m[index]),
    )
    return turn_angle / arc_length


def _curvature_speed_limit_mps(curvature: float) -> float:
    if curvature <= CURVATURE_EPSILON:
        return PROFILE_MAX_SPEED_MPS
    speed = sqrt(PROFILE_LATERAL_ACCEL_MPS2 / curvature)
    return min(PROFILE_MAX_SPEED_MPS, max(PROFILE_MIN_SPEED_MPS, speed))


def _apply_braking_and_acceleration_limits(
    speeds: list[float],
    segment_lengths_m: list[float],
    *,
    acceleration_mps2: float,
    braking_mps2: float,
) -> list[float]:
    limited = speeds[:]
    count = len(limited)
    if count < 2:
        return limited

    for _ in range(PROFILE_PASS_COUNT):
        for index in range(count - 1, -1, -1):
            next_index = (index + 1) % count
            distance_m = max(0.0, segment_lengths_m[index])
            allowed = sqrt(limited[next_index] ** 2 + 2.0 * braking_mps2 * distance_m)
            limited[index] = min(limited[index], allowed)

        for index in range(count):
            next_index = (index + 1) % count
            distance_m = max(0.0, segment_lengths_m[index])
            allowed = sqrt(limited[index] ** 2 + 2.0 * acceleration_mps2 * distance_m)
            limited[next_index] = min(limited[next_index], allowed)

    return limited


def build_speed_profile(
    circuit: Circuit,
    *,
    acceleration_mps2: float,
    braking_mps2: float,
) -> SpeedProfile | None:
    """Build a lap-relative speed profile from track geometry curvature."""
    points = _prepare_closed_points(circuit.track_coords)
    if len(points) < 4:
        return None

    track_length_m = max(1.0, float(circuit.track_length_m))
    segment_lengths_m, _ = _segment_lengths_m(points, track_length_m)
    if sum(segment_lengths_m) <= 0:
        return None

    curvature_limits = [
        _curvature_speed_limit_mps(_curvature_at(points, segment_lengths_m, index))
        for index in range(len(points))
    ]
    limited_speeds = _apply_braking_and_acceleration_limits(
        curvature_limits,
        segment_lengths_m,
        acceleration_mps2=acceleration_mps2,
        braking_mps2=braking_mps2,
    )
    total_length = sum(segment_lengths_m)
    cumulative = 0.0
    progress: list[float] = []
    for length in segment_lengths_m:
        progress.append(cumulative / total_length)
        cumulative += length

    return SpeedProfile(
        progress=progress,
        raw_speeds_mps=limited_speeds,
    )
