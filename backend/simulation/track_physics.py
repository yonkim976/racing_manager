"""Metric track-width and automatically generated racing-line profiles."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from math import atan2, hypot

from models.schemas import Circuit
from simulation.track_geometry import normalize_points

DEFAULT_TRACK_WIDTH_M = 12.0
PHYSICAL_CAR_WIDTH_M = 1.9
PHYSICAL_CAR_LENGTH_M = 5.0
TRACK_EDGE_MARGIN_M = 0.35


@dataclass(frozen=True)
class TrackPhysicsSample:
    progress: float
    left_width_m: float
    right_width_m: float
    racing_line_offset_m: float
    turn_signal: float


class TrackPhysicsProfile:
    def __init__(self, samples: list[TrackPhysicsSample]) -> None:
        self.samples = samples
        self.progress = [sample.progress for sample in samples]

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


def _closed_points(circuit: Circuit) -> list[tuple[float, float]]:
    points = normalize_points(circuit.track_coords)
    if len(points) > 1 and hypot(
        points[0][0] - points[-1][0],
        points[0][1] - points[-1][1],
    ) <= 1e-6:
        points = points[:-1]
    return points


def _metric_widths(circuit: Circuit, count: int) -> list[tuple[float, float]]:
    fallback_half = max(4.0, float(circuit.track_width_m or DEFAULT_TRACK_WIDTH_M) / 2.0)
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


def build_track_physics_profile(circuit: Circuit) -> TrackPhysicsProfile:
    points = _closed_points(circuit)
    if len(points) < 3:
        return TrackPhysicsProfile([])

    lengths = [
        hypot(
            points[(index + 1) % len(points)][0] - point[0],
            points[(index + 1) % len(points)][1] - point[1],
        )
        for index, point in enumerate(points)
    ]
    total_length = max(1e-9, sum(lengths))
    cumulative = 0.0
    progress: list[float] = []
    for length in lengths:
        progress.append(cumulative / total_length)
        cumulative += length

    widths = _metric_widths(circuit, len(points))
    turns = [_signed_turn_signal(points, index) for index in range(len(points))]
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

    smoothed = raw_offsets
    for _ in range(3):
        smoothed = [
            0.25 * smoothed[(index - 1) % count]
            + 0.5 * smoothed[index]
            + 0.25 * smoothed[(index + 1) % count]
            for index in range(count)
        ]

    return TrackPhysicsProfile([
        TrackPhysicsSample(
            progress=progress[index],
            left_width_m=widths[index][0],
            right_width_m=widths[index][1],
            racing_line_offset_m=smoothed[index],
            turn_signal=turns[index],
        )
        for index in range(count)
    ])
