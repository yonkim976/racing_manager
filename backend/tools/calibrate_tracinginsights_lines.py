"""Derive measured racing-line references from TracingInsights telemetry.

The script reads only qualifying lap JSON from a local sparse checkout of the
TracingInsights 2025 archive.  It selects the fastest clean laps, averages their
position channels, aligns that trajectory to the project's OSM centerline and
stores compact lateral-offset samples in ``circuits.json``.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

BACKEND_DIR = Path(__file__).resolve().parents[1]
if str(BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(BACKEND_DIR))

from data_loader import load_circuits  # noqa: E402
from simulation.track_geometry import normalize_points  # noqa: E402


ARCHIVE_URL = "https://github.com/TracingInsights-Archive/2025"
CAR_WIDTH_M = 1.9
TRACK_EDGE_MARGIN_M = 0.35


@dataclass(frozen=True)
class CircuitSource:
    circuit_id: int
    event_name: str
    # The telemetry trajectory and the OSM-derived game centerline do not have
    # identical geometry.  Keep the measured local shape, but reduce its
    # amplitude where the global alignment residual is large enough to make a
    # literal projection unsafe for the in-game track width.
    data_confidence: float


SOURCES = (
    CircuitSource(3, "Bahrain Grand Prix", 0.60),
    CircuitSource(4, "Austrian Grand Prix", 1.00),
    CircuitSource(5, "British Grand Prix", 0.40),
    CircuitSource(7, "Hungarian Grand Prix", 0.30),
)


@dataclass(frozen=True)
class ReferenceLap:
    time_seconds: float
    driver: str
    lap_number: int
    telemetry_path: Path

    @property
    def label(self) -> str:
        minutes = int(self.time_seconds // 60)
        seconds = self.time_seconds - minutes * 60
        return f"{self.driver} {self.lap_number} {minutes}:{seconds:06.3f}"


def _closed_length(points: list[complex]) -> float:
    return sum(
        abs(points[(index + 1) % len(points)] - point)
        for index, point in enumerate(points)
    )


def _resample_closed(points: list[complex], count: int) -> list[complex]:
    if points and abs(points[0] - points[-1]) <= 1e-9:
        points = points[:-1]
    lengths = [
        abs(points[(index + 1) % len(points)] - point)
        for index, point in enumerate(points)
    ]
    total = max(1e-9, sum(lengths))
    cumulative = [0.0]
    for length in lengths:
        cumulative.append(cumulative[-1] + length)
    result: list[complex] = []
    segment = 0
    for sample_index in range(count):
        target = total * sample_index / count
        while segment + 1 < len(cumulative) and cumulative[segment + 1] < target:
            segment += 1
        length = max(1e-9, lengths[segment % len(lengths)])
        ratio = (target - cumulative[segment]) / length
        start = points[segment % len(points)]
        end = points[(segment + 1) % len(points)]
        result.append(start + (end - start) * ratio)
    return result


def _interpolate_channel(
    distances: list[float],
    values: list[complex],
    target: float,
    start_index: int,
) -> tuple[complex, int]:
    index = start_index
    while index + 1 < len(distances) and distances[index + 1] < target:
        index += 1
    following = min(len(distances) - 1, index + 1)
    span = max(1e-9, distances[following] - distances[index])
    ratio = max(0.0, min(1.0, (target - distances[index]) / span))
    return values[index] + (values[following] - values[index]) * ratio, index


def _telemetry_path(path: Path, count: int) -> list[complex]:
    telemetry = json.loads(path.read_text(encoding="utf-8"))["tel"]
    records = sorted(
        (
            float(distance),
            complex(float(x), float(y)),
        )
        for distance, x, y in zip(
            telemetry["distance"],
            telemetry["x"],
            telemetry["y"],
        )
        if all(math.isfinite(float(value)) for value in (distance, x, y))
    )
    deduped: list[tuple[float, complex]] = []
    for record in records:
        if deduped and record[0] <= deduped[-1][0] + 1e-6:
            deduped[-1] = record
        else:
            deduped.append(record)
    distances = [record[0] for record in deduped]
    values = [record[1] for record in deduped]
    lap_distance = max(1.0, distances[-1] - distances[0])
    result: list[complex] = []
    index = 0
    for sample_index in range(count):
        target = distances[0] + lap_distance * sample_index / count
        value, index = _interpolate_channel(distances, values, target, index)
        result.append(value)
    return result


def _fastest_laps(session_dir: Path, count: int) -> list[ReferenceLap]:
    candidates: list[ReferenceLap] = []
    for lap_times_path in session_dir.glob("*/laptimes.json"):
        driver = lap_times_path.parent.name
        payload = json.loads(lap_times_path.read_text(encoding="utf-8"))
        for lap_number, lap_time in zip(payload["lap"], payload["time"]):
            if not isinstance(lap_time, (int, float)):
                continue
            telemetry_path = lap_times_path.parent / f"{int(lap_number)}_tel.json"
            if not telemetry_path.exists():
                continue
            candidates.append(
                ReferenceLap(
                    time_seconds=float(lap_time),
                    driver=driver,
                    lap_number=int(lap_number),
                    telemetry_path=telemetry_path,
                )
            )
    candidates.sort(key=lambda lap: lap.time_seconds)
    selected: list[ReferenceLap] = []
    used_drivers: set[str] = set()
    for lap in candidates:
        if lap.driver in used_drivers:
            continue
        selected.append(lap)
        used_drivers.add(lap.driver)
        if len(selected) >= count:
            break
    if len(selected) < count:
        raise ValueError(f"Only {len(selected)} distinct-driver laps found in {session_dir}")
    return selected


def _median_path(paths: list[list[complex]]) -> list[complex]:
    return [
        complex(
            statistics.median(path[index].real for path in paths),
            statistics.median(path[index].imag for path in paths),
        )
        for index in range(len(paths[0]))
    ]


def _fit_similarity(
    source: list[complex],
    target: list[complex],
) -> tuple[list[complex], float, str, int]:
    """Return source transformed and reordered into target progress order."""
    best: tuple[float, list[complex], str, int] | None = None
    count = len(source)
    for reverse in (False, True):
        ordered = [source[(-index) % count] for index in range(count)] if reverse else source
        for reflect in (False, True):
            prepared = [value.conjugate() for value in ordered] if reflect else ordered
            source_mean = sum(prepared) / count
            centered_source = [value - source_mean for value in prepared]
            denominator = max(1e-9, sum(abs(value) ** 2 for value in centered_source))
            for shift in range(count):
                paired_target = [target[(index + shift) % count] for index in range(count)]
                target_mean = sum(paired_target) / count
                multiplier = sum(
                    (paired_target[index] - target_mean)
                    * centered_source[index].conjugate()
                    for index in range(count)
                ) / denominator
                transformed = [
                    target_mean + multiplier * centered_source[index]
                    for index in range(count)
                ]
                error = sum(
                    abs(transformed[index] - paired_target[index]) ** 2
                    for index in range(count)
                ) / count
                aligned = [0j] * count
                for index, value in enumerate(transformed):
                    aligned[(index + shift) % count] = value
                mode = (
                    ("reverse" if reverse else "forward")
                    + ("+mirror" if reflect else "")
                )
                candidate = (error, aligned, mode, shift)
                if best is None or candidate[0] < best[0]:
                    best = candidate
    assert best is not None
    return best[1], math.sqrt(best[0]), best[2], best[3]


def _circular_smooth(values: list[float], radius: int = 2) -> list[float]:
    count = len(values)
    return [
        sum(values[(index + delta) % count] for delta in range(-radius, radius + 1))
        / (radius * 2 + 1)
        for index in range(count)
    ]


def _fill_circular_samples(values: list[float | None]) -> list[float]:
    populated = [index for index, value in enumerate(values) if value is not None]
    if not populated:
        return [0.0 for _ in values]
    count = len(values)
    result = [0.0] * count
    for index, value in enumerate(values):
        if value is not None:
            result[index] = value
            continue
        previous = max((item for item in populated if item < index), default=populated[-1] - count)
        following = min((item for item in populated if item > index), default=populated[0] + count)
        previous_value = values[previous % count]
        following_value = values[following % count]
        assert previous_value is not None and following_value is not None
        ratio = (index - previous) / max(1, following - previous)
        result[index] = previous_value + (following_value - previous_value) * ratio
    return result


def _line_reference(
    circuit,
    laps: list[ReferenceLap],
    analysis_samples: int,
    output_samples: int,
    data_confidence: float,
) -> tuple[list[dict[str, float]], dict[str, object]]:
    centerline = [complex(x, y) for x, y in normalize_points(circuit.track_coords)]
    centerline = _resample_closed(centerline, analysis_samples)
    telemetry_paths = [
        _telemetry_path(lap.telemetry_path, analysis_samples)
        for lap in laps
    ]
    measured_path = _median_path(telemetry_paths)
    aligned, rms_units, mode, shift = _fit_similarity(measured_path, centerline)
    meters_per_unit = circuit.track_length_m / max(1e-9, _closed_length(centerline))
    sample_spacing_m = circuit.track_length_m / analysis_samples
    search_radius = max(3, round(140.0 / sample_spacing_m))
    offset_buckets: list[list[float]] = [[] for _ in centerline]
    for approximate_index, measured_point in enumerate(aligned):
        candidates = [
            (approximate_index + delta) % analysis_samples
            for delta in range(-search_radius, search_radius + 1)
        ]
        index = min(
            candidates,
            key=lambda candidate: abs(measured_point - centerline[candidate]),
        )
        point = centerline[index]
        previous = centerline[(index - 1) % analysis_samples]
        following = centerline[(index + 1) % analysis_samples]
        tangent = following - previous
        tangent_length = max(1e-9, abs(tangent))
        normal = complex(-tangent.imag / tangent_length, tangent.real / tangent_length)
        residual = measured_point - point
        offset_buckets[index].append(
            (residual.real * normal.real + residual.imag * normal.imag)
            * meters_per_unit
        )
    raw_offsets = _fill_circular_samples(
        [statistics.median(bucket) if bucket else None for bucket in offset_buckets]
    )
    baseline_radius = max(8, round(300.0 / sample_spacing_m))
    geometry_baseline = _circular_smooth(raw_offsets, radius=baseline_radius)
    local_offsets = [
        offset - baseline
        for offset, baseline in zip(raw_offsets, geometry_baseline)
    ]
    offsets = [
        offset * data_confidence
        for offset in _circular_smooth(_circular_smooth(local_offsets))
    ]
    half_width = max(4.0, float(circuit.track_width_m) / 2.0)
    limit = half_width - CAR_WIDTH_M / 2.0 - TRACK_EDGE_MARGIN_M
    offsets = [max(-limit, min(limit, value)) for value in offsets]
    references = []
    for output_index in range(output_samples):
        source_index = round(output_index * analysis_samples / output_samples) % analysis_samples
        references.append(
            {
                "progress": round(output_index / output_samples, 6),
                "lateral_offset_m": round(offsets[source_index], 3),
            }
        )
    report = {
        "fit_rms_m": round(rms_units * meters_per_unit, 3),
        "line_offset_rms_m": round(
            math.sqrt(sum(offset * offset for offset in offsets) / len(offsets)),
            3,
        ),
        "alignment": mode,
        "progress_shift": round(shift / analysis_samples, 6),
        "offset_min_m": round(min(offsets), 3),
        "offset_max_m": round(max(offsets), 3),
        "edge_limited_samples": sum(abs(offset) >= limit - 1e-6 for offset in offsets),
        "data_confidence": data_confidence,
        "reference_laps": [lap.label for lap in laps],
    }
    return references, report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive", type=Path, required=True)
    parser.add_argument("--circuits-json", type=Path, default=BACKEND_DIR / "data/circuits.json")
    parser.add_argument("--top-laps", type=int, default=5)
    parser.add_argument("--analysis-samples", type=int, default=256)
    parser.add_argument("--output-samples", type=int, default=64)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    circuits = {int(circuit.id): circuit for circuit in load_circuits()}
    updates: dict[int, dict[str, object]] = {}
    reports: dict[str, dict[str, object]] = {}
    for source in SOURCES:
        circuit = circuits[source.circuit_id]
        session_dir = args.archive / source.event_name / "Qualifying"
        laps = _fastest_laps(session_dir, args.top_laps)
        references, report = _line_reference(
            circuit,
            laps,
            args.analysis_samples,
            args.output_samples,
            source.data_confidence,
        )
        updates[source.circuit_id] = {
            "source": "TracingInsights-Archive/2025",
            "source_url": ARCHIVE_URL,
            "reference_season": 2025,
            "reference_session": f"{source.event_name} Qualifying",
            "reference_laps": [lap.label for lap in laps],
            "racing_line_reference": references,
            "racing_line_reference_weight": 0.65,
            "racing_line_initial_smoothing_passes": 1,
        }
        reports[circuit.name] = report

    if args.write:
        payload = json.loads(args.circuits_json.read_text(encoding="utf-8"))
        for circuit_data in payload:
            circuit_id = int(circuit_data["id"])
            if circuit_id in updates:
                circuit_data["physics_calibration"] = updates[circuit_id]
        args.circuits_json.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(reports, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
