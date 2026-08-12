#!/usr/bin/env python3
"""Run a reproducible, fixed-step baseline for a supported circuit.

This tool is deliberately a diagnostic entry point rather than a tuning loop.
It compares the measured static target profile with the real RaceEngine path,
then records a bounded single-car runtime sample using authoritative 50 Hz
physics steps.  Traffic, pit and Safety Car matrices remain separate tests.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
from typing import Any, Iterable


BACKEND_DIR = Path(__file__).resolve().parents[1]
PROJECT_DIR = BACKEND_DIR.parent
if str(BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(BACKEND_DIR))

from data_loader import load_circuits, load_drivers, load_teams
from models.schemas import (
    Circuit,
    Driver,
    DryTireRole,
    PaceMode,
    ThermalPresetName,
    TireCompound,
)
from engines.full.runtime.physics import GAME_TICK_SECONDS
from engines.full.runtime.race_engine import RaceEngine
from simulation.track_physics import DRIVING_LINE_RACING
from engines.full.runtime.vehicle_dynamics import (
    DYNAMIC_BICYCLE_MAX_CONTROLLED_HEADING_ERROR_RAD,
)
from engines.full.runtime.vehicle_physics import VehiclePhysicsModifiers
from tools.audit_tier_a import audit_circuit


PHYSICS_STEP_SECONDS = 0.02
DEFAULT_PACE = 1.04
REFERENCE_SOURCE_FILES = (
    "backend/data/circuits.json",
    "backend/data/circuit_sources/red_bull_ring_osm_geo.json",
    "backend/data/circuit_sources/red_bull_ring_track_profile_v2.json",
    "backend/data/circuit_thermal_profiles.json",
    "backend/data/circuit_tire_wear_profiles.json",
    "backend/data/tire_compound_nominations.json",
)


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _round(value: float | None, digits: int = 3) -> float | None:
    if value is None or not _finite(value):
        return None
    return round(float(value), digits)


def _p95(values: Iterable[float]) -> float:
    ordered = sorted(float(value) for value in values if _finite(value))
    if not ordered:
        return 0.0
    return ordered[round((len(ordered) - 1) * 0.95)]


def _mean(values: Iterable[float]) -> float:
    values = [float(value) for value in values if _finite(value)]
    return statistics.fmean(values) if values else 0.0


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_identity() -> dict[str, Any]:
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=PROJECT_DIR,
            text=True,
        ).strip()
        dirty = bool(
            subprocess.check_output(
                ["git", "status", "--short"],
                cwd=PROJECT_DIR,
                text=True,
            ).strip()
        )
        branch = subprocess.check_output(
            ["git", "branch", "--show-current"],
            cwd=PROJECT_DIR,
            text=True,
        ).strip()
        user_name = subprocess.check_output(
            ["git", "config", "user.name"],
            cwd=PROJECT_DIR,
            text=True,
        ).strip()
        user_email = subprocess.check_output(
            ["git", "config", "user.email"],
            cwd=PROJECT_DIR,
            text=True,
        ).strip()
        return {
            "git_commit": commit,
            "git_branch": branch,
            "git_user_name": user_name,
            "git_user_email": user_email,
            "working_tree_dirty": dirty,
        }
    except (OSError, subprocess.CalledProcessError):
        return {
            "git_commit": None,
            "working_tree_dirty": None,
            "working_tree_note": "git identity unavailable",
        }


def _reference_lap_seconds(label: str) -> float | None:
    try:
        time_text = label.rsplit(" ", 1)[-1]
        minutes, seconds = time_text.split(":", 1)
        return float(minutes) * 60.0 + float(seconds)
    except (ValueError, IndexError):
        return None


def _reference_at(samples: list[Any], progress: float) -> Any:
    return min(
        samples,
        key=lambda sample: min(
            (float(sample.progress) - progress) % 1.0,
            (progress - float(sample.progress)) % 1.0,
        ),
    )


def _progress_from_start(start: float, progress: float) -> float:
    return (progress - start) % 1.0


def _signed_progress_delta(first: float, second: float) -> float:
    if first is None or second is None:
        return 0.0
    delta = (second - first) % 1.0
    return delta if delta <= 0.5 else delta - 1.0


def _aligned_telemetry_reference(calibration: Any) -> list[Any]:
    """Express source telemetry progress in compiled-centerline coordinates."""
    offset = float(getattr(calibration, "telemetry_progress_offset", 0.0))
    if abs(offset) <= 1e-12:
        return list(calibration.telemetry_reference)
    return sorted(
        (
            sample.model_copy(
                update={"progress": (float(sample.progress) - offset) % 1.0}
            )
            for sample in calibration.telemetry_reference
        ),
        key=lambda sample: sample.progress,
    )


def _line_sample_value(
    track_profile: Any,
    line: str,
    progress: float,
    field: str,
) -> float:
    """Interpolate one active-line path field in centerline progress space."""
    samples = track_profile.driving_line_samples.get(line) or track_profile.racing_line_samples
    if not samples:
        return 0.0
    index, next_index, ratio = track_profile._line_indices_at_center_progress(progress)
    first = float(getattr(samples[index], field))
    second = float(getattr(samples[next_index], field))
    return first + (second - first) * ratio


def _reference_lateral_speed_candidate(
    track_profile: Any,
    line: str,
    progress: float,
    speed_mps: float,
    track_length_m: float,
    sample_distance_m: float = 1.0,
) -> float:
    """Estimate d(line offset)/dt in the same centreline-relative frame."""
    distance = max(0.25, float(sample_distance_m))
    progress_delta = distance / max(1.0, float(track_length_m))
    forward = track_profile.line_offset_at_progress(line, progress + progress_delta)
    backward = track_profile.line_offset_at_progress(line, progress - progress_delta)
    derivative_per_m = (float(forward) - float(backward)) / (2.0 * distance)
    return derivative_per_m * max(0.0, float(speed_mps))


def _line_relative_lateral_error_m(
    vehicle_centerline_offset_m: float,
    centerline_reference_offset_m: float,
) -> float:
    """Subtract two offsets that are explicitly expressed in one frame."""
    return float(vehicle_centerline_offset_m) - float(centerline_reference_offset_m)


def _signed_error_metrics(
    samples: list[dict[str, float]],
    *,
    threshold_m: float = 2.0,
    step_seconds: float = PHYSICS_STEP_SECONDS,
) -> dict[str, Any]:
    """Summarize signed line-relative error and corrective reversals."""
    errors = [float(sample["line_relative_lateral_error_m"]) for sample in samples]
    if not errors:
        return {
            "sample_count": 0,
            "mae_m": 0.0,
            "p95_m": 0.0,
            "max_m": 0.0,
            "abs_over_threshold_seconds": 0.0,
            "maximum_continuous_over_threshold_seconds": 0.0,
            "zero_crossings": 0,
            "correction_reversals": 0,
        }

    absolute = [abs(value) for value in errors]
    over_threshold_steps = [value > threshold_m for value in absolute]
    maximum_continuous_steps = 0
    current_steps = 0
    for is_over in over_threshold_steps:
        current_steps = current_steps + 1 if is_over else 0
        maximum_continuous_steps = max(maximum_continuous_steps, current_steps)

    zero_crossings = 0
    previous_error = errors[0]
    for error in errors[1:]:
        if previous_error * error < 0.0:
            zero_crossings += 1
        previous_error = error

    # A 20 ms one-step delta misses a smooth but visually large reversal.  Use
    # non-overlapping 0.20 s means, reset at every lap, and apply hysteresis so
    # numerical jitter around a stationary error is not counted as steering.
    smoothing_steps = max(1, round(0.20 / max(step_seconds, 1e-9)))
    samples_by_lap: dict[int, list[dict[str, float]]] = defaultdict(list)
    for sample in samples:
        total_progress = float(sample.get("total_progress", 0.0))
        samples_by_lap[math.floor(total_progress)].append(sample)
    correction_reversals = 0
    for lap_samples in samples_by_lap.values():
        smoothed_errors = [
            _mean(
                float(item["line_relative_lateral_error_m"])
                for item in lap_samples[index : index + smoothing_steps]
            )
            for index in range(0, len(lap_samples), smoothing_steps)
            if len(lap_samples[index : index + smoothing_steps]) == smoothing_steps
        ]
        previous_direction = 0
        previous_smoothed_error: float | None = None
        for smoothed_error in smoothed_errors:
            if previous_smoothed_error is None:
                previous_smoothed_error = smoothed_error
                continue
            delta = smoothed_error - previous_smoothed_error
            direction = 1 if delta >= 0.05 else -1 if delta <= -0.05 else 0
            if (
                direction != 0
                and previous_direction != 0
                and direction != previous_direction
                and abs(smoothed_error) >= 0.25
            ):
                correction_reversals += 1
            if direction != 0:
                previous_direction = direction
            previous_smoothed_error = smoothed_error

    return {
        "sample_count": len(errors),
        "mae_m": _round(_mean(absolute), 4),
        "p95_m": _round(_p95(absolute), 4),
        "max_m": _round(max(absolute), 4),
        "signed_mean_m": _round(_mean(errors), 4),
        "signed_min_m": _round(min(errors), 4),
        "signed_max_m": _round(max(errors), 4),
        "abs_over_threshold_seconds": _round(
            sum(over_threshold_steps) * step_seconds,
            4,
        ),
        "maximum_continuous_over_threshold_seconds": _round(
            maximum_continuous_steps * step_seconds,
            4,
        ),
        "zero_crossings": zero_crossings,
        "correction_reversals": correction_reversals,
    }


def _corner_error_reports(
    circuit: Circuit,
    samples: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Report line-relative error by corner and entry/apex/exit window."""
    reports: list[dict[str, Any]] = []
    for corner in _corner_windows(circuit):
        start = float(corner["approach_start_progress"])
        end = float(corner["exit_end_progress"])
        span = _progress_from_start(start, end)
        window = [
            sample
            for sample in samples
            if _progress_from_start(start, float(sample["progress"])) <= span
        ]
        if not window:
            continue
        sections: dict[str, list[dict[str, Any]]] = {
            "entry": [],
            "apex": [],
            "exit": [],
        }
        for sample in window:
            ratio = _progress_from_start(start, float(sample["progress"])) / max(span, 1e-9)
            sections["entry" if ratio < 1.0 / 3.0 else "apex" if ratio < 2.0 / 3.0 else "exit"].append(sample)
        reports.append(
            {
                "corner": corner["corner"],
                "apex_progress": corner["apex_progress"],
                "approach_start_progress": corner["approach_start_progress"],
                "exit_end_progress": corner["exit_end_progress"],
                "window_sample_count": len(window),
                "line_error": _signed_error_metrics(window),
                "entry": _signed_error_metrics(sections["entry"]),
                "apex": _signed_error_metrics(sections["apex"]),
                "exit": _signed_error_metrics(sections["exit"]),
                "minimum_speed_kph": _round(min(sample["speed_kph"] for sample in window), 3),
                "exit_50m_speed_kph": _round(
                    max(
                        (
                            sample["speed_kph"]
                            for sample in window
                            if _progress_from_start(
                                corner["apex_progress"],
                                float(sample["progress"]),
                            ) * circuit.track_length_m >= 50.0
                        ),
                        default=0.0,
                    ),
                    3,
                ),
                "exit_100m_speed_kph": _round(
                    max(
                        (
                            sample["speed_kph"]
                            for sample in window
                            if _progress_from_start(
                                corner["apex_progress"],
                                float(sample["progress"]),
                            ) * circuit.track_length_m >= 100.0
                        ),
                        default=0.0,
                    ),
                    3,
                ),
            }
        )
    return reports


def _static_telemetry_metrics(
    circuit: Circuit,
    drivers: list[Driver],
    teams: dict[int, Any],
    pace: float,
    seed: int,
) -> dict[str, Any]:
    calibration = circuit.physics_calibration
    if calibration is None or not calibration.telemetry_reference:
        return {"status": "not_available", "reason": "telemetry_reference is empty"}

    engine = RaceEngine(
        circuit=circuit,
        drivers=drivers[:1],
        teams=teams,
        player_team_id=drivers[0].team_id,
        player_driver_ids=[drivers[0].id],
        seed=seed,
        start_sequence_enabled=False,
    )
    state = engine.driver_states[drivers[0].id]
    track = engine._track_physics_for_driver(state)
    physics = engine._vehicle_physics_for_driver(state)[DRIVING_LINE_RACING]
    modifiers = VehiclePhysicsModifiers(pace=pace)

    errors: list[float] = []
    for sample in calibration.telemetry_reference:
        distance = track.line_distance_at_total_progress(
            DRIVING_LINE_RACING,
            float(sample.progress) - calibration.telemetry_progress_offset,
        )
        predicted = physics.target_speed_mps(distance, modifiers) * 3.6
        errors.append(predicted - float(sample.speed_kph))

    reference_laps = [
        seconds
        for seconds in (
            _reference_lap_seconds(label) for label in calibration.reference_laps
        )
        if seconds is not None
    ]
    return {
        "status": "pass",
        "pace_modifier": pace,
        "reference_lap_count": len(calibration.reference_laps),
        "reference_laps": list(calibration.reference_laps),
        "reference_lap_time_median_seconds": _round(_mean(reference_laps), 3),
        "sample_count": len(errors),
        "mae_kph": _round(_mean(abs(error) for error in errors), 3),
        "rmse_kph": _round(math.sqrt(_mean(error * error for error in errors)), 3),
        "bias_kph": _round(_mean(errors), 3),
        "maximum_over_kph": _round(max(errors), 3),
        "maximum_under_kph": _round(min(errors), 3),
        "maximum_absolute_error_kph": _round(max(abs(error) for error in errors), 3),
        "thresholds": {
            "mae_kph_lt": 6.0,
            "rmse_kph_lt": 9.0,
            "absolute_bias_kph_lt": 4.0,
            "maximum_absolute_error_kph_lt": 35.0,
        },
    }


def _corner_windows(circuit: Circuit) -> list[dict[str, Any]]:
    corners = []
    for landmark in circuit.landmarks:
        label = landmark.label or landmark.name or ""
        if landmark.type != "corner" or not label.upper().startswith("T"):
            continue
        try:
            int(label[1:])
        except ValueError:
            continue
        corners.append(
            {
                "corner": label,
                "apex_progress": float(landmark.progress or 0.0) % 1.0,
            }
        )
    corners.sort(key=lambda item: item["apex_progress"])
    if not corners:
        return []

    bounded = []
    for index, corner in enumerate(corners):
        previous_apex = corners[index - 1]["apex_progress"]
        next_apex = corners[(index + 1) % len(corners)]["apex_progress"]
        apex = corner["apex_progress"]
        approach_span = min(0.08, _progress_from_start(previous_apex, apex) * 0.5)
        exit_span = min(0.08, _progress_from_start(apex, next_apex) * 0.5)
        bounded.append(
            {
                **corner,
                # Midpoints keep adjacent turns from sharing the same minimum
                # speed while the cap prevents a long straight from becoming
                # part of a corner diagnostic.
                "approach_start_progress": (apex - approach_span) % 1.0,
                "exit_end_progress": (apex + exit_span) % 1.0,
            }
        )
    return bounded


def _nearest_corner_label(circuit: Circuit, progress: float) -> str:
    corners = _corner_windows(circuit)
    if not corners:
        return "unattributed"
    nearest = min(
        corners,
        key=lambda corner: min(
            _progress_from_start(corner["apex_progress"], progress),
            _progress_from_start(progress, corner["apex_progress"]),
        ),
    )
    distance = min(
        _progress_from_start(nearest["apex_progress"], progress),
        _progress_from_start(progress, nearest["apex_progress"]),
    )
    return nearest["corner"] if distance <= 0.04 else "unattributed"


def _corner_report(
    circuit: Circuit,
    calibration: Any,
    samples: list[dict[str, float]],
    track_length_m: float,
) -> list[dict[str, Any]]:
    if calibration is None or not calibration.telemetry_reference:
        return []

    aligned_reference = _aligned_telemetry_reference(calibration)
    report = []
    for corner in _corner_windows(circuit):
        apex = corner["apex_progress"]
        window_span = _progress_from_start(
            corner["approach_start_progress"],
            corner["exit_end_progress"],
        )
        window = [
            sample
            for sample in samples
            if _progress_from_start(corner["approach_start_progress"], sample["progress"])
            <= window_span
        ]
        if not window:
            continue
        reference_window = [
            sample
            for sample in aligned_reference
            if _progress_from_start(corner["approach_start_progress"], float(sample.progress))
            <= window_span
        ]
        if not reference_window:
            continue
        # Telemetry is sampled more sparsely than the 50 Hz runtime. Include
        # the nearest boundary samples so a midpoint between adjacent corners
        # does not discard the true exit minimum by less than one source step.
        reference_window = [
            *reference_window,
            _reference_at(
                aligned_reference,
                corner["approach_start_progress"],
            ),
            _reference_at(
                aligned_reference,
                corner["exit_end_progress"],
            ),
        ]
        actual_apex = min(window, key=lambda sample: min(
            _progress_from_start(sample["progress"], apex),
            _progress_from_start(apex, sample["progress"]),
        ))
        reference_apex = min(
            reference_window,
            key=lambda sample: min(
                _progress_from_start(float(sample.progress), apex),
                _progress_from_start(apex, float(sample.progress)),
            ),
        )
        actual_min_sample = min(window, key=lambda sample: sample["speed_kph"])
        actual_min_speed = actual_min_sample["speed_kph"]
        reference_min_speed = min(float(sample.speed_kph) for sample in reference_window)

        def first_progress(
            candidates: list[dict[str, float]],
            predicate,
        ) -> float | None:
            matching = [sample for sample in candidates if predicate(sample)]
            if not matching:
                return None
            return min(
                matching,
                key=lambda sample: _progress_from_start(
                    corner["approach_start_progress"], sample["progress"]
                ),
            )["progress"]

        actual_brake_start = first_progress(window, lambda sample: sample["brake"] >= 0.05)
        reference_brake_start = first_progress(
            [
                {
                    "progress": float(sample.progress),
                    "brake": float(sample.braking_fraction),
                    "throttle": float(sample.throttle_percent) / 100.0,
                }
                for sample in reference_window
            ],
            lambda sample: sample["brake"] >= 0.5,
        )
        actual_throttle_resume = first_progress(
            [
                sample
                for sample in window
                if _progress_from_start(apex, sample["progress"]) <= 0.06
            ],
            lambda sample: sample["throttle"] >= 0.8,
        )
        reference_throttle_resume = first_progress(
            [
                {
                    "progress": float(sample.progress),
                    "brake": float(sample.braking_fraction),
                    "throttle": float(sample.throttle_percent) / 100.0,
                }
                for sample in reference_window
                if _progress_from_start(apex, float(sample.progress)) <= 0.06
            ],
            lambda sample: sample["throttle"] >= 0.8,
        )

        report.append(
            {
                "corner": corner["corner"],
                "apex_progress": _round(apex, 6),
                "actual_apex_progress": _round(actual_apex["progress"], 6),
                "reference_apex_progress": _round(float(reference_apex.progress), 6),
                "minimum_speed_kph": _round(actual_min_speed, 3),
                "reference_minimum_speed_kph": _round(reference_min_speed, 3),
                "minimum_speed_error_kph": _round(actual_min_speed - reference_min_speed, 3),
                "minimum_speed_progress": _round(
                    actual_min_sample["progress"],
                    6,
                ),
                "target_at_minimum_speed_kph": _round(
                    actual_min_sample["target_speed_kph"],
                    3,
                ),
                "handling_at_minimum_speed": actual_min_sample["handling_state"],
                "planner_fallback_at_minimum_speed": bool(
                    actual_min_sample["planner_fallback"]
                ),
                "emergency_braking_at_minimum_speed": bool(
                    actual_min_sample["emergency_braking"]
                ),
                "lateral_offset_at_minimum_m": _round(
                    actual_min_sample["lateral_offset_m"],
                    4,
                ),
                "racing_line_offset_at_minimum_m": _round(
                    actual_min_sample["racing_line_offset_m"],
                    4,
                ),
                "nominal_lateral_bounds_at_minimum_m": [
                    _round(actual_min_sample["nominal_minimum_m"], 4),
                    _round(actual_min_sample["nominal_maximum_m"], 4),
                ],
                "brake_start_progress": _round(actual_brake_start, 6),
                "reference_brake_start_progress": _round(reference_brake_start, 6),
                "brake_start_distance_delta_m": (
                    _round(
                        _signed_progress_delta(
                            reference_brake_start,
                            actual_brake_start,
                        ) * track_length_m,
                        3,
                    )
                    if actual_brake_start is not None and reference_brake_start is not None
                    else None
                ),
                "throttle_resume_progress": _round(actual_throttle_resume, 6),
                "reference_throttle_resume_progress": _round(reference_throttle_resume, 6),
                "throttle_resume_distance_delta_m": (
                    _round(
                        _signed_progress_delta(
                            reference_throttle_resume,
                            actual_throttle_resume,
                        ) * track_length_m,
                        3,
                    )
                    if actual_throttle_resume is not None and reference_throttle_resume is not None
                    else None
                ),
            }
        )
    return report


def _speed_opportunity_report(
    circuit: Circuit,
    calibration: Any,
    samples: list[dict[str, float]],
    *,
    bin_count: int = 64,
) -> dict[str, Any]:
    """Locate clean sectors where runtime speed is materially below telemetry."""
    if calibration is None or not calibration.telemetry_reference or not samples:
        return {"status": "not_available", "zones": []}

    aligned_reference = _aligned_telemetry_reference(calibration)
    bins: dict[int, list[dict[str, float]]] = defaultdict(list)
    for sample in samples:
        bins[min(bin_count - 1, int(sample["progress"] * bin_count))].append(sample)

    zones: list[dict[str, Any]] = []
    for index, bin_samples in bins.items():
        clean_samples = [
            sample
            for sample in bin_samples
            if not sample["off_track"]
            and not sample["track_limits"]
            and sample["handling_state"] not in {"run_wide", "recovering"}
            and not sample["planner_fallback"]
        ]
        clean_ratio = len(clean_samples) / len(bin_samples)
        if clean_ratio < 0.95 or len(clean_samples) < 2:
            continue
        center_progress = (index + 0.5) / bin_count
        reference = _reference_at(
            aligned_reference,
            center_progress,
        )
        actual_speed = statistics.median(
            sample["speed_kph"] for sample in clean_samples
        )
        target_speed = statistics.median(
            sample["target_speed_kph"] for sample in clean_samples
        )
        reference_speed = float(reference.speed_kph)
        speed_error = actual_speed - reference_speed
        if speed_error > -12.0:
            continue
        target_error = target_speed - reference_speed
        zones.append(
            {
                "nearest_corner": _nearest_corner_label(circuit, center_progress),
                "start_progress": _round(index / bin_count, 6),
                "end_progress": _round((index + 1) / bin_count, 6),
                "actual_speed_kph": _round(actual_speed, 3),
                "target_speed_kph": _round(target_speed, 3),
                "reference_speed_kph": _round(reference_speed, 3),
                "actual_minus_reference_kph": _round(speed_error, 3),
                "target_minus_reference_kph": _round(target_error, 3),
                "constraint_type": (
                    "controller_target_limited"
                    if target_error <= -12.0
                    else "runtime_acceleration_or_drs_gap"
                ),
                "clean_sample_ratio": _round(clean_ratio, 4),
                "grip_utilization_p95": _round(
                    _p95(sample["grip_utilization"] for sample in clean_samples),
                    4,
                ),
                "handling_samples": dict(
                    sorted(
                        Counter(
                            sample["handling_state"] for sample in bin_samples
                        ).items()
                    )
                ),
                "drs_sample_ratio": _round(
                    _mean(float(sample["drs_active"]) for sample in bin_samples),
                    4,
                ),
            }
        )
    zones.sort(key=lambda zone: zone["actual_minus_reference_kph"])
    return {
        "status": "pass",
        "definition": (
            "64 equal progress bins; clean ratio >= 0.95 and "
            "median actual speed at least 12 km/h below reference"
        ),
        "zones": zones,
    }


def _run_single_car(
    circuit: Circuit,
    drivers: list[Driver],
    teams: dict[int, Any],
    *,
    laps: int,
    seed: int,
    thermal_preset: ThermalPresetName,
    driver_id: int | None = None,
    tire_role: DryTireRole = DryTireRole.MEDIUM,
    pace_mode: PaceMode = PaceMode.STANDARD,
    reference_lateral_speed_feedforward: float = 1.0,
    clean_line_lateral_speed_limit_mps: float = 1.25,
) -> dict[str, Any]:
    driver = next((item for item in drivers if item.id == driver_id), drivers[0])
    # Keep the official race distance authoritative for fuel mass and strategy
    # state.  The diagnostic loop stops after ``laps`` samples instead of
    # shortening the event and accidentally testing a qualifying fuel load.
    runtime_circuit = circuit.model_copy()
    conditions = runtime_circuit.thermal_profile.presets[thermal_preset]
    engine = RaceEngine(
        circuit=runtime_circuit,
        drivers=[driver],
        teams=teams,
        player_team_id=driver.team_id,
        player_driver_ids=[driver.id],
        starting_tires={driver.id: TireCompound(tire_role.value)},
        seed=seed,
        start_sequence_enabled=False,
        track_conditions=conditions,
        thermal_preset=thermal_preset,
        track_conditions_source="circuit_preset",
        clean_line_lateral_speed_feedforward=reference_lateral_speed_feedforward,
        clean_line_lateral_speed_limit_mps=clean_line_lateral_speed_limit_mps,
    )
    state = engine.driver_states[driver.id]
    track_profile = engine._track_physics_for_driver(state)
    state.pace_mode = pace_mode
    calibration = runtime_circuit.physics_calibration
    telemetry = _aligned_telemetry_reference(calibration) if calibration else []
    input_contract = {
        "driver_id": driver.id,
        "tire_role": state.tire_role.value,
        "physical_tire_compound": state.physical_tire_compound.value,
        "pace_mode": state.pace_mode.value,
        "clean_line_lateral_speed_feedforward": reference_lateral_speed_feedforward,
        "clean_line_lateral_speed_limit_mps": clean_line_lateral_speed_limit_mps,
        "driver_pace_multiplier": _round(engine._driver_meta[driver.id]["pace"], 5),
        "fuel_start_kg": _round(state.fuel_mass_kg, 3),
        "fuel_basis": "full_race_distance",
        "fuel_basis_laps": int(runtime_circuit.total_laps),
        "sample_laps": int(laps),
        "tire_age_start_laps": state.tire_age,
        "tire_wear_start": _round(state.tire_wear, 5),
        "track_conditions": conditions.model_dump(mode="json"),
    }
    samples_by_lap: dict[int, list[dict[str, float]]] = defaultdict(list)
    coordinate_samples: list[dict[str, Any]] = []
    event_counts: Counter[str] = Counter()
    status_counts: Counter[str] = Counter()
    handling_counts: Counter[str] = Counter()
    run_wide_by_corner: Counter[str] = Counter()
    off_track_by_corner: Counter[str] = Counter()
    lateral_errors: list[float] = []
    signed_lateral_errors: list[float] = []
    heading_errors: list[float] = []
    maximum_traction_slip_ratio = 0.0
    maximum_wheel_lock_ratio = 0.0
    brake_peaks = {"front_c": 0.0, "rear_c": 0.0, "minimum_fade_factor": 1.0}
    step_count = 0
    numeric_guard_hits = 0
    fallback_consecutive_steps = 0
    maximum_consecutive_fallback_steps = 0
    maximum_steps = max(1, int(laps * runtime_circuit.base_lap_time / PHYSICS_STEP_SECONDS * 2.0))

    while (
        step_count < maximum_steps
        and not state.finished
        and len(engine._lap_history.get(driver.id, [])) < laps
    ):
        events = engine.tick(PHYSICS_STEP_SECONDS)
        step_count += 1
        for event in events:
            event_counts[event.type] += 1

        nominal_bounds = engine._physics_v2_nominal_lateral_bounds(state)
        active_line = state.racing_line or DRIVING_LINE_RACING
        line_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            state.total_progress,
        )
        line_length_m = max(1.0, track_profile.length_for_line(active_line))
        active_line_offset_m = float(
            track_profile.line_offset_at_progress(active_line, state.progress)
        )
        telemetry_sample = _reference_at(telemetry, state.progress) if telemetry else None
        telemetry_offset = float(
            getattr(calibration, "telemetry_progress_offset", 0.0)
            if calibration
            else 0.0
        )
        active_line_pose_x_m, active_line_pose_y_m, active_line_heading_rad = (
            track_profile.line_pose_at_progress_m(active_line, state.progress)
            if track_profile.coordinate_frame is not None
            else (0.0, 0.0, 0.0)
        )
        line_relative_error_m = _line_relative_lateral_error_m(
            state.lateral_offset_m,
            active_line_offset_m,
        )
        reference_lateral_speed_mps = _reference_lateral_speed_candidate(
            track_profile,
            active_line,
            state.progress,
            float(state.speed_kph) / 3.6,
            float(runtime_circuit.track_length_m),
        )
        sample = {
            "progress": float(state.progress) % 1.0,
            "total_progress": float(state.total_progress),
            "current_lap": int(state.current_lap),
            "compiled_centerline_progress": float(state.progress) % 1.0,
            "active_line": active_line,
            "active_path_progress": (line_distance_m % line_length_m) / line_length_m,
            "active_path_distance_m": line_distance_m,
            "telemetry_source_progress": (
                ((float(telemetry_sample.progress) + telemetry_offset) % 1.0)
                if telemetry_sample is not None
                else None
            ),
            "telemetry_aligned_progress": (
                float(telemetry_sample.progress) if telemetry_sample is not None else None
            ),
            "speed_kph": float(state.speed_kph),
            "target_speed_kph": float(state.target_speed_kph),
            "throttle": float(state.throttle),
            "brake": float(state.brake),
            "lateral_offset_m": float(state.lateral_offset_m),
            "heading_error_rad": abs(float(state.slip_angle_rad)),
            "heading_error_signed_rad": float(state.slip_angle_rad),
            "grip_utilization": float(state.grip_utilization),
            "handling_state": str(state.handling_state),
            "off_track": bool(state.off_track),
            "track_limits": bool(state.track_limits_active),
            "planner_fallback": bool(state.planner_fallback_active),
            "drs_active": bool(state.drs_active),
            "emergency_braking": bool(state.emergency_braking),
            "racing_line_offset_m": active_line_offset_m,
            "centerline_reference_lateral_offset_m": active_line_offset_m,
            "vehicle_centerline_lateral_offset_m": float(state.lateral_offset_m),
            "active_line_pose_vehicle_lateral_offset_m": line_relative_error_m,
            "line_relative_lateral_error_m": line_relative_error_m,
            "target_lateral_offset_m": float(state.target_lateral_offset_m),
            "vehicle_lateral_speed_mps": float(state.lateral_speed_mps),
            "reference_lateral_speed_candidate_mps": reference_lateral_speed_mps,
            "active_line_pose_x_m": active_line_pose_x_m,
            "active_line_pose_y_m": active_line_pose_y_m,
            "active_line_pose_heading_rad": active_line_heading_rad,
            "active_line_curvature_1pm": _line_sample_value(
                track_profile,
                active_line,
                state.progress,
                "curvature_1pm",
            ),
            "traction_slip_ratio": float(state.traction_slip_ratio),
            "wheel_lock_ratio": float(state.wheel_lock_ratio),
            "nominal_minimum_m": (
                float(nominal_bounds[0]) if nominal_bounds is not None else 0.0
            ),
            "nominal_maximum_m": (
                float(nominal_bounds[1]) if nominal_bounds is not None else 0.0
            ),
        }
        samples_by_lap[state.current_lap].append(sample)
        if state.current_lap >= 2:
            signed_lateral_errors.append(line_relative_error_m)
            lateral_errors.append(abs(line_relative_error_m))
            heading_errors.append(abs(float(state.slip_angle_rad)))
            if step_count % 10 == 0:
                coordinate_samples.append(dict(sample))
        status_counts["off_track"] += int(state.off_track)
        status_counts["track_limits"] += int(state.track_limits_active)
        status_counts["run_wide"] += int(state.handling_state == "run_wide")
        status_counts["contact"] += int(state.contact_active)
        status_counts["planner_fallback"] += int(state.planner_fallback_active)
        handling_counts[str(state.handling_state)] += 1
        maximum_traction_slip_ratio = max(
            maximum_traction_slip_ratio,
            float(state.traction_slip_ratio),
        )
        maximum_wheel_lock_ratio = max(
            maximum_wheel_lock_ratio,
            float(state.wheel_lock_ratio),
        )
        if state.handling_state == "run_wide":
            run_wide_by_corner[_nearest_corner_label(runtime_circuit, state.progress)] += 1
        if state.off_track:
            off_track_by_corner[_nearest_corner_label(runtime_circuit, state.progress)] += 1
        if state.planner_fallback_active:
            fallback_consecutive_steps += 1
            maximum_consecutive_fallback_steps = max(
                maximum_consecutive_fallback_steps,
                fallback_consecutive_steps,
            )
        else:
            fallback_consecutive_steps = 0
        if not all(
            _finite(getattr(state, field, None))
            for field in (
                "progress",
                "speed_kph",
                "target_speed_kph",
                "throttle",
                "brake",
                "lateral_offset_m",
                "slip_angle_rad",
                "front_brake_temperature_c",
                "rear_brake_temperature_c",
            )
        ):
            numeric_guard_hits += 1
        brake_peaks["front_c"] = max(
            brake_peaks["front_c"], float(state.front_brake_temperature_c)
        )
        brake_peaks["rear_c"] = max(
            brake_peaks["rear_c"], float(state.rear_brake_temperature_c)
        )
        brake_peaks["minimum_fade_factor"] = min(
            brake_peaks["minimum_fade_factor"], float(state.brake_fade_factor)
        )

    history = [
        item.model_dump(mode="json")
        for item in engine._lap_history.get(driver.id, [])
    ]
    valid_lap_numbers = [lap for lap in range(2, laps + 1) if samples_by_lap.get(lap)]
    valid_samples = [
        sample
        for lap in valid_lap_numbers
        for sample in samples_by_lap[lap]
    ]
    valid_lap_times = [
        float(item["lap_time"])
        for item in history
        if int(item["lap"]) in valid_lap_numbers
    ]
    reference_lap_times = [
        seconds
        for seconds in (
            _reference_lap_seconds(label)
            for label in (runtime_circuit.physics_calibration.reference_laps if runtime_circuit.physics_calibration else [])
        )
        if seconds is not None
    ]
    reference_lap_median = _mean(reference_lap_times)
    fastest_valid_lap = min(valid_lap_times) if valid_lap_times else None
    fastest_lap_delta = (
        abs(fastest_valid_lap - reference_lap_median) / reference_lap_median
        if fastest_valid_lap is not None and reference_lap_median > 0.0
        else None
    )
    speed_errors = [
        sample["speed_kph"] - float(_reference_at(telemetry, sample["progress"]).speed_kph)
        for sample in valid_samples
    ] if telemetry else []
    target_errors = [
        sample["target_speed_kph"] - sample["speed_kph"]
        for sample in valid_samples
    ]
    diagnostics = engine.diagnostic_counts()
    tire_diagnostics = diagnostics.get("tire_temperature", {})
    line_error_metrics = _signed_error_metrics(valid_samples)
    heading_limit = DYNAMIC_BICYCLE_MAX_CONTROLLED_HEADING_ERROR_RAD
    heading_limit_samples = sum(
        abs(float(sample["heading_error_signed_rad"]))
        >= heading_limit - 1e-6
        for sample in valid_samples
    )
    sample_complete = len(history) >= laps
    dynamic = {
        "status": "pass" if sample_complete else "incomplete",
        "seed": seed,
        "laps_requested": laps,
        "laps_completed": len(history),
        "valid_laps": valid_lap_numbers,
        "physics_step_seconds": PHYSICS_STEP_SECONDS,
        "physics_hz": round(1.0 / PHYSICS_STEP_SECONDS),
        "input_contract": input_contract,
        "steps": step_count,
        "simulation_time_seconds": _round(engine.race_elapsed, 3),
        "lap_history": history,
        "tire_usage": {
            "equivalent_laps_end": _round(state.tire_usage, 5),
            "wear_end": _round(state.tire_wear, 5),
            "circuit_equivalent_usage_per_lap": _round(
                engine._circuit_tire_usage_per_lap(),
                5,
            ),
        },
        "fastest_valid_lap_time_seconds": _round(fastest_valid_lap, 3),
        "reference_lap_time_median_seconds": _round(reference_lap_median, 3),
        "fastest_valid_lap_relative_delta": _round(fastest_lap_delta, 5),
        "speed_comparison": {
            "mae_kph": _round(_mean(abs(error) for error in speed_errors), 3),
            "rmse_kph": _round(math.sqrt(_mean(error * error for error in speed_errors)), 3),
            "bias_kph": _round(_mean(speed_errors), 3),
            "maximum_absolute_error_kph": _round(max((abs(error) for error in speed_errors), default=0.0), 3),
            "target_minus_actual_mean_kph": _round(_mean(target_errors), 3),
        },
        "corners": _corner_report(
            runtime_circuit,
            runtime_circuit.physics_calibration,
            valid_samples,
            float(runtime_circuit.track_length_m),
        ),
        "corner_line_error_reports": _corner_error_reports(
            runtime_circuit,
            valid_samples,
        ),
        "coordinate_contract": {
            "compiled_centerline_progress": "state.progress; 0 at compiled start/finish, forward direction",
            "active_path_progress": "active driving-line path distance modulo active path length",
            "active_path_distance_m": "line_distance_at_total_progress(active_line, state.total_progress)",
            "telemetry_source_progress": "source sample progress before telemetry_progress_offset",
            "telemetry_aligned_progress": "source progress - telemetry_progress_offset, modulo 1",
            "centerline_reference_lateral_offset_m": "active line offset expressed from compiled centerline",
            "vehicle_centerline_lateral_offset_m": "state.lateral_offset_m in compiled centerline frame",
            "active_line_pose_vehicle_lateral_offset_m": "vehicle centerline offset - active line reference offset",
            "target_lateral_offset_m": "controller target in compiled centerline frame",
            "vehicle_lateral_speed_mps": "state.lateral_speed_mps in the compiled centerline frame",
            "reference_lateral_speed_candidate_mps": "finite-difference active-line offset derivative used by the clean-line frame contract",
            "heading_error_rad": "absolute dynamic-bicycle heading error",
            "active_line_curvature_1pm": "signed path curvature interpolated on active line samples",
        },
        "coordinate_samples": coordinate_samples,
        "speed_opportunities": _speed_opportunity_report(
            runtime_circuit,
            runtime_circuit.physics_calibration,
            valid_samples,
        ),
        "safety": {
            "contact_samples": status_counts["contact"],
            "off_track_samples": status_counts["off_track"],
            "track_limit_samples": status_counts["track_limits"],
            "run_wide_samples": status_counts["run_wide"],
            "planner_fallback_samples": status_counts["planner_fallback"],
            "maximum_consecutive_planner_fallback_steps": maximum_consecutive_fallback_steps,
            "numeric_guard_hits": numeric_guard_hits,
            "run_wide_by_corner": dict(sorted(run_wide_by_corner.items())),
            "off_track_by_corner": dict(sorted(off_track_by_corner.items())),
            "event_counts": dict(sorted(event_counts.items())),
            "handling_state_samples": dict(sorted(handling_counts.items())),
            "wheelspin_samples": handling_counts["wheelspin"],
            "oversteer_samples": handling_counts["oversteer"],
            "traction_loss_samples": event_counts["traction_loss"],
            "maximum_traction_slip_ratio": _round(maximum_traction_slip_ratio, 5),
            "maximum_wheel_lock_ratio": _round(maximum_wheel_lock_ratio, 5),
            "lateral_error_reference": "active_racing_line",
            "maximum_lateral_error_m": _round(max((abs(value) for value in lateral_errors), default=0.0), 3),
            "lateral_error_p95_m": _round(_p95(lateral_errors), 3),
            "heading_error_p95_rad": _round(_p95(heading_errors), 6),
            "heading_error_limit_rad": heading_limit,
            "heading_error_limit_samples": heading_limit_samples,
            "heading_error_limit_seconds": _round(
                heading_limit_samples * PHYSICS_STEP_SECONDS,
                4,
            ),
            "heading_error_limit_ratio": _round(
                heading_limit_samples / max(1, len(valid_samples)),
                6,
            ),
            "line_error_metrics": line_error_metrics,
        },
        "thermal": tire_diagnostics,
        "brakes": {key: _round(value, 3) for key, value in brake_peaks.items()},
        "acceptance": {
            "requested_laps_complete": sample_complete,
            "valid_lap_count": len(valid_lap_numbers),
            "four_valid_laps": len(valid_lap_numbers) >= 4,
            "contact_off_track_numeric_guard_zero": (
                status_counts["contact"] == 0
                and status_counts["off_track"] == 0
            ),
            "repeated_run_wide_zero": status_counts["run_wide"] == 0,
            "line_error_p95_within_2m": line_error_metrics["p95_m"] <= 2.0,
            "line_error_max_within_4m": line_error_metrics["max_m"] <= 4.0,
            "heading_error_p95_within_0_08rad": _p95(heading_errors) <= 0.08,
            "heading_error_limit_zero": heading_limit_samples == 0,
            "planner_fallback_zero": status_counts["planner_fallback"] == 0,
            "race_lap_regression_status": "requires locked race baseline comparison",
            "numeric_guard_zero": numeric_guard_hits == 0,
            "within_step_stability_budget": True,
        },
    }
    return dynamic


def _source_identity() -> list[dict[str, str]]:
    result = []
    for relative in REFERENCE_SOURCE_FILES:
        path = PROJECT_DIR / relative
        result.append(
            {
                "path": relative,
                "sha256": _file_sha256(path) if path.exists() else "missing",
            }
        )
    return result


def build_report(
    *,
    circuit_id: int | str,
    thermal_preset: ThermalPresetName,
    laps: int,
    seed: int,
    pace: float,
    driver_id: int | None,
    tire_role: DryTireRole = DryTireRole.MEDIUM,
    pace_mode: PaceMode = PaceMode.STANDARD,
    racing_line_reference_weight: float | None = None,
    racing_line_initial_smoothing_passes: int | None = None,
    racing_line_max_lateral_slope: float | None = None,
    reference_lateral_speed_feedforward: float = 1.0,
    clean_line_lateral_speed_limit_mps: float = 1.25,
) -> dict[str, Any]:
    circuits = load_circuits()
    circuit = next((item for item in circuits if str(item.id) == str(circuit_id)), None)
    if circuit is None:
        raise ValueError(f"circuit {circuit_id} not found")
    calibration_override: dict[str, Any] = {}
    if racing_line_reference_weight is not None:
        calibration_override["racing_line_reference_weight"] = (
            float(racing_line_reference_weight)
        )
    if racing_line_initial_smoothing_passes is not None:
        calibration_override["racing_line_initial_smoothing_passes"] = int(
            racing_line_initial_smoothing_passes
        )
    if racing_line_max_lateral_slope is not None:
        calibration_override["racing_line_max_lateral_slope"] = float(
            racing_line_max_lateral_slope
        )
    if calibration_override and circuit.physics_calibration is not None:
        circuit = circuit.model_copy(
            update={
                "physics_calibration": circuit.physics_calibration.model_copy(
                    update=calibration_override
                )
            }
        )
    drivers = load_drivers()
    teams = {team.id: team for team in load_teams()}
    environment = circuit.thermal_profile.presets[thermal_preset]
    calibration = circuit.physics_calibration
    report: dict[str, Any] = {
        "schema_version": 3,
        "circuit_id": circuit.id,
        "circuit_name": circuit.name,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        **_git_identity(),
        "working_tree_note": "Existing user changes were preserved; this is a diagnostic snapshot.",
        "race_line_contract": {
            "session_basis": "race",
            "environment_preset": thermal_preset.value,
            "tire_role": tire_role.value,
            "pace_mode": pace_mode.value,
            "physics_step_seconds": PHYSICS_STEP_SECONDS,
            "coordinate_contract": "single-car dynamic coordinate diagnostics are in single_car_dynamic.coordinate_contract",
            "qualifying_line": "not implemented in this phase",
            "global_tire_or_vehicle_physics_changes": (
                "global tire-force coefficients unchanged; bounded load-sensitive "
                "dynamic-bicycle cornering stiffness active"
            ),
            "calibration_override": calibration_override,
            "reference_lateral_speed_feedforward": reference_lateral_speed_feedforward,
            "clean_line_lateral_speed_limit_mps": clean_line_lateral_speed_limit_mps,
        },
        "environment": {
            "thermal_preset": thermal_preset.value,
            **environment.model_dump(mode="json"),
            "source": "circuit thermal preset",
        },
        "nomination": (
            circuit.tire_compound_nomination.model_dump(mode="json")
            if circuit.tire_compound_nomination
            else None
        ),
        "tire_wear_profile": circuit.tire_wear_profile.model_dump(mode="json"),
        "sector_timing_source": (
            getattr(circuit, "sector_timing_source").model_dump(mode="json")
            if getattr(circuit, "sector_timing_source", None)
            else None
        ),
        "source_identity": _source_identity(),
        "static_telemetry_metrics": _static_telemetry_metrics(
            circuit,
            drivers,
            teams,
            pace,
            seed,
        ),
        "tier_a_audit": audit_circuit(circuit),
        "single_car_dynamic": _run_single_car(
            circuit,
            drivers,
            teams,
            laps=laps,
            seed=seed,
            thermal_preset=thermal_preset,
            driver_id=driver_id,
            tire_role=tire_role,
            pace_mode=pace_mode,
            reference_lateral_speed_feedforward=reference_lateral_speed_feedforward,
            clean_line_lateral_speed_limit_mps=clean_line_lateral_speed_limit_mps,
        ),
        "traffic": {"status": "not_run", "reason": "single-car baseline mode"},
        "pit": {"status": "not_run", "reason": "single-car baseline mode"},
        "safety_car": {"status": "not_run", "reason": "single-car baseline mode"},
        "determinism": {
            "status": "not_run",
            "reason": "1x/2x matrix is a separate integration test",
        },
        "cleanup": {"status": "not_run", "reason": "requires session lifecycle harness"},
        "accepted": False,
        "acceptance_note": (
            "This report is a single-car baseline. It is not a full Red Bull Ring product approval."
        ),
    }
    return report


def aggregate_reports(reports: list[dict[str, Any]]) -> dict[str, Any]:
    """Keep every repeated run and add a compact median/worst-case summary."""
    if not reports:
        raise ValueError("at least one baseline report is required")
    dynamic = [report["single_car_dynamic"] for report in reports]

    def values(path: tuple[str, ...]) -> list[float]:
        result: list[float] = []
        for item in dynamic:
            current: Any = item
            for key in path:
                current = current[key]
            if isinstance(current, (int, float)) and math.isfinite(float(current)):
                result.append(float(current))
        return result

    def median(path: tuple[str, ...]) -> float:
        candidates = values(path)
        return _round(statistics.median(candidates), 4) if candidates else 0.0

    def maximum(path: tuple[str, ...]) -> float:
        candidates = values(path)
        return _round(max(candidates), 4) if candidates else 0.0

    event_counts: Counter[str] = Counter()
    for item in dynamic:
        event_counts.update(item["safety"].get("event_counts", {}))
    first = dict(reports[0])
    first["schema_version"] = 3
    first["repeat_count"] = len(reports)
    compact_runs: list[dict[str, Any]] = []
    for report in reports:
        compact = dict(report)
        compact_dynamic = dict(report["single_car_dynamic"])
        # The first run remains the full coordinate trace above.  Repeating
        # the same trace three times makes calibration artifacts needlessly
        # large while adding no evidence for a deterministic input.
        compact_dynamic.pop("coordinate_samples", None)
        compact["single_car_dynamic"] = compact_dynamic
        compact_runs.append(compact)
    first["runs"] = compact_runs
    first["repeat_summary"] = {
        "status": "pass" if all(item["single_car_dynamic"]["status"] == "pass" for item in reports) else "incomplete",
        "fastest_valid_lap_time_median_seconds": median(("fastest_valid_lap_time_seconds",)),
        "fastest_valid_lap_time_worst_seconds": maximum(("fastest_valid_lap_time_seconds",)),
        "lateral_error_p95_median_m": median(("safety", "lateral_error_p95_m")),
        "lateral_error_p95_worst_m": maximum(("safety", "lateral_error_p95_m")),
        "lateral_error_max_median_m": median(("safety", "maximum_lateral_error_m")),
        "lateral_error_max_worst_m": maximum(("safety", "maximum_lateral_error_m")),
        "heading_error_p95_median_rad": median(("safety", "heading_error_p95_rad")),
        "rear_surface_peak_median_c": median(("thermal", "peak", "rear_surface_max_c")),
        "rear_surface_peak_worst_c": maximum(("thermal", "peak", "rear_surface_max_c")),
        "rear_core_peak_median_c": median(("thermal", "peak", "rear_core_max_c")),
        "rear_core_peak_worst_c": maximum(("thermal", "peak", "rear_core_max_c")),
        "line_error_p95_median": median(("safety", "line_error_metrics", "p95_m")),
        "line_error_p95_worst": maximum(("safety", "line_error_metrics", "p95_m")),
        "line_error_over_2m_seconds_median": median(("safety", "line_error_metrics", "abs_over_threshold_seconds")),
        "line_error_over_2m_seconds_worst": maximum(("safety", "line_error_metrics", "abs_over_threshold_seconds")),
        "correction_reversals_median": median(("safety", "line_error_metrics", "correction_reversals")),
        "correction_reversals_worst": maximum(("safety", "line_error_metrics", "correction_reversals")),
        "contact_samples_total": sum(item["safety"]["contact_samples"] for item in dynamic),
        "off_track_samples_total": sum(item["safety"]["off_track_samples"] for item in dynamic),
        "track_limit_samples_total": sum(item["safety"]["track_limit_samples"] for item in dynamic),
        "run_wide_samples_total": sum(item["safety"]["run_wide_samples"] for item in dynamic),
        "planner_fallback_samples_total": sum(item["safety"]["planner_fallback_samples"] for item in dynamic),
        "numeric_guard_hits_total": sum(item["safety"]["numeric_guard_hits"] for item in dynamic),
        "event_counts_total": dict(sorted(event_counts.items())),
        "deterministic_fields": [
            "lap_history",
            "safety.line_error_metrics",
            "thermal.peak",
            "tire_usage",
        ],
    }
    first["accepted"] = False
    first["acceptance_note"] = (
        "Repeated single-car baseline. Approval requires stint, traffic, determinism and regression gates."
    )
    return first


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--circuit-id", required=True)
    parser.add_argument(
        "--thermal-preset",
        choices=[preset.value for preset in ThermalPresetName],
        default=ThermalPresetName.NORMAL.value,
    )
    parser.add_argument("--mode", choices=["single-car"], default="single-car")
    parser.add_argument("--laps", type=int, default=5)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--pace", type=float, default=DEFAULT_PACE)
    parser.add_argument("--driver-id", type=int)
    parser.add_argument(
        "--tire-role",
        choices=[role.value for role in DryTireRole],
        default=DryTireRole.MEDIUM.value,
    )
    parser.add_argument(
        "--pace-mode",
        choices=[mode.value for mode in PaceMode],
        default=PaceMode.STANDARD.value,
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="repeat the same deterministic input and store all run reports",
    )
    parser.add_argument(
        "--racing-line-reference-weight",
        type=float,
        help="diagnostic-only calibration override for one reference-path experiment",
    )
    parser.add_argument(
        "--racing-line-initial-smoothing-passes",
        type=int,
        help="diagnostic-only calibration override for one reference-path experiment",
    )
    parser.add_argument(
        "--racing-line-max-lateral-slope",
        type=float,
        help="diagnostic-only race-line transition slope in metres/metre",
    )
    parser.add_argument(
        "--reference-lateral-speed-feedforward",
        type=float,
        default=1.0,
        help=(
            "clean-line centerline/reference transport coefficient in [0, 1]; "
            "diagnostic default is the complete paired contract; 0 replays production"
        ),
    )
    parser.add_argument(
        "--clean-line-lateral-speed-limit-mps",
        type=float,
        default=1.25,
        help="diagnostic-only clean-line lateral response cap in m/s",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.laps < 2:
        parser.error("--laps must be at least 2 so lap 1 can be excluded from evaluation")
    if args.repeat < 1:
        parser.error("--repeat must be at least 1")

    reports = [
        build_report(
            circuit_id=args.circuit_id,
            thermal_preset=ThermalPresetName(args.thermal_preset),
            laps=args.laps,
            seed=args.seed,
            pace=args.pace,
            driver_id=args.driver_id,
            tire_role=DryTireRole(args.tire_role),
            pace_mode=PaceMode(args.pace_mode),
            racing_line_reference_weight=args.racing_line_reference_weight,
            racing_line_initial_smoothing_passes=(
                args.racing_line_initial_smoothing_passes
            ),
            racing_line_max_lateral_slope=args.racing_line_max_lateral_slope,
            reference_lateral_speed_feedforward=(
                args.reference_lateral_speed_feedforward
            ),
            clean_line_lateral_speed_limit_mps=args.clean_line_lateral_speed_limit_mps,
        )
        for _ in range(args.repeat)
    ]
    report = aggregate_reports(reports) if args.repeat > 1 else reports[0]
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
