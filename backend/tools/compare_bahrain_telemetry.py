#!/usr/bin/env python3
"""Compare Bahrain's physical controller with measured qualifying telemetry.

The raw TracingInsights laps are intentionally supplied at execution time and
are not vendored into the project.  This tool reduces multiple clean laps into
a compact distance-normalized reference, evaluates the physical target-speed
controller, and can write a reproducible calibration report.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Callable

BACKEND_DIR = Path(__file__).resolve().parents[1]
if str(BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(BACKEND_DIR))

from data_loader import load_circuits, load_drivers, load_teams
from simulation.race_engine import RaceEngine
from simulation.track_physics import DRIVING_LINE_RACING
from simulation.vehicle_physics import (
    LongitudinalVehiclePhysics,
    VehiclePhysicsModifiers,
)

SOURCE_URL = "https://github.com/TracingInsights-Archive/2025"
REFERENCE_LABELS = (
    "PIA lap 14 1:29.841",
    "RUS lap 19 1:30.009",
    "LEC lap 15 1:30.175",
    "ANT lap 19 1:30.213",
    "GAS lap 18 1:30.216",
)
QUALIFYING_TELEMETRY_PACE = 1.04


def _channel_at(telemetry: dict[str, Any], progress: float, channel: str) -> float:
    distances = telemetry["rel_distance"]
    values = telemetry[channel]
    target = progress % 1.0
    low = 0
    high = len(distances) - 1
    while low + 1 < high:
        middle = (low + high) // 2
        if float(distances[middle]) <= target:
            low = middle
        else:
            high = middle
    following = min(len(distances) - 1, low + 1)
    span = max(1e-9, float(distances[following]) - float(distances[low]))
    ratio = max(
        0.0,
        min(1.0, (target - float(distances[low])) / span),
    )
    return float(values[low]) + (float(values[following]) - float(values[low])) * ratio


def _load_laps(paths: list[Path]) -> list[dict[str, Any]]:
    laps: list[dict[str, Any]] = []
    for path in paths:
        payload = json.loads(path.read_text(encoding="utf-8"))
        telemetry = payload["tel"]
        required = {"rel_distance", "speed", "throttle", "brake"}
        missing = required - telemetry.keys()
        if missing:
            raise ValueError(f"{path} is missing telemetry channels: {sorted(missing)}")
        laps.append(telemetry)
    if len(laps) < 3:
        raise ValueError("at least three independent reference laps are required")
    return laps


def _measured_samples(
    laps: list[dict[str, Any]],
    count: int,
) -> list[dict[str, float]]:
    samples: list[dict[str, float]] = []
    for index in range(count):
        progress = index / count
        samples.append(
            {
                "progress": round(progress, 6),
                "speed_kph": round(
                    statistics.median(
                        _channel_at(lap, progress, "speed") for lap in laps
                    ),
                    3,
                ),
                "throttle_percent": round(
                    statistics.median(
                        _channel_at(lap, progress, "throttle") for lap in laps
                    ),
                    3,
                ),
                "braking_fraction": round(
                    sum(
                        _channel_at(lap, progress, "brake") >= 0.5
                        for lap in laps
                    )
                    / len(laps),
                    3,
                ),
            }
        )
    return samples


def _physical_controller() -> tuple[
    LongitudinalVehiclePhysics,
    Callable[[float], float],
    float,
    float,
]:
    circuit = next(item for item in load_circuits() if item.id == 3)
    drivers = load_drivers()[:1]
    teams = {team.id: team for team in load_teams()}
    engine = RaceEngine(
        circuit=circuit,
        drivers=drivers,
        teams=teams,
        player_team_id=drivers[0].team_id,
        player_driver_ids=[drivers[0].id],
        seed=42,
        start_sequence_enabled=False,
    )
    state = engine.driver_states[drivers[0].id]
    physics = engine._vehicle_physics_for_driver(state)[DRIVING_LINE_RACING]
    track_profile = engine._track_physics_for_driver(state)
    calibration = circuit.physics_calibration
    return (
        physics,
        lambda progress: track_profile.line_distance_at_total_progress(
            DRIVING_LINE_RACING,
            progress,
        ),
        calibration.planner_braking_utilization if calibration else 1.0,
        calibration.controller_sample_distance_m if calibration else 50.0,
    )


def _predicted_speeds(
    physics: LongitudinalVehiclePhysics,
    progresses: list[float],
    distance_at_progress: Callable[[float], float],
) -> list[float]:
    # The reference is a median of fastest qualifying laps.  Compare against
    # the physical qualifying envelope rather than the 0.98 standard-race
    # driver utilization baked into a neutral pace modifier.
    modifiers = VehiclePhysicsModifiers(pace=QUALIFYING_TELEMETRY_PACE)
    return [
        physics.target_speed_mps(distance_at_progress(progress), modifiers) * 3.6
        for progress in progresses
    ]


def _errors(reference: list[float], predicted: list[float]) -> dict[str, float]:
    deltas = [candidate - measured for measured, candidate in zip(reference, predicted)]
    return {
        "mae_kph": round(statistics.fmean(abs(value) for value in deltas), 3),
        "rmse_kph": round(
            math.sqrt(statistics.fmean(value * value for value in deltas)),
            3,
        ),
        "bias_kph": round(statistics.fmean(deltas), 3),
        "maximum_over_kph": round(max(deltas), 3),
        "maximum_under_kph": round(min(deltas), 3),
    }


def _candidate_controller(
    base: LongitudinalVehiclePhysics,
    braking_utilization: float,
    sample_distance_m: float,
) -> LongitudinalVehiclePhysics:
    return LongitudinalVehiclePhysics(
        base.profile,
        base.track_length_m,
        base.reference_lap_time,
        planner_braking_utilization=braking_utilization,
        controller_sample_distance_m=sample_distance_m,
        controller_speed_scale_floor=base.controller_speed_scale_floor,
        telemetry_speed_reference_weight=base.telemetry_speed_reference_weight,
        telemetry_braking_curvature_threshold=(
            base.telemetry_braking_curvature_threshold
        ),
        telemetry_max_braking_utilization=(
            base.telemetry_max_braking_utilization
        ),
        telemetry_braking_speed_reserve=(
            base.telemetry_braking_speed_reserve
        ),
        braking_longitudinal_grip_factor=(
            base.braking_longitudinal_grip_factor
        ),
        brake_control_error_fraction=base.brake_control_error_fraction,
    )


def _find_controller_calibration(
    base: LongitudinalVehiclePhysics,
    samples: list[dict[str, float]],
    distance_at_progress: Callable[[float], float],
) -> dict[str, float]:
    progresses = [sample["progress"] for sample in samples]
    measured = [sample["speed_kph"] for sample in samples]
    best: tuple[float, float, float, dict[str, float]] | None = None
    # A static target-profile comparison cannot prove that a late-braking
    # candidate remains inside the circuit once force integration and lateral
    # dynamics are active.  Keep the dynamically validated braking utilization
    # and only optimize the controller's spatial sampling resolution here.
    for braking_utilization in (base.planner_braking_utilization,):
        for sample_distance_m in (base.controller_sample_distance_m,):
            candidate = _candidate_controller(
                base,
                braking_utilization,
                sample_distance_m,
            )
            errors = _errors(
                measured,
                _predicted_speeds(candidate, progresses, distance_at_progress),
            )
            score = errors["rmse_kph"] + abs(errors["bias_kph"]) * 0.25
            item = (score, braking_utilization, sample_distance_m, errors)
            if best is None or item[:3] < best[:3]:
                best = item
    assert best is not None
    return {
        "planner_braking_utilization": best[1],
        "telemetry_max_braking_utilization": (
            base.telemetry_max_braking_utilization
        ),
        "braking_longitudinal_grip_factor": (
            base.braking_longitudinal_grip_factor
        ),
        "brake_control_error_fraction": base.brake_control_error_fraction,
        "controller_sample_distance_m": best[2],
        **best[3],
    }


def _corner_report(
    corners_path: Path,
    track_length_m: float,
    samples: list[dict[str, float]],
    predicted: list[float],
) -> list[dict[str, float | int]]:
    corners = json.loads(corners_path.read_text(encoding="utf-8"))
    count = len(samples)
    window_samples = max(1, round(110.0 / track_length_m * count))
    reports: list[dict[str, float | int]] = []
    for number, distance in zip(corners["CornerNumber"], corners["Distance"]):
        progress = (float(distance) / track_length_m) % 1.0
        center = round(progress * count) % count
        indices = [
            (center + delta) % count
            for delta in range(-window_samples, window_samples + 1)
        ]
        measured_min = min(float(samples[index]["speed_kph"]) for index in indices)
        predicted_min = min(predicted[index] for index in indices)
        brake_fraction = max(
            float(samples[index]["braking_fraction"]) for index in indices
        )
        reports.append(
            {
                "corner": int(number),
                "progress": round(progress, 6),
                "measured_min_speed_kph": round(measured_min, 3),
                "predicted_min_speed_kph": round(predicted_min, 3),
                "minimum_speed_error_kph": round(predicted_min - measured_min, 3),
                "measured_braking_fraction": round(brake_fraction, 3),
            }
        )
    return reports


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--telemetry", type=Path, nargs="+", required=True)
    parser.add_argument("--corners", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=128)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--update-circuit",
        type=Path,
        help="write the compact reference and recommended controller values to circuits.json",
    )
    args = parser.parse_args()

    laps = _load_laps(args.telemetry)
    samples = _measured_samples(laps, max(32, args.samples))
    (
        physics,
        distance_at_progress,
        current_braking,
        current_sample_distance,
    ) = _physical_controller()
    progresses = [sample["progress"] for sample in samples]
    measured = [sample["speed_kph"] for sample in samples]
    predicted = _predicted_speeds(physics, progresses, distance_at_progress)
    recommendation = _find_controller_calibration(
        physics,
        samples,
        distance_at_progress,
    )
    recommended_physics = _candidate_controller(
        physics,
        recommendation["planner_braking_utilization"],
        recommendation["controller_sample_distance_m"],
    )
    recommended_predicted = _predicted_speeds(
        recommended_physics,
        progresses,
        distance_at_progress,
    )
    report = {
        "source": "TracingInsights-Archive/2025",
        "source_url": SOURCE_URL,
        "reference_session": "2025 Bahrain Grand Prix Qualifying",
        "reference_laps": list(REFERENCE_LABELS[: len(laps)]),
        "sample_count": len(samples),
        "current_calibration": {
            "planner_braking_utilization": current_braking,
            "telemetry_max_braking_utilization": (
                physics.telemetry_max_braking_utilization
            ),
            "braking_longitudinal_grip_factor": (
                physics.braking_longitudinal_grip_factor
            ),
            "brake_control_error_fraction": physics.brake_control_error_fraction,
            "controller_sample_distance_m": current_sample_distance,
            **_errors(measured, predicted),
        },
        "recommended_calibration": recommendation,
        "corner_comparison": _corner_report(
            args.corners,
            physics.track_length_m,
            samples,
            recommended_predicted,
        ),
        "telemetry_reference": samples,
    }
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    if args.update_circuit:
        circuits_payload = json.loads(args.update_circuit.read_text(encoding="utf-8"))
        circuit_payload = next(
            item for item in circuits_payload if int(item["id"]) == 3
        )
        calibration_payload = circuit_payload.setdefault("physics_calibration", {})
        calibration_payload["telemetry_reference"] = samples
        calibration_payload["telemetry_speed_reference_weight"] = 1.0
        calibration_payload["planner_braking_utilization"] = recommendation[
            "planner_braking_utilization"
        ]
        calibration_payload["controller_sample_distance_m"] = recommendation[
            "controller_sample_distance_m"
        ]
        calibration_payload["telemetry_max_braking_utilization"] = recommendation[
            "telemetry_max_braking_utilization"
        ]
        calibration_payload["braking_longitudinal_grip_factor"] = recommendation[
            "braking_longitudinal_grip_factor"
        ]
        calibration_payload["brake_control_error_fraction"] = recommendation[
            "brake_control_error_fraction"
        ]
        args.update_circuit.write_text(
            json.dumps(circuits_payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
