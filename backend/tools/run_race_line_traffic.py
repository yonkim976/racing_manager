#!/usr/bin/env python3
"""Run a fixed-step 20-car race-line traffic approval sample.

This is intentionally a direct RaceEngine harness.  It keeps the 0.02 second
physics step authoritative while recording tactical-line transitions, return
to the clean line, safety counters, and thermal diagnostics for a bounded
multi-car stint.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

BACKEND_DIR = Path(__file__).resolve().parents[1]
if str(BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(BACKEND_DIR))

from data_loader import load_circuits, load_drivers, load_teams
from models.schemas import DryTireRole, PaceMode, ThermalPresetName, TireCompound
from simulation.race_engine import RaceEngine
from simulation.track_physics import DRIVING_LINE_RACING


PHYSICS_STEP_SECONDS = 0.02


def _digest_step(states: list[Any]) -> str:
    digest = hashlib.sha256()
    for state in sorted(states, key=lambda item: item.driver_id):
        digest.update(
            (
                f"{state.driver_id}:{state.current_lap}:{state.progress:.9f}:"
                f"{state.lateral_offset_m:.6f}:{state.racing_line}:"
                f"{state.finished}:{state.retired}\n"
            ).encode()
        )
    return digest.hexdigest()


def run_traffic(
    *,
    circuit_id: int,
    laps: int,
    seed: int,
    thermal_preset: ThermalPresetName,
) -> dict[str, Any]:
    base_circuit = next(item for item in load_circuits() if item.id == circuit_id)
    circuit = base_circuit.model_copy(update={"total_laps": laps})
    drivers = load_drivers()
    teams = {team.id: team for team in load_teams()}
    conditions = circuit.thermal_profile.presets[thermal_preset]
    engine = RaceEngine(
        circuit=circuit,
        drivers=drivers,
        teams=teams,
        player_team_id=drivers[0].team_id,
        player_driver_ids=[driver.id for driver in drivers if driver.team_id == 1],
        starting_tires={driver.id: TireCompound(DryTireRole.MEDIUM.value) for driver in drivers},
        seed=seed,
        start_sequence_enabled=False,
        track_conditions=conditions,
        thermal_preset=thermal_preset,
        track_conditions_source="circuit_preset",
    )

    safety = Counter()
    handling = Counter()
    event_counts = Counter()
    line_counts = Counter()
    maximum_fallback_steps = 0
    fallback_steps_by_driver: Counter[int] = Counter()
    current_fallback_steps: Counter[int] = Counter()
    maximum_lateral_step_m = 0.0
    maximum_lateral_step_driver_id: int | None = None
    previous_lateral: dict[int, float] = {}
    previous_line: dict[int, str] = {}
    tactical_line_transitions = 0
    clean_line_rejoins = 0
    maximum_rejoin_seconds = 0.0
    tactical_started_at: dict[int, float] = {}
    digest = hashlib.sha256()
    step_count = 0
    maximum_steps = max(
        1,
        int(laps * circuit.base_lap_time / PHYSICS_STEP_SECONDS * 3.0),
    )

    while not engine.finished and step_count < maximum_steps:
        events = engine.tick(PHYSICS_STEP_SECONDS)
        step_count += 1
        event_counts.update(event.type for event in events)
        states = list(engine.driver_states.values())
        digest.update(_digest_step(states).encode())
        for state in states:
            driver_id = state.driver_id
            line = state.racing_line or DRIVING_LINE_RACING
            line_counts[line] += 1
            handling[state.handling_state] += 1
            safety["contact_samples"] += int(state.contact_active)
            safety["off_track_samples"] += int(state.off_track)
            safety["track_limit_samples"] += int(state.track_limits_active)
            safety["run_wide_samples"] += int(state.handling_state == "run_wide")
            safety["planner_fallback_samples"] += int(state.planner_fallback_active)
            safety["wheelspin_samples"] += int(state.handling_state == "wheelspin")
            safety["oversteer_samples"] += int(state.handling_state == "oversteer")
            safety["traction_loss_samples"] += int(
                state.handling_state == "traction_loss"
            )
            previous = previous_lateral.get(driver_id)
            if previous is not None:
                step_m = abs(float(state.lateral_offset_m) - previous)
                if step_m > maximum_lateral_step_m:
                    maximum_lateral_step_m = step_m
                    maximum_lateral_step_driver_id = driver_id
            previous_lateral[driver_id] = float(state.lateral_offset_m)

            old_line = previous_line.get(driver_id)
            if old_line is not None and old_line != line:
                tactical_line_transitions += 1
                if line != DRIVING_LINE_RACING:
                    tactical_started_at[driver_id] = engine.race_elapsed
                elif driver_id in tactical_started_at:
                    clean_line_rejoins += 1
                    maximum_rejoin_seconds = max(
                        maximum_rejoin_seconds,
                        engine.race_elapsed - tactical_started_at.pop(driver_id),
                    )
            previous_line[driver_id] = line

            if state.planner_fallback_active:
                current_fallback_steps[driver_id] += 1
                fallback_steps_by_driver[driver_id] += 1
                maximum_fallback_steps = max(
                    maximum_fallback_steps,
                    current_fallback_steps[driver_id],
                )
            else:
                current_fallback_steps[driver_id] = 0

    diagnostics = engine.diagnostic_counts()
    thermal = diagnostics.get("tire_temperature", {})
    per_driver = {
        str(state.driver_id): {
            "finished": bool(state.finished),
            "retired": bool(state.retired),
            "current_lap": int(state.current_lap),
            "total_progress": round(float(state.total_progress), 6),
            "line": state.racing_line,
        }
        for state in sorted(engine.driver_states.values(), key=lambda item: item.driver_id)
    }
    finished_count = sum(item["finished"] for item in per_driver.values())
    passed = (
        len(drivers) == 20
        and engine.finished
        and finished_count == len(drivers)
        and safety["contact_samples"] == 0
        and safety["off_track_samples"] == 0
        and safety["track_limit_samples"] == 0
        and safety["planner_fallback_samples"] == 0
        and thermal.get("peak", {}).get("max_continuous_overheat_seconds", 0.0) == 0.0
    )
    return {
        "circuit_id": circuit_id,
        "circuit_name": circuit.name,
        "seed": seed,
        "laps_requested": laps,
        "physics_step_seconds": PHYSICS_STEP_SECONDS,
        "physics_hz": round(1.0 / PHYSICS_STEP_SECONDS),
        "driver_count": len(drivers),
        "step_count": step_count,
        "simulation_time_seconds": round(engine.race_elapsed, 3),
        "engine_finished": bool(engine.finished),
        "finished_driver_count": finished_count,
        "retired_driver_count": sum(item["retired"] for item in per_driver.values()),
        "per_driver": per_driver,
        "line_samples": dict(sorted(line_counts.items())),
        "tactical_line_transitions": tactical_line_transitions,
        "clean_line_rejoins": clean_line_rejoins,
        "maximum_rejoin_seconds": round(maximum_rejoin_seconds, 3),
        "maximum_lateral_step_m": round(maximum_lateral_step_m, 4),
        "maximum_lateral_step_driver_id": maximum_lateral_step_driver_id,
        "maximum_consecutive_planner_fallback_steps": maximum_fallback_steps,
        "fallback_steps_by_driver": dict(sorted(fallback_steps_by_driver.items())),
        "safety": dict(sorted(safety.items())),
        "handling_samples": dict(sorted(handling.items())),
        "event_counts": dict(sorted(event_counts.items())),
        "thermal": thermal,
        "deterministic_digest": digest.hexdigest(),
        "passed": passed,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--circuit-id", type=int, choices=(3, 4), required=True)
    parser.add_argument("--laps", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--thermal-preset",
        choices=[item.value for item in ThermalPresetName],
        default=ThermalPresetName.NORMAL.value,
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.laps < 10:
        parser.error("--laps must be at least 10 for the traffic approval gate")
    result = run_traffic(
        circuit_id=args.circuit_id,
        laps=args.laps,
        seed=args.seed,
        thermal_preset=ThermalPresetName(args.thermal_preset),
    )
    rendered = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
