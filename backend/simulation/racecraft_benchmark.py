"""Deterministic Bahrain racecraft scenarios and explainable AI metrics."""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from math import cos, sin, sqrt
from typing import Any, Iterable

from data_loader import load_circuits, load_drivers, load_teams
from simulation.race_engine import ManeuverGroup, RaceEngine
from simulation.track_geometry import TRACK_GEOMETRY_MODE
from simulation.track_physics import DRIVING_LINE_RACING
from simulation.vehicle_physics import PHYSICS_STEP_SECONDS


@dataclass(frozen=True)
class BahrainRacecraftScenario:
    name: str
    progress: float
    center_gap_m: float
    attacker_speed_kph: float
    defender_speed_kph: float
    expected_decision: str
    expected_reason_code: str


BAHRAIN_RACECRAFT_SCENARIOS: tuple[BahrainRacecraftScenario, ...] = (
    BahrainRacecraftScenario(
        "main_straight_attack",
        0.030,
        12.0,
        310.0,
        295.0,
        "attack",
        "straight_window_open",
    ),
    BahrainRacecraftScenario(
        "t1_late_braking_attack",
        0.115,
        9.0,
        255.0,
        248.0,
        "attack",
        "heavy_braking_window_open",
    ),
    BahrainRacecraftScenario(
        "middle_technical_hold",
        0.400,
        9.0,
        205.0,
        202.0,
        "hold",
        "segment_disallows_attack",
    ),
    BahrainRacecraftScenario(
        "t10_distant_braking_hold",
        0.520,
        12.0,
        230.0,
        225.0,
        "hold",
        "braking_gap_too_large",
    ),
    BahrainRacecraftScenario(
        "t11_t12_prepare_next_straight",
        0.690,
        10.0,
        235.0,
        232.0,
        "hold",
        "segment_disallows_attack",
    ),
    BahrainRacecraftScenario(
        "t13_t14_straight_attack",
        0.830,
        12.0,
        304.0,
        296.0,
        "attack",
        "straight_window_open",
    ),
    BahrainRacecraftScenario(
        "final_braking_attack",
        0.900,
        9.0,
        285.0,
        278.0,
        "attack",
        "heavy_braking_window_open",
    ),
)


def _make_bahrain_engine(seed: int, driver_count: int = 2) -> RaceEngine:
    drivers = load_drivers()[:driver_count]
    return RaceEngine(
        circuit=next(circuit for circuit in load_circuits() if circuit.id == 3),
        drivers=drivers,
        teams={team.id: team for team in load_teams()},
        player_team_id=drivers[0].team_id,
        player_driver_ids=[drivers[0].id],
        seed=seed,
        start_sequence_enabled=False,
    )


def _prepare_overtake_scenario(
    scenario: BahrainRacecraftScenario,
    seed: int,
) -> tuple[RaceEngine, int, int]:
    engine = _make_bahrain_engine(seed)
    defender, attacker = sorted(
        engine.driver_states.values(),
        key=lambda state: state.position,
    )
    defender_total = 1.0 + scenario.progress
    engine._set_state_total_progress(defender, defender_total)
    engine._set_state_total_progress(
        attacker,
        defender_total - scenario.center_gap_m / engine.track_length_m,
    )
    defender.speed_kph = scenario.defender_speed_kph
    attacker.speed_kph = scenario.attacker_speed_kph
    defender.lateral_offset_m = 0.0
    attacker.lateral_offset_m = 0.0
    defender.target_lateral_offset_m = 0.0
    attacker.target_lateral_offset_m = 0.0
    return engine, attacker.driver_id, defender.driver_id


def _run_overtake_scenario(
    scenario: BahrainRacecraftScenario,
    seed: int,
    duration_seconds: float,
) -> dict[str, Any]:
    engine, attacker_id, defender_id = _prepare_overtake_scenario(scenario, seed)
    assessment = engine.explain_overtake_decision(attacker_id, defender_id)
    event_counts: Counter[str] = Counter()
    maximum_lateral_separation_m = 0.0
    steps = max(0, round(max(0.0, duration_seconds) / PHYSICS_STEP_SECONDS))
    for _ in range(steps):
        for event in engine.tick(PHYSICS_STEP_SECONDS):
            event_counts[event.type] += 1
        attacker = engine.driver_states[attacker_id]
        defender = engine.driver_states[defender_id]
        maximum_lateral_separation_m = max(
            maximum_lateral_separation_m,
            abs(attacker.lateral_offset_m - defender.lateral_offset_m),
        )

    attacker = engine.driver_states[attacker_id]
    defender = engine.driver_states[defender_id]
    payload = asdict(assessment)
    payload.update(
        {
            "scenario": scenario.name,
            "seed": seed,
            "expected_decision": scenario.expected_decision,
            "expected_reason_code": scenario.expected_reason_code,
            "decision_match": assessment.decision == scenario.expected_decision,
            "reason_match": assessment.reason_code == scenario.expected_reason_code,
            "event_counts": dict(sorted(event_counts.items())),
            "attacker_finished_ahead": (
                attacker.total_progress > defender.total_progress
            ),
            "maximum_lateral_separation_m": round(
                maximum_lateral_separation_m,
                6,
            ),
            "contact": attacker.contact_active or defender.contact_active,
            "off_track": attacker.off_track or defender.off_track,
        }
    )
    for key, value in tuple(payload.items()):
        if isinstance(value, float):
            payload[key] = round(value, 6)
    return payload


def _pit_merge_audit(seed: int) -> dict[str, Any]:
    """Reproduce YIELD -> HOLD -> MERGE against a main-track group."""
    engine = _make_bahrain_engine(seed, driver_count=3)
    pit_car, traffic, group_mate = sorted(
        engine.driver_states.values(),
        key=lambda state: state.position,
    )
    pit_lane = engine.circuit.pit_lane
    assert pit_lane is not None
    lane_progress = pit_lane.speed_limit_end + 0.01
    speed_mps = 30.0
    route_length_m = engine._pit_route_length_m()
    remaining_m = (pit_lane.side_rejoin_progress - lane_progress) * route_length_m
    time_to_rejoin_s = remaining_m / speed_mps
    entry = engine._pit_entry_progress()
    exit_ = engine._pit_exit_progress()
    assert entry is not None and exit_ is not None
    mapped_total = (
        pit_car.current_lap
        + entry
        + engine._progress_distance(entry, exit_) * pit_lane.side_rejoin_progress
    )
    track_progress = mapped_total % 1.0
    pit_x, pit_y, _ = engine._pit_lane_pose_at_progress_m(
        pit_lane.side_rejoin_progress
    )
    profile = engine._track_physics_for_driver(pit_car)
    line_length_m = profile.length_for_line(DRIVING_LINE_RACING)
    for _ in range(4):
        line_x, line_y, heading = profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            track_progress,
        )
        tangent_m = (
            (pit_x - line_x) * cos(heading)
            + (pit_y - line_y) * sin(heading)
        )
        track_progress = (track_progress + tangent_m / line_length_m) % 1.0
    target_total = int(mapped_total) + track_progress
    line_x, line_y, heading = profile.line_pose_at_progress_m(
        DRIVING_LINE_RACING,
        track_progress,
    )
    traffic.lateral_offset_m = profile.line_offset_at_progress(
        DRIVING_LINE_RACING,
        track_progress,
    ) + (
        (pit_x - line_x) * -sin(heading)
        + (pit_y - line_y) * cos(heading)
    )
    traffic.speed_kph = 80.0 * 3.6
    traffic.total_progress = (
        target_total - 80.0 * time_to_rejoin_s / engine.track_length_m
    )
    group_mate.lateral_offset_m = traffic.lateral_offset_m + 3.0
    group_mate.speed_kph = traffic.speed_kph
    group_mate.total_progress = target_total + 200.0 / engine.track_length_m
    engine._maneuver_groups = {
        "mg-benchmark-pit-traffic": ManeuverGroup(
            group_id="mg-benchmark-pit-traffic",
            member_ids=(traffic.driver_id, group_mate.driver_id),
            pair_keys=(),
            phase="overlap",
            minimum_lateral_m=traffic.lateral_offset_m,
            maximum_lateral_m=group_mate.lateral_offset_m,
        )
    }

    yielded = engine._pit_rejoin_decision(
        pit_car.driver_id,
        pit_car,
        lane_progress,
        speed_mps,
    )
    hold_progress = pit_lane.side_rejoin_progress - 4.0 / route_length_m
    hold_time_s = sqrt(
        2.0
        * (pit_lane.side_rejoin_progress - hold_progress)
        * route_length_m
        / 9.0
    )
    traffic.total_progress = target_total - 80.0 * hold_time_s / engine.track_length_m
    held = engine._pit_rejoin_decision(
        pit_car.driver_id,
        pit_car,
        hold_progress,
        0.0,
    )
    traffic.total_progress = target_total + 100.0 / engine.track_length_m
    merged = engine._pit_rejoin_decision(
        pit_car.driver_id,
        pit_car,
        hold_progress,
        0.0,
    )
    sequence = [yielded.state, held.state, merged.state]
    return {
        "seed": seed,
        "decision_sequence": sequence,
        "expected_sequence": ["yield", "hold", "merge"],
        "sequence_match": sequence == ["yield", "hold", "merge"],
        "conflict_driver_id": yielded.conflict_driver_id,
        "conflict_group_id": yielded.conflict_group_id,
        "conflict_group_member_ids": list(yielded.conflict_group_member_ids),
    }


def run_bahrain_racecraft_benchmark(
    seeds: Iterable[int] = range(10),
    duration_seconds: float = 2.0,
) -> dict[str, Any]:
    """Run the fixed scenario matrix and return a JSON-serializable report."""
    seed_values = tuple(int(seed) for seed in seeds)
    results = [
        _run_overtake_scenario(scenario, seed, duration_seconds)
        for seed in seed_values
        for scenario in BAHRAIN_RACECRAFT_SCENARIOS
    ]
    pit_results = [_pit_merge_audit(seed) for seed in seed_values]
    by_scenario: dict[str, dict[str, Any]] = {}
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for result in results:
        grouped[result["scenario"]].append(result)
    for scenario, items in grouped.items():
        by_scenario[scenario] = {
            "runs": len(items),
            "decision_match_rate": round(
                sum(item["decision_match"] for item in items) / len(items),
                6,
            ),
            "reason_match_rate": round(
                sum(item["reason_match"] for item in items) / len(items),
                6,
            ),
            "contact_runs": sum(item["contact"] for item in items),
            "off_track_runs": sum(item["off_track"] for item in items),
            "pass_runs": sum(item["attacker_finished_ahead"] for item in items),
        }

    total = max(1, len(results))
    summary = {
        "scenario_runs": len(results),
        "decision_match_rate": round(
            sum(item["decision_match"] for item in results) / total,
            6,
        ),
        "reason_match_rate": round(
            sum(item["reason_match"] for item in results) / total,
            6,
        ),
        "contact_runs": sum(item["contact"] for item in results),
        "off_track_runs": sum(item["off_track"] for item in results),
        "pit_merge_sequence_match_rate": round(
            sum(item["sequence_match"] for item in pit_results)
            / max(1, len(pit_results)),
            6,
        ),
    }
    passed = (
        summary["decision_match_rate"] == 1.0
        and summary["reason_match_rate"] == 1.0
        and summary["contact_runs"] == 0
        and summary["off_track_runs"] == 0
        and summary["pit_merge_sequence_match_rate"] == 1.0
    )
    return {
        "benchmark_version": "bahrain-racecraft-v1",
        "track_geometry_mode": TRACK_GEOMETRY_MODE,
        "circuit": "Bahrain International Circuit",
        "duration_seconds_per_scenario": duration_seconds,
        "seeds": list(seed_values),
        "status": "pass" if passed else "fail",
        "summary": summary,
        "scenario_summary": by_scenario,
        "pit_merge_results": pit_results,
        "results": results,
    }
