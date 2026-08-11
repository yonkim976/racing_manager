"""Measure the Stage 4 shared traffic cursor without retaining a replay."""

from __future__ import annotations

import argparse
import asyncio
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
import resource
import subprocess
import sys
import tempfile
import time
from unittest.mock import AsyncMock, patch
from collections import Counter
from math import hypot
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_loader import load_circuits, load_drivers, load_teams  # noqa: E402
from models.schemas import TrackConditions  # noqa: E402
from simulation.abstract import AbstractRaceEngine, AbstractSessionSnapshot  # noqa: E402
from simulation.abstract.broadcast import AbstractBroadcastSession  # noqa: E402
from simulation.abstract.race import AbstractTrafficSimulationCursor  # noqa: E402
from simulation.abstract.racecraft import audit_accepted_frame_reservations  # noqa: E402
from simulation.abstract.state import canonical_json  # noqa: E402


DT = 0.10
MAX_SPEED_MPS = 370.0 / 3.6
MAX_LATERAL_SPEED_MPS = 8.0
MAX_LATERAL_ACCELERATION_MPS2 = 20.0


class _DiagnosticWebSocket:
    def __init__(self) -> None:
        self.messages: list[dict[str, Any]] = []
        self.closed = False

    async def send_json(self, message: dict[str, Any]) -> None:
        self.messages.append(message)

    async def close(self, **_: Any) -> None:
        self.closed = True


def _peak_rss_bytes() -> int:
    value = int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    return value if sys.platform == "darwin" else value * 1024


def _ordered_hash(items: list[dict[str, Any]]) -> str:
    return hashlib.sha256(
        canonical_json(items).encode("utf-8")
    ).hexdigest()


def _one_run(circuit, seed: int, *, total_laps: int = 10) -> dict[str, Any]:
    drivers = load_drivers()
    teams = load_teams()
    snapshot = AbstractSessionSnapshot.from_content(
        session_id=f"stage4-diagnostic:{circuit.id}:{seed}:{total_laps}",
        session_seed=seed,
        circuit=circuit,
        drivers=drivers,
        teams=teams,
    )
    cursor = AbstractTrafficSimulationCursor(snapshot, total_laps=total_laps, tick_seconds=DT)
    previous: dict[int | str, Any] = {}
    max_speed = 0.0
    min_acceleration = 0.0
    max_acceleration = 0.0
    max_lateral_speed = 0.0
    max_lateral_acceleration = 0.0
    min_boundary_clearance = float("inf")
    boundary_violations = 0
    max_tick_displacement = 0.0
    tick_displacement_violations = 0
    integrator_distance_error_max = 0.0
    integrator_identity_violations = 0
    reported_acceleration_identity_violations = 0
    world_jump_max = 0.0
    overlap_count = 0
    frame_count = 0
    lap_wrap_ok = True
    first_three_launch: list[dict[str, Any]] = []
    pose_digest = hashlib.sha256(b"abstract-broadcast-full-field-pose-v1\0")
    independent_reservation_conflicts = 0
    independent_third_conflicts = 0
    independent_body_violations = 0

    def inspect(frame) -> None:
        nonlocal max_speed, min_acceleration, max_acceleration
        nonlocal max_lateral_speed, max_lateral_acceleration, min_boundary_clearance
        nonlocal boundary_violations, max_tick_displacement, tick_displacement_violations
        nonlocal integrator_distance_error_max, integrator_identity_violations
        nonlocal reported_acceleration_identity_violations, world_jump_max, overlap_count
        nonlocal frame_count, lap_wrap_ok
        nonlocal independent_reservation_conflicts, independent_third_conflicts
        nonlocal independent_body_violations
        frame_count += 1
        encoded_frame = canonical_json(frame.to_dict()).encode("utf-8")
        pose_digest.update(len(encoded_frame).to_bytes(8, "big"))
        pose_digest.update(encoded_frame)
        audit = audit_accepted_frame_reservations(
            frame,
            cursor.active_reservations,
            line_length_m=cursor.distance_contract.line_length_m,
            vehicle_length_m=cursor.geometry.car_length_m,
            vehicle_width_m=cursor.geometry.car_width_m,
            vehicle_target_speed_mps_by_id={
                driver_id: cursor._forecast_target_speed_mps(
                    cursor.cars[driver_id],
                    max(
                        (
                            reservation.expiry_time_s - reservation.start_time_s
                            for reservation in cursor.active_reservations
                        ),
                        default=0.0,
                    ),
                )
                for driver_id in sorted(cursor.cars, key=str)
            },
        )
        independent_reservation_conflicts += audit["admitted_reservation_conflict_count"]
        independent_third_conflicts += audit["third_vehicle_occupancy_conflict_count"]
        independent_body_violations += audit["reservation_body_clearance_violation_count"]
        by_id = {item.driver_id: item for item in frame.vehicles}
        for vehicle in frame.vehicles:
            max_speed = max(max_speed, vehicle.speed_mps)
            min_acceleration = min(min_acceleration, vehicle.longitudinal_acceleration_mps2)
            max_acceleration = max(max_acceleration, vehicle.longitudinal_acceleration_mps2)
            limits = cursor.distance_contract.lateral_limits_at_progress(
                vehicle.total_progress,
                car_width_m=cursor.geometry.car_width_m,
            )
            clearance = min(vehicle.lateral_offset_m - limits[0], limits[1] - vehicle.lateral_offset_m)
            min_boundary_clearance = min(min_boundary_clearance, clearance)
            if clearance < -1e-8:
                boundary_violations += 1
            old = previous.get(vehicle.driver_id)
            if old is not None:
                ds = vehicle.line_distance_m - old.line_distance_m
                identity_distance = (old.speed_mps + vehicle.speed_mps) * 0.5 * DT
                identity_acceleration = (vehicle.speed_mps - old.speed_mps) / DT
                distance_error = abs(ds - identity_distance)
                integrator_distance_error_max = max(integrator_distance_error_max, distance_error)
                if distance_error > 1e-7:
                    integrator_identity_violations += 1
                if abs(vehicle.longitudinal_acceleration_mps2 - identity_acceleration) > 1e-7:
                    reported_acceleration_identity_violations += 1
                lateral_speed = abs(vehicle.lateral_velocity_mps)
                lateral_acceleration = abs(vehicle.lateral_acceleration_mps2)
                max_lateral_speed = max(max_lateral_speed, lateral_speed)
                max_lateral_acceleration = max(max_lateral_acceleration, lateral_acceleration)
                jump = hypot(vehicle.world_x_m - old.world_x_m, vehicle.world_y_m - old.world_y_m)
                world_jump_max = max(world_jump_max, jump)
                max_tick_displacement = max(max_tick_displacement, jump)
                if jump > old.speed_mps * DT + 0.5 + 1e-7:
                    tick_displacement_violations += 1
                if vehicle.total_progress < old.total_progress - 1e-9:
                    lap_wrap_ok = False
            previous[vehicle.driver_id] = vehicle
        for ahead, behind in zip(frame.vehicles, frame.vehicles[1:]):
            if (
                ahead.line_distance_m - behind.line_distance_m < cursor.geometry.car_length_m
                and abs(ahead.lateral_offset_m - behind.lateral_offset_m) < cursor.geometry.car_width_m + 0.15
            ):
                overlap_count += 1
        if frame.logical_time_s <= 3.0 and frame.vehicles:
            first_three_launch.append(
                {
                    "tick": frame.tick_index,
                    "time_s": frame.logical_time_s,
                    "leader_speed_mps": frame.vehicles[0].speed_mps,
                    "leader_progress": frame.vehicles[0].total_progress,
                    "leader_lateral_offset_m": frame.vehicles[0].lateral_offset_m,
                }
            )

    inspect(cursor.current_frame)
    while not cursor.finished:
        inspect(cursor.advance_one_tick().frame)
    result = cursor.finalize()
    event_counts = Counter(event.event_type for event in result.logical_events)
    starts = [event for event in result.logical_events if event.event_type == "attack_started"]
    attack_counts_by_driver_lap: Counter[tuple[int | str, int]] = Counter()
    for event in starts:
        payload = dict(event.payload)
        lap_number = int(payload.get("lap_number", 1))
        for driver_id in event.driver_ids[:1]:
            attack_counts_by_driver_lap[(driver_id, lap_number)] += 1
    attack_frequency_values = [
        attack_counts_by_driver_lap[(driver_id, lap_number)]
        for driver_id in sorted(cursor.cars, key=str)
        for lap_number in range(1, total_laps + 1)
    ]
    sorted_attack_frequency = sorted(attack_frequency_values)
    p95_index = min(
        len(sorted_attack_frequency) - 1,
        int(0.95 * len(sorted_attack_frequency)),
    )
    by_pair: dict[tuple[int | str, ...], float] = {}
    cooldown_intervals: list[float] = []
    for event in starts:
        pair = tuple(event.driver_ids)
        if pair in by_pair:
            cooldown_intervals.append(event.logical_time_s - by_pair[pair])
        by_pair[pair] = event.logical_time_s
    crossing_ids = {
        event.event_id.rsplit(":", 1)[0]
        for event in result.logical_events
        if event.event_type == "overtake_crossing_confirmed"
    }
    completed_without_clearance = sum(
        event.event_id.rsplit(":", 1)[0] not in crossing_ids
        for event in result.logical_events
        if event.event_type == "overtake_completed"
    )
    cursor_metrics = dict(cursor.metrics)
    driver_count = len(cursor.cars)
    terminal_ids = Counter(
        event.event_id.rsplit(":", 1)[0]
        for event in result.logical_events
        if event.event_type in {"overtake_completed", "defense_hold", "attack_aborted"}
    )
    terminal_missing = max(0, event_counts["attack_started"] - len(terminal_ids))
    terminal_duplicates = sum(max(0, count - 1) for count in terminal_ids.values())
    event_hash = hashlib.sha256(
        json.dumps(
            [event.to_dict() for event in result.logical_events],
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    ).hexdigest()
    checkpoint_hash = hashlib.sha256(
        json.dumps(
            [checkpoint.to_dict() for checkpoint in result.timing_checkpoints],
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    ).hexdigest()
    cursor.dispose()
    result_payload = {
        "circuit_id": circuit.id,
        "seed": seed,
        "total_laps": total_laps,
        "frame_count_diagnostic_only": frame_count,
        "pose_hash": pose_digest.hexdigest(),
        "event_hash": event_hash,
        "checkpoint_hash": checkpoint_hash,
        "logical_tick_count": result.logical_tick_count,
        "max_speed_mps": max_speed,
        "max_speed_kph": max_speed * 3.6,
        "min_acceleration_mps2": min_acceleration,
        "max_acceleration_mps2": max_acceleration,
        "max_lateral_speed_mps": max_lateral_speed,
        "max_lateral_acceleration_mps2": max_lateral_acceleration,
        "minimum_boundary_clearance_m": min_boundary_clearance,
        "boundary_violation_count": boundary_violations,
        "maximum_tick_displacement_m": max_tick_displacement,
        "tick_displacement_violation_count": tick_displacement_violations,
        "integrator_distance_error_max_m": integrator_distance_error_max,
        "integrator_identity_violation_count": integrator_identity_violations,
        "reported_acceleration_identity_violation_count": reported_acceleration_identity_violations,
        "post_integrator_distance_correction_count": cursor_metrics["post_integrator_distance_correction_count"],
        "world_chord_precommit_replan_count": cursor_metrics[
            "world_chord_precommit_replan_count"
        ],
        "crossing_before_rank_swap_count": cursor_metrics["crossing_before_rank_swap_count"],
        "body_overlap_count": overlap_count,
        "corridor_conflict_count": cursor_metrics["corridor_conflict_count"],
        "independent_reservation_conflict_count": independent_reservation_conflicts,
        "independent_third_vehicle_conflict_count": independent_third_conflicts,
        "independent_reservation_body_clearance_violation_count": independent_body_violations,
        "third_vehicle_conflict_rejection_count": cursor_metrics[
            "third_vehicle_conflict_rejection_count"
        ],
        "reservation_conflict_rejection_count": cursor_metrics[
            "reservation_conflict_rejection_count"
        ],
        "corridor_rejection_reason_counts": cursor_metrics[
            "corridor_rejection_reason_counts"
        ],
        "clearance_without_pass_count": completed_without_clearance,
        "attack_start_count": event_counts["attack_started"],
        "pass_count": event_counts["overtake_completed"],
        "defense_count": event_counts["defense_hold"],
        "abort_count": event_counts["attack_aborted"],
        "contact_count": event_counts["contact_started"],
        "terminal_event_missing_count": terminal_missing,
        "terminal_event_duplicate_count": terminal_duplicates,
        "finish_count": len(result.finish_order),
        "cooldown_min_s": min(cooldown_intervals) if cooldown_intervals else None,
        "cooldown_violation_count": sum(value < 8.0 - 1e-9 for value in cooldown_intervals),
        "attack_start_average_per_driver_lap": (
            len(starts) / max(1, driver_count * total_laps)
        ),
        "attack_start_max_per_driver_lap": max(attack_frequency_values, default=0),
        "attack_start_p95_per_driver_lap": (
            sorted_attack_frequency[p95_index] if sorted_attack_frequency else 0
        ),
        "contact_average_per_race_lap": event_counts["contact_started"] / max(1, total_laps),
        "lap_wrap_ok": lap_wrap_ok,
        "canonical_result_hash": result.canonical_result_hash,
        "grid_order": list(result.grid_order),
        "finish_order": list(result.finish_order),
        "launch_first_3s": first_three_launch,
    }
    result_payload["approval_passed"] = all(
        (
            max_speed <= MAX_SPEED_MPS + 1e-9,
            max_acceleration <= 18.0 + 1e-9,
            min_acceleration >= -50.0 - 1e-9,
            max_lateral_speed <= MAX_LATERAL_SPEED_MPS + 1e-9,
            max_lateral_acceleration <= MAX_LATERAL_ACCELERATION_MPS2 + 1e-9,
            boundary_violations == 0,
            tick_displacement_violations == 0,
            integrator_identity_violations == 0,
            reported_acceleration_identity_violations == 0,
            result_payload["post_integrator_distance_correction_count"] == 0,
            result_payload["crossing_before_rank_swap_count"] == 0,
            overlap_count == 0,
            independent_reservation_conflicts == 0,
            independent_third_conflicts == 0,
            independent_body_violations == 0,
            terminal_missing == 0,
            terminal_duplicates == 0,
            completed_without_clearance == 0,
            result_payload["finish_count"] == len(snapshot.entries),
            result_payload["cooldown_violation_count"] == 0,
            result_payload["attack_start_average_per_driver_lap"] <= 3.0 + 1e-9,
            result_payload["contact_average_per_race_lap"] <= 2.0 + 1e-9,
        )
    )
    result_payload["_authority_result"] = result
    return result_payload


def _broadcast_run(
    circuit,
    seed: int,
    *,
    total_laps: int,
    speed_multiplier: int,
    authority_result: Any | None = None,
) -> dict[str, Any]:
    """Run one controlled-clock full-field broadcast without wall-time waits."""

    drivers = load_drivers()
    teams = load_teams()
    snapshot = AbstractSessionSnapshot.from_content(
        session_id=f"stage4-diagnostic:{circuit.id}:{seed}:{total_laps}",
        session_seed=seed,
        circuit=circuit,
        drivers=drivers,
        teams=teams,
    )
    result = authority_result or AbstractRaceEngine(snapshot).run_race(
        total_laps=total_laps,
        tick_seconds=DT,
    )
    session = AbstractBroadcastSession(
        snapshot=snapshot,
        result=result,
        circuit=circuit,
        player_team=teams[0],
        player_drivers=[driver for driver in drivers if driver.team_id == teams[0].id],
        drivers=drivers,
        teams=teams,
        track_conditions=TrackConditions(),
        thermal_preset=None,
        conditions_source="stage4-matrix",
    )
    socket = _DiagnosticWebSocket()
    started = time.perf_counter()
    rss_before = _peak_rss_bytes()

    async def run() -> None:
        await session.add_client(socket)
        await session.handle_command({"type": "set_speed", "multiplier": speed_multiplier})
        with patch.object(session.replay_clock, "wait", new=AsyncMock(return_value=True)):
            await session.start_loop()
            if session._loop_task is not None:
                await session._loop_task

    asyncio.run(run())
    elapsed_ms = (time.perf_counter() - started) * 1_000.0
    rss_after = _peak_rss_bytes()
    counts = session.diagnostic_counts()
    pose_messages = [message for message in socket.messages if message.get("type") == "pose_tick"]
    event_ids = [
        event["event_id"]
        for message in socket.messages
        if message.get("type") == "race_events"
        for event in message.get("events", [])
    ]
    event_hash = _ordered_hash([event.to_dict() for event in result.logical_events])
    checkpoint_hash = _ordered_hash(
        [checkpoint.to_dict() for checkpoint in result.timing_checkpoints]
    )
    result_event_counts = Counter(event.event_type for event in result.logical_events)
    message_frame_ids = [message["physics_frame"] for message in pose_messages]
    frame_missing = sum(
        max(0, current - previous - 1)
        for previous, current in zip(message_frame_ids, message_frame_ids[1:])
    )
    frame_duplicates = sum(
        current <= previous
        for previous, current in zip(message_frame_ids, message_frame_ids[1:])
    )
    summary = {
        "mode": f"Broadcast {speed_multiplier}x",
        "circuit_id": circuit.id,
        "seed": seed,
        "total_laps": total_laps,
        "logical_tick_count": result.logical_tick_count,
        "canonical_result_hash": result.canonical_result_hash,
        "event_hash": event_hash,
        "checkpoint_hash": checkpoint_hash,
        "pose_hash": counts["broadcast_pose_hash"],
        "grid_order": list(result.grid_order),
        "finish_order": list(result.finish_order),
        "event_count": len(event_ids),
        "attack_start_count": result_event_counts["attack_started"],
        "pass_count": result_event_counts["overtake_completed"],
        "defense_count": result_event_counts["defense_hold"],
        "abort_count": result_event_counts["attack_aborted"],
        "contact_count": result_event_counts["contact_started"],
        "event_duplicate_count": len(event_ids) - len(set(event_ids)),
        "frame_count": len(pose_messages),
        "frame_missing_count": frame_missing,
        "frame_duplicate_count": frame_duplicates,
        "frame_driver_missing_or_duplicate_count": sum(
            len(message.get("poses", [])) != 20
            or len({pose.get("driver_id") for pose in message.get("poses", [])}) != 20
            for message in pose_messages
        ),
        "max_frame_gap": counts["max_frame_gap"],
        "max_logical_time_gap": counts["max_logical_time_gap"],
        "position_speed_violation_count": counts["position_speed_violation_count"],
        "max_position_derived_speed_mps": counts["max_position_derived_speed_mps"],
        "traffic_corridor_conflict_count": counts["traffic_corridor_conflict_count"],
        "traffic_admitted_reservation_conflict_count": counts["traffic_admitted_reservation_conflict_count"],
        "traffic_third_vehicle_occupancy_conflict_count": counts["traffic_third_vehicle_occupancy_conflict_count"],
        "traffic_reservation_body_clearance_violation_count": counts["traffic_reservation_body_clearance_violation_count"],
        "traffic_same_lane_longitudinal_overlap_count": counts["traffic_same_lane_longitudinal_overlap_count"],
        "traffic_attack_terminal_event_count": counts["traffic_attack_terminal_event_count"],
        "traffic_attack_terminal_event_duplicate_count": counts["traffic_attack_terminal_event_duplicate_count"],
        "race_end_count": sum(message.get("type") == "race_end" for message in socket.messages),
        "event_order": event_ids,
        "runtime_ms": round(elapsed_ms, 3),
        "peak_rss_delta_bytes": max(0, rss_after - rss_before),
        "retained_frame_count_before_close": counts["frame_count"],
        "retained_pose_count_before_close": counts["frame_count"] * 20,
        "frame_buffer_capacity": counts["frame_capacity"],
        "retained_frame_count_after_close": None,
        "retained_pose_count_after_close": None,
        "loop_task_after_close": None,
        "traffic_cursor_after_close": None,
        "client_count_after_close": None,
    }
    session_result_hash = session.result.canonical_result_hash
    asyncio.run(session.close())
    after_close = session.diagnostic_counts()
    summary.update(
        {
            "retained_frame_count_after_close": after_close["frame_count"],
            "retained_pose_count_after_close": after_close["retained_pose_count_after_close"],
            "loop_task_after_close": after_close["loop_task_exists"],
            "traffic_cursor_after_close": after_close["traffic_cursor_exists"],
            "client_count_after_close": after_close["client_count"],
            "session_result_hash": session_result_hash,
        }
    )
    summary["approval_passed"] = all(
        (
            session_result_hash == result.canonical_result_hash,
            event_ids == [event.event_id for event in result.logical_events],
            message_frame_ids == list(range(0, result.logical_tick_count + 1)),
            summary["event_duplicate_count"] == 0,
            summary["frame_missing_count"] == 0,
            summary["frame_duplicate_count"] == 0,
            summary["frame_driver_missing_or_duplicate_count"] == 0,
            counts["missing_pose_count"] == 0,
            counts["duplicate_pose_count"] == 0,
            counts["max_frame_gap"] == 1,
            abs(counts["max_logical_time_gap"] - DT) <= 1e-9,
            counts["position_speed_violation_count"] == 0,
            counts["max_position_derived_speed_mps"] <= MAX_SPEED_MPS + 1e-9,
            counts["traffic_corridor_conflict_count"] == 0,
            counts["traffic_admitted_reservation_conflict_count"] == 0,
            counts["traffic_third_vehicle_occupancy_conflict_count"] == 0,
            counts["traffic_reservation_body_clearance_violation_count"] == 0,
            counts["traffic_same_lane_longitudinal_overlap_count"] == 0,
            counts["traffic_attack_terminal_event_duplicate_count"] == 0,
            summary["race_end_count"] == 1,
            summary["retained_frame_count_before_close"] <= 128,
            summary["retained_frame_count_after_close"] == 0,
            summary["retained_pose_count_after_close"] == 0,
            summary["loop_task_after_close"] is False,
            summary["traffic_cursor_after_close"] is False,
            summary["client_count_after_close"] == 0,
        )
    )
    return summary


PARITY_FIELDS = (
    "logical_tick_count",
    "canonical_result_hash",
    "event_hash",
    "checkpoint_hash",
    "pose_hash",
    "grid_order",
    "finish_order",
)

ZERO_GUARD_FIELDS = (
    "boundary_violation_count",
    "tick_displacement_violation_count",
    "integrator_identity_violation_count",
    "reported_acceleration_identity_violation_count",
    "post_integrator_distance_correction_count",
    "crossing_before_rank_swap_count",
    "body_overlap_count",
    "corridor_conflict_count",
    "independent_reservation_conflict_count",
    "independent_third_vehicle_conflict_count",
    "independent_reservation_body_clearance_violation_count",
    "clearance_without_pass_count",
    "terminal_event_missing_count",
    "terminal_event_duplicate_count",
    "cooldown_violation_count",
    "event_duplicate_count",
    "frame_missing_count",
    "frame_duplicate_count",
    "frame_driver_missing_or_duplicate_count",
    "position_speed_violation_count",
    "traffic_corridor_conflict_count",
    "traffic_admitted_reservation_conflict_count",
    "traffic_third_vehicle_occupancy_conflict_count",
    "traffic_reservation_body_clearance_violation_count",
    "traffic_same_lane_longitudinal_overlap_count",
    "traffic_attack_terminal_event_duplicate_count",
)


def _aggregate_diagnostic_payload(
    runs: list[dict[str, Any]],
    *,
    circuits: list[int],
    seeds: list[int],
    laps: int,
    modes: tuple[str, ...] | list[str],
) -> dict[str, Any]:
    """Build the complete Stage 4 artifact from the run records.

    Keeping this aggregation beside the run loop makes the committed JSON
    reproducible by one command; no external merge or notebook post-process
    is part of the diagnostic contract.
    """

    for run in runs:
        if "max_speed_kph" not in run and "max_speed_mps" in run:
            run["max_speed_kph"] = run["max_speed_mps"] * 3.6

    parity_failures: list[dict[str, Any]] = []
    by_key: dict[tuple[int, int], dict[str, dict[str, Any]]] = {}
    for run in runs:
        by_key.setdefault((run["circuit_id"], run["seed"]), {})[run["mode"]] = run
    for key, mode_runs in sorted(by_key.items()):
        instant = mode_runs.get("Instant")
        if instant is None:
            continue
        for mode, run in sorted(mode_runs.items()):
            if mode == "Instant":
                continue
            mismatches = {
                field: {"instant": instant[field], "broadcast": run[field]}
                for field in PARITY_FIELDS
                if instant[field] != run[field]
            }
            run["parity_passed"] = not mismatches
            if mismatches:
                parity_failures.append(
                    {
                        "circuit_id": key[0],
                        "seed": key[1],
                        "mode": mode,
                        "mismatches": mismatches,
                    }
                )

    zero_guard_failures = []
    for index, run in enumerate(runs):
        failures = {
            field: run[field]
            for field in ZERO_GUARD_FIELDS
            if field in run and run[field] not in (0, False, None)
        }
        if failures:
            zero_guard_failures.append(
                {
                    "run_index": index,
                    "circuit_id": run["circuit_id"],
                    "seed": run["seed"],
                    "mode": run["mode"],
                    "fields": failures,
                }
            )

    reasons: dict[str, int] = {}
    for run in runs:
        for reason, count in run.get("corridor_rejection_reason_counts", {}).items():
            reasons[reason] = max(reasons.get(reason, 0), int(count))
    broadcast_runs = [run for run in runs if run["mode"].startswith("Broadcast")]
    activity_guard_required = (
        laps >= 10
        and len(set(seeds)) >= 10
        and "Instant" in modes
    )
    circuit_activity: dict[int, dict[str, int]] = {}
    for circuit_id in circuits:
        instant_runs = [
            run
            for run in runs
            if run["mode"] == "Instant" and run["circuit_id"] == circuit_id
        ]
        circuit_activity[circuit_id] = {
            "run_count": len(instant_runs),
            "attack_start_count": sum(
                int(run.get("attack_start_count", 0)) for run in instant_runs
            ),
            "pass_count": sum(int(run.get("pass_count", 0)) for run in instant_runs),
        }
    production_activity_failures = (
        [
            {
                "circuit_id": circuit_id,
                **counts,
            }
            for circuit_id, counts in sorted(circuit_activity.items())
            if (
                counts["run_count"] != len(set(seeds))
                or counts["attack_start_count"] <= 0
                or counts["pass_count"] <= 0
            )
        ]
        if activity_guard_required
        else []
    )
    resources = {
        "measured_on_broadcast_modes": sorted(
            {run["mode"] for run in broadcast_runs}
        ),
        "runtime_ms": {
            "min": min((run["runtime_ms"] for run in broadcast_runs), default=0.0),
            "max": max((run["runtime_ms"] for run in broadcast_runs), default=0.0),
            "mean": (
                sum(run["runtime_ms"] for run in broadcast_runs) / len(broadcast_runs)
                if broadcast_runs
                else 0.0
            ),
        },
        "peak_rss_delta_bytes": {
            "max": max(
                (run["peak_rss_delta_bytes"] for run in broadcast_runs),
                default=0,
            ),
            "mean": (
                sum(run["peak_rss_delta_bytes"] for run in broadcast_runs) / len(broadcast_runs)
                if broadcast_runs
                else 0.0
            ),
        },
        "retained_frame_count_before_close_max": max(
            (run["retained_frame_count_before_close"] for run in broadcast_runs),
            default=0,
        ),
        "retained_pose_count_before_close_max": max(
            (run["retained_pose_count_before_close"] for run in broadcast_runs),
            default=0,
        ),
        "frame_buffer_capacity": sorted(
            {run["frame_buffer_capacity"] for run in broadcast_runs}
        ),
        "cleanup_all_runs": all(
            run.get("retained_frame_count_after_close") == 0
            and run.get("retained_pose_count_after_close") == 0
            and run.get("loop_task_after_close") is False
            and run.get("traffic_cursor_after_close") is False
            and run.get("client_count_after_close") == 0
            for run in broadcast_runs
        ),
    }
    quality = {
        "run_count": len(runs),
        "expected_run_count": len(set(circuits)) * len(set(seeds)) * len(set(modes)),
        "approval_run_count": sum(bool(run.get("approval_passed")) for run in runs),
        "zero_guard_failures": zero_guard_failures,
        "max_max_speed_kph": max((run.get("max_speed_kph", 0.0) for run in runs), default=0.0),
        "max_max_acceleration_mps2": max((run.get("max_acceleration_mps2", 0.0) for run in runs), default=0.0),
        "min_min_acceleration_mps2": min((run.get("min_acceleration_mps2", 0.0) for run in runs), default=0.0),
        "max_attack_start_average_per_driver_lap": max((run.get("attack_start_average_per_driver_lap", 0.0) for run in runs), default=0.0),
        "max_attack_start_p95_per_driver_lap": max((run.get("attack_start_p95_per_driver_lap", 0.0) for run in runs), default=0),
        "max_attack_start_per_driver_lap": max((run.get("attack_start_max_per_driver_lap", 0.0) for run in runs), default=0),
        "min_cooldown_s": min((run["cooldown_min_s"] for run in runs if run.get("cooldown_min_s") is not None), default=None),
        "max_contact_count": max((run.get("contact_count", 0) for run in runs), default=0),
        "max_contact_average_per_race_lap": max((run.get("contact_average_per_race_lap", 0.0) for run in runs), default=0.0),
        "max_corridor_rejection_reason_counts": dict(sorted(reasons.items())),
        "production_activity_guard_required": activity_guard_required,
        "production_activity_by_circuit": {
            str(circuit_id): counts
            for circuit_id, counts in sorted(circuit_activity.items())
        },
        "production_activity_failures": production_activity_failures,
    }
    matrix_modes = [
        "Broadcast 1x" if mode == "Broadcast1x" else
        "Broadcast 2x" if mode == "Broadcast2x" else
        "Broadcast 5x" if mode == "Broadcast5x" else mode
        for mode in modes
    ]
    payload = {
        "contract": "abstract-stage4-traffic-v4",
        "tick_seconds": DT,
        "matrix": {
            "circuits": list(circuits),
            "seeds": list(seeds),
            "laps": laps,
            "drivers": 20,
            "modes": matrix_modes,
        },
        "modes": list(modes),
        "run_count": len(runs),
        "runs": runs,
        "parity_fields": list(PARITY_FIELDS),
        "parity_failures": parity_failures,
        "quality": quality,
        "resources": resources,
        "approval_passed": (
            quality["run_count"] == quality["expected_run_count"]
            and quality["approval_run_count"] == quality["expected_run_count"]
            and not parity_failures
            and not zero_guard_failures
            and not production_activity_failures
            and resources["cleanup_all_runs"]
        ),
    }
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", nargs="+", type=int, default=[44])
    parser.add_argument("--circuits", nargs="+", type=int, default=[3, 4])
    parser.add_argument("--laps", type=int, default=10)
    parser.add_argument(
        "--modes",
        nargs="+",
        choices=("Instant", "Broadcast1x", "Broadcast2x", "Broadcast5x"),
        default=("Instant",),
    )
    parser.add_argument("--matrix", action="store_true")
    parser.add_argument("--case", nargs=2, metavar=("CIRCUIT", "SEED"))
    parser.add_argument(
        "--workers",
        type=int,
        default=4,
        help="isolated matrix workers (default: 4; ignored without --matrix)",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.matrix:
        args.seeds = list(range(10))
        args.circuits = [3, 4]
        args.laps = 10
        args.modes = ("Instant", "Broadcast1x", "Broadcast2x", "Broadcast5x")
    elif args.case:
        args.circuits = [int(args.case[0])]
        args.seeds = [int(args.case[1])]
    circuits = {item.id: item for item in load_circuits()}
    runs: list[dict[str, Any]] = []
    if args.matrix:
        cases = [
            (circuit_id, seed, args.laps, tuple(args.modes))
            for circuit_id in sorted(args.circuits)
            for seed in sorted(args.seeds)
        ]

        def run_case(case: tuple[int, int, int, tuple[str, ...]]) -> list[dict[str, Any]]:
            circuit_id, seed, laps, modes = case
            with tempfile.NamedTemporaryFile(
                prefix=f"abstract-stage4-{circuit_id}-{seed}-",
                suffix=".json",
                delete=False,
            ) as output_file:
                output_path = Path(output_file.name)
            command = [
                sys.executable,
                str(Path(__file__).resolve()),
                "--case",
                str(circuit_id),
                str(seed),
                "--laps",
                str(laps),
                "--modes",
                *modes,
                "--output",
                str(output_path),
            ]
            environment = os.environ.copy()
            completed = subprocess.run(
                command,
                cwd=Path(__file__).resolve().parents[2],
                env=environment,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )
            if completed.returncode != 0:
                raise RuntimeError(
                    f"matrix case {circuit_id}/{seed} failed: {completed.stderr[-4000:]}"
                )
            try:
                return json.loads(output_path.read_text(encoding="utf-8"))["runs"]
            finally:
                output_path.unlink(missing_ok=True)

        with ThreadPoolExecutor(max_workers=max(1, args.workers)) as executor:
            for case_runs in executor.map(run_case, cases):
                runs.extend(case_runs)
    else:
        for circuit_id in sorted(args.circuits):
            for seed in sorted(args.seeds):
                circuit = circuits[circuit_id]
                authority_result = None
                if "Instant" in args.modes:
                    instant = _one_run(circuit, seed, total_laps=args.laps)
                    authority_result = instant.pop("_authority_result")
                    instant["mode"] = "Instant"
                    runs.append(instant)
                for mode in args.modes:
                    if mode == "Instant":
                        continue
                    speed = int(mode.removeprefix("Broadcast").removesuffix("x"))
                    runs.append(
                        _broadcast_run(
                            circuit,
                            seed,
                            total_laps=args.laps,
                            speed_multiplier=speed,
                            authority_result=authority_result,
                        )
                    )
    payload = _aggregate_diagnostic_payload(
        runs,
        circuits=sorted(args.circuits),
        seeds=sorted(args.seeds),
        laps=args.laps,
        modes=args.modes,
    )
    encoded = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if payload["approval_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
