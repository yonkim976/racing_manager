"""Stage 4 traffic contracts.

These tests intentionally describe the post-prototype contract.  They are
expected to fail against the pre-Stage-4 implementation because that version
clamped progress after integration, snapped lateral offsets and broadcast a
single probe.
"""

from __future__ import annotations

import asyncio
import json
import subprocess
import sys
import unittest
from pathlib import Path

from data_loader import load_circuits, load_drivers, load_teams
from models.schemas import TrackConditions
from engines.abstract.runtime import AbstractRaceEngine, AbstractSessionSnapshot
from engines.abstract.runtime.broadcast import AbstractBroadcastSession
from engines.abstract.runtime.race import AbstractTrafficSimulationCursor
from engines.abstract.runtime.racecraft import local_corridor_assessment
from tools.diagnose_abstract_stage4_traffic import _aggregate_diagnostic_payload


class _WebSocket:
    def __init__(self) -> None:
        self.messages: list[dict] = []

    async def send_json(self, message: dict) -> None:
        self.messages.append(message)

    async def close(self, **_: object) -> None:
        return None


class AbstractStage4TrafficTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.circuits = {item.id: item for item in load_circuits()}
        cls.drivers = load_drivers()
        cls.teams = load_teams()

    def snapshot(self, circuit_id: int = 3, seed: int = 44):
        return AbstractSessionSnapshot.from_content(
            session_id=f"stage4-traffic-{circuit_id}-{seed}",
            session_seed=seed,
            circuit=self.circuits[circuit_id],
            drivers=self.drivers,
            teams=self.teams,
        )

    def test_all_cars_use_atomic_monotonic_distance(self) -> None:
        snapshot = self.snapshot()
        frames = []
        AbstractRaceEngine(snapshot).run_race(
            total_laps=1,
            tick_seconds=0.10,
            frame_sink=frames.append,
        )
        previous: dict[int | str, float] = {}
        reversals = 0
        for frame in frames:
            for vehicle in frame.vehicles:
                if vehicle.driver_id in previous and vehicle.total_progress < previous[vehicle.driver_id] - 1e-9:
                    reversals += 1
                previous[vehicle.driver_id] = vehicle.total_progress
        self.assertEqual(reversals, 0)

    def test_lateral_transition_is_tick_bounded(self) -> None:
        snapshot = self.snapshot()
        frames = []
        AbstractRaceEngine(snapshot).run_race(
            total_laps=1,
            tick_seconds=0.10,
            frame_sink=frames.append,
        )
        previous: dict[int | str, float] = {}
        maximum_jump = 0.0
        for frame in frames:
            for vehicle in frame.vehicles:
                if vehicle.driver_id in previous:
                    maximum_jump = max(
                        maximum_jump,
                        abs(vehicle.lateral_offset_m - previous[vehicle.driver_id]),
                    )
                previous[vehicle.driver_id] = vehicle.lateral_offset_m
        self.assertLessEqual(maximum_jump, 8.0 * 0.10 + 1e-6)

    def test_standing_grid_launches_together_without_sideways_slide(self) -> None:
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                cursor = AbstractTrafficSimulationCursor(
                    self.snapshot(circuit_id=circuit_id, seed=42),
                    total_laps=1,
                    tick_seconds=0.10,
                )
                try:
                    initial = {
                        vehicle.driver_id: vehicle
                        for vehicle in cursor.current_frame.vehicles
                    }
                    hold_tick = None
                    for _ in range(cursor.lights_out_tick):
                        hold_tick = cursor.advance_one_tick()
                    self.assertIsNotNone(hold_tick)
                    for vehicle in hold_tick.frame.vehicles:
                        self.assertEqual(vehicle.speed_mps, 0.0)
                        self.assertAlmostEqual(vehicle.lateral_velocity_mps, 0.0)
                        self.assertAlmostEqual(
                            vehicle.lateral_offset_m,
                            initial[vehicle.driver_id].lateral_offset_m,
                        )

                    launch = cursor.advance_one_tick()
                    self.assertTrue(
                        all(vehicle.speed_mps > 0.0 for vehicle in launch.frame.vehicles)
                    )
                    self.assertTrue(
                        all(
                            vehicle.line_distance_m
                            > initial[vehicle.driver_id].line_distance_m
                            for vehicle in launch.frame.vehicles
                        )
                    )
                    self.assertTrue(
                        all(
                            abs(vehicle.lateral_velocity_mps) <= 1e-12
                            for vehicle in launch.frame.vehicles
                        )
                    )

                    stopped_lateral_motion = 0
                    while cursor.logical_time_s < 5.0 - 1e-9:
                        tick = cursor.advance_one_tick()
                        stopped_lateral_motion += sum(
                            1
                            for vehicle in tick.frame.vehicles
                            if vehicle.speed_mps <= 1e-9
                            and abs(vehicle.lateral_velocity_mps) > 1e-9
                        )
                    self.assertEqual(stopped_lateral_motion, 0)
                    self.assertEqual(cursor.metrics["body_overlap_count"], 0)
                finally:
                    cursor.dispose()

    def test_pair_attack_cooldown_is_logical_seconds(self) -> None:
        for tick_seconds in (0.05, 0.10, 0.50):
            result = AbstractRaceEngine(self.snapshot()).run_race(
                total_laps=3,
                tick_seconds=tick_seconds,
            )
            last: dict[tuple[int | str, ...], float] = {}
            intervals: list[float] = []
            for event in result.logical_events:
                if event.event_type != "attack_started":
                    continue
                pair = tuple(event.driver_ids)
                if pair in last:
                    intervals.append(event.logical_time_s - last[pair])
                last[pair] = event.logical_time_s
            self.assertTrue(not intervals or min(intervals) >= 8.0 - tick_seconds - 1e-9)

    def test_accepted_distance_speed_and_acceleration_are_atomic(self) -> None:
        snapshot = self.snapshot(seed=45)
        frames = []
        AbstractRaceEngine(snapshot).run_race(
            total_laps=2,
            tick_seconds=0.10,
            frame_sink=frames.append,
        )
        maximum_distance_error = 0.0
        maximum_acceleration_error = 0.0
        previous: dict[int | str, object] = {}
        for frame in frames:
            for vehicle in frame.vehicles:
                old = previous.get(vehicle.driver_id)
                if old is not None:
                    maximum_distance_error = max(
                        maximum_distance_error,
                        abs(
                            vehicle.line_distance_m - old.line_distance_m
                            - (old.speed_mps + vehicle.speed_mps) * 0.05
                        ),
                    )
                    maximum_acceleration_error = max(
                        maximum_acceleration_error,
                        abs(
                            vehicle.longitudinal_acceleration_mps2
                            - (vehicle.speed_mps - old.speed_mps) / 0.10
                        ),
                    )
                previous[vehicle.driver_id] = vehicle
        self.assertLessEqual(maximum_distance_error, 1e-7)
        self.assertLessEqual(maximum_acceleration_error, 1e-7)

    def test_rank_swap_requires_crossing_and_clearance_events(self) -> None:
        # Use the explicit geometry-backed injection boundary instead of
        # repeating a production RNG draw until a desired pass appears.
        cursor = AbstractTrafficSimulationCursor(
            self.snapshot(circuit_id=4, seed=0),
            total_laps=2,
            tick_seconds=0.10,
        )
        cursor.inject_maneuver_fixture(
            attacker_id=2,
            defender_id=1,
            progress=1.10,
            gap_m=12.0,
            attacker_speed_mps=60.0,
            defender_speed_mps=0.0,
            outcome="success",
        )
        try:
            while not cursor.finished:
                cursor.advance_one_tick()
            result = cursor.finalize()
        finally:
            cursor.dispose()
        events_by_prefix: dict[str, list[str]] = {}
        for event in result.logical_events:
            prefix = event.event_id.rsplit(":", 1)[0]
            events_by_prefix.setdefault(prefix, []).append(event.event_type)
        passes = [event for event in result.logical_events if event.event_type == "overtake_completed"]
        self.assertTrue(passes)
        for event in passes:
            prefix = event.event_id.rsplit(":", 1)[0]
            phases = events_by_prefix[prefix]
            self.assertIn("overtake_crossing_confirmed", phases)
            self.assertEqual(dict(event.payload)["rank_swap"], "atomic")

    def test_frame_sink_does_not_change_traffic_result(self) -> None:
        snapshot = self.snapshot(seed=46)
        without_sink = AbstractRaceEngine(snapshot).run_race(total_laps=2, tick_seconds=0.10)
        frames = []
        with_sink = AbstractRaceEngine(snapshot).run_race(
            total_laps=2,
            tick_seconds=0.10,
            frame_sink=frames.append,
        )
        self.assertEqual(without_sink.canonical_result_hash, with_sink.canonical_result_hash)
        self.assertEqual(without_sink.finish_order, with_sink.finish_order)
        self.assertGreater(len(frames), 1)

    def test_runtime_checkpoint_replays_identical_unpublished_ticks(self) -> None:
        cursor = AbstractTrafficSimulationCursor(
            self.snapshot(seed=146),
            total_laps=2,
            tick_seconds=0.10,
        )
        try:
            for _ in range(20):
                cursor.advance_one_tick()
            checkpoint = cursor.export_runtime_checkpoint()
            for _ in range(20):
                cursor.advance_one_tick()
            expected_frame = cursor.current_frame.to_dict()
            expected_events = tuple(event.to_dict() for event in cursor.events)

            cursor.restore_runtime_checkpoint(checkpoint)
            for _ in range(20):
                cursor.advance_one_tick()
            self.assertEqual(cursor.current_frame.to_dict(), expected_frame)
            self.assertEqual(
                tuple(event.to_dict() for event in cursor.events),
                expected_events,
            )
        finally:
            cursor.dispose()

    def test_interactive_commands_are_deterministic_result_authority(self) -> None:
        def run_once():
            cursor = AbstractTrafficSimulationCursor(
                self.snapshot(seed=147),
                total_laps=2,
                tick_seconds=0.10,
            )
            driver_id = cursor.initial_grid_order[0]
            try:
                cursor.set_pace_mode(driver_id, "ATTACK")
                cursor.request_pit(
                    driver_id,
                    tire_role="HARD",
                    physical_compound="C1",
                )
                while not cursor.finished:
                    cursor.advance_one_tick()
                return cursor.finalize()
            finally:
                cursor.dispose()

        first = run_once()
        second = run_once()
        self.assertEqual(first.canonical_result_hash, second.canonical_result_hash)
        self.assertEqual(
            [record.command for record in first.command_log],
            ["set_pace_mode", "pit_call"],
        )
        self.assertTrue(
            any(event.event_type == "pit_call_registered" for event in first.logical_events)
        )

    def test_broadcast_is_bounded_full_field(self) -> None:
        snapshot = self.snapshot()
        result = AbstractRaceEngine(snapshot).run_race(total_laps=1, tick_seconds=0.10)
        player_team = self.teams[0]
        session = AbstractBroadcastSession(
            snapshot=snapshot,
            result=result,
            circuit=self.circuits[3],
            player_team=player_team,
            player_drivers=[driver for driver in self.drivers if driver.team_id == player_team.id],
            drivers=self.drivers,
            teams=self.teams,
            track_conditions=TrackConditions(),
            thermal_preset=None,
            conditions_source="stage4-test",
        )
        self.assertEqual(session.race_info["pose_count"], 20)
        self.assertEqual(session.race_info["presentation_contract"], "stage4-full-field-traffic")
        socket = _WebSocket()
        asyncio.run(session.add_client(socket))
        self.assertEqual(socket.messages[1]["pose_count"], 20)
        self.assertEqual(len(socket.messages[1]["poses"]), 20)
        asyncio.run(session.close())

    def test_stage4_diagnostic_script_emits_reproducible_aggregate_schema(self) -> None:
        repository_root = Path(__file__).resolve().parents[2]
        completed = subprocess.run(
            [
                sys.executable,
                str(repository_root / "backend" / "tools" / "diagnose_abstract_stage4_traffic.py"),
                "--circuits",
                "3",
                "--seeds",
                "0",
                "--laps",
                "1",
                "--modes",
                "Instant",
            ],
            cwd=repository_root,
            check=True,
            capture_output=True,
            text=True,
        )
        payload = json.loads(completed.stdout)
        self.assertEqual(payload["run_count"], 1)
        self.assertIn("matrix", payload)
        self.assertIn("parity_fields", payload)
        self.assertIn("quality", payload)
        self.assertIn("resources", payload)
        self.assertTrue(payload["approval_passed"])

    def test_full_matrix_approval_rejects_zero_production_activity(self) -> None:
        runs = [
            {
                "circuit_id": circuit_id,
                "seed": seed,
                "mode": "Instant",
                "approval_passed": True,
                "attack_start_count": 0,
                "pass_count": 0,
            }
            for circuit_id in (3, 4)
            for seed in range(10)
        ]
        payload = _aggregate_diagnostic_payload(
            runs,
            circuits=[3, 4],
            seeds=list(range(10)),
            laps=10,
            modes=("Instant",),
        )
        self.assertTrue(payload["quality"]["production_activity_guard_required"])
        self.assertEqual(
            [failure["circuit_id"] for failure in payload["quality"]["production_activity_failures"]],
            [3, 4],
        )
        self.assertFalse(payload["approval_passed"])

        for circuit_id in (3, 4):
            active = next(run for run in runs if run["circuit_id"] == circuit_id)
            active["attack_start_count"] = 1
            active["pass_count"] = 1
        active_payload = _aggregate_diagnostic_payload(
            runs,
            circuits=[3, 4],
            seeds=list(range(10)),
            laps=10,
            modes=("Instant",),
        )
        self.assertEqual(active_payload["quality"]["production_activity_failures"], [])
        self.assertTrue(active_payload["approval_passed"])

    def test_bahrain_and_red_bull_ring_production_runs_have_safe_overtakes(self) -> None:
        for circuit_id, seed in ((3, 0), (4, 1)):
            with self.subTest(circuit_id=circuit_id, seed=seed):
                cursor = AbstractTrafficSimulationCursor(
                    self.snapshot(circuit_id=circuit_id, seed=seed),
                    total_laps=10,
                    tick_seconds=0.10,
                )
                try:
                    while not cursor.finished:
                        cursor.advance_one_tick()
                    self.assertGreater(cursor.metrics["attack_start_count"], 0)
                    self.assertGreater(cursor.metrics["overtake_completed_count"], 0)
                    for metric_name in (
                        "corridor_conflict_count",
                        "admitted_reservation_conflict_count",
                        "third_vehicle_occupancy_conflict_count",
                        "reservation_body_clearance_violation_count",
                        "same_lane_longitudinal_overlap_count",
                        "body_overlap_count",
                        "crossing_before_rank_swap_count",
                        "post_integrator_distance_correction_count",
                    ):
                        self.assertEqual(cursor.metrics[metric_name], 0, metric_name)
                finally:
                    cursor.dispose()

    def test_local_corridor_uses_local_width_and_third_vehicle(self) -> None:
        assessment = local_corridor_assessment(
            local_left_width_m=3.0,
            local_right_width_m=3.0,
            racing_line_offset_m=0.0,
            vehicle_width_m=1.9,
            vehicle_length_m=5.4,
            segment_type="straight",
            corner_phase="entry",
            remaining_distance_m=20.0,
            gap_m=8.0,
            closing_speed_mps=4.0,
            third_vehicle_conflict=True,
            reservation_conflict=False,
        )
        self.assertFalse(assessment.allowed)
        self.assertEqual(assessment.reason_code, "corridor_conflict")


if __name__ == "__main__":
    unittest.main()
