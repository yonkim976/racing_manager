"""Abstract result-engine contracts for Stages A through D."""

from __future__ import annotations

import asyncio
import json
import os
import subprocess
import sys
import unittest
from collections import Counter
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from data_loader import load_circuits, load_drivers, load_teams
from simulation.abstract import (
    AbstractCommandRecord,
    AbstractPoseFrame,
    AbstractRaceEngine,
    AbstractRaceResult,
    AbstractSessionSnapshot,
    AbstractTimingCheckpoint,
    IndependentRNG,
    InterruptibleReplayClock,
    LogicalReplayCursor,
    SingleVehiclePoseSynthesizer,
    VehiclePerformance,
    corridor_required_width_m,
    segment_time,
)


class AbstractRaceSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.circuit = next(circuit for circuit in load_circuits() if circuit.id == 3)
        cls.drivers = load_drivers()
        cls.teams = load_teams()

    def make_snapshot(self, drivers=None, teams=None, seed=42):
        return AbstractSessionSnapshot.from_content(
            session_id="abstract-test-session",
            session_seed=seed,
            circuit=self.circuit,
            drivers=drivers or self.drivers,
            teams=teams or self.teams,
            content_version="content-test-v1",
            ruleset_version="rules-test-v1",
        )

    def test_same_input_and_seed_is_identical_for_100_runs(self) -> None:
        snapshot = self.make_snapshot()
        outputs = [AbstractRaceEngine(snapshot).run_qualifying().canonical_json() for _ in range(100)]
        self.assertEqual(len(set(outputs)), 1)
        self.assertEqual(
            len({AbstractRaceEngine(snapshot).run_qualifying().canonical_result_hash for _ in range(100)}),
            1,
        )

    def test_input_order_is_canonicalized_before_simulation(self) -> None:
        normal = self.make_snapshot()
        reversed_snapshot = self.make_snapshot(
            drivers=list(reversed(self.drivers)),
            teams=list(reversed(self.teams)),
        )
        normal_result = AbstractRaceEngine(normal).run_qualifying()
        reversed_result = AbstractRaceEngine(reversed_snapshot).run_qualifying()
        self.assertEqual(normal.snapshot_hash, reversed_snapshot.snapshot_hash)
        self.assertEqual(normal_result.canonical_result_hash, reversed_result.canonical_result_hash)
        self.assertEqual(normal_result.grid, reversed_result.grid)

    def test_presentation_stream_does_not_change_qualifying(self) -> None:
        snapshot = self.make_snapshot()
        baseline = AbstractRaceEngine(snapshot).run_qualifying()
        engine_with_presentation_calls = AbstractRaceEngine(snapshot)
        for event_id in ("q1-camera", "q1-camera", "sector-replay", "last-lap"):
            engine_with_presentation_calls.rng.presentation(event_id).uniform(-1.0, 1.0)
        after_presentation = engine_with_presentation_calls.run_qualifying()
        self.assertEqual(baseline.canonical_result_hash, after_presentation.canonical_result_hash)

    def test_pace_stream_calls_do_not_change_reliability_stream(self) -> None:
        first = IndependentRNG(2026)
        first.driver_pace(7).random()
        reliability_a = first.vehicle_reliability("vehicle:3:7").random()

        second = IndependentRNG(2026)
        second.driver_pace(7).random()
        second.driver_pace(7).random()
        second.driver_pace(7).random()
        reliability_b = second.vehicle_reliability("vehicle:3:7").random()
        self.assertEqual(reliability_a, reliability_b)

    def test_power_upgrade_improves_straight_segment(self) -> None:
        snapshot = self.make_snapshot()
        entry = snapshot.entries[0]
        straight = next(segment for segment in snapshot.track.segments if segment.segment_type == "straight")
        baseline = segment_time(straight, entry.vehicle.performance, entry.driver, entry.tire)
        upgraded_vehicle = entry.vehicle.performance.with_upgrades({"power": 0.08})
        upgraded = segment_time(straight, upgraded_vehicle, entry.driver, entry.tire)
        self.assertLess(upgraded, baseline)

    def test_braking_upgrade_improves_heavy_braking_segment(self) -> None:
        snapshot = self.make_snapshot()
        entry = snapshot.entries[0]
        braking = next(
            segment for segment in snapshot.track.segments if segment.segment_type == "heavy_braking"
        )
        baseline = segment_time(braking, entry.vehicle.performance, entry.driver, entry.tire)
        upgraded_vehicle = entry.vehicle.performance.with_upgrades({"braking": 0.08})
        upgraded = segment_time(braking, upgraded_vehicle, entry.driver, entry.tire)
        self.assertLess(upgraded, baseline)

    def test_high_speed_aero_is_concentrated_in_sweeping_segments(self) -> None:
        sweeping_circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        snapshot = AbstractSessionSnapshot.from_content(
            session_id="abstract-test-rbr",
            session_seed=42,
            circuit=sweeping_circuit,
            drivers=self.drivers,
            teams=self.teams,
        )
        entry = snapshot.entries[0]
        high_speed = next(segment for segment in snapshot.track.segments if segment.segment_type == "sweeping")
        straight = next(segment for segment in snapshot.track.segments if segment.segment_type == "straight")
        upgraded_vehicle = entry.vehicle.performance.with_upgrades({"high_speed_aero": 0.10})
        high_speed_delta = segment_time(high_speed, entry.vehicle.performance, entry.driver, entry.tire) - segment_time(
            high_speed, upgraded_vehicle, entry.driver, entry.tire
        )
        straight_delta = segment_time(straight, entry.vehicle.performance, entry.driver, entry.tire) - segment_time(
            straight, upgraded_vehicle, entry.driver, entry.tire
        )
        self.assertGreater(high_speed_delta, straight_delta * 5.0)

    def test_reliability_is_not_a_normal_lap_time_bonus(self) -> None:
        snapshot = self.make_snapshot()
        entry = snapshot.entries[0]
        segment = snapshot.track.segments[0]
        baseline = segment_time(segment, entry.vehicle.performance, entry.driver, entry.tire)
        low_reliability = replace(
            entry.vehicle.performance,
            reliability=entry.vehicle.performance.reliability * 0.5,
        )
        self.assertEqual(
            baseline,
            segment_time(segment, low_reliability, entry.driver, entry.tire),
        )

    def test_seed_variance_does_not_make_slow_car_consistently_dominate(self) -> None:
        fast_driver = self.drivers[0]
        slow_driver = self.drivers[-1]
        fast_team = self.teams[0]
        slow_team = self.teams[-1]
        fast_times = []
        slow_times = []
        for seed in range(20):
            fast_snapshot = AbstractSessionSnapshot.from_content(
                session_id=f"fast-{seed}",
                session_seed=seed,
                circuit=self.circuit,
                drivers=[fast_driver],
                teams=[fast_team],
            )
            slow_snapshot = AbstractSessionSnapshot.from_content(
                session_id=f"slow-{seed}",
                session_seed=seed,
                circuit=self.circuit,
                drivers=[slow_driver],
                teams=[slow_team],
            )
            fast_times.append(AbstractRaceEngine(fast_snapshot).run_qualifying().grid[0].best_lap_time_s)
            slow_times.append(AbstractRaceEngine(slow_snapshot).run_qualifying().grid[0].best_lap_time_s)
        self.assertGreater(sum(slow > fast for slow, fast in zip(slow_times, fast_times)), 15)

    def test_knockout_counts_duplicates_and_grid_are_valid(self) -> None:
        result = AbstractRaceEngine(self.make_snapshot()).run_qualifying()
        by_session = {summary.session_name: summary for summary in result.sessions}
        self.assertEqual(len(by_session), 3)
        self.assertEqual(len(by_session["Q1"].eliminated_driver_ids), 5)
        self.assertEqual(len(by_session["Q2"].eliminated_driver_ids), 5)
        self.assertEqual(len(by_session["Q3"].eliminated_driver_ids), 0)
        driver_ids = [entry.driver_id for entry in result.grid]
        self.assertEqual(len(driver_ids), 20)
        self.assertEqual(len(set(driver_ids)), 20)
        self.assertEqual([entry.position for entry in result.grid], list(range(1, 21)))
        self.assertEqual({item.driver_id for item in result.driver_results}, set(driver_ids))

    def test_result_survives_canonical_json_reload(self) -> None:
        result = AbstractRaceEngine(self.make_snapshot()).run_qualifying()
        encoded = result.canonical_json()
        decoded = json.loads(encoded)
        reloaded = type(result).from_dict(decoded)
        self.assertEqual(result.canonical_result_hash, decoded["canonical_result_hash"])
        self.assertEqual(result.canonical_result_hash, reloaded.canonical_result_hash)
        self.assertEqual(encoded, reloaded.canonical_json())

    def test_performance_axes_are_explicit_and_provisional(self) -> None:
        performance = self.make_snapshot().entries[0].vehicle.performance
        self.assertEqual(
            {
                "power",
                "drag_efficiency",
                "high_speed_aero",
                "medium_speed_aero",
                "low_speed_grip",
                "braking",
                "traction",
                "tire_management",
                "cooling",
                "reliability",
            },
            set(performance.to_dict()) - {"vehicle_id", "team_id", "calibration_status"},
        )
        self.assertEqual(performance.calibration_status, "game_calibration_provisional")
        self.assertNotIn("overall_rating", performance.to_dict())

    def test_stage_b_pose_wraps_on_closed_spline(self) -> None:
        snapshot = self.make_snapshot()
        synthesizer = SingleVehiclePoseSynthesizer(snapshot)
        at_start = synthesizer.pose_at(1, 0.0)
        at_lap_boundary = synthesizer.pose_at(1, 1.0)
        self.assertEqual(
            (at_start.world_x_m, at_start.world_y_m, at_start.heading_rad),
            (at_lap_boundary.world_x_m, at_lap_boundary.world_y_m, at_lap_boundary.heading_rad),
        )
        self.assertEqual(at_start.track_distance_m, 0.0)
        self.assertAlmostEqual(
            synthesizer.pose_at(1, 0.5).track_distance_m,
            snapshot.track.track_length_m * 0.5,
            places=6,
        )

    def test_stage_b_lateral_offset_uses_spline_normal(self) -> None:
        snapshot = self.make_snapshot()
        synthesizer = SingleVehiclePoseSynthesizer(snapshot)
        center = synthesizer.pose_at(1, 0.37)
        offset = synthesizer.pose_at(1, 0.37, lateral_offset_m=2.0)
        sample = synthesizer.spline.sample(0.37)
        delta_x = offset.world_x_m - center.world_x_m
        delta_y = offset.world_y_m - center.world_y_m
        self.assertAlmostEqual(delta_x, sample.normal_x * 2.0, places=6)
        self.assertAlmostEqual(delta_y, sample.normal_y * 2.0, places=6)
        self.assertEqual(offset.source_mode, "abstract")
        self.assertNotIn("slip_angle_rad", offset.to_dict())

    def test_stage_b_speed_multiplier_preserves_same_logical_pose(self) -> None:
        snapshot = self.make_snapshot()
        synthesizer = SingleVehiclePoseSynthesizer(snapshot)
        one_x = synthesizer.sequence(
            1,
            start_total_progress=0.1,
            progress_rate_per_s=0.02,
            real_duration_s=1.0,
            speed_multiplier=1.0,
        )
        five_x = synthesizer.sequence(
            1,
            start_total_progress=0.1,
            progress_rate_per_s=0.02,
            real_duration_s=0.2,
            speed_multiplier=5.0,
        )
        self.assertEqual([frame.to_dict() for frame in one_x], [frame.to_dict() for frame in five_x])

    def test_stage_b_pose_sequence_is_continuous_and_deterministic(self) -> None:
        snapshot = self.make_snapshot()
        synthesizer = SingleVehiclePoseSynthesizer(snapshot)
        first = synthesizer.sequence(
            1,
            start_total_progress=0.95,
            progress_rate_per_s=0.03,
            real_duration_s=2.0,
            speed_multiplier=5.0,
        )
        second = synthesizer.sequence(
            1,
            start_total_progress=0.95,
            progress_rate_per_s=0.03,
            real_duration_s=2.0,
            speed_multiplier=5.0,
        )
        self.assertEqual([frame.to_dict() for frame in first], [frame.to_dict() for frame in second])
        self.assertGreater(first[-1].lap_number, first[0].lap_number)
        step_distances = [
            ((current.world_x_m - previous.world_x_m) ** 2 + (current.world_y_m - previous.world_y_m) ** 2) ** 0.5
            for previous, current in zip(first, first[1:])
        ]
        self.assertLess(max(step_distances), 100.0)

    def test_stage_c_race_is_identical_for_100_runs(self) -> None:
        snapshot = self.make_snapshot()
        baseline = AbstractRaceEngine(snapshot).run_race(total_laps=1, tick_seconds=0.5)
        for _ in range(99):
            result = AbstractRaceEngine(snapshot).run_race(total_laps=1, tick_seconds=0.5)
            self.assertEqual(result.canonical_result_hash, baseline.canonical_result_hash)
            self.assertEqual(result.canonical_json(), baseline.canonical_json())

    def test_stage1_result_retains_only_logical_checkpoints(self) -> None:
        result = AbstractRaceEngine(self.make_snapshot()).run_race(
            total_laps=10,
            tick_seconds=0.10,
        )
        self.assertFalse(hasattr(result, "frames"))
        # Stage 4 removes the old post-integrator clamp, so the logical
        # completion tick is intentionally a new versioned value.
        self.assertGreater(result.logical_tick_count, 0)
        self.assertGreater(len(result.timing_checkpoints), 1)
        self.assertLess(len(result.timing_checkpoints), result.logical_tick_count)
        self.assertTrue(all(not hasattr(item, "world_x_m") for item in result.timing_checkpoints))

    def test_stage1_result_hash_is_cached_after_construction(self) -> None:
        result = AbstractRaceEngine(self.make_snapshot()).run_race(
            total_laps=1,
            tick_seconds=0.5,
        )
        expected = result.canonical_result_hash
        with patch(
            "simulation.abstract.state.canonical_hash_sections",
            side_effect=AssertionError("hash must not be recomputed on property access"),
        ):
            self.assertEqual(result.canonical_result_hash, expected)
            self.assertEqual(result.canonical_result_hash, expected)
            self.assertEqual(result.canonical_json(), result.canonical_json())

    def test_stage1_logical_tick_count_is_part_of_hash_contract(self) -> None:
        result = AbstractRaceEngine(self.make_snapshot()).run_race(
            total_laps=1,
            tick_seconds=0.5,
        )
        changed = replace(result, logical_tick_count=result.logical_tick_count + 1)
        self.assertNotEqual(result.canonical_payload(), changed.canonical_payload())
        self.assertNotEqual(result.canonical_result_hash, changed.canonical_result_hash)

    def test_stage1_command_log_order_and_stable_time_are_hashed(self) -> None:
        result = AbstractRaceEngine(self.make_snapshot()).run_race(
            total_laps=1,
            tick_seconds=0.5,
        )
        pause = AbstractCommandRecord(
            sequence=0,
            command="pause",
            logical_time_s=1.0,
            payload={"source": "test"},
        )
        resume = AbstractCommandRecord(
            sequence=1,
            command="resume",
            logical_time_s=2.0,
            payload={"source": "test"},
        )
        ordered = replace(result, command_log=(pause, resume))
        reversed_order = replace(result, command_log=(resume, pause))
        self.assertNotEqual(ordered.canonical_payload(), reversed_order.canonical_payload())
        self.assertNotEqual(ordered.canonical_result_hash, reversed_order.canonical_result_hash)

    def test_stage1_logical_replay_cursor_runs_start_to_finish(self) -> None:
        result = AbstractRaceEngine(self.make_snapshot()).run_race(
            total_laps=2,
            tick_seconds=0.5,
        )
        cursor = LogicalReplayCursor(result.timing_checkpoints, result.logical_events)
        checkpoints = []
        while True:
            checkpoints.append(cursor.current_checkpoint)
            if cursor.is_final_checkpoint:
                break
            cursor.advance_checkpoint()
        self.assertEqual(checkpoints[0].checkpoint_kind, "race_start")
        self.assertEqual(checkpoints[-1].checkpoint_kind, "race_finish")
        self.assertEqual(
            [item.tick_index for item in checkpoints],
            sorted(item.tick_index for item in checkpoints),
        )

    def test_stage1_replay_clock_speed_change_interrupts_wait(self) -> None:
        async def scenario() -> float:
            clock = InterruptibleReplayClock(1)
            started = asyncio.get_running_loop().time()
            wait_task = asyncio.create_task(clock.wait(1.0))
            await asyncio.sleep(0.1)
            clock.set_speed(5)
            self.assertTrue(await wait_task)
            elapsed = asyncio.get_running_loop().time() - started
            clock.dispose()
            return elapsed

        elapsed = asyncio.run(scenario())
        self.assertLess(elapsed, 0.65)

    def test_stage1_replay_clock_pause_freezes_remaining_time(self) -> None:
        async def scenario() -> tuple[float, float]:
            clock = InterruptibleReplayClock(1)
            wait_task = asyncio.create_task(clock.wait(0.5))
            await asyncio.sleep(0.05)
            clock.pause()
            remaining_at_pause = clock.remaining_logical_s
            await asyncio.sleep(0.2)
            remaining_while_paused = clock.remaining_logical_s
            self.assertFalse(wait_task.done())
            clock.resume()
            self.assertTrue(await wait_task)
            clock.dispose()
            return remaining_at_pause, remaining_while_paused

        remaining_at_pause, remaining_while_paused = asyncio.run(scenario())
        self.assertAlmostEqual(remaining_at_pause, remaining_while_paused, delta=0.03)

    def test_stage1_pause_does_not_advance_replay_cursor(self) -> None:
        checkpoints = tuple(
            AbstractTimingCheckpoint(
                checkpoint_id=checkpoint_id,
                checkpoint_kind=checkpoint_kind,
                tick_index=index,
                logical_time_s=float(index),
                lap_number=index,
                order=(1,),
                progress_by_driver=((1, float(index)),),
            )
            for index, (checkpoint_id, checkpoint_kind) in enumerate(
                (("race:start", "race_start"), ("race:finish", "race_finish"))
            )
        )

        async def scenario() -> tuple[int, bool, str]:
            cursor = LogicalReplayCursor(checkpoints, ())
            clock = InterruptibleReplayClock(1)
            wait_task = asyncio.create_task(clock.wait(1.0))
            await asyncio.sleep(0.05)
            clock.pause()
            checkpoint_index = cursor.checkpoint_index
            await asyncio.sleep(0.2)
            self.assertEqual(cursor.checkpoint_index, checkpoint_index)
            self.assertFalse(wait_task.done())
            clock.resume()
            self.assertTrue(await wait_task)
            cursor.advance_checkpoint()
            checkpoint_id = cursor.current_checkpoint.checkpoint_id
            clock.dispose()
            return checkpoint_index, wait_task.done(), checkpoint_id

        checkpoint_index, wait_done, checkpoint_id = asyncio.run(scenario())
        self.assertEqual(checkpoint_index, 0)
        self.assertTrue(wait_done)
        self.assertEqual(checkpoint_id, "race:finish")

    def test_stage1_resume_uses_replay_cursor_remaining_logical_time(self) -> None:
        checkpoints = tuple(
            AbstractTimingCheckpoint(
                checkpoint_id=checkpoint_id,
                checkpoint_kind=checkpoint_kind,
                tick_index=index,
                logical_time_s=float(index),
                lap_number=index,
                order=(1,),
                progress_by_driver=((1, float(index)),),
            )
            for index, (checkpoint_id, checkpoint_kind) in enumerate(
                (("race:start", "race_start"), ("race:finish", "race_finish"))
            )
        )

        async def scenario() -> tuple[float, float, str]:
            cursor = LogicalReplayCursor(checkpoints, ())
            clock = InterruptibleReplayClock(1)
            wait_task = asyncio.create_task(clock.wait(1.0))
            await asyncio.sleep(0.05)
            clock.pause()
            remaining = clock.remaining_logical_s
            await asyncio.sleep(0.1)
            clock.resume()
            resumed_at = asyncio.get_running_loop().time()
            self.assertTrue(await wait_task)
            elapsed_after_resume = asyncio.get_running_loop().time() - resumed_at
            cursor.advance_checkpoint()
            checkpoint_id = cursor.current_checkpoint.checkpoint_id
            clock.dispose()
            return remaining, elapsed_after_resume, checkpoint_id

        remaining, elapsed_after_resume, checkpoint_id = asyncio.run(scenario())
        self.assertGreater(remaining, 0.85)
        self.assertGreater(elapsed_after_resume, 0.75)
        self.assertLess(elapsed_after_resume, 1.1)
        self.assertEqual(checkpoint_id, "race:finish")

    def test_stage1_replay_clock_resume_consumes_only_remaining_time(self) -> None:
        async def scenario() -> float:
            clock = InterruptibleReplayClock(1)
            wait_task = asyncio.create_task(clock.wait(0.4))
            await asyncio.sleep(0.05)
            clock.pause()
            remaining = clock.remaining_logical_s
            await asyncio.sleep(0.1)
            clock.resume()
            resumed_at = asyncio.get_running_loop().time()
            self.assertTrue(await wait_task)
            elapsed_after_resume = asyncio.get_running_loop().time() - resumed_at
            clock.dispose()
            self.assertGreater(remaining, 0.25)
            return elapsed_after_resume

        elapsed_after_resume = asyncio.run(scenario())
        self.assertGreater(elapsed_after_resume, 0.20)
        self.assertLess(elapsed_after_resume, 0.50)

    def test_stage1_replay_clock_speed_changes_preserve_checkpoint_sequence(self) -> None:
        async def scenario() -> list[str]:
            checkpoints = tuple(
                AbstractTimingCheckpoint(
                    checkpoint_id=checkpoint_id,
                    checkpoint_kind=checkpoint_kind,
                    tick_index=index,
                    logical_time_s=float(index),
                    lap_number=index,
                    order=(1,),
                    progress_by_driver=((1, float(index)),),
                )
                for index, (checkpoint_id, checkpoint_kind) in enumerate(
                    (
                        ("race:start", "race_start"),
                        ("lap:1:tick:1", "lap_boundary"),
                        ("lap:2:tick:2", "lap_boundary"),
                        ("race:finish", "race_finish"),
                    )
                )
            )
            cursor = LogicalReplayCursor(checkpoints, ())
            clock = InterruptibleReplayClock(1)
            seen = []
            while True:
                checkpoint = cursor.current_checkpoint
                seen.append(checkpoint.checkpoint_id)
                if cursor.is_final_checkpoint:
                    break
                next_checkpoint = cursor.checkpoints[cursor.checkpoint_index + 1]
                wait_task = asyncio.create_task(
                    clock.wait(next_checkpoint.logical_time_s - checkpoint.logical_time_s)
                )
                await asyncio.sleep(0.02)
                clock.set_speed(5)
                await asyncio.sleep(0.01)
                clock.set_speed(2)
                self.assertTrue(await wait_task)
                cursor.advance_checkpoint()
            clock.dispose()
            return seen

        seen = asyncio.run(scenario())
        self.assertEqual(len(seen), len(set(seen)))
        self.assertEqual(seen[0], "race:start")
        self.assertEqual(seen[-1], "race:finish")

    def test_stage1_replay_clock_close_cancels_wait_without_residue(self) -> None:
        async def scenario() -> tuple[bool, bool]:
            clock = InterruptibleReplayClock(1)
            wait_task = asyncio.create_task(clock.wait(10.0))
            await asyncio.sleep(0.02)
            clock.close()
            self.assertFalse(await wait_task)
            waiting = clock.waiting
            clock.dispose()
            return waiting, clock._wake_event is None

        waiting, event_disposed = asyncio.run(scenario())
        self.assertFalse(waiting)
        self.assertTrue(event_disposed)

    def test_stage1_benchmark_script_runs_without_pythonpath(self) -> None:
        repository_root = Path(__file__).resolve().parents[2]
        script = repository_root / "backend" / "tools" / "benchmark_abstract_stage1.py"
        environment = os.environ.copy()
        environment.pop("PYTHONPATH", None)
        completed = subprocess.run(
            [sys.executable, str(script), "--laps", "1"],
            cwd=repository_root,
            env=environment,
            check=True,
            capture_output=True,
            text=True,
        )
        payload = json.loads(completed.stdout)
        self.assertEqual(payload["lap_count"], 1)
        self.assertEqual(payload["frame_count"], 0)
        self.assertEqual(payload["vehicle_state_record_count"], 0)

    def test_stage_c_race_canonicalizes_input_order(self) -> None:
        normal = self.make_snapshot()
        reversed_snapshot = self.make_snapshot(
            drivers=list(reversed(self.drivers)),
            teams=list(reversed(self.teams)),
        )
        normal_result = AbstractRaceEngine(normal).run_race(total_laps=1, tick_seconds=0.5)
        reversed_result = AbstractRaceEngine(reversed_snapshot).run_race(total_laps=1, tick_seconds=0.5)
        self.assertEqual(normal.snapshot_hash, reversed_snapshot.snapshot_hash)
        self.assertEqual(normal_result.canonical_result_hash, reversed_result.canonical_result_hash)
        self.assertEqual(normal_result.finish_order, reversed_result.finish_order)

    def test_stage_c_presentation_calls_do_not_change_race(self) -> None:
        snapshot = self.make_snapshot()
        baseline = AbstractRaceEngine(snapshot).run_race(total_laps=1, tick_seconds=0.5)
        engine = AbstractRaceEngine(snapshot)
        for event_id in ("race-camera", "race-camera", "sector-replay", "finish-banner"):
            engine.rng.presentation(event_id).uniform(-1.0, 1.0)
        after_calls = engine.run_race(total_laps=1, tick_seconds=0.5)
        self.assertEqual(baseline.canonical_result_hash, after_calls.canonical_result_hash)

    def test_stage_c_traffic_has_contiguous_ranks_and_two_wide_corridors(self) -> None:
        from simulation.abstract.race import AbstractTrafficSimulationCursor

        snapshot = self.make_snapshot()
        cursor = AbstractTrafficSimulationCursor(
            snapshot,
            grid_order=tuple(reversed(snapshot.initial_grid_order)),
            total_laps=2,
            tick_seconds=0.10,
        )
        cursor.inject_maneuver_fixture(
            attacker_id=2,
            defender_id=1,
            progress=0.36,
            gap_m=40.0,
            attacker_speed_mps=60.0,
            defender_speed_mps=0.0,
            outcome="success",
        )
        presentation_frames = [cursor.current_frame]
        try:
            while not cursor.finished:
                presentation_frames.append(cursor.advance_one_tick().frame)
        finally:
            cursor.dispose()
        self.assertEqual(corridor_required_width_m(), 4.55)
        all_driver_ids = set(snapshot.initial_grid_order)
        corridor_states = {}
        for frame in presentation_frames:
            self.assertEqual([vehicle.position for vehicle in frame.vehicles], list(range(1, 21)))
            self.assertEqual({vehicle.driver_id for vehicle in frame.vehicles}, all_driver_ids)
            for vehicle in frame.vehicles:
                if vehicle.position > 1 and vehicle.corridor_id is None:
                    self.assertGreaterEqual(vehicle.gap_to_ahead_m, 5.0 - 1e-6)
                if vehicle.corridor_id is not None:
                    corridor_states.setdefault(vehicle.corridor_id, []).append(vehicle)
        self.assertTrue(corridor_states)
        for corridor_id, states in corridor_states.items():
            self.assertEqual(len({vehicle.driver_id for vehicle in states}), 2)
            self.assertEqual({vehicle.corridor_side for vehicle in states}, {"inside", "outside"})
            self.assertTrue(
                {vehicle.maneuver for vehicle in states}.issubset(
                    {
                        "approach",
                        "pull_out",
                        "overlap",
                        "crossing_confirmed",
                        "clearance_confirmed",
                        "yield_or_abort",
                        "fall_back_to_safe_gap",
                        "rejoin",
                        "rejoin_complete",
                        "normal",
                    }
                )
            )
            self.assertTrue(all(vehicle.corridor_id == corridor_id for vehicle in states))

    def test_stage_c_attack_defend_timeline_and_finish_order_are_valid(self) -> None:
        # Exercise the production state updater through an explicit geometry-
        # backed fixture.  Requiring a pass from a production RNG seed would
        # make this regression probabilistic and would not test the state
        # machine contract.
        from simulation.abstract.race import AbstractTrafficSimulationCursor

        snapshot = AbstractSessionSnapshot.from_content(
            session_id="abstract-stage4-attack-test",
            session_seed=0,
            circuit=next(circuit for circuit in load_circuits() if circuit.id == 4),
            drivers=self.drivers,
            teams=self.teams,
            content_version="content-test-v1",
            ruleset_version="rules-test-v1",
        )
        cursor = AbstractTrafficSimulationCursor(snapshot, total_laps=2, tick_seconds=0.10)
        corridor_id = cursor.inject_maneuver_fixture(
            attacker_id=2,
            defender_id=1,
            progress=0.36,
            gap_m=40.0,
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
        starts = [event for event in result.logical_events if event.event_type == "attack_started"]
        terminal_types = {"overtake_completed", "defense_hold", "attack_aborted", "rejoin_complete"}
        self.assertTrue(starts)
        self.assertTrue(any(event.event_type == "overtake_completed" for event in result.logical_events))
        for start in starts:
            corridor_id = dict(start.payload)["corridor_id"]
            terminal = next(
                event
                for event in result.logical_events
                if event.event_type in terminal_types
                and event.event_id.startswith(f"{corridor_id}:")
            )
            self.assertGreaterEqual(terminal.logical_time_s, start.logical_time_s)
        self.assertEqual(len(result.finish_order), 20)
        self.assertEqual(len(set(result.finish_order)), 20)
        self.assertEqual(set(result.finish_order), set(result.grid_order))

    def test_stage_c_race_result_survives_canonical_json_reload(self) -> None:
        result = AbstractRaceEngine(self.make_snapshot()).run_race(total_laps=1, tick_seconds=0.5)
        encoded = result.canonical_json()
        reloaded = AbstractRaceResult.from_dict(json.loads(encoded))
        self.assertEqual(result.canonical_result_hash, json.loads(encoded)["canonical_result_hash"])
        self.assertEqual(result.canonical_result_hash, reloaded.canonical_result_hash)
        self.assertEqual(encoded, reloaded.canonical_json())

    def test_stage_d_ten_lap_race_is_deterministic_and_completes(self) -> None:
        snapshot = self.make_snapshot()
        baseline = AbstractRaceEngine(snapshot).run_race(
            total_laps=10,
            tick_seconds=0.5,
            max_ticks=2000,
        )
        repeat = AbstractRaceEngine(snapshot).run_race(
            total_laps=10,
            tick_seconds=0.5,
            max_ticks=2000,
        )
        self.assertEqual(baseline.canonical_result_hash, repeat.canonical_result_hash)
        self.assertEqual(baseline.canonical_json(), repeat.canonical_json())
        self.assertEqual(len(baseline.finish_order), 20)
        self.assertEqual(len(set(baseline.finish_order)), 20)
        self.assertEqual(set(baseline.finish_order), set(baseline.grid_order))

        event_types = Counter(event.event_type for event in baseline.logical_events)
        self.assertEqual(event_types["race_started"], 1)
        self.assertEqual(event_types["race_finished"], 1)
        self.assertEqual(event_types["pit_requested"], 20)
        self.assertGreater(event_types["lockup_started"], 0)
        # Contact is validated through an explicit deterministic injection
        # boundary.  A production seed is not a valid fixture for requiring
        # a low-probability event.
        _, contact_events = self._run_deterministic_contact_fixture()
        self.assertGreater(
            Counter(event.event_type for event in contact_events)["contact_started"],
            0,
        )

    def _run_deterministic_contact_fixture(self):
        from simulation.abstract.race import AbstractTrafficSimulationCursor

        cursor = AbstractTrafficSimulationCursor(
            self.make_snapshot(),
            total_laps=2,
            tick_seconds=0.5,
        )
        try:
            corridor_id = cursor.inject_maneuver_fixture(
                attacker_id=2,
                defender_id=1,
                progress=0.55,
                gap_m=20.0,
                attacker_speed_mps=80.0,
                defender_speed_mps=0.0,
                outcome="abort",
                force_contact=True,
            )
            frames = [cursor.current_frame]
            for _ in range(40):
                if cursor.finished:
                    break
                frames.append(cursor.advance_one_tick().frame)
                if any(
                    event.event_id.startswith(corridor_id)
                    and event.event_type == "contact_started"
                    for event in cursor.events
                ):
                    break
            events = tuple(
                event
                for event in cursor.events
                if event.event_id.startswith(corridor_id)
            )
            return frames, events
        finally:
            cursor.dispose()

    def test_stage_d_pit_strategy_has_complete_timeline_and_tire_state(self) -> None:
        snapshot = self.make_snapshot()
        pit_strategy = {entry.driver_id: (1,) for entry in snapshot.entries}
        presentation_frames = []
        result = AbstractRaceEngine(snapshot).run_race(
            total_laps=2,
            tick_seconds=0.5,
            max_ticks=1000,
            pit_strategy=pit_strategy,
            frame_sink=presentation_frames.append,
        )
        event_types = Counter(event.event_type for event in result.logical_events)
        for event_type in (
            "pit_requested",
            "pit_lane_entry",
            "pit_stop_started",
            "pit_stop_completed",
            "pit_exit",
        ):
            self.assertEqual(event_types[event_type], 20)
        last_frame = presentation_frames[-1]
        self.assertTrue(all(vehicle.pit_state == "none" for vehicle in last_frame.vehicles))
        self.assertTrue(all(vehicle.pit_stop_count == 1 for vehicle in last_frame.vehicles))
        self.assertTrue(all(vehicle.wear_laps >= 0.0 for vehicle in last_frame.vehicles))
        self.assertTrue(all(vehicle.physical_compound == "C2" for vehicle in last_frame.vehicles))

    def test_stage_d_incidents_are_logical_and_damage_is_bounded(self) -> None:
        presentation_frames = []
        result = AbstractRaceEngine(self.make_snapshot()).run_race(
            total_laps=10,
            tick_seconds=0.5,
            max_ticks=2000,
            frame_sink=presentation_frames.append,
        )
        incident_events = [
            event
            for event in result.logical_events
            if event.event_type in {"lockup_started", "contact_started"}
        ]
        fixture_frames, fixture_events = self._run_deterministic_contact_fixture()
        incident_events.extend(
            event
            for event in fixture_events
            if event.event_type in {"lockup_started", "contact_started"}
        )
        self.assertTrue(incident_events)
        for event in incident_events:
            payload = dict(event.payload)
            self.assertNotIn("slip_angle", payload)
            self.assertNotIn("force_n", payload)
            self.assertNotIn("temperature_kelvin", payload)
        damage_values = [
            vehicle.damage_level
            for frame in presentation_frames
            for vehicle in frame.vehicles
        ]
        damage_values.extend(
            vehicle.damage_level
            for frame in fixture_frames
            for vehicle in frame.vehicles
        )
        self.assertGreater(max(damage_values), 0.0)
        self.assertLessEqual(max(damage_values), 1.0)

    def test_stage_d_output_survives_canonical_json_reload(self) -> None:
        result = AbstractRaceEngine(self.make_snapshot()).run_race(
            total_laps=2,
            tick_seconds=0.5,
            max_ticks=1000,
        )
        encoded = result.canonical_json()
        reloaded = AbstractRaceResult.from_dict(json.loads(encoded))
        self.assertEqual(result.canonical_result_hash, json.loads(encoded)["canonical_result_hash"])
        self.assertEqual(result.canonical_result_hash, reloaded.canonical_result_hash)
        self.assertEqual(encoded, reloaded.canonical_json())


if __name__ == "__main__":
    unittest.main()
