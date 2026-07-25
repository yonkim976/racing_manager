"""Tests for race-session delivery cadence."""

from __future__ import annotations

import unittest
import asyncio
from types import SimpleNamespace

from session import (
    BROADCAST_HZ,
    BROADCAST_INTERVAL,
    DASHBOARD_BROADCAST_HZ,
    PAUSED_BROADCAST_HZ,
    POSE_DRIVER_HEADER,
    POSE_PACKET_HEADER,
    POSE_PACKET_MAGIC,
    POSE_SAMPLE,
    POSE_BROADCAST_HZ,
    RaceSession,
    SessionCadenceMetrics,
    TIMING_BROADCAST_HZ,
)
from models.schemas import (
    LapTimeInfo,
    RaceEvent,
    RaceTickState,
)
from simulation.race_engine import RaceEngine
from simulation.vehicle_physics import PHYSICS_STEP_SECONDS


class RaceSessionCadenceTests(unittest.TestCase):
    def test_pose_dashboard_and_paused_streams_have_independent_cadence(self) -> None:
        self.assertEqual(BROADCAST_HZ, 30)
        self.assertAlmostEqual(BROADCAST_INTERVAL, 1.0 / 30.0)
        self.assertEqual(POSE_BROADCAST_HZ, 30)
        self.assertEqual(DASHBOARD_BROADCAST_HZ, 4)
        self.assertEqual(TIMING_BROADCAST_HZ, 1)
        self.assertEqual(PAUSED_BROADCAST_HZ, 1)

    def test_compact_payloads_keep_protocol_fields_and_omit_defaults(self) -> None:
        pose_state = SimpleNamespace(
            driver_id=7,
            simulation_time_s=0.0,
            physics_frame=12,
            world_x_m=0.0,
            world_y_m=0.0,
            heading_rad=0.0,
            retired=False,
            hazard_active=False,
        )
        pose_engine = SimpleNamespace(
            _physics_frame=12,
            speed_multiplier=1,
            paused=False,
            driver_states={7: pose_state},
        )
        pose_session = RaceSession("pose", pose_engine, None, None, [])
        pose_payload = pose_session._pose_payload()
        self.assertIsInstance(pose_payload, bytes)
        magic, physics_frame, speed, paused, driver_count = (
            POSE_PACKET_HEADER.unpack_from(pose_payload)
        )
        self.assertEqual(magic, POSE_PACKET_MAGIC)
        self.assertEqual(physics_frame, 12)
        self.assertEqual(speed, 1)
        self.assertEqual(paused, 0)
        self.assertEqual(driver_count, 1)
        driver_id, flags, sample_count = POSE_DRIVER_HEADER.unpack_from(
            pose_payload,
            POSE_PACKET_HEADER.size,
        )
        self.assertEqual((driver_id, flags, sample_count), (7, 0, 1))
        sample = POSE_SAMPLE.unpack_from(
            pose_payload,
            POSE_PACKET_HEADER.size + POSE_DRIVER_HEADER.size,
        )
        self.assertEqual(
            sample,
            (0.0, 12, 0.0, 0.0, 0.0),
        )

        dashboard = RaceTickState(
            type="race_state",
            lap=1,
            total_laps=57,
        )
        dashboard_payload = RaceSession._dashboard_payload(dashboard)
        self.assertEqual(dashboard_payload["type"], "race_state")
        self.assertEqual(dashboard_payload["lap"], 1)
        self.assertEqual(dashboard_payload["total_laps"], 57)
        self.assertNotIn("events", dashboard_payload)

    def test_public_feed_hides_internal_events_and_applies_cooldown(self) -> None:
        class FakeEngine:
            finished = False

        session = RaceSession("test", FakeEngine(), None, None, [])
        first = session._prepare_public_events(
            [
                RaceEvent(
                    type="maneuver_group_formed",
                    driver="VER",
                    message="internal",
                ),
                RaceEvent(type="lockup", driver="VER", message="lockup one"),
                RaceEvent(type="lockup", driver="VER", message="lockup duplicate"),
                RaceEvent(type="pit_stop", driver="NOR", message="pit complete"),
            ],
            now=10.0,
        )

        self.assertEqual([event.type for event in first], ["lockup", "pit_stop"])
        self.assertEqual([event.event_id for event in first], [1, 2])

        during_cooldown = session._prepare_public_events(
            [RaceEvent(type="lockup", driver="VER", message="lockup later")],
            now=15.0,
        )
        after_cooldown = session._prepare_public_events(
            [RaceEvent(type="lockup", driver="VER", message="lockup later")],
            now=18.1,
        )
        self.assertEqual(during_cooldown, [])
        self.assertEqual(after_cooldown[0].event_id, 3)

    def test_routine_battle_feed_is_globally_rate_limited(self) -> None:
        class FakeEngine:
            finished = False

        session = RaceSession("test", FakeEngine(), None, None, [])
        first = session._prepare_public_events(
            [RaceEvent(type="defend", driver="VER", message="defends")],
            now=10.0,
        )
        suppressed = session._prepare_public_events(
            [RaceEvent(type="overtake_abort", driver="NOR", message="aborts")],
            now=10.5,
        )
        later = session._prepare_public_events(
            [RaceEvent(type="attack", driver="LEC", message="attacks")],
            now=11.6,
        )

        self.assertEqual(len(first), 1)
        self.assertEqual(suppressed, [])
        self.assertEqual(len(later), 1)

    def test_history_state_sends_only_new_laps_after_full_snapshot(self) -> None:
        engine = object.__new__(RaceEngine)
        engine.driver_states = {
            1: SimpleNamespace(driver_id=1),
            2: SimpleNamespace(driver_id=2),
        }
        first_lap = LapTimeInfo(
            lap=1,
            lap_time=95.0,
            tire_compound="MEDIUM",
            stint=1,
        )
        second_lap = first_lap.model_copy(update={"lap": 2, "lap_time": 94.5})
        engine._lap_history = {1: [first_lap], 2: [first_lap]}

        full = engine.build_history_state()
        self.assertTrue(full.full_snapshot)
        self.assertEqual(len(full.histories), 2)

        engine._lap_history[1].append(second_lap)
        update = engine.build_history_state(since_by_driver={1: 1, 2: 1})
        self.assertFalse(update.full_snapshot)
        self.assertEqual(len(update.histories), 1)
        self.assertEqual(update.histories[0].driver_id, 1)
        self.assertEqual(update.histories[0].start_index, 1)
        self.assertEqual([lap.lap for lap in update.histories[0].lap_history], [2])

    def test_runtime_metrics_report_effective_speed_backlog_and_jitter(self) -> None:
        metrics = SessionCadenceMetrics()
        metrics.reset(now=10.0, requested_speed_multiplier=2)
        for _ in range(5):
            metrics.record_physics_step(PHYSICS_STEP_SECONDS)

        snapshot = metrics.snapshot(
            now=10.05,
            scheduled_broadcast_at=10.04,
            simulation_backlog_seconds=0.03,
            requested_speed_multiplier=2,
            paused=False,
        )

        self.assertAlmostEqual(snapshot["effective_speed_multiplier"], 2.0)
        self.assertAlmostEqual(snapshot["simulation_backlog_seconds"], 0.03)
        self.assertAlmostEqual(snapshot["broadcast_jitter_ms"], 10.0)
        self.assertEqual(snapshot["physics_steps_last_broadcast"], 5)
        self.assertEqual(snapshot["broadcast_hz"], 30)

        following = metrics.snapshot(
            now=10.08,
            scheduled_broadcast_at=10.08,
            simulation_backlog_seconds=0.0,
            requested_speed_multiplier=2,
            paused=False,
        )
        self.assertEqual(following["physics_steps_last_broadcast"], 0)

    def test_runtime_metrics_restart_window_when_speed_changes(self) -> None:
        metrics = SessionCadenceMetrics()
        metrics.reset(now=1.0, requested_speed_multiplier=1)
        metrics.record_physics_step(PHYSICS_STEP_SECONDS)

        snapshot = metrics.snapshot(
            now=1.02,
            scheduled_broadcast_at=1.02,
            simulation_backlog_seconds=0.0,
            requested_speed_multiplier=2,
            paused=False,
        )

        self.assertEqual(snapshot["effective_speed_multiplier"], 0.0)
        self.assertEqual(snapshot["physics_steps_last_broadcast"], 0)

    def test_advance_engine_uses_exact_physics_steps_and_retains_poses(self) -> None:
        state = SimpleNamespace(
            driver_id=1,
            simulation_time_s=0.0,
            physics_frame=0,
            world_x_m=0.0,
            world_y_m=0.0,
            heading_rad=0.0,
            progress=0.0,
            lateral_offset_m=0.0,
            in_pit=False,
        )

        class FakeEngine:
            finished = False
            driver_states = {1: state}

            def tick(self, delta: float) -> list:
                self.driver_states[1].simulation_time_s += delta
                self.driver_states[1].physics_frame += 1
                self.driver_states[1].world_x_m += 1.0
                return []

            @staticmethod
            def _pit_lane_progress(driver_id: int) -> float:
                return 0.0

        session = RaceSession("test", FakeEngine(), None, None, [])
        session._advance_engine(PHYSICS_STEP_SECONDS * 2)

        samples = session._trajectory_samples[1]
        decoded = [POSE_SAMPLE.unpack(sample) for sample in samples]
        self.assertEqual([sample[1] for sample in decoded], [1, 2])
        self.assertEqual([sample[2] for sample in decoded], [1.0, 2.0])
        self.assertAlmostEqual(decoded[1][0], 0.04)

    def test_battle_probability_is_invariant_over_equal_elapsed_time(self) -> None:
        per_step = 0.04
        elapsed_probability = RaceEngine._probability_for_elapsed_time(per_step, 0.10)
        self.assertAlmostEqual(elapsed_probability, 1.0 - (1.0 - per_step) ** 5)

    def test_speed_command_accepts_two_and_rejects_three(self) -> None:
        class FakeEngine:
            speed_multiplier = 1

            def set_speed(self, multiplier: int) -> bool:
                if multiplier not in (1, 2):
                    return False
                self.speed_multiplier = multiplier
                return True

        session = RaceSession("test", FakeEngine(), None, None, [])
        accepted = asyncio.run(session.handle_command({"type": "set_speed", "multiplier": 2}))
        rejected = asyncio.run(session.handle_command({"type": "set_speed", "multiplier": 3}))
        self.assertEqual(accepted["type"], "command_ack")
        self.assertEqual(rejected["type"], "command_error")

    def test_close_releases_loop_clients_and_trajectory_buffers(self) -> None:
        class FakeEngine:
            finished = False

        class FakeSocket:
            def __init__(self) -> None:
                self.closed = False

            async def close(self, **_kwargs) -> None:
                self.closed = True

        async def exercise_close() -> None:
            session = RaceSession("test", FakeEngine(), None, None, [])
            socket = FakeSocket()
            session.clients.add(socket)
            session._trajectory_samples[1] = []
            loop_task = asyncio.create_task(asyncio.sleep(60))
            session._loop_task = loop_task

            await session.close()

            self.assertTrue(session.engine.finished)
            self.assertTrue(loop_task.cancelled())
            self.assertIsNone(session._loop_task)
            self.assertEqual(session.clients, set())
            self.assertEqual(session._trajectory_samples, {})
            self.assertTrue(socket.closed)

        asyncio.run(exercise_close())


if __name__ == "__main__":
    unittest.main()
