"""Tests for race-session delivery cadence."""

from __future__ import annotations

import unittest
import asyncio
from types import SimpleNamespace

from session import (
    BROADCAST_HZ,
    BROADCAST_INTERVAL,
    RaceSession,
    SessionCadenceMetrics,
)
from simulation.race_engine import RaceEngine
from simulation.vehicle_physics import PHYSICS_STEP_SECONDS


class RaceSessionCadenceTests(unittest.TestCase):
    def test_screen_state_broadcasts_at_thirty_hertz(self) -> None:
        self.assertEqual(BROADCAST_HZ, 30)
        self.assertAlmostEqual(BROADCAST_INTERVAL, 1.0 / 30.0)

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
            requested_speed_multiplier=3,
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
        self.assertEqual([sample.physics_frame for sample in samples], [1, 2])
        self.assertEqual([sample.world_x_m for sample in samples], [1.0, 2.0])
        self.assertAlmostEqual(samples[1].simulation_time_s, 0.04)

    def test_battle_probability_is_invariant_over_equal_elapsed_time(self) -> None:
        per_step = 0.04
        elapsed_probability = RaceEngine._probability_for_elapsed_time(per_step, 0.10)
        self.assertAlmostEqual(elapsed_probability, 1.0 - (1.0 - per_step) ** 5)

    def test_speed_command_accepts_three_and_rejects_five(self) -> None:
        class FakeEngine:
            speed_multiplier = 1

            def set_speed(self, multiplier: int) -> bool:
                if multiplier not in (1, 2, 3):
                    return False
                self.speed_multiplier = multiplier
                return True

        session = RaceSession("test", FakeEngine(), None, None, [])
        accepted = asyncio.run(session.handle_command({"type": "set_speed", "multiplier": 3}))
        rejected = asyncio.run(session.handle_command({"type": "set_speed", "multiplier": 5}))
        self.assertEqual(accepted["type"], "command_ack")
        self.assertEqual(rejected["type"], "command_error")


if __name__ == "__main__":
    unittest.main()
