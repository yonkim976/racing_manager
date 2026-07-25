"""Regression gates for measured Bahrain speed and braking calibration."""

from __future__ import annotations

import json
import math
import statistics
import unittest
from pathlib import Path

from data_loader import load_circuits, load_drivers, load_teams
from simulation.race_engine import RaceEngine
from simulation.physics import GAME_TICK_SECONDS
from simulation.track_physics import DRIVING_LINE_RACING
from simulation.vehicle_physics import (
    LOCKUP_SLIP_RATIO_THRESHOLD,
    TRACTION_LOSS_SLIP_RATIO_THRESHOLD,
    VehiclePhysicsModifiers,
)
from tools.compare_bahrain_telemetry import QUALIFYING_TELEMETRY_PACE

BACKEND_DIR = Path(__file__).resolve().parents[1]


class BahrainTelemetryCalibrationTests(unittest.TestCase):
    def test_t1_keeps_official_total_width_without_folding_inside_edge(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 3)
        t1_apex = next(
            sample
            for sample in circuit.track_width_profile
            if math.isclose(sample.progress, 0.125)
        )

        self.assertEqual(t1_apex.left_width_m, 7.0)
        self.assertEqual(t1_apex.right_width_m, 15.0)
        self.assertEqual(t1_apex.left_width_m + t1_apex.right_width_m, 22.0)

    def test_bahrain_keeps_a_measured_five_lap_reference(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 3)
        calibration = circuit.physics_calibration
        assert calibration is not None

        self.assertEqual(calibration.source, "TracingInsights-Archive/2025")
        self.assertEqual(len(calibration.reference_laps), 5)
        self.assertEqual(len(calibration.telemetry_reference), 128)
        self.assertEqual(calibration.planner_braking_utilization, 0.55)
        self.assertEqual(calibration.telemetry_max_braking_utilization, 0.8)
        self.assertEqual(calibration.telemetry_braking_speed_reserve, 0.98)
        self.assertEqual(calibration.braking_longitudinal_grip_factor, 1.35)
        self.assertEqual(calibration.brake_control_error_fraction, 0.06)
        self.assertEqual(calibration.controller_sample_distance_m, 12.5)
        self.assertEqual(calibration.telemetry_speed_reference_weight, 1.0)
        self.assertTrue(
            any(sample.braking_fraction >= 0.8 for sample in calibration.telemetry_reference)
        )

    def test_physical_target_profile_stays_close_to_measured_speed_shape(self) -> None:
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
        calibration = circuit.physics_calibration
        assert calibration is not None
        modifiers = VehiclePhysicsModifiers(pace=QUALIFYING_TELEMETRY_PACE)
        errors = [
            physics.target_speed_mps(
                sample.progress * physics.track_length_m,
                modifiers,
            )
            * 3.6
            - sample.speed_kph
            for sample in calibration.telemetry_reference
        ]

        self.assertLess(statistics.fmean(abs(error) for error in errors), 5.0)
        self.assertLess(abs(statistics.fmean(errors)), 1.0)

    def test_integrated_bahrain_lap_brakes_cleanly_through_turn_one(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 3)
        driver = load_drivers()[0]
        engine = RaceEngine(
            circuit=circuit,
            drivers=[driver],
            teams={team.id: team for team in load_teams()},
            player_team_id=driver.team_id,
            player_driver_ids=[driver.id],
            seed=2026,
            start_sequence_enabled=False,
        )
        state = engine.driver_states[driver.id]
        status_ticks = {
            "off_track": 0,
            "track_limits": 0,
            "run_wide": 0,
            "lockup": 0,
            "wheelspin": 0,
        }
        turn_one_minimum_kph = float("inf")
        body_heading_errors_rad: list[float] = []
        maximum_lateral_acceleration_mps2 = 0.0

        for _ in range(round(120.0 / GAME_TICK_SECONDS)):
            engine.tick(GAME_TICK_SECONDS)
            status_ticks["off_track"] += int(state.off_track)
            status_ticks["track_limits"] += int(state.track_limits_active)
            status_ticks["run_wide"] += int(state.handling_state == "run_wide")
            status_ticks["lockup"] += int(
                state.wheel_lock_ratio >= LOCKUP_SLIP_RATIO_THRESHOLD
            )
            status_ticks["wheelspin"] += int(
                state.traction_slip_ratio >= TRACTION_LOSS_SLIP_RATIO_THRESHOLD
            )
            body_heading_errors_rad.append(abs(state.slip_angle_rad))
            maximum_lateral_acceleration_mps2 = max(
                maximum_lateral_acceleration_mps2,
                abs(state.lateral_acceleration_mps2),
            )
            if 0.13 <= state.progress <= 0.14:
                turn_one_minimum_kph = min(
                    turn_one_minimum_kph,
                    state.speed_kph,
                )

        self.assertEqual(status_ticks, {key: 0 for key in status_ticks})
        self.assertLess(turn_one_minimum_kph, 100.0)
        ordered_heading_errors = sorted(body_heading_errors_rad)
        heading_p95 = ordered_heading_errors[
            round((len(ordered_heading_errors) - 1) * 0.95)
        ]
        self.assertLessEqual(heading_p95, math.radians(6.1))
        self.assertLessEqual(maximum_lateral_acceleration_mps2, 35.0)

    def test_corner_report_has_all_fifteen_turns_and_bounded_minimum_speed_error(self) -> None:
        report = json.loads(
            (BACKEND_DIR / "data/calibration/bahrain_2025_qualifying.json").read_text(
                encoding="utf-8"
            )
        )
        corners = report["corner_comparison"]
        self.assertEqual([item["corner"] for item in corners], list(range(1, 16)))
        self.assertLessEqual(
            max(abs(item["minimum_speed_error_kph"]) for item in corners),
            16.0,
        )


class RedBullRingTelemetryCalibrationTests(unittest.TestCase):
    def test_red_bull_ring_loads_measured_width_and_five_lap_reference(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 4)
        calibration = circuit.physics_calibration
        assert calibration is not None

        self.assertEqual(len(circuit.track_width_profile), 64)
        total_widths = [
            sample.left_width_m + sample.right_width_m
            for sample in circuit.track_width_profile
        ]
        self.assertGreaterEqual(min(total_widths), 10.0)
        self.assertLessEqual(max(total_widths), 13.5)
        self.assertEqual(len(calibration.reference_laps), 5)
        self.assertEqual(len(calibration.telemetry_reference), 128)
        self.assertEqual(calibration.telemetry_speed_reference_weight, 1.0)

    def test_red_bull_ring_physical_profile_matches_qualifying_shape(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 4)
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
        track = engine._track_physics_for_driver(state)
        physics = engine._vehicle_physics_for_driver(state)[DRIVING_LINE_RACING]
        calibration = circuit.physics_calibration
        assert calibration is not None
        modifiers = VehiclePhysicsModifiers(pace=QUALIFYING_TELEMETRY_PACE)
        errors = [
            physics.target_speed_mps(
                track.line_distance_at_total_progress(
                    DRIVING_LINE_RACING,
                    sample.progress,
                ),
                modifiers,
            )
            * 3.6
            - sample.speed_kph
            for sample in calibration.telemetry_reference
        ]
        mae = statistics.fmean(abs(error) for error in errors)
        rmse = math.sqrt(statistics.fmean(error * error for error in errors))

        self.assertLess(mae, 6.0)
        self.assertLess(rmse, 9.0)
        self.assertLess(abs(statistics.fmean(errors)), 4.0)
        self.assertLess(max(abs(error) for error in errors), 35.0)


if __name__ == "__main__":
    unittest.main()
