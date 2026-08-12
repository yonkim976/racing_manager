"""Fast contracts for the reusable circuit runtime baseline diagnostic."""

from __future__ import annotations

import unittest

from models.schemas import DryTireRole, PaceMode, ThermalPresetName
from data_loader import load_circuits, load_drivers, load_teams
from engines.full.runtime.race_engine import RaceEngine
from tools.run_circuit_baseline import (
    _corner_windows,
    _line_relative_lateral_error_m,
    _reference_lateral_speed_candidate,
    _signed_error_metrics,
    build_report,
)


class CircuitRuntimeBaselineTests(unittest.TestCase):
    class _SyntheticLine:
        def __init__(self, offset_fn):
            self.offset_fn = offset_fn

        def line_offset_at_progress(self, _line, progress):
            return self.offset_fn(progress)

    def _engine(
        self,
        *,
        clean_line_lateral_speed_feedforward: float | None = None,
    ) -> RaceEngine:
        circuit = next(item for item in load_circuits() if item.id == 4)
        drivers = load_drivers()
        teams = {team.id: team for team in load_teams()}
        kwargs = {}
        if clean_line_lateral_speed_feedforward is not None:
            kwargs["clean_line_lateral_speed_feedforward"] = (
                clean_line_lateral_speed_feedforward
            )
        return RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=1,
            player_driver_ids=[1, 2],
            seed=42,
            start_sequence_enabled=False,
            **kwargs,
        )

    def test_coordinate_contract_keeps_centerline_and_line_relative_offsets_separate(self) -> None:
        self.assertAlmostEqual(_line_relative_lateral_error_m(3.0, 1.25), 1.75)
        self.assertAlmostEqual(_line_relative_lateral_error_m(-2.0, 1.25), -3.25)

    def test_straight_reference_has_no_unnecessary_lateral_speed_candidate(self) -> None:
        profile = self._SyntheticLine(lambda _progress: 1.5)
        self.assertAlmostEqual(
            _reference_lateral_speed_candidate(
                profile,
                "racing_line",
                0.35,
                60.0,
                4000.0,
            ),
            0.0,
            places=12,
        )

    def test_reference_offset_direction_sets_candidate_direction(self) -> None:
        profile = self._SyntheticLine(lambda progress: 0.25 + 0.8 * progress)
        self.assertGreater(
            _reference_lateral_speed_candidate(
                profile,
                "racing_line",
                0.35,
                60.0,
                4000.0,
            ),
            0.0,
        )

    def test_clean_line_feed_forward_is_enabled_after_product_approval(self) -> None:
        engine = self._engine()
        self.assertEqual(engine.clean_line_lateral_speed_feedforward, 1.0)

    def test_clean_line_target_and_reference_speeds_share_the_same_transport(self) -> None:
        engine = self._engine(clean_line_lateral_speed_feedforward=1.0)
        state = engine.driver_states[1]
        profile = engine._track_physics_for_driver(state)
        line = "racing_line"
        sample_distance_progress = 1.0 / engine.track_length_m
        progress = max(
            (sample.center_progress for sample in profile.racing_line_samples),
            key=lambda candidate: abs(
                profile.line_offset_at_progress(
                    line,
                    candidate + sample_distance_progress,
                )
                - profile.line_offset_at_progress(
                    line,
                    candidate - sample_distance_progress,
                )
            ),
        )
        state.progress = progress
        state.total_progress = 2.0 + progress
        state.speed_kph = 220.0

        target_speed, reference_speed = engine._physics_v2_lateral_speed_inputs(
            state,
            line,
            profile,
            0.0,
            tactical_lateral_motion=False,
        )

        self.assertNotAlmostEqual(reference_speed, 0.0, places=6)
        self.assertAlmostEqual(target_speed, reference_speed, places=9)

    def test_legacy_zero_transport_is_available_only_as_diagnostic_replay(self) -> None:
        engine = self._engine(clean_line_lateral_speed_feedforward=0.0)
        state = engine.driver_states[1]
        profile = engine._track_physics_for_driver(state)

        target_speed, reference_speed = engine._physics_v2_lateral_speed_inputs(
            state,
            "racing_line",
            profile,
            0.35,
            tactical_lateral_motion=False,
        )

        self.assertEqual(target_speed, 0.35)
        self.assertEqual(reference_speed, 0.0)

    def test_smoothed_reversal_metric_detects_gradual_direction_change(self) -> None:
        errors = [0.5 + index * 0.005 for index in range(200)]
        errors.extend(1.5 - index * 0.005 for index in range(200))
        samples = [
            {
                "total_progress": 2.0 + index / 10000.0,
                "line_relative_lateral_error_m": error,
            }
            for index, error in enumerate(errors)
        ]

        metrics = _signed_error_metrics(samples)

        self.assertEqual(metrics["correction_reversals"], 1)

    def test_smoothed_reversal_metric_resets_at_lap_boundary(self) -> None:
        samples = [
            {
                "total_progress": 2.0 + index / 10000.0,
                "line_relative_lateral_error_m": 0.5 + index * 0.01,
            }
            for index in range(100)
        ]
        samples.extend(
            {
                "total_progress": 3.0 + index / 10000.0,
                "line_relative_lateral_error_m": 1.5 - index * 0.01,
            }
            for index in range(100)
        )

        metrics = _signed_error_metrics(samples)

        self.assertEqual(metrics["correction_reversals"], 0)

    def test_non_authoritative_step_inputs_consume_the_same_fixed_steps(self) -> None:
        two_ten_ms = self._engine()
        two_ten_ms.tick(0.01)
        two_ten_ms.tick(0.01)

        one_twenty_ms = self._engine()
        one_twenty_ms.tick(0.02)

        fifty_ms = self._engine()
        fifty_ms.tick(0.05)
        fifty_ms.tick(0.01)

        three_twenty_ms = self._engine()
        three_twenty_ms.tick(0.02)
        three_twenty_ms.tick(0.02)
        three_twenty_ms.tick(0.02)

        for first, second in zip(
            (
                two_ten_ms.driver_states[1],
                fifty_ms.driver_states[1],
            ),
            (
                one_twenty_ms.driver_states[1],
                three_twenty_ms.driver_states[1],
            ),
        ):
            self.assertAlmostEqual(first.total_progress, second.total_progress, places=9)
            self.assertAlmostEqual(first.lateral_offset_m, second.lateral_offset_m, places=9)
            self.assertAlmostEqual(first.speed_kph, second.speed_kph, places=9)

    def test_adjacent_red_bull_ring_corner_windows_do_not_share_samples(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 4)
        windows = {
            item["corner"]: item
            for item in _corner_windows(circuit)
        }

        self.assertAlmostEqual(
            windows["T4"]["exit_end_progress"],
            windows["T5"]["approach_start_progress"],
        )
        self.assertAlmostEqual(
            windows["T5"]["exit_end_progress"],
            windows["T6"]["approach_start_progress"],
        )

    def test_red_bull_ring_report_uses_normal_environment_and_fixed_steps(self) -> None:
        report = build_report(
            circuit_id=4,
            thermal_preset=ThermalPresetName.NORMAL,
            laps=5,
            seed=42,
            pace=1.04,
            driver_id=None,
            tire_role=DryTireRole.SOFT,
            pace_mode=PaceMode.ATTACK,
        )

        self.assertEqual(report["environment"]["ambient_temperature_c"], 18.0)
        self.assertEqual(report["environment"]["track_temperature_c"], 30.0)
        self.assertEqual(report["nomination"]["hard"], "C2")
        self.assertEqual(report["nomination"]["medium"], "C3")
        self.assertEqual(report["nomination"]["soft"], "C4")
        self.assertEqual(report["single_car_dynamic"]["input_contract"]["tire_role"], "SOFT")
        self.assertEqual(report["single_car_dynamic"]["input_contract"]["physical_tire_compound"], "C4")
        self.assertEqual(report["single_car_dynamic"]["input_contract"]["pace_mode"], "ATTACK")
        self.assertEqual(report["single_car_dynamic"]["physics_step_seconds"], 0.02)
        self.assertEqual(report["single_car_dynamic"]["physics_hz"], 50)
        self.assertEqual(report["single_car_dynamic"]["laps_completed"], 5)
        self.assertEqual(
            report["single_car_dynamic"]["input_contract"]["fuel_basis"],
            "full_race_distance",
        )
        self.assertEqual(
            report["single_car_dynamic"]["input_contract"]["fuel_basis_laps"],
            71,
        )
        self.assertGreater(
            report["single_car_dynamic"]["input_contract"]["fuel_start_kg"],
            50.0,
        )
        self.assertEqual(
            report["single_car_dynamic"]["speed_opportunities"]["status"],
            "pass",
        )
        safety = report["single_car_dynamic"]["safety"]
        self.assertEqual(safety["off_track_samples"], 0)
        self.assertEqual(safety["track_limit_samples"], 0)
        self.assertEqual(safety["run_wide_samples"], 0)
        self.assertEqual(safety["event_counts"].get("traction_loss", 0), 0)

    def test_report_exposes_sector_lineage_and_static_thresholds(self) -> None:
        report = build_report(
            circuit_id=4,
            thermal_preset=ThermalPresetName.NORMAL,
            laps=2,
            seed=42,
            pace=1.04,
            driver_id=None,
        )

        source = report["sector_timing_source"]
        self.assertIn("fia.com", source["source_url"])
        self.assertEqual(source["sector_lengths_m"], [1215.0, 1697.0, 1414.0])
        self.assertTrue(report["tier_a_audit"]["sectors_have_timing_ranges"])
        self.assertLess(report["static_telemetry_metrics"]["mae_kph"], 6.0)
        self.assertLess(report["static_telemetry_metrics"]["rmse_kph"], 9.0)


if __name__ == "__main__":
    unittest.main()
