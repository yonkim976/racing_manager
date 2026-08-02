"""Circuit thermal profile, session resolution and environment sensitivity tests."""

from __future__ import annotations

import copy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from data_loader import load_circuits
from models.schemas import (
    CircuitThermalProfile,
    RaceSetupRequest,
    ThermalPresetName,
    TireCompound,
    TrackConditions,
)
from session import SessionManager
from simulation.brake_model import advance_brake_thermal_state
from simulation.tire_model import advance_tire_thermal_state


BACKEND_DIR = Path(__file__).resolve().parents[1]


class CircuitThermalProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.circuits = load_circuits()

    def test_all_registered_circuits_have_exactly_three_ordered_presets(self) -> None:
        self.assertEqual({c.id for c in self.circuits}, {3, 4, 5, 6, 7, 8, 9, 10, 11})
        for circuit in self.circuits:
            self.assertEqual(
                set(circuit.thermal_profile.presets),
                set(ThermalPresetName),
            )
            profile = circuit.thermal_profile
            self.assertIn(profile.default_preset, profile.presets)
            for lower, upper in zip(ThermalPresetName, (ThermalPresetName.NORMAL, ThermalPresetName.HOT)):
                self.assertLessEqual(
                    profile.presets[lower].ambient_temperature_c,
                    profile.presets[upper].ambient_temperature_c,
                )
                self.assertLessEqual(
                    profile.presets[lower].track_temperature_c,
                    profile.presets[upper].track_temperature_c,
                )

    def test_bahrain_profile_preserves_thermal_regression_baseline(self) -> None:
        bahrain = next(c for c in self.circuits if c.id == 3)
        profile = bahrain.thermal_profile.presets
        self.assertEqual(
            (profile[ThermalPresetName.COOL].ambient_temperature_c,
             profile[ThermalPresetName.COOL].track_temperature_c),
            (24.0, 32.0),
        )
        self.assertEqual(
            (profile[ThermalPresetName.NORMAL].ambient_temperature_c,
             profile[ThermalPresetName.NORMAL].track_temperature_c),
            (30.0, 40.0),
        )
        self.assertEqual(
            (profile[ThermalPresetName.HOT].ambient_temperature_c,
             profile[ThermalPresetName.HOT].track_temperature_c),
            (36.0, 52.0),
        )

    def test_geometry_track_conditions_remain_separate_from_thermal_profile(self) -> None:
        self.assertTrue(all(isinstance(circuit.track_conditions, list) for circuit in self.circuits))
        self.assertTrue(all(hasattr(circuit, "thermal_profile") for circuit in self.circuits))

    def test_loader_rejects_duplicate_missing_and_unknown_profile_ids(self) -> None:
        circuits = json.loads((BACKEND_DIR / "data" / "circuits.json").read_text())
        profiles = json.loads(
            (BACKEND_DIR / "data" / "circuit_thermal_profiles.json").read_text()
        )

        cases = {
            "duplicate": profiles + [copy.deepcopy(profiles[0])],
            "missing": profiles[:-1],
            "unknown": profiles + [{**copy.deepcopy(profiles[0]), "circuit_id": 999}],
        }
        for name, invalid_profiles in cases.items():
            with self.subTest(name=name):
                with patch(
                    "data_loader._load_json",
                    side_effect=[circuits, invalid_profiles],
                ):
                    with self.assertRaises(ValueError):
                        load_circuits()

    def test_profile_model_rejects_missing_presets_reversed_order_and_bad_values(self) -> None:
        normal = TrackConditions(ambient_temperature_c=30.0, track_temperature_c=40.0)
        cool = TrackConditions(ambient_temperature_c=24.0, track_temperature_c=32.0)
        hot = TrackConditions(ambient_temperature_c=36.0, track_temperature_c=52.0)
        with self.assertRaises(ValueError):
            CircuitThermalProfile(presets={ThermalPresetName.NORMAL: normal})
        with self.assertRaises(ValueError):
            CircuitThermalProfile(
                presets={
                    ThermalPresetName.COOL: hot,
                    ThermalPresetName.NORMAL: normal,
                    ThermalPresetName.HOT: cool,
                }
            )
        with self.assertRaises(ValueError):
            TrackConditions(ambient_temperature_c=101.0, track_temperature_c=40.0)

    def test_session_resolves_default_requested_and_explicit_override_once(self) -> None:
        from data_loader import load_drivers, load_teams

        drivers = load_drivers()
        teams = load_teams()
        manager = SessionManager()
        try:
            default_session = manager.create_session(
                RaceSetupRequest(circuit_id=3, player_team_id=1, total_laps=5),
                drivers,
                teams,
                self.circuits,
            )
            self.assertEqual(default_session.resolved_thermal_preset, ThermalPresetName.NORMAL)
            self.assertEqual(default_session.resolved_track_conditions.ambient_temperature_c, 30.0)
            self.assertEqual(default_session.track_conditions_source, "circuit_preset")

            hot_session = manager.create_session(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    thermal_preset=ThermalPresetName.HOT,
                ),
                drivers,
                teams,
                self.circuits,
            )
            self.assertEqual(hot_session.engine.track_conditions.ambient_temperature_c, 36.0)
            self.assertEqual(hot_session.race_info.environment_conditions.track_temperature_c, 52.0)
            self.assertEqual(hot_session.engine.build_tick_state().thermal_preset, ThermalPresetName.HOT)
            self.assertEqual(hot_session.engine.diagnostic_counts()["thermal_preset"], "HOT")

            override = TrackConditions(ambient_temperature_c=19.0, track_temperature_c=27.0)
            override_session = manager.create_session(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    track_conditions=override,
                ),
                drivers,
                teams,
                self.circuits,
            )
            self.assertEqual(override_session.engine.track_conditions, override)
            self.assertIsNone(override_session.resolved_thermal_preset)
            self.assertEqual(override_session.track_conditions_source, "explicit_override")

            both = RaceSetupRequest(
                circuit_id=3,
                player_team_id=1,
                thermal_preset=ThermalPresetName.HOT,
                track_conditions=override,
            )
            with self.assertRaisesRegex(ValueError, "mutually exclusive"):
                manager.create_session(both, drivers, teams, self.circuits)
        finally:
            manager.clear()

    def test_tire_environment_sensitivity_keeps_energy_input_identical(self) -> None:
        results = []
        heat_fields = (
            "baseline_heat_j",
            "lateral_heat_j",
            "braking_heat_j",
            "traction_heat_j",
            "slide_heat_j",
        )
        for ambient, track in ((24.0, 32.0), (30.0, 40.0), (36.0, 52.0)):
            surface = core = 80.0
            cumulative_heat = {field: 0.0 for field in heat_fields}
            for _ in range(3000):
                state = advance_tire_thermal_state(
                    TireCompound.MEDIUM,
                    surface_temperature_c=surface,
                    core_temperature_c=core,
                    delta_seconds=0.02,
                    speed_mps=45.0,
                    lateral_acceleration_mps2=2.5,
                    throttle=0.6,
                    brake=0.0,
                    slide_energy_j=0.0,
                    ambient_temperature_c=ambient,
                    track_temperature_c=track,
                )
                surface, core = state.surface_temperature_c, state.core_temperature_c
                for field in heat_fields:
                    cumulative_heat[field] += getattr(state.budget, field)
            self.assertFalse(state.surface_clamp_hit)
            self.assertFalse(state.core_clamp_hit)
            results.append((surface, core, cumulative_heat))

        self.assertLess(results[0][0], results[1][0])
        self.assertLess(results[1][0], results[2][0])
        self.assertLess(results[0][1], results[1][1])
        self.assertLess(results[1][1], results[2][1])
        for field in heat_fields:
            cool_total = results[0][2][field]
            normal_total = results[1][2][field]
            hot_total = results[2][2][field]
            tolerance = max(1e-6, abs(normal_total) * 0.01)
            self.assertAlmostEqual(cool_total, normal_total, delta=tolerance)
            self.assertAlmostEqual(hot_total, normal_total, delta=tolerance)

    def test_brake_environment_sensitivity_uses_ambient_as_cooling_reference(self) -> None:
        final_temperatures = []
        for ambient in (24.0, 30.0, 36.0):
            front = rear = 700.0
            for _ in range(100):
                state = advance_brake_thermal_state(
                    front_temperature_c=front,
                    rear_temperature_c=rear,
                    delta_seconds=0.1,
                    speed_mps=50.0,
                    applied_brake_force_n=0.0,
                    front_brake_bias=0.58,
                    ambient_temperature_c=ambient,
                )
                front, rear = state.front_temperature_c, state.rear_temperature_c
            final_temperatures.append(front)

        self.assertLess(final_temperatures[0], final_temperatures[1])
        self.assertLess(final_temperatures[1], final_temperatures[2])


if __name__ == "__main__":
    unittest.main()
