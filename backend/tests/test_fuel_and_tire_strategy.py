"""Thermal tyre, fuel-mass and AI resource-management regressions."""

from __future__ import annotations

import unittest

from data_loader import load_circuits, load_drivers, load_teams
from models.schemas import PaceMode, TireCompound
from engines.full.runtime.brake_model import (
    advance_brake_thermal_state,
    brake_temperature_force_factor,
)
from engines.full.runtime.race_engine import RaceEngine
from engines.full.runtime.tire_model import (
    advance_tire_thermal_state,
    compute_tire_physics_factors,
    tire_temperature_grip_factor,
)


class FuelAndTireStrategyTests(unittest.TestCase):
    def _engine(self) -> RaceEngine:
        circuit = next(item for item in load_circuits() if item.id == 3)
        drivers = load_drivers()
        teams = {team.id: team for team in load_teams()}
        return RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=drivers[0].team_id,
            player_driver_ids=[drivers[0].id],
            seed=37,
            start_sequence_enabled=False,
        )

    def test_cold_and_overheated_tires_have_less_grip_than_optimal(self) -> None:
        optimal = tire_temperature_grip_factor(TireCompound.MEDIUM, 100.0)
        cold = tire_temperature_grip_factor(TireCompound.MEDIUM, 50.0)
        hot = tire_temperature_grip_factor(TireCompound.MEDIUM, 145.0)
        cold_factors = compute_tire_physics_factors(
            TireCompound.MEDIUM,
            0.0,
            temperature_c=50.0,
        )
        optimal_factors = compute_tire_physics_factors(
            TireCompound.MEDIUM,
            0.0,
            temperature_c=100.0,
        )

        self.assertGreater(optimal, cold)
        self.assertGreater(optimal, hot)
        self.assertLess(cold_factors.lateral_grip, optimal_factors.lateral_grip)
        self.assertLess(cold_factors.braking_grip, optimal_factors.braking_grip)

    def test_slide_energy_heats_surface_more_than_clean_running(self) -> None:
        common = dict(
            compound=TireCompound.MEDIUM,
            surface_temperature_c=90.0,
            core_temperature_c=90.0,
            delta_seconds=1.0,
            speed_mps=50.0,
            lateral_acceleration_mps2=8.0,
            throttle=0.8,
            brake=0.0,
            ambient_temperature_c=30.0,
            track_temperature_c=40.0,
        )
        clean = advance_tire_thermal_state(**common, slide_energy_j=0.0)
        sliding = advance_tire_thermal_state(**common, slide_energy_j=20000.0)

        self.assertGreater(
            sliding.surface_temperature_c,
            clean.surface_temperature_c,
        )

    def test_heavy_braking_heats_discs_more_than_coasting(self) -> None:
        common = dict(
            front_temperature_c=400.0,
            rear_temperature_c=400.0,
            delta_seconds=1.0,
            speed_mps=70.0,
            front_brake_bias=0.58,
            ambient_temperature_c=30.0,
        )
        coasting = advance_brake_thermal_state(
            **common,
            applied_brake_force_n=0.0,
        )
        braking = advance_brake_thermal_state(
            **common,
            applied_brake_force_n=40_000.0,
        )

        self.assertGreater(
            braking.front_temperature_c,
            coasting.front_temperature_c,
        )
        self.assertGreater(
            braking.rear_temperature_c,
            coasting.rear_temperature_c,
        )
        self.assertGreater(
            braking.front_temperature_c,
            braking.rear_temperature_c,
        )

    def test_extreme_brake_temperature_reduces_available_force(self) -> None:
        optimal = brake_temperature_force_factor(500.0, 450.0)
        overheated = brake_temperature_force_factor(1100.0, 1000.0)

        self.assertEqual(optimal, 1.0)
        self.assertLess(overheated, optimal)

    def test_repeated_race_braking_keeps_both_axles_in_operating_range(
        self,
    ) -> None:
        front = 400.0
        rear = 400.0
        thermal = None
        # Deterministic 90-second lap surrogate: eight braking zones followed
        # by airflow cooling. It protects the long-run equilibrium calibration
        # without tying the unit test to a particular circuit trajectory.
        for _ in range(52):
            for _ in range(8):
                for _ in range(15):
                    thermal = advance_brake_thermal_state(
                        front_temperature_c=front,
                        rear_temperature_c=rear,
                        delta_seconds=0.1,
                        speed_mps=55.0,
                        applied_brake_force_n=16_000.0,
                        front_brake_bias=0.58,
                        ambient_temperature_c=30.0,
                    )
                    front = thermal.front_temperature_c
                    rear = thermal.rear_temperature_c
                for _ in range(95):
                    thermal = advance_brake_thermal_state(
                        front_temperature_c=front,
                        rear_temperature_c=rear,
                        delta_seconds=0.1,
                        speed_mps=48.0,
                        applied_brake_force_n=0.0,
                        front_brake_bias=0.58,
                        ambient_temperature_c=30.0,
                    )
                    front = thermal.front_temperature_c
                    rear = thermal.rear_temperature_c

        self.assertIsNotNone(thermal)
        self.assertGreater(front, 250.0)
        self.assertGreater(rear, 250.0)
        self.assertLess(front, 950.0)
        self.assertLess(rear, 950.0)
        self.assertGreater(front, rear)
        self.assertEqual(thermal.fade_factor, 1.0)

    def test_fuel_load_changes_mass_and_is_consumed_by_physics(self) -> None:
        engine = self._engine()
        state = next(iter(engine.driver_states.values()))
        dry_mass_kg = engine._driver_meta[state.driver_id]["car_factors"].mass_kg
        starting_fuel_kg = state.fuel_mass_kg

        self.assertGreater(starting_fuel_kg, 100.0)
        self.assertAlmostEqual(
            engine._physics_v2_modifiers(state).mass_kg,
            dry_mass_kg + starting_fuel_kg,
        )
        engine.tick(0.1)

        self.assertLess(state.fuel_mass_kg, starting_fuel_kg)
        self.assertGreater(state.fuel_burned_kg, 0.0)
        self.assertGreater(state.front_normal_load_n, 0.0)
        self.assertGreater(state.rear_normal_load_n, 0.0)
        self.assertGreaterEqual(state.front_brake_temperature_c, 30.0)
        self.assertGreaterEqual(state.rear_brake_temperature_c, 30.0)

    def test_live_dashboard_exposes_and_refreshes_vehicle_condition(self) -> None:
        engine = self._engine()
        driver_id = next(iter(engine.driver_states))

        engine.tick(0.1)
        first_position = next(
            position
            for position in engine.build_dashboard_payload()["positions"]
            if position["driver_id"] == driver_id
        )
        for _ in range(10):
            engine.tick(0.1)
        second_position = next(
            position
            for position in engine.build_dashboard_payload()["positions"]
            if position["driver_id"] == driver_id
        )

        required_fields = {
            "tire_surface_temperature_c",
            "tire_core_temperature_c",
            "tire_thermal_grip",
            "front_tire_surface_temperature_c",
            "front_tire_core_temperature_c",
            "front_tire_thermal_grip",
            "rear_tire_surface_temperature_c",
            "rear_tire_core_temperature_c",
            "rear_tire_thermal_grip",
            "fuel_mass_kg",
            "fuel_burned_kg",
            "fuel_laps_remaining",
            "vehicle_mass_kg",
            "front_normal_load_n",
            "rear_normal_load_n",
            "front_axle_slip_ratio",
            "rear_axle_slip_ratio",
            "front_brake_temperature_c",
            "rear_brake_temperature_c",
            "brake_fade_factor",
        }
        self.assertTrue(required_fields.issubset(first_position))
        self.assertLess(
            second_position["fuel_mass_kg"],
            first_position["fuel_mass_kg"],
        )
        self.assertNotEqual(
            second_position["tire_surface_temperature_c"],
            first_position["tire_surface_temperature_c"],
        )
        self.assertNotEqual(
            second_position["front_tire_surface_temperature_c"],
            second_position["rear_tire_surface_temperature_c"],
        )

    def test_ai_conserves_when_projected_fuel_margin_is_critical(self) -> None:
        engine = self._engine()
        state = next(
            item
            for item in engine.driver_states.values()
            if item.driver_id not in engine.player_driver_ids
        )
        remaining_laps = engine.total_laps - state.current_lap
        state.fuel_laps_remaining = remaining_laps + 0.2

        mode = engine._choose_ai_pace_mode(state, None, None)

        self.assertEqual(mode, PaceMode.CONSERVE)


if __name__ == "__main__":
    unittest.main()
