"""Thermal tyre, fuel-mass and AI resource-management regressions."""

from __future__ import annotations

import unittest

from data_loader import load_circuits, load_drivers, load_teams
from models.schemas import PaceMode, TireCompound
from simulation.race_engine import RaceEngine
from simulation.tire_model import (
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
        )
        clean = advance_tire_thermal_state(**common, slide_energy_j=0.0)
        sliding = advance_tire_thermal_state(**common, slide_energy_j=20000.0)

        self.assertGreater(
            sliding.surface_temperature_c,
            clean.surface_temperature_c,
        )

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
