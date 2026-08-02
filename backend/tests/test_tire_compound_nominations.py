"""C1-C5 physical compound and weekend nomination contracts."""

from __future__ import annotations

import copy
import json
import math
from dataclasses import replace
from pathlib import Path
import unittest
from unittest.mock import patch

from data_loader import (
    DATA_DIR,
    load_circuits,
    load_drivers,
    load_teams,
    resolve_tire_compound,
)
from models.schemas import (
    DryTireRole,
    PhysicalTireCompound,
    RaceSetupRequest,
    TireCompound,
    TireCompoundNomination,
)
from session import SessionManager
from simulation.qualifying import run_qualifying
from simulation.race_engine import RaceEngine
from simulation.state_contract import TickPhase
from simulation.tire_model import (
    COMPOUND_SPECS,
    LEGACY_TIRE_COMPOUND_TO_PHYSICAL,
    advance_tire_thermal_state,
    circuit_tire_usage_per_lap,
    compound_spec_for,
    compute_tire_physics_factors,
    compute_tire_performance,
    physical_compound_for_state,
    tire_grip_indices_c3,
    tire_thermal_wear_multiplier,
    tire_temperature_grip_factor,
)


class TireCompoundNominationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.circuits = load_circuits()

    def test_physical_codes_and_specs_are_complete(self) -> None:
        dry_codes = [PhysicalTireCompound.C1, PhysicalTireCompound.C2,
                     PhysicalTireCompound.C3, PhysicalTireCompound.C4,
                     PhysicalTireCompound.C5]
        self.assertEqual(set(COMPOUND_SPECS), set(PhysicalTireCompound))
        for code in PhysicalTireCompound:
            spec = COMPOUND_SPECS[code]
            self.assertEqual(spec.code, code)
            for value in (
                spec.grip,
                spec.degradation_rate,
                spec.cliff_threshold_laps,
                spec.lateral_grip,
                spec.traction_grip,
                spec.braking_grip,
                spec.optimal_surface_temperature_c,
                spec.surface_operating_window_c,
                spec.hot_diagnostic_threshold_c,
                spec.blanket_temperature_c,
            ):
                self.assertTrue(isinstance(value, (int, float)))
        self.assertEqual(
            [compound_spec_for(code).grip for code in dry_codes],
            sorted(compound_spec_for(code).grip for code in dry_codes),
        )
        self.assertEqual(
            [compound_spec_for(code).cliff_threshold_laps for code in dry_codes],
            sorted(
                (compound_spec_for(code).cliff_threshold_laps for code in dry_codes),
                reverse=True,
            ),
        )

    def test_specs_are_finite_positive_and_degradation_ordered(self) -> None:
        dry_codes = [
            PhysicalTireCompound.C1,
            PhysicalTireCompound.C2,
            PhysicalTireCompound.C3,
            PhysicalTireCompound.C4,
            PhysicalTireCompound.C5,
        ]
        for code in PhysicalTireCompound:
            spec = compound_spec_for(code)
            for value in (
                spec.grip,
                spec.degradation_rate,
                spec.cliff_threshold_laps,
                spec.lateral_grip,
                spec.traction_grip,
                spec.braking_grip,
                spec.optimal_surface_temperature_c,
                spec.surface_operating_window_c,
                spec.hot_diagnostic_threshold_c,
                spec.blanket_temperature_c,
            ):
                self.assertTrue(math.isfinite(value))
                self.assertGreater(value, 0.0)
        self.assertEqual(
            [compound_spec_for(code).degradation_rate for code in dry_codes],
            sorted(compound_spec_for(code).degradation_rate for code in dry_codes),
        )

        base = compound_spec_for(PhysicalTireCompound.C1)
        with self.assertRaises(ValueError):
            replace(base, grip=math.nan)
        with self.assertRaises(ValueError):
            replace(base, degradation_rate=-0.1)
        with self.assertRaises(ValueError):
            replace(base, hot_diagnostic_threshold_c=100.0)

    def test_nomination_requires_source_metadata(self) -> None:
        with self.assertRaises(ValueError):
            TireCompoundNomination(
                circuit_id=3,
                ruleset="2026_C1_C5",
                hard="C1",
                medium="C2",
                soft="C3",
                source_class="test",
                status="provisional",
            )

    def test_loader_rejects_nomination_duplicate_missing_and_unknown_ids(self) -> None:
        circuits = json.loads((Path(DATA_DIR) / "circuits.json").read_text())
        profiles = json.loads(
            (Path(DATA_DIR) / "circuit_thermal_profiles.json").read_text()
        )
        nominations = json.loads(
            (Path(DATA_DIR) / "tire_compound_nominations.json").read_text()
        )
        wear_profiles = json.loads(
            (Path(DATA_DIR) / "circuit_tire_wear_profiles.json").read_text()
        )
        cases = {
            "duplicate": {
                **nominations,
                "circuits": nominations["circuits"]
                + [copy.deepcopy(nominations["circuits"][0])],
            },
            "missing": {
                **nominations,
                "circuits": nominations["circuits"][:-1],
            },
            "unknown": {
                **nominations,
                "circuits": nominations["circuits"]
                + [{**copy.deepcopy(nominations["circuits"][0]), "circuit_id": 999}],
            },
        }
        for invalid in cases.values():
            with self.subTest(invalid=invalid):
                with patch(
                    "data_loader._load_json",
                    side_effect=[circuits, profiles, wear_profiles, invalid],
                ):
                    with self.assertRaises(ValueError):
                        load_circuits()

    def test_direct_engine_always_uses_monza_nomination_for_start_and_pit(self) -> None:
        circuit = next(item for item in self.circuits if item.id == 8)
        drivers = load_drivers()
        teams = {team.id: team for team in load_teams()}
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=1,
            player_driver_ids=[1, 2],
            starting_tires={1: TireCompound.SOFT},
            start_sequence_enabled=False,
        )
        state = engine.driver_states[1]
        self.assertEqual(circuit.tire_compound_nomination.soft, PhysicalTireCompound.C5)
        self.assertEqual(state.physical_tire_compound, PhysicalTireCompound.C5)
        self.assertEqual(
            engine.physical_compound_for_role(TireCompound.SOFT),
            PhysicalTireCompound.C5,
        )

        engine._tick_phase = TickPhase.RULES
        events = []
        engine._start_pit_stop(
            1,
            state,
            engine._driver_meta[1],
            teams[drivers[0].team_id],
            TireCompound.SOFT,
            events,
        )
        self.assertEqual(engine._pit_physical_tire[1], PhysicalTireCompound.C5)

    def test_physical_state_is_authoritative_over_legacy_fields(self) -> None:
        circuit = next(item for item in self.circuits if item.id == 8)
        drivers = load_drivers()
        teams = {team.id: team for team in load_teams()}
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=1,
            player_driver_ids=[1, 2],
            start_sequence_enabled=False,
        )
        state = engine.driver_states[1]
        physical_before_legacy_mutation = state.physical_tire_compound
        state.tire_compound = TireCompound.SOFT
        state.tire_role = DryTireRole.SOFT
        self.assertEqual(
            physical_compound_for_state(state),
            physical_before_legacy_mutation,
        )

    def test_legacy_dry_roles_migrate_one_to_one(self) -> None:
        self.assertEqual(
            LEGACY_TIRE_COMPOUND_TO_PHYSICAL,
            {
                TireCompound.HARD: PhysicalTireCompound.C1,
                TireCompound.MEDIUM: PhysicalTireCompound.C2,
                TireCompound.SOFT: PhysicalTireCompound.C3,
                TireCompound.INTER: PhysicalTireCompound.INTER,
                TireCompound.WET: PhysicalTireCompound.WET,
            },
        )
        self.assertEqual(
            compound_spec_for(TireCompound.HARD),
            compound_spec_for(PhysicalTireCompound.C1),
        )
        self.assertEqual(
            compound_spec_for(TireCompound.MEDIUM),
            compound_spec_for(PhysicalTireCompound.C2),
        )
        self.assertEqual(
            compound_spec_for(TireCompound.SOFT),
            compound_spec_for(PhysicalTireCompound.C3),
        )

    def test_compound_performance_and_temperature_curves_are_ordered(self) -> None:
        dry_codes = [
            PhysicalTireCompound.C1,
            PhysicalTireCompound.C2,
            PhysicalTireCompound.C3,
            PhysicalTireCompound.C4,
            PhysicalTireCompound.C5,
        ]
        fresh_pace = [compute_tire_performance(code, 0.0) for code in dry_codes]
        self.assertEqual(fresh_pace, sorted(fresh_pace))
        durability = [compute_tire_performance(code, 10.0) for code in dry_codes]
        self.assertGreater(durability[0], durability[-1])

        for code in dry_codes:
            spec = compound_spec_for(code)
            self.assertEqual(
                tire_temperature_grip_factor(code, spec.optimal_surface_temperature_c),
                1.0,
            )
            before = tire_temperature_grip_factor(
                code,
                spec.optimal_surface_temperature_c + spec.surface_operating_window_c - 0.001,
            )
            after = tire_temperature_grip_factor(
                code,
                spec.optimal_surface_temperature_c + spec.surface_operating_window_c + 0.001,
            )
            self.assertLessEqual(abs(before - after), 0.001)

    def test_live_grip_index_uses_fresh_optimal_c3_as_100(self) -> None:
        c3 = compound_spec_for(PhysicalTireCompound.C3)
        factors = compute_tire_physics_factors(
            PhysicalTireCompound.C3,
            0.0,
            temperature_c=c3.optimal_surface_temperature_c,
        )
        lateral, traction, braking = tire_grip_indices_c3(factors)
        self.assertAlmostEqual(lateral, 1.0)
        self.assertAlmostEqual(traction, 1.0)
        self.assertAlmostEqual(braking, 1.0)

    def test_circuit_wear_uses_distance_abrasion_and_bounded_thermal_stress(self) -> None:
        bahrain = next(circuit for circuit in self.circuits if circuit.id == 3)
        red_bull_ring = next(circuit for circuit in self.circuits if circuit.id == 4)
        bahrain_usage = circuit_tire_usage_per_lap(
            bahrain.track_length_m,
            bahrain.tire_wear_profile.abrasion_multiplier,
        )
        red_bull_ring_usage = circuit_tire_usage_per_lap(
            red_bull_ring.track_length_m,
            red_bull_ring.tire_wear_profile.abrasion_multiplier,
        )
        self.assertAlmostEqual(bahrain_usage, 1.0)
        self.assertLess(red_bull_ring_usage, bahrain_usage)
        self.assertGreater(
            compound_spec_for(PhysicalTireCompound.C4).cliff_threshold_laps
            / red_bull_ring_usage,
            16.0,
        )

        c4 = compound_spec_for(PhysicalTireCompound.C4)
        self.assertEqual(
            tire_thermal_wear_multiplier(
                PhysicalTireCompound.C4,
                c4.optimal_surface_temperature_c,
            ),
            1.0,
        )
        self.assertEqual(
            tire_thermal_wear_multiplier(
                PhysicalTireCompound.C4,
                c4.optimal_surface_temperature_c
                + c4.surface_operating_window_c,
            ),
            1.0,
        )
        self.assertLessEqual(
            tire_thermal_wear_multiplier(PhysicalTireCompound.C4, 160.0),
            1.30,
        )

    def test_same_input_keeps_thermal_energy_independent_of_compound(self) -> None:
        budgets = []
        for code in PhysicalTireCompound:
            result = advance_tire_thermal_state(
                code,
                surface_temperature_c=90.0,
                core_temperature_c=85.0,
                delta_seconds=0.02,
                speed_mps=60.0,
                lateral_acceleration_mps2=8.0,
                throttle=0.7,
                brake=0.1,
                slide_energy_j=25.0,
                ambient_temperature_c=30.0,
                track_temperature_c=40.0,
            )
            budgets.append(result.budget)
        self.assertTrue(all(budget == budgets[0] for budget in budgets))

    def test_all_loaded_circuits_have_valid_three_dry_nomination(self) -> None:
        self.assertEqual(len(self.circuits), 9)
        for circuit in self.circuits:
            nomination = circuit.tire_compound_nomination
            self.assertIsNotNone(nomination)
            self.assertEqual(nomination.ruleset, "2026_C1_C5")
            self.assertEqual(nomination.status, "provisional")
            self.assertEqual(len({nomination.hard, nomination.medium, nomination.soft}), 3)
            self.assertEqual(
                resolve_tire_compound(circuit, DryTireRole.HARD)[0], nomination.hard,
            )

    def test_bahrain_role_mapping_is_c1_c2_c3(self) -> None:
        bahrain = next(circuit for circuit in self.circuits if circuit.id == 3)
        self.assertEqual(resolve_tire_compound(bahrain, DryTireRole.HARD)[0], PhysicalTireCompound.C1)
        self.assertEqual(resolve_tire_compound(bahrain, DryTireRole.MEDIUM)[0], PhysicalTireCompound.C2)
        self.assertEqual(resolve_tire_compound(bahrain, TireCompound.SOFT)[0], PhysicalTireCompound.C3)

    def test_invalid_nomination_fails_validation(self) -> None:
        with self.assertRaises(ValueError):
            TireCompoundNomination(
                circuit_id=3,
                ruleset="2026_C1_C5",
                hard="C1",
                medium="C1",
                soft="C3",
                source_class="test",
                status="provisional",
            )

    def test_session_snapshot_resolves_bahrain_weekend_roles(self) -> None:
        manager = SessionManager()
        try:
            session = manager.create_session(
                RaceSetupRequest(
                    circuit_id=3,
                    player_team_id=1,
                    total_laps=5,
                    starting_tires={
                        1: TireCompound.SOFT,
                        2: TireCompound.HARD,
                    },
                ),
                load_drivers(),
                load_teams(),
                self.circuits,
            )
            self.assertEqual(
                session.engine.driver_states[1].physical_tire_compound,
                PhysicalTireCompound.C3,
            )
            self.assertEqual(
                session.engine.driver_states[2].physical_tire_compound,
                PhysicalTireCompound.C1,
            )
            self.assertEqual(
                session.engine.driver_states[3].physical_tire_compound,
                PhysicalTireCompound.C2,
            )
            self.assertEqual(
                session.race_info.tire_compound_nomination.medium,
                PhysicalTireCompound.C2,
            )
        finally:
            manager.clear()

    def test_bahrain_qualifying_reports_soft_role_and_c3(self) -> None:
        bahrain = next(circuit for circuit in self.circuits if circuit.id == 3)
        teams = {team.id: team for team in load_teams()}
        result = run_qualifying(
            circuit=bahrain,
            drivers=load_drivers(),
            teams=teams,
            player_team=teams[1],
            attempt_laps=1,
            seed=7,
        )
        self.assertEqual(result.tire_compound_nomination.soft, PhysicalTireCompound.C3)
        self.assertTrue(result.results)
        self.assertTrue(all(item.tire_role == DryTireRole.SOFT.value for item in result.results))
        self.assertTrue(
            all(item.physical_tire_compound == PhysicalTireCompound.C3.value for item in result.results)
        )


if __name__ == "__main__":
    unittest.main()
