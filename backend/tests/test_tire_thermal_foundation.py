"""Regression tests for the axle-aware, session-owned tire thermal model."""

from __future__ import annotations

import inspect
import math
import unittest

from data_loader import load_circuits, load_drivers, load_teams
from models.schemas import (
    PhysicalTireCompound,
    RaceSetupRequest,
    TrackConditions,
    TireCompound,
)
from simulation.race_engine import RaceEngine
from simulation.tire_model import (
    TIRE_CORE_NUMERIC_GUARD_C,
    TIRE_SURFACE_NUMERIC_GUARD_C,
    TireThermalBudget,
    advance_tire_thermal_state,
)


class TireThermalFoundationTests(unittest.TestCase):
    @staticmethod
    def _advance(
        *,
        surface: float = 90.0,
        core: float = 90.0,
        delta: float = 0.02,
        speed: float = 50.0,
        lateral: float = 0.0,
        throttle: float = 0.0,
        brake: float = 0.0,
        slide: float = 0.0,
        ambient: float = 30.0,
        track: float = 40.0,
        **kwargs: float,
    ):
        return advance_tire_thermal_state(
            TireCompound.MEDIUM,
            surface_temperature_c=surface,
            core_temperature_c=core,
            delta_seconds=delta,
            speed_mps=speed,
            lateral_acceleration_mps2=lateral,
            throttle=throttle,
            brake=brake,
            slide_energy_j=slide,
            ambient_temperature_c=ambient,
            track_temperature_c=track,
            **kwargs,
        )

    def test_environment_is_required_at_the_tire_model_boundary(self) -> None:
        parameters = inspect.signature(advance_tire_thermal_state).parameters
        self.assertIs(parameters["ambient_temperature_c"].default, inspect.Parameter.empty)
        self.assertIs(parameters["track_temperature_c"].default, inspect.Parameter.empty)

    def test_session_boundary_owns_immutable_environment_defaults(self) -> None:
        request = RaceSetupRequest(circuit_id=3, player_team_id=1)
        self.assertIsNone(request.track_conditions)

    def test_engine_passes_conditions_to_dashboard_and_bounded_diagnostics(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 3)
        drivers = load_drivers()
        teams = {team.id: team for team in load_teams()}
        conditions = TrackConditions(
            ambient_temperature_c=24.0,
            track_temperature_c=32.0,
        )
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=drivers[0].team_id,
            player_driver_ids=[drivers[0].id],
            seed=11,
            start_sequence_enabled=False,
            track_conditions=conditions,
        )
        engine.tick(0.02)
        self.assertEqual(engine.track_conditions, conditions)
        self.assertEqual(engine.build_dashboard_payload()["track_conditions"], {
            "ambient_temperature_c": 24.0,
            "track_temperature_c": 32.0,
        })
        snapshot = engine.diagnostic_counts()["tire_temperature"]
        self.assertEqual(snapshot["track_conditions"], {
            "ambient_temperature_c": 24.0,
            "track_temperature_c": 32.0,
        })
        self.assertLessEqual(len(snapshot["clamp"]["first"]), 2)

    def test_bahrain_twenty_car_thermal_integration_does_not_hit_guards(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 3)
        drivers = load_drivers()
        teams = {team.id: team for team in load_teams()}
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=drivers[0].team_id,
            player_driver_ids=[drivers[0].id],
            seed=2026,
            start_sequence_enabled=False,
            track_conditions=TrackConditions(
                ambient_temperature_c=30.0,
                track_temperature_c=40.0,
            ),
        )
        for _ in range(20):
            engine.tick(0.1)
        snapshot = engine.diagnostic_counts()["tire_temperature"]
        self.assertEqual(snapshot["current"]["active_driver_count"], 20)
        self.assertEqual(snapshot["clamp"]["cumulative"]["surface_hit_count"], 0)
        self.assertEqual(snapshot["clamp"]["cumulative"]["core_hit_count"], 0)

    def test_bahrain_twenty_car_thermal_integration_remains_bounded_for_17_lap_equivalent(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 3)
        drivers = load_drivers()
        teams = {team.id: team for team in load_teams()}
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=1,
            player_driver_ids=[1, 2],
            seed=42,
            start_sequence_enabled=False,
            track_conditions=TrackConditions(
                ambient_temperature_c=30.0,
                track_temperature_c=40.0,
            ),
        )
        for _ in range(int(1_700.0 / 0.1)):
            engine.tick(0.1)

        snapshot = engine.diagnostic_counts()["tire_temperature"]
        # This is the deterministic direct-engine baseline for the current
        # physics revision. Load-sensitive bicycle stiffness and the approved
        # Bahrain lateral-slope continuity guard reduce excess slip heat
        # without changing the thermal coefficients. Keep this gate tied to
        # the exact setup above so it is reproducible.
        self.assertAlmostEqual(
            snapshot["peak"]["rear_surface_max_c"],
            110.834,
            delta=1.0,
        )
        self.assertAlmostEqual(
            snapshot["peak"]["rear_core_max_c"],
            97.999,
            delta=1.0,
        )
        self.assertEqual(snapshot["current"]["rear_overheat_driver_count"], 0)
        self.assertEqual(snapshot["peak"]["max_rear_overheat_driver_count"], 0)
        self.assertEqual(
            snapshot["peak"]["max_continuous_overheat_seconds"],
            0.0,
        )
        self.assertEqual(snapshot["clamp"]["cumulative"]["surface_hit_count"], 0)
        self.assertEqual(snapshot["clamp"]["cumulative"]["core_hit_count"], 0)

    def test_peak_rear_overheat_count_survives_recovery_without_double_counting(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 3)
        drivers = load_drivers()
        teams = {team.id: team for team in load_teams()}
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=1,
            player_driver_ids=[1, 2],
            seed=42,
            start_sequence_enabled=False,
            track_conditions=TrackConditions(
                ambient_temperature_c=30.0,
                track_temperature_c=40.0,
            ),
        )
        overheat_ids = sorted(engine.driver_states)[:3]
        for driver_id in overheat_ids:
            engine.driver_states[driver_id].rear_tire_surface_temperature_c = 131.0
        engine._record_tire_temperature_diagnostics(0.02)

        snapshot = engine.diagnostic_counts()["tire_temperature"]
        self.assertEqual(snapshot["current"]["rear_overheat_driver_count"], 3)
        self.assertEqual(snapshot["peak"]["max_rear_overheat_driver_count"], 3)
        self.assertEqual(snapshot["peak"]["max_rear_overheat_driver_ids"], overheat_ids)
        for driver_id in overheat_ids:
            engine.driver_states[driver_id].rear_tire_surface_temperature_c = 90.0
        engine._record_tire_temperature_diagnostics(0.02)

        snapshot = engine.diagnostic_counts()["tire_temperature"]
        self.assertEqual(snapshot["current"]["rear_overheat_driver_count"], 0)
        self.assertEqual(snapshot["peak"]["max_rear_overheat_driver_count"], 3)
        self.assertEqual(snapshot["peak"]["max_rear_overheat_driver_ids"], overheat_ids)

    def test_compound_diagnostics_preserve_peak_and_continuous_overheat_evidence(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 8)
        drivers = load_drivers()
        teams = {team.id: team for team in load_teams()}
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=1,
            player_driver_ids=[1, 2],
            seed=42,
            start_sequence_enabled=False,
            track_conditions=TrackConditions(
                ambient_temperature_c=30.0,
                track_temperature_c=40.0,
            ),
        )
        state = engine.driver_states[1]
        engine.set_driver_tire_compound(state, TireCompound.SOFT)
        self.assertEqual(state.physical_tire_compound, PhysicalTireCompound.C5)
        state.rear_tire_surface_temperature_c = 116.0
        state.rear_tire_core_temperature_c = 108.0
        engine._record_tire_temperature_diagnostics(0.1)

        snapshot = engine.diagnostic_counts()["tire_temperature"]
        c5 = snapshot["per_compound"]["C5"]
        self.assertEqual(c5["current_overheat_count"], 1)
        self.assertEqual(c5["current_max_continuous_overheat_driver_id"], 1)
        self.assertAlmostEqual(c5["current_max_continuous_overheat_seconds"], 0.1)
        self.assertAlmostEqual(c5["max_continuous_overheat_seconds"], 0.1)
        self.assertEqual(c5["max_continuous_overheat_driver_id"], 1)
        self.assertEqual(c5["peak_surface_max_c"], 116.0)
        self.assertEqual(c5["peak_core_max_c"], 108.0)

        engine.set_driver_tire_compound(state, TireCompound.HARD)
        state.rear_tire_surface_temperature_c = 100.0
        engine._record_tire_temperature_diagnostics(0.1)
        snapshot = engine.diagnostic_counts()["tire_temperature"]
        c5 = snapshot["per_compound"]["C5"]
        self.assertEqual(c5["current_overheat_count"], 0)
        self.assertEqual(c5["current_max_continuous_overheat_seconds"], 0.0)
        self.assertAlmostEqual(c5["max_continuous_overheat_seconds"], 0.1)
        self.assertEqual(snapshot["peak"]["max_compound_overheat_driver_id"], 1)

    def test_clamp_diagnostic_counts_steps_once_and_keeps_first_fact(self) -> None:
        circuit = next(item for item in load_circuits() if item.id == 3)
        drivers = load_drivers()
        teams = {team.id: team for team in load_teams()}
        engine = RaceEngine(
            circuit=circuit,
            drivers=drivers,
            teams=teams,
            player_team_id=drivers[0].team_id,
            player_driver_ids=[drivers[0].id],
            seed=5,
            start_sequence_enabled=False,
        )
        state = engine.driver_states[drivers[0].id]
        for _ in range(2):
            thermal = self._advance(
                surface=150.0,
                core=90.0,
                delta=0.02,
                throttle=1.0,
                slide=1_000_000.0,
            )
            engine._accumulate_tire_thermal_diagnostics(
                state,
                thermal,
                self._advance(delta=0.02),
                0.02,
            )
        engine._record_tire_temperature_diagnostics(0.04)
        snapshot = engine.diagnostic_counts()["tire_temperature"]
        self.assertEqual(snapshot["clamp"]["cumulative"]["surface_hit_count"], 2)
        self.assertIsNotNone(snapshot["clamp"]["first"]["surface"])
        self.assertAlmostEqual(
            snapshot["clamp"]["cumulative"]["surface_active_seconds"],
            0.04,
        )
        engine._record_tire_temperature_diagnostics(0.04)
        self.assertEqual(
            engine.diagnostic_counts()["tire_temperature"]["clamp"]["cumulative"]["surface_hit_count"],
            2,
        )

    def test_thermal_budget_separates_sources_and_balances_nodes(self) -> None:
        result = self._advance(
            surface=105.0,
            core=92.0,
            delta=0.2,
            speed=60.0,
            lateral=4.0,
            throttle=0.0,
            brake=0.0,
            slide=0.0,
            baseline_heat_share=0.0,
            lateral_heat_share=1.0,
            brake_heat_share=0.0,
            traction_heat_share=0.0,
            slide_heat_share=0.0,
        )
        budget = result.budget
        self.assertGreater(budget.lateral_heat_j, 0.0)
        self.assertEqual(budget.baseline_heat_j, 0.0)
        self.assertEqual(budget.braking_heat_j, 0.0)
        self.assertEqual(budget.traction_heat_j, 0.0)
        self.assertEqual(budget.slide_heat_j, 0.0)
        self.assertAlmostEqual(
            (result.surface_temperature_c - 105.0) * 50_000.0,
            budget.surface_net_energy_j,
            places=8,
        )
        self.assertAlmostEqual(
            (result.core_temperature_c - 92.0) * 160_000.0,
            budget.core_net_energy_j,
            places=8,
        )

    def test_applied_axle_brake_work_is_converted_once(self) -> None:
        front = self._advance(
            delta=0.1,
            brake=1.0,
            brake_heat_share=0.58,
            applied_brake_work_energy_j=10_000.0,
        )
        rear = self._advance(
            delta=0.1,
            brake=1.0,
            brake_heat_share=0.42,
            applied_brake_work_energy_j=5_000.0,
        )

        self.assertAlmostEqual(front.budget.braking_heat_j, 45.0)
        self.assertAlmostEqual(rear.budget.braking_heat_j, 22.5)
        self.assertAlmostEqual(
            front.budget.braking_heat_j + rear.budget.braking_heat_j,
            15_000.0 * 0.0045,
        )

    def test_surface_to_core_transfer_is_one_signed_shared_energy(self) -> None:
        result = self._advance(surface=110.0, core=90.0, delta=0.1)
        self.assertGreater(result.budget.surface_to_core_transfer_j, 0.0)
        self.assertGreater(result.budget.core_net_energy_j, 0.0)
        self.assertAlmostEqual(
            result.budget.surface_net_energy_j
            + result.budget.core_net_energy_j,
            result.budget.heat_input_j - result.budget.cooling_j,
            places=8,
        )

    def test_zero_delta_does_not_change_temperature_or_energy(self) -> None:
        result = self._advance(
            surface=120.0,
            core=115.0,
            delta=0.0,
            slide=100_000.0,
            throttle=1.0,
        )
        self.assertEqual(result.surface_temperature_c, 120.0)
        self.assertEqual(result.core_temperature_c, 115.0)
        self.assertEqual(result.budget, TireThermalBudget.zero())

    def test_numeric_guard_preserves_unclamped_overshoot(self) -> None:
        result = self._advance(
            surface=150.0,
            core=135.0,
            delta=1.0,
            speed=30.0,
            throttle=1.0,
            slide=1_000_000.0,
        )
        self.assertTrue(result.surface_clamp_hit)
        self.assertGreater(result.surface_unclamped_temperature_c, TIRE_SURFACE_NUMERIC_GUARD_C)
        self.assertGreater(result.surface_clamp_overshoot_c, 0.0)
        self.assertEqual(result.surface_temperature_c, TIRE_SURFACE_NUMERIC_GUARD_C)

        core_result = None
        surface, core = 160.0, 139.0
        for _ in range(3_000):
            core_result = self._advance(
                surface=surface,
                core=core,
                delta=0.02,
                speed=30.0,
                throttle=1.0,
                slide=1_000_000.0,
            )
            surface, core = core_result.surface_temperature_c, core_result.core_temperature_c
            if core_result.core_clamp_hit:
                break
        self.assertIsNotNone(core_result)
        self.assertTrue(core_result.core_clamp_hit)
        self.assertGreater(core_result.core_unclamped_temperature_c, TIRE_CORE_NUMERIC_GUARD_C)
        self.assertGreater(core_result.core_clamp_overshoot_c, 0.0)
        self.assertEqual(core_result.core_temperature_c, TIRE_CORE_NUMERIC_GUARD_C)

    def test_repeated_clean_laps_reach_bounded_equilibrium_for_current_compounds(self) -> None:
        for compound in (TireCompound.SOFT, TireCompound.MEDIUM, TireCompound.HARD):
            surface = core = 90.0
            maximum_surface = maximum_core = -math.inf
            for step_index in range(int(57 * 90 / 0.1)):
                phase = (step_index * 0.1) % 90.0
                if phase < 15.0:
                    speed, lateral, throttle, brake, slide = 70.0, 2.0, 0.8, 0.0, 0.0
                elif phase < 25.0:
                    speed, lateral, throttle, brake, slide = 55.0, 3.0, 0.0, 0.7, 100.0
                elif phase < 60.0:
                    speed, lateral, throttle, brake, slide = 65.0, 6.0, 0.7, 0.0, 0.0
                elif phase < 75.0:
                    speed, lateral, throttle, brake, slide = 35.0, 5.0, 0.7, 0.0, 200.0
                else:
                    speed, lateral, throttle, brake, slide = 80.0, 0.0, 0.5, 0.0, 0.0
                result = advance_tire_thermal_state(
                    compound,
                    surface_temperature_c=surface,
                    core_temperature_c=core,
                    delta_seconds=0.1,
                    speed_mps=speed,
                    lateral_acceleration_mps2=lateral,
                    throttle=throttle,
                    brake=brake,
                    slide_energy_j=slide,
                    ambient_temperature_c=30.0,
                    track_temperature_c=40.0,
                )
                surface, core = result.surface_temperature_c, result.core_temperature_c
                maximum_surface = max(maximum_surface, result.surface_unclamped_temperature_c)
                maximum_core = max(maximum_core, result.core_unclamped_temperature_c)
                self.assertTrue(math.isfinite(surface))
                self.assertTrue(math.isfinite(core))
            self.assertLess(maximum_surface, TIRE_SURFACE_NUMERIC_GUARD_C)
            self.assertLess(maximum_core, TIRE_CORE_NUMERIC_GUARD_C)
            self.assertLess(surface, 140.0)
            self.assertLess(core, 130.0)

    def test_environment_ordering_changes_equilibrium_without_guard_hits(self) -> None:
        def run(conditions: TrackConditions) -> tuple[float, float, bool, bool]:
            surface = core = 90.0
            hit_surface = hit_core = False
            for _ in range(int(180.0 / 0.1)):
                result = advance_tire_thermal_state(
                    TireCompound.MEDIUM,
                    surface_temperature_c=surface,
                    core_temperature_c=core,
                    delta_seconds=0.1,
                    speed_mps=60.0,
                    lateral_acceleration_mps2=4.0,
                    throttle=0.6,
                    brake=0.0,
                    slide_energy_j=0.0,
                    ambient_temperature_c=conditions.ambient_temperature_c,
                    track_temperature_c=conditions.track_temperature_c,
                )
                surface, core = result.surface_temperature_c, result.core_temperature_c
                hit_surface |= result.surface_clamp_hit
                hit_core |= result.core_clamp_hit
            return surface, core, hit_surface, hit_core

        cool = run(TrackConditions(ambient_temperature_c=24.0, track_temperature_c=32.0))
        normal = run(TrackConditions(ambient_temperature_c=30.0, track_temperature_c=40.0))
        hot = run(TrackConditions(ambient_temperature_c=36.0, track_temperature_c=52.0))
        self.assertLessEqual(cool[0], normal[0])
        self.assertLessEqual(normal[0], hot[0])
        self.assertLessEqual(cool[1], normal[1])
        self.assertLessEqual(normal[1], hot[1])
        self.assertFalse(cool[2] or cool[3] or normal[2] or normal[3] or hot[2] or hot[3])

    def test_overheated_rear_surrogate_recovers_with_surface_leading_core(self) -> None:
        surface = core = 90.0
        for _ in range(int(15.0 / 0.02)):
            result = self._advance(
                surface=surface,
                core=core,
                delta=0.02,
                speed=60.0,
                lateral=8.0,
                throttle=1.0,
                slide=5_000.0,
            )
            surface, core = result.surface_temperature_c, result.core_temperature_c
        overheated_surface, overheated_core = surface, core
        self.assertGreater(overheated_surface, 130.0)
        self.assertFalse(result.surface_clamp_hit or result.core_clamp_hit)

        first_recovery = self._advance(
            surface=surface,
            core=core,
            delta=0.02,
            speed=25.0,
            throttle=0.05,
        )
        self.assertLess(first_recovery.surface_temperature_c, overheated_surface)
        self.assertGreater(first_recovery.core_temperature_c, overheated_core)

        surface, core = first_recovery.surface_temperature_c, first_recovery.core_temperature_c
        peak_core = core
        for _ in range(int(480.0 / 0.02) - 1):
            result = self._advance(
                surface=surface,
                core=core,
                delta=0.02,
                speed=25.0,
                throttle=0.05,
            )
            surface, core = result.surface_temperature_c, result.core_temperature_c
            peak_core = max(peak_core, core)
        self.assertLess(surface, overheated_surface - 5.0)
        self.assertLess(core, peak_core)
        self.assertFalse(result.surface_clamp_hit or result.core_clamp_hit)

    def test_thermal_integration_is_stable_across_step_sizes(self) -> None:
        budget_fields = (
            "baseline_heat_j",
            "lateral_heat_j",
            "braking_heat_j",
            "traction_heat_j",
            "slide_heat_j",
            "surface_air_track_cooling_j",
            "surface_to_core_transfer_j",
            "core_ambient_cooling_j",
            "surface_net_energy_j",
            "core_net_energy_j",
        )

        def run(delta: float) -> tuple[float, float, dict[str, float]]:
            surface = core = 90.0
            elapsed = 0.0
            accumulated = {field: 0.0 for field in budget_fields}
            while elapsed < 180.0 - 1e-12:
                step = min(delta, 180.0 - elapsed)
                result = self._advance(
                    surface=surface,
                    core=core,
                    delta=step,
                    speed=60.0,
                    lateral=4.0,
                    throttle=0.6,
                )
                surface, core = result.surface_temperature_c, result.core_temperature_c
                for field in budget_fields:
                    accumulated[field] += getattr(result.budget, field)
                elapsed += step
            return surface, core, accumulated

        baseline = run(0.02)
        for delta in (0.05, 0.1):
            candidate = run(delta)
            self.assertAlmostEqual(candidate[0], baseline[0], delta=0.02)
            self.assertAlmostEqual(candidate[1], baseline[1], delta=0.02)
            for field in budget_fields:
                tolerance = max(0.05, abs(baseline[2][field]) * 0.005)
                self.assertAlmostEqual(
                    candidate[2][field],
                    baseline[2][field],
                    delta=tolerance,
                    msg=f"step-size energy drift in {field}",
                )


if __name__ == "__main__":
    unittest.main()
