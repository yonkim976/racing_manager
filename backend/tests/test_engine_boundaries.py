"""Architecture contracts for independently developed simulation engines."""

from __future__ import annotations

import ast
import importlib
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

from engines import (
    EngineFamily,
    SimulationEngineAdapter,
    simulation_engine_factory,
)
from engines.full.runtime.qualifying import run_qualifying as full_run_qualifying
from engines.full.runtime import race_engine as full_race_engine
from models.schemas import SimulationMode
from simulation.qualifying import run_qualifying as compatibility_run_qualifying
from simulation import race_engine as compatibility_race_engine


BACKEND_ROOT = Path(__file__).resolve().parents[1]


def imported_modules(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    modules: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.add(node.module)
    return modules


class EngineBoundaryTests(unittest.TestCase):
    def test_shared_track_compiler_does_not_initialize_full_vehicle_solver(self) -> None:
        solver_modules = (
            "engines.full.runtime.vehicle_track_solver",
            "engines.full.runtime.car_performance",
            "engines.full.runtime.tire_model",
            "engines.full.runtime.track_surface",
            "engines.full.runtime.trajectory_physics",
            "engines.full.runtime.vehicle_dynamics",
        )
        probe = (
            "import sys; import simulation.track_physics; "
            f"blocked={solver_modules!r}; "
            "loaded=[name for name in blocked if name in sys.modules]; "
            "assert not loaded, loaded"
        )
        result = subprocess.run(
            [sys.executable, "-c", probe],
            cwd=BACKEND_ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr or result.stdout)

    def test_vehicle_track_solver_legacy_exports_are_runtime_symbols(self) -> None:
        compatibility_module = importlib.import_module("simulation.track_physics")
        runtime_module = importlib.import_module(
            "engines.full.runtime.vehicle_track_solver"
        )
        exported_names = (
            "_VEHICLE_TRACK_PHYSICS_CACHE",
            "PhysicalGlobalTrajectoryCostModel",
            "optimize_vehicle_trajectory",
            "build_vehicle_track_physics_profile",
        )
        for name in exported_names:
            with self.subTest(name=name):
                self.assertIs(
                    getattr(compatibility_module, name),
                    getattr(runtime_module, name),
                )

    def test_abstract_presentation_uses_track_contract_not_solver_module(self) -> None:
        consumers = (
            BACKEND_ROOT / "engines" / "abstract" / "runtime" / "kinematics.py",
            BACKEND_ROOT / "engines" / "abstract" / "runtime" / "pose.py",
            BACKEND_ROOT / "simulation" / "start_grid_geometry.py",
        )
        for path in consumers:
            with self.subTest(path=path.name):
                modules = imported_modules(path)
                self.assertIn("simulation.track_contracts", modules)
                self.assertNotIn("simulation.track_physics", modules)

    def test_shared_geometry_does_not_import_full_runtime_aliases(self) -> None:
        shared_module_names = (
            "start_grid_geometry",
            "track_contracts",
            "track_display",
            "vehicle_dimensions",
        )
        full_aliases = {
            "simulation.ai_strategy",
            "simulation.brake_model",
            "simulation.car_performance",
            "simulation.collision",
            "simulation.events",
            "simulation.fixed_step",
            "simulation.incident_ops",
            "simulation.incidents",
            "simulation.local_trajectory_planner",
            "simulation.physics",
            "simulation.pit_ops",
            "simulation.pit_stop",
            "simulation.racecraft_ops",
            "simulation.planner_scheduler",
            "simulation.runtime_constants",
            "simulation.safety_car",
            "simulation.speed_profile",
            "simulation.start_ops",
            "simulation.state_contract",
            "simulation.strategy_ops",
            "simulation.timing_ops",
            "simulation.tire_model",
            "simulation.track_surface",
            "simulation.trajectory_physics",
            "simulation.vehicle_dynamics",
            "simulation.vehicle_physics",
            "simulation.wake_model",
        }
        for module_name in shared_module_names:
            with self.subTest(module=module_name):
                modules = imported_modules(
                    BACKEND_ROOT / "simulation" / f"{module_name}.py"
                )
                self.assertFalse(modules & full_aliases)
                self.assertFalse(
                    {module for module in modules if module.startswith("engines.full")}
                )

    def test_full_runtime_compatibility_paths_alias_runtime_modules(self) -> None:
        module_names = (
            "ai_strategy",
            "brake_model",
            "car_performance",
            "collision",
            "events",
            "fixed_step",
            "incident_ops",
            "incidents",
            "local_trajectory_planner",
            "physics",
            "pit_ops",
            "pit_stop",
            "racecraft_ops",
            "planner_scheduler",
            "runtime_constants",
            "safety_car",
            "speed_profile",
            "start_ops",
            "state_contract",
            "strategy_ops",
            "timing_ops",
            "tire_model",
            "track_surface",
            "trajectory_physics",
            "vehicle_dynamics",
            "vehicle_physics",
            "wake_model",
        )
        for module_name in module_names:
            with self.subTest(module=module_name):
                compatibility_module = importlib.import_module(
                    f"simulation.{module_name}"
                )
                runtime_module = importlib.import_module(
                    f"engines.full.runtime.{module_name}"
                )
                self.assertIs(compatibility_module, runtime_module)

    def test_full_qualifying_compatibility_import_is_the_runtime_symbol(self) -> None:
        self.assertIs(compatibility_run_qualifying, full_run_qualifying)

    def test_abstract_runtime_compatibility_paths_alias_runtime_modules(self) -> None:
        module_names = (
            "broadcast",
            "clock",
            "engine",
            "incidents",
            "kinematics",
            "performance",
            "pit",
            "pose",
            "progress_broadcast",
            "progress_race",
            "qualifying",
            "race",
            "racecraft",
            "replay",
            "rng",
            "state",
        )
        for module_name in module_names:
            with self.subTest(module=module_name):
                compatibility_module = importlib.import_module(
                    f"simulation.abstract.{module_name}"
                )
                runtime_module = importlib.import_module(
                    f"engines.abstract.runtime.{module_name}"
                )
                self.assertIs(compatibility_module, runtime_module)

    def test_abstract_state_compatibility_path_is_patch_safe(self) -> None:
        compatibility_state = importlib.import_module("simulation.abstract.state")
        runtime_state = importlib.import_module("engines.abstract.runtime.state")
        replacement = object()
        with patch.object(
            compatibility_state,
            "canonical_hash_sections",
            replacement,
        ):
            self.assertIs(runtime_state.canonical_hash_sections, replacement)

    def test_full_race_engine_compatibility_path_is_a_patch_safe_alias(self) -> None:
        self.assertIs(compatibility_race_engine, full_race_engine)
        replacement = object()
        with patch.object(compatibility_race_engine, "roll_solo_incident", replacement):
            self.assertIs(full_race_engine.roll_solo_incident, replacement)

    def test_factory_routes_every_public_mode_to_one_engine_family(self) -> None:
        expected = {
            SimulationMode.FULL: EngineFamily.FULL,
            SimulationMode.ABSTRACT: EngineFamily.ABSTRACT,
            SimulationMode.ABSTRACT_INSTANT: EngineFamily.ABSTRACT,
            SimulationMode.ABSTRACT_BROADCAST: EngineFamily.ABSTRACT,
        }
        self.assertEqual(
            {
                mode: simulation_engine_factory.for_mode(mode).capabilities.family
                for mode in SimulationMode
            },
            expected,
        )
        self.assertEqual(set(expected), set(SimulationMode))

    def test_adapters_publish_distinct_authority_and_telemetry_contracts(self) -> None:
        full = simulation_engine_factory.for_mode(SimulationMode.FULL)
        abstract = simulation_engine_factory.for_mode(SimulationMode.ABSTRACT)
        self.assertIsInstance(full, SimulationEngineAdapter)
        self.assertIsInstance(abstract, SimulationEngineAdapter)
        self.assertTrue(full.capabilities.physics_telemetry)
        self.assertFalse(abstract.capabilities.physics_telemetry)
        self.assertNotEqual(
            full.capabilities.result_authority,
            abstract.capabilities.result_authority,
        )
        self.assertNotEqual(
            full.capabilities.presentation_authority,
            abstract.capabilities.presentation_authority,
        )

    def test_engine_adapters_do_not_import_each_others_internals(self) -> None:
        full_modules = set()
        for path in (BACKEND_ROOT / "engines" / "full").rglob("*.py"):
            full_modules.update(imported_modules(path))
        abstract_modules = set()
        for path in (BACKEND_ROOT / "engines" / "abstract").rglob("*.py"):
            abstract_modules.update(imported_modules(path))

        self.assertFalse(
            {
                module for module in full_modules
                if module.startswith("engines.abstract.runtime")
                or module.startswith("engines.abstract")
            }
        )
        self.assertFalse(
            {
                module for module in abstract_modules
                if module.startswith("simulation.race_engine")
                or module.startswith("simulation.vehicle_physics")
                or module.startswith("engines.full")
            }
        )

    def test_api_selection_is_centralized_in_engine_factory(self) -> None:
        source = (BACKEND_ROOT / "main.py").read_text(encoding="utf-8")
        self.assertIn("simulation_engine_factory.for_mode", source)
        self.assertNotIn("simulation.abstract", source)
        qualifying_source = source.split(
            'def run_qualifying_session', 1
        )[1].split('@app.post("/api/race/setup"', 1)[0]
        race_source = source.split('async def setup_race', 1)[1].split(
            '@app.delete("/api/race/session"', 1
        )[0]
        self.assertNotIn("AbstractRaceEngine(", qualifying_source)
        self.assertNotIn("RaceEngine(", qualifying_source)
        self.assertNotIn("AbstractRaceEngine(", race_source)
        self.assertNotIn("RaceEngine(", race_source)


if __name__ == "__main__":
    unittest.main()
