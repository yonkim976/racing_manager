"""Architecture contracts for independently developed simulation engines."""

from __future__ import annotations

import ast
from pathlib import Path
import unittest

from engines import (
    EngineFamily,
    SimulationEngineAdapter,
    simulation_engine_factory,
)
from models.schemas import SimulationMode


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
        for path in (BACKEND_ROOT / "engines" / "full").glob("*.py"):
            full_modules.update(imported_modules(path))
        abstract_modules = set()
        for path in (BACKEND_ROOT / "engines" / "abstract").glob("*.py"):
            abstract_modules.update(imported_modules(path))

        self.assertFalse(
            {
                module for module in full_modules
                if module.startswith("simulation.abstract")
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
