"""Public selection boundary for independently developed race engines."""

from .contracts import EngineCapabilities, EngineFamily, SimulationEngineAdapter
from .factory import SimulationEngineFactory, simulation_engine_factory

__all__ = [
    "EngineCapabilities",
    "EngineFamily",
    "SimulationEngineAdapter",
    "SimulationEngineFactory",
    "simulation_engine_factory",
]
