"""Public selection boundary for independently developed race engines."""

from typing import TYPE_CHECKING, Any

from .contracts import EngineCapabilities, EngineFamily, SimulationEngineAdapter

if TYPE_CHECKING:
    from .factory import SimulationEngineFactory


def __getattr__(name: str) -> Any:
    """Load the factory only when a public selector is requested.

    Runtime compatibility aliases import ``engines.full.runtime`` directly.
    Eagerly importing the factory here would initialize both engine families and
    can create a cycle while shared geometry modules are still loading.
    """
    if name in {"SimulationEngineFactory", "simulation_engine_factory"}:
        from .factory import SimulationEngineFactory, simulation_engine_factory

        return {
            "SimulationEngineFactory": SimulationEngineFactory,
            "simulation_engine_factory": simulation_engine_factory,
        }[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = [
    "EngineCapabilities",
    "EngineFamily",
    "SimulationEngineAdapter",
    "SimulationEngineFactory",
    "simulation_engine_factory",
]
