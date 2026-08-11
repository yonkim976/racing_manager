"""Single selection point for FULL and ABSTRACT product paths."""

from __future__ import annotations

from models.schemas import SimulationMode

from .abstract import AbstractEngineAdapter
from .contracts import EngineFamily, SimulationEngineAdapter
from .full import FullEngineAdapter


class SimulationEngineFactory:
    def __init__(
        self,
        *,
        full: FullEngineAdapter | None = None,
        abstract: AbstractEngineAdapter | None = None,
    ) -> None:
        self._adapters: dict[EngineFamily, SimulationEngineAdapter] = {
            EngineFamily.FULL: full or FullEngineAdapter(),
            EngineFamily.ABSTRACT: abstract or AbstractEngineAdapter(),
        }

    @staticmethod
    def family_for_mode(mode: SimulationMode) -> EngineFamily:
        if mode is SimulationMode.FULL:
            return EngineFamily.FULL
        if mode in {
            SimulationMode.ABSTRACT,
            SimulationMode.ABSTRACT_INSTANT,
            SimulationMode.ABSTRACT_BROADCAST,
        }:
            return EngineFamily.ABSTRACT
        raise ValueError(f"Unsupported simulation mode: {mode}")

    def for_mode(self, mode: SimulationMode) -> SimulationEngineAdapter:
        family = self.family_for_mode(mode)
        adapter = self._adapters[family]
        if not adapter.supports(mode):
            raise ValueError(f"{family.value} engine does not support {mode.value}")
        return adapter

    def for_family(self, family: EngineFamily) -> SimulationEngineAdapter:
        return self._adapters[family]


simulation_engine_factory = SimulationEngineFactory()
