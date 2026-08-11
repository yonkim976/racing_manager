"""Shared product contract; engine internals must not cross this boundary."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Protocol, runtime_checkable

from models.schemas import SimulationMode


class EngineFamily(str, Enum):
    FULL = "full"
    ABSTRACT = "abstract"


@dataclass(frozen=True, slots=True)
class EngineCapabilities:
    family: EngineFamily
    supported_modes: frozenset[SimulationMode]
    result_authority: str
    presentation_authority: str
    physics_telemetry: bool
    interactive_strategy: bool


@runtime_checkable
class SimulationEngineAdapter(Protocol):
    """Minimal identity contract shared by API-facing engine adapters."""

    capabilities: EngineCapabilities

    def supports(self, mode: SimulationMode) -> bool: ...

    def run_qualifying(self, **kwargs: Any) -> Any: ...

    async def setup_race(self, **kwargs: Any) -> Any: ...
