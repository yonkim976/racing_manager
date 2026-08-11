"""FULL physics engine public adapter."""

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .adapter import FullEngineAdapter, FullRaceBuild


def __getattr__(name: str) -> Any:
    """Keep runtime leaf imports independent from adapter initialization."""
    if name in {
        "FullEngineAdapter",
        "FullRaceBuild",
        "empty_tire_temperature_diagnostic_snapshot",
    }:
        from .adapter import (
            FullEngineAdapter,
            FullRaceBuild,
            empty_tire_temperature_diagnostic_snapshot,
        )

        return {
            "FullEngineAdapter": FullEngineAdapter,
            "FullRaceBuild": FullRaceBuild,
            "empty_tire_temperature_diagnostic_snapshot": (
                empty_tire_temperature_diagnostic_snapshot
            ),
        }[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = [
    "FullEngineAdapter",
    "FullRaceBuild",
    "empty_tire_temperature_diagnostic_snapshot",
]
