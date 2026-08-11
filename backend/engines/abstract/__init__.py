"""ABSTRACT management engine public adapter."""

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .adapter import AbstractEngineAdapter


def __getattr__(name: str) -> Any:
    """Load the ABSTRACT adapter only through the public selection boundary."""
    if name == "AbstractEngineAdapter":
        from .adapter import AbstractEngineAdapter

        return AbstractEngineAdapter
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = ["AbstractEngineAdapter"]
