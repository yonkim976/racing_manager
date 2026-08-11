"""Logical fixed-step clock used by result calculations."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil


@dataclass(frozen=True, slots=True)
class LogicalClock:
    """Immutable 0.10 second logical clock.

    Advancing the clock returns a new value, which keeps a result calculation
    independent from wall-clock timing and render/update frequency.
    """

    tick_seconds: float = 0.10
    tick_index: int = 0

    def __post_init__(self) -> None:
        if self.tick_seconds <= 0.0:
            raise ValueError("tick_seconds must be positive")
        if self.tick_index < 0:
            raise ValueError("tick_index must not be negative")

    @property
    def time_s(self) -> float:
        return round(self.tick_index * self.tick_seconds, 9)

    def advance(self, ticks: int = 1) -> "LogicalClock":
        if ticks < 0:
            raise ValueError("ticks must not be negative")
        return LogicalClock(self.tick_seconds, self.tick_index + ticks)

    step = advance

    def advance_to(self, time_s: float) -> "LogicalClock":
        if time_s < self.time_s:
            raise ValueError("logical clock cannot move backwards")
        ticks = ceil((time_s - self.time_s) / self.tick_seconds - 1e-12)
        return self.advance(ticks)
