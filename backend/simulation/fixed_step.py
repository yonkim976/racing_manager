"""Residual-time accumulators for deterministic fixed-rate simulation."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class FixedStepAccumulator:
    """Convert arbitrary elapsed-time calls into exact fixed-step counts.

    The remainder is intentionally retained between calls.  No caller may
    pass a partial step to the vehicle integrator.
    """

    step_seconds: float
    remainder_seconds: float = 0.0

    def consume(self, elapsed_seconds: float) -> int:
        elapsed = max(0.0, float(elapsed_seconds))
        self.remainder_seconds += elapsed
        steps = int((self.remainder_seconds + 1e-12) / self.step_seconds)
        if steps:
            self.remainder_seconds -= steps * self.step_seconds
            if self.remainder_seconds < 1e-12:
                self.remainder_seconds = 0.0
        return steps

    @property
    def remainder_s(self) -> float:
        """Short alias used by telemetry/debugging callers."""
        return self.remainder_seconds
