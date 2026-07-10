"""Pit stop timing and tire change logic."""

from __future__ import annotations

import random

from models.schemas import TireCompound


def compute_tire_change_time(
    pit_crew_skill: float,
    rng: random.Random,
) -> float:
    """Stationary tire-change time (the time the car is stopped in the box)."""
    return 2.0 + rng.uniform(0.0, 1.5) / max(pit_crew_skill, 0.5)


def compute_pit_components(
    pit_loss_time: float,
    pit_crew_skill: float,
    rng: random.Random,
) -> tuple[float, float]:
    """Split a pit stop into (pit-lane drive time, stationary tire-change time).

    ``pit_loss_time`` is the time lost driving through the pit lane (entry +
    box approach + exit), modeled separately from the stationary work so the UI
    can show both growing components instead of a single countdown.
    """
    return pit_loss_time, compute_tire_change_time(pit_crew_skill, rng)


def compute_pit_duration(
    pit_loss_time: float,
    pit_crew_skill: float,
    rng: random.Random,
) -> float:
    """Total pit stop duration in seconds (lane drive + tire change)."""
    return pit_loss_time + compute_tire_change_time(pit_crew_skill, rng)


def parse_tire_choice(choice: str) -> TireCompound:
    """Parse tire choice string to enum."""
    return TireCompound(choice.upper())
