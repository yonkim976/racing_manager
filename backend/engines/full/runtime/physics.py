"""FULL lap-time and progress calculations."""

from __future__ import annotations

import random

GAME_TICK_SECONDS = 0.1


def driver_pace_multiplier(pace_stat: float) -> float:
    """Map driver pace stat to a compressed lap-time multiplier."""
    return 1.0 + (pace_stat - 0.88) * 0.28


def compute_effective_lap_time(
    base_lap_time: float,
    vehicle_performance: float,
    driver_pace: float,
    tire_performance: float,
    rng: random.Random,
    lap_random_variation: float = 0.0,
) -> float:
    """Compute effective lap time in seconds for one tick slice."""
    if lap_random_variation == 0.0:
        lap_random_variation = rng.uniform(-0.3, 0.3)

    denominator = vehicle_performance * driver_pace * tire_performance
    if denominator <= 0:
        return base_lap_time * 10 + abs(lap_random_variation)
    return base_lap_time / denominator + lap_random_variation


def compute_progress_delta(delta_game_seconds: float, effective_lap_time: float) -> float:
    """Return progress increment (0.0–1.0 fraction of a lap)."""
    if effective_lap_time <= 0:
        return 0.0
    return delta_game_seconds / effective_lap_time
