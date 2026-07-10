"""AI pit strategy decisions."""

from __future__ import annotations

from models.schemas import DriverRaceState, TireCompound
from simulation.tire_model import COMPOUND_SPECS, compute_managed_tire_age, compute_wear


def choose_tire_for_remaining_laps(remaining_laps: int) -> TireCompound:
    """Pick the best compound to finish remaining laps."""
    if remaining_laps <= 12:
        return TireCompound.SOFT
    if remaining_laps <= 28:
        return TireCompound.MEDIUM
    return TireCompound.HARD


def should_pit(
    state: DriverRaceState,
    remaining_laps: int,
    is_player: bool,
    tire_management: float = 0.85,
) -> bool:
    """Decide if AI should request a pit stop."""
    if is_player or state.retired or state.in_pit or state.pit_request is not None:
        return False

    raw_tire_age = state.tire_usage if state.tire_usage > 0 else float(state.tire_age)
    managed_tire_age = compute_managed_tire_age(raw_tire_age, tire_management)
    managed_next_lap_age = compute_managed_tire_age(raw_tire_age + 1, tire_management)
    wear = compute_wear(state.tire_compound, managed_tire_age)
    spec = COMPOUND_SPECS[state.tire_compound]
    approaching_cliff = managed_next_lap_age >= spec.cliff_threshold

    if wear >= 0.7 or approaching_cliff:
        return True

    return False


def choose_pit_tire(state: DriverRaceState, remaining_laps: int) -> TireCompound:
    """Choose tire compound for an AI pit stop."""
    return choose_tire_for_remaining_laps(remaining_laps)
