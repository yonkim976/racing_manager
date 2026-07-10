"""Tire compound specs, wear, and performance calculations."""

from __future__ import annotations

from dataclasses import dataclass

from models.schemas import TireCompound

TIRE_MANAGEMENT_BASELINE = 0.85
TIRE_MANAGEMENT_SENSITIVITY = 0.75
MIN_MANAGED_AGE_MULTIPLIER = 0.85
MAX_MANAGED_AGE_MULTIPLIER = 1.20


@dataclass(frozen=True)
class CompoundSpec:
    grip: float
    degradation_rate: float
    cliff_threshold: int


COMPOUND_SPECS: dict[TireCompound, CompoundSpec] = {
    TireCompound.SOFT: CompoundSpec(grip=1.005, degradation_rate=0.022, cliff_threshold=16),
    TireCompound.MEDIUM: CompoundSpec(grip=0.990, degradation_rate=0.014, cliff_threshold=31),
    TireCompound.HARD: CompoundSpec(grip=0.970, degradation_rate=0.010, cliff_threshold=44),
    TireCompound.INTER: CompoundSpec(grip=0.960, degradation_rate=0.025, cliff_threshold=20),
    TireCompound.WET: CompoundSpec(grip=0.950, degradation_rate=0.020, cliff_threshold=25),
}


def tire_management_age_multiplier(tire_management: float) -> float:
    """Return how quickly tire age should count after driver management."""
    multiplier = 1.0 + (TIRE_MANAGEMENT_BASELINE - tire_management) * TIRE_MANAGEMENT_SENSITIVITY
    return min(MAX_MANAGED_AGE_MULTIPLIER, max(MIN_MANAGED_AGE_MULTIPLIER, multiplier))


def compute_managed_tire_age(tire_age: float, tire_management: float) -> float:
    """Return tire age adjusted by driver tire management skill."""
    return max(0.0, tire_age) * tire_management_age_multiplier(tire_management)


def compute_wear(compound: TireCompound, tire_age: float) -> float:
    """Return curved wear value in 0.0–1.0 range for UI/strategy."""
    spec = COMPOUND_SPECS[compound]
    age_ratio = max(0.0, tire_age / spec.cliff_threshold)
    return min(1.0, age_ratio ** 1.35)


def compute_tire_performance(
    compound: TireCompound,
    tire_age: float,
    performance_variation: float = 0.0,
) -> float:
    """Return tire performance multiplier (higher = faster)."""
    spec = COMPOUND_SPECS[compound]
    age_ratio = max(0.0, tire_age / spec.cliff_threshold)
    pre_cliff_curve = min(age_ratio, 1.0) ** 1.6
    progressive_loss = (
        spec.degradation_rate
        * tire_age
        * 0.35
        * (0.55 + 0.45 * pre_cliff_curve)
    )
    cliff_overage = max(0.0, age_ratio - 1.0)
    cliff_loss = 0.045 * (cliff_overage ** 1.4)
    return spec.grip - progressive_loss - cliff_loss + performance_variation
