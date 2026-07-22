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
    lateral_grip: float
    traction_grip: float
    braking_grip: float
    optimal_temperature_c: float
    operating_window_c: float


@dataclass(frozen=True)
class TirePhysicsFactors:
    """Independent tire-force factors consumed by the vehicle physics model."""

    wear: float
    lateral_grip: float
    traction_grip: float
    braking_grip: float
    thermal_grip: float


@dataclass(frozen=True)
class TireThermalState:
    surface_temperature_c: float
    core_temperature_c: float
    thermal_grip: float


COMPOUND_SPECS: dict[TireCompound, CompoundSpec] = {
    TireCompound.SOFT: CompoundSpec(
        grip=1.005,
        degradation_rate=0.022,
        cliff_threshold=16,
        lateral_grip=1.025,
        traction_grip=1.025,
        braking_grip=1.015,
        optimal_temperature_c=95.0,
        operating_window_c=12.0,
    ),
    TireCompound.MEDIUM: CompoundSpec(
        grip=0.990,
        degradation_rate=0.014,
        cliff_threshold=31,
        lateral_grip=1.000,
        traction_grip=1.000,
        braking_grip=1.000,
        optimal_temperature_c=100.0,
        operating_window_c=14.0,
    ),
    TireCompound.HARD: CompoundSpec(
        grip=0.970,
        degradation_rate=0.010,
        cliff_threshold=44,
        lateral_grip=0.985,
        traction_grip=0.985,
        braking_grip=0.990,
        optimal_temperature_c=105.0,
        operating_window_c=16.0,
    ),
    # The current game runs in dry conditions. These values intentionally make
    # wet-weather tires a poor dry-track choice until weather is implemented.
    TireCompound.INTER: CompoundSpec(
        grip=0.960,
        degradation_rate=0.025,
        cliff_threshold=20,
        lateral_grip=0.930,
        traction_grip=0.915,
        braking_grip=0.925,
        optimal_temperature_c=75.0,
        operating_window_c=15.0,
    ),
    TireCompound.WET: CompoundSpec(
        grip=0.950,
        degradation_rate=0.020,
        cliff_threshold=25,
        lateral_grip=0.900,
        traction_grip=0.885,
        braking_grip=0.900,
        optimal_temperature_c=65.0,
        operating_window_c=15.0,
    ),
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


def compute_tire_physics_factors(
    compound: TireCompound,
    tire_age: float,
    performance_variation: float = 0.0,
    temperature_c: float | None = None,
) -> TirePhysicsFactors:
    """Return compound- and wear-dependent force factors for dry-track physics."""
    spec = COMPOUND_SPECS[compound]
    wear = compute_wear(compound, tire_age)
    progressive = wear ** 1.25
    cliff_ratio = max(0.0, (wear - 0.75) / 0.25)
    cliff = cliff_ratio ** 1.6

    thermal_grip = tire_temperature_grip_factor(compound, temperature_c)

    def factor(base: float, progressive_loss: float, cliff_loss: float) -> float:
        loss = progressive_loss * progressive + cliff_loss * cliff
        return max(
            0.60,
            (base + performance_variation) * (1.0 - loss) * thermal_grip,
        )

    return TirePhysicsFactors(
        wear=wear,
        lateral_grip=factor(spec.lateral_grip, 0.09, 0.09),
        traction_grip=factor(spec.traction_grip, 0.12, 0.10),
        braking_grip=factor(spec.braking_grip, 0.07, 0.07),
        thermal_grip=thermal_grip,
    )


def tire_blanket_temperature_c(compound: TireCompound) -> float:
    if compound == TireCompound.WET:
        return 60.0
    if compound == TireCompound.INTER:
        return 70.0
    return 90.0


def tire_temperature_grip_factor(
    compound: TireCompound,
    temperature_c: float | None,
) -> float:
    """Return a continuous cold/optimal/hot grip multiplier."""
    if temperature_c is None:
        return 1.0
    spec = COMPOUND_SPECS[compound]
    deviation_c = float(temperature_c) - spec.optimal_temperature_c
    window = spec.operating_window_c
    if abs(deviation_c) <= window:
        return 1.0
    if deviation_c < -window:
        cold_excess = (-deviation_c - window) / 35.0
        return max(0.78, 1.0 - 0.20 * cold_excess)
    hot_excess = (deviation_c - window) / 30.0
    return max(0.80, 1.0 - 0.17 * hot_excess)


def advance_tire_thermal_state(
    compound: TireCompound,
    *,
    surface_temperature_c: float,
    core_temperature_c: float,
    delta_seconds: float,
    speed_mps: float,
    lateral_acceleration_mps2: float,
    throttle: float,
    brake: float,
    slide_energy_j: float,
    ambient_temperature_c: float = 30.0,
    track_temperature_c: float = 40.0,
) -> TireThermalState:
    """Advance a deterministic two-node tyre heat model for one car set."""
    step = max(0.0, float(delta_seconds))
    surface = float(surface_temperature_c)
    core = float(core_temperature_c)
    speed = max(0.0, float(speed_mps))
    mechanical_heat_w = (
        1000.0
        + abs(lateral_acceleration_mps2) * speed * 18.0
        + max(0.0, brake) * 7000.0
        + max(0.0, throttle) * 1800.0
        + max(0.0, slide_energy_j) / max(step, 1e-9) * 0.65
    )
    surface_cooling_w = max(0.0, surface - track_temperature_c) * 42.0 * (
        1.0 + speed / 55.0
    )
    surface_core_exchange_w = (surface - core) * 230.0
    surface += (
        mechanical_heat_w - surface_cooling_w - surface_core_exchange_w
    ) / 50000.0 * step
    core += (
        surface_core_exchange_w
        - max(0.0, core - ambient_temperature_c) * 14.0
    ) / 160000.0 * step
    surface = max(ambient_temperature_c, min(160.0, surface))
    core = max(ambient_temperature_c, min(140.0, core))
    return TireThermalState(
        surface_temperature_c=surface,
        core_temperature_c=core,
        thermal_grip=tire_temperature_grip_factor(compound, surface),
    )
