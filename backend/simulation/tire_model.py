"""Tire compound specs, wear, and performance calculations."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

from models.schemas import PhysicalTireCompound, TireCompound

TIRE_MANAGEMENT_BASELINE = 0.85
TIRE_MANAGEMENT_SENSITIVITY = 0.75
MIN_MANAGED_AGE_MULTIPLIER = 0.85
MAX_MANAGED_AGE_MULTIPLIER = 1.20

# These are numerical safety guards, not validated tire failure limits.  Keep
# them stable so diagnostics can expose when the physical model exceeds the
# safe numerical range instead of hiding the excursion by changing the cap.
TIRE_SURFACE_NUMERIC_GUARD_C = 160.0
TIRE_CORE_NUMERIC_GUARD_C = 140.0

# Game-calibration conversions for mechanical work to tire heat.  The
# authoritative inputs are force-work energies; these factors are not claimed
# to be measured tire material properties.
TIRE_TRACTION_WORK_TO_HEAT_FACTOR = 0.0036
TIRE_BRAKING_WORK_TO_HEAT_FACTOR = 0.0045
TIRE_SLIDE_ENERGY_TO_HEAT_FACTOR = 0.65
# The lateral input is an acceleration-speed proxy rather than a measured
# contact-patch work value. Calibrated down from 18.0 after the Bahrain
# 17-lap run showed a sustained rear lateral heat surplus; a second pass
# reduced the remaining positive surface heat balance.
TIRE_LATERAL_MOTION_TO_HEAT_FACTOR = 6.0

# The original compound durability calibration was approved at Bahrain. Tire
# usage is therefore expressed in Bahrain-equivalent laps so a 4.3 km lap does
# not consume the same rubber as a 7.0 km lap before circuit abrasion is
# considered.
TIRE_WEAR_REFERENCE_LAP_DISTANCE_M = 5412.0
MAX_THERMAL_WEAR_MULTIPLIER = 1.30


@dataclass(frozen=True)
class CompoundSpec:
    code: PhysicalTireCompound
    grip: float
    degradation_rate: float
    cliff_threshold_laps: float
    lateral_grip: float
    traction_grip: float
    braking_grip: float
    optimal_surface_temperature_c: float
    surface_operating_window_c: float
    hot_diagnostic_threshold_c: float
    blanket_temperature_c: float
    source_class: str

    def __post_init__(self) -> None:
        """Fail fast on malformed provisional or future compound data."""
        if not isinstance(self.code, PhysicalTireCompound):
            raise ValueError("compound code must be a PhysicalTireCompound")
        bounded_values = {
            "grip": (self.grip, 0.5, 1.5),
            "degradation_rate": (self.degradation_rate, 0.000001, 1.0),
            "cliff_threshold_laps": (self.cliff_threshold_laps, 0.001, 200.0),
            "lateral_grip": (self.lateral_grip, 0.5, 1.5),
            "traction_grip": (self.traction_grip, 0.5, 1.5),
            "braking_grip": (self.braking_grip, 0.5, 1.5),
            "optimal_surface_temperature_c": (
                self.optimal_surface_temperature_c,
                0.001,
                160.0,
            ),
            "surface_operating_window_c": (
                self.surface_operating_window_c,
                0.001,
                80.0,
            ),
            "hot_diagnostic_threshold_c": (
                self.hot_diagnostic_threshold_c,
                0.001,
                160.0,
            ),
            "blanket_temperature_c": (
                self.blanket_temperature_c,
                0.001,
                160.0,
            ),
        }
        for name, (value, lower, upper) in bounded_values.items():
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise ValueError(f"{name} must be numeric")
            if not isfinite(value) or not lower <= value <= upper:
                raise ValueError(f"{name} must be finite and within [{lower}, {upper}]")
        if self.hot_diagnostic_threshold_c < (
            self.optimal_surface_temperature_c + self.surface_operating_window_c
        ):
            raise ValueError("hot diagnostic threshold must be above the operating window")
        if not isinstance(self.source_class, str) or not self.source_class.strip():
            raise ValueError("compound source_class is required")

    @property
    def cliff_threshold(self) -> float:
        """Legacy field name retained for existing strategy code."""
        return self.cliff_threshold_laps

    @property
    def optimal_temperature_c(self) -> float:
        """Legacy field name retained for existing strategy code."""
        return self.optimal_surface_temperature_c

    @property
    def operating_window_c(self) -> float:
        """Legacy field name retained for existing strategy code."""
        return self.surface_operating_window_c


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
    budget: "TireThermalBudget"
    surface_unclamped_temperature_c: float
    core_unclamped_temperature_c: float
    surface_clamp_hit: bool
    core_clamp_hit: bool
    surface_clamp_overshoot_c: float
    core_clamp_overshoot_c: float


@dataclass(frozen=True)
class TireThermalBudget:
    """Energy ledger for one axle's two-node thermal update."""

    baseline_heat_j: float = 0.0
    lateral_heat_j: float = 0.0
    braking_heat_j: float = 0.0
    traction_heat_j: float = 0.0
    slide_heat_j: float = 0.0
    surface_air_track_cooling_j: float = 0.0
    surface_to_core_transfer_j: float = 0.0
    core_ambient_cooling_j: float = 0.0
    surface_net_energy_j: float = 0.0
    core_net_energy_j: float = 0.0

    @classmethod
    def zero(cls) -> "TireThermalBudget":
        return cls()

    @property
    def heat_input_j(self) -> float:
        return (
            self.baseline_heat_j
            + self.lateral_heat_j
            + self.braking_heat_j
            + self.traction_heat_j
            + self.slide_heat_j
        )

    @property
    def cooling_j(self) -> float:
        return self.surface_air_track_cooling_j + self.core_ambient_cooling_j


COMPOUND_SPECS: dict[PhysicalTireCompound, CompoundSpec] = {
    PhysicalTireCompound.C1: CompoundSpec(
        code=PhysicalTireCompound.C1,
        grip=0.970,
        degradation_rate=0.010,
        cliff_threshold_laps=44,
        lateral_grip=0.985,
        traction_grip=0.985,
        braking_grip=0.990,
        optimal_surface_temperature_c=105.0,
        surface_operating_window_c=16.0,
        hot_diagnostic_threshold_c=136.0,
        blanket_temperature_c=90.0,
        source_class="game_calibration_provisional",
    ),
    PhysicalTireCompound.C2: CompoundSpec(
        code=PhysicalTireCompound.C2,
        grip=0.990,
        degradation_rate=0.014,
        cliff_threshold_laps=31,
        lateral_grip=1.000,
        traction_grip=1.000,
        braking_grip=1.000,
        optimal_surface_temperature_c=100.0,
        surface_operating_window_c=14.0,
        hot_diagnostic_threshold_c=129.0,
        blanket_temperature_c=90.0,
        source_class="game_calibration_provisional",
    ),
    PhysicalTireCompound.C3: CompoundSpec(
        code=PhysicalTireCompound.C3,
        grip=1.005,
        degradation_rate=0.022,
        cliff_threshold_laps=16,
        lateral_grip=1.025,
        traction_grip=1.025,
        braking_grip=1.015,
        optimal_surface_temperature_c=95.0,
        surface_operating_window_c=12.0,
        hot_diagnostic_threshold_c=122.0,
        blanket_temperature_c=90.0,
        source_class="game_calibration_provisional",
    ),
    PhysicalTireCompound.C4: CompoundSpec(
        code=PhysicalTireCompound.C4,
        grip=1.012,
        degradation_rate=0.028,
        cliff_threshold_laps=13,
        lateral_grip=1.035,
        traction_grip=1.035,
        braking_grip=1.022,
        optimal_surface_temperature_c=92.0,
        surface_operating_window_c=11.0,
        hot_diagnostic_threshold_c=118.0,
        blanket_temperature_c=90.0,
        source_class="game_calibration_provisional",
    ),
    PhysicalTireCompound.C5: CompoundSpec(
        code=PhysicalTireCompound.C5,
        grip=1.020,
        degradation_rate=0.035,
        cliff_threshold_laps=10,
        lateral_grip=1.045,
        traction_grip=1.045,
        braking_grip=1.030,
        optimal_surface_temperature_c=90.0,
        surface_operating_window_c=10.0,
        hot_diagnostic_threshold_c=115.0,
        blanket_temperature_c=90.0,
        source_class="game_calibration_provisional",
    ),
    # The current game runs in dry conditions. These values intentionally make
    # wet-weather tires a poor dry-track choice until weather is implemented.
    PhysicalTireCompound.INTER: CompoundSpec(
        code=PhysicalTireCompound.INTER,
        grip=0.960,
        degradation_rate=0.025,
        cliff_threshold_laps=20,
        lateral_grip=0.930,
        traction_grip=0.915,
        braking_grip=0.925,
        optimal_surface_temperature_c=75.0,
        surface_operating_window_c=15.0,
        hot_diagnostic_threshold_c=105.0,
        blanket_temperature_c=70.0,
        source_class="game_calibration_provisional",
    ),
    PhysicalTireCompound.WET: CompoundSpec(
        code=PhysicalTireCompound.WET,
        grip=0.950,
        degradation_rate=0.020,
        cliff_threshold_laps=25,
        lateral_grip=0.900,
        traction_grip=0.885,
        braking_grip=0.900,
        optimal_surface_temperature_c=65.0,
        surface_operating_window_c=15.0,
        hot_diagnostic_threshold_c=95.0,
        blanket_temperature_c=60.0,
        source_class="game_calibration_provisional",
    ),
}


LEGACY_TIRE_COMPOUND_TO_PHYSICAL: dict[TireCompound, PhysicalTireCompound] = {
    TireCompound.HARD: PhysicalTireCompound.C1,
    TireCompound.MEDIUM: PhysicalTireCompound.C2,
    TireCompound.SOFT: PhysicalTireCompound.C3,
    TireCompound.INTER: PhysicalTireCompound.INTER,
    TireCompound.WET: PhysicalTireCompound.WET,
}


def physical_compound_for(compound: PhysicalTireCompound | TireCompound | str) -> PhysicalTireCompound:
    """Convert a physical code or legacy weekend value to a physical code."""
    if isinstance(compound, PhysicalTireCompound):
        return compound
    if isinstance(compound, TireCompound):
        return LEGACY_TIRE_COMPOUND_TO_PHYSICAL[compound]
    raw = str(compound).upper()
    try:
        return PhysicalTireCompound(raw)
    except ValueError:
        try:
            return LEGACY_TIRE_COMPOUND_TO_PHYSICAL[TireCompound(raw)]
        except ValueError as exc:
            raise ValueError(f"unknown tire compound {compound}") from exc


def compound_spec_for(compound: PhysicalTireCompound | TireCompound | str) -> CompoundSpec:
    return COMPOUND_SPECS[physical_compound_for(compound)]


def physical_compound_for_state(state) -> PhysicalTireCompound:
    """Read the session-owned physical code with legacy-state fallback."""
    physical = getattr(state, "physical_tire_compound", None)
    if physical is not None:
        return physical_compound_for(physical)
    return physical_compound_for(getattr(state, "tire_compound", None))


def tire_management_age_multiplier(tire_management: float) -> float:
    """Return how quickly tire age should count after driver management."""
    multiplier = 1.0 + (TIRE_MANAGEMENT_BASELINE - tire_management) * TIRE_MANAGEMENT_SENSITIVITY
    return min(MAX_MANAGED_AGE_MULTIPLIER, max(MIN_MANAGED_AGE_MULTIPLIER, multiplier))


def compute_managed_tire_age(tire_age: float, tire_management: float) -> float:
    """Return tire age adjusted by driver tire management skill."""
    return max(0.0, tire_age) * tire_management_age_multiplier(tire_management)


def compute_wear(compound: PhysicalTireCompound | TireCompound | str, tire_age: float) -> float:
    """Return curved wear value in 0.0–1.0 range for UI/strategy."""
    spec = compound_spec_for(compound)
    age_ratio = max(0.0, tire_age / spec.cliff_threshold)
    return min(1.0, age_ratio ** 1.35)


def circuit_tire_usage_per_lap(
    track_length_m: float,
    abrasion_multiplier: float,
) -> float:
    """Return one lap as Bahrain-equivalent mechanical tire usage."""
    distance_ratio = max(0.1, float(track_length_m)) / TIRE_WEAR_REFERENCE_LAP_DISTANCE_M
    return distance_ratio * max(0.65, min(1.35, float(abrasion_multiplier)))


def tire_thermal_wear_multiplier(
    compound: PhysicalTireCompound | TireCompound | str,
    surface_temperature_c: float | None,
) -> float:
    """Apply bounded extra wear only outside the compound's operating window."""
    if surface_temperature_c is None:
        return 1.0
    spec = compound_spec_for(compound)
    deviation = abs(float(surface_temperature_c) - spec.optimal_surface_temperature_c)
    excess = max(0.0, deviation - spec.surface_operating_window_c)
    return min(MAX_THERMAL_WEAR_MULTIPLIER, 1.0 + excess * 0.01)


def compute_tire_performance(
    compound: PhysicalTireCompound | TireCompound | str,
    tire_age: float,
    performance_variation: float = 0.0,
) -> float:
    """Return tire performance multiplier (higher = faster)."""
    spec = compound_spec_for(compound)
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
    compound: PhysicalTireCompound | TireCompound | str,
    tire_age: float,
    performance_variation: float = 0.0,
    temperature_c: float | None = None,
) -> TirePhysicsFactors:
    """Return compound- and wear-dependent force factors for dry-track physics."""
    spec = compound_spec_for(compound)
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


def tire_grip_indices_c3(
    factors: TirePhysicsFactors,
) -> tuple[float, float, float]:
    """Normalize live force factors to a fresh, optimal-temperature C3."""
    baseline = COMPOUND_SPECS[PhysicalTireCompound.C3]
    return (
        factors.lateral_grip / baseline.lateral_grip,
        factors.traction_grip / baseline.traction_grip,
        factors.braking_grip / baseline.braking_grip,
    )


def tire_blanket_temperature_c(compound: PhysicalTireCompound | TireCompound | str) -> float:
    return compound_spec_for(compound).blanket_temperature_c


def tire_temperature_grip_factor(
    compound: PhysicalTireCompound | TireCompound | str,
    temperature_c: float | None,
) -> float:
    """Return a continuous cold/optimal/hot grip multiplier."""
    if temperature_c is None:
        return 1.0
    spec = compound_spec_for(compound)
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
    compound: PhysicalTireCompound | TireCompound | str,
    *,
    surface_temperature_c: float,
    core_temperature_c: float,
    delta_seconds: float,
    speed_mps: float,
    lateral_acceleration_mps2: float,
    throttle: float,
    brake: float,
    slide_energy_j: float,
    ambient_temperature_c: float,
    track_temperature_c: float,
    baseline_heat_share: float = 1.0,
    lateral_heat_share: float = 1.0,
    brake_heat_share: float = 1.0,
    traction_heat_share: float = 1.0,
    slide_heat_share: float = 1.0,
    thermal_mass_share: float = 1.0,
    applied_drive_energy_j: float | None = None,
    applied_brake_work_energy_j: float | None = None,
) -> TireThermalState:
    """Advance a deterministic two-node tyre heat model.

    Heat is calculated as step energy and then converted to temperature using
    the two node heat capacities.  Axle-level callers pass actual force-work
    energies for traction and braking; direct callers may omit those values and
    use the legacy input-based fallback for isolated model tests.
    """
    numeric_inputs = {
        "surface_temperature_c": surface_temperature_c,
        "core_temperature_c": core_temperature_c,
        "delta_seconds": delta_seconds,
        "speed_mps": speed_mps,
        "lateral_acceleration_mps2": lateral_acceleration_mps2,
        "throttle": throttle,
        "brake": brake,
        "slide_energy_j": slide_energy_j,
        "ambient_temperature_c": ambient_temperature_c,
        "track_temperature_c": track_temperature_c,
    }
    if applied_drive_energy_j is not None:
        numeric_inputs["applied_drive_energy_j"] = applied_drive_energy_j
    if applied_brake_work_energy_j is not None:
        numeric_inputs["applied_brake_work_energy_j"] = applied_brake_work_energy_j
    if not all(isfinite(float(value)) for value in numeric_inputs.values()):
        raise ValueError("tire thermal inputs must be finite numbers")

    step = max(0.0, float(delta_seconds))
    surface = float(surface_temperature_c)
    core = float(core_temperature_c)
    speed = max(0.0, float(speed_mps))
    mass_share = max(0.1, min(1.0, float(thermal_mass_share)))
    if step <= 0.0:
        return TireThermalState(
            surface_temperature_c=surface,
            core_temperature_c=core,
            thermal_grip=tire_temperature_grip_factor(compound, surface),
            budget=TireThermalBudget.zero(),
            surface_unclamped_temperature_c=surface,
            core_unclamped_temperature_c=core,
            surface_clamp_hit=False,
            core_clamp_hit=False,
            surface_clamp_overshoot_c=0.0,
            core_clamp_overshoot_c=0.0,
        )

    baseline_heat_j = (
        1000.0
        * max(0.0, float(baseline_heat_share))
        * step
    )
    lateral_heat_j = (
        abs(float(lateral_acceleration_mps2))
        * speed
        * TIRE_LATERAL_MOTION_TO_HEAT_FACTOR
        * max(0.0, float(lateral_heat_share))
        * step
    )
    if applied_brake_work_energy_j is None:
        braking_heat_j = (
            max(0.0, float(brake))
            * 7000.0
            * max(0.0, float(brake_heat_share))
            * step
        )
    else:
        braking_heat_j = (
            max(0.0, float(applied_brake_work_energy_j))
            * TIRE_BRAKING_WORK_TO_HEAT_FACTOR
        )
    if applied_drive_energy_j is None:
        traction_heat_j = (
            max(0.0, float(throttle))
            * 1800.0
            * max(0.0, float(traction_heat_share))
            * step
        )
    else:
        traction_heat_j = (
            max(0.0, float(applied_drive_energy_j))
            * TIRE_TRACTION_WORK_TO_HEAT_FACTOR
            * max(0.0, float(traction_heat_share))
        )
    slide_heat_j = (
        max(0.0, float(slide_energy_j))
        * TIRE_SLIDE_ENERGY_TO_HEAT_FACTOR
        * max(0.0, float(slide_heat_share))
    )
    heat_input_j = (
        baseline_heat_j
        + lateral_heat_j
        + braking_heat_j
        + traction_heat_j
        + slide_heat_j
    )
    mechanical_heat_w = heat_input_j / step
    surface_cooling_w = (
        max(0.0, surface - float(track_temperature_c))
        * 42.0
        * (1.0 + speed / 55.0)
        * mass_share
    )
    surface_core_exchange_w = (surface - core) * 230.0 * mass_share
    core_cooling_w = max(0.0, core - float(ambient_temperature_c)) * 14.0 * mass_share
    surface_air_track_cooling_j = surface_cooling_w * step
    surface_to_core_transfer_j = surface_core_exchange_w * step
    core_ambient_cooling_j = core_cooling_w * step
    surface_net_energy_j = (
        heat_input_j
        - surface_air_track_cooling_j
        - surface_to_core_transfer_j
    )
    core_net_energy_j = surface_to_core_transfer_j - core_ambient_cooling_j
    surface += surface_net_energy_j / (50000.0 * mass_share)
    core += core_net_energy_j / (160000.0 * mass_share)
    surface_unclamped = surface
    core_unclamped = core
    surface_clamp_hit = surface_unclamped > TIRE_SURFACE_NUMERIC_GUARD_C
    core_clamp_hit = core_unclamped > TIRE_CORE_NUMERIC_GUARD_C
    surface = max(
        float(ambient_temperature_c),
        min(TIRE_SURFACE_NUMERIC_GUARD_C, surface_unclamped),
    )
    core = max(
        float(ambient_temperature_c),
        min(TIRE_CORE_NUMERIC_GUARD_C, core_unclamped),
    )
    return TireThermalState(
        surface_temperature_c=surface,
        core_temperature_c=core,
        thermal_grip=tire_temperature_grip_factor(compound, surface),
        budget=TireThermalBudget(
            baseline_heat_j=baseline_heat_j,
            lateral_heat_j=lateral_heat_j,
            braking_heat_j=braking_heat_j,
            traction_heat_j=traction_heat_j,
            slide_heat_j=slide_heat_j,
            surface_air_track_cooling_j=surface_air_track_cooling_j,
            surface_to_core_transfer_j=surface_to_core_transfer_j,
            core_ambient_cooling_j=core_ambient_cooling_j,
            surface_net_energy_j=surface_net_energy_j,
            core_net_energy_j=core_net_energy_j,
        ),
        surface_unclamped_temperature_c=surface_unclamped,
        core_unclamped_temperature_c=core_unclamped,
        surface_clamp_hit=surface_clamp_hit,
        core_clamp_hit=core_clamp_hit,
        surface_clamp_overshoot_c=max(
            0.0,
            surface_unclamped - TIRE_SURFACE_NUMERIC_GUARD_C,
        ),
        core_clamp_overshoot_c=max(
            0.0,
            core_unclamped - TIRE_CORE_NUMERIC_GUARD_C,
        ),
    )
