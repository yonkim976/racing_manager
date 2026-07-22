"""Convert constructor specifications into independent vehicle performance factors."""

from __future__ import annotations

from dataclasses import dataclass

from models.schemas import Team

REFERENCE_MASS_KG = 768.0
REFERENCE_ENGINE_POWER_KW = 750.0
REFERENCE_DRIVETRAIN_EFFICIENCY = 0.965
REFERENCE_CORNER_DRAG_AREA_M2 = 1.10
REFERENCE_STRAIGHT_DRAG_AREA_M2 = 0.85
REFERENCE_CORNER_DOWNFORCE_AREA_M2 = 5.20
REFERENCE_STRAIGHT_DOWNFORCE_AREA_M2 = 3.30
REFERENCE_BRAKE_FORCE_N = 44000.0
REFERENCE_FRONT_AERO_SHARE = 0.455


@dataclass(frozen=True)
class CarPerformanceFactors:
    mass_kg: float
    effective_power_kw: float
    drivetrain_efficiency: float
    corner_drag_area_m2: float
    straight_drag_area_m2: float
    corner_downforce_area_m2: float
    straight_downforce_area_m2: float
    brake_force_n: float
    mechanical_grip: float
    front_aero_share: float
    traction_factor: float
    power: float
    top_speed: float
    high_speed_grip: float
    low_speed_grip: float
    braking: float
    traction: float
    qualifying: float


def _compress(value: float, strength: float = 0.35) -> float:
    return 1.0 + (value - 1.0) * strength


def car_performance_factors(team: Team) -> CarPerformanceFactors:
    """Return normalized factors while preserving distinct constructor strengths."""
    mass_factor = REFERENCE_MASS_KG / team.mass_kg
    raw_power = (
        team.engine_power_kw / REFERENCE_ENGINE_POWER_KW
        * team.drivetrain_efficiency / REFERENCE_DRIVETRAIN_EFFICIENCY
        * mass_factor
    )
    raw_top_speed = (
        raw_power
        * REFERENCE_STRAIGHT_DRAG_AREA_M2
        / team.straight_drag_area_m2
    ) ** (1.0 / 3.0)
    raw_high_speed = (
        team.corner_downforce_area_m2 / REFERENCE_CORNER_DOWNFORCE_AREA_M2
        * team.mechanical_grip
        * mass_factor
    ) ** 0.5
    raw_low_speed = team.mechanical_grip * mass_factor ** 0.35
    raw_braking = (
        team.brake_force_n / REFERENCE_BRAKE_FORCE_N
        * mass_factor
    )
    raw_traction = team.traction_factor * team.mechanical_grip * mass_factor

    power = _compress(raw_power)
    top_speed = _compress(raw_top_speed)
    high_speed = _compress(raw_high_speed)
    low_speed = _compress(raw_low_speed)
    braking = _compress(raw_braking)
    traction = _compress(raw_traction)
    qualifying = (
        top_speed * 0.24
        + high_speed * 0.27
        + low_speed * 0.18
        + braking * 0.16
        + traction * 0.15
    )
    return CarPerformanceFactors(
        mass_kg=team.mass_kg,
        effective_power_kw=team.engine_power_kw,
        drivetrain_efficiency=team.drivetrain_efficiency,
        corner_drag_area_m2=team.corner_drag_area_m2,
        straight_drag_area_m2=team.straight_drag_area_m2,
        corner_downforce_area_m2=team.corner_downforce_area_m2,
        straight_downforce_area_m2=team.straight_downforce_area_m2,
        brake_force_n=team.brake_force_n,
        mechanical_grip=team.mechanical_grip,
        front_aero_share=team.front_aero_share,
        traction_factor=team.traction_factor,
        power=power,
        top_speed=top_speed,
        high_speed_grip=high_speed,
        low_speed_grip=low_speed,
        braking=braking,
        traction=traction,
        qualifying=qualifying,
    )
