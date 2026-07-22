"""Continuous aerodynamic wake model for towing and dirty air."""

from __future__ import annotations

from dataclasses import dataclass

WAKE_MIN_GAP_M = 4.5
WAKE_MAX_GAP_M = 120.0
WAKE_FULL_SPEED_MPS = 60.0
WAKE_MIN_SPEED_MPS = 15.0
WAKE_DIRTY_AIR_ONSET_CURVATURE_1PM = 0.0015
WAKE_DIRTY_AIR_FULL_CURVATURE_1PM = 0.012


@dataclass(frozen=True)
class WakeEffects:
    wake_strength: float = 0.0
    tow_strength: float = 0.0
    dirty_air_strength: float = 0.0
    drag_multiplier: float = 1.0
    downforce_multiplier: float = 1.0
    braking_grip_multiplier: float = 1.0
    lateral_grip_multiplier: float = 1.0


def _clamp01(value: float) -> float:
    return min(1.0, max(0.0, value))


def _smoothstep(value: float) -> float:
    value = _clamp01(value)
    return value * value * (3.0 - 2.0 * value)


def compute_wake_effects(
    longitudinal_gap_m: float,
    lateral_separation_m: float,
    curvature_1pm: float,
    speed_mps: float,
) -> WakeEffects:
    """Return wake effects from physical separation and local track curvature."""
    gap_m = float(longitudinal_gap_m)
    if gap_m <= WAKE_MIN_GAP_M or gap_m >= WAKE_MAX_GAP_M:
        return WakeEffects()

    distance_ratio = (
        (WAKE_MAX_GAP_M - gap_m)
        / (WAKE_MAX_GAP_M - WAKE_MIN_GAP_M)
    )
    distance_strength = _smoothstep(distance_ratio)

    wake_half_width_m = 1.2 + 0.025 * gap_m
    effective_lateral_m = max(0.0, abs(lateral_separation_m) - 0.95)
    lateral_strength = _smoothstep(
        1.0 - effective_lateral_m / max(0.1, wake_half_width_m)
    )
    speed_strength = _smoothstep(
        (speed_mps - WAKE_MIN_SPEED_MPS)
        / (WAKE_FULL_SPEED_MPS - WAKE_MIN_SPEED_MPS)
    )
    wake_strength = _clamp01(
        distance_strength * lateral_strength * speed_strength
    )
    if wake_strength <= 1e-6:
        return WakeEffects()

    # A wake exists on every straight, but aerodynamic balance loss is only
    # "dirty air" once the car actually asks for meaningful cornering load.
    # Using an onset band prevents tiny spline curvature/noise on a visually
    # straight section from lighting the dirty-air state.
    corner_demand = _smoothstep(
        (
            abs(curvature_1pm) - WAKE_DIRTY_AIR_ONSET_CURVATURE_1PM
        )
        / (
            WAKE_DIRTY_AIR_FULL_CURVATURE_1PM
            - WAKE_DIRTY_AIR_ONSET_CURVATURE_1PM
        )
    )
    tow_strength = wake_strength * (1.0 - 0.80 * corner_demand)
    dirty_air_strength = wake_strength * corner_demand

    return WakeEffects(
        wake_strength=wake_strength,
        tow_strength=tow_strength,
        dirty_air_strength=dirty_air_strength,
        drag_multiplier=1.0 - 0.10 * tow_strength,
        downforce_multiplier=(
            1.0 - wake_strength * 0.155 * corner_demand
        ),
        braking_grip_multiplier=(
            1.0 - wake_strength * 0.080 * corner_demand
        ),
        lateral_grip_multiplier=1.0 - 0.020 * dirty_air_strength,
    )
