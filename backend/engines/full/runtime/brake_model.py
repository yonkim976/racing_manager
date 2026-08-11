"""Deterministic axle-level brake thermal model for the FULL runtime.

The race simulation does not need a full brake-disc finite-element model, but
it does need persistent heat, cooling and fade so strategy and telemetry can
observe the consequence of repeated braking.  Temperatures represent the
front and rear axle averages.
"""

from __future__ import annotations

from dataclasses import dataclass


BRAKE_MINIMUM_TEMPERATURE_C = 30.0
BRAKE_MAXIMUM_TEMPERATURE_C = 1200.0
BRAKE_OPTIMAL_MINIMUM_C = 250.0
BRAKE_FADE_START_C = 950.0
BRAKE_AXLE_HEAT_CAPACITY_J_PER_C = 90_000.0
BRAKE_ENERGY_ABSORPTION = 0.65
BRAKE_BASE_COOLING_W_PER_C = 18.0
BRAKE_AIRFLOW_COOLING_W_PER_C_PER_MPS = 1.0


@dataclass(frozen=True)
class BrakeThermalState:
    front_temperature_c: float
    rear_temperature_c: float
    fade_factor: float


def brake_temperature_force_factor(
    front_temperature_c: float,
    rear_temperature_c: float,
) -> float:
    """Return available braking force from the current axle temperatures."""
    coldest = min(float(front_temperature_c), float(rear_temperature_c))
    hottest = max(float(front_temperature_c), float(rear_temperature_c))
    cold_factor = 1.0
    if coldest < BRAKE_OPTIMAL_MINIMUM_C:
        cold_factor = 0.92 + 0.08 * max(
            0.0,
            min(
                1.0,
                (coldest - BRAKE_MINIMUM_TEMPERATURE_C)
                / (BRAKE_OPTIMAL_MINIMUM_C - BRAKE_MINIMUM_TEMPERATURE_C),
            ),
        )
    hot_factor = 1.0
    if hottest > BRAKE_FADE_START_C:
        hot_factor = 1.0 - 0.22 * min(
            1.0,
            (hottest - BRAKE_FADE_START_C)
            / (BRAKE_MAXIMUM_TEMPERATURE_C - BRAKE_FADE_START_C),
        )
    return max(0.78, min(cold_factor, hot_factor))


def advance_brake_thermal_state(
    *,
    front_temperature_c: float,
    rear_temperature_c: float,
    delta_seconds: float,
    speed_mps: float,
    applied_brake_force_n: float,
    front_brake_bias: float,
    ambient_temperature_c: float,
) -> BrakeThermalState:
    """Advance axle-average disc temperatures from braking work and airflow."""
    step = max(0.0, float(delta_seconds))
    speed = max(0.0, float(speed_mps))
    brake_force = max(0.0, float(applied_brake_force_n))
    front_bias = max(0.50, min(0.68, float(front_brake_bias)))
    ambient_temperature = float(ambient_temperature_c)
    rear_bias = 1.0 - front_bias
    braking_heat_w = brake_force * speed * BRAKE_ENERGY_ABSORPTION

    def advance_axle(temperature_c: float, energy_share: float) -> float:
        temperature = float(temperature_c)
        # Cooling grows with airflow, while a small low-speed coefficient keeps
        # parked and safety-car running deterministic without instant cooling.
        cooling_w = max(
            0.0,
            temperature - ambient_temperature,
        ) * (
            BRAKE_BASE_COOLING_W_PER_C
            + BRAKE_AIRFLOW_COOLING_W_PER_C_PER_MPS * speed
        )
        delta_c = (
            braking_heat_w * energy_share - cooling_w
        ) / BRAKE_AXLE_HEAT_CAPACITY_J_PER_C * step
        return max(
            BRAKE_MINIMUM_TEMPERATURE_C,
            min(BRAKE_MAXIMUM_TEMPERATURE_C, temperature + delta_c),
        )

    front = advance_axle(front_temperature_c, front_bias)
    rear = advance_axle(rear_temperature_c, rear_bias)
    return BrakeThermalState(
        front_temperature_c=front,
        rear_temperature_c=rear,
        fade_factor=brake_temperature_force_factor(front, rear),
    )
