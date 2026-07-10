"""Incident model: classifies on-track incidents by cause and severity.

Incidents are layered on top of the existing harmless battle events (lockup,
run_wide, traction_loss, light contact). An incident may stay ``MINOR`` (time
loss only) or escalate to ``CAR_STOPPED`` (driver out, virtual safety car) or
``CRASH`` (driver(s) out, full safety car).

Probabilities are expressed as a *rate per game-second per driver* so that the
expected number of incidents is independent of the simulation speed multiplier
(the engine always advances in fixed GAME_TICK_SECONDS steps).
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from enum import Enum


class IncidentCause(str, Enum):
    DRIVER_ERROR = "driver_error"
    COLLISION = "collision"
    MECHANICAL = "mechanical"


class IncidentSeverity(str, Enum):
    MINOR = "minor"            # time/tire loss, driver continues
    CAR_STOPPED = "car_stopped"  # driver out / stranded -> virtual safety car
    CRASH = "crash"           # driver(s) out, heavy -> full safety car


@dataclass
class Incident:
    cause: IncidentCause
    severity: IncidentSeverity
    primary_driver_id: int
    secondary_driver_id: int | None = None


# ── Base rates (per game-second, per driver) ──────────────────────────────────
DRIVER_ERROR_BASE_RATE = 2.6e-5
MECHANICAL_BASE_RATE = 1.0e-5

# ── Driver-error context multipliers ─────────────────────────────────────────
SEGMENT_RISK_MULTIPLIER = {
    "heavy_braking": 2.2,
    "technical": 1.5,
    "traction": 1.2,
    "sweeping": 0.9,
    "straight": 0.3,
}
ATTACK_MODE_RISK_MULTIPLIER = 1.6
DIRTY_AIR_RISK_MULTIPLIER = 1.35
# consistency 0..1 -> low consistency raises risk up to ~2.5x
CONSISTENCY_RISK_RANGE = 1.8
# tire wear 0..1 -> worn tires raise risk up to ~1.8x
TIRE_WEAR_RISK_RANGE = 0.8


def _driver_error_severity(
    rng: random.Random,
    *,
    tire_wear: float,
    is_high_speed: bool,
    attack_mode: bool,
) -> IncidentSeverity:
    """Most driver errors are recoverable; high speed / worn tires push it up."""
    car_stopped_p = 0.10 + 0.06 * tire_wear
    crash_p = 0.04 + (0.05 if is_high_speed else 0.0) + (0.02 if attack_mode else 0.0)
    roll = rng.random()
    if roll < crash_p:
        return IncidentSeverity.CRASH
    if roll < crash_p + car_stopped_p:
        return IncidentSeverity.CAR_STOPPED
    return IncidentSeverity.MINOR


def _mechanical_severity(rng: random.Random) -> IncidentSeverity:
    """Mechanical issues either cost time or stop the car; rarely a crash."""
    roll = rng.random()
    if roll < 0.55:
        return IncidentSeverity.CAR_STOPPED
    return IncidentSeverity.MINOR


def roll_solo_incident(
    rng: random.Random,
    driver_id: int,
    *,
    delta_seconds: float,
    consistency: float,
    tire_wear: float,
    segment_type: str | None,
    attack_mode: bool,
    dirty_air: bool,
    race_progress: float,
) -> Incident | None:
    """Roll for a single-car incident (driver error or mechanical) this tick."""
    # Mechanical failures: roughly constant, slightly more likely late in the race.
    mechanical_rate = MECHANICAL_BASE_RATE * (1.0 + 0.6 * race_progress)
    if rng.random() < mechanical_rate * delta_seconds:
        return Incident(
            cause=IncidentCause.MECHANICAL,
            severity=_mechanical_severity(rng),
            primary_driver_id=driver_id,
        )

    # Driver errors: scaled by consistency, tire wear, segment, mode, dirty air.
    risk = DRIVER_ERROR_BASE_RATE
    risk *= 1.0 + (1.0 - consistency) * CONSISTENCY_RISK_RANGE
    risk *= 1.0 + tire_wear * TIRE_WEAR_RISK_RANGE
    risk *= SEGMENT_RISK_MULTIPLIER.get(segment_type or "", 1.0)
    if attack_mode:
        risk *= ATTACK_MODE_RISK_MULTIPLIER
    if dirty_air:
        risk *= DIRTY_AIR_RISK_MULTIPLIER

    if rng.random() < risk * delta_seconds:
        is_high_speed = segment_type in ("straight", "sweeping")
        return Incident(
            cause=IncidentCause.DRIVER_ERROR,
            severity=_driver_error_severity(
                rng,
                tire_wear=tire_wear,
                is_high_speed=is_high_speed,
                attack_mode=attack_mode,
            ),
            primary_driver_id=driver_id,
        )
    return None


# ── Collision escalation ─────────────────────────────────────────────────────
COLLISION_CAR_STOPPED_PROBABILITY = 0.07
COLLISION_CRASH_PROBABILITY = 0.03
COLLISION_HEAVY_BRAKING_CRASH_BONUS = 0.04


def escalate_collision(
    rng: random.Random,
    *,
    segment_type: str | None,
) -> IncidentSeverity:
    """Decide whether a light contact stays minor or escalates to out/SC."""
    crash_p = COLLISION_CRASH_PROBABILITY
    if segment_type == "heavy_braking":
        crash_p += COLLISION_HEAVY_BRAKING_CRASH_BONUS
    roll = rng.random()
    if roll < crash_p:
        return IncidentSeverity.CRASH
    if roll < crash_p + COLLISION_CAR_STOPPED_PROBABILITY:
        return IncidentSeverity.CAR_STOPPED
    return IncidentSeverity.MINOR
