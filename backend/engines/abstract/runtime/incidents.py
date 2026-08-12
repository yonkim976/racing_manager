"""Stage D logical incident assessments.

The assessments deliberately return game-level outcomes rather than physical
quantities.  They are deterministic when supplied with the same snapshot
inputs and a value from the caller's isolated incident stream.
"""

from __future__ import annotations

# Implementation authority: engines.abstract.runtime

from dataclasses import dataclass
from typing import Any

from .state import TireConditionSnapshot


LOCKUP_SEGMENT_TYPE = "heavy_braking"


def _bounded_probability(value: float) -> float:
    return max(0.0, min(0.40, value))


@dataclass(frozen=True, slots=True)
class LockupAssessment:
    allowed: bool
    triggered: bool
    reason_code: str
    probability: float
    locked_axle: str | None
    severity: float
    time_loss_s: float
    run_wide: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "allowed": self.allowed,
            "triggered": self.triggered,
            "reason_code": self.reason_code,
            "probability": round(self.probability, 9),
            "locked_axle": self.locked_axle,
            "severity": round(self.severity, 9),
            "time_loss_s": round(self.time_loss_s, 9),
            "run_wide": self.run_wide,
        }


def assess_lockup(
    *,
    segment_type: str,
    braking: float,
    tire: TireConditionSnapshot,
    driver_consistency: float,
    attack_mode: bool,
    random_value: float,
    severity_value: float = 0.5,
    axle_value: float = 0.5,
    track_condition: str = "dry",
) -> LockupAssessment:
    """Assess one braking-entry opportunity, not every physics substep."""

    if segment_type != LOCKUP_SEGMENT_TYPE:
        return LockupAssessment(False, False, "segment_not_braking", 0.0, None, 0.0, 0.0, False)

    wear_risk = max(tire.wear_laps - 1.0, 0.0) * 0.006
    temperature_risk = {"cold": 0.006, "optimal": 0.0, "hot": 0.014}.get(
        tire.temperature_band,
        0.0,
    )
    weather_risk = 0.010 if track_condition in {"light_rain", "heavy_rain"} else 0.0
    probability = _bounded_probability(
        0.003
        + max(0.0, 1.0 - braking) * 0.030
        + max(0.0, 1.0 - driver_consistency) * 0.020
        + wear_risk
        + temperature_risk
        + weather_risk
        + (0.008 if attack_mode else 0.0)
    )
    if random_value >= probability:
        return LockupAssessment(True, False, "no_lockup", probability, None, 0.0, 0.0, False)

    severity = max(0.0, min(1.0, severity_value))
    time_loss_s = 0.12 + severity * 0.48
    return LockupAssessment(
        True,
        True,
        "lockup_triggered",
        probability,
        "front" if axle_value < 0.65 else "rear",
        severity,
        time_loss_s,
        severity >= 0.72,
    )


@dataclass(frozen=True, slots=True)
class ContactAssessment:
    allowed: bool
    triggered: bool
    reason_code: str
    probability: float
    template: str | None
    severity: float
    damage_delta: float
    time_loss_s: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "allowed": self.allowed,
            "triggered": self.triggered,
            "reason_code": self.reason_code,
            "probability": round(self.probability, 9),
            "template": self.template,
            "severity": round(self.severity, 9),
            "damage_delta": round(self.damage_delta, 9),
            "time_loss_s": round(self.time_loss_s, 9),
        }


def assess_contact(
    *,
    corridor_active: bool,
    closing_speed_mps: float,
    gap_m: float,
    overtaking_difficulty: float,
    attacker_risk_tolerance: float,
    random_value: float,
    severity_value: float = 0.5,
) -> ContactAssessment:
    """Assess contact only for an already reserved two-wide corridor."""

    if not corridor_active:
        return ContactAssessment(False, False, "no_active_corridor", 0.0, None, 0.0, 0.0, 0.0)
    if closing_speed_mps <= 0.0:
        return ContactAssessment(False, False, "no_closing_speed", 0.0, None, 0.0, 0.0, 0.0)

    probability = _bounded_probability(
        0.006
        + min(0.035, closing_speed_mps / 100.0)
        + max(0.0, 0.012 - gap_m * 0.0004)
        + overtaking_difficulty * 0.018
        + attacker_risk_tolerance * 0.012
    )
    if random_value >= probability:
        return ContactAssessment(True, False, "corridor_clear", probability, None, 0.0, 0.0, 0.0)

    severity = max(0.0, min(1.0, severity_value))
    return ContactAssessment(
        True,
        True,
        "minor_side_contact",
        probability,
        "minor_side_contact",
        severity,
        0.015 + severity * 0.055,
        0.20 + severity * 0.60,
    )
