"""Small contracts shared by the authoritative simulation state and output."""

from dataclasses import dataclass
from enum import Enum


class TickPhase(str, Enum):
    """Ordered phases of one authoritative simulation tick."""

    COMMAND = "command"
    PHYSICS = "physics"
    RULES = "rules"
    TELEMETRY = "telemetry"


@dataclass(frozen=True)
class ProgressCrossingFact:
    """Raw physical crossing consumed by the rules phase."""

    driver_id: int
    previous_progress: float
    progress_delta: float


@dataclass(frozen=True)
class CollisionFact:
    """Raw contact fact; it is not a published race event."""

    first_driver_id: int
    second_driver_id: int
    impact_speed_mps: float
    contact_type: str
    normal_longitudinal: float
    normal_lateral: float
    first_speed_mps: float
    second_speed_mps: float


@dataclass(frozen=True)
class PhysicsStepResult:
    """Facts emitted by PHYSICS and consumed exactly once by RULES."""

    physics_frame_id: int
    delta_seconds: float
    progress_crossings: tuple[ProgressCrossingFact, ...] = ()
    collision_facts: tuple[CollisionFact, ...] = ()


TELEMETRY_SOURCE_PHYSICS = "physics"
TELEMETRY_SOURCE_DERIVED = "physics_derived"
