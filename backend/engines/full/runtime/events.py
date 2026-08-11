"""FULL random race events (spin, safety car, DNF)."""

from __future__ import annotations

import random
from dataclasses import dataclass

from models.schemas import RaceEvent


@dataclass
class EventResult:
    time_penalty: float = 0.0
    retired: bool = False
    safety_car: bool = False
    event: RaceEvent | None = None


def roll_events(
    driver_abbr: str,
    driver_name: str,
    rng: random.Random,
    *,
    enable_events: bool = True,
) -> EventResult:
    """Roll for random events on a single tick. Very low probability."""
    if not enable_events:
        return EventResult()

    roll = rng.random()

    if roll < 0.00005:
        return EventResult(
            retired=True,
            event=RaceEvent(
                type="dnf",
                driver=driver_abbr,
                message=f"{driver_name} retires — mechanical failure",
                message_ko=f"{driver_name} 기계적 문제로 리타이어합니다",
            ),
        )

    if roll < 0.00015:
        return EventResult(
            time_penalty=rng.uniform(2.0, 5.0),
            event=RaceEvent(
                type="spin",
                driver=driver_abbr,
                message=f"{driver_name} spins — loses time",
                message_ko=f"{driver_name} 스핀으로 시간을 잃습니다",
            ),
        )

    return EventResult()
