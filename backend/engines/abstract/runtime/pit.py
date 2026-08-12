"""Stage D deterministic pit-stop planning for the abstract race."""

from __future__ import annotations

# Implementation authority: engines.abstract.runtime

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from .rng import RNGStream
from .state import AbstractEntrySnapshot


PIT_PHASES = ("entry", "lane", "stop", "exit")


@dataclass(frozen=True, slots=True)
class PitStopPlan:
    driver_id: int | str
    stop_lap: int
    replacement_compound: str
    replacement_role: str
    service_time_s: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "driver_id": self.driver_id,
            "stop_lap": self.stop_lap,
            "replacement_compound": self.replacement_compound,
            "replacement_role": self.replacement_role,
            "service_time_s": round(self.service_time_s, 9),
        }


def _replacement_compound(current: str) -> str:
    return {
        "C5": "C3",
        "C4": "C3",
        "C3": "C2",
        "C2": "C1",
        "C1": "C1",
    }.get(current, current)


def build_pit_plans(
    entries: Sequence[AbstractEntrySnapshot],
    *,
    total_laps: int,
    pit_strategy: Mapping[int | str, Sequence[int]] | None,
    pit_streams: Mapping[int | str, RNGStream],
) -> dict[int | str, tuple[PitStopPlan, ...]]:
    """Build validated, stable plans; a missing strategy entry means no stop."""

    ordered_entries = tuple(sorted(entries, key=lambda entry: (str(entry.driver_id), entry.vehicle_id)))
    plans: dict[int | str, tuple[PitStopPlan, ...]] = {}
    for index, entry in enumerate(ordered_entries):
        if pit_strategy is None:
            if total_laps < 5:
                requested_laps: Sequence[int] = ()
            else:
                base_lap = max(2, total_laps // 2)
                requested_laps = (min(total_laps - 1, base_lap + index % 3),)
        else:
            requested_laps = pit_strategy.get(entry.driver_id, ())
        stop_laps = tuple(sorted(set(int(lap) for lap in requested_laps)))
        if any(lap < 1 or lap >= total_laps for lap in stop_laps):
            raise ValueError("pit stop laps must be within the race and before the final lap")
        current_compound = entry.tire.physical_compound
        plans[entry.driver_id] = tuple(
            PitStopPlan(
                driver_id=entry.driver_id,
                stop_lap=stop_lap,
                replacement_compound=_replacement_compound(current_compound),
                replacement_role="MEDIUM",
                service_time_s=pit_streams[entry.driver_id].uniform(1.90, 2.30),
            )
            for stop_lap in stop_laps
        )
    return plans
