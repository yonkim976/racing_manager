"""Pure starting-grid display geometry shared by FULL and ABSTRACT."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable

from simulation.track_contracts import TrackGeometryProfile


GRID_SLOT_PROGRESS_GAP = 0.0028
GRID_POLE_DISTANCE_BEHIND_LINE_M = 8.0
GRID_SLOT_SPACING_M = 8.0
GRID_COLUMN_OFFSET_M = 2.35


@dataclass(frozen=True, slots=True)
class GridDisplaySlot:
    """Immutable physical grid-box placement in the shared display contract."""

    position: int
    progress: float
    lateral_offset_m: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "position": self.position,
            "progress": round(self.progress, 9),
            "lateral_offset_m": round(self.lateral_offset_m, 9),
        }


def build_grid_slots(
    driver_ids: Iterable[int | str],
    track_length_m: float,
    *,
    start_sequence_enabled: bool = True,
    track_profile: TrackGeometryProfile | None = None,
) -> tuple[GridDisplaySlot, ...]:
    """Build the same staggered boxes used by the FULL start operation.

    The helper is deliberately independent of ``RaceEngine`` state.  The
    non-start-sequence branch remains available for isolated FULL callers and
    uses the compiled racing-line offset when a profile is provided.
    """

    length_m = max(1.0, float(track_length_m))
    slots: list[GridDisplaySlot] = []
    for position, _driver_id in enumerate(driver_ids, start=1):
        if start_sequence_enabled:
            grid_distance_m = (
                GRID_POLE_DISTANCE_BEHIND_LINE_M
                + (position - 1) * GRID_SLOT_SPACING_M
            )
            progress = -grid_distance_m / length_m
            lateral_offset_m = (
                GRID_COLUMN_OFFSET_M if position % 2 == 1 else -GRID_COLUMN_OFFSET_M
            )
        else:
            progress = -(position - 1) * GRID_SLOT_PROGRESS_GAP
            lateral_offset_m = (
                track_profile.at_progress(progress).racing_line_offset_m
                if track_profile is not None
                else 0.0
            )
        slots.append(
            GridDisplaySlot(
                position=position,
                progress=progress % 1.0,
                lateral_offset_m=lateral_offset_m,
            )
        )
    return tuple(slots)
