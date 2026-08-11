"""Engine-neutral contracts for compiled track geometry and display poses."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


DRIVING_LINE_RACING = "racing_line"
DRIVING_LINE_INSIDE = "inside"
DRIVING_LINE_OUTSIDE = "outside"
DRIVING_LINE_DEFENSIVE = "defensive_line"


@dataclass(frozen=True)
class TrackPhysicsSample:
    progress: float
    left_width_m: float
    right_width_m: float
    racing_line_offset_m: float
    turn_signal: float


@dataclass(frozen=True)
class LocalMetricCoordinateFrame:
    """Transform compiled render units into immutable local track metres."""

    origin_x_render: float
    origin_y_render: float
    meters_per_render_unit: float
    track_length_m: float

    def to_local_m(self, x_render: float, y_render: float) -> tuple[float, float]:
        return (
            (x_render - self.origin_x_render) * self.meters_per_render_unit,
            (y_render - self.origin_y_render) * self.meters_per_render_unit,
        )

    def from_local_m(self, x_m: float, y_m: float) -> tuple[float, float]:
        scale = max(1e-12, self.meters_per_render_unit)
        return (
            self.origin_x_render + x_m / scale,
            self.origin_y_render + y_m / scale,
        )


class TrackGeometryProfile(Protocol):
    """Minimum compiled-line interface shared by FULL and ABSTRACT.

    The protocol deliberately excludes optimizer diagnostics, tire forces and
    vehicle-controller state. Implementations may provide those privately, but
    presentation and grid consumers only depend on this geometry surface.
    """

    coordinate_frame: LocalMetricCoordinateFrame | None

    def at_progress(self, progress: float) -> TrackPhysicsSample: ...

    def line_offset_at_progress(self, name: str, progress: float) -> float: ...

    def line_pose_at_progress_m(
        self,
        name: str,
        progress: float,
    ) -> tuple[float, float, float]: ...

    def line_distance_at_total_progress(
        self,
        name: str,
        total_progress: float,
    ) -> float: ...

    def total_progress_at_line_distance(
        self,
        name: str,
        distance_m: float,
    ) -> float: ...

    def length_for_line(self, name: str) -> float: ...
