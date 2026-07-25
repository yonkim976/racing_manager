"""Shared track-edge surface profile and four-wheel contact model."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Sequence

from models.schemas import (
    Circuit,
    RunoffSurface,
    TrackSide,
    TrackSurfaceZone,
)
from simulation.vehicle_dimensions import PHYSICAL_CAR_WHEELBASE_M
if TYPE_CHECKING:
    from simulation.track_physics import TrackPhysicsProfile

DEFAULT_KERB_WIDTH_M = 1.2
DEFAULT_RUNOFF_WIDTH_M = 3.0
DEFAULT_SAFETY_MARGIN_M = 4.0
CAR_WHEELBASE_M = PHYSICAL_CAR_WHEELBASE_M
CAR_WHEEL_TRACK_M = 1.65
AUTO_KERB_TURN_THRESHOLD = 0.08
TRAJECTORY_LOW_KERB_ALLOWANCE_M = 0.60


class WheelSurface(str, Enum):
    TRACK = "track"
    KERB_LOW = "kerb_low"
    KERB_HIGH = "kerb_high"
    ASPHALT_RUNOFF = "asphalt_runoff"
    GRASS = "grass"
    GRAVEL = "gravel"


TRAJECTORY_SURFACE_COST_SECONDS: dict[WheelSurface, float] = {
    WheelSurface.TRACK: 0.0,
    WheelSurface.KERB_LOW: 0.08,
    WheelSurface.KERB_HIGH: 2.0,
    WheelSurface.ASPHALT_RUNOFF: 3.0,
    WheelSurface.GRASS: 25.0,
    WheelSurface.GRAVEL: 30.0,
}


@dataclass(frozen=True)
class SurfaceProperties:
    lateral_grip: float
    braking_grip: float
    traction_grip: float
    drag_deceleration_mps2: float
    instability: float = 0.0


SURFACE_PROPERTIES: dict[WheelSurface, SurfaceProperties] = {
    WheelSurface.TRACK: SurfaceProperties(1.0, 1.0, 1.0, 0.0),
    WheelSurface.KERB_LOW: SurfaceProperties(0.96, 0.94, 0.90, 0.15, 0.08),
    WheelSurface.KERB_HIGH: SurfaceProperties(0.84, 0.78, 0.70, 0.85, 0.42),
    WheelSurface.ASPHALT_RUNOFF: SurfaceProperties(0.92, 0.90, 0.88, 0.20),
    WheelSurface.GRASS: SurfaceProperties(0.48, 0.42, 0.38, 3.5, 0.28),
    WheelSurface.GRAVEL: SurfaceProperties(0.36, 0.30, 0.24, 8.0, 0.48),
}


@dataclass(frozen=True)
class WheelContact:
    name: str
    progress: float
    lateral_offset_m: float
    surface: WheelSurface
    side: TrackSide | None
    distance_outside_white_line_m: float


@dataclass(frozen=True)
class VehicleSurfaceState:
    contacts: tuple[WheelContact, ...]
    surface_state: WheelSurface
    lateral_grip_multiplier: float
    braking_grip_multiplier: float
    traction_grip_multiplier: float
    drag_deceleration_mps2: float
    instability: float
    kerb_contact: bool
    off_track: bool
    track_limits_active: bool

    @property
    def wheel_surfaces(self) -> list[str]:
        return [contact.surface.value for contact in self.contacts]


@dataclass(frozen=True)
class TrajectorySurfaceAssessment:
    """Whole-line surface use and physical body-boundary validation."""

    cost_seconds: float
    body_boundary_violations: int
    low_kerb_contacts: int
    high_kerb_contacts: int
    runoff_contacts: int
    grass_contacts: int
    gravel_contacts: int


def _progress_in_range(progress: float, start: float, end: float) -> bool:
    progress %= 1.0
    start %= 1.0
    end %= 1.0
    if start <= end:
        return start <= progress <= end
    return progress >= start or progress <= end


def _runs_for_side(
    samples,
    side: TrackSide,
) -> list[tuple[float, float]]:
    """Return circular runs where this side belongs to an actual corner."""
    if not samples:
        return []
    active = [
        abs(sample.turn_signal) >= AUTO_KERB_TURN_THRESHOLD
        and (
            (sample.turn_signal > 0.0 and side == TrackSide.LEFT)
            or (sample.turn_signal < 0.0 and side == TrackSide.RIGHT)
        )
        for sample in samples
    ]
    # Add the exit kerb on the opposite side wherever a corner exists. This
    # intentionally overlaps the apex range in v1; explicit circuit data can
    # later trim entry/apex/exit endpoints independently.
    active = [
        value
        or (
            abs(sample.turn_signal) >= AUTO_KERB_TURN_THRESHOLD
            and (
                (sample.turn_signal > 0.0 and side == TrackSide.RIGHT)
                or (sample.turn_signal < 0.0 and side == TrackSide.LEFT)
            )
        )
        for value, sample in zip(active, samples)
    ]
    if not any(active):
        return []
    if all(active):
        return [(0.0, 1.0)]

    runs: list[tuple[int, int]] = []
    index = 0
    while index < len(active):
        if not active[index]:
            index += 1
            continue
        start = index
        while index + 1 < len(active) and active[index + 1]:
            index += 1
        runs.append((start, index))
        index += 1

    if active[0] and active[-1] and len(runs) > 1:
        first = runs.pop(0)
        last = runs.pop()
        runs.insert(0, (last[0], first[1]))

    spacing = 1.0 / len(samples)
    result = []
    for start_index, end_index in runs:
        start = (samples[start_index].progress - 0.5 * spacing) % 1.0
        end = (samples[end_index].progress + 0.5 * spacing) % 1.0
        result.append((start, end))
    return result


def generate_surface_zones(profile: TrackPhysicsProfile) -> list[TrackSurfaceZone]:
    """Generate low apex/exit kerbs from the signed centerline turn signal."""
    zones: list[TrackSurfaceZone] = []
    for side in (TrackSide.LEFT, TrackSide.RIGHT):
        for start, end in _runs_for_side(profile.samples, side):
            zones.append(
                TrackSurfaceZone(
                    start=start,
                    end=end,
                    side=side,
                    kerb_width_m=DEFAULT_KERB_WIDTH_M,
                    kerb_height="low",
                    runoff_surface=RunoffSurface.ASPHALT,
                    runoff_width_m=DEFAULT_RUNOFF_WIDTH_M,
                    generated=True,
                )
            )
    return zones


class TrackSurfaceProfile:
    """Classify wheel points against white lines, kerbs and runoff."""

    def __init__(
        self,
        track_profile: TrackPhysicsProfile,
        zones: list[TrackSurfaceZone],
    ) -> None:
        self.track_profile = track_profile
        self.zones = zones
        self._zones_by_side = {
            side: [zone for zone in zones if zone.side == side]
            for side in (TrackSide.LEFT, TrackSide.RIGHT)
        }

    @classmethod
    def for_circuit(
        cls,
        circuit: Circuit,
        track_profile: TrackPhysicsProfile,
    ) -> "TrackSurfaceProfile":
        zones = list(circuit.surface_zones) or generate_surface_zones(track_profile)
        return cls(track_profile, zones)

    def zone_at(self, progress: float, side: TrackSide) -> TrackSurfaceZone | None:
        explicit_match = None
        generated_match = None
        for zone in self._zones_by_side[side]:
            if not _progress_in_range(progress, zone.start, zone.end):
                continue
            if zone.generated:
                generated_match = zone
            else:
                explicit_match = zone
        return explicit_match or generated_match

    def classify_point(
        self,
        progress: float,
        lateral_offset_m: float,
    ) -> tuple[WheelSurface, TrackSide | None, float]:
        track = self.track_profile.at_progress(progress)
        if -track.right_width_m <= lateral_offset_m <= track.left_width_m:
            return WheelSurface.TRACK, None, 0.0

        if lateral_offset_m > track.left_width_m:
            side = TrackSide.LEFT
            outside = lateral_offset_m - track.left_width_m
        else:
            side = TrackSide.RIGHT
            outside = -track.right_width_m - lateral_offset_m

        zone = self.zone_at(progress, side)
        kerb_width = zone.kerb_width_m if zone is not None else 0.0
        if zone is not None and outside <= kerb_width:
            kerb = (
                WheelSurface.KERB_HIGH
                if zone.kerb_height == "high"
                else WheelSurface.KERB_LOW
            )
            return kerb, side, outside

        runoff_width = zone.runoff_width_m if zone is not None else DEFAULT_RUNOFF_WIDTH_M
        runoff_surface = (
            zone.runoff_surface if zone is not None else RunoffSurface.ASPHALT
        )
        if outside <= kerb_width + runoff_width:
            return WheelSurface(runoff_surface.value), side, outside
        return WheelSurface.GRASS, side, outside

    def trajectory_kerb_allowance_m(
        self,
        progress: float,
        side: TrackSide,
    ) -> float:
        """Return the portion of a low kerb usable by a nominal fast line."""
        zone = self.zone_at(progress, side)
        if zone is None or zone.kerb_height != "low":
            return 0.0
        return min(TRAJECTORY_LOW_KERB_ALLOWANCE_M, zone.kerb_width_m)

    def trajectory_optimization_widths(
        self,
        progress: float,
    ) -> tuple[float, float]:
        """Track widths extended only over a physically usable low kerb."""
        track = self.track_profile.at_progress(progress)
        return (
            track.left_width_m
            + self.trajectory_kerb_allowance_m(progress, TrackSide.LEFT),
            track.right_width_m
            + self.trajectory_kerb_allowance_m(progress, TrackSide.RIGHT),
        )

    def trajectory_body_widths(
        self,
        progress: float,
    ) -> tuple[float, float]:
        """Return the body envelope available over a complete low kerb.

        The optimized nominal line still uses only the conservative 0.60m
        kerb allowance above.  A local traffic candidate may, however, place
        two wheels on a low kerb while the other two remain inside the white
        line.  Using the complete low-kerb width for body validation permits
        that legal racecraft without making the racing line target the kerb.
        """
        track = self.track_profile.at_progress(progress)

        def allowance(side: TrackSide) -> float:
            zone = self.zone_at(progress, side)
            if zone is None or zone.kerb_height != "low":
                return 0.0
            return zone.kerb_width_m

        return (
            track.left_width_m + allowance(TrackSide.LEFT),
            track.right_width_m + allowance(TrackSide.RIGHT),
        )

    def trajectory_body_lateral_bounds(
        self,
        progress: float,
        *,
        body_width_m: float,
        edge_margin_m: float,
    ) -> tuple[float, float]:
        left_width_m, right_width_m = self.trajectory_body_widths(progress)
        half_width_m = max(0.0, body_width_m) / 2.0
        return (
            -right_width_m + half_width_m + edge_margin_m,
            left_width_m - half_width_m - edge_margin_m,
        )

    def assess_trajectory(
        self,
        progresses: Sequence[float],
        lateral_offsets_m: Sequence[float],
        *,
        track_length_m: float,
        body_width_m: float,
        body_length_m: float,
        edge_margin_m: float,
        heading_offsets_rad: Sequence[float] | None = None,
    ) -> TrajectorySurfaceAssessment:
        """Score wheel surfaces and validate front/rear body corners."""
        if len(progresses) != len(lateral_offsets_m):
            raise ValueError("trajectory progress and offsets must have equal size")
        if heading_offsets_rad is not None and len(heading_offsets_rad) != len(progresses):
            raise ValueError("trajectory headings and progress must have equal size")
        if not progresses:
            return TrajectorySurfaceAssessment(0.0, 0, 0, 0, 0, 0, 0)

        surface_counts = {surface: 0 for surface in WheelSurface}
        body_boundary_violations = 0
        surface_cost = 0.0
        half_length_progress = (
            max(0.0, body_length_m) / 2.0 / max(1.0, track_length_m)
        )
        headings = heading_offsets_rad or (0.0,) * len(progresses)
        for progress, lateral_offset_m, heading_rad in zip(
            progresses,
            lateral_offsets_m,
            headings,
        ):
            vehicle = self.vehicle_state(
                progress=progress,
                lateral_offset_m=lateral_offset_m,
                track_length_m=track_length_m,
                slip_angle_rad=heading_rad,
            )
            for contact in vehicle.contacts:
                surface_counts[contact.surface] += 1
                surface_cost += TRAJECTORY_SURFACE_COST_SECONDS[contact.surface]

            for axle_progress, axle_direction in (
                (progress - half_length_progress, -1.0),
                (progress + half_length_progress, 1.0),
            ):
                axle_lateral_offset_m = (
                    lateral_offset_m
                    + axle_direction * body_length_m / 2.0 * heading_rad
                )
                minimum, maximum = self.trajectory_body_lateral_bounds(
                    axle_progress,
                    body_width_m=body_width_m,
                    edge_margin_m=edge_margin_m,
                )
                if (
                    axle_lateral_offset_m < minimum
                    or axle_lateral_offset_m > maximum
                ):
                    body_boundary_violations += 1

        contact_count = len(progresses) * 4
        return TrajectorySurfaceAssessment(
            cost_seconds=surface_cost / max(1, contact_count),
            body_boundary_violations=body_boundary_violations,
            low_kerb_contacts=surface_counts[WheelSurface.KERB_LOW],
            high_kerb_contacts=surface_counts[WheelSurface.KERB_HIGH],
            runoff_contacts=surface_counts[WheelSurface.ASPHALT_RUNOFF],
            grass_contacts=surface_counts[WheelSurface.GRASS],
            gravel_contacts=surface_counts[WheelSurface.GRAVEL],
        )

    def vehicle_state(
        self,
        *,
        progress: float,
        lateral_offset_m: float,
        track_length_m: float,
        slip_angle_rad: float = 0.0,
    ) -> VehicleSurfaceState:
        half_wheelbase = CAR_WHEELBASE_M / 2.0
        half_track = CAR_WHEEL_TRACK_M / 2.0
        contacts: list[WheelContact] = []
        for axle_name, longitudinal_m in (("front", half_wheelbase), ("rear", -half_wheelbase)):
            axle_progress = progress + longitudinal_m / max(1.0, track_length_m)
            yaw_lateral_shift = longitudinal_m * slip_angle_rad
            for side_name, wheel_lateral_m in (("left", half_track), ("right", -half_track)):
                point_lateral = lateral_offset_m + wheel_lateral_m + yaw_lateral_shift
                surface, side, outside = self.classify_point(axle_progress, point_lateral)
                contacts.append(
                    WheelContact(
                        name=f"{axle_name}_{side_name}",
                        progress=axle_progress % 1.0,
                        lateral_offset_m=point_lateral,
                        surface=surface,
                        side=side,
                        distance_outside_white_line_m=outside,
                    )
                )

        properties = [SURFACE_PROPERTIES[contact.surface] for contact in contacts]
        average = lambda name: sum(getattr(item, name) for item in properties) / len(properties)
        rank = {
            WheelSurface.TRACK: 0,
            WheelSurface.KERB_LOW: 1,
            WheelSurface.ASPHALT_RUNOFF: 2,
            WheelSurface.KERB_HIGH: 3,
            WheelSurface.GRASS: 4,
            WheelSurface.GRAVEL: 5,
        }
        surface_state = max((contact.surface for contact in contacts), key=rank.get)
        kerb_contact = any("kerb" in contact.surface.value for contact in contacts)
        off_track = any(
            contact.surface
            in {WheelSurface.ASPHALT_RUNOFF, WheelSurface.GRASS, WheelSurface.GRAVEL}
            for contact in contacts
        )
        track_limits_active = all(contact.surface != WheelSurface.TRACK for contact in contacts)
        return VehicleSurfaceState(
            contacts=tuple(contacts),
            surface_state=surface_state,
            lateral_grip_multiplier=average("lateral_grip"),
            braking_grip_multiplier=average("braking_grip"),
            traction_grip_multiplier=average("traction_grip"),
            drag_deceleration_mps2=average("drag_deceleration_mps2"),
            instability=average("instability"),
            kerb_contact=kerb_contact,
            off_track=off_track,
            track_limits_active=track_limits_active,
        )

    def safety_lateral_bounds(self, progress: float) -> tuple[float, float]:
        track = self.track_profile.at_progress(progress)
        active_zones = [
            zone
            for side in (TrackSide.LEFT, TrackSide.RIGHT)
            if (zone := self.zone_at(progress, side)) is not None
        ]
        maximum_zone_runoff = max(
            (zone.kerb_width_m + zone.runoff_width_m for zone in active_zones),
            default=DEFAULT_RUNOFF_WIDTH_M,
        )
        envelope = maximum_zone_runoff + DEFAULT_SAFETY_MARGIN_M
        return -track.right_width_m - envelope, track.left_width_m + envelope
