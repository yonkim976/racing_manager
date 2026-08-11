"""Stage C traffic and overtake decision rules.

This module decides logical maneuver outcomes only.  It does not create
collision, damage, lockup, pit or safety-car events; those belong to later
stages of the abstract design.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil
from typing import Any, Sequence

from .performance import VehiclePerformance
from .state import AbstractDriverProfile


VEHICLE_WIDTH_M = 1.9
TWO_WIDE_MARGIN_M = 0.75
MIN_LONGITUDINAL_GAP_M = 5.0
ATTACK_WINDOW_M = 24.0
MIN_CLOSING_SPEED_MPS = 0.15
BODY_CLEARANCE_MARGIN_M = 0.15
TIME_ALIGNED_OCCUPANCY_STEP_S = 0.50

# These are provisional Stage 4 game guards used only to bound a future
# occupancy envelope.  They are not official F1 vehicle or physical values.
MAX_ACCEPTED_SPEED_MPS = 98.0
MAX_ACCEPTED_ACCELERATION_MPS2 = 18.0
MAX_ACCEPTED_BRAKING_MPS2 = 50.0

_ALLOWED_ATTACK_SEGMENTS = {"straight", "heavy_braking", "traction"}


def corridor_required_width_m() -> float:
    return 2.0 * VEHICLE_WIDTH_M + TWO_WIDE_MARGIN_M


def accepted_swept_distance_m(
    speed_mps: float,
    horizon_s: float,
    *,
    target_speed_mps: float | None = None,
) -> float:
    """Return a conservative accepted-distance forecast for an occupancy audit."""

    horizon = max(0.0, float(horizon_s))
    initial = min(MAX_ACCEPTED_SPEED_MPS, max(0.0, float(speed_mps)))
    target = (
        initial + MAX_ACCEPTED_ACCELERATION_MPS2 * horizon
        if target_speed_mps is None
        else min(MAX_ACCEPTED_SPEED_MPS, max(0.0, float(target_speed_mps)))
    )
    if target < initial - 1e-12:
        time_to_target = (initial - target) / MAX_ACCEPTED_BRAKING_MPS2
        if time_to_target >= horizon:
            return max(
                0.0,
                initial * horizon - 0.5 * MAX_ACCEPTED_BRAKING_MPS2 * horizon * horizon,
            )
        return (
            initial * time_to_target
            - 0.5 * MAX_ACCEPTED_BRAKING_MPS2 * time_to_target * time_to_target
            + target * (horizon - time_to_target)
        )
    if target <= initial + 1e-12:
        return initial * horizon
    time_to_target = (target - initial) / MAX_ACCEPTED_ACCELERATION_MPS2
    if time_to_target >= horizon:
        return initial * horizon + 0.5 * MAX_ACCEPTED_ACCELERATION_MPS2 * horizon * horizon
    return (
        initial * time_to_target
        + 0.5 * MAX_ACCEPTED_ACCELERATION_MPS2 * time_to_target * time_to_target
        + target * (horizon - time_to_target)
    )


def swept_vehicle_intersects_reservation(
    *,
    reservation_start_arc_m: float,
    reservation_end_arc_m: float,
    reservation_horizon_s: float,
    reservation_elapsed_s: float = 0.0,
    vehicle_start_arc_m: float,
    vehicle_speed_mps: float,
    vehicle_target_speed_mps: float | None,
    vehicle_lateral_offset_m: float,
    line_length_m: float,
    vehicle_length_m: float,
    vehicle_width_m: float,
    lateral_min_m: float,
    lateral_max_m: float,
    reservation_attacker_start_arc_m: float | None = None,
    reservation_attacker_speed_mps: float | None = None,
    reservation_attacker_target_speed_mps: float | None = None,
    reservation_defender_start_arc_m: float | None = None,
    reservation_defender_speed_mps: float | None = None,
    reservation_defender_target_speed_mps: float | None = None,
) -> bool:
    """Check body occupancy against an immutable reservation.

    Production admission/audit callers provide the current attacker and
    defender states.  Those calls compare all three cars at the same future
    logical instant instead of treating the complete eight-second swept arc as
    occupied at once.  The conservative swept-envelope fallback remains for
    standalone reservation audits that do not have participant state.
    """

    if line_length_m <= 0.0:
        raise ValueError("line_length_m must be positive")
    horizon = max(0.0, float(reservation_horizon_s))
    elapsed = min(horizon, max(0.0, float(reservation_elapsed_s)))
    remaining = max(0.0, horizon - elapsed)
    reservation_start, reservation_end = TrafficCorridorReservation._normalise_interval(
        reservation_start_arc_m,
        reservation_end_arc_m,
        line_length_m,
    )
    reservation_span = max(0.0, reservation_end - reservation_start)
    progress_fraction = elapsed / horizon if horizon > 1e-12 else 1.0
    current_reservation_center = reservation_start + reservation_span * progress_fraction
    remaining_reservation_span = reservation_end - current_reservation_center
    vehicle_swept = accepted_swept_distance_m(
        vehicle_speed_mps,
        remaining,
        target_speed_mps=vehicle_target_speed_mps,
    )
    half_length = max(0.0, float(vehicle_length_m)) * 0.5
    body_lateral_min = float(vehicle_lateral_offset_m) - float(vehicle_width_m) * 0.5
    body_lateral_max = float(vehicle_lateral_offset_m) + float(vehicle_width_m) * 0.5
    if max(lateral_min_m, body_lateral_min) >= min(lateral_max_m, body_lateral_max) - 1e-9:
        return False

    participant_values = (
        reservation_attacker_start_arc_m,
        reservation_attacker_speed_mps,
        reservation_defender_start_arc_m,
        reservation_defender_speed_mps,
    )
    if all(value is not None for value in participant_values):
        attacker_start = float(reservation_attacker_start_arc_m)
        defender_start = float(reservation_defender_start_arc_m)
        defender_start += round((attacker_start - defender_start) / line_length_m) * line_length_m
        third_start = float(vehicle_start_arc_m)
        third_start += round((attacker_start - third_start) / line_length_m) * line_length_m
        sample_count = max(1, int(ceil(remaining / TIME_ALIGNED_OCCUPANCY_STEP_S)))
        body_half_length = max(0.0, float(vehicle_length_m)) * 0.5
        # Between two deterministic samples, the largest midpoint deviation
        # from a linear chord is ``a * dt^2 / 8``.  Inflating the moving
        # corridor by the accepted relative-acceleration bound prevents a
        # brief crossing from hiding between samples without returning to the
        # prohibitively expensive 0.1-second full-field scan.
        sample_interval_s = remaining / sample_count
        interpolation_guard_m = (
            (MAX_ACCEPTED_ACCELERATION_MPS2 + MAX_ACCEPTED_BRAKING_MPS2)
            * sample_interval_s
            * sample_interval_s
            / 8.0
        )
        sample_indices = (0,) if remaining <= 1e-12 else range(sample_count + 1)
        for sample_index in sample_indices:
            sample_time_s = remaining * sample_index / sample_count
            attacker_center = attacker_start + accepted_swept_distance_m(
                float(reservation_attacker_speed_mps),
                sample_time_s,
                target_speed_mps=reservation_attacker_target_speed_mps,
            )
            defender_center = defender_start + accepted_swept_distance_m(
                float(reservation_defender_speed_mps),
                sample_time_s,
                target_speed_mps=reservation_defender_target_speed_mps,
            )
            third_center = third_start + accepted_swept_distance_m(
                vehicle_speed_mps,
                sample_time_s,
                target_speed_mps=vehicle_target_speed_mps,
            )
            corridor_min = (
                min(attacker_center, defender_center)
                - body_half_length
                - BODY_CLEARANCE_MARGIN_M
                - interpolation_guard_m
            )
            corridor_max = (
                max(attacker_center, defender_center)
                + body_half_length
                + BODY_CLEARANCE_MARGIN_M
                + interpolation_guard_m
            )
            third_min = third_center - body_half_length
            third_max = third_center + body_half_length
            if max(corridor_min, third_min) < min(corridor_max, third_max) - 1e-9:
                return True
        return False

    # Reservation-only fallback: without participant state the immutable swept
    # envelope is the only authoritative occupancy information available.
    current_vehicle_center = vehicle_start_arc_m
    current_vehicle_center += round(
        (current_reservation_center - current_vehicle_center) / line_length_m
    ) * line_length_m
    # An unlisted vehicle may already be inside the immutable swept corridor
    # at admission (or may enter it after admission).  This check covers the
    # whole accepted arc envelope; limiting it to the first attack window was
    # the conditional-approval hole that allowed a third car at mid-corridor.
    if (
        reservation_start - half_length - BODY_CLEARANCE_MARGIN_M
        <= current_vehicle_center
        <= reservation_end + half_length + BODY_CLEARANCE_MARGIN_M
    ):
        return True

    # Both forecasts are linear over this accepted audit interval.  Compare
    # the signed separation at both ends and detect a zero crossing instead
    # of relying on a coarse sample that could miss a brief overlap.
    aligned_vehicle_start = vehicle_start_arc_m + round(
        (current_reservation_center - vehicle_start_arc_m) / line_length_m
    ) * line_length_m
    separation_start = current_reservation_center - aligned_vehicle_start
    separation_end = reservation_end - (aligned_vehicle_start + vehicle_swept)
    clearance_threshold = half_length * 2.0 + BODY_CLEARANCE_MARGIN_M
    if min(abs(separation_start), abs(separation_end)) < clearance_threshold:
        return True
    return separation_start * separation_end < 0.0


@dataclass(frozen=True, slots=True)
class TrafficCorridorReservation:
    """Immutable admission record for one logical two-wide corridor.

    The arc interval is stored in unwrapped line-distance space whenever it
    is produced by the traffic cursor.  ``arc_intervals_overlap`` also
    accepts an explicit wrapped interval (``end < start``) for deterministic
    fixtures near start/finish.  Lateral bounds describe the occupied body
    envelope, not merely the two centre points.
    """

    corridor_id: str
    attacker_id: int | str
    defender_id: int | str
    start_arc_distance_m: float
    end_arc_distance_m: float
    start_time_s: float
    expiry_time_s: float
    lateral_min_m: float
    lateral_max_m: float
    side_assignment: tuple[tuple[int | str, str], ...]
    required_width_m: float
    available_width_m: float
    segment_type: str
    corner_phase: str
    third_vehicle_ids: tuple[int | str, ...]
    admission_reason: str
    vehicle_length_m: float = 5.4
    vehicle_width_m: float = VEHICLE_WIDTH_M
    occupancy_model: str = "swept_envelope"

    def __post_init__(self) -> None:
        if self.end_arc_distance_m == self.start_arc_distance_m:
            raise ValueError("corridor reservation arc interval must not be empty")
        if self.expiry_time_s <= self.start_time_s:
            raise ValueError("corridor reservation time window must be positive")
        if self.lateral_max_m <= self.lateral_min_m:
            raise ValueError("corridor reservation lateral bounds must be positive")
        if self.required_width_m <= 0.0 or self.available_width_m <= 0.0:
            raise ValueError("corridor reservation widths must be positive")
        if self.occupancy_model not in {"swept_envelope", "time_aligned"}:
            raise ValueError("unsupported corridor reservation occupancy model")
        object.__setattr__(self, "side_assignment", tuple(self.side_assignment))
        object.__setattr__(self, "third_vehicle_ids", tuple(sorted(self.third_vehicle_ids, key=str)))

    @staticmethod
    def _normalise_interval(
        start: float,
        end: float,
        line_length_m: float,
    ) -> tuple[float, float]:
        if line_length_m <= 0.0:
            raise ValueError("line_length_m must be positive")
        if end < start:
            end += line_length_m
        return start, end

    def arc_intervals_overlap(self, other: "TrafficCorridorReservation", *, line_length_m: float) -> bool:
        first = self._normalise_interval(
            self.start_arc_distance_m,
            self.end_arc_distance_m,
            line_length_m,
        )
        second = self._normalise_interval(
            other.start_arc_distance_m,
            other.end_arc_distance_m,
            line_length_m,
        )
        for shift in (-line_length_m, 0.0, line_length_m):
            shifted = (second[0] + shift, second[1] + shift)
            if max(first[0], shifted[0]) < min(first[1], shifted[1]) - 1e-9:
                return True
        return False

    def time_windows_overlap(self, other: "TrafficCorridorReservation") -> bool:
        return (
            max(self.start_time_s, other.start_time_s)
            < min(self.expiry_time_s, other.expiry_time_s) - 1e-9
        )

    def lateral_envelopes_overlap(self, other: "TrafficCorridorReservation") -> bool:
        return (
            max(self.lateral_min_m, other.lateral_min_m)
            < min(self.lateral_max_m, other.lateral_max_m) - 1e-9
        )

    def overlaps(self, other: "TrafficCorridorReservation", *, line_length_m: float) -> bool:
        return (
            self.time_windows_overlap(other)
            and self.arc_intervals_overlap(other, line_length_m=line_length_m)
            and self.lateral_envelopes_overlap(other)
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "corridor_id": self.corridor_id,
            "attacker_id": self.attacker_id,
            "defender_id": self.defender_id,
            "start_arc_distance_m": round(self.start_arc_distance_m, 9),
            "end_arc_distance_m": round(self.end_arc_distance_m, 9),
            "start_time_s": round(self.start_time_s, 9),
            "expiry_time_s": round(self.expiry_time_s, 9),
            "lateral_min_m": round(self.lateral_min_m, 9),
            "lateral_max_m": round(self.lateral_max_m, 9),
            "side_assignment": [
                {"driver_id": driver_id, "side": side}
                for driver_id, side in self.side_assignment
            ],
            "required_width_m": round(self.required_width_m, 9),
            "available_width_m": round(self.available_width_m, 9),
            "segment_type": self.segment_type,
            "corner_phase": self.corner_phase,
            "third_vehicle_ids": list(self.third_vehicle_ids),
            "admission_reason": self.admission_reason,
            "vehicle_length_m": round(self.vehicle_length_m, 9),
            "vehicle_width_m": round(self.vehicle_width_m, 9),
            "occupancy_model": self.occupancy_model,
        }


def audit_reservation_intersections(
    reservations: Sequence[TrafficCorridorReservation],
    *,
    line_length_m: float,
) -> dict[str, Any]:
    """Independently count admitted reservation envelope intersections."""

    ordered = tuple(sorted(reservations, key=lambda item: item.corridor_id))
    pairs: list[tuple[str, str]] = []
    for index, first in enumerate(ordered):
        for second in ordered[index + 1 :]:
            if first.overlaps(second, line_length_m=line_length_m):
                pairs.append((first.corridor_id, second.corridor_id))
    return {
        "admitted_reservation_conflict_count": len(pairs),
        "reservation_conflict_pairs": tuple(pairs),
    }


def audit_accepted_frame_reservations(
    frame: Any,
    reservations: Sequence[TrafficCorridorReservation],
    *,
    line_length_m: float,
    vehicle_length_m: float = 5.4,
    vehicle_width_m: float = VEHICLE_WIDTH_M,
    vehicle_target_speed_mps_by_id: dict[int | str, float] | None = None,
) -> dict[str, Any]:
    """Audit active reservations against one accepted frame/state.

    This function deliberately consumes only the accepted frame and the
    immutable reservations.  It does not read cursor counters, maneuver
    phase strings or a precomputed approval value.
    """

    now = float(frame.logical_time_s)
    active = tuple(
        item
        for item in reservations
        if item.start_time_s <= now + 1e-9 and item.expiry_time_s >= now - 1e-9
    )
    intersection = audit_reservation_intersections(active, line_length_m=line_length_m)
    vehicles = {vehicle.driver_id: vehicle for vehicle in frame.vehicles}
    third_pairs: set[tuple[str, int | str]] = set()
    body_violations = 0
    minimum_body_clearance = float("inf")
    minimum_corridor_clearance = float("inf")

    for reservation in active:
        horizon_s = max(0.0, reservation.expiry_time_s - reservation.start_time_s)
        attacker = vehicles.get(reservation.attacker_id)
        defender = vehicles.get(reservation.defender_id)
        time_aligned = reservation.occupancy_model == "time_aligned"
        # ``third_vehicle_ids`` is an admission-time explanation, not the
        # audit universe.  A vehicle can enter the corridor after admission,
        # so every non-participant in the accepted frame must be checked.
        for driver_id, vehicle in sorted(vehicles.items(), key=lambda item: str(item[0])):
            if driver_id in {reservation.attacker_id, reservation.defender_id}:
                continue
            if swept_vehicle_intersects_reservation(
                reservation_start_arc_m=reservation.start_arc_distance_m,
                reservation_end_arc_m=reservation.end_arc_distance_m,
                # Admission owns the full future forecast.  This independent
                # accepted-frame audit runs every tick and therefore checks
                # actual occupancy at this logical instant; forecasting the
                # remaining horizon again here would count a changing plan as
                # an already accepted collision.
                reservation_horizon_s=0.0 if time_aligned else horizon_s,
                reservation_elapsed_s=(
                    0.0 if time_aligned else max(0.0, now - reservation.start_time_s)
                ),
                vehicle_start_arc_m=vehicle.line_distance_m,
                vehicle_speed_mps=vehicle.speed_mps,
                # The accepted frame gives the current state authority.  A
                # cursor that owns the accepted plan may provide its
                # deterministic horizon target; standalone audits fall back
                # to constant current speed.  In both cases every
                # non-participant is inspected, not only the admission-time
                # ``third_vehicle_ids`` explanation list.
                vehicle_target_speed_mps=(
                    vehicle_target_speed_mps_by_id.get(driver_id, vehicle.speed_mps)
                    if vehicle_target_speed_mps_by_id is not None
                    else vehicle.speed_mps
                ),
                vehicle_lateral_offset_m=vehicle.lateral_offset_m,
                line_length_m=line_length_m,
                vehicle_length_m=vehicle_length_m,
                vehicle_width_m=vehicle_width_m,
                lateral_min_m=reservation.lateral_min_m,
                lateral_max_m=reservation.lateral_max_m,
                reservation_attacker_start_arc_m=(
                    attacker.line_distance_m if time_aligned and attacker is not None else None
                ),
                reservation_attacker_speed_mps=(
                    attacker.speed_mps if time_aligned and attacker is not None else None
                ),
                reservation_attacker_target_speed_mps=(
                    vehicle_target_speed_mps_by_id.get(
                        reservation.attacker_id,
                        attacker.speed_mps,
                    )
                    if time_aligned and vehicle_target_speed_mps_by_id is not None and attacker is not None
                    else (attacker.speed_mps if time_aligned and attacker is not None else None)
                ),
                reservation_defender_start_arc_m=(
                    defender.line_distance_m if time_aligned and defender is not None else None
                ),
                reservation_defender_speed_mps=(
                    defender.speed_mps if time_aligned and defender is not None else None
                ),
                reservation_defender_target_speed_mps=(
                    vehicle_target_speed_mps_by_id.get(
                        reservation.defender_id,
                        defender.speed_mps,
                    )
                    if time_aligned and vehicle_target_speed_mps_by_id is not None and defender is not None
                    else (defender.speed_mps if time_aligned and defender is not None else None)
                ),
            ):
                third_pairs.add((reservation.corridor_id, driver_id))

        if attacker is not None and defender is not None:
            longitudinal_gap = abs(attacker.line_distance_m - defender.line_distance_m)
            lateral_gap = abs(attacker.lateral_offset_m - defender.lateral_offset_m)
            if longitudinal_gap < vehicle_length_m:
                minimum_body_clearance = min(
                    minimum_body_clearance,
                    lateral_gap - vehicle_width_m,
                )
                if lateral_gap < vehicle_width_m + BODY_CLEARANCE_MARGIN_M:
                    body_violations += 1
            minimum_corridor_clearance = min(
                minimum_corridor_clearance,
                reservation.available_width_m - reservation.required_width_m,
            )

    same_lane_overlap_count = 0
    vehicles_by_position = sorted(vehicles.values(), key=lambda item: item.position)
    for index, first in enumerate(vehicles_by_position):
        for second in vehicles_by_position[index + 1 :]:
            if abs(first.lateral_offset_m - second.lateral_offset_m) >= vehicle_width_m + BODY_CLEARANCE_MARGIN_M:
                continue
            if abs(first.line_distance_m - second.line_distance_m) < vehicle_length_m:
                same_lane_overlap_count += 1

    if minimum_body_clearance == float("inf"):
        minimum_body_clearance = 0.0
    if minimum_corridor_clearance == float("inf"):
        minimum_corridor_clearance = 0.0
    return {
        "active_reservation_count": len(active),
        "admitted_reservation_conflict_count": intersection["admitted_reservation_conflict_count"],
        "reservation_conflict_pairs": intersection["reservation_conflict_pairs"],
        "third_vehicle_occupancy_conflict_count": len(third_pairs),
        "third_vehicle_conflict_pairs": tuple(sorted(third_pairs, key=lambda item: (item[0], str(item[1])))),
        "reservation_body_clearance_violation_count": body_violations,
        "same_lane_longitudinal_overlap_count": same_lane_overlap_count,
        "minimum_body_clearance_m": round(minimum_body_clearance, 9),
        "minimum_corridor_clearance_m": round(minimum_corridor_clearance, 9),
    }


def _relevant_vehicle_score(performance: VehiclePerformance, segment_type: str) -> float:
    if segment_type == "straight":
        return performance.power * 0.58 + performance.drag_efficiency * 0.42
    if segment_type == "heavy_braking":
        return performance.braking * 0.70 + performance.high_speed_aero * 0.30
    if segment_type == "traction":
        return performance.traction * 0.60 + performance.low_speed_grip * 0.40
    if segment_type == "sweeping":
        return performance.high_speed_aero * 0.75 + performance.medium_speed_aero * 0.25
    return performance.medium_speed_aero * 0.60 + performance.low_speed_grip * 0.40


@dataclass(frozen=True, slots=True)
class OvertakeAssessment:
    allowed: bool
    success: bool
    reason_code: str
    segment_type: str
    corridor_width_m: float
    required_width_m: float
    gap_m: float
    closing_speed_mps: float
    attacker_score: float
    defender_score: float
    probability: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "allowed": self.allowed,
            "success": self.success,
            "reason_code": self.reason_code,
            "segment_type": self.segment_type,
            "corridor_width_m": round(self.corridor_width_m, 9),
            "required_width_m": round(self.required_width_m, 9),
            "gap_m": round(self.gap_m, 9),
            "closing_speed_mps": round(self.closing_speed_mps, 9),
            "attacker_score": round(self.attacker_score, 9),
            "defender_score": round(self.defender_score, 9),
            "probability": round(self.probability, 9),
        }


@dataclass(frozen=True, slots=True)
class LocalCorridorAssessment:
    """Deterministic local two-wide corridor admission result.

    Width is measured at the current compiled-track sample.  The assessment
    deliberately carries the blocking causes so a rejected attack is
    explainable without treating a global circuit width as a reservation.
    """

    allowed: bool
    reason_code: str
    left_available_m: float
    right_available_m: float
    corridor_width_m: float
    required_width_m: float
    third_vehicle_conflict: bool
    reservation_conflict: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "allowed": self.allowed,
            "reason_code": self.reason_code,
            "left_available_m": round(self.left_available_m, 9),
            "right_available_m": round(self.right_available_m, 9),
            "corridor_width_m": round(self.corridor_width_m, 9),
            "required_width_m": round(self.required_width_m, 9),
            "third_vehicle_conflict": self.third_vehicle_conflict,
            "reservation_conflict": self.reservation_conflict,
        }


def local_corridor_assessment(
    *,
    local_left_width_m: float,
    local_right_width_m: float,
    racing_line_offset_m: float,
    vehicle_width_m: float = VEHICLE_WIDTH_M,
    vehicle_length_m: float = 5.4,
    segment_type: str,
    corner_phase: str,
    remaining_distance_m: float,
    gap_m: float,
    closing_speed_mps: float,
    third_vehicle_conflict: bool,
    reservation_conflict: bool,
) -> LocalCorridorAssessment:
    """Check a proposed two-wide corridor using local geometry and traffic.

    ``vehicle_length_m`` and the phase/distance inputs are intentionally part
    of the public contract even though this first provisional game model uses
    them as conservative admission guards rather than physical forces.
    """

    del vehicle_length_m  # retained in the contract for future route detail
    left = max(0.0, float(local_left_width_m) - max(0.0, float(racing_line_offset_m)))
    right = max(0.0, float(local_right_width_m) + min(0.0, float(racing_line_offset_m)))
    corridor_width = left + right
    required = corridor_required_width_m()
    if segment_type not in _ALLOWED_ATTACK_SEGMENTS:
        reason = "segment_not_attackable"
    elif corner_phase in {"apex", "exit"} and remaining_distance_m < 8.0:
        reason = "corner_phase_unsafe"
    elif gap_m > ATTACK_WINDOW_M:
        reason = "gap_too_large"
    elif closing_speed_mps < MIN_CLOSING_SPEED_MPS:
        reason = "no_closing_speed"
    elif corridor_width < required:
        reason = "corridor_too_narrow"
    elif third_vehicle_conflict or reservation_conflict:
        reason = "corridor_conflict"
    else:
        reason = "corridor_available"
    return LocalCorridorAssessment(
        allowed=reason == "corridor_available",
        reason_code=reason,
        left_available_m=left,
        right_available_m=right,
        corridor_width_m=corridor_width,
        required_width_m=required,
        third_vehicle_conflict=bool(third_vehicle_conflict),
        reservation_conflict=bool(reservation_conflict),
    )


def assess_overtake(
    *,
    attacker_driver: AbstractDriverProfile,
    attacker_vehicle: VehiclePerformance,
    defender_driver: AbstractDriverProfile,
    defender_vehicle: VehiclePerformance,
    segment_type: str,
    gap_m: float,
    closing_speed_mps: float,
    track_width_m: float,
    random_value: float,
    overtaking_difficulty: float = 0.5,
) -> OvertakeAssessment:
    """Evaluate a pass only after gap, closing and two-wide space exist."""

    required_width = corridor_required_width_m()
    attacker_score = _relevant_vehicle_score(attacker_vehicle, segment_type) * 0.58 + attacker_driver.overtaking * 0.42
    defender_score = _relevant_vehicle_score(defender_vehicle, segment_type) * 0.42 + defender_driver.defending * 0.58
    common = dict(
        segment_type=segment_type,
        corridor_width_m=track_width_m,
        required_width_m=required_width,
        gap_m=max(0.0, gap_m),
        closing_speed_mps=max(0.0, closing_speed_mps),
        attacker_score=attacker_score,
        defender_score=defender_score,
    )
    if segment_type not in _ALLOWED_ATTACK_SEGMENTS:
        return OvertakeAssessment(False, False, "segment_not_attackable", probability=0.0, **common)
    if track_width_m < required_width:
        return OvertakeAssessment(False, False, "corridor_too_narrow", probability=0.0, **common)
    if gap_m > ATTACK_WINDOW_M:
        return OvertakeAssessment(False, False, "gap_too_large", probability=0.0, **common)
    if closing_speed_mps < MIN_CLOSING_SPEED_MPS:
        return OvertakeAssessment(False, False, "no_closing_speed", probability=0.0, **common)

    pace_edge = attacker_score - defender_score
    closing_bonus = min(0.16, closing_speed_mps / 80.0)
    probability = max(
        0.08,
        min(
            0.92,
            0.46
            + pace_edge * 0.80
            + closing_bonus
            - overtaking_difficulty * 0.18,
        ),
    )
    success = random_value < probability
    return OvertakeAssessment(
        True,
        success,
        "pass_success" if success else "defense_hold",
        probability=probability,
        **common,
    )
