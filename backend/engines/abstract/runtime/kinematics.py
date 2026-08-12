"""Single-vehicle presentation kinematics for ABSTRACT Stage 3.

This module is deliberately separate from the logical result engine.  It owns
only a bounded, deterministic presentation plan: compiled racing-line distance,
speed/acceleration-limited longitudinal motion, and one safe lateral trajectory.
It does not create forces, tire state, throttle, brake pressure, or result data.
"""

from __future__ import annotations

# Implementation authority: engines.abstract.runtime

import hashlib
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass, field
from math import floor, isfinite, sqrt
from typing import Any, Iterable

from simulation.track_contracts import DRIVING_LINE_RACING, TrackGeometryProfile

from .performance import calculate_segment_time
from .state import canonical_json


KINEMATIC_LOGICAL_TICK_SECONDS = 0.10
# A standing start uses a deliberately separate envelope from normal racing.
# Grid slots are closer longitudinally because the cars are staggered; applying
# the normal single-file following gap at lights-out serializes the launch.
GRID_HOLD_DURATION_S = 1.0
GRID_LATERAL_HOLD_AFTER_LIGHTS_S = 1.5
GRID_LATERAL_REJOIN_DURATION_S = 2.5
GRID_LAUNCH_FOLLOWING_BLEND_DURATION_S = 8.0
GRID_LAUNCH_MIN_LONGITUDINAL_GAP_M = 1.0
MAX_DISPLAY_SPEED_KPH = 370.0
MAX_DISPLAY_SPEED_MPS = MAX_DISPLAY_SPEED_KPH / 3.6
MAX_LONGITUDINAL_ACCELERATION_MPS2 = 18.0
MAX_BRAKING_DECELERATION_MPS2 = 50.0
MAX_LATERAL_SPEED_MPS = 8.0
MAX_LATERAL_ACCELERATION_MPS2 = 20.0
DEFAULT_EDGE_MARGIN_M = 0.15
QUINTIC_MAX_SLOPE = 1.875
QUINTIC_MAX_CURVATURE = 5.773502691896258
KINEMATICS_PLAN_VERSION = "abstract-stage3-kinematics-v1"
ARC_LENGTH_LUT_SAMPLE_COUNT = 16_384


@dataclass(frozen=True, slots=True)
class CompiledSplineArcLengthAdapter:
    """Periodic arc-length parameterization of the shared world pose spline.

    The adapter samples the exact public ``TrackGeometryProfile`` spline used by
    pose rendering.  Its distance axis is therefore the distance of that same
    curve, rather than the profile's older path-distance approximation.
    """

    profile: TrackGeometryProfile = field(repr=False, compare=False)
    line_name: str
    progress_samples: tuple[float, ...]
    arc_distances_m: tuple[float, ...]
    line_length_m: float
    sample_count: int

    @classmethod
    def from_profile(
        cls,
        profile: TrackGeometryProfile,
        line_name: str = DRIVING_LINE_RACING,
        *,
        sample_count: int = ARC_LENGTH_LUT_SAMPLE_COUNT,
    ) -> "CompiledSplineArcLengthAdapter":
        if sample_count < 256:
            raise ValueError("arc-length LUT requires at least 256 samples")
        progress_samples = tuple(index / sample_count for index in range(sample_count + 1))
        points = tuple(
            profile.line_pose_at_progress_m(line_name, progress)[0:2]
            for progress in progress_samples
        )
        cumulative = [0.0]
        for first, second in zip(points, points[1:]):
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            cumulative.append(cumulative[-1] + sqrt(dx * dx + dy * dy))
        line_length_m = cumulative[-1]
        if line_length_m <= 0.0:
            raise ValueError("compiled pose spline must have positive arc length")
        return cls(
            profile=profile,
            line_name=line_name,
            progress_samples=progress_samples,
            arc_distances_m=tuple(cumulative),
            line_length_m=line_length_m,
            sample_count=sample_count,
        )

    def arc_distance_at_total_progress(self, total_progress: float) -> float:
        lap = floor(float(total_progress))
        fraction = float(total_progress) - lap
        position = min(self.sample_count - 1e-12, max(0.0, fraction * self.sample_count))
        index = min(self.sample_count - 1, int(position))
        ratio = position - index
        local_distance = self.arc_distances_m[index] + (
            self.arc_distances_m[index + 1] - self.arc_distances_m[index]
        ) * ratio
        return lap * self.line_length_m + local_distance

    def total_progress_at_arc_distance(self, arc_distance_m: float) -> float:
        lap = floor(float(arc_distance_m) / self.line_length_m)
        local_distance = float(arc_distance_m) - lap * self.line_length_m
        if local_distance < 0.0:
            local_distance = 0.0
        if local_distance >= self.line_length_m:
            return float(lap + 1)
        index = bisect_right(self.arc_distances_m, local_distance) - 1
        index = min(self.sample_count - 1, max(0, index))
        span = self.arc_distances_m[index + 1] - self.arc_distances_m[index]
        ratio = (local_distance - self.arc_distances_m[index]) / max(span, 1e-12)
        return lap + (index + ratio) / self.sample_count

    def pose_at_arc_distance(self, arc_distance_m: float) -> tuple[float, float, float]:
        progress = self.total_progress_at_arc_distance(arc_distance_m)
        return self.profile.line_pose_at_progress_m(self.line_name, progress)


@dataclass(frozen=True, slots=True)
class RacingLineDistanceContract:
    """The one public distance authority for a compiled racing line."""

    profile: TrackGeometryProfile
    line_name: str = DRIVING_LINE_RACING
    arc_length_adapter: CompiledSplineArcLengthAdapter | None = field(
        default=None,
        repr=False,
        compare=False,
    )

    def __post_init__(self) -> None:
        if self.arc_length_adapter is None:
            object.__setattr__(
                self,
                "arc_length_adapter",
                CompiledSplineArcLengthAdapter.from_profile(
                    self.profile,
                    self.line_name,
                ),
            )

    @property
    def line_length_m(self) -> float:
        length = self.arc_length_adapter.line_length_m
        if length <= 0.0:
            raise ValueError("compiled racing line must have positive length")
        return length

    def distance_at_total_progress(self, total_progress: float) -> float:
        return self.arc_length_adapter.arc_distance_at_total_progress(total_progress)

    def total_progress_at_distance(self, distance_m: float) -> float:
        return self.arc_length_adapter.total_progress_at_arc_distance(distance_m)

    def pose_at_distance(self, distance_m: float) -> tuple[float, float, float]:
        return self.arc_length_adapter.pose_at_arc_distance(distance_m)

    def lateral_limits_at_progress(
        self,
        progress: float,
        *,
        car_width_m: float,
        edge_margin_m: float = DEFAULT_EDGE_MARGIN_M,
    ) -> tuple[float, float]:
        """Return safe ``(min_offset, max_offset)`` around the racing line.

        Positive lateral offset is the profile's left normal.  The limits
        include the vehicle half-width and a display safety margin.
        """

        sample = self.profile.at_progress(progress)
        half_width = max(0.0, float(car_width_m)) * 0.5
        margin = max(0.0, float(edge_margin_m))
        min_offset = -sample.right_width_m + half_width + margin - sample.racing_line_offset_m
        max_offset = sample.left_width_m - half_width - margin - sample.racing_line_offset_m
        if min_offset > max_offset:
            midpoint = (min_offset + max_offset) * 0.5
            return midpoint, midpoint
        return min_offset, max_offset


@dataclass(frozen=True, slots=True)
class SpeedKnot:
    """A periodic target-speed knot on the compiled line-distance axis."""

    distance_m: float
    target_speed_mps: float
    segment_id: str
    transition: str = "segment"

    def to_dict(self) -> dict[str, Any]:
        return {
            "distance_m": round(self.distance_m, 9),
            "target_speed_mps": round(self.target_speed_mps, 9),
            "segment_id": self.segment_id,
            "transition": self.transition,
        }


@dataclass(frozen=True, slots=True)
class KinematicStep:
    distance_m: float
    speed_mps: float
    longitudinal_acceleration_mps2: float
    displacement_m: float
    target_speed_mps: float
    previous_distance_m: float = 0.0
    previous_speed_mps: float = 0.0
    integrator_adjusted: bool = False
    adjustment_reason: str | None = None


@dataclass(frozen=True, slots=True)
class LongitudinalKinematicPlan:
    """Periodic smooth target speeds plus bounded forward integration."""

    distance_contract: RacingLineDistanceContract
    knots: tuple[SpeedKnot, ...]
    tick_seconds: float = KINEMATIC_LOGICAL_TICK_SECONDS
    plan_version: str = KINEMATICS_PLAN_VERSION
    _knot_distances: tuple[float, ...] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        if abs(self.tick_seconds - KINEMATIC_LOGICAL_TICK_SECONDS) > 1e-9:
            raise ValueError("Stage 3 kinematics requires a 0.10s logical tick")
        if not self.knots:
            raise ValueError("kinematic plan requires at least one speed knot")
        object.__setattr__(
            self,
            "_knot_distances",
            tuple(knot.distance_m for knot in self.knots),
        )
        previous = -1.0
        for knot in self.knots:
            if knot.distance_m < previous:
                raise ValueError("speed knots must be sorted by distance")
            if not 0.0 <= knot.target_speed_mps <= MAX_DISPLAY_SPEED_MPS + 1e-9:
                raise ValueError("speed knot exceeds presentation speed guard")
            previous = knot.distance_m

    @property
    def line_length_m(self) -> float:
        return self.distance_contract.line_length_m

    @property
    def maximum_target_speed_mps(self) -> float:
        return max(knot.target_speed_mps for knot in self.knots)

    def target_speed_at_line_distance(self, distance_m: float) -> float:
        """Smoothly interpolate the periodic knots with quintic easing."""

        length = self.line_length_m
        normalized = float(distance_m) % length
        distances = self._knot_distances
        index = bisect_right(distances, normalized) - 1
        if index < 0:
            index = len(self.knots) - 1
        next_index = (index + 1) % len(self.knots)
        start = self.knots[index]
        end = self.knots[next_index]
        start_distance = start.distance_m
        end_distance = end.distance_m if next_index != 0 else length + end.distance_m
        target_distance = normalized
        if target_distance < start_distance:
            target_distance += length
        span = max(1e-9, end_distance - start_distance)
        ratio = min(1.0, max(0.0, (target_distance - start_distance) / span))
        eased = ratio * ratio * ratio * (
            ratio * (ratio * 6.0 - 15.0) + 10.0
        )
        return start.target_speed_mps + (
            end.target_speed_mps - start.target_speed_mps
        ) * eased

    def target_speed_at_progress(self, total_progress: float) -> float:
        return self.target_speed_at_line_distance(
            self.distance_contract.distance_at_total_progress(total_progress),
        )

    def integrate(
        self,
        distance_m: float,
        speed_mps: float,
        *,
        tick_seconds: float | None = None,
    ) -> KinematicStep:
        dt = self.tick_seconds if tick_seconds is None else float(tick_seconds)
        if abs(dt - self.tick_seconds) > 1e-9:
            raise ValueError("kinematic integration must use the plan tick")
        current_speed = min(MAX_DISPLAY_SPEED_MPS, max(0.0, float(speed_mps)))
        previous_distance = max(0.0, float(distance_m))
        target = min(MAX_DISPLAY_SPEED_MPS, max(0.0, self.target_speed_at_line_distance(distance_m)))
        requested_acceleration = (target - current_speed) / dt
        acceleration = min(
            MAX_LONGITUDINAL_ACCELERATION_MPS2,
            max(-MAX_BRAKING_DECELERATION_MPS2, requested_acceleration),
        )
        next_speed = min(
            MAX_DISPLAY_SPEED_MPS,
            max(0.0, current_speed + acceleration * dt),
        )
        displacement = current_speed * dt + 0.5 * acceleration * dt * dt
        integrator_adjusted = False
        adjustment_reason = None
        maximum_displacement = current_speed * dt + 0.5
        if displacement > maximum_displacement:
            displacement = maximum_displacement
            next_speed = max(0.0, (2.0 * displacement / dt) - current_speed)
            acceleration = (next_speed - current_speed) / dt
            integrator_adjusted = True
            adjustment_reason = "tick_displacement_guard"
        if displacement < 0.0:
            displacement = 0.0
            next_speed = 0.0
            acceleration = -current_speed / dt
            integrator_adjusted = True
            adjustment_reason = "non_negative_distance_guard"
        return KinematicStep(
            distance_m=previous_distance + displacement,
            speed_mps=next_speed,
            longitudinal_acceleration_mps2=acceleration,
            displacement_m=displacement,
            target_speed_mps=target,
            previous_distance_m=previous_distance,
            previous_speed_mps=current_speed,
            integrator_adjusted=integrator_adjusted,
            adjustment_reason=adjustment_reason,
        )


@dataclass(frozen=True, slots=True)
class LateralTrajectory:
    """Quintic, boundary-continuous lateral offset trajectory."""

    start_offset_m: float
    end_offset_m: float
    start_time_s: float
    duration_s: float
    requested_duration_s: float
    duration_extended: bool = False
    clamp_reason: str | None = None

    @classmethod
    def create(
        cls,
        *,
        start_offset_m: float,
        end_offset_m: float,
        start_time_s: float,
        duration_s: float,
        target_limits_m: tuple[float, float] | None = None,
    ) -> "LateralTrajectory":
        start = float(start_offset_m)
        requested_end = float(end_offset_m)
        if not all(isfinite(value) for value in (start, requested_end, start_time_s, duration_s)):
            raise ValueError("lateral trajectory values must be finite")
        if start_time_s < 0.0 or duration_s < 0.0:
            raise ValueError("lateral trajectory time must be non-negative")
        end = requested_end
        reason = None
        if target_limits_m is not None:
            minimum, maximum = target_limits_m
            if minimum > maximum:
                raise ValueError("lateral target limits are inverted")
            clamped = min(maximum, max(minimum, end))
            if abs(clamped - end) > 1e-9:
                reason = "target_clamped_to_track_envelope"
                end = clamped
        displacement = abs(end - start)
        if displacement <= 1e-12:
            minimum_duration = 0.0
        else:
            minimum_duration = max(
                QUINTIC_MAX_SLOPE * displacement / MAX_LATERAL_SPEED_MPS,
                sqrt(
                    QUINTIC_MAX_CURVATURE
                    * displacement
                    / MAX_LATERAL_ACCELERATION_MPS2,
                ),
            )
        actual_duration = max(float(duration_s), minimum_duration)
        extended = actual_duration > float(duration_s) + 1e-9
        if extended:
            reason = reason or "duration_extended_to_lateral_guard"
        return cls(
            start_offset_m=start,
            end_offset_m=end,
            start_time_s=float(start_time_s),
            duration_s=actual_duration,
            requested_duration_s=float(duration_s),
            duration_extended=extended,
            clamp_reason=reason,
        )

    @property
    def max_lateral_speed_mps(self) -> float:
        return QUINTIC_MAX_SLOPE * abs(self.end_offset_m - self.start_offset_m) / max(self.duration_s, 1e-9)

    @property
    def max_lateral_acceleration_mps2(self) -> float:
        return QUINTIC_MAX_CURVATURE * abs(self.end_offset_m - self.start_offset_m) / max(self.duration_s, 1e-9) ** 2

    def _normalized_time(self, logical_time_s: float) -> float:
        if logical_time_s <= self.start_time_s or self.duration_s <= 0.0:
            return 0.0
        if logical_time_s >= self.start_time_s + self.duration_s:
            return 1.0
        return (logical_time_s - self.start_time_s) / self.duration_s

    def offset_at(self, logical_time_s: float) -> float:
        ratio = self._normalized_time(logical_time_s)
        eased = ratio * ratio * ratio * (ratio * (ratio * 6.0 - 15.0) + 10.0)
        return self.start_offset_m + (self.end_offset_m - self.start_offset_m) * eased

    def velocity_at(self, logical_time_s: float) -> float:
        ratio = self._normalized_time(logical_time_s)
        derivative = 30.0 * ratio * ratio * (ratio - 1.0) * (ratio - 1.0)
        return (self.end_offset_m - self.start_offset_m) * derivative / max(self.duration_s, 1e-9)

    def acceleration_at(self, logical_time_s: float) -> float:
        ratio = self._normalized_time(logical_time_s)
        derivative = 60.0 * ratio * (1.0 - ratio) * (1.0 - 2.0 * ratio)
        return (self.end_offset_m - self.start_offset_m) * derivative / max(self.duration_s, 1e-9) ** 2


def _safe_speed(value: float) -> float:
    return min(MAX_DISPLAY_SPEED_MPS, max(0.0, float(value)))


def build_longitudinal_plan(
    snapshot: Any,
    driver_id: int | str | None = None,
    *,
    tick_seconds: float = KINEMATIC_LOGICAL_TICK_SECONDS,
    distance_contract: RacingLineDistanceContract | None = None,
) -> LongitudinalKinematicPlan:
    """Build a periodic target-speed plan from provisional segment timings."""

    track = snapshot.track if hasattr(snapshot, "track") else snapshot
    if track.display_geometry is None:
        raise ValueError("kinematic plan requires shared display geometry")
    profile = track.display_geometry.compiled_profile
    contract = distance_contract or RacingLineDistanceContract(profile)
    entry = None
    entries = getattr(snapshot, "entries", ())
    if driver_id is not None:
        entry = next((item for item in entries if item.driver_id == driver_id), None)
    if entry is None and entries:
        entry = entries[0]

    segments = tuple(sorted(track.segments, key=lambda item: (item.start_progress, item.segment_id)))
    segment_data: list[tuple[Any, float, float, float]] = []
    for segment in segments:
        start_distance = contract.distance_at_total_progress(segment.start_progress)
        end_distance = contract.distance_at_total_progress(segment.end_progress)
        if end_distance <= start_distance:
            end_distance += contract.line_length_m
        segment_length = max(0.01, end_distance - start_distance)
        segment_time = segment.base_time_s
        if entry is not None:
            segment_time = calculate_segment_time(
                segment,
                entry.vehicle.performance,
                entry.driver,
                entry.tire,
            ).time_s
        average_speed = _safe_speed(segment_length / max(0.001, segment_time))
        segment_data.append((segment, start_distance, end_distance, average_speed))

    if not segment_data:
        average_speed = _safe_speed(contract.line_length_m / max(0.1, track.base_lap_time_s))
        return LongitudinalKinematicPlan(
            contract,
            (SpeedKnot(0.0, average_speed, "fallback", "fallback"),),
            tick_seconds=tick_seconds,
        )

    knots_by_distance: dict[float, SpeedKnot] = {}
    segment_count = len(segment_data)
    for index, (segment, start_distance, end_distance, average_speed) in enumerate(segment_data):
        previous_speed = segment_data[(index - 1) % segment_count][3]
        start = start_distance % contract.line_length_m
        end = end_distance % contract.line_length_m
        knots_by_distance[round(start, 9)] = SpeedKnot(
            start,
            previous_speed,
            segment.segment_id,
            "segment_start",
        )
        knots_by_distance[round(end, 9)] = SpeedKnot(
            end,
            average_speed,
            segment.segment_id,
            "segment_end",
        )
        if previous_speed > average_speed:
            braking_distance = (
                previous_speed * previous_speed - average_speed * average_speed
            ) / (2.0 * MAX_BRAKING_DECELERATION_MPS2)
            anticipation = (start_distance - braking_distance) % contract.line_length_m
            knots_by_distance[round(anticipation, 9)] = SpeedKnot(
                anticipation,
                previous_speed,
                segment.segment_id,
                "braking_anticipation",
            )

    knots = tuple(
        sorted(knots_by_distance.values(), key=lambda knot: knot.distance_m)
    )
    return LongitudinalKinematicPlan(contract, knots, tick_seconds=tick_seconds)


def pose_sequence_hash(frames: Iterable[Any]) -> str:
    """Hash only presentation frames; never feed this into result hash."""

    payload = [frame.to_dict() if hasattr(frame, "to_dict") else frame for frame in frames]
    return hashlib.sha256(canonical_json(payload).encode("utf-8")).hexdigest()


class BoundedKinematicPoseBuffer:
    """Bounded single-probe presentation storage."""

    def __init__(self, max_frames: int = 128) -> None:
        if max_frames < 1:
            raise ValueError("max_frames must be positive")
        self.max_frames = max_frames
        self._frames: deque[Any] = deque(maxlen=max_frames)

    def append(self, frame: Any) -> None:
        self._frames.append(frame)

    def snapshot(self) -> tuple[Any, ...]:
        return tuple(self._frames)

    @property
    def latest(self) -> Any | None:
        return self._frames[-1] if self._frames else None

    def clear(self) -> None:
        self._frames.clear()

    def __len__(self) -> int:
        return len(self._frames)


KinematicPoseBuffer = BoundedKinematicPoseBuffer
