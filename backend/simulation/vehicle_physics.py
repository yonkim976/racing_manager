"""Fixed-step longitudinal vehicle physics for the optional Physics V2 engine."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil

from simulation.speed_profile import SpeedProfile

PHYSICS_STEP_SECONDS = 0.02  # 50 Hz
PHYSICS_MIN_SPEED_MPS = 18.0
PHYSICS_MAX_SPEED_MPS = 105.0
PHYSICS_ACCELERATION_MPS2 = 12.0
PHYSICS_BRAKING_MPS2 = 34.0
PHYSICS_LATERAL_ACCELERATION_MPS2 = 4.5
PHYSICS_MAX_LATERAL_SPEED_MPS = 4.0


@dataclass(frozen=True)
class VehiclePhysicsModifiers:
    """Dimensionless modifiers supplied by car, driver, tires and race control."""

    power: float = 1.0
    grip: float = 1.0
    pace: float = 1.0
    speed_limit_factor: float = 1.0


@dataclass(frozen=True)
class VehicleFollowingConstraint:
    """Predicted motion of a same-line car that must not be intersected."""

    leader_distance_m: float
    leader_speed_mps: float
    leader_end_distance_m: float
    leader_end_speed_mps: float
    desired_gap_m: float
    minimum_gap_m: float


@dataclass(frozen=True)
class VehiclePhysicsResult:
    distance_m: float
    speed_mps: float
    acceleration_mps2: float
    target_speed_mps: float
    throttle: float
    brake: float
    lateral_offset_m: float
    lateral_speed_mps: float
    lateral_acceleration_mps2: float


class LongitudinalVehiclePhysics:
    """Advance a point-mass race car along a closed one-dimensional track."""

    def __init__(
        self,
        profile: SpeedProfile | None,
        track_length_m: float,
        reference_lap_time: float,
    ) -> None:
        self.profile = profile
        self.track_length_m = max(1.0, float(track_length_m))
        self.reference_lap_time = max(1.0, float(reference_lap_time))
        raw_lap_time = self._estimate_raw_lap_time()
        self.reference_speed_scale = raw_lap_time / self.reference_lap_time

    def _estimate_raw_lap_time(self) -> float:
        if self.profile is None or not self.profile.progress or not self.profile.raw_speeds_mps:
            return self.reference_lap_time

        lap_time = 0.0
        count = len(self.profile.progress)
        for index, start in enumerate(self.profile.progress):
            end = self.profile.progress[(index + 1) % count]
            if index == count - 1:
                end += 1.0
            distance_m = max(0.0, end - start) * self.track_length_m
            speed_mps = max(PHYSICS_MIN_SPEED_MPS, self.profile.raw_speeds_mps[index])
            lap_time += distance_m / speed_mps
        return max(1.0, lap_time)

    def target_speed_mps(
        self,
        distance_m: float,
        modifiers: VehiclePhysicsModifiers,
    ) -> float:
        progress = (distance_m / self.track_length_m) % 1.0
        if self.profile is None:
            raw_limit = self.track_length_m / self.reference_lap_time
        else:
            raw_limit = self.profile.raw_speed_at_progress(progress)

        performance = (
            max(0.8, min(1.2, modifiers.power))
            * max(0.8, min(1.2, modifiers.grip))
            * max(0.8, min(1.2, modifiers.pace))
            * max(0.2, min(1.0, modifiers.speed_limit_factor))
        )
        target = raw_limit * self.reference_speed_scale * performance
        return max(PHYSICS_MIN_SPEED_MPS, min(PHYSICS_MAX_SPEED_MPS, target))

    def advance(
        self,
        *,
        distance_m: float,
        speed_mps: float,
        delta_seconds: float,
        modifiers: VehiclePhysicsModifiers,
        following: VehicleFollowingConstraint | None = None,
        lateral_offset_m: float = 0.0,
        lateral_speed_mps: float = 0.0,
        target_lateral_offset_m: float = 0.0,
        minimum_lateral_offset_m: float = -5.0,
        maximum_lateral_offset_m: float = 5.0,
    ) -> VehiclePhysicsResult:
        """Advance using deterministic 50 Hz substeps, including partial final steps."""
        remaining = max(0.0, float(delta_seconds))
        distance = float(distance_m)
        speed = max(0.0, float(speed_mps))
        acceleration = 0.0
        target = self.target_speed_mps(distance, modifiers)
        throttle = 0.0
        brake = 0.0
        elapsed = 0.0
        lateral_offset = float(lateral_offset_m)
        lateral_speed = float(lateral_speed_mps)
        lateral_acceleration = 0.0

        substeps = max(1, ceil(remaining / PHYSICS_STEP_SECONDS)) if remaining > 0 else 0
        for _ in range(substeps):
            step = min(PHYSICS_STEP_SECONDS, remaining)
            if step <= 0:
                break
            target = self.target_speed_mps(distance, modifiers)
            leader_distance = None
            if following is not None:
                elapsed_ratio = min(1.0, elapsed / max(delta_seconds, 1e-9))
                leader_distance = following.leader_distance_m + (
                    following.leader_end_distance_m - following.leader_distance_m
                ) * elapsed_ratio
                leader_speed = following.leader_speed_mps + (
                    following.leader_end_speed_mps - following.leader_speed_mps
                ) * elapsed_ratio
                gap = leader_distance - distance
                if gap < following.desired_gap_m:
                    following_target = (
                        leader_speed
                        + 0.85 * (gap - following.desired_gap_m)
                    )
                    target = min(target, max(0.0, following_target))
                if gap <= following.minimum_gap_m:
                    target = min(target, max(0.0, leader_speed - 2.0))

            difference = target - speed
            if difference >= 0:
                throttle = min(1.0, difference / max(1.0, target * 0.12))
                brake = 0.0
                high_speed_falloff = max(0.25, 1.0 - speed / (PHYSICS_MAX_SPEED_MPS * 1.2))
                acceleration = PHYSICS_ACCELERATION_MPS2 * modifiers.power * high_speed_falloff * throttle
            else:
                throttle = 0.0
                brake = min(1.0, -difference / max(1.0, speed * 0.10))
                acceleration = -PHYSICS_BRAKING_MPS2 * modifiers.grip * brake

            next_speed = max(0.0, speed + acceleration * step)
            next_distance = distance + 0.5 * (speed + next_speed) * step
            if following is not None and leader_distance is not None:
                next_elapsed_ratio = min(
                    1.0,
                    (elapsed + step) / max(delta_seconds, 1e-9),
                )
                leader_next_distance = following.leader_distance_m + (
                    following.leader_end_distance_m - following.leader_distance_m
                ) * next_elapsed_ratio
                maximum_distance = leader_next_distance - following.minimum_gap_m
                if next_distance > maximum_distance:
                    next_distance = maximum_distance
                    next_speed = min(next_speed, following.leader_end_speed_mps)
                    acceleration = min(0.0, (next_speed - speed) / step)
                    throttle = 0.0
                    brake = 1.0
            distance = next_distance
            speed = next_speed

            lateral_error = target_lateral_offset_m - lateral_offset
            lateral_acceleration = max(
                -PHYSICS_LATERAL_ACCELERATION_MPS2,
                min(
                    PHYSICS_LATERAL_ACCELERATION_MPS2,
                    lateral_error * 2.8 - lateral_speed * 2.4,
                ),
            )
            next_lateral_speed = max(
                -PHYSICS_MAX_LATERAL_SPEED_MPS,
                min(
                    PHYSICS_MAX_LATERAL_SPEED_MPS,
                    lateral_speed + lateral_acceleration * step,
                ),
            )
            next_lateral_offset = lateral_offset + 0.5 * (
                lateral_speed + next_lateral_speed
            ) * step
            if next_lateral_offset < minimum_lateral_offset_m:
                next_lateral_offset = minimum_lateral_offset_m
                next_lateral_speed = max(0.0, next_lateral_speed)
            elif next_lateral_offset > maximum_lateral_offset_m:
                next_lateral_offset = maximum_lateral_offset_m
                next_lateral_speed = min(0.0, next_lateral_speed)
            lateral_offset = next_lateral_offset
            lateral_speed = next_lateral_speed
            remaining -= step
            elapsed += step

        return VehiclePhysicsResult(
            distance_m=distance,
            speed_mps=speed,
            acceleration_mps2=acceleration,
            target_speed_mps=target,
            throttle=throttle,
            brake=brake,
            lateral_offset_m=lateral_offset,
            lateral_speed_mps=lateral_speed,
            lateral_acceleration_mps2=lateral_acceleration,
        )
