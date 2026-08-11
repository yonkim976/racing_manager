"""FULL deterministic oriented-car and swept collision detection."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin


@dataclass(frozen=True)
class BodyPose:
    longitudinal_m: float
    lateral_m: float
    heading_rad: float
    length_m: float
    width_m: float
    longitudinal_speed_mps: float = 0.0
    lateral_speed_mps: float = 0.0


@dataclass(frozen=True)
class BodyMotion:
    start: BodyPose
    end: BodyPose


@dataclass(frozen=True)
class CollisionContact:
    time_fraction: float
    normal_longitudinal: float
    normal_lateral: float
    penetration_m: float
    impact_speed_mps: float
    contact_type: str


def interpolate_pose(motion: BodyMotion, time_fraction: float) -> BodyPose:
    ratio = min(1.0, max(0.0, time_fraction))
    interpolate = lambda start, end: start + (end - start) * ratio
    return BodyPose(
        longitudinal_m=interpolate(
            motion.start.longitudinal_m,
            motion.end.longitudinal_m,
        ),
        lateral_m=interpolate(motion.start.lateral_m, motion.end.lateral_m),
        heading_rad=interpolate(motion.start.heading_rad, motion.end.heading_rad),
        length_m=interpolate(motion.start.length_m, motion.end.length_m),
        width_m=interpolate(motion.start.width_m, motion.end.width_m),
        longitudinal_speed_mps=interpolate(
            motion.start.longitudinal_speed_mps,
            motion.end.longitudinal_speed_mps,
        ),
        lateral_speed_mps=interpolate(
            motion.start.lateral_speed_mps,
            motion.end.lateral_speed_mps,
        ),
    )


def _axes(pose: BodyPose) -> tuple[tuple[float, float], tuple[float, float]]:
    forward = (cos(pose.heading_rad), sin(pose.heading_rad))
    lateral = (-forward[1], forward[0])
    return forward, lateral


def _projection_radius(
    pose: BodyPose,
    axis: tuple[float, float],
) -> float:
    forward, lateral = _axes(pose)
    dot_forward = abs(forward[0] * axis[0] + forward[1] * axis[1])
    dot_lateral = abs(lateral[0] * axis[0] + lateral[1] * axis[1])
    return 0.5 * pose.length_m * dot_forward + 0.5 * pose.width_m * dot_lateral


def oriented_body_overlap(
    first: BodyPose,
    second: BodyPose,
) -> tuple[float, tuple[float, float]] | None:
    """Return minimum penetration and an A-to-B collision normal using SAT."""
    delta = (
        second.longitudinal_m - first.longitudinal_m,
        second.lateral_m - first.lateral_m,
    )
    minimum_overlap = float("inf")
    minimum_axis = (1.0, 0.0)
    for axis in (*_axes(first), *_axes(second)):
        center_distance = abs(delta[0] * axis[0] + delta[1] * axis[1])
        overlap = (
            _projection_radius(first, axis)
            + _projection_radius(second, axis)
            - center_distance
        )
        if overlap < -1e-7:
            return None
        if overlap < minimum_overlap:
            minimum_overlap = max(0.0, overlap)
            direction = 1.0 if delta[0] * axis[0] + delta[1] * axis[1] >= 0 else -1.0
            minimum_axis = (axis[0] * direction, axis[1] * direction)
    return minimum_overlap, minimum_axis


def oriented_body_separation_m(first: BodyPose, second: BodyPose) -> float:
    """Return a conservative SAT separation distance, or zero when overlapping."""
    delta = (
        second.longitudinal_m - first.longitudinal_m,
        second.lateral_m - first.lateral_m,
    )
    maximum_separation = 0.0
    for axis in (*_axes(first), *_axes(second)):
        center_distance = abs(delta[0] * axis[0] + delta[1] * axis[1])
        separation = center_distance - (
            _projection_radius(first, axis) + _projection_radius(second, axis)
        )
        maximum_separation = max(maximum_separation, separation)
    return maximum_separation


def swept_body_collision(
    first: BodyMotion,
    second: BodyMotion,
    *,
    substeps: int = 5,
) -> CollisionContact | None:
    """Find the first contact between two linearly moving oriented bodies."""
    steps = max(1, substeps)
    previous_time = 0.0
    overlap = oriented_body_overlap(first.start, second.start)
    if overlap is not None:
        collision_time = 0.0
        penetration, normal = overlap
    else:
        collision_time = -1.0
        penetration = 0.0
        normal = (1.0, 0.0)
        for index in range(1, steps + 1):
            candidate_time = index / steps
            first_pose = interpolate_pose(first, candidate_time)
            second_pose = interpolate_pose(second, candidate_time)
            overlap = oriented_body_overlap(first_pose, second_pose)
            if overlap is None:
                previous_time = candidate_time
                continue
            low = previous_time
            high = candidate_time
            for _ in range(12):
                midpoint = 0.5 * (low + high)
                if oriented_body_overlap(
                    interpolate_pose(first, midpoint),
                    interpolate_pose(second, midpoint),
                ) is None:
                    low = midpoint
                else:
                    high = midpoint
            collision_time = high
            first_pose = interpolate_pose(first, collision_time)
            second_pose = interpolate_pose(second, collision_time)
            penetration, normal = oriented_body_overlap(first_pose, second_pose) or (
                0.0,
                normal,
            )
            break
        if collision_time < 0.0:
            return None

    first_pose = interpolate_pose(first, collision_time)
    second_pose = interpolate_pose(second, collision_time)
    relative_velocity = (
        second_pose.longitudinal_speed_mps - first_pose.longitudinal_speed_mps,
        second_pose.lateral_speed_mps - first_pose.lateral_speed_mps,
    )
    closing_speed = max(
        0.0,
        -(relative_velocity[0] * normal[0] + relative_velocity[1] * normal[1]),
    )
    contact_type = "front_rear" if abs(normal[0]) >= abs(normal[1]) else "side"
    return CollisionContact(
        time_fraction=collision_time,
        normal_longitudinal=normal[0],
        normal_lateral=normal[1],
        penetration_m=penetration,
        impact_speed_mps=closing_speed,
        contact_type=contact_type,
    )
