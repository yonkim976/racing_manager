"""FULL adaptive planner cadence and nearby-car spatial buckets."""

from __future__ import annotations

from dataclasses import dataclass
from math import floor
from typing import Iterable, Protocol


class SpatialVehicle(Protocol):
    driver_id: int
    total_progress: float
    retired: bool
    finished: bool
    in_pit: bool


@dataclass(frozen=True)
class PlannerSchedule:
    tier_hz: int
    mode: str
    validation_interval_seconds: float
    full_plan_interval_seconds: float


class AdaptivePlannerScheduler:
    """Select a 5/10/20/50 Hz safety-validation tier.

    The expensive lattice has a separate bounded cadence.  At high tiers the
    cached trajectory is checked more often and reactive 50 Hz control remains
    authoritative; full regeneration is never multiplied to 50 Hz.
    """

    def choose(
        self,
        *,
        emergency: bool,
        battle: bool,
        nearest_distance_m: float | None,
    ) -> PlannerSchedule:
        if emergency:
            return PlannerSchedule(50, "critical", 0.02, 0.20)
        if battle:
            return PlannerSchedule(20, "battle", 0.05, 0.20)
        if nearest_distance_m is not None and nearest_distance_m <= 45.0:
            return PlannerSchedule(10, "traffic", 0.10, 0.50)
        return PlannerSchedule(5, "cruise", 0.20, 1.00)


class TrackSpatialIndex:
    """One-dimensional progress buckets for local trajectory traffic queries."""

    def __init__(self, track_length_m: float, bucket_size_m: float = 60.0) -> None:
        self.track_length_m = max(1.0, float(track_length_m))
        self.bucket_size_m = max(10.0, float(bucket_size_m))
        self._buckets: dict[int, list[SpatialVehicle]] = {}

    def rebuild(self, vehicles: Iterable[SpatialVehicle]) -> None:
        buckets: dict[int, list[SpatialVehicle]] = {}
        for vehicle in vehicles:
            if vehicle.retired or vehicle.finished or vehicle.in_pit:
                continue
            distance_m = vehicle.total_progress * self.track_length_m
            bucket = floor(distance_m / self.bucket_size_m)
            buckets.setdefault(bucket, []).append(vehicle)
        for values in buckets.values():
            values.sort(key=lambda item: (item.total_progress, item.driver_id))
        self._buckets = buckets

    def nearby(
        self,
        vehicle: SpatialVehicle,
        radius_m: float,
    ) -> tuple[SpatialVehicle, ...]:
        center_m = vehicle.total_progress * self.track_length_m
        center_bucket = floor(center_m / self.bucket_size_m)
        bucket_radius = max(1, int(radius_m / self.bucket_size_m) + 1)
        candidates: list[SpatialVehicle] = []
        for bucket in range(center_bucket - bucket_radius, center_bucket + bucket_radius + 1):
            candidates.extend(self._buckets.get(bucket, ()))
        nearby = [
            other
            for other in candidates
            if other.driver_id != vehicle.driver_id
            and abs(other.total_progress - vehicle.total_progress) * self.track_length_m
            <= radius_m
        ]
        nearby.sort(
            key=lambda other: (
                abs(other.total_progress - vehicle.total_progress),
                other.driver_id,
            )
        )
        return tuple(nearby)
