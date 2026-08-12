"""FULL vehicle and tyre physics for whole-lap trajectory evaluation."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from models.schemas import PhysicalTireCompound, TireCompound
from .car_performance import CarPerformanceFactors
from .tire_model import TirePhysicsFactors
from .vehicle_dynamics import (
    MAX_SPEED_MPS,
    drive_force_limits_n,
    force_envelope,
    lateral_speed_limit_mps,
    maximum_brake_force_n,
    maximum_braking_deceleration_mps2,
    maximum_drive_acceleration_mps2,
)

TRAJECTORY_SPEED_PASS_COUNT = 8
MIN_TRAJECTORY_SPEED_MPS = 5.0


@dataclass(frozen=True)
class VehicleTrajectorySpec:
    """Stable car specification used while optimizing a nominal whole lap."""

    mass_kg: float
    engine_power_kw: float
    drivetrain_efficiency: float
    corner_drag_area_m2: float
    straight_drag_area_m2: float
    corner_downforce_area_m2: float
    straight_downforce_area_m2: float
    brake_force_n: float
    mechanical_grip: float
    traction_factor: float

    @classmethod
    def from_car_performance(
        cls,
        factors: CarPerformanceFactors,
    ) -> "VehicleTrajectorySpec":
        return cls(
            mass_kg=factors.mass_kg,
            engine_power_kw=factors.effective_power_kw,
            drivetrain_efficiency=factors.drivetrain_efficiency,
            corner_drag_area_m2=factors.corner_drag_area_m2,
            straight_drag_area_m2=factors.straight_drag_area_m2,
            corner_downforce_area_m2=factors.corner_downforce_area_m2,
            straight_downforce_area_m2=factors.straight_downforce_area_m2,
            brake_force_n=factors.brake_force_n,
            mechanical_grip=factors.mechanical_grip,
            traction_factor=factors.traction_factor,
        )


@dataclass(frozen=True)
class TireTrajectorySpec:
    """Compound and wear-dependent force factors for a nominal trajectory."""

    compound: PhysicalTireCompound | TireCompound
    wear: float
    lateral_grip: float
    braking_grip: float
    traction_grip: float

    @classmethod
    def from_tire_physics(
        cls,
        compound: PhysicalTireCompound | TireCompound,
        factors: TirePhysicsFactors,
    ) -> "TireTrajectorySpec":
        return cls(
            compound=compound,
            wear=factors.wear,
            lateral_grip=factors.lateral_grip,
            braking_grip=factors.braking_grip,
            traction_grip=factors.traction_grip,
        )


@dataclass(frozen=True)
class NominalTrajectoryDynamics:
    """Adapter satisfying the shared vehicle-dynamics parameter protocol."""

    power: float
    grip: float
    braking: float
    traction: float
    mass_kg: float
    effective_power_kw: float
    drivetrain_efficiency: float
    corner_drag_area_m2: float
    straight_drag_area_m2: float
    corner_downforce_area_m2: float
    straight_downforce_area_m2: float
    brake_force_n: float
    mechanical_grip: float
    traction_factor: float
    drag_multiplier: float = 1.0
    downforce_multiplier: float = 1.0


@dataclass(frozen=True)
class TrajectorySpeedProfile:
    target_speeds_mps: tuple[float, ...]
    brake_utilization: tuple[float, ...]
    throttle_utilization: tuple[float, ...]
    lap_time_seconds: float


def nominal_dynamics_parameters(
    vehicle: VehicleTrajectorySpec,
    tire: TireTrajectorySpec,
) -> NominalTrajectoryDynamics:
    return NominalTrajectoryDynamics(
        power=1.0,
        grip=tire.lateral_grip,
        braking=tire.braking_grip,
        traction=tire.traction_grip,
        mass_kg=vehicle.mass_kg,
        effective_power_kw=vehicle.engine_power_kw,
        drivetrain_efficiency=vehicle.drivetrain_efficiency,
        corner_drag_area_m2=vehicle.corner_drag_area_m2,
        straight_drag_area_m2=vehicle.straight_drag_area_m2,
        corner_downforce_area_m2=vehicle.corner_downforce_area_m2,
        straight_downforce_area_m2=vehicle.straight_downforce_area_m2,
        brake_force_n=vehicle.brake_force_n,
        mechanical_grip=vehicle.mechanical_grip,
        traction_factor=vehicle.traction_factor,
    )


def build_trajectory_speed_profile(
    segment_lengths_m: list[float] | tuple[float, ...],
    curvatures_1pm: list[float] | tuple[float, ...],
    vehicle: VehicleTrajectorySpec,
    tire: TireTrajectorySpec,
    *,
    braking_utilization: float = 1.0,
    pass_count: int = TRAJECTORY_SPEED_PASS_COUNT,
) -> TrajectorySpeedProfile:
    """Solve a closed-lap speed profile with shared force-envelope equations."""
    if len(segment_lengths_m) != len(curvatures_1pm):
        raise ValueError("trajectory lengths and curvatures must have equal size")
    if not segment_lengths_m:
        return TrajectorySpeedProfile((), (), (), 0.0)

    parameters = nominal_dynamics_parameters(vehicle, tire)
    count = len(segment_lengths_m)
    speeds = [
        lateral_speed_limit_mps(curvature, parameters)
        for curvature in curvatures_1pm
    ]
    braking_factor = max(0.3, min(1.0, braking_utilization))

    for _ in range(max(1, pass_count)):
        for index in range(count - 1, -1, -1):
            next_index = (index + 1) % count
            distance_m = max(0.0, segment_lengths_m[index])
            deceleration = maximum_braking_deceleration_mps2(
                curvatures_1pm[index],
                speeds[index],
                parameters,
            ) * braking_factor
            allowed = sqrt(
                max(
                    0.0,
                    speeds[next_index] * speeds[next_index]
                    + 2.0 * deceleration * distance_m,
                )
            )
            speeds[index] = min(speeds[index], allowed)

        for index in range(count):
            next_index = (index + 1) % count
            distance_m = max(0.0, segment_lengths_m[index])
            acceleration = maximum_drive_acceleration_mps2(
                curvatures_1pm[index],
                speeds[index],
                parameters,
            )
            reachable = sqrt(
                max(
                    0.0,
                    speeds[index] * speeds[index]
                    + 2.0 * acceleration * distance_m,
                )
            )
            speeds[next_index] = min(
                speeds[next_index],
                max(MIN_TRAJECTORY_SPEED_MPS, min(MAX_SPEED_MPS, reachable)),
            )

    brake_utilization: list[float] = []
    throttle_utilization: list[float] = []
    lap_time = 0.0
    for index, distance_m in enumerate(segment_lengths_m):
        next_index = (index + 1) % count
        distance = max(1e-6, distance_m)
        speed = speeds[index]
        next_speed = speeds[next_index]
        lap_time += 2.0 * distance / max(
            1.0,
            speed + next_speed,
        )
        kinematic_acceleration = (
            next_speed * next_speed - speed * speed
        ) / (2.0 * distance)
        envelope = force_envelope(curvatures_1pm[index], speed, parameters)
        if kinematic_acceleration < -1e-6:
            requested_brake_force_n = max(
                0.0,
                -kinematic_acceleration * envelope.mass_kg
                - envelope.drag_force_n
                - envelope.rolling_resistance_n,
            )
            maximum_braking_force_n = maximum_brake_force_n(
                envelope,
                parameters,
            )
            brake_utilization.append(
                min(
                    1.0,
                    requested_brake_force_n
                    / max(1e-9, maximum_braking_force_n),
                )
            )
            throttle_utilization.append(0.0)
        else:
            _, _, maximum_drive_force = drive_force_limits_n(
                speed,
                envelope,
                parameters,
            )
            requested_drive_force_n = max(
                0.0,
                kinematic_acceleration * envelope.mass_kg
                + envelope.drag_force_n
                + envelope.rolling_resistance_n,
            )
            throttle_utilization.append(
                min(
                    1.0,
                    requested_drive_force_n / max(1e-9, maximum_drive_force),
                )
            )
            brake_utilization.append(0.0)

    return TrajectorySpeedProfile(
        target_speeds_mps=tuple(speeds),
        brake_utilization=tuple(brake_utilization),
        throttle_utilization=tuple(throttle_utilization),
        lap_time_seconds=lap_time,
    )
