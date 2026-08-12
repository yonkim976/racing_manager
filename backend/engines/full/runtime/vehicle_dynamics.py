"""FULL stateless force calculations for planning and 50 Hz physics."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan, atan2, sqrt
from typing import Protocol

AIR_DENSITY_KG_M3 = 1.225
GRAVITY_MPS2 = 9.81
BASE_TYRE_FRICTION = 1.72
DOWNFORCE_GRIP_EFFICIENCY = 0.55
REAR_TRACTION_SHARE = 0.62
ROLLING_RESISTANCE_COEFFICIENT = 0.015
MAX_ACCELERATION_MPS2 = 15.0
MAX_BRAKING_MPS2 = 55.0
MAX_SPEED_MPS = 105.0
DYNAMIC_BICYCLE_MAX_STEERING_RAD = 0.35
DYNAMIC_BICYCLE_MAX_CONTROLLED_HEADING_ERROR_RAD = 0.105
DYNAMIC_BICYCLE_FRONT_CORNERING_STIFFNESS_NPRAD = 90000.0
DYNAMIC_BICYCLE_REAR_CORNERING_STIFFNESS_NPRAD = 105000.0
DYNAMIC_BICYCLE_AERO_LOAD_STIFFNESS_EXPONENT = 0.65
DYNAMIC_BICYCLE_MAX_AERO_LOAD_STIFFNESS_SCALE = 2.5
DYNAMIC_BICYCLE_YAW_RATE_RELAXATION_PER_SECOND = 18.0


class VehicleDynamicsParameters(Protocol):
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
    drag_multiplier: float
    downforce_multiplier: float


@dataclass(frozen=True)
class VehicleForceEnvelope:
    mass_kg: float
    drag_area_m2: float
    downforce_area_m2: float
    drag_force_n: float
    downforce_n: float
    normal_force_n: float
    rolling_resistance_n: float
    maximum_tire_force_n: float
    required_lateral_force_n: float
    available_longitudinal_force_n: float
    lateral_grip_utilization: float


@dataclass(frozen=True)
class DynamicBicycleState:
    """Planar body state expressed relative to the active track tangent."""

    lateral_offset_m: float
    lateral_speed_mps: float
    heading_error_rad: float
    yaw_rate_rad_s: float


@dataclass(frozen=True)
class DynamicBicycleResult:
    lateral_offset_m: float
    lateral_speed_mps: float
    lateral_acceleration_mps2: float
    heading_error_rad: float
    yaw_rate_rad_s: float
    steering_angle_rad: float
    front_slip_angle_rad: float
    rear_slip_angle_rad: float
    front_lateral_force_n: float
    rear_lateral_force_n: float


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def advance_dynamic_bicycle(
    state: DynamicBicycleState,
    *,
    speed_mps: float,
    curvature_1pm: float,
    target_lateral_offset_m: float,
    target_lateral_speed_mps: float,
    delta_seconds: float,
    mass_kg: float,
    wheelbase_m: float,
    yaw_inertia_kgm2: float,
    maximum_tire_force_n: float,
    front_force_share: float,
    grip_factor: float,
    nominal_tire_force_n: float | None = None,
    external_lateral_acceleration_mps2: float = 0.0,
    steering_curvature_1pm: float | None = None,
) -> DynamicBicycleResult:
    """Advance a linear dynamic bicycle model in track-relative coordinates.

    The controller supplies a small feed-forward steering angle for path
    curvature, then corrects lateral and heading error.  Front and rear slip
    angles generate tyre force independently and are clamped by the current
    physical tyre envelope.  This keeps body rotation continuous instead of
    moving the car sideways as an orientation-free point mass.
    """

    step = max(0.0, float(delta_seconds))
    speed = max(0.5, float(speed_mps))
    mass = max(1.0, float(mass_kg))
    wheelbase = max(1.0, float(wheelbase_m))
    inertia = max(500.0, float(yaw_inertia_kgm2))
    front_share = _clamp(float(front_force_share), 0.35, 0.65)
    rear_share = 1.0 - front_share
    # The CG location follows the static force split: front load share = b/L.
    rear_axle_to_cg_m = wheelbase * front_share
    front_axle_to_cg_m = wheelbase - rear_axle_to_cg_m

    # Convert track-relative lateral velocity to body lateral velocity using
    # the small-angle curvilinear relation e_y_dot = v_y + u * e_psi.
    body_lateral_speed_mps = (
        state.lateral_speed_mps - speed * state.heading_error_rad
    )
    lateral_error_m = target_lateral_offset_m - state.lateral_offset_m
    desired_lateral_speed_mps = _clamp(
        target_lateral_speed_mps + lateral_error_m * 0.85,
        -5.0,
        5.0,
    )
    desired_heading_error_rad = _clamp(
        atan2(desired_lateral_speed_mps, speed),
        -0.18,
        0.18,
    )
    steering_curvature = (
        float(steering_curvature_1pm)
        if steering_curvature_1pm is not None
        else curvature_1pm
    )
    desired_yaw_rate_rad_s = speed * steering_curvature
    feed_forward_rad = atan(wheelbase * steering_curvature)
    velocity_heading_error_rad = state.heading_error_rad + atan2(
        body_lateral_speed_mps,
        speed,
    )
    steering_angle_rad = _clamp(
        feed_forward_rad
        + 0.90 * (desired_heading_error_rad - velocity_heading_error_rad)
        + 0.02 * (desired_yaw_rate_rad_s - state.yaw_rate_rad_s),
        -DYNAMIC_BICYCLE_MAX_STEERING_RAD,
        DYNAMIC_BICYCLE_MAX_STEERING_RAD,
    )

    front_slip_angle_rad = steering_angle_rad - atan2(
        body_lateral_speed_mps
        + front_axle_to_cg_m * state.yaw_rate_rad_s,
        speed,
    )
    rear_slip_angle_rad = -atan2(
        body_lateral_speed_mps
        - rear_axle_to_cg_m * state.yaw_rate_rad_s,
        speed,
    )

    # Cornering stiffness grows with vertical load, but more slowly than the
    # available lateral force because pneumatic tyres are load-sensitive.  A
    # fixed static-load stiffness made a high-downforce car require excessive
    # body sideslip even while it still had ample force capacity.  That drove
    # the track-relative heading controller into its safety clamp and caused
    # the car to drift away from an otherwise feasible constant-radius path.
    #
    # Use the nominal force envelope here, not the force left after braking or
    # traction demand.  Combined-slip demand reduces the force that can be
    # applied, but does not instantly remove the tyre's vertical-load-derived
    # cornering stiffness.
    effective_grip_factor = _clamp(grip_factor, 0.45, 1.35)
    static_tire_force_n = (
        mass * GRAVITY_MPS2 * BASE_TYRE_FRICTION * effective_grip_factor
    )
    load_force_n = (
        float(nominal_tire_force_n)
        if nominal_tire_force_n is not None
        else float(maximum_tire_force_n)
    )
    aero_load_ratio = max(1.0, load_force_n / max(1.0, static_tire_force_n))
    aero_load_stiffness_scale = _clamp(
        aero_load_ratio ** DYNAMIC_BICYCLE_AERO_LOAD_STIFFNESS_EXPONENT,
        1.0,
        DYNAMIC_BICYCLE_MAX_AERO_LOAD_STIFFNESS_SCALE,
    )
    stiffness_scale = effective_grip_factor * aero_load_stiffness_scale
    front_force_n = (
        DYNAMIC_BICYCLE_FRONT_CORNERING_STIFFNESS_NPRAD
        * stiffness_scale
        * front_slip_angle_rad
    )
    rear_force_n = (
        DYNAMIC_BICYCLE_REAR_CORNERING_STIFFNESS_NPRAD
        * stiffness_scale
        * rear_slip_angle_rad
    )
    # Scale the axle forces together at the total tyre limit.  Independently
    # pinning both axles at their static load shares erases the restoring yaw
    # moment at saturation and lets yaw rate grow without bound.
    combined_lateral_force_n = abs(front_force_n) + abs(rear_force_n)
    if combined_lateral_force_n > max(1.0, maximum_tire_force_n):
        force_scale = maximum_tire_force_n / combined_lateral_force_n
        front_force_n *= force_scale
        rear_force_n *= force_scale

    body_lateral_acceleration_mps2 = (
        (front_force_n + rear_force_n) / mass
        - speed * state.yaw_rate_rad_s
        + external_lateral_acceleration_mps2
    )
    yaw_acceleration_rad_s2 = (
        front_axle_to_cg_m * front_force_n
        - rear_axle_to_cg_m * rear_force_n
    ) / inertia
    # The single-track reduction omits several real damping sources (four
    # contact patches, steering compliance and differential response).  A
    # bounded yaw relaxation represents those sources and prevents the linear
    # tyre approximation from becoming non-physical after force saturation.
    yaw_acceleration_rad_s2 -= DYNAMIC_BICYCLE_YAW_RATE_RELAXATION_PER_SECOND * (
        state.yaw_rate_rad_s - desired_yaw_rate_rad_s
    )
    yaw_acceleration_rad_s2 = _clamp(yaw_acceleration_rad_s2, -18.0, 18.0)
    next_body_lateral_speed_mps = (
        body_lateral_speed_mps + body_lateral_acceleration_mps2 * step
    )
    next_yaw_rate_rad_s = state.yaw_rate_rad_s + yaw_acceleration_rad_s2 * step
    unconstrained_heading_error_rad = state.heading_error_rad + (
        0.5 * (state.yaw_rate_rad_s + next_yaw_rate_rad_s)
        - speed * curvature_1pm
    ) * step
    next_heading_error_rad = _clamp(
        unconstrained_heading_error_rad,
        -DYNAMIC_BICYCLE_MAX_CONTROLLED_HEADING_ERROR_RAD,
        DYNAMIC_BICYCLE_MAX_CONTROLLED_HEADING_ERROR_RAD,
    )
    if next_heading_error_rad != unconstrained_heading_error_rad:
        next_yaw_rate_rad_s = desired_yaw_rate_rad_s
    next_lateral_speed_mps = (
        next_body_lateral_speed_mps + speed * next_heading_error_rad
    )
    next_lateral_speed_mps = _clamp(next_lateral_speed_mps, -4.0, 4.0)
    next_lateral_offset_m = state.lateral_offset_m + 0.5 * (
        state.lateral_speed_mps + next_lateral_speed_mps
    ) * step
    # An accelerometer reports the tyre-force acceleration of the body.  The
    # derivative of track-relative lateral speed also contains the rotating
    # track frame and controller/reference changes; reporting that derivative
    # produced non-physical 20 g spikes at sharp curvature transitions.
    physical_lateral_acceleration_mps2 = (
        (front_force_n + rear_force_n) / mass
        + external_lateral_acceleration_mps2
    )
    return DynamicBicycleResult(
        lateral_offset_m=next_lateral_offset_m,
        lateral_speed_mps=next_lateral_speed_mps,
        lateral_acceleration_mps2=physical_lateral_acceleration_mps2,
        heading_error_rad=next_heading_error_rad,
        yaw_rate_rad_s=next_yaw_rate_rad_s,
        steering_angle_rad=steering_angle_rad,
        front_slip_angle_rad=front_slip_angle_rad,
        rear_slip_angle_rad=rear_slip_angle_rad,
        front_lateral_force_n=front_force_n,
        rear_lateral_force_n=rear_force_n,
    )


def aero_coefficients(
    curvature_1pm: float,
    parameters: VehicleDynamicsParameters,
) -> tuple[float, float]:
    """Blend straight/corner aero modes using the same curvature demand."""
    corner_demand = max(0.0, min(1.0, abs(curvature_1pm) / 0.018))
    drag_area = (
        parameters.straight_drag_area_m2 * (1.0 - corner_demand)
        + parameters.corner_drag_area_m2 * corner_demand
    ) * parameters.drag_multiplier
    downforce_area = (
        parameters.straight_downforce_area_m2 * (1.0 - corner_demand)
        + parameters.corner_downforce_area_m2 * corner_demand
    ) * parameters.downforce_multiplier
    return max(0.1, drag_area), max(0.1, downforce_area)


def force_envelope(
    curvature_1pm: float,
    speed_mps: float,
    parameters: VehicleDynamicsParameters,
) -> VehicleForceEnvelope:
    """Return available tyre and resistance forces at one path state."""
    speed = max(0.0, speed_mps)
    mass_kg = max(1.0, parameters.mass_kg)
    drag_area_m2, downforce_area_m2 = aero_coefficients(
        curvature_1pm,
        parameters,
    )
    dynamic_pressure = 0.5 * AIR_DENSITY_KG_M3 * speed * speed
    drag_force_n = dynamic_pressure * drag_area_m2
    downforce_n = dynamic_pressure * downforce_area_m2
    normal_force_n = mass_kg * GRAVITY_MPS2 + downforce_n
    grip_normal_force_n = (
        mass_kg * GRAVITY_MPS2
        + downforce_n * DOWNFORCE_GRIP_EFFICIENCY
    )
    tire_friction = (
        BASE_TYRE_FRICTION
        * parameters.mechanical_grip
        * parameters.grip
    )
    maximum_tire_force_n = grip_normal_force_n * tire_friction
    required_lateral_force_n = mass_kg * speed * speed * abs(curvature_1pm)
    available_longitudinal_force_n = sqrt(
        max(
            0.0,
            maximum_tire_force_n * maximum_tire_force_n
            - min(required_lateral_force_n, maximum_tire_force_n) ** 2,
        )
    )
    lateral_grip_utilization = (
        required_lateral_force_n / maximum_tire_force_n
        if maximum_tire_force_n > 1e-9
        else 0.0
    )
    return VehicleForceEnvelope(
        mass_kg=mass_kg,
        drag_area_m2=drag_area_m2,
        downforce_area_m2=downforce_area_m2,
        drag_force_n=drag_force_n,
        downforce_n=downforce_n,
        normal_force_n=normal_force_n,
        rolling_resistance_n=(
            ROLLING_RESISTANCE_COEFFICIENT * normal_force_n
        ),
        maximum_tire_force_n=maximum_tire_force_n,
        required_lateral_force_n=required_lateral_force_n,
        available_longitudinal_force_n=available_longitudinal_force_n,
        lateral_grip_utilization=lateral_grip_utilization,
    )


def lateral_speed_limit_mps(
    curvature_1pm: float,
    parameters: VehicleDynamicsParameters,
) -> float:
    """Solve steady-state speed where lateral tyre force is exhausted."""
    curvature = abs(curvature_1pm)
    if curvature <= 1e-8:
        return MAX_SPEED_MPS

    mass_kg = max(1.0, parameters.mass_kg)
    _, downforce_area_m2 = aero_coefficients(curvature, parameters)
    tire_friction = (
        BASE_TYRE_FRICTION
        * parameters.mechanical_grip
        * parameters.grip
    )
    speed_squared_denominator = (
        mass_kg * curvature
        - tire_friction
        * 0.5
        * AIR_DENSITY_KG_M3
        * downforce_area_m2
        * DOWNFORCE_GRIP_EFFICIENCY
    )
    if speed_squared_denominator <= 1e-9:
        return MAX_SPEED_MPS
    speed_squared = (
        tire_friction * mass_kg * GRAVITY_MPS2
        / speed_squared_denominator
    )
    return min(MAX_SPEED_MPS, sqrt(max(0.0, speed_squared)))


def maximum_braking_deceleration_mps2(
    curvature_1pm: float,
    speed_mps: float,
    parameters: VehicleDynamicsParameters,
) -> float:
    envelope = force_envelope(curvature_1pm, speed_mps, parameters)
    brake_force_n = maximum_brake_force_n(envelope, parameters)
    return max(
        1.0,
        min(
            MAX_BRAKING_MPS2,
            (
                brake_force_n
                + envelope.drag_force_n
                + envelope.rolling_resistance_n
            )
            / envelope.mass_kg,
        ),
    )


def maximum_brake_force_n(
    envelope: VehicleForceEnvelope,
    parameters: VehicleDynamicsParameters,
) -> float:
    return min(
        parameters.brake_force_n,
        envelope.available_longitudinal_force_n * parameters.braking,
    )


def drive_force_limits_n(
    speed_mps: float,
    envelope: VehicleForceEnvelope,
    parameters: VehicleDynamicsParameters,
) -> tuple[float, float, float]:
    """Return power, traction and selected full-throttle drive forces."""
    wheel_power_w = (
        parameters.effective_power_kw
        * 1000.0
        * parameters.drivetrain_efficiency
        * parameters.power
    )
    power_limited_force_n = wheel_power_w / max(8.0, speed_mps)
    traction_limited_force_n = (
        envelope.available_longitudinal_force_n
        * REAR_TRACTION_SHARE
        * parameters.traction_factor
        * parameters.traction
    )
    return (
        power_limited_force_n,
        traction_limited_force_n,
        min(power_limited_force_n, traction_limited_force_n),
    )


def maximum_drive_force_n(
    curvature_1pm: float,
    speed_mps: float,
    parameters: VehicleDynamicsParameters,
) -> tuple[float, VehicleForceEnvelope]:
    envelope = force_envelope(curvature_1pm, speed_mps, parameters)
    _, _, drive_force_n = drive_force_limits_n(
        speed_mps,
        envelope,
        parameters,
    )
    return drive_force_n, envelope


def maximum_drive_acceleration_mps2(
    curvature_1pm: float,
    speed_mps: float,
    parameters: VehicleDynamicsParameters,
) -> float:
    drive_force_n, envelope = maximum_drive_force_n(
        curvature_1pm,
        speed_mps,
        parameters,
    )
    return max(
        -MAX_BRAKING_MPS2,
        min(
            MAX_ACCELERATION_MPS2,
            (
                drive_force_n
                - envelope.drag_force_n
                - envelope.rolling_resistance_n
            )
            / envelope.mass_kg,
        ),
    )
