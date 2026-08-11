"""Stage B single-vehicle kinematic pose synthesis."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, ceil, cos, floor, pi, sin
from typing import Any

from simulation.track_contracts import DRIVING_LINE_RACING
from simulation.start_grid_geometry import GridDisplaySlot

from .clock import LogicalClock
from .kinematics import (
    BoundedKinematicPoseBuffer,
    GRID_HOLD_DURATION_S,
    GRID_LATERAL_HOLD_AFTER_LIGHTS_S,
    GRID_LATERAL_REJOIN_DURATION_S,
    KINEMATIC_LOGICAL_TICK_SECONDS,
    KinematicStep,
    LateralTrajectory,
    LongitudinalKinematicPlan,
    RacingLineDistanceContract,
    build_longitudinal_plan,
    pose_sequence_hash,
)
from .state import AbstractSessionSnapshot, AbstractTrackSnapshot


Point = tuple[float, float]


def _normalize(vector: Point) -> Point:
    length = (vector[0] * vector[0] + vector[1] * vector[1]) ** 0.5
    if length <= 1e-12:
        return 1.0, 0.0
    return vector[0] / length, vector[1] / length


@dataclass(frozen=True, slots=True)
class TrackSplineSample:
    progress: float
    track_distance_m: float
    line_distance_m: float
    world_x_m: float
    world_y_m: float
    tangent_x: float
    tangent_y: float
    normal_x: float
    normal_y: float
    heading_rad: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "progress": round(self.progress, 9),
            "track_distance_m": round(self.track_distance_m, 9),
            "line_distance_m": round(self.line_distance_m, 9),
            "world_x_m": round(self.world_x_m, 9),
            "world_y_m": round(self.world_y_m, 9),
            "tangent_x": round(self.tangent_x, 9),
            "tangent_y": round(self.tangent_y, 9),
            "normal_x": round(self.normal_x, 9),
            "normal_y": round(self.normal_y, 9),
            "heading_rad": round(self.heading_rad, 9),
        }


class TrackSpline:
    """Compatibility facade over the compiled FULL racing-line sampler.

    The name remains for existing pose callers, but this class no longer owns
    an independent centerline spline.  All samples come from the public
    ``TrackGeometryProfile.line_pose_at_progress_m`` contract.
    """

    def __init__(self, track: AbstractTrackSnapshot, samples_per_segment: int = 16):
        if samples_per_segment < 4:
            raise ValueError("samples_per_segment must be at least four")
        if track.display_geometry is None:
            raise ValueError("pose synthesis requires compiled display geometry")
        self.track = track
        self.profile = track.display_geometry.compiled_profile
        self.distance_contract = RacingLineDistanceContract(self.profile)
        self.total_length_m = self.distance_contract.line_length_m
        if self.total_length_m <= 0.0:
            raise ValueError("compiled racing line has no measurable length")

    def sample(self, progress: float) -> TrackSplineSample:
        normalized = progress % 1.0
        world_x_m, world_y_m, heading_rad = self.profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            normalized,
        )
        tangent = _normalize((cos(heading_rad), sin(heading_rad)))
        normal = (-tangent[1], tangent[0])
        return TrackSplineSample(
            progress=normalized,
            # Compatibility field: callers that still consume this legacy
            # value receive circuit-frame distance.  New kinematics use
            # ``line_distance_m`` exclusively as their distance authority.
            track_distance_m=normalized * self.track.track_length_m,
            line_distance_m=self.distance_contract.distance_at_total_progress(progress),
            world_x_m=world_x_m,
            world_y_m=world_y_m,
            tangent_x=tangent[0],
            tangent_y=tangent[1],
            normal_x=normal[0],
            normal_y=normal[1],
            heading_rad=heading_rad,
        )


def _wrap_heading(heading_rad: float) -> float:
    return (heading_rad + pi) % (2.0 * pi) - pi


@dataclass(frozen=True, slots=True)
class AbstractPoseFrame:
    """Minimal display pose; no fake force, slip or thermal fields."""

    driver_id: int | str
    simulation_time_s: float
    physics_frame: int
    total_progress: float
    lap_number: int
    progress: float
    track_distance_m: float
    lateral_offset_m: float
    world_x_m: float
    world_y_m: float
    heading_rad: float
    visual_state: str = "normal"
    source_mode: str = "abstract"
    line_distance_m: float = 0.0
    speed_mps: float = 0.0
    longitudinal_acceleration_mps2: float = 0.0
    lateral_velocity_mps: float = 0.0
    lateral_acceleration_mps2: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "driver_id": self.driver_id,
            "simulation_time_s": round(self.simulation_time_s, 9),
            "physics_frame": self.physics_frame,
            "total_progress": round(self.total_progress, 9),
            "lap_number": self.lap_number,
            "progress": round(self.progress, 9),
            "track_distance_m": round(self.track_distance_m, 9),
            "lateral_offset_m": round(self.lateral_offset_m, 9),
            "world_x_m": round(self.world_x_m, 9),
            "world_y_m": round(self.world_y_m, 9),
            "heading_rad": round(self.heading_rad, 9),
            "visual_state": self.visual_state,
            "source_mode": self.source_mode,
            "line_distance_m": round(self.line_distance_m, 9),
            "speed_mps": round(self.speed_mps, 9),
            "speed_kph": round(self.speed_mps * 3.6, 9),
            "longitudinal_acceleration_mps2": round(self.longitudinal_acceleration_mps2, 9),
            "lateral_velocity_mps": round(self.lateral_velocity_mps, 9),
            "lateral_acceleration_mps2": round(self.lateral_acceleration_mps2, 9),
        }


@dataclass(frozen=True, slots=True)
class SingleProbeKinematicTick:
    """One accepted logical probe tick and the pose derived from that step."""

    logical_tick_index: int
    logical_time_s: float
    pose: AbstractPoseFrame
    accepted_step: KinematicStep
    grid_hold: bool = False


class SingleVehiclePoseSynthesizer:
    """Create one continuous vehicle pose stream from logical progress."""

    def __init__(
        self,
        track: AbstractTrackSnapshot | AbstractSessionSnapshot,
        *,
        samples_per_segment: int = 16,
    ):
        track_snapshot = track.track if isinstance(track, AbstractSessionSnapshot) else track
        self.snapshot = track if isinstance(track, AbstractSessionSnapshot) else None
        self.track = track_snapshot
        self.spline = TrackSpline(track_snapshot, samples_per_segment=samples_per_segment)
        self.distance_contract = self.spline.distance_contract
        self.default_plan = build_longitudinal_plan(
            self.snapshot or track_snapshot,
            distance_contract=self.distance_contract,
        )

    @property
    def line_length_m(self) -> float:
        return self.distance_contract.line_length_m

    def build_plan(self, driver_id: int | str | None = None) -> LongitudinalKinematicPlan:
        return build_longitudinal_plan(
            self.snapshot or self.track,
            driver_id,
            distance_contract=self.distance_contract,
        )

    def validate_lateral_offset(
        self,
        line_distance_m: float,
        lateral_offset_m: float,
        *,
        edge_margin_m: float = 0.15,
    ) -> None:
        """Reject a pose outside the metric track envelope; never snap it."""

        geometry = self.track.display_geometry
        if geometry is None:
            raise ValueError("lateral boundary validation requires display geometry")
        progress = self.distance_contract.total_progress_at_distance(line_distance_m)
        minimum, maximum = self.distance_contract.lateral_limits_at_progress(
            progress,
            car_width_m=geometry.car_width_m,
            edge_margin_m=edge_margin_m,
        )
        if not minimum - 1e-9 <= lateral_offset_m <= maximum + 1e-9:
            raise ValueError(
                "lateral trajectory leaves the track envelope "
                f"at line distance {line_distance_m:.6f}m: "
                f"{lateral_offset_m:.6f} not in [{minimum:.6f}, {maximum:.6f}]"
            )

    def pose_at_line_distance(
        self,
        driver_id: int | str,
        line_distance_m: float,
        *,
        simulation_time_s: float = 0.0,
        physics_frame: int = 0,
        speed_mps: float | None = None,
        longitudinal_acceleration_mps2: float = 0.0,
        lateral_offset_m: float = 0.0,
        lateral_velocity_mps: float = 0.0,
        lateral_acceleration_mps2: float = 0.0,
        heading_offset_rad: float = 0.0,
        visual_state: str = "normal",
        plan: LongitudinalKinematicPlan | None = None,
    ) -> AbstractPoseFrame:
        if simulation_time_s < 0.0 or physics_frame < 0:
            raise ValueError("pose time and frame must not be negative")
        plan = plan or self.default_plan
        total_progress = self.distance_contract.total_progress_at_distance(line_distance_m)
        sample = self.spline.sample(total_progress)
        actual_speed = (
            plan.target_speed_at_line_distance(line_distance_m)
            if speed_mps is None
            else max(0.0, float(speed_mps))
        )
        world_x_m = sample.world_x_m + sample.normal_x * lateral_offset_m
        world_y_m = sample.world_y_m + sample.normal_y * lateral_offset_m
        tangent_x = cos(sample.heading_rad)
        tangent_y = sin(sample.heading_rad)
        velocity_x = tangent_x * actual_speed + sample.normal_x * lateral_velocity_mps
        velocity_y = tangent_y * actual_speed + sample.normal_y * lateral_velocity_mps
        if abs(velocity_x) + abs(velocity_y) > 1e-9:
            heading = atan2(velocity_y, velocity_x)
        else:
            heading = sample.heading_rad
        heading = _wrap_heading(heading + heading_offset_rad)
        return AbstractPoseFrame(
            driver_id=driver_id,
            simulation_time_s=round(simulation_time_s, 9),
            physics_frame=physics_frame,
            total_progress=total_progress,
            lap_number=max(1, floor(total_progress) + 1),
            progress=sample.progress,
            track_distance_m=total_progress * self.track.track_length_m,
            lateral_offset_m=lateral_offset_m,
            world_x_m=world_x_m,
            world_y_m=world_y_m,
            heading_rad=heading,
            visual_state=visual_state,
            line_distance_m=max(0.0, float(line_distance_m)),
            speed_mps=max(0.0, actual_speed),
            longitudinal_acceleration_mps2=longitudinal_acceleration_mps2,
            lateral_velocity_mps=lateral_velocity_mps,
            lateral_acceleration_mps2=lateral_acceleration_mps2,
        )

    def pose_at(
        self,
        driver_id: int | str,
        total_progress: float,
        *,
        simulation_time_s: float = 0.0,
        physics_frame: int = 0,
        lateral_offset_m: float = 0.0,
        heading_offset_rad: float = 0.0,
        visual_state: str = "normal",
        speed_mps: float | None = None,
        longitudinal_acceleration_mps2: float = 0.0,
        lateral_velocity_mps: float = 0.0,
        lateral_acceleration_mps2: float = 0.0,
    ) -> AbstractPoseFrame:
        if simulation_time_s < 0.0 or physics_frame < 0:
            raise ValueError("pose time and frame must not be negative")
        return self.pose_at_line_distance(
            driver_id,
            self.distance_contract.distance_at_total_progress(total_progress),
            simulation_time_s=simulation_time_s,
            physics_frame=physics_frame,
            speed_mps=speed_mps,
            longitudinal_acceleration_mps2=longitudinal_acceleration_mps2,
            lateral_offset_m=lateral_offset_m,
            lateral_velocity_mps=lateral_velocity_mps,
            lateral_acceleration_mps2=lateral_acceleration_mps2,
            heading_offset_rad=heading_offset_rad,
            visual_state=visual_state,
        )

    def sequence(
        self,
        driver_id: int | str,
        *,
        start_total_progress: float,
        progress_rate_per_s: float | None = None,
        real_duration_s: float | None = None,
        speed_multiplier: float = 1.0,
        lateral_offset_m: float = 0.0,
        heading_offset_rad: float = 0.0,
        visual_state: str = "normal",
        clock: LogicalClock | None = None,
        logical_start_s: float | None = None,
        logical_duration_s: float | None = None,
        plan: LongitudinalKinematicPlan | None = None,
        lateral_trajectory: LateralTrajectory | None = None,
        grid_slot: GridDisplaySlot | None = None,
        lights_out_time_s: float = GRID_HOLD_DURATION_S,
        launch_lateral_hold_s: float = GRID_LATERAL_HOLD_AFTER_LIGHTS_S,
        launch_lateral_duration_s: float = GRID_LATERAL_REJOIN_DURATION_S,
    ) -> tuple[AbstractPoseFrame, ...]:
        """Sample one logical interval; speed multiplier only schedules wall time.

        ``real_duration_s`` remains a compatibility form for Stage B callers:
        it describes wall duration and is multiplied by ``speed_multiplier``.
        New callers should provide ``logical_duration_s`` directly; in that
        form 1x/2x/5x produce the same logical frames.
        """

        if speed_multiplier <= 0.0:
            raise ValueError("speed multiplier must be positive")
        if logical_duration_s is None:
            if real_duration_s is None:
                raise ValueError("logical_duration_s or real_duration_s is required")
            if real_duration_s < 0.0:
                raise ValueError("duration must be non-negative")
            logical_duration_s = real_duration_s * speed_multiplier
        if logical_duration_s < 0.0:
            raise ValueError("logical duration must be non-negative")
        base_clock = clock or LogicalClock()
        if abs(base_clock.tick_seconds - KINEMATIC_LOGICAL_TICK_SECONDS) > 1e-9:
            raise ValueError("Stage 3 sequence requires a 0.10s logical tick")
        logical_start = base_clock.time_s if logical_start_s is None else float(logical_start_s)
        if logical_start < 0.0:
            raise ValueError("logical start must be non-negative")
        tick_count = ceil(logical_duration_s / KINEMATIC_LOGICAL_TICK_SECONDS)
        selected_plan = plan or self.build_plan(driver_id)
        if grid_slot is not None:
            current_distance = self.distance_contract.distance_at_total_progress(grid_slot.progress)
            current_speed = 0.0
            if lateral_trajectory is None:
                limits = self.distance_contract.lateral_limits_at_progress(
                    grid_slot.progress,
                    car_width_m=self.track.display_geometry.car_width_m,
                )
                lateral_trajectory = LateralTrajectory.create(
                    start_offset_m=grid_slot.lateral_offset_m,
                    end_offset_m=0.0,
                    start_time_s=(
                        logical_start + lights_out_time_s + launch_lateral_hold_s
                    ),
                    duration_s=launch_lateral_duration_s,
                    target_limits_m=limits,
                )
        else:
            current_distance = self.distance_contract.distance_at_total_progress(start_total_progress)
            current_speed = selected_plan.target_speed_at_line_distance(current_distance)
        if lateral_trajectory is None:
            lateral_trajectory = LateralTrajectory.create(
                start_offset_m=lateral_offset_m,
                end_offset_m=lateral_offset_m,
                start_time_s=logical_start,
                duration_s=0.0,
            )
        frames: list[AbstractPoseFrame] = []
        for tick in range(tick_count + 1):
            elapsed = min(tick * KINEMATIC_LOGICAL_TICK_SECONDS, logical_duration_s)
            trajectory_time = logical_start + elapsed
            lateral_offset = lateral_trajectory.offset_at(trajectory_time)
            lateral_velocity = lateral_trajectory.velocity_at(trajectory_time)
            lateral_acceleration = lateral_trajectory.acceleration_at(trajectory_time)
            if tick > 0:
                integrate = True
                if grid_slot is not None and elapsed <= lights_out_time_s + 1e-9:
                    integrate = False
                if integrate:
                    step = selected_plan.integrate(current_distance, current_speed)
                    current_distance = step.distance_m
                    current_speed = step.speed_mps
                    acceleration = step.longitudinal_acceleration_mps2
                else:
                    acceleration = 0.0
            else:
                acceleration = 0.0
            self.validate_lateral_offset(current_distance, lateral_offset)
            frame = self.pose_at_line_distance(
                driver_id,
                current_distance,
                simulation_time_s=logical_start + elapsed,
                physics_frame=base_clock.tick_index + tick,
                speed_mps=current_speed,
                longitudinal_acceleration_mps2=acceleration,
                lateral_offset_m=lateral_offset,
                lateral_velocity_mps=lateral_velocity,
                lateral_acceleration_mps2=lateral_acceleration,
                heading_offset_rad=heading_offset_rad,
                visual_state=visual_state,
                plan=selected_plan,
            )
            frames.append(frame)
        return tuple(frames)

    def logical_sequence(
        self,
        driver_id: int | str,
        *,
        start_logical_time_s: float,
        end_logical_time_s: float,
        start_total_progress: float = 0.0,
        speed_multiplier: float = 1.0,
        **kwargs: Any,
    ) -> tuple[AbstractPoseFrame, ...]:
        """Canonical logical-interval API shared by every wall-clock speed."""

        if end_logical_time_s < start_logical_time_s:
            raise ValueError("logical end time must not precede start time")
        return self.sequence(
            driver_id,
            start_total_progress=start_total_progress,
            logical_start_s=start_logical_time_s,
            logical_duration_s=end_logical_time_s - start_logical_time_s,
            speed_multiplier=speed_multiplier,
            **kwargs,
        )

    def grid_sequence(
        self,
        driver_id: int | str,
        *,
        grid_position: int,
        logical_duration_s: float = 3.0,
        speed_multiplier: float = 1.0,
        **kwargs: Any,
    ) -> tuple[AbstractPoseFrame, ...]:
        """Build one shared-grid hold and lights-out launch sequence."""

        geometry = self.track.display_geometry
        if geometry is None:
            raise ValueError("grid sequence requires shared display geometry")
        if not 1 <= grid_position <= len(geometry.grid_slots):
            raise ValueError("grid position is outside the shared grid")
        return self.sequence(
            driver_id,
            start_total_progress=0.0,
            logical_duration_s=logical_duration_s,
            speed_multiplier=speed_multiplier,
            grid_slot=geometry.grid_slots[grid_position - 1],
            **kwargs,
        )

    @staticmethod
    def sequence_hash(frames: tuple[AbstractPoseFrame, ...] | list[AbstractPoseFrame]) -> str:
        return pose_sequence_hash(frames)


class SingleProbeKinematicCursor:
    """Bounded, continuous one-car presenter for the broadcast path.

    The cursor owns only the current accepted state.  It never uses a logical
    checkpoint's progress as a presentation position and never materializes a
    full-race pose sequence.
    """

    def __init__(
        self,
        synthesizer: SingleVehiclePoseSynthesizer,
        driver_id: int | str,
        *,
        grid_position: int,
        lights_out_time_s: float = 1.0,
        launch_lateral_duration_s: float = 2.0,
    ) -> None:
        if lights_out_time_s < 0.0:
            raise ValueError("lights-out time must be non-negative")
        self.driver_id = driver_id
        self._synthesizer: SingleVehiclePoseSynthesizer | None = synthesizer
        self._plan: LongitudinalKinematicPlan | None = synthesizer.build_plan(driver_id)
        self._grid_slot = synthesizer.track.display_geometry.grid_slots[grid_position - 1]
        self._lights_out_tick = int(round(lights_out_time_s / KINEMATIC_LOGICAL_TICK_SECONDS))
        self._logical_tick_index = 0
        self._logical_time_s = 0.0
        self._line_distance_m = synthesizer.distance_contract.distance_at_total_progress(
            self._grid_slot.progress,
        )
        self._speed_mps = 0.0
        limits = synthesizer.distance_contract.lateral_limits_at_progress(
            self._grid_slot.progress,
            car_width_m=synthesizer.track.display_geometry.car_width_m,
        )
        self._lateral_trajectory: LateralTrajectory | None = LateralTrajectory.create(
            start_offset_m=self._grid_slot.lateral_offset_m,
            end_offset_m=0.0,
            start_time_s=lights_out_time_s,
            duration_s=launch_lateral_duration_s,
            target_limits_m=limits,
        )
        self._last_step = self._hold_step()
        self._current_pose: AbstractPoseFrame | None = self._build_pose(0.0, 0)
        self._disposed = False

    @property
    def logical_tick_index(self) -> int:
        return self._logical_tick_index

    @property
    def logical_time_s(self) -> float:
        return self._logical_time_s

    @property
    def line_distance_m(self) -> float:
        return self._line_distance_m

    @property
    def speed_mps(self) -> float:
        return self._speed_mps

    @property
    def last_step(self) -> KinematicStep:
        return self._last_step

    @property
    def grid_hold(self) -> bool:
        return self._logical_tick_index <= self._lights_out_tick

    @property
    def disposed(self) -> bool:
        return self._disposed

    def _hold_step(self) -> KinematicStep:
        return KinematicStep(
            distance_m=self._line_distance_m,
            speed_mps=0.0,
            longitudinal_acceleration_mps2=0.0,
            displacement_m=0.0,
            target_speed_mps=0.0,
            previous_distance_m=self._line_distance_m,
            previous_speed_mps=0.0,
            adjustment_reason="grid_hold",
        )

    def _build_pose(self, logical_time_s: float, physics_frame: int) -> AbstractPoseFrame:
        synthesizer = self._require_synthesizer()
        trajectory = self._lateral_trajectory
        lateral_offset = trajectory.offset_at(logical_time_s) if trajectory else 0.0
        lateral_velocity = trajectory.velocity_at(logical_time_s) if trajectory else 0.0
        lateral_acceleration = trajectory.acceleration_at(logical_time_s) if trajectory else 0.0
        synthesizer.validate_lateral_offset(self._line_distance_m, lateral_offset)
        return synthesizer.pose_at_line_distance(
            self.driver_id,
            self._line_distance_m,
            simulation_time_s=logical_time_s,
            physics_frame=physics_frame,
            speed_mps=self._speed_mps,
            longitudinal_acceleration_mps2=self._last_step.longitudinal_acceleration_mps2,
            lateral_offset_m=lateral_offset,
            lateral_velocity_mps=lateral_velocity,
            lateral_acceleration_mps2=lateral_acceleration,
            plan=self._require_plan(),
        )

    def _require_plan(self) -> LongitudinalKinematicPlan:
        if self._plan is None:
            raise RuntimeError("single probe cursor is disposed")
        return self._plan

    def _require_synthesizer(self) -> SingleVehiclePoseSynthesizer:
        if self._synthesizer is None:
            raise RuntimeError("single probe cursor is disposed")
        return self._synthesizer

    def current_pose(self) -> AbstractPoseFrame:
        """Return the current pose without advancing or appending a frame."""

        if self._current_pose is None:
            raise RuntimeError("single probe cursor is disposed")
        return self._current_pose

    def advance_one_tick(self) -> SingleProbeKinematicTick:
        """Advance exactly one 0.10s logical tick and return its accepted step."""

        if self._disposed:
            raise RuntimeError("single probe cursor is disposed")
        next_tick = self._logical_tick_index + 1
        next_time = next_tick * KINEMATIC_LOGICAL_TICK_SECONDS
        if next_tick <= self._lights_out_tick:
            step = self._hold_step()
        else:
            step = self._require_plan().integrate(self._line_distance_m, self._speed_mps)
        self._line_distance_m = step.distance_m
        self._speed_mps = step.speed_mps
        self._last_step = step
        self._logical_tick_index = next_tick
        self._logical_time_s = next_time
        self._current_pose = self._build_pose(next_time, next_tick)
        return SingleProbeKinematicTick(
            logical_tick_index=next_tick,
            logical_time_s=next_time,
            pose=self._current_pose,
            accepted_step=step,
            grid_hold=next_tick <= self._lights_out_tick,
        )

    def dispose(self) -> None:
        self._disposed = True
        self._current_pose = None
        self._last_step = None  # type: ignore[assignment]
        self._lateral_trajectory = None
        self._grid_slot = None  # type: ignore[assignment]
        self._plan = None
        self._synthesizer = None


KinematicPoseBuffer = BoundedKinematicPoseBuffer


PoseSynthesizer = SingleVehiclePoseSynthesizer
