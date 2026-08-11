"""Presentation-only adapter from progress authority to broadcast frames."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, floor, hypot, pi
from typing import Any

from simulation.track_display import TrackDisplayGeometry

from .pose import SingleVehiclePoseSynthesizer
from .progress_race import (
    ProgressRaceCursor,
    ProgressRaceSummary,
    ProgressRuntimeCheckpoint,
)
from .race import AbstractTrafficTick
from .state import (
    AbstractRaceFrame,
    AbstractSessionSnapshot,
    AbstractTimingCheckpoint,
    LogicalEvent,
    RaceVehicleState,
)


PROGRESS_BROADCAST_PRESENTATION_CONTRACT = "progress-v5-derived-pose-v2"
PROGRESS_PRESENTATION_MAX_SPEED_MPS = 370.0 / 3.6


@dataclass(frozen=True, slots=True)
class ProgressBroadcastRuntimeCheckpoint:
    progress: ProgressRuntimeCheckpoint
    current_frame: AbstractRaceFrame
    timing_checkpoint_count: int
    last_checkpoint_lap: int
    lateral_offsets_m: dict[int | str, float]
    previous_speeds_mps: dict[int | str, float]
    previous_world_poses: dict[int | str, tuple[float, float, float]]


def _sample_polyline(
    points: tuple[tuple[float, float], ...],
    progress: float,
) -> tuple[float, float, float]:
    if len(points) < 2:
        raise ValueError("pit presentation route requires at least two points")
    clamped = min(1.0, max(0.0, float(progress)))
    lengths = tuple(
        hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(points, points[1:])
    )
    total = sum(lengths)
    if total <= 1e-9:
        first, second = points[0], points[-1]
        return first[0], first[1], atan2(second[1] - first[1], second[0] - first[0])
    target = clamped * total
    traversed = 0.0
    for index, length in enumerate(lengths):
        if traversed + length >= target - 1e-12:
            ratio = (target - traversed) / max(length, 1e-9)
            first, second = points[index], points[index + 1]
            return (
                first[0] + (second[0] - first[0]) * ratio,
                first[1] + (second[1] - first[1]) * ratio,
                atan2(second[1] - first[1], second[0] - first[0]),
            )
        traversed += length
    first, second = points[-2], points[-1]
    return second[0], second[1], atan2(
        second[1] - first[1], second[0] - first[0]
    )


class ProgressBroadcastCursor:
    """Expose progress v5 through the existing bounded broadcast tick contract.

    Progress distance, rank, incidents, pit state and race control remain the
    only result authority.  World pose and lateral movement are deterministic
    presentation derivatives and never feed back into the progress cursor.
    """

    def __init__(
        self,
        snapshot: AbstractSessionSnapshot,
        *,
        grid_order: tuple[int | str, ...] | list[int | str] | None = None,
        total_laps: int = 10,
        tick_seconds: float = 0.10,
        display_geometry: TrackDisplayGeometry,
    ) -> None:
        self.snapshot = snapshot
        self.display_geometry = display_geometry
        self.progress_cursor = ProgressRaceCursor(
            snapshot,
            grid_order=grid_order,
            total_laps=total_laps,
            tick_seconds=tick_seconds,
        )
        self.synthesizer = SingleVehiclePoseSynthesizer(snapshot)
        self._segment_sector = {
            segment.segment_id: segment.sector_id for segment in snapshot.track.segments
        }
        coordinate_frame = display_geometry.coordinate_frame
        self._pit_route_m = tuple(
            coordinate_frame.to_local_m(x, y)
            for x, y in display_geometry.pit_lane_coords
        )
        self._lateral_offsets_m = {
            driver_id: 0.0 for driver_id in self.progress_cursor.grid_order
        }
        self._previous_speeds_mps = {
            driver_id: 0.0 for driver_id in self.progress_cursor.grid_order
        }
        self._previous_world_poses: dict[
            int | str, tuple[float, float, float]
        ] = {}
        self.timing_checkpoints: list[AbstractTimingCheckpoint] = [
            self._checkpoint("race_start", lap_number=0)
        ]
        self._last_checkpoint_lap = 0
        self._current_frame = self._build_frame()
        self._disposed = False

    @property
    def current_tick(self) -> int:
        return self.progress_cursor.current_tick.tick_index

    @property
    def current_frame(self) -> AbstractRaceFrame:
        return self._current_frame

    @property
    def events(self) -> list[LogicalEvent]:
        return self.progress_cursor.logical_events

    @property
    def metrics(self) -> dict[str, int]:
        return self.progress_cursor.metrics

    @property
    def finished(self) -> bool:
        return self.progress_cursor.finished

    @property
    def active_reservation_count(self) -> int:
        return 0

    @property
    def race_control_state(self) -> str:
        return self.progress_cursor.race_control_state

    @property
    def race_control_phase(self) -> str:
        return self.progress_cursor.race_control_phase

    @property
    def race_control_remaining_s(self) -> float:
        until = self.progress_cursor._race_control_phase_until_s
        if until < 0.0:
            return 0.0
        return max(0.0, until - self.progress_cursor.logical_time_s)

    def _checkpoint(
        self,
        checkpoint_kind: str,
        *,
        lap_number: int,
    ) -> AbstractTimingCheckpoint:
        tick = self.progress_cursor.current_tick
        order = tuple(vehicle.driver_id for vehicle in tick.vehicles)
        return AbstractTimingCheckpoint(
            checkpoint_id=(
                f"progress-broadcast:{checkpoint_kind}:{tick.tick_index}:"
                f"{lap_number}"
            ),
            checkpoint_kind=checkpoint_kind,
            tick_index=tick.tick_index,
            logical_time_s=tick.logical_time_s,
            lap_number=lap_number,
            order=order,
            progress_by_driver=tuple(
                (vehicle.driver_id, vehicle.total_progress)
                for vehicle in tick.vehicles
            ),
        )

    @staticmethod
    def _pair_side(first: int | str, second: int | str | None) -> float:
        token = f"{min(str(first), str(second))}:{max(str(first), str(second))}"
        return -1.0 if sum(ord(character) for character in token) % 2 else 1.0

    def _target_lateral_offset(self, vehicle) -> float:
        if vehicle.pit_state != "none" or vehicle.race_status != "running":
            return 0.0
        side = float(
            vehicle.maneuver_side
            or self._pair_side(vehicle.driver_id, vehicle.target_driver_id)
        )
        progress = min(1.0, max(0.0, vehicle.maneuver_progress))
        eased = progress * progress * (3.0 - 2.0 * progress)
        if (
            getattr(vehicle, "maneuver_group_size", 0) >= 3
            and getattr(vehicle, "maneuver_group_corridor_index", None) is not None
        ):
            corridor_target = (-2.10, 0.0, 2.10)[
                min(2, max(0, vehicle.maneuver_group_corridor_index))
            ]
            if getattr(vehicle, "maneuver_group_phase", None) == "approach":
                return corridor_target * eased
            if getattr(vehicle, "maneuver_group_phase", None) == "merge":
                return corridor_target * (1.0 - eased)
            return corridor_target
        if vehicle.traffic_state == "attack":
            return side * (0.25 + 1.55 * eased)
        if vehicle.traffic_state == "side_by_side":
            return side * 2.10
        if vehicle.traffic_state == "defend":
            return side * (0.45 + 0.75 * eased)
        if vehicle.traffic_state == "clearance":
            return side * 2.10 * (1.0 - eased)
        return 0.0

    def _presentation_pose(
        self,
        vehicle,
        *,
        lateral_offset_m: float,
        acceleration_mps2: float,
    ) -> tuple[float, float, float, float]:
        line_distance_m = self.synthesizer.distance_contract.distance_at_total_progress(
            vehicle.total_progress
        )
        if vehicle.pit_state != "none" and len(self._pit_route_m) >= 2:
            world_x_m, world_y_m, heading_rad = _sample_polyline(
                self._pit_route_m, vehicle.pit_lane_progress
            )
            return world_x_m, world_y_m, heading_rad, line_distance_m
        pose = self.synthesizer.pose_at_line_distance(
            vehicle.driver_id,
            line_distance_m,
            simulation_time_s=self.progress_cursor.current_tick.logical_time_s,
            physics_frame=self.progress_cursor.current_tick.tick_index,
            speed_mps=vehicle.logical_speed_mps,
            longitudinal_acceleration_mps2=acceleration_mps2,
            lateral_offset_m=lateral_offset_m,
            visual_state=(
                "stopped"
                if vehicle.race_status == "retired"
                else vehicle.incident_state
                or ("pit" if vehicle.pit_state != "none" else vehicle.traffic_state)
            ),
        )
        return pose.world_x_m, pose.world_y_m, pose.heading_rad, line_distance_m

    def _build_frame(self) -> AbstractRaceFrame:
        tick = self.progress_cursor.current_tick
        vehicles: list[RaceVehicleState] = []
        tick_seconds = self.progress_cursor.tick_seconds
        for vehicle in tick.vehicles:
            previous_offset = self._lateral_offsets_m.get(vehicle.driver_id, 0.0)
            target_offset = self._target_lateral_offset(vehicle)
            maximum_lateral_step_m = 1.0 * tick_seconds
            delta = min(
                maximum_lateral_step_m,
                max(-maximum_lateral_step_m, target_offset - previous_offset),
            )
            lateral_offset = previous_offset + delta
            self._lateral_offsets_m[vehicle.driver_id] = lateral_offset
            target_x_m, target_y_m, target_heading_rad, line_distance_m = self._presentation_pose(
                vehicle,
                lateral_offset_m=lateral_offset,
                acceleration_mps2=0.0,
            )
            previous_pose = self._previous_world_poses.get(vehicle.driver_id)
            if previous_pose is None:
                world_x_m = target_x_m
                world_y_m = target_y_m
                heading_rad = target_heading_rad
                presented_speed_mps = 0.0
            else:
                old_x_m, old_y_m, old_heading_rad = previous_pose
                delta_x_m = target_x_m - old_x_m
                delta_y_m = target_y_m - old_y_m
                target_distance_m = hypot(delta_x_m, delta_y_m)
                maximum_step_m = PROGRESS_PRESENTATION_MAX_SPEED_MPS * tick_seconds
                ratio = min(
                    1.0,
                    maximum_step_m / max(target_distance_m, 1e-12),
                )
                world_x_m = old_x_m + delta_x_m * ratio
                world_y_m = old_y_m + delta_y_m * ratio
                heading_delta = (
                    (target_heading_rad - old_heading_rad + pi) % (2.0 * pi)
                ) - pi
                heading_rad = old_heading_rad + heading_delta * ratio
                presented_speed_mps = (
                    hypot(world_x_m - old_x_m, world_y_m - old_y_m)
                    / max(tick_seconds, 1e-9)
                )
            self._previous_world_poses[vehicle.driver_id] = (
                world_x_m,
                world_y_m,
                heading_rad,
            )
            previous_speed = self._previous_speeds_mps.get(vehicle.driver_id, 0.0)
            acceleration = (presented_speed_mps - previous_speed) / max(
                tick_seconds, 1e-9
            )
            acceleration = min(18.0, max(-50.0, acceleration))
            self._previous_speeds_mps[vehicle.driver_id] = presented_speed_mps
            vehicles.append(
                RaceVehicleState(
                    driver_id=vehicle.driver_id,
                    vehicle_id=vehicle.vehicle_id,
                    position=vehicle.position,
                    total_progress=vehicle.total_progress,
                    progress=vehicle.total_progress % 1.0,
                    lap_number=vehicle.lap_number,
                    sector_id=self._segment_sector.get(vehicle.segment_id, "S1"),
                    progress_rate_per_s=(
                        vehicle.logical_speed_mps
                        / max(self.progress_cursor.lap_length_m, 1e-9)
                    ),
                    gap_to_ahead_m=vehicle.gap_to_ahead_m,
                    interval_to_ahead_s=vehicle.interval_to_ahead_s,
                    ahead_driver_id=vehicle.ahead_driver_id,
                    lateral_offset_m=lateral_offset,
                    visual_state=(
                        "stopped"
                        if vehicle.race_status == "retired"
                        else vehicle.incident_state
                        or ("pit" if vehicle.pit_state != "none" else vehicle.traffic_state)
                    ),
                    maneuver=vehicle.traffic_state,
                    world_x_m=world_x_m,
                    world_y_m=world_y_m,
                    heading_rad=heading_rad,
                    gap_to_leader_s=vehicle.gap_to_leader_s,
                    timing_gap_valid=vehicle.timing_gap_valid,
                    interval_timing_gap_valid=vehicle.interval_timing_gap_valid,
                    drs_active=vehicle.drs_active,
                    dirty_air_active=vehicle.dirty_air_active,
                    dirty_air_strength=vehicle.dirty_air_strength,
                    tow_strength=vehicle.tow_strength,
                    attack_mode=vehicle.attack_mode,
                    maneuver_side=vehicle.maneuver_side,
                    maneuver_progress=vehicle.maneuver_progress,
                    drs_train_id=vehicle.drs_train_id,
                    drs_train_size=vehicle.drs_train_size,
                    drs_train_position=vehicle.drs_train_position,
                    drs_train_member_ids=vehicle.drs_train_member_ids,
                    maneuver_group_id=vehicle.maneuver_group_id,
                    maneuver_group_size=vehicle.maneuver_group_size,
                    maneuver_group_member_ids=vehicle.maneuver_group_member_ids,
                    maneuver_group_phase=vehicle.maneuver_group_phase,
                    maneuver_group_corridor_index=(
                        vehicle.maneuver_group_corridor_index
                    ),
                    physical_compound=vehicle.physical_compound,
                    tire_role=vehicle.tire_role,
                    stint_lap=vehicle.stint_lap,
                    wear_laps=vehicle.wear_laps,
                    temperature_band=vehicle.temperature_band,
                    pit_state=vehicle.pit_state,
                    pit_lane_progress=vehicle.pit_lane_progress,
                    pit_stop_count=vehicle.pit_stop_count,
                    pit_request_pending=vehicle.pit_request_pending,
                    pit_request_role=vehicle.pit_request_role,
                    pit_request_compound=vehicle.pit_request_compound,
                    pace_mode=vehicle.pace_mode,
                    damage_level=vehicle.damage_level,
                    incident_state=vehicle.incident_state,
                    reliability_state=(
                        "failed" if vehicle.race_status == "retired" else "nominal"
                    ),
                    line_distance_m=line_distance_m,
                    speed_mps=presented_speed_mps,
                    longitudinal_acceleration_mps2=acceleration,
                    lateral_velocity_mps=delta / max(tick_seconds, 1e-9),
                    lateral_acceleration_mps2=0.0,
                )
            )
        return AbstractRaceFrame(
            tick_index=tick.tick_index,
            logical_time_s=tick.logical_time_s,
            vehicles=tuple(vehicles),
        )

    def advance_one_tick(self) -> AbstractTrafficTick:
        if self._disposed:
            raise RuntimeError("progress broadcast cursor is disposed")
        event_count = len(self.progress_cursor.logical_events)
        tick = self.progress_cursor.advance_one_tick()
        self._current_frame = self._build_frame()
        checkpoints: list[AbstractTimingCheckpoint] = []
        completed_lap = max(
            0,
            floor(max((vehicle.total_progress for vehicle in tick.vehicles), default=0.0)),
        )
        if completed_lap > self._last_checkpoint_lap and not self.finished:
            self._last_checkpoint_lap = completed_lap
            checkpoint = self._checkpoint("leader_lap", lap_number=completed_lap)
            self.timing_checkpoints.append(checkpoint)
            checkpoints.append(checkpoint)
        if self.finished:
            checkpoint = self._checkpoint("race_finish", lap_number=self.progress_cursor.total_laps)
            self.timing_checkpoints.append(checkpoint)
            checkpoints.append(checkpoint)
        return AbstractTrafficTick(
            frame=self._current_frame,
            logical_events=tuple(self.progress_cursor.logical_events[event_count:]),
            checkpoints=tuple(checkpoints),
        )

    def finalize(self) -> ProgressRaceSummary:
        if not self.finished:
            raise RuntimeError("progress broadcast cannot finalize before finish")
        return self.progress_cursor.run_to_finish()

    def export_runtime_checkpoint(self) -> ProgressBroadcastRuntimeCheckpoint:
        return ProgressBroadcastRuntimeCheckpoint(
            progress=self.progress_cursor.capture_runtime_checkpoint(),
            current_frame=self._current_frame,
            timing_checkpoint_count=len(self.timing_checkpoints),
            last_checkpoint_lap=self._last_checkpoint_lap,
            lateral_offsets_m=dict(self._lateral_offsets_m),
            previous_speeds_mps=dict(self._previous_speeds_mps),
            previous_world_poses=dict(self._previous_world_poses),
        )

    def restore_runtime_checkpoint(
        self,
        checkpoint: ProgressBroadcastRuntimeCheckpoint,
    ) -> None:
        self.progress_cursor.restore_runtime_checkpoint(checkpoint.progress)
        del self.timing_checkpoints[checkpoint.timing_checkpoint_count :]
        self._last_checkpoint_lap = checkpoint.last_checkpoint_lap
        self._lateral_offsets_m = dict(checkpoint.lateral_offsets_m)
        self._previous_speeds_mps = dict(checkpoint.previous_speeds_mps)
        self._previous_world_poses = dict(checkpoint.previous_world_poses)
        self._current_frame = checkpoint.current_frame

    def set_pace_mode(self, driver_id: int | str, pace_mode: str) -> LogicalEvent:
        event = self.progress_cursor.set_pace_mode(driver_id, pace_mode)
        self._current_frame = self._build_frame()
        return event

    def request_pit(
        self,
        driver_id: int | str,
        *,
        tire_role: str,
        physical_compound: str,
    ) -> LogicalEvent:
        event = self.progress_cursor.request_pit(
            driver_id,
            tire_role=tire_role,
            physical_compound=physical_compound,
        )
        self._current_frame = self._build_frame()
        return event

    def cancel_pit(self, driver_id: int | str) -> LogicalEvent:
        event = self.progress_cursor.cancel_pit(driver_id)
        self._current_frame = self._build_frame()
        return event

    def dispose(self) -> None:
        self._disposed = True
        self.timing_checkpoints.clear()
        self._lateral_offsets_m.clear()
        self._previous_speeds_mps.clear()
        self._previous_world_poses.clear()
