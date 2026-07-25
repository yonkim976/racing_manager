"""Pit-lane route, box, merge and stop lifecycle extracted from RaceEngine.

RaceEngine inherits PitOpsMixin so existing call sites and tests keep the same
method names.  Timing helpers remain in ``pit_stop.py``; this module owns the
runtime pit-ops domain.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin, sqrt
from typing import Any

from models.schemas import DriverRaceState, RaceEvent, Team, TireCompound
from simulation.collision import BodyPose, oriented_body_overlap
from simulation.pit_stop import compute_pit_components
from simulation.state_contract import TickPhase
from simulation.tire_model import tire_blanket_temperature_c
from simulation.track_physics import DRIVING_LINE_RACING, TrackPhysicsProfile
from simulation.vehicle_physics import PHYSICS_STEP_SECONDS

PROGRESS_EPSILON = 1e-9

PIT_LANE_SPEED_LIMIT_KPH = 80.0
PIT_ENTRY_BRAKING_MPS2 = 35.0
PIT_BOX_BRAKING_MPS2 = 10.0
PIT_LANE_ACCELERATION_MPS2 = 5.0
PIT_EXIT_ACCELERATION_MPS2 = 9.0
PIT_TEAM_BOX_SPACING_M = 16.0
PIT_BOX_LATERAL_BLEND_DISTANCE_M = 22.0
PIT_MERGE_BRAKING_MPS2 = 7.0
PIT_MERGE_HOLD_DISTANCE_M = 4.0
PIT_MERGE_LONGITUDINAL_MARGIN_M = 7.0
PIT_MERGE_LATERAL_MARGIN_M = 0.60
PIT_MERGE_GROUP_CLEARANCE_SECONDS = 0.40
PIT_ROUTE_ANCHOR_MERGE_DISTANCE_M = 5.0
PIT_ROUTE_TANGENT_LEAD_M = 8.0


@dataclass(frozen=True)
class PitMergeDecision:
    """Main-track priority result for one predicted pit-exit occupancy."""

    state: str
    conflict_driver_id: int | None = None
    conflict_group_id: str | None = None
    conflict_group_member_ids: tuple[int, ...] = ()


class PitOpsMixin:
    """Pit route geometry, box assignment, lane integration and merge."""

    def _init_pit_ops_state(self) -> None:
        """Initialize per-driver pit-lane runtime state on the host engine."""
        self._pit_phase: dict[int, str] = {}  # "in" | "stop" | "out"
        self._pit_phase_remaining: dict[int, float] = {}
        self._pit_phase_duration: dict[int, float] = {}
        self._pit_lane_half_time: dict[int, float] = {}
        self._pit_route_progress: dict[int, float] = {}
        self._pit_route_speed_mps: dict[int, float] = {}
        self._pit_merge_state: dict[int, str] = {}
        self._pit_merge_conflict_driver_id: dict[int, int | None] = {}
        self._pit_merge_conflict_group_id: dict[int, str | None] = {}
        self._pit_merge_conflict_group_member_ids: dict[int, tuple[int, ...]] = {}
        self._pit_tire_change_time: dict[int, float] = {}
        self._pit_elapsed: dict[int, float] = {}
        self._pit_stop_elapsed: dict[int, float] = {}
        self._pit_tire: dict[int, TireCompound] = {}
        self._pit_entry_compound: dict[int, TireCompound] = {}

    def _pit_route_points_m(
        self,
        track_profile: TrackPhysicsProfile | None = None,
    ) -> list[tuple[float, float]]:
        """Return the compiled pit route in the immutable local metre frame."""
        track_profile = track_profile or self._track_physics
        entry = self._pit_entry_progress()
        exit_ = self._pit_exit_progress()
        coordinate_frame = track_profile.coordinate_frame
        if (
            entry is None
            or exit_ is None
            or coordinate_frame is None
            or len(self.circuit.pit_lane_coords) < 2
        ):
            return []

        entry_x, entry_y, entry_heading = track_profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            entry,
        )
        exit_x, exit_y, exit_heading = track_profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            exit_,
        )
        interior_points = [
            coordinate_frame.to_local_m(x, y)
            for x, y in self.circuit.pit_lane_coords
        ]
        # Imported pit paths commonly repeat each track anchor with a point
        # only centimetres away in the lateral direction. Keeping that point
        # creates an artificial 90-degree tangent on pit-in/pit-out even
        # though the remaining route is correctly aligned. Collapse the
        # near-anchor samples and use the first meaningful route segment.
        while interior_points and hypot(
            interior_points[0][0] - entry_x,
            interior_points[0][1] - entry_y,
        ) < PIT_ROUTE_ANCHOR_MERGE_DISTANCE_M:
            interior_points.pop(0)
        while interior_points and hypot(
            interior_points[-1][0] - exit_x,
            interior_points[-1][1] - exit_y,
        ) < PIT_ROUTE_ANCHOR_MERGE_DISTANCE_M:
            interior_points.pop()

        points = [(entry_x, entry_y)]
        if interior_points and hypot(
            interior_points[0][0] - entry_x,
            interior_points[0][1] - entry_y,
        ) > 2.0 * PIT_ROUTE_TANGENT_LEAD_M:
            points.append(
                (
                    entry_x + cos(entry_heading) * PIT_ROUTE_TANGENT_LEAD_M,
                    entry_y + sin(entry_heading) * PIT_ROUTE_TANGENT_LEAD_M,
                )
            )
        points.extend(interior_points)
        if interior_points and hypot(
            interior_points[-1][0] - exit_x,
            interior_points[-1][1] - exit_y,
        ) > 2.0 * PIT_ROUTE_TANGENT_LEAD_M:
            points.append(
                (
                    exit_x - cos(exit_heading) * PIT_ROUTE_TANGENT_LEAD_M,
                    exit_y - sin(exit_heading) * PIT_ROUTE_TANGENT_LEAD_M,
                )
            )
        points.append((exit_x, exit_y))

        deduplicated = [points[0]]
        for point in points[1:]:
            if hypot(point[0] - deduplicated[-1][0], point[1] - deduplicated[-1][1]) > 1e-9:
                deduplicated.append(point)
        return deduplicated

    def _pit_route_length_m(
        self,
        track_profile: TrackPhysicsProfile | None = None,
    ) -> float:
        points = self._pit_route_points_m(track_profile)
        return sum(
            hypot(next_point[0] - point[0], next_point[1] - point[1])
            for point, next_point in zip(points, points[1:])
        )

    def get_pit_route_coords(self) -> list[list[float]]:
        """Expose the exact route used by pit physics in circuit coordinates."""
        coordinate_frame = self._track_physics.coordinate_frame
        if coordinate_frame is None:
            return [list(point) for point in self.circuit.pit_lane_coords]
        return [
            list(coordinate_frame.from_local_m(x_m, y_m))
            for x_m, y_m in self._pit_route_points_m()
        ]

    def _pit_lane_pose_at_progress_m(
        self,
        lane_progress: float,
        track_profile: TrackPhysicsProfile | None = None,
    ) -> tuple[float, float, float]:
        """Interpolate the authoritative pit-lane pose by route distance."""
        track_profile = track_profile or self._track_physics
        points = self._pit_route_points_m(track_profile)
        entry = self._pit_entry_progress()
        exit_ = self._pit_exit_progress()
        if not points or entry is None or exit_ is None:
            return track_profile.line_pose_at_progress_m(
                DRIVING_LINE_RACING,
                entry or 0.0,
            )

        clamped = min(1.0, max(0.0, lane_progress))
        if clamped <= 0.0:
            x, y, heading = track_profile.line_pose_at_progress_m(
                DRIVING_LINE_RACING,
                entry,
            )
            return x, y, heading
        if clamped >= 1.0:
            x, y, heading = track_profile.line_pose_at_progress_m(
                DRIVING_LINE_RACING,
                exit_,
            )
            return x, y, heading

        segment_lengths = [
            hypot(next_point[0] - point[0], next_point[1] - point[1])
            for point, next_point in zip(points, points[1:])
        ]
        total_length = sum(segment_lengths)
        if total_length <= 1e-9:
            return points[0][0], points[0][1], 0.0
        target_distance = clamped * total_length
        traversed = 0.0
        for index, segment_length in enumerate(segment_lengths):
            if traversed + segment_length >= target_distance:
                ratio = (target_distance - traversed) / max(segment_length, 1e-9)
                point = points[index]
                next_point = points[index + 1]
                heading = atan2(
                    next_point[1] - point[1],
                    next_point[0] - point[0],
                )
                return (
                    point[0] + (next_point[0] - point[0]) * ratio,
                    point[1] + (next_point[1] - point[1]) * ratio,
                    heading,
                )
            traversed += segment_length
        point = points[-2]
        next_point = points[-1]
        return point[0], point[1], atan2(
            next_point[1] - point[1],
            next_point[0] - point[0],
        )

    def _pit_box_progress_for_team(self, team_id: int) -> float:
        """Return one stable longitudinal pit-box position per constructor."""
        pit_lane = self.circuit.pit_lane
        route_length_m = self._pit_route_length_m()
        if pit_lane is None or route_length_m <= 1e-9:
            return 0.5
        team_ids = sorted({driver.team_id for driver in self.drivers.values()})
        if team_id not in team_ids or len(team_ids) <= 1:
            return pit_lane.box_progress
        team_index = team_ids.index(team_id)
        centered_index = team_index - (len(team_ids) - 1) / 2.0
        assigned = (
            pit_lane.box_progress
            + centered_index * PIT_TEAM_BOX_SPACING_M / route_length_m
        )
        operational_margin = max(
            0.01,
            (PIT_BOX_LATERAL_BLEND_DISTANCE_M + 4.0) / route_length_m,
        )
        return min(
            pit_lane.speed_limit_end - operational_margin,
            max(pit_lane.speed_limit_start + operational_margin, assigned),
        )

    def _pit_box_progress_for_driver(self, driver_id: int) -> float:
        driver = self.drivers.get(driver_id)
        if driver is None:
            pit_lane = self.circuit.pit_lane
            return pit_lane.box_progress if pit_lane is not None else 0.5
        return self._pit_box_progress_for_team(driver.team_id)

    def _pit_box_lateral_offset_m(
        self,
        driver_id: int,
        lane_progress: float,
    ) -> float:
        """Blend from the pit travel lane into the assigned garage-front box."""
        pit_lane = self.circuit.pit_lane
        route_length_m = self._pit_route_length_m()
        if pit_lane is None or route_length_m <= 1e-9:
            return 0.0
        stall_progress = self._pit_box_progress_for_driver(driver_id)
        blend_progress = PIT_BOX_LATERAL_BLEND_DISTANCE_M / route_length_m
        distance = abs(lane_progress - stall_progress)
        if distance >= blend_progress:
            return 0.0
        normalized = 1.0 - distance / max(1e-9, blend_progress)
        smooth = normalized * normalized * (3.0 - 2.0 * normalized)
        return pit_lane.box_offset * smooth

    def _pit_vehicle_pose_at_progress_m(
        self,
        driver_id: int,
        lane_progress: float,
        track_profile: TrackPhysicsProfile | None = None,
    ) -> tuple[float, float, float]:
        """Return the pit vehicle pose including the garage-front lateral path."""
        track_profile = track_profile or self._track_physics
        route_length_m = self._pit_route_length_m(track_profile)

        def offset_point(progress: float) -> tuple[float, float]:
            x, y, heading = self._pit_lane_pose_at_progress_m(
                progress,
                track_profile,
            )
            offset_m = self._pit_box_lateral_offset_m(driver_id, progress)
            return (
                x - sin(heading) * offset_m,
                y + cos(heading) * offset_m,
            )

        clamped = min(1.0, max(0.0, lane_progress))
        x, y = offset_point(clamped)
        sample_progress = max(1e-6, 0.30 / max(1.0, route_length_m))
        before = max(0.0, clamped - sample_progress)
        after = min(1.0, clamped + sample_progress)
        before_x, before_y = offset_point(before)
        after_x, after_y = offset_point(after)
        if after - before <= 1e-12:
            _, _, heading = self._pit_lane_pose_at_progress_m(
                clamped,
                track_profile,
            )
        else:
            heading = atan2(after_y - before_y, after_x - before_x)
        return x, y, heading

    def _pit_phase_speed_kph(self, driver_id: int) -> float:
        return self._pit_route_speed_mps.get(driver_id, 0.0) * 3.6

    def request_pit(self, driver_id: int, tire_choice: TireCompound) -> str | None:
        """Register a pit request. Returns error message or None."""
        state = self.driver_states.get(driver_id)
        if state is None:
            return "Unknown driver"
        if state.retired:
            return "Driver has retired"
        if state.in_pit:
            return "Driver already in pit"
        if state.pit_request is not None:
            return "Pit already requested"
        if driver_id not in self.player_driver_ids:
            return "Not a player driver"
        state.pit_request = tire_choice
        return None

    def _pit_entry_progress(self) -> float | None:
        pit_lane = self.circuit.pit_lane
        if pit_lane is None or pit_lane.entry_progress is None:
            return None
        return pit_lane.entry_progress % 1.0

    def _pit_exit_progress(self) -> float | None:
        pit_lane = self.circuit.pit_lane
        if pit_lane is None or pit_lane.exit_progress is None:
            return None
        return pit_lane.exit_progress % 1.0

    def _has_pit_progress_anchors(self) -> bool:
        return self._pit_entry_progress() is not None and self._pit_exit_progress() is not None

    def _should_enter_pit(
        self,
        state: DriverRaceState,
        previous_progress: float,
        progress_delta: float,
    ) -> bool:
        entry = self._pit_entry_progress()
        if entry is None or state.pit_request is None:
            return False
        return self._crossed_progress(previous_progress, progress_delta, entry)

    def _start_pit_stop(
        self,
        driver_id: int,
        state: DriverRaceState,
        meta: dict,
        team: Team,
        tire: TireCompound,
        events: list[RaceEvent],
    ) -> None:
        self._require_tick_phase(TickPhase.RULES, "pit stop state transition")
        lane_time, tire_change_time = compute_pit_components(
            self.circuit.pit_loss_time,
            team.pit_crew_skill,
            self.rng,
        )
        half_lane = lane_time / 2.0
        # A car leaving the racing surface cannot remain in an on-track
        # maneuver graph; stale edges would override live pit ordering.
        self._clear_side_by_side_for_driver(driver_id)
        self._pending_overtake_commands.pop(driver_id, None)
        if self.race_phase == "sc":
            self._remove_from_sc_running_order(driver_id)
        state.in_pit = True
        self._pit_phase[driver_id] = "in"
        self._pit_phase_duration[driver_id] = 0.0
        self._pit_phase_remaining[driver_id] = 0.0
        self._pit_lane_half_time[driver_id] = half_lane
        self._pit_route_progress[driver_id] = 0.0
        self._pit_route_speed_mps[driver_id] = max(0.0, state.speed_kph / 3.6)
        self._pit_merge_state[driver_id] = "approach"
        self._pit_merge_conflict_driver_id[driver_id] = None
        self._pit_merge_conflict_group_id[driver_id] = None
        self._pit_merge_conflict_group_member_ids[driver_id] = ()
        self._pit_tire_change_time[driver_id] = tire_change_time
        self._pit_elapsed[driver_id] = 0.0
        self._pit_stop_elapsed[driver_id] = 0.0
        self._pit_tire[driver_id] = tire
        self._pit_entry_compound[driver_id] = state.tire_compound
        state.grip_utilization = 0.0
        state.handling_state = "stable"
        state.slip_angle_rad = 0.0
        state.wheel_lock_ratio = 0.0
        state.traction_slip_ratio = 0.0
        state.tire_slide_energy_j = 0.0
        events.append(
            RaceEvent(
                type="pit_entry",
                driver=meta["abbreviation"],
                message=f"{meta['full_name']} pits for {tire.value} tires",
                message_ko=f"{meta['full_name']}가 {tire.value} 타이어로 교체하기 위해 피트인합니다",
            )
        )

    def _tick_in_pit(
        self,
        driver_id: int,
        state: DriverRaceState,
        meta: dict,
        delta: float,
        events: list[RaceEvent],
    ) -> None:
        """Integrate the car along the physical pit route by distance and speed."""
        state.total_time += delta
        self._pit_elapsed[driver_id] += delta
        phase = self._pit_phase[driver_id]
        if phase == "stop":
            self._pit_merge_state[driver_id] = "stop"
            self._pit_stop_elapsed[driver_id] += delta
            self._pit_phase_remaining[driver_id] = max(
                0.0,
                self._pit_phase_remaining[driver_id] - delta,
            )
            state.speed_kph = 0.0
            state.acceleration_mps2 = 0.0
            state.throttle = 0.0
            state.brake = 1.0
            if self._pit_phase_remaining[driver_id] <= 0.0:
                new_compound = self._pit_tire[driver_id]
                state.tire_compound = new_compound
                state.tire_age = 0
                state.tire_usage = 0.0
                state.tire_wear = 0.0
                blanket_temperature_c = tire_blanket_temperature_c(new_compound)
                state.tire_surface_temperature_c = blanket_temperature_c
                state.tire_core_temperature_c = blanket_temperature_c
                state.tire_thermal_grip = 1.0
                self._pit_phase[driver_id] = "out"
                self._pit_phase_duration[driver_id] = 0.0
                self._pit_route_speed_mps[driver_id] = 0.0
            self._sync_pit_race_progress(driver_id, state)
            return

        pit_lane = self.circuit.pit_lane
        route_length_m = self._pit_route_length_m()
        if pit_lane is None or route_length_m <= 1e-6:
            return

        progress = self._pit_route_progress.get(driver_id, 0.0)
        speed_mps = self._pit_route_speed_mps.get(driver_id, state.speed_kph / 3.6)
        pit_speed_limit_kph = pit_lane.speed_limit_kph
        limit_mps = pit_speed_limit_kph / 3.6
        assigned_box_progress = self._pit_box_progress_for_driver(driver_id)
        acceleration_mps2 = 0.0
        landmark_progress: float
        landmark_speed_mps: float

        if phase == "in" and progress < pit_lane.speed_limit_start:
            self._pit_merge_state[driver_id] = "approach"
            landmark_progress = pit_lane.speed_limit_start
            landmark_speed_mps = limit_mps
            distance_m = max(1e-6, (landmark_progress - progress) * route_length_m)
            required_acceleration_mps2 = (
                landmark_speed_mps**2 - speed_mps**2
            ) / (2.0 * distance_m)
            # Use the kinematic acceleration required to arrive at the line at
            # exactly the event speed limit. The old bang-bang controller always applied
            # at least 7 m/s² of braking, reached the limit too early, then
            # oscillated around the configured limit before the line.
            acceleration_mps2 = max(
                -PIT_ENTRY_BRAKING_MPS2,
                min(PIT_LANE_ACCELERATION_MPS2, required_acceleration_mps2),
            )
        elif phase == "in":
            self._pit_merge_state[driver_id] = "limited"
            landmark_progress = assigned_box_progress
            landmark_speed_mps = 0.0
            distance_m = max(0.0, (landmark_progress - progress) * route_length_m)
            braking_envelope = sqrt(2.0 * PIT_BOX_BRAKING_MPS2 * distance_m)
            desired_speed_mps = min(limit_mps, braking_envelope)
            if speed_mps > desired_speed_mps + 1e-6:
                acceleration_mps2 = -PIT_BOX_BRAKING_MPS2
            elif speed_mps < desired_speed_mps - 1e-6:
                acceleration_mps2 = PIT_LANE_ACCELERATION_MPS2
        elif progress < pit_lane.speed_limit_end:
            self._pit_merge_state[driver_id] = "limited"
            landmark_progress = pit_lane.speed_limit_end
            landmark_speed_mps = limit_mps
            if speed_mps < limit_mps - 1e-6:
                acceleration_mps2 = PIT_LANE_ACCELERATION_MPS2
            elif speed_mps > limit_mps + 1e-6:
                acceleration_mps2 = -PIT_BOX_BRAKING_MPS2
        else:
            previous_merge_state = self._pit_merge_state.get(driver_id)
            merge_decision = self._pit_rejoin_decision(
                driver_id,
                state,
                progress,
                speed_mps,
            )
            merge_state = merge_decision.state
            conflict_driver_id = merge_decision.conflict_driver_id
            self._pit_merge_state[driver_id] = merge_state
            self._pit_merge_conflict_driver_id[driver_id] = conflict_driver_id
            self._pit_merge_conflict_group_id[driver_id] = (
                merge_decision.conflict_group_id
            )
            self._pit_merge_conflict_group_member_ids[driver_id] = (
                merge_decision.conflict_group_member_ids
            )
            if (
                merge_state in {"yield", "hold"}
                and merge_state != previous_merge_state
            ):
                conflict_name = (
                    self._driver_meta[conflict_driver_id]["abbreviation"]
                    if conflict_driver_id in self._driver_meta
                    else "traffic"
                )
                events.append(
                    RaceEvent(
                        type=f"pit_merge_{merge_state}",
                        driver=meta["abbreviation"],
                        message=(
                            f"{meta['full_name']} {merge_state}s for {conflict_name} "
                            "at pit exit"
                        ),
                        message_ko=(
                            f"{meta['full_name']}가 피트 출구에서 {conflict_name} 차량에 "
                            f"{('양보합니다' if merge_state == 'yield' else '대기합니다')}"
                        ),
                        payload={
                            "driver_id": driver_id,
                            "conflict_driver_id": conflict_driver_id or 0,
                            "conflict_group_id": (
                                merge_decision.conflict_group_id or ""
                            ),
                            "conflict_group_member_ids": ",".join(
                                str(item)
                                for item in merge_decision.conflict_group_member_ids
                            ),
                            "merge_state": merge_state,
                        },
                    )
                )
            if merge_state == "merge":
                landmark_progress = pit_lane.side_rejoin_progress
                landmark_speed_mps = speed_mps
                acceleration_mps2 = PIT_EXIT_ACCELERATION_MPS2
            else:
                hold_progress = max(
                    progress,
                    pit_lane.side_rejoin_progress
                    - PIT_MERGE_HOLD_DISTANCE_M / route_length_m,
                )
                landmark_progress = hold_progress
                landmark_speed_mps = 0.0
                distance_m = max(0.0, (hold_progress - progress) * route_length_m)
                braking_envelope = sqrt(
                    2.0 * PIT_MERGE_BRAKING_MPS2 * distance_m
                )
                if speed_mps > braking_envelope + 1e-6:
                    acceleration_mps2 = -PIT_MERGE_BRAKING_MPS2
                elif distance_m <= 1e-6:
                    acceleration_mps2 = -PIT_MERGE_BRAKING_MPS2

        next_speed_mps = max(0.0, speed_mps + acceleration_mps2 * delta)
        if phase == "in" and progress >= pit_lane.speed_limit_start:
            next_speed_mps = min(limit_mps, next_speed_mps)
        elif phase == "out" and progress < pit_lane.speed_limit_end:
            next_speed_mps = min(limit_mps, next_speed_mps)
        distance_step_m = max(0.0, 0.5 * (speed_mps + next_speed_mps) * delta)
        next_progress = progress + distance_step_m / route_length_m

        if next_progress >= landmark_progress - 1e-12:
            next_progress = landmark_progress
            next_speed_mps = landmark_speed_mps
            if phase == "in" and landmark_progress == pit_lane.speed_limit_start:
                next_speed_mps = limit_mps
            elif phase == "in" and landmark_progress == assigned_box_progress:
                next_speed_mps = 0.0
                self._pit_phase[driver_id] = "stop"
                duration = self._pit_tire_change_time[driver_id]
                self._pit_phase_duration[driver_id] = duration
                self._pit_phase_remaining[driver_id] = duration
            elif phase == "out" and landmark_progress == pit_lane.speed_limit_end:
                next_speed_mps = limit_mps

        self._pit_route_progress[driver_id] = next_progress
        self._pit_route_speed_mps[driver_id] = next_speed_mps
        state.speed_kph = next_speed_mps * 3.6
        state.acceleration_mps2 = acceleration_mps2
        state.target_speed_kph = (
            pit_speed_limit_kph
            if next_progress <= pit_lane.speed_limit_end
            else state.speed_kph
        )
        state.throttle = 1.0 if acceleration_mps2 > 0.0 else 0.0
        state.brake = min(1.0, max(0.0, -acceleration_mps2 / PIT_ENTRY_BRAKING_MPS2))
        self._sync_pit_race_progress(driver_id, state)

        if phase == "out" and next_progress >= pit_lane.side_rejoin_progress:
            self._finish_pit_rejoin(driver_id, state, meta, events)

    def _pit_rejoin_decision(
        self,
        driver_id: int,
        state: DriverRaceState,
        lane_progress: float,
        speed_mps: float,
    ) -> PitMergeDecision:
        """Choose MERGE, YIELD, or HOLD with main-track group priority."""
        pit_lane = self.circuit.pit_lane
        entry = self._pit_entry_progress()
        exit_ = self._pit_exit_progress()
        route_length_m = self._pit_route_length_m()
        if pit_lane is None or entry is None or exit_ is None or route_length_m <= 1e-6:
            return PitMergeDecision("merge")

        rejoin_progress = pit_lane.side_rejoin_progress
        remaining_m = max(0.0, (rejoin_progress - lane_progress) * route_length_m)
        if speed_mps > 1.0:
            time_to_rejoin_s = remaining_m / speed_mps
        else:
            time_to_rejoin_s = sqrt(
                2.0 * remaining_m / max(0.1, PIT_EXIT_ACCELERATION_MPS2)
            )
        time_to_rejoin_s = min(5.0, max(PHYSICS_STEP_SECONDS, time_to_rejoin_s))

        mapped_total_progress = (
            state.current_lap
            + entry
            + self._progress_distance(entry, exit_) * rejoin_progress
        )
        track_profile = self._track_physics_for_driver(state)
        track_progress = mapped_total_progress % 1.0
        pit_x, pit_y, pit_heading = self._pit_lane_pose_at_progress_m(
            rejoin_progress,
            track_profile,
        )
        line_length_m = max(1.0, track_profile.length_for_line(DRIVING_LINE_RACING))
        for _ in range(4):
            line_x, line_y, line_heading = track_profile.line_pose_at_progress_m(
                DRIVING_LINE_RACING,
                track_progress,
            )
            tangent_delta_m = (
                (pit_x - line_x) * cos(line_heading)
                + (pit_y - line_y) * sin(line_heading)
            )
            track_progress = (track_progress + tangent_delta_m / line_length_m) % 1.0
        mapped_fraction = mapped_total_progress % 1.0
        refinement_delta = (
            track_progress - mapped_fraction + 0.5
        ) % 1.0 - 0.5
        target_total_progress = mapped_total_progress + refinement_delta
        line_x, line_y, line_heading = track_profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            track_progress,
        )
        lateral_offset_m = track_profile.line_offset_at_progress(
            DRIVING_LINE_RACING,
            track_progress,
        ) + (
            (pit_x - line_x) * -sin(line_heading)
            + (pit_y - line_y) * cos(line_heading)
        )
        pit_pose = BodyPose(
            longitudinal_m=target_total_progress * self.track_length_m,
            lateral_m=lateral_offset_m,
            heading_rad=(pit_heading - line_heading + pi) % (2.0 * pi) - pi,
            length_m=state.car_length_m + 2.0 * PIT_MERGE_LONGITUDINAL_MARGIN_M,
            width_m=state.car_width_m + 2.0 * PIT_MERGE_LATERAL_MARGIN_M,
            longitudinal_speed_mps=max(0.0, speed_mps),
        )

        conflicts: list[tuple[float, DriverRaceState, Any]] = []
        for other in self.driver_states.values():
            if (
                other.driver_id == driver_id
                or other.retired
                or other.finished
                or other.in_pit
            ):
                continue
            maneuver_group = self._maneuver_group_for_driver(other.driver_id)
            time_offsets = (
                (-PIT_MERGE_GROUP_CLEARANCE_SECONDS, 0.0, PIT_MERGE_GROUP_CLEARANCE_SECONDS)
                if maneuver_group is not None
                else (0.0,)
            )
            overlapping_poses: list[BodyPose] = []
            for time_offset in time_offsets:
                prediction_seconds = max(
                    PHYSICS_STEP_SECONDS,
                    time_to_rejoin_s + time_offset,
                )
                predicted_total_progress = (
                    other.total_progress
                    + max(0.0, other.speed_kph / 3.6)
                    * prediction_seconds
                    / self.track_length_m
                )
                predicted_lateral_m = (
                    other.lateral_offset_m
                    + other.lateral_speed_mps * prediction_seconds
                )
                other_pose = BodyPose(
                    longitudinal_m=predicted_total_progress * self.track_length_m,
                    lateral_m=predicted_lateral_m,
                    heading_rad=other.slip_angle_rad,
                    length_m=other.car_length_m,
                    width_m=other.car_width_m,
                    longitudinal_speed_mps=max(0.0, other.speed_kph / 3.6),
                    lateral_speed_mps=other.lateral_speed_mps,
                )
                if oriented_body_overlap(pit_pose, other_pose) is not None:
                    overlapping_poses.append(other_pose)
            if overlapping_poses:
                conflicts.append(
                    (
                        min(
                            abs(other_pose.longitudinal_m - pit_pose.longitudinal_m)
                            for other_pose in overlapping_poses
                        ),
                        other,
                        maneuver_group,
                    )
                )

        if not conflicts:
            return PitMergeDecision("merge")
        _, conflict, conflict_group = min(
            conflicts,
            key=lambda item: (item[0], item[1].driver_id),
        )
        hold_threshold_m = PIT_MERGE_HOLD_DISTANCE_M + 0.5
        decision = "hold" if remaining_m <= hold_threshold_m or speed_mps <= 1.0 else "yield"
        return PitMergeDecision(
            decision,
            conflict_driver_id=conflict.driver_id,
            conflict_group_id=(conflict_group.group_id if conflict_group else None),
            conflict_group_member_ids=(
                conflict_group.member_ids if conflict_group else (conflict.driver_id,)
            ),
        )

    def _finish_pit_rejoin(
        self,
        driver_id: int,
        state: DriverRaceState,
        meta: dict,
        events: list[RaceEvent],
    ) -> None:
        """Project the side-road pose into track coordinates without moving it."""
        new_compound = self._pit_tire[driver_id]
        entry = self._pit_entry_progress()
        exit_ = self._pit_exit_progress()
        pit_entry_stint = state.pit_count + 1
        lane_progress = self._pit_route_progress[driver_id]
        pit_x, pit_y, pit_heading = self._pit_lane_pose_at_progress_m(lane_progress)

        if entry is not None and exit_ is not None:
            mapped_total_progress = (
                state.current_lap
                + entry
                + self._progress_distance(entry, exit_) * lane_progress
            )
            if mapped_total_progress >= state.current_lap + 1.0:
                state.current_lap += 1
                lap_start = getattr(state, "_lap_start_time", 0.0)
                state.last_lap_time = max(
                    state.total_time - lap_start,
                    self.circuit.base_lap_time * 0.85,
                )
                if state.best_lap_time <= 0 or state.last_lap_time < state.best_lap_time:
                    state.best_lap_time = state.last_lap_time
                self._record_lap_time(
                    driver_id,
                    state.current_lap,
                    state.last_lap_time,
                    self._pit_entry_compound.get(driver_id, new_compound),
                    pit_entry_stint,
                    pit_stop=True,
                )
                state._lap_start_time = state.total_time  # type: ignore[attr-defined]
            state.total_progress = mapped_total_progress
            state.progress = mapped_total_progress % 1.0

        track_profile = self._track_physics_for_driver(state)
        line_length_m = max(
            1.0,
            track_profile.length_for_line(DRIVING_LINE_RACING),
        )
        # The pit route and circuit progress use different arc-length frames.
        # Refine the mapped circuit progress along its tangent so converting
        # from pit-route coordinates does not create a longitudinal jump.
        for _ in range(4):
            candidate_x, candidate_y, candidate_heading = (
                track_profile.line_pose_at_progress_m(
                    DRIVING_LINE_RACING,
                    state.progress,
                )
            )
            tangent_delta_m = (
                (pit_x - candidate_x) * cos(candidate_heading)
                + (pit_y - candidate_y) * sin(candidate_heading)
            )
            state.progress = (state.progress + tangent_delta_m / line_length_m) % 1.0
        state.total_progress = state.current_lap + state.progress
        line_x, line_y, line_heading = track_profile.line_pose_at_progress_m(
            DRIVING_LINE_RACING,
            state.progress,
        )
        lateral_delta_m = (
            (pit_x - line_x) * -sin(line_heading)
            + (pit_y - line_y) * cos(line_heading)
        )
        racing_offset_m = track_profile.line_offset_at_progress(
            DRIVING_LINE_RACING,
            state.progress,
        )
        state.in_pit = False
        state.pit_count += 1
        state.racing_line = DRIVING_LINE_RACING
        state.lateral_offset_m = racing_offset_m + lateral_delta_m
        state.target_lateral_offset_m = state.lateral_offset_m
        state.slip_angle_rad = (pit_heading - line_heading + pi) % (2.0 * pi) - pi
        state.total_distance_m = state.total_progress * self.track_length_m
        self._begin_sc_pit_exit_ordering(state)
        self._refresh_lap_variation(driver_id)
        merge_state = self._pit_merge_state.get(driver_id, "merge")
        self._clear_pit_state(driver_id)
        events.append(
            RaceEvent(
                type="pit_exit",
                driver=meta["abbreviation"],
                message=f"{meta['full_name']} exits pits on {new_compound.value} tires",
                message_ko=f"{meta['full_name']}가 {new_compound.value} 타이어로 피트를 빠져나옵니다",
                payload={"driver_id": driver_id, "merge_state": merge_state},
            )
        )

    def _clear_pit_state(self, driver_id: int) -> None:
        for store in (
            self._pit_phase,
            self._pit_phase_remaining,
            self._pit_phase_duration,
            self._pit_lane_half_time,
            self._pit_route_progress,
            self._pit_route_speed_mps,
            self._pit_merge_state,
            self._pit_merge_conflict_driver_id,
            self._pit_merge_conflict_group_id,
            self._pit_merge_conflict_group_member_ids,
            self._pit_tire_change_time,
            self._pit_elapsed,
            self._pit_stop_elapsed,
            self._pit_tire,
            self._pit_entry_compound,
        ):
            store.pop(driver_id, None)

    def _pit_lane_progress(self, driver_id: int) -> float:
        """Position along the pit lane (0=entry, 1=exit) for an in-pit car."""
        return min(1.0, max(0.0, self._pit_route_progress.get(driver_id, 0.0)))

    def _sync_pit_race_progress(
        self,
        driver_id: int,
        state: DriverRaceState,
    ) -> None:
        """Map pit-lane travel onto continuous lap distance for live order."""
        entry = self._pit_entry_progress()
        exit_ = self._pit_exit_progress()
        if entry is None or exit_ is None:
            return

        lane_progress = self._pit_lane_progress(driver_id)
        route_distance = self._progress_distance(entry, exit_)
        state.total_progress = state.current_lap + entry + route_distance * lane_progress
        state.total_distance_m = state.total_progress * self.track_length_m

    def _pit_lane_progress_rate(self, driver_id: int) -> float:
        """Pit-lane progress rate per game second for front-end prediction."""
        if self._pit_phase.get(driver_id) in {None, "stop"}:
            return 0.0
        route_length_m = self._pit_route_length_m()
        if route_length_m <= 1e-9:
            return 0.0
        return self._pit_route_speed_mps.get(driver_id, 0.0) / route_length_m
