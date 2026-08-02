"""Stopped-car hazards, collisions and incident application extracted from RaceEngine.

RaceEngine inherits IncidentOpsMixin so existing call sites and tests keep the
same method names.  Behavior is unchanged.

Classification helpers remain in ``incidents.py`` / ``collision.py``; this
module owns the runtime hazard corridor, contact resolution and retirement path.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from models.schemas import DriverRaceState, RaceEvent
from simulation.collision import (
    BodyMotion,
    BodyPose,
    interpolate_pose,
    oriented_body_overlap,
    swept_body_collision,
)
from simulation.incidents import (
    Incident,
    IncidentCause,
    IncidentSeverity,
    escalate_collision,
)
from simulation.racecraft_ops import MANEUVER_CLEARANCE_MARGIN_M
from simulation.state_contract import CollisionFact, TickPhase
from simulation.track_physics import (
    PHYSICAL_CAR_WIDTH_M,
    TRACK_EDGE_MARGIN_M,
)
from simulation.vehicle_physics import (
    PHYSICS_STEP_SECONDS,
    VehicleFollowingConstraint,
)

PROGRESS_EPSILON = 1e-9

INCIDENT_MINOR_TIME_PENALTY = (1.5, 4.0)
INCIDENT_MINOR_TIRE_USAGE = 0.01
FOLLOWING_MIN_BUMPER_GAP_M = 1.25
COLLISION_PAIR_COOLDOWN_SECONDS = 0.8
COLLISION_DISPLAY_SECONDS = 0.7
COLLISION_ESCALATION_IMPACT_MPS = 5.0
COLLISION_MIN_REPORT_IMPACT_MPS = 1.5
COLLISION_STATIC_PENETRATION_TOLERANCE_M = 0.05
HAZARD_DETECTION_DISTANCE_M = 300.0
HAZARD_CLEAR_RADIUS_M = 300.0
HAZARD_PASS_CLEARANCE_M = 15.0
HAZARD_LATERAL_MARGIN_M = 0.55
HAZARD_MOVING_RELEASE_MARGIN_M = 0.15
HAZARD_BRAKING_DECELERATION_MPS2 = 28.0
HAZARD_REACTION_SECONDS = 0.70
HAZARD_TRAFFIC_WINDOW_M = 8.0
HAZARD_CORRIDOR_PREFERENCE_PENALTY = 4.0
HAZARD_CORRIDOR_TARGET_MARGIN_M = 0.8
HAZARD_EVASIVE_LATERAL_ACCELERATION_MPS2 = 4.0
HAZARD_EVASIVE_TIME_BUFFER_SECONDS = 0.5
HAZARD_CLUSTER_LONGITUDINAL_GAP_M = 12.0
HAZARD_BLOCKED_CLEAR_SECONDS = 5.0
HAZARD_RECOVERY_GRACE_SECONDS = 2.0
HAZARD_RECOVERY_RELEASE_SPEED_KPH = 12.0


@dataclass
class DriverInputError:
    """A temporary AI control error; physics, not RNG, decides its outcome."""

    kind: str
    remaining_seconds: float
    intensity: float
    trigger: str
    segment_name: str = ""
    opponent_id: int | None = None


class IncidentOpsMixin:
    """Hazard corridors, vehicle collisions and incident application."""

    def _init_incident_ops_state(self) -> None:
        """Initialize hazard, collision and pending-incident runtime state."""
        self._driver_input_errors: dict[int, DriverInputError] = {}
        self._pending_incidents: list[Incident] = []
        self._collision_pair_cooldown: dict[tuple[int, int], float] = {}
        self._contact_display_remaining: dict[int, float] = {}
        self._hazard_tail_targets: dict[int, tuple[int, float]] = {}
        self._hazard_activated_at: dict[int, float] = {}
        self._hazard_recovery_until: dict[int, float] = {}

    def _hazard_recovery_active(self, state: DriverRaceState) -> bool:
        return self._hazard_recovery_until.get(state.driver_id, 0.0) > self.race_elapsed

    def _active_stopped_hazards(self) -> list[DriverRaceState]:
        return [
            state
            for state in self.driver_states.values()
            if state.hazard_active and state.vehicle_status == "stopped_on_track"
        ]

    def _stopped_hazard_cluster(
        self,
        seed: DriverRaceState,
    ) -> tuple[DriverRaceState, ...]:
        """Return stopped cars that form one connected physical obstruction."""
        active = self._active_stopped_hazards()
        cluster = {seed.driver_id}
        changed = True
        while changed:
            changed = False
            for candidate in active:
                if candidate.driver_id in cluster:
                    continue
                if any(
                    abs(candidate.total_progress - member.total_progress)
                    * self.track_length_m
                    <= HAZARD_CLUSTER_LONGITUDINAL_GAP_M
                    for member in active
                    if member.driver_id in cluster
                ):
                    cluster.add(candidate.driver_id)
                    changed = True
        return tuple(
            sorted(
                (
                    hazard
                    for hazard in active
                    if hazard.driver_id in cluster
                ),
                key=lambda hazard: (hazard.total_progress, hazard.driver_id),
            )
        )

    def _nearest_active_hazard_cluster(
        self,
        state: DriverRaceState,
        maximum_distance_m: float = HAZARD_DETECTION_DISTANCE_M,
    ) -> tuple[tuple[DriverRaceState, ...], DriverRaceState, float] | None:
        nearest = self._nearest_active_hazard(state, maximum_distance_m)
        if nearest is None:
            return None
        nearest_hazard, distance_m = nearest
        return (
            self._stopped_hazard_cluster(nearest_hazard),
            nearest_hazard,
            distance_m,
        )

    @staticmethod

    def _hazard_cluster_anchor(
        hazards: tuple[DriverRaceState, ...],
    ) -> DriverRaceState:
        return min(hazards, key=lambda hazard: hazard.driver_id)

    def _hazard_cluster_has_passable_corridor(
        self,
        hazards: tuple[DriverRaceState, ...],
    ) -> bool:
        """Check whether a full-width car center can pass either outer side."""
        if not hazards:
            return True
        car_width_m = PHYSICAL_CAR_WIDTH_M
        left_target_m = max(
            hazard.lateral_offset_m
            + 0.5 * (car_width_m + hazard.car_width_m)
            + HAZARD_LATERAL_MARGIN_M
            + HAZARD_CORRIDOR_TARGET_MARGIN_M
            for hazard in hazards
        )
        right_target_m = min(
            hazard.lateral_offset_m
            - 0.5 * (car_width_m + hazard.car_width_m)
            - HAZARD_LATERAL_MARGIN_M
            - HAZARD_CORRIDOR_TARGET_MARGIN_M
            for hazard in hazards
        )
        left_limit_m = min(
            self._track_physics.at_progress(hazard.progress).left_width_m
            - car_width_m / 2.0
            - TRACK_EDGE_MARGIN_M
            for hazard in hazards
        )
        right_limit_m = max(
            -self._track_physics.at_progress(hazard.progress).right_width_m
            + car_width_m / 2.0
            + TRACK_EDGE_MARGIN_M
            for hazard in hazards
        )
        candidate_targets_m = [left_target_m, right_target_m]
        ordered = sorted(hazards, key=lambda hazard: hazard.lateral_offset_m)
        for right_hazard, left_hazard in zip(ordered, ordered[1:]):
            right_edge_m = (
                right_hazard.lateral_offset_m
                + 0.5 * (car_width_m + right_hazard.car_width_m)
                + HAZARD_LATERAL_MARGIN_M
                + HAZARD_CORRIDOR_TARGET_MARGIN_M
            )
            left_edge_m = (
                left_hazard.lateral_offset_m
                - 0.5 * (car_width_m + left_hazard.car_width_m)
                - HAZARD_LATERAL_MARGIN_M
                - HAZARD_CORRIDOR_TARGET_MARGIN_M
            )
            if right_edge_m <= left_edge_m:
                candidate_targets_m.append(0.5 * (right_edge_m + left_edge_m))
        return any(
            right_limit_m <= target_m <= left_limit_m
            for target_m in candidate_targets_m
        )

    def _activate_stopped_hazard(
        self,
        state: DriverRaceState,
        cause: IncidentCause,
    ) -> None:
        state.vehicle_status = "stopped_on_track"
        state.hazard_active = True
        state.hazard_cause = cause.value
        self._hazard_activated_at[state.driver_id] = self.race_elapsed
        state.speed_kph = 0.0
        state.acceleration_mps2 = 0.0
        state.target_speed_kph = 0.0
        state.throttle = 0.0
        state.brake = 1.0
        state.lateral_speed_mps = 0.0
        self._progress_rate[state.driver_id] = 0.0

        running = [
            candidate
            for candidate in self.driver_states.values()
            if not candidate.retired
            and not candidate.finished
            and not candidate.in_pit
            and candidate.driver_id != state.driver_id
        ]
        if not running:
            self._hazard_tail_targets.pop(state.driver_id, None)
            return
        tail = max(running, key=lambda candidate: candidate.position)
        pass_target = self._total_progress_at_or_after(
            tail.total_progress,
            state.progress,
        ) + HAZARD_PASS_CLEARANCE_M / self.track_length_m
        self._hazard_tail_targets[state.driver_id] = (
            tail.driver_id,
            pass_target,
        )

    def _nearest_active_hazard(
        self,
        state: DriverRaceState,
        maximum_distance_m: float = HAZARD_DETECTION_DISTANCE_M,
    ) -> tuple[DriverRaceState, float] | None:
        nearest: tuple[DriverRaceState, float] | None = None
        for hazard in self._active_stopped_hazards():
            if hazard.driver_id == state.driver_id:
                continue
            distance_m = (
                hazard.total_progress - state.total_progress
            ) * self.track_length_m
            if distance_m <= 0.0 or distance_m > maximum_distance_m:
                continue
            if nearest is None or distance_m < nearest[1]:
                nearest = hazard, distance_m
        return nearest

    def _local_yellow_active_for(self, state: DriverRaceState) -> bool:
        """Return whether a driver is approaching an active 300m hazard zone."""
        return bool(
            self.race_phase == "green"
            and not state.in_pit
            and not state.retired
            and not state.finished
            and self._nearest_active_hazard(state) is not None
        )

    def _overtaking_candidate_allowed(
        self,
        attacker: DriverRaceState,
        defender: DriverRaceState | None = None,
    ) -> bool:
        """Central race-control gate for competitive overtake candidates."""
        if not self.race_started or self.race_phase != "green":
            return False
        checked = (attacker,) if defender is None else (attacker, defender)
        return all(
            not state.in_pit
            and not state.retired
            and not state.finished
            and not state.hazard_active
            and not state.avoidance_active
            and not state.emergency_braking
            and not self._local_yellow_active_for(state)
            for state in checked
        )

    def _reset_avoidance_state(self, state: DriverRaceState) -> None:
        state.avoidance_active = False
        state.avoidance_ttc_seconds = 0.0
        state.emergency_braking = False

    def _hazard_required_lateral_clearance_m(
        self,
        state: DriverRaceState,
        hazard: DriverRaceState,
    ) -> float:
        return (
            0.5 * (state.car_width_m + hazard.car_width_m)
            + HAZARD_LATERAL_MARGIN_M
        )

    def _hazard_preferred_avoidance_side(
        self,
        state: DriverRaceState,
        hazard: DriverRaceState,
    ) -> str:
        """Reserve alternating escape corridors for an approaching field.

        A stopped car is detected by several cars at nearly the same time.  A
        driver-id preference lets the whole queue choose the geometrically
        shortest side and creates a single-file blockage.  Ordering cars by
        their live distance to this hazard keeps the assignment deterministic
        while spreading the queue across both usable corridors.
        """
        approaching: list[tuple[float, int]] = []
        for other in self.driver_states.values():
            if (
                other.driver_id == hazard.driver_id
                or other.retired
                or other.finished
                or other.in_pit
            ):
                continue
            distance_m = (
                hazard.total_progress - other.total_progress
            ) * self.track_length_m
            if 0.0 < distance_m <= HAZARD_DETECTION_DISTANCE_M:
                approaching.append((distance_m, other.driver_id))
        approaching.sort(key=lambda item: (item[0], item[1]))
        rank = next(
            (
                index
                for index, (_, driver_id) in enumerate(approaching)
                if driver_id == state.driver_id
            ),
            0,
        )
        return "left" if (rank + hazard.driver_id) % 2 == 0 else "right"

    def _hazard_avoidance_target(
        self,
        state: DriverRaceState,
        track_sample,
        base_target: float,
    ) -> float | None:
        nearest_cluster = self._nearest_active_hazard_cluster(state)
        if nearest_cluster is None:
            state.avoidance_hazard_driver_id = None
            state.avoidance_side = ""
            state.avoidance_target_lateral_offset_m = base_target
            return None
        hazards, hazard, distance_m = nearest_cluster
        cluster_anchor = self._hazard_cluster_anchor(hazards)
        cluster_driver_ids = {item.driver_id for item in hazards}
        previous_hazard_id = state.avoidance_hazard_driver_id
        previous_side = state.avoidance_side
        required_clearances = {
            item.driver_id: self._hazard_required_lateral_clearance_m(
                state,
                item,
            )
            for item in hazards
        }
        required_clearance = required_clearances[hazard.driver_id]
        closest_bumper_gap_m = min(
            max(
                0.0,
                (item.total_progress - state.total_progress)
                * self.track_length_m
                - 0.5 * (state.car_length_m + item.car_length_m),
            )
            for item in hazards
            if item.total_progress > state.total_progress
        )
        closing_speed_mps = max(0.1, state.speed_kph / 3.6)
        state.avoidance_hazard_driver_id = cluster_anchor.driver_id
        state.avoidance_ttc_seconds = round(
            min(99.0, closest_bumper_gap_m / closing_speed_mps),
            3,
        )

        positive_limit = (
            track_sample.left_width_m
            - state.car_width_m / 2.0
            - TRACK_EDGE_MARGIN_M
        )
        negative_limit = (
            track_sample.right_width_m
            - state.car_width_m / 2.0
            - TRACK_EDGE_MARGIN_M
        )
        minimum_target = -negative_limit
        maximum_target = positive_limit
        if minimum_target > maximum_target:
            center_target = 0.5 * (minimum_target + maximum_target)
            minimum_target = center_target
            maximum_target = center_target
        base_target = max(minimum_target, min(maximum_target, base_target))
        if all(
            abs(base_target - item.lateral_offset_m)
            >= required_clearances[item.driver_id]
            for item in hazards
        ):
            state.avoidance_active = True
            state.avoidance_side = (
                "left"
                if base_target
                > max(item.lateral_offset_m for item in hazards)
                else "right"
            )
            state.avoidance_target_lateral_offset_m = base_target
            return base_target

        raw_candidates = [
            (
                "left",
                max(
                    item.lateral_offset_m
                    + required_clearances[item.driver_id]
                    for item in hazards
                )
                + HAZARD_CORRIDOR_TARGET_MARGIN_M,
            ),
            (
                "right",
                min(
                    item.lateral_offset_m
                    - required_clearances[item.driver_id]
                    for item in hazards
                )
                - HAZARD_CORRIDOR_TARGET_MARGIN_M,
            ),
        ]
        preferred_side = self._hazard_preferred_avoidance_side(
            state,
            cluster_anchor,
        )
        ordered_hazards = sorted(
            hazards,
            key=lambda item: item.lateral_offset_m,
        )
        cluster_center_m = sum(
            item.lateral_offset_m for item in hazards
        ) / len(hazards)
        for right_hazard, left_hazard in zip(
            ordered_hazards,
            ordered_hazards[1:],
        ):
            right_edge_m = (
                right_hazard.lateral_offset_m
                + required_clearances[right_hazard.driver_id]
                + HAZARD_CORRIDOR_TARGET_MARGIN_M
            )
            left_edge_m = (
                left_hazard.lateral_offset_m
                - required_clearances[left_hazard.driver_id]
                - HAZARD_CORRIDOR_TARGET_MARGIN_M
            )
            if right_edge_m > left_edge_m:
                continue
            middle_target_m = 0.5 * (right_edge_m + left_edge_m)
            raw_candidates.append(
                (
                    "left"
                    if middle_target_m > cluster_center_m
                    else (
                        "right"
                        if middle_target_m < cluster_center_m
                        else preferred_side
                    ),
                    middle_target_m,
                )
            )
        candidates: list[tuple[float, str, float, float]] = []
        for side, raw_target in raw_candidates:
            target = max(minimum_target, min(maximum_target, raw_target))
            if any(
                abs(target - item.lateral_offset_m)
                < required_clearances[item.driver_id] - 1e-6
                for item in hazards
            ):
                continue
            traffic_penalty = 0.0
            for other in self.driver_states.values():
                if (
                    other.driver_id == state.driver_id
                    or other.driver_id in cluster_driver_ids
                    or (other.retired and not other.hazard_active)
                    or other.finished
                    or other.in_pit
                ):
                    continue
                longitudinal_gap_m = abs(
                    other.total_progress - hazard.total_progress
                ) * self.track_length_m
                if longitudinal_gap_m > HAZARD_TRAFFIC_WINDOW_M:
                    continue
                other_clearance = 0.5 * (
                    state.car_width_m + other.car_width_m
                ) + MANEUVER_CLEARANCE_MARGIN_M
                lateral_gap_m = abs(target - other.lateral_offset_m)
                traffic_penalty += max(
                    0.0,
                    other_clearance - lateral_gap_m,
                ) * 100.0
            corridor_preference_penalty = (
                0.0
                if side == preferred_side
                else HAZARD_CORRIDOR_PREFERENCE_PENALTY
            )
            score = (
                abs(target - base_target)
                + 0.35 * abs(target - state.lateral_offset_m)
                + traffic_penalty
                + corridor_preference_penalty
            )
            candidates.append((score, side, target, traffic_penalty))

        state.avoidance_active = True
        if not candidates:
            state.avoidance_side = "blocked"
            state.avoidance_target_lateral_offset_m = base_target
            state.emergency_braking = True
            return base_target

        stable_candidate = next(
            (
                candidate
                for candidate in candidates
                if previous_hazard_id == cluster_anchor.driver_id
                and candidate[1] == previous_side
            ),
            None,
        )
        _, side, target, traffic_penalty = stable_candidate or min(
            candidates,
            key=lambda candidate: candidate[0],
        )
        state.avoidance_side = side
        state.avoidance_target_lateral_offset_m = target
        return target

    def _hazard_following_constraint(
        self,
        state: DriverRaceState,
        active_line: str,
    ) -> VehicleFollowingConstraint | None:
        track_profile = self._track_physics_for_driver(state)
        nearest_cluster = self._nearest_active_hazard_cluster(state)
        if nearest_cluster is None:
            return None
        hazards, hazard, distance_m = nearest_cluster
        required_clearances = {
            item.driver_id: self._hazard_required_lateral_clearance_m(
                state,
                item,
            )
            for item in hazards
        }
        lateral_separations_m = {
            item.driver_id: abs(
                state.lateral_offset_m - item.lateral_offset_m
            )
            for item in hazards
        }
        if all(
            lateral_separations_m[item.driver_id]
            >= required_clearances[item.driver_id]
            for item in hazards
        ):
            return None

        follower_speed_mps = max(0.0, state.speed_kph / 3.6)
        half_lengths_m = 0.5 * (state.car_length_m + hazard.car_length_m)
        minimum_gap_m = half_lengths_m + 0.35
        has_escape_corridor = (
            state.avoidance_active
            and state.avoidance_side in {"left", "right"}
            and all(
                abs(
                    state.avoidance_target_lateral_offset_m
                    - item.lateral_offset_m
                )
                >= required_clearances[item.driver_id] - 1e-6
                for item in hazards
            )
        )
        if has_escape_corridor:
            moving_away_from_hazard = (
                (
                    state.avoidance_target_lateral_offset_m
                    - state.lateral_offset_m
                )
                * state.lateral_speed_mps
                > 0.0
            )
            bodies_clear = all(
                lateral_separations_m[item.driver_id]
                >= (
                    0.5 * (state.car_width_m + item.car_width_m)
                    + HAZARD_MOVING_RELEASE_MARGIN_M
                )
                for item in hazards
            )
            if moving_away_from_hazard and bodies_clear:
                # Once the two physical bodies are clear and lateral velocity
                # is still outward, holding the car at the obstacle for one
                # more 100 ms race tick creates a visible false stop.
                return None
            # The stationary-car constraint remains as a final body-overlap
            # guard, but it must not ask every evading car to stop at the full
            # longitudinal braking distance.  The lateral controller is
            # already moving the car into a clear corridor; only lower the
            # speed when the remaining time is genuinely tight.
            lateral_remaining_m = max(
                (
                    max(
                        0.0,
                        required_clearances[item.driver_id]
                        - lateral_separations_m[item.driver_id],
                    )
                    for item in hazards
                ),
                default=0.0,
            )
            lateral_clearance_seconds = sqrt(
                2.0
                * lateral_remaining_m
                / HAZARD_EVASIVE_LATERAL_ACCELERATION_MPS2
            )
            bumper_gap_m = max(0.0, distance_m - half_lengths_m)
            time_to_bumper_seconds = bumper_gap_m / max(0.1, follower_speed_mps)
            if (
                time_to_bumper_seconds
                <= lateral_clearance_seconds + HAZARD_EVASIVE_TIME_BUFFER_SECONDS
            ):
                state.emergency_braking = True
            desired_gap_m = minimum_gap_m + 0.5
            hazard_distance_m = track_profile.line_distance_at_total_progress(
                active_line,
                hazard.total_progress,
            )
            return VehicleFollowingConstraint(
                leader_distance_m=hazard_distance_m,
                leader_speed_mps=0.0,
                leader_end_distance_m=hazard_distance_m,
                leader_end_speed_mps=0.0,
                desired_gap_m=desired_gap_m,
                minimum_gap_m=minimum_gap_m,
            )

        stopping_gap_m = (
            minimum_gap_m
            + follower_speed_mps * HAZARD_REACTION_SECONDS
            + follower_speed_mps * follower_speed_mps
            / (2.0 * HAZARD_BRAKING_DECELERATION_MPS2)
        )
        desired_gap_m = min(
            HAZARD_DETECTION_DISTANCE_M,
            max(minimum_gap_m + 2.0, stopping_gap_m),
        )
        if distance_m <= desired_gap_m:
            state.emergency_braking = True
        hazard_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            hazard.total_progress,
        )
        return VehicleFollowingConstraint(
            leader_distance_m=hazard_distance_m,
            leader_speed_mps=0.0,
            leader_end_distance_m=hazard_distance_m,
            leader_end_speed_mps=0.0,
            desired_gap_m=desired_gap_m,
            minimum_gap_m=minimum_gap_m,
        )

    def _hazard_corridor_following_constraint(
        self,
        state: DriverRaceState,
        active_line: str,
        start_snapshot: dict[int, tuple[float, float]],
    ) -> VehicleFollowingConstraint | None:
        """Follow the nearest moving car assigned to the same escape corridor."""
        track_profile = self._track_physics_for_driver(state)
        if (
            not state.avoidance_active
            or state.avoidance_hazard_driver_id is None
            or state.avoidance_side not in {"left", "right"}
        ):
            return None
        same_corridor = [
            candidate
            for candidate in self.driver_states.values()
            if candidate.driver_id != state.driver_id
            and not candidate.retired
            and not candidate.finished
            and not candidate.in_pit
            and candidate.avoidance_active
            and candidate.avoidance_hazard_driver_id
            == state.avoidance_hazard_driver_id
            and candidate.avoidance_side == state.avoidance_side
            and candidate.total_progress > state.total_progress
        ]
        if not same_corridor:
            return None
        leader = min(
            same_corridor,
            key=lambda candidate: candidate.total_progress - state.total_progress,
        )
        leader_snapshot = start_snapshot.get(leader.driver_id)
        follower_snapshot = start_snapshot.get(state.driver_id)
        if leader_snapshot is None or follower_snapshot is None:
            return None
        leader_total_progress, leader_speed_mps = leader_snapshot
        follower_total_progress, follower_speed_mps = follower_snapshot
        if leader_total_progress <= follower_total_progress + PROGRESS_EPSILON:
            return None
        half_lengths_m = 0.5 * (state.car_length_m + leader.car_length_m)
        minimum_gap_m = half_lengths_m + 0.35
        desired_gap_m = min(
            18.0,
            max(8.0, minimum_gap_m + max(0.0, follower_speed_mps) * 0.12),
        )
        return VehicleFollowingConstraint(
            leader_distance_m=track_profile.line_distance_at_total_progress(
                active_line,
                leader_total_progress,
            ),
            leader_speed_mps=max(0.0, leader_speed_mps),
            leader_end_distance_m=track_profile.line_distance_at_total_progress(
                active_line,
                leader.total_progress,
            ),
            leader_end_speed_mps=max(0.0, leader.speed_kph / 3.6),
            desired_gap_m=desired_gap_m,
            minimum_gap_m=minimum_gap_m,
        )

    def _start_driver_input_error(
        self,
        driver_id: int,
        *,
        kind: str,
        duration_seconds: float,
        intensity: float,
        trigger: str,
        segment_name: str = "",
        opponent_id: int | None = None,
    ) -> None:
        """Inject a bounded control mistake without claiming a physical outcome."""
        current = self._driver_input_errors.get(driver_id)
        if current is not None and current.kind == kind:
            current.remaining_seconds = max(current.remaining_seconds, duration_seconds)
            current.intensity = max(current.intensity, intensity)
            current.trigger = trigger
            current.segment_name = segment_name
            current.opponent_id = opponent_id
            return
        self._driver_input_errors[driver_id] = DriverInputError(
            kind=kind,
            remaining_seconds=duration_seconds,
            intensity=intensity,
            trigger=trigger,
            segment_name=segment_name,
            opponent_id=opponent_id,
        )

    def _tick_driver_input_errors(self, delta: float) -> None:
        for driver_id, input_error in list(self._driver_input_errors.items()):
            input_error.remaining_seconds -= delta
            if input_error.remaining_seconds <= 0.0:
                del self._driver_input_errors[driver_id]

    def _tick_collision_states(self, delta: float) -> None:
        self._tick_driver_input_errors(delta)
        for driver_id, remaining in list(
            self._physical_handling_event_cooldown.items()
        ):
            remaining -= delta
            if remaining <= 0.0:
                del self._physical_handling_event_cooldown[driver_id]
            else:
                self._physical_handling_event_cooldown[driver_id] = remaining
        for driver_id, until in list(self._hazard_recovery_until.items()):
            if until <= self.race_elapsed:
                del self._hazard_recovery_until[driver_id]
        for key, remaining in list(self._collision_pair_cooldown.items()):
            remaining -= delta
            if remaining <= 0.0:
                del self._collision_pair_cooldown[key]
            else:
                self._collision_pair_cooldown[key] = remaining
        for driver_id, remaining in list(self._contact_display_remaining.items()):
            remaining -= delta
            if remaining > 0.0:
                self._contact_display_remaining[driver_id] = remaining
                continue
            del self._contact_display_remaining[driver_id]
            state = self.driver_states.get(driver_id)
            if state is None:
                continue
            state.contact_active = False
            state.contact_opponent_id = None
            state.contact_impact_speed_mps = 0.0
            state.contact_type = ""
            state.contact_severity = ""
            state.contact_normal_longitudinal = 0.0
            state.contact_normal_lateral = 0.0

    def _record_contact_state(
        self,
        state: DriverRaceState,
        opponent_id: int,
        *,
        impact_speed_mps: float,
        contact_type: str,
        severity: IncidentSeverity,
        progress: float,
        lateral_offset_m: float,
        normal_longitudinal: float,
        normal_lateral: float,
    ) -> None:
        state.contact_active = True
        state.contact_opponent_id = opponent_id
        state.contact_impact_speed_mps = round(impact_speed_mps, 3)
        state.contact_type = contact_type
        state.contact_severity = severity.value
        state.contact_progress = progress % 1.0
        state.contact_lateral_offset_m = round(lateral_offset_m, 4)
        state.contact_normal_longitudinal = round(normal_longitudinal, 4)
        state.contact_normal_lateral = round(normal_lateral, 4)
        self._contact_display_remaining[state.driver_id] = COLLISION_DISPLAY_SECONDS

    def _apply_collision_response(
        self,
        first: DriverRaceState,
        second: DriverRaceState,
        first_motion: BodyMotion,
        second_motion: BodyMotion,
        contact,
        delta: float,
    ) -> tuple[BodyPose, BodyPose]:
        """Resolve at impact, exchange momentum, then finish the remaining tick."""
        safe_fraction = max(0.0, contact.time_fraction - 1e-4)
        first_pose = interpolate_pose(first_motion, safe_fraction)
        second_pose = interpolate_pose(second_motion, safe_fraction)

        if contact.time_fraction <= 1e-9 and contact.penetration_m > 0.0:
            correction = 0.5 * (contact.penetration_m + 0.005)
            first_pose = BodyPose(
                first_pose.longitudinal_m - contact.normal_longitudinal * correction,
                first_pose.lateral_m - contact.normal_lateral * correction,
                first_pose.heading_rad,
                first_pose.length_m,
                first_pose.width_m,
                first_pose.longitudinal_speed_mps,
                first_pose.lateral_speed_mps,
            )
            second_pose = BodyPose(
                second_pose.longitudinal_m + contact.normal_longitudinal * correction,
                second_pose.lateral_m + contact.normal_lateral * correction,
                second_pose.heading_rad,
                second_pose.length_m,
                second_pose.width_m,
                second_pose.longitudinal_speed_mps,
                second_pose.lateral_speed_mps,
            )

        self._set_state_total_progress(
            first,
            first_pose.longitudinal_m / self.track_length_m,
        )
        self._set_state_total_progress(
            second,
            second_pose.longitudinal_m / self.track_length_m,
        )
        first.lateral_offset_m = round(first_pose.lateral_m, 4)
        second.lateral_offset_m = round(second_pose.lateral_m, 4)

        first_speed = max(0.0, first_pose.longitudinal_speed_mps)
        second_speed = max(0.0, second_pose.longitudinal_speed_mps)
        impact = max(0.0, contact.impact_speed_mps)
        if contact.contact_type == "front_rear":
            faster_is_first = first_speed >= second_speed
            slower_speed = min(first_speed, second_speed)
            speed_difference = abs(first_speed - second_speed)
            slower_after = max(0.0, slower_speed - min(2.0, 0.03 * impact))
            faster_after = max(
                0.0,
                slower_after
                + 0.10 * speed_difference
                - min(4.0, 0.08 * impact),
            )
            if faster_is_first:
                first_speed, second_speed = faster_after, slower_after
            else:
                first_speed, second_speed = slower_after, faster_after
        else:
            # A static overlap is already resolved by positional correction.
            # Do not invent a minimum loss or lateral kick when two cars have
            # no closing speed; that made harmless close running look like a
            # magnetic rejection.  Real impact still dissipates energy.
            energy_loss = min(8.0, 0.16 * impact)
            first_speed -= energy_loss
            second_speed -= energy_loss

        first.speed_kph = round(max(0.0, first_speed) * 3.6, 3)
        second.speed_kph = round(max(0.0, second_speed) * 3.6, 3)
        lateral_impulse = min(4.0, 0.18 * impact)
        first.lateral_speed_mps = max(
            -4.0,
            min(
                4.0,
                first_pose.lateral_speed_mps
                - contact.normal_lateral * lateral_impulse,
            ),
        )
        second.lateral_speed_mps = max(
            -4.0,
            min(
                4.0,
                second_pose.lateral_speed_mps
                + contact.normal_lateral * lateral_impulse,
            ),
        )
        yaw_impulse = min(0.12, 0.008 * impact)
        first.slip_angle_rad = max(
            -0.20,
            min(
                0.20,
                first.slip_angle_rad - contact.normal_lateral * yaw_impulse,
            ),
        )
        second.slip_angle_rad = max(
            -0.20,
            min(
                0.20,
                second.slip_angle_rad + contact.normal_lateral * yaw_impulse,
            ),
        )

        # Continue through the unused part of this physics tick at the
        # post-impact speed. Previously both cars stayed at the impact point
        # until the next tick; progress reconciliation then interpreted that
        # truncated motion as a near stop, which invited a following car into
        # an otherwise minor contact.
        if contact.contact_type == "front_rear":
            leader, follower = (
                (first, second)
                if first.total_progress >= second.total_progress
                else (second, first)
            )
            follower.speed_kph = min(follower.speed_kph, leader.speed_kph)
        remaining_seconds = max(0.0, delta * (1.0 - contact.time_fraction))
        if remaining_seconds > 0.0:
            self._set_state_total_progress(
                first,
                first.total_progress
                + (first.speed_kph / 3.6) * remaining_seconds / self.track_length_m,
            )
            self._set_state_total_progress(
                second,
                second.total_progress
                + (second.speed_kph / 3.6) * remaining_seconds / self.track_length_m,
            )
        for _ in range(4):
            residual_overlap = oriented_body_overlap(
                self._body_pose(first),
                self._body_pose(second),
            )
            if residual_overlap is None or residual_overlap[0] <= 1e-6:
                break
            penetration, normal = residual_overlap
            correction = 0.5 * (penetration + 0.005)
            self._set_state_total_progress(
                first,
                first.total_progress
                - normal[0] * correction / self.track_length_m,
            )
            self._set_state_total_progress(
                second,
                second.total_progress
                + normal[0] * correction / self.track_length_m,
            )
            first.lateral_offset_m = round(
                first.lateral_offset_m - normal[1] * correction,
                4,
            )
            second.lateral_offset_m = round(
                second.lateral_offset_m + normal[1] * correction,
                4,
            )
        collision_deceleration = impact / max(delta, PHYSICS_STEP_SECONDS) * 0.20
        first.acceleration_mps2 = min(first.acceleration_mps2, -collision_deceleration)
        second.acceleration_mps2 = min(second.acceleration_mps2, -collision_deceleration)
        first.handling_state = "contact"
        second.handling_state = "contact"
        first.tire_usage += min(0.02, impact * 0.0008)
        second.tire_usage += min(0.02, impact * 0.0008)
        return self._body_pose(first), self._body_pose(second)

    def _resolve_vehicle_collisions(
        self,
        start_poses: dict[int, BodyPose],
        delta: float,
        *,
        collect_facts: bool = False,
    ) -> list[CollisionFact] | list[RaceEvent]:
        running = [
            state
            for state in self.driver_states.values()
            if not state.retired and not state.finished and not state.in_pit
        ]
        if self.race_phase != "green":
            self._resolve_static_body_penetrations(running)
            return []
        facts: list[CollisionFact] = []
        end_poses = {
            state.driver_id: self._body_pose(state)
            for state in running
        }
        for first_index, first in enumerate(running):
            if first.retired:
                continue
            first_start = start_poses.get(first.driver_id)
            if first_start is None:
                continue
            for second in running[first_index + 1:]:
                if first.retired:
                    break
                if second.retired:
                    continue
                key = self._battle_pair_key(first.driver_id, second.driver_id)
                if self._collision_pair_cooldown.get(key, 0.0) > 0.0:
                    continue
                second_start = start_poses.get(second.driver_id)
                if second_start is None:
                    continue
                first_end = end_poses[first.driver_id]
                second_end = end_poses[second.driver_id]
                start_gap = abs(
                    first_start.longitudinal_m - second_start.longitudinal_m
                )
                end_gap = abs(first_end.longitudinal_m - second_end.longitudinal_m)
                if min(start_gap, end_gap) > 14.0:
                    continue
                first_motion = BodyMotion(first_start, first_end)
                second_motion = BodyMotion(second_start, second_end)
                contact = swept_body_collision(
                    first_motion,
                    second_motion,
                    substeps=max(1, round(delta / PHYSICS_STEP_SECONDS)),
                )
                if contact is None:
                    continue
                if (
                    contact.impact_speed_mps < COLLISION_MIN_REPORT_IMPACT_MPS
                    and contact.penetration_m
                    < COLLISION_STATIC_PENETRATION_TOLERANCE_M
                ):
                    continue

                first_pose, second_pose = self._apply_collision_response(
                    first,
                    second,
                    first_motion,
                    second_motion,
                    contact,
                    delta,
                )
                end_poses[first.driver_id] = first_pose
                end_poses[second.driver_id] = second_pose
                # PHYSICS records contact facts only.  Severity, damage
                # escalation and visible events belong to RULES.
                contact_progress = (
                    0.5 * (first.total_progress + second.total_progress)
                ) % 1.0
                contact_lateral = 0.5 * (
                    first.lateral_offset_m + second.lateral_offset_m
                )
                self._record_contact_state(
                    first,
                    second.driver_id,
                    impact_speed_mps=contact.impact_speed_mps,
                    contact_type=contact.contact_type,
                    severity=IncidentSeverity.MINOR,
                    progress=contact_progress,
                    lateral_offset_m=contact_lateral,
                    normal_longitudinal=contact.normal_longitudinal,
                    normal_lateral=contact.normal_lateral,
                )
                self._record_contact_state(
                    second,
                    first.driver_id,
                    impact_speed_mps=contact.impact_speed_mps,
                    contact_type=contact.contact_type,
                    severity=IncidentSeverity.MINOR,
                    progress=contact_progress,
                    lateral_offset_m=contact_lateral,
                    normal_longitudinal=-contact.normal_longitudinal,
                    normal_lateral=-contact.normal_lateral,
                )
                self._collision_pair_cooldown[key] = COLLISION_PAIR_COOLDOWN_SECONDS
                facts.append(
                    CollisionFact(
                        first_driver_id=first.driver_id,
                        second_driver_id=second.driver_id,
                        impact_speed_mps=contact.impact_speed_mps,
                        contact_type=contact.contact_type,
                        normal_longitudinal=contact.normal_longitudinal,
                        normal_lateral=contact.normal_lateral,
                        first_speed_mps=first_start.longitudinal_speed_mps,
                        second_speed_mps=second_start.longitudinal_speed_mps,
                    )
                )
        self._resolve_static_body_penetrations(running)
        if collect_facts:
            return facts
        events: list[RaceEvent] = []
        self._apply_collision_facts(facts, events)
        return events

    def _apply_collision_facts(
        self,
        facts: list[CollisionFact],
        events: list[RaceEvent],
    ) -> None:
        """Resolve raw PHYSICS contacts into RULES outcomes exactly once."""
        self._require_rules_phase("apply collision facts")
        for fact in facts:
            first = self.driver_states.get(fact.first_driver_id)
            second = self.driver_states.get(fact.second_driver_id)
            if first is None or second is None:
                continue
            severity = IncidentSeverity.MINOR
            if fact.impact_speed_mps >= COLLISION_ESCALATION_IMPACT_MPS:
                severity = escalate_collision(
                    self.rng,
                    segment_type=self._segment_type_at(first),
                    impact_speed_mps=fact.impact_speed_mps,
                )
            damage_increment = min(
                0.35,
                0.006 * fact.impact_speed_mps
                + (
                    0.30
                    if severity == IncidentSeverity.CRASH
                    else 0.12
                    if severity == IncidentSeverity.CAR_STOPPED
                    else 0.0
                ),
            )
            first.collision_damage = min(1.0, first.collision_damage + damage_increment)
            second.collision_damage = min(1.0, second.collision_damage + damage_increment)
            first.contact_severity = severity.value
            second.contact_severity = severity.value

            first_meta = self._driver_meta[first.driver_id]
            second_meta = self._driver_meta[second.driver_id]
            events.append(
                RaceEvent(
                    type="minor_contact" if severity == IncidentSeverity.MINOR else "collision",
                    driver=first_meta["abbreviation"],
                    message=(
                        f"{first_meta['full_name']} and {second_meta['full_name']}"
                        f" make {fact.contact_type.replace('_', '-')} contact"
                        f" at {fact.impact_speed_mps:.1f}m/s relative speed"
                    ),
                    message_ko=(
                        f"{first_meta['full_name']}와 {second_meta['full_name']}가"
                        f" 상대속도 {fact.impact_speed_mps:.1f}m/s로 실제 접촉합니다"
                    ),
                )
            )
            battle = self._side_by_side_battles.get(
                self._battle_pair_key(first.driver_id, second.driver_id)
            )
            if battle is not None:
                self._set_maneuver_phase(battle, "abort")
            if severity != IncidentSeverity.MINOR:
                primary, secondary = (
                    (first, second)
                    if fact.first_speed_mps >= fact.second_speed_mps
                    else (second, first)
                )
                self._apply_incident(
                    Incident(
                        cause=IncidentCause.COLLISION,
                        severity=severity,
                        primary_driver_id=primary.driver_id,
                        secondary_driver_id=(
                            secondary.driver_id
                            if severity == IncidentSeverity.CRASH
                            else None
                        ),
                    ),
                    events,
                )

    def _resolve_static_body_penetrations(
        self,
        states: list[DriverRaceState],
    ) -> None:
        """Settle multi-car contact chains with minimum translation vectors."""
        active = sorted(
            (state for state in states if not state.retired),
            key=lambda state: -state.total_progress,
        )
        for _ in range(24):
            moved = False
            for first_index, first in enumerate(active):
                for second in active[first_index + 1:]:
                    if abs(first.total_progress - second.total_progress) * self.track_length_m > 8.0:
                        continue
                    overlap = oriented_body_overlap(
                        self._body_pose(first),
                        self._body_pose(second),
                    )
                    if overlap is None or overlap[0] <= 1e-5:
                        continue
                    penetration, normal = overlap
                    correction = penetration + 0.002
                    if abs(normal[0]) >= abs(normal[1]):
                        # Preserve the leading car and propagate the exact minimum
                        # translation rearward through a contact train. Symmetric
                        # corrections can otherwise reopen an already solved pair.
                        self._set_state_total_progress(
                            second,
                            second.total_progress
                            + normal[0] * correction / self.track_length_m,
                        )
                        second.lateral_offset_m = round(
                            second.lateral_offset_m + normal[1] * correction,
                            4,
                        )
                    else:
                        half_correction = 0.5 * correction
                        self._set_state_total_progress(
                            first,
                            first.total_progress
                            - normal[0] * half_correction / self.track_length_m,
                        )
                        self._set_state_total_progress(
                            second,
                            second.total_progress
                            + normal[0] * half_correction / self.track_length_m,
                        )
                        first.lateral_offset_m = round(
                            first.lateral_offset_m - normal[1] * half_correction,
                            4,
                        )
                        second.lateral_offset_m = round(
                            second.lateral_offset_m + normal[1] * half_correction,
                            4,
                        )
                    moved = True
            if not moved:
                break

    def retire_driver_for_testing(self, driver_id: int) -> list[RaceEvent]:
        """Create a repeatable stopped-car retirement for browser integration tests."""
        state = self.driver_states.get(int(driver_id))
        if state is None:
            raise ValueError(f"Unknown driver: {driver_id}")
        if state.retired or state.finished:
            raise ValueError(f"Driver is not running: {driver_id}")
        events: list[RaceEvent] = []
        self._apply_incident(
            Incident(
                cause=IncidentCause.MECHANICAL,
                severity=IncidentSeverity.CAR_STOPPED,
                primary_driver_id=state.driver_id,
            ),
            events,
        )
        return events

    def _physical_track_distance_m(
        self,
        first: DriverRaceState,
        second: DriverRaceState,
    ) -> float:
        progress_gap = abs((first.progress % 1.0) - (second.progress % 1.0))
        return min(progress_gap, 1.0 - progress_gap) * self.track_length_m

    def _tick_stopped_hazard_clearance(self, events: list[RaceEvent]) -> None:
        if self.race_phase != "sc":
            return
        running = [
            state
            for state in self.driver_states.values()
            if not state.retired and not state.finished and not state.in_pit
        ]
        active_hazards = list(self._active_stopped_hazards())
        blocked_clear_ids: set[int] = set()
        assessed_cluster_ids: set[int] = set()
        for hazard in active_hazards:
            if hazard.driver_id in assessed_cluster_ids:
                continue
            cluster = self._stopped_hazard_cluster(hazard)
            cluster_ids = {item.driver_id for item in cluster}
            assessed_cluster_ids.update(cluster_ids)
            latest_activation = max(
                self._hazard_activated_at.get(
                    item.driver_id,
                    self.race_elapsed,
                )
                for item in cluster
            )
            if (
                not self._hazard_cluster_has_passable_corridor(cluster)
                and self.race_elapsed - latest_activation
                >= HAZARD_BLOCKED_CLEAR_SECONDS
            ):
                blocked_clear_ids.update(cluster_ids)

        for hazard in active_hazards:
            tail_target = self._hazard_tail_targets.get(hazard.driver_id)
            tail_passed = False
            if tail_target is not None:
                tail_driver_id, target_progress = tail_target
                tail = self.driver_states.get(tail_driver_id)
                tail_passed = bool(
                    tail is not None
                    and not tail.retired
                    and tail.total_progress >= target_progress - PROGRESS_EPSILON
                )
            no_car_within_radius = not any(
                self._physical_track_distance_m(hazard, state)
                <= HAZARD_CLEAR_RADIUS_M
                for state in running
            )
            blocked_clearance = hazard.driver_id in blocked_clear_ids
            if (
                not tail_passed
                and not no_car_within_radius
                and not blocked_clearance
            ):
                continue
            recovery_until = self.race_elapsed + HAZARD_RECOVERY_GRACE_SECONDS
            for state in running:
                if (
                    self._physical_track_distance_m(hazard, state)
                    <= HAZARD_DETECTION_DISTANCE_M
                ):
                    self._hazard_recovery_until[state.driver_id] = max(
                        self._hazard_recovery_until.get(state.driver_id, 0.0),
                        recovery_until,
                    )
                    if state.speed_kph < 5.0:
                        # A blocked cluster can leave the last cars at zero
                        # after the hazard is neutralized.  Give the bounded
                        # recovery controller a rolling start; this changes no
                        # position and expires with the recovery grace period.
                        state.speed_kph = HAZARD_RECOVERY_RELEASE_SPEED_KPH
            hazard.hazard_active = False
            hazard.vehicle_status = "cleared"
            self._hazard_tail_targets.pop(hazard.driver_id, None)
            self._hazard_activated_at.pop(hazard.driver_id, None)
            meta = self._driver_meta[hazard.driver_id]
            reason = (
                "the tail of the field has passed"
                if tail_passed
                else (
                    "the blocked circuit has been neutralized under Safety Car"
                    if blocked_clearance
                    else "the 300m safety zone is clear"
                )
            )
            reason_ko = (
                "최후미 차량이 사고 지점을 통과해"
                if tail_passed
                else (
                    "통과할 물리적 공간이 없어 세이프티카 아래 사고 구역을 통제한 뒤"
                    if blocked_clearance
                    else "반경 300m에 주행 차량이 없어"
                )
            )
            events.append(
                RaceEvent(
                    type="hazard_cleared",
                    driver=meta["abbreviation"],
                    message=(
                        f"{meta['full_name']}'s stopped car is removed after {reason}"
                    ),
                    message_ko=(
                        f"{reason_ko} {meta['full_name']}의 정지 차량을 제거합니다"
                    ),
                )
            )

    def _apply_incident(self, incident: Incident, events: list[RaceEvent]) -> None:
        """Apply an incident's effect and trigger SC/VSC for severe outcomes."""
        primary = self.driver_states.get(incident.primary_driver_id)
        if primary is None or primary.retired or primary.finished:
            return
        meta = self._driver_meta[incident.primary_driver_id]

        if incident.severity == IncidentSeverity.MINOR:
            penalty = self.rng.uniform(*INCIDENT_MINOR_TIME_PENALTY)
            primary.total_time += penalty
            primary.tire_usage += INCIDENT_MINOR_TIRE_USAGE
            if incident.cause == IncidentCause.MECHANICAL:
                message = f"{meta['full_name']} reports a problem — loses time"
                message_ko = f"{meta['full_name']}가 차량 문제로 시간을 잃습니다"
            else:
                message = f"{meta['full_name']} runs wide — loses time"
                message_ko = f"{meta['full_name']}가 코스를 벗어나 시간을 잃습니다"
            events.append(
                RaceEvent(type="incident", driver=meta["abbreviation"], message=message, message_ko=message_ko)
            )
            return

        # CAR_STOPPED or CRASH: the primary driver is out.
        primary.retired = True
        stopped_states = [primary]
        if incident.cause == IncidentCause.MECHANICAL:
            message = f"{meta['full_name']} stops on track — mechanical failure"
            message_ko = f"{meta['full_name']}가 기계 결함으로 트랙에 멈춥니다"
        elif incident.severity == IncidentSeverity.CRASH:
            message = f"{meta['full_name']} crashes out"
            message_ko = f"{meta['full_name']}가 충돌로 리타이어합니다"
        else:
            message = f"{meta['full_name']} stops on track"
            message_ko = f"{meta['full_name']}가 트랙에 멈춰 섭니다"
        events.append(
            RaceEvent(type="retirement", driver=meta["abbreviation"], message=message, message_ko=message_ko)
        )

        if incident.secondary_driver_id is not None:
            secondary = self.driver_states.get(incident.secondary_driver_id)
            if secondary is not None and not secondary.retired and not secondary.finished:
                secondary.retired = True
                stopped_states.append(secondary)
                smeta = self._driver_meta[incident.secondary_driver_id]
                events.append(
                    RaceEvent(
                        type="retirement",
                        driver=smeta["abbreviation"],
                        message=f"{smeta['full_name']} is collected in the incident",
                        message_ko=f"{smeta['full_name']}가 사고에 휘말려 리타이어합니다",
                    )
                )

        for stopped_state in stopped_states:
            self._activate_stopped_hazard(stopped_state, incident.cause)
        self._trigger_safety_car(events)
