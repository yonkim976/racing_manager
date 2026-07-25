"""Safety Car / VSC race-control extracted from RaceEngine.

RaceEngine inherits SafetyCarMixin so existing call sites and tests keep the
same method names on the engine instance.  Behavior is unchanged; this module
only relocates the SC/VSC domain.
"""

from __future__ import annotations

from math import sqrt

from models.schemas import DriverRaceState, RaceEvent, TrackSegmentType
from simulation.ai_strategy import choose_pit_tire
from simulation.track_geometry import segment_at_progress
from simulation.track_physics import DRIVING_LINE_RACING, PHYSICAL_CAR_LENGTH_M
from simulation.vehicle_physics import (
    VehicleFollowingConstraint,
    VehiclePhysicsModifiers,
)

# Shared with RaceEngine following logic; keep the numeric value identical.
FOLLOWING_MIN_BUMPER_GAP_M = 1.25
PROGRESS_EPSILON = 1e-9

VSC_DURATION_SECONDS = 25.0
VSC_LAP_TIME_FACTOR = 1.4
SC_LAP_TIME_FACTOR = 1.8
SC_COLLECTION_LAP_TIME_FACTOR = 2.5
SC_CATCH_UP_FAST_LAP_TIME_FACTOR = 0.85
SC_CATCH_UP_NEAR_LAP_TIME_FACTOR = 1.30
SC_CAUGHT_RECOVERY_LAP_TIME_FACTOR = 1.60
SC_UNLAP_LAP_TIME_FACTOR = 1.0
SC_LEAD_PROGRESS_GAP = 0.012  # lap-fraction gap between the SC and the on-track leader
SC_CAR_LENGTH_M = PHYSICAL_CAR_LENGTH_M
SC_MAX_GAP_CAR_LENGTHS = 10.0
SC_RELEASE_GAP_CAR_LENGTHS = 12.0
SC_QUEUE_TARGET_CAR_LENGTHS = 7.0
SC_CATCH_UP_MAX_SPEED_KPH = 300.0
SC_NEAR_QUEUE_MAX_SPEED_KPH = 220.0
SC_NEAR_QUEUE_DISTANCE_CAR_LENGTHS = 40.0
SC_CAUGHT_MAX_SPEED_KPH = 180.0
SC_QUEUE_APPROACH_REACTION_SECONDS = 0.65
SC_QUEUE_APPROACH_DECELERATION_MPS2 = 5.0
SC_QUEUE_APPROACH_MAX_GAP_M = 800.0
SC_QUEUE_PROPAGATION_HORIZON_M = 600.0
SC_QUEUE_JOIN_MAX_RELATIVE_SPEED_KPH = 24.0
SC_ORDER_RESTORE_TRIGGER_M = 1.0
SC_ORDER_RESTORE_RELEASE_CAR_LENGTHS = 1.5
SC_ORDER_RESTORE_SPEED_DELTA_KPH = 18.0
SC_UNLAP_MAX_SPEED_KPH = 280.0
SC_LEADER_ACQUISITION_SPEED_KPH = 5.0
SC_DEPLOY_PIT_SECONDS = 2.5
SC_WITHDRAW_PIT_SECONDS = 3.0
SC_CLEANUP_SECONDS = 35.0
SC_ADDITIONAL_INCIDENT_SECONDS = 18.0
SC_PIT_WEAR_THRESHOLD = 0.30  # AI takes the "free" SC pit once tires are this worn
SC_PIT_PROBABILITY = 0.4  # per-eligible-driver chance to dive in under SC (avoids all-stop)


class SafetyCarMixin:
    """SC/VSC lifecycle, queue formation, unlapping and speed caps."""

    def _init_safety_car_state(self) -> None:
        """Initialize Safety Car / VSC race-control state on the host engine."""
        self.safety_car = False
        self.race_phase = "green"  # green | vsc | sc
        self._phase_until = 0.0  # race_elapsed deadline for VSC
        self.safety_car_stage = "inactive"
        self._safety_car_visible = False
        self._safety_car_route = "track"
        self._safety_car_pit_lane_progress = 0.0
        self._safety_car_queue_formed = False
        self._sc_cleanup_until = 0.0
        self._sc_caught_driver_ids: set[int] = set()
        self._sc_unlap_driver_ids: set[int] = set()
        self._sc_unlap_targets: dict[int, float] = {}
        self._sc_running_order: list[int] = []
        self._sc_pit_exit_order_targets: dict[int, float] = {}
        self._sc_order_yield_targets: dict[int, int | None] = {}
        self._sc_withdraw_target: float | None = None
        self._sc_restart_target: float | None = None
        self._sc_restart_accel_progress: float | None = None
        self._safety_car_total_progress: float | None = None
        self._safety_car_progress_rate = 0.0
        self._safety_car_speed_mps = 0.0

    def _race_control_speed_cap_mps(
        self,
        state: DriverRaceState,
    ) -> float | None:
        """Return a hard SC speed ceiling in addition to the delta target."""
        if self.race_phase != "sc":
            return None
        if state.driver_id in self._sc_unlap_driver_ids:
            return SC_UNLAP_MAX_SPEED_KPH / 3.6
        base_cap_mps = (
            SC_CAUGHT_MAX_SPEED_KPH / 3.6
            if state.driver_id in self._sc_caught_driver_ids
            else SC_CATCH_UP_MAX_SPEED_KPH / 3.6
        )
        if state.driver_id in self._sc_order_yield_targets:
            predecessor_id = self._sc_order_yield_targets[state.driver_id]
            if predecessor_id is None:
                predecessor_speed_mps = max(0.0, self._safety_car_speed_mps)
            else:
                predecessor_state = self.driver_states.get(predecessor_id)
                predecessor_speed_mps = (
                    max(0.0, predecessor_state.speed_kph / 3.6)
                    if predecessor_state is not None
                    else 0.0
                )
            # A car that has physically crossed the frozen SC order gives the
            # place back by driving slightly slower than its predecessor.  The
            # normal longitudinal model supplies the braking; no position or
            # world pose is snapped.
            return min(
                base_cap_mps,
                max(
                    0.0,
                    predecessor_speed_mps
                    - SC_ORDER_RESTORE_SPEED_DELTA_KPH / 3.6,
                ),
            )
        predecessor = self._sc_queue_predecessor(state)
        if predecessor is None:
            ahead_total = self._safety_car_total_progress
            ahead_is_queued = True
            ahead_speed_mps = self._safety_car_speed_mps
        else:
            ahead_total = predecessor.total_progress
            ahead_is_queued = (
                predecessor.driver_id in self._sc_caught_driver_ids
                or self._sc_queue_control_reaches(predecessor)
            )
            ahead_speed_mps = max(0.0, predecessor.speed_kph / 3.6)
        gap_m = (
            (ahead_total - state.total_progress) * self.track_length_m
            if ahead_total is not None
            else float("inf")
        )
        if (
            ahead_is_queued
            and 0.0 <= gap_m
            <= max(
                SC_QUEUE_APPROACH_MAX_GAP_M,
                SC_CAR_LENGTH_M * SC_NEAR_QUEUE_DISTANCE_CAR_LENGTHS,
            )
        ):
            follower_speed_mps = max(0.0, state.speed_kph / 3.6)
            closing_speed_mps = max(0.0, follower_speed_mps - ahead_speed_mps)
            available_braking_distance_m = max(
                0.0,
                gap_m
                - SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS
                - closing_speed_mps * SC_QUEUE_APPROACH_REACTION_SECONDS,
            )
            safe_approach_speed_mps = sqrt(
                ahead_speed_mps * ahead_speed_mps
                + 2.0
                * SC_QUEUE_APPROACH_DECELERATION_MPS2
                * available_braking_distance_m
            )
            approach_cap_mps = min(base_cap_mps, safe_approach_speed_mps)
            if gap_m <= SC_CAR_LENGTH_M * SC_NEAR_QUEUE_DISTANCE_CAR_LENGTHS:
                approach_cap_mps = min(
                    approach_cap_mps,
                    SC_NEAR_QUEUE_MAX_SPEED_KPH / 3.6,
                )
            return approach_cap_mps
        return base_cap_mps

    def _safety_car_following_constraint(
        self,
        state: DriverRaceState,
        car_ahead: DriverRaceState | None,
        active_line: str,
        delta: float,
    ) -> VehicleFollowingConstraint | None:
        """Represent the on-track Safety Car as the queue leader without snapping cars."""
        track_profile = self._track_physics_for_driver(state)
        if (
            self.race_phase != "sc"
            or self.safety_car_stage in {"deploying", "restart"}
            or self._safety_car_total_progress is None
        ):
            return None
        on_track_leader = self._on_track_leader()
        if on_track_leader is None or on_track_leader.driver_id != state.driver_id:
            return None
        if (
            car_ahead is not None
            and not car_ahead.in_pit
            and not car_ahead.retired
            and not car_ahead.finished
        ):
            return None

        end_progress = self._safety_car_total_progress
        start_progress = end_progress - self._safety_car_progress_rate * max(0.0, delta)
        leader_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            start_progress,
        )
        leader_end_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            end_progress,
        )
        follower_distance_m = track_profile.line_distance_at_total_progress(
            active_line,
            state.total_progress,
        )
        if leader_end_distance_m <= follower_distance_m + PROGRESS_EPSILON:
            return None
        safety_car_speed_mps = max(0.0, self._safety_car_speed_mps)
        return VehicleFollowingConstraint(
            leader_distance_m=leader_distance_m,
            leader_speed_mps=safety_car_speed_mps,
            leader_end_distance_m=leader_end_distance_m,
            leader_end_speed_mps=safety_car_speed_mps,
            desired_gap_m=SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS,
            minimum_gap_m=SC_CAR_LENGTH_M + FOLLOWING_MIN_BUMPER_GAP_M,
        )

    def _phase_lap_time_factor(self, state: DriverRaceState | None = None) -> float:
        if self.race_phase == "sc":
            if state is not None and state.driver_id in self._sc_unlap_driver_ids:
                return SC_UNLAP_LAP_TIME_FACTOR
            if self.safety_car_stage == "restart":
                leader = self._on_track_leader()
                if (
                    leader is not None
                    and self._sc_restart_accel_progress is not None
                    and leader.total_progress >= self._sc_restart_accel_progress
                ):
                    return 1.0
            if state is None:
                return SC_CATCH_UP_FAST_LAP_TIME_FACTOR
            return self._sc_lap_time_factor_for_gap(state)
        if self.race_phase == "vsc":
            return VSC_LAP_TIME_FACTOR
        return 1.0

    def _phase_vehicle_speed_factor(
        self,
        state: DriverRaceState | None = None,
    ) -> float:
        """Convert race-control pace into a physically safe speed ceiling.

        A car catching the Safety Car queue may exceed its normal target only
        on a segment explicitly classified as straight.  Race control must
        never multiply the calibrated heavy-braking or corner speed above the
        green-flag envelope. Slower VSC, queued-SC and near-queue factors still
        apply fully.
        """
        requested = 1.0 / max(1e-6, self._phase_lap_time_factor(state))
        if self.race_phase == "sc" and requested > 1.0:
            segment = (
                segment_at_progress(self.circuit, state.progress)
                if state is not None
                else None
            )
            return (
                requested
                if segment is not None and segment.type == TrackSegmentType.STRAIGHT
                else 1.0
            )
        return requested

    def _race_phase_remaining_seconds(self) -> float:
        if self.race_phase != "vsc":
            return 0.0
        return max(0.0, self._phase_until - self.race_elapsed)

    def _race_phase_remaining_laps(self) -> int:
        if self.race_phase != "sc":
            return 0
        return 1 if self.safety_car_stage in {"in_this_lap", "restart"} else 0

    def _trigger_vsc(self, events: list[RaceEvent]) -> None:
        """Deploy a virtual safety car (skipped if a full SC is already out)."""
        if self.race_phase == "sc":
            self._cancel_maneuvers_for_neutralization()
            return
        target = self.race_elapsed + VSC_DURATION_SECONDS
        if self.race_phase == "vsc":
            self._cancel_maneuvers_for_neutralization()
            self._phase_until = max(self._phase_until, target)
            return
        self._cancel_maneuvers_for_neutralization()
        self.race_phase = "vsc"
        self._phase_until = target
        events.append(
            RaceEvent(
                type="vsc_start",
                message="Virtual Safety Car deployed",
                message_ko="버추얼 세이프티카가 발동됩니다",
            )
        )

    def _trigger_safety_car(self, events: list[RaceEvent]) -> None:
        """Deploy a full safety car (upgrades an active VSC)."""
        already_sc = self.race_phase == "sc"
        self._cancel_maneuvers_for_neutralization()
        self.race_phase = "sc"
        self.safety_car = True
        cleanup_target = self.race_elapsed + (
            SC_ADDITIONAL_INCIDENT_SECONDS if already_sc else SC_CLEANUP_SECONDS
        )
        self._sc_cleanup_until = max(self._sc_cleanup_until, cleanup_target)
        if not already_sc:
            self.pit_window_open = True
            self._initialize_safety_car_progress()
            self._ai_sc_pit_decisions()
            events.append(
                RaceEvent(
                    type="sc_start",
                    message="Safety Car deployed — it is leaving the pits",
                    message_ko="세이프티카가 피트에서 출동합니다",
                )
            )
            events.append(
                RaceEvent(
                    type="pit_window",
                    message="Pit window open under the safety car",
                    message_ko="세이프티카 동안 피트 기회가 열렸습니다",
                )
            )

    def _initialize_safety_car_progress(self) -> None:
        self._initialize_sc_running_order()
        leader = self._on_track_leader()
        if leader is None:
            self._safety_car_total_progress = None
            self._safety_car_progress_rate = 0.0
            self._safety_car_speed_mps = 0.0
            return
        exit_progress = self._pit_exit_progress()
        if exit_progress is None:
            exit_progress = (leader.progress + SC_LEAD_PROGRESS_GAP) % 1.0
        self._safety_car_total_progress = self._total_progress_at_or_after(
            leader.total_progress,
            exit_progress,
        )
        self._safety_car_progress_rate = 0.0
        self._safety_car_speed_mps = 0.0
        self.safety_car_stage = "deploying"
        self._safety_car_visible = True
        self._safety_car_route = "pit"
        self._safety_car_pit_lane_progress = 0.82
        self._safety_car_queue_formed = False
        self._sc_caught_driver_ids.clear()
        self._sc_unlap_driver_ids.clear()
        self._sc_unlap_targets.clear()
        self._sc_order_yield_targets.clear()
        self._sc_withdraw_target = None
        self._sc_restart_target = None
        self._sc_restart_accel_progress = None

    def _sc_max_gap_progress(self) -> float:
        return (SC_CAR_LENGTH_M * SC_MAX_GAP_CAR_LENGTHS) / self.track_length_m

    def _sc_release_gap_progress(self) -> float:
        return (SC_CAR_LENGTH_M * SC_RELEASE_GAP_CAR_LENGTHS) / self.track_length_m

    def _sc_target_gap_progress(self) -> float:
        return (SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS) / self.track_length_m

    def _safety_car_line_2_progress(self) -> float | None:
        """Return the circuit progress where a pit-out order becomes final."""
        pit_lane = self.circuit.pit_lane
        if pit_lane is None:
            return None
        if pit_lane.safety_car_line_2_progress is not None:
            return pit_lane.safety_car_line_2_progress % 1.0
        return self._pit_exit_progress()

    def _initialize_sc_running_order(self) -> None:
        self._sc_running_order = [
            state.driver_id
            for state in sorted(
                (
                    state
                    for state in self.driver_states.values()
                    if not state.retired and not state.finished and not state.in_pit
                ),
                key=lambda state: state.position,
            )
        ]
        self._sc_pit_exit_order_targets.clear()

    def _remove_from_sc_running_order(self, driver_id: int) -> None:
        self._sc_running_order = [
            item for item in self._sc_running_order if item != driver_id
        ]
        self._sc_pit_exit_order_targets.pop(driver_id, None)
        self._sc_caught_driver_ids.discard(driver_id)
        self._sc_order_yield_targets.pop(driver_id, None)
        stale_yields = [
            yielding_driver_id
            for (
                yielding_driver_id,
                predecessor_id,
            ) in self._sc_order_yield_targets.items()
            if predecessor_id == driver_id
        ]
        for yielding_driver_id in stale_yields:
            self._sc_order_yield_targets.pop(yielding_driver_id, None)

    @staticmethod

    def _insert_by_live_race_distance(
        ordered: list[DriverRaceState],
        state: DriverRaceState,
    ) -> None:
        insert_at = len(ordered)
        for index, other in enumerate(ordered):
            ahead_by_distance = state.total_progress > (
                other.total_progress + PROGRESS_EPSILON
            )
            tied_ahead_by_order = (
                abs(state.total_progress - other.total_progress)
                <= PROGRESS_EPSILON
                and state.position < other.position
            )
            if ahead_by_distance or tied_ahead_by_order:
                insert_at = index
                break
        ordered.insert(insert_at, state)

    def _refresh_sc_running_order(self) -> None:
        """Drop ineligible entries and recover any untracked on-track cars."""
        eligible = {
            state.driver_id: state
            for state in self.driver_states.values()
            if not state.retired
            and not state.finished
            and not state.in_pit
            and state.driver_id not in self._sc_pit_exit_order_targets
        }
        refreshed = [
            driver_id
            for driver_id in self._sc_running_order
            if driver_id in eligible
        ]
        known = set(refreshed)
        missing = sorted(
            (
                state
                for driver_id, state in eligible.items()
                if driver_id not in known
            ),
            key=lambda state: state.position,
        )
        for state in missing:
            ordered_states = [eligible[driver_id] for driver_id in refreshed]
            self._insert_by_live_race_distance(ordered_states, state)
            refreshed = [item.driver_id for item in ordered_states]
        self._sc_running_order = refreshed

    def _sc_ordered_on_track_states(
        self,
        *,
        include_pending: bool = True,
    ) -> list[DriverRaceState]:
        self._refresh_sc_running_order()
        ordered = [
            self.driver_states[driver_id]
            for driver_id in self._sc_running_order
            if driver_id in self.driver_states
        ]
        if include_pending:
            pending = sorted(
                (
                    self.driver_states[driver_id]
                    for driver_id in self._sc_pit_exit_order_targets
                    if driver_id in self.driver_states
                    and not self.driver_states[driver_id].retired
                    and not self.driver_states[driver_id].finished
                    and not self.driver_states[driver_id].in_pit
                ),
                key=lambda state: (-state.total_progress, state.position),
            )
            for state in pending:
                self._insert_by_live_race_distance(ordered, state)
        return ordered

    def _begin_sc_pit_exit_ordering(self, state: DriverRaceState) -> None:
        """Keep a pit-out result provisional until the car reaches SC2."""
        if self.race_phase != "sc":
            return
        self._remove_from_sc_running_order(state.driver_id)
        line_progress = self._safety_car_line_2_progress()
        if line_progress is None:
            target = state.total_progress
        else:
            target = int(state.total_progress // 1.0) + line_progress
        self._sc_pit_exit_order_targets[state.driver_id] = target

    def _commit_ready_sc_pit_exit_orders(self) -> None:
        """Freeze each pit-out car into its physical order at the SC2 line."""
        if self.race_phase != "sc":
            return
        self._refresh_sc_running_order()
        ready = sorted(
            (
                self.driver_states[driver_id]
                for driver_id, target in self._sc_pit_exit_order_targets.items()
                if driver_id in self.driver_states
                and (
                    self.driver_states[driver_id].retired
                    or self.driver_states[driver_id].finished
                    or self.driver_states[driver_id].total_progress
                    >= target - PROGRESS_EPSILON
                )
            ),
            key=lambda state: (-state.total_progress, state.position),
        )
        for state in ready:
            self._sc_pit_exit_order_targets.pop(state.driver_id, None)
            if state.retired or state.finished or state.in_pit:
                continue
            ordered = [
                self.driver_states[driver_id]
                for driver_id in self._sc_running_order
                if driver_id in self.driver_states
            ]
            self._insert_by_live_race_distance(ordered, state)
            self._sc_running_order = [item.driver_id for item in ordered]
            self._sc_caught_driver_ids.discard(state.driver_id)

    def _sc_queue_predecessor(self, state: DriverRaceState) -> DriverRaceState | None:
        """Return the next eligible car ahead in the frozen SC running order."""
        running = [
            candidate
            for candidate in self._sc_ordered_on_track_states()
            if candidate.driver_id not in self._sc_unlap_driver_ids
        ]
        for index, candidate in enumerate(running):
            if candidate.driver_id != state.driver_id:
                continue
            return running[index - 1] if index > 0 else None
        return None

    def _refresh_sc_order_yield_targets(
        self,
        running: list[DriverRaceState],
    ) -> None:
        """Make physical vehicle order converge on the frozen SC sporting order.

        Live timing deliberately keeps the order present at SC deployment (plus
        pit-exit positions committed at SC2).  Graphics use physical progress.
        If a car crosses that order through a lateral corridor, simply freezing
        the timing order leaves both views inconsistent and prevents the queue
        from ever becoming complete.  Persist a give-back target until the
        predecessor is safely ahead again.
        """
        if self.safety_car_stage in {"deploying", "restart"}:
            self._sc_order_yield_targets.clear()
            return

        active_ids = {state.driver_id for state in running}
        self._sc_order_yield_targets = {
            driver_id: predecessor_id
            for driver_id, predecessor_id in self._sc_order_yield_targets.items()
            if driver_id in active_ids
            and (predecessor_id is None or predecessor_id in active_ids)
        }
        release_gap_m = (
            SC_CAR_LENGTH_M * SC_ORDER_RESTORE_RELEASE_CAR_LENGTHS
        )

        for index, state in enumerate(running):
            predecessor = running[index - 1] if index > 0 else None
            predecessor_id = predecessor.driver_id if predecessor is not None else None
            if predecessor is None:
                if self._safety_car_total_progress is None:
                    self._sc_order_yield_targets.pop(state.driver_id, None)
                    continue
                predecessor_total = self._safety_car_total_progress
            else:
                predecessor_total = predecessor.total_progress

            has_existing_target = state.driver_id in self._sc_order_yield_targets
            existing_target = self._sc_order_yield_targets.get(state.driver_id)
            if has_existing_target and existing_target != predecessor_id:
                self._sc_order_yield_targets.pop(state.driver_id, None)
                has_existing_target = False

            signed_gap_m = (
                predecessor_total - state.total_progress
            ) * self.track_length_m
            if has_existing_target:
                if signed_gap_m >= release_gap_m:
                    self._sc_order_yield_targets.pop(state.driver_id, None)
                continue
            if signed_gap_m < -SC_ORDER_RESTORE_TRIGGER_M:
                self._sc_order_yield_targets[state.driver_id] = predecessor_id

    def _sc_queue_control_reaches(
        self,
        state: DriverRaceState,
        visited: set[int] | None = None,
    ) -> bool:
        """Return whether the car is within the queue-tail control horizon."""
        return self._sc_queue_control_distance_m(state, visited) is not None

    def _sc_queue_control_distance_m(
        self,
        state: DriverRaceState,
        visited: set[int] | None = None,
    ) -> float | None:
        """Return cumulative distance back from the caught SC queue.

        The horizon is cumulative rather than 800 m per predecessor.  The
        latter chained across the whole field and slowed distant rear cars
        before they had caught the queue, preventing timely formation.
        """
        if (
            self.race_phase != "sc"
            or self.safety_car_stage in {"deploying", "restart"}
            or self._safety_car_total_progress is None
        ):
            return None
        if state.driver_id in self._sc_caught_driver_ids:
            return 0.0
        seen = set() if visited is None else visited
        if state.driver_id in seen:
            return None
        seen.add(state.driver_id)
        predecessor = self._sc_queue_predecessor(state)
        if predecessor is None:
            ahead_total = self._safety_car_total_progress
            distance_ahead_m = 0.0
        else:
            ahead_total = predecessor.total_progress
            distance_ahead_m = self._sc_queue_control_distance_m(predecessor, seen)
            if distance_ahead_m is None:
                return None
        gap_m = (ahead_total - state.total_progress) * self.track_length_m
        if gap_m < 0.0:
            return None
        cumulative_distance_m = distance_ahead_m + gap_m
        if cumulative_distance_m > SC_QUEUE_PROPAGATION_HORIZON_M:
            return None
        return cumulative_distance_m

    def _sc_lap_time_factor_for_gap(self, state: DriverRaceState) -> float:
        """Choose a smooth catch-up pace from the gap to the queued car ahead."""
        predecessor = self._sc_queue_predecessor(state)
        if predecessor is None:
            ahead_total = self._safety_car_total_progress
            ahead_is_queued = True
        else:
            ahead_total = predecessor.total_progress
            ahead_is_queued = predecessor.driver_id in self._sc_caught_driver_ids

        if ahead_total is None:
            return SC_CATCH_UP_FAST_LAP_TIME_FACTOR

        gap = max(0.0, ahead_total - state.total_progress)
        target_gap = self._sc_target_gap_progress()
        max_gap = self._sc_max_gap_progress()
        caught = state.driver_id in self._sc_caught_driver_ids and ahead_is_queued

        if caught:
            if gap <= target_gap + PROGRESS_EPSILON:
                return SC_LAP_TIME_FACTOR
            recovery_range = max(PROGRESS_EPSILON, max_gap - target_gap)
            recovery_ratio = min(1.0, (gap - target_gap) / recovery_range)
            return SC_LAP_TIME_FACTOR - (
                (SC_LAP_TIME_FACTOR - SC_CAUGHT_RECOVERY_LAP_TIME_FACTOR)
                * recovery_ratio
            )

        if not ahead_is_queued:
            return SC_CATCH_UP_FAST_LAP_TIME_FACTOR

        far_gap = max(max_gap, 0.5)
        approach_range = max(PROGRESS_EPSILON, far_gap - max_gap)
        far_ratio = min(1.0, max(0.0, gap - max_gap) / approach_range)
        return SC_CATCH_UP_NEAR_LAP_TIME_FACTOR - (
            (SC_CATCH_UP_NEAR_LAP_TIME_FACTOR - SC_CATCH_UP_FAST_LAP_TIME_FACTOR)
            * far_ratio
        )

    def _advance_safety_car(self, delta: float, events: list[RaceEvent]) -> None:
        if self.race_phase != "sc" or self._safety_car_total_progress is None:
            self._safety_car_progress_rate = 0.0
            self._safety_car_speed_mps = 0.0
            return

        if self.safety_car_stage == "deploying":
            remaining = 1.0 - self._safety_car_pit_lane_progress
            advance = delta * 0.18 / SC_DEPLOY_PIT_SECONDS
            self._safety_car_pit_lane_progress = min(1.0, self._safety_car_pit_lane_progress + advance)
            self._safety_car_progress_rate = 0.0
            self._safety_car_speed_mps = 0.0
            if remaining <= advance + PROGRESS_EPSILON:
                self._safety_car_route = "track"
                self.safety_car_stage = "collecting"
                events.append(
                    RaceEvent(
                        type="sc_track_join",
                        message="Safety Car joins the track from the Pit Lane exit",
                        message_ko="세이프티카가 피트 출구에서 트랙에 합류합니다",
                    )
                )
            return

        if self.safety_car_stage == "restart":
            self._safety_car_progress_rate = 0.0
            self._safety_car_speed_mps = 0.0
            self._safety_car_pit_lane_progress = min(
                1.0,
                self._safety_car_pit_lane_progress + delta / SC_WITHDRAW_PIT_SECONDS,
            )
            if self._safety_car_pit_lane_progress >= 0.35:
                self._safety_car_visible = False
            return

        previous = self._safety_car_total_progress
        safety_car_distance_m = self._track_physics.line_distance_at_total_progress(
            DRIVING_LINE_RACING,
            self._safety_car_total_progress,
        )
        safety_car_lap_time_factor = (
            SC_COLLECTION_LAP_TIME_FACTOR
            if self.safety_car_stage == "collecting"
            else SC_LAP_TIME_FACTOR
        )
        target_safety_car_speed_mps = self._vehicle_physics.target_speed_mps(
            safety_car_distance_m,
            VehiclePhysicsModifiers(
                speed_limit_factor=1.0 / safety_car_lap_time_factor,
                maximum_speed_mps=SC_CAUGHT_MAX_SPEED_KPH / 3.6,
            ),
        )
        if self.safety_car_stage == "collecting" and not self._sc_caught_driver_ids:
            leader = self._on_track_leader()
            if (
                leader is not None
                and self._safety_car_total_progress - leader.total_progress
                > 2.0 * self._sc_max_gap_progress()
            ):
                # If the SC exits just after the leader has passed the pit
                # exit, it waits at a controlled pace to pick that leader up
                # instead of forcing the field to chase almost a full lap.
                target_safety_car_speed_mps = min(
                    target_safety_car_speed_mps,
                    SC_LEADER_ACQUISITION_SPEED_KPH / 3.6,
                )
        speed_delta_mps = target_safety_car_speed_mps - self._safety_car_speed_mps
        maximum_speed_change_mps = (
            6.0 * delta if speed_delta_mps >= 0.0 else 8.0 * delta
        )
        self._safety_car_speed_mps += max(
            -maximum_speed_change_mps,
            min(maximum_speed_change_mps, speed_delta_mps),
        )
        self._safety_car_progress_rate = (
            self._safety_car_speed_mps / self.track_length_m
        )
        self._safety_car_total_progress += self._safety_car_progress_rate * delta

        if (
            self.safety_car_stage == "in_this_lap"
            and self._sc_withdraw_target is not None
            and previous < self._sc_withdraw_target <= self._safety_car_total_progress
        ):
            self._safety_car_total_progress = self._sc_withdraw_target
            self._safety_car_progress_rate = 0.0
            self._safety_car_speed_mps = 0.0
            self._safety_car_route = "pit"
            self._safety_car_pit_lane_progress = 0.0
            self.safety_car_stage = "restart"
            leader = self._on_track_leader()
            if leader is not None:
                self._sc_restart_target = self._total_progress_at_or_after(
                    leader.total_progress,
                    0.0,
                )
                restart_distance = self.rng.uniform(0.04, 0.09)
                self._sc_restart_accel_progress = max(
                    leader.total_progress,
                    self._sc_restart_target - restart_distance,
                )
            events.append(
                RaceEvent(
                    type="sc_pit",
                    message="Safety Car enters the Pit Lane — the leader controls the restart",
                    message_ko="세이프티카가 피트로 들어가며 선두가 재출발을 통제합니다",
                )
            )

    def _finish_race_phase(self, ended: str, events: list[RaceEvent]) -> None:
        """Reset phase state and publish the matching green-flag event."""
        self.race_phase = "green"
        self.safety_car = False
        self._phase_until = 0.0
        self._safety_car_total_progress = None
        self._safety_car_progress_rate = 0.0
        self._safety_car_speed_mps = 0.0
        self.safety_car_stage = "inactive"
        self._safety_car_visible = False
        self._safety_car_route = "track"
        self._safety_car_pit_lane_progress = 0.0
        self._safety_car_queue_formed = False
        self._sc_cleanup_until = 0.0
        self._sc_caught_driver_ids.clear()
        self._sc_unlap_driver_ids.clear()
        self._sc_unlap_targets.clear()
        self._sc_running_order.clear()
        self._sc_pit_exit_order_targets.clear()
        self._sc_order_yield_targets.clear()
        self._sc_withdraw_target = None
        self._sc_restart_target = None
        self._sc_restart_accel_progress = None
        self.pit_window_open = False
        if ended == "sc":
            events.append(
                RaceEvent(
                    type="sc_end",
                    message="Safety Car in this lap — racing resumes",
                    message_ko="세이프티카가 들어옵니다 — 레이스 재개",
                )
            )
        else:
            events.append(
                RaceEvent(
                    type="vsc_end",
                    message="Virtual Safety Car ending — green flag",
                    message_ko="버추얼 세이프티카 해제 — 그린 플래그",
                )
            )

    def _tick_race_phase(self, delta: float, events: list[RaceEvent]) -> None:
        """Advance VSC timing or the physical Safety Car route."""
        if self.race_phase == "green":
            return
        if self.race_phase == "sc":
            self._advance_safety_car(delta, events)
            return
        if self.race_elapsed >= self._phase_until:
            self._finish_race_phase("vsc", events)

    def _sync_safety_car_queue(self, events: list[RaceEvent]) -> None:
        if self.race_phase != "sc" or self.safety_car_stage in {"deploying", "restart"}:
            return
        if self._safety_car_total_progress is None:
            return

        running = self._sc_ordered_on_track_states()
        self._refresh_sc_order_yield_targets(running)
        active_ids = {state.driver_id for state in running}
        self._sc_caught_driver_ids.intersection_update(active_ids)

        max_gap = self._sc_max_gap_progress()
        release_gap = self._sc_release_gap_progress()
        queue_locked = self.safety_car_stage in {
            "queued",
            "unlapping",
            "in_this_lap",
        }
        ahead_total = self._safety_car_total_progress
        ahead_is_queued = True
        ahead_speed_mps = max(0.0, self._safety_car_speed_mps)

        for state in running:
            if state.driver_id in self._sc_unlap_driver_ids:
                continue

            gap = ahead_total - state.total_progress
            caught = state.driver_id in self._sc_caught_driver_ids
            if caught and not queue_locked and (
                not ahead_is_queued
                or gap < -PROGRESS_EPSILON
                or gap > release_gap + PROGRESS_EPSILON
            ):
                self._sc_caught_driver_ids.discard(state.driver_id)
                caught = False

            if (
                ahead_is_queued
                and gap >= -PROGRESS_EPSILON
                and gap <= max_gap + PROGRESS_EPSILON
                and (
                    (caught and queue_locked)
                    or max(0.0, state.speed_kph / 3.6 - ahead_speed_mps)
                    <= SC_QUEUE_JOIN_MAX_RELATIVE_SPEED_KPH / 3.6
                )
            ):
                self._sc_caught_driver_ids.add(state.driver_id)
                caught = True
            else:
                caught = False

            ahead_total = state.total_progress
            ahead_is_queued = caught
            ahead_speed_mps = max(0.0, state.speed_kph / 3.6)

        queue_ids = {
            state.driver_id
            for state in running
            if state.driver_id not in self._sc_unlap_driver_ids
        }
        formed = bool(queue_ids) and queue_ids.issubset(self._sc_caught_driver_ids)
        was_formed = self._safety_car_queue_formed
        self._safety_car_queue_formed = formed

        if formed and not was_formed and self.safety_car_stage == "collecting":
            self.safety_car_stage = "queued"
            events.append(
                RaceEvent(
                    type="sc_queue",
                    message="The field is queued behind the Safety Car",
                    message_ko="전체 차량이 세이프티카 뒤에 대열을 형성했습니다",
                )
            )
        elif not formed and self.safety_car_stage == "queued":
            self.safety_car_stage = "collecting"

    def _eligible_sc_unlap_drivers(self) -> list[DriverRaceState]:
        leader = self._on_track_leader()
        if leader is None or leader.current_lap >= self.total_laps - 1:
            return []
        return sorted(
            (
                state
                for state in self.driver_states.values()
                if not state.retired
                and not state.finished
                and not state.in_pit
                and state.driver_id != leader.driver_id
                and state.current_lap < leader.current_lap
            ),
            key=lambda state: state.position,
        )

    def _start_sc_unlapping(
        self,
        drivers: list[DriverRaceState],
        events: list[RaceEvent],
    ) -> None:
        self.safety_car_stage = "unlapping"
        self._safety_car_queue_formed = False
        self._sc_unlap_driver_ids = {state.driver_id for state in drivers}
        self._sc_unlap_targets = {
            state.driver_id: state.total_progress + 1.0 for state in drivers
        }
        self._sc_caught_driver_ids.difference_update(self._sc_unlap_driver_ids)
        events.append(
            RaceEvent(
                type="unlap_start",
                message="Lapped cars may now overtake",
                message_ko="랩 다운 차량의 추월이 허용됩니다",
            )
        )

    def _complete_sc_unlapping(self, events: list[RaceEvent]) -> None:
        unlapping = [
            self.driver_states[driver_id]
            for driver_id in self._sc_unlap_driver_ids
            if driver_id in self.driver_states
            and not self.driver_states[driver_id].retired
            and not self.driver_states[driver_id].finished
        ]
        main_queue = sorted(
            (
                state
                for state in self.driver_states.values()
                if not state.retired
                and not state.finished
                and not state.in_pit
                and state.driver_id not in self._sc_unlap_driver_ids
            ),
            key=lambda state: state.position,
        )
        tail_total = (
            main_queue[-1].total_progress
            if main_queue
            else (self._safety_car_total_progress or 0.0) - self._sc_target_gap_progress()
        )
        gap = self._sc_target_gap_progress()
        for index, state in enumerate(sorted(unlapping, key=lambda item: item.position), start=1):
            self._set_state_total_progress(state, tail_total - index * gap)
            state._lap_start_time = state.total_time  # type: ignore[attr-defined]
            self._sc_caught_driver_ids.add(state.driver_id)
            meta = self._driver_meta[state.driver_id]
            events.append(
                RaceEvent(
                    type="unlap",
                    driver=meta["abbreviation"],
                    message=f"{meta['full_name']} rejoins at the back of the Safety Car queue",
                    message_ko=f"{meta['full_name']}가 랩을 회복하고 대열 뒤에 합류합니다",
                )
            )
        self._sc_unlap_driver_ids.clear()
        self._sc_unlap_targets.clear()
        self._safety_car_queue_formed = True

    def _begin_sc_in_this_lap(self, events: list[RaceEvent]) -> None:
        if self.race_phase != "sc" or self.safety_car_stage in {"in_this_lap", "restart"}:
            return
        if self._safety_car_total_progress is None:
            self._initialize_safety_car_progress()
        if self._safety_car_total_progress is None:
            return
        if self.safety_car_stage == "deploying":
            self._safety_car_route = "track"
            self._safety_car_pit_lane_progress = 1.0
        entry_progress = self._pit_entry_progress()
        if entry_progress is None:
            entry_progress = 0.98
        self.safety_car_stage = "in_this_lap"
        self._sc_withdraw_target = self._total_progress_at_or_after(
            self._safety_car_total_progress,
            entry_progress,
        )
        events.append(
            RaceEvent(
                type="sc_in_this_lap",
                message="Safety Car in this lap",
                message_ko="이번 랩에 세이프티카가 들어갑니다",
            )
        )

    def _ai_sc_pit_decisions(self) -> None:
        """Let AI take the cheap "free" pit stop when the safety car is deployed.

        Decided once at SC deployment (not every tick). Only drivers whose tires
        are worn past ``SC_PIT_WEAR_THRESHOLD`` are eligible, and a per-driver
        probability keeps the whole field from stacking into the pits at once.
        """
        for driver_id, state in self.driver_states.items():
            if driver_id in self.player_driver_ids:
                continue
            if (
                state.retired
                or state.finished
                or state.in_pit
                or state.pit_request is not None
            ):
                continue
            remaining = self.total_laps - state.current_lap
            if remaining <= 3:
                continue
            if self._current_tire_wear(state) < SC_PIT_WEAR_THRESHOLD:
                continue
            if self.rng.random() < SC_PIT_PROBABILITY:
                state.pit_request = choose_pit_tire(state, remaining)
