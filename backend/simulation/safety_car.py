"""Safety Car / VSC race-control extracted from RaceEngine.

RaceEngine inherits SafetyCarMixin so existing call sites and tests keep the
same method names on the engine instance.  Behavior is unchanged; this module
only relocates the SC/VSC domain.
"""

from __future__ import annotations

from dataclasses import dataclass, field
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
SC_UNCAUGHT_MIN_CLOSING_SPEED_KPH = 8.0
SC_UNCAUGHT_MAX_CLOSING_SPEED_KPH = 50.0
SC_UNCAUGHT_CLOSING_GAIN_KPH_PER_M = 0.5
SC_ORDER_RESTORE_TRIGGER_M = 1.0
SC_ORDER_RESTORE_RELEASE_CAR_LENGTHS = 1.5
SC_ORDER_RESTORE_SPEED_DELTA_KPH = 18.0
SC_ORDER_RESTORE_APPROACH_MARGIN_KPH = 55.0
SC_ORDER_RESTORE_TIMEOUT_SECONDS = 20.0
SC_ORDER_RESTORE_RETRY_DELAY_SECONDS = 8.0
SC_ORDER_RESTORE_MIN_CLEARANCE_M = PHYSICAL_CAR_LENGTH_M * 0.55
SC_UNLAP_MAX_SPEED_KPH = 280.0
SC_LEADER_ACQUISITION_SPEED_KPH = 5.0
SC_DEPLOY_PIT_SECONDS = 2.5
SC_WITHDRAW_PIT_SECONDS = 3.0
SC_CLEANUP_SECONDS = 35.0
SC_RESTART_CONFIRMATION_TOLERANCE_PROGRESS = 0.05
SC_ADDITIONAL_INCIDENT_SECONDS = 18.0
SC_PIT_WEAR_THRESHOLD = 0.30  # AI takes the "free" SC pit once tires are this worn
SC_PIT_PROBABILITY = 0.4  # per-eligible-driver chance to dive in under SC (avoids all-stop)

SC_QUEUE_STATE_DEPLOYING = "DEPLOYING"
SC_QUEUE_STATE_PICKUP = "PICKUP"
SC_QUEUE_STATE_FORMING = "FORMING"
SC_QUEUE_STATE_STABLE = "STABLE_QUEUE"
SC_QUEUE_STATE_UNLAPPING = "UNLAPPING"
SC_QUEUE_STATE_IN_THIS_LAP = "IN_THIS_LAP"
SC_QUEUE_STATE_RESTART = "RESTART"
SC_QUEUE_STATE_GREEN = "GREEN"

SC_DRIVER_MODE_CATCH_UP = "CATCH_UP"
SC_DRIVER_MODE_APPROACHING = "APPROACHING"
SC_DRIVER_MODE_FOLLOWING = "FOLLOWING"
SC_DRIVER_MODE_STABLE = "STABLE"
SC_DRIVER_MODE_PIT_TRANSIT = "PIT_TRANSIT"
SC_DRIVER_MODE_PIT_EXIT_PENDING = "PIT_EXIT_PENDING"
SC_DRIVER_MODE_UNLAPPING = "UNLAPPING"
SC_DRIVER_MODE_ORDER_RECOVERY = "ORDER_RECOVERY"
SC_DRIVER_MODE_RESTART = "RESTART"

SC_SLOT_TARGET_GAP_M = SC_CAR_LENGTH_M * SC_QUEUE_TARGET_CAR_LENGTHS
SC_QUEUE_STABLE_GAP_TOLERANCE_M = SC_CAR_LENGTH_M * 2.5
SC_QUEUE_STABLE_RELATIVE_SPEED_KPH = 8.0
SC_QUEUE_STABLE_RELATIVE_SPEED_EXIT_KPH = 14.0
SC_QUEUE_STABLE_RELATIVE_SPEED_GRACE_SECONDS = 1.0
SC_QUEUE_RELATIVE_SPEED_FILTER_SECONDS = 0.75
SC_QUEUE_STABLE_SECONDS = 2.5
SC_QUEUE_CATCH_UP_THRESHOLD_M = 500.0
SC_QUEUE_APPROACH_THRESHOLD_M = 50.0
SC_QUEUE_GAP_CONTROLLER_GAIN = 0.25
SC_QUEUE_MAX_CLOSING_SPEED_KPH = 50.0
SC_QUEUE_MAX_OPENING_SPEED_KPH = 25.0


@dataclass
class SafetyCarQueueSlot:
    """One immutable-in-time sporting slot in the active SC train."""

    driver_id: int
    slot_index: int
    status: str = "ACTIVE"
    inserted_reason: str = "SC_DEPLOYED"
    revision: int = 0


@dataclass
class SafetyCarQueuePlan:
    """Event-owned SC order; live physical positions never reorder this list."""

    revision: int
    created_at: float
    slots: list[SafetyCarQueueSlot] = field(default_factory=list)
    pending_pit_exit_ids: set[int] = field(default_factory=set)
    unlapping_ids: set[int] = field(default_factory=set)


@dataclass
class DriverSafetyCarState:
    """Per-driver SC controller state derived from one QueuePlan slot."""

    mode: str
    slot_index: int | None
    predecessor_id: int | None
    gap_error_m: float = 0.0
    relative_speed_kph: float = 0.0
    relative_speed_filtered_kph: float | None = None
    relative_speed_violation_seconds: float = 0.0
    stable_seconds: float = 0.0
    mode_entered_at: float = 0.0


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
        self._sc_order_correction: dict[str, object] | None = None
        self._sc_order_correction_retry_after = 0.0
        self._sc_queue_plan: SafetyCarQueuePlan | None = None
        self._sc_driver_states: dict[int, DriverSafetyCarState] = {}
        self._sc_queue_state = SC_QUEUE_STATE_GREEN
        self._sc_queue_stable_seconds = 0.0
        self._sc_last_queue_sync_elapsed = 0.0
        self._sc_withdraw_target: float | None = None
        self._sc_restart_target: float | None = None
        self._sc_restart_accel_progress: float | None = None
        self._sc_restart_entered_at = 0.0
        self._sc_restart_hold_ticks = 0
        self._safety_car_total_progress: float | None = None
        self._safety_car_progress_rate = 0.0
        self._safety_car_speed_mps = 0.0

    def _race_control_speed_cap_mps(
        self,
        state: DriverRaceState,
        visited: set[int] | None = None,
    ) -> float | None:
        """Return a hard SC speed ceiling in addition to the delta target."""
        if self.race_phase != "sc":
            return None
        seen = set() if visited is None else visited
        if state.driver_id in seen:
            return None
        seen.add(state.driver_id)
        if self.safety_car_stage == "restart":
            # Once the SC has crossed the pit-entry withdrawal target it is
            # stationary in the pit lane.  It must no longer be used as the
            # on-track leader's speed reference: doing so caps the leader at
            # 0 km/h, then propagates that stop through the whole queue while
            # the restart-line transition waits for the leader to move.
            #
            # Before the leader-selected acceleration point the field keeps
            # the ordinary caught-SC ceiling.  From that point onward the
            # calibrated vehicle target and physical predecessor following
            # own the launch; race control no longer supplies a speed cap.
            leader = self._on_track_leader()
            if (
                leader is not None
                and self._sc_restart_accel_progress is not None
                and leader.total_progress
                >= self._sc_restart_accel_progress - PROGRESS_EPSILON
            ):
                return None
            return SC_CAUGHT_MAX_SPEED_KPH / 3.6
        if (
            self._sc_queue_plan is not None
            and self.safety_car_stage not in {"deploying", "restart"}
            and state.driver_id in self._sc_plan_driver_ids()
        ):
            return self._sc_slot_speed_cap_mps(state)
        if state.driver_id in self._sc_unlap_driver_ids:
            return SC_UNLAP_MAX_SPEED_KPH / 3.6
        control = self._sc_driver_states.get(state.driver_id)
        base_cap_mps = (
            SC_CAUGHT_MAX_SPEED_KPH / 3.6
            if self.safety_car_stage in {"queued", "in_this_lap"}
            or (
                control is not None
                and control.mode in {
                    SC_DRIVER_MODE_FOLLOWING,
                    SC_DRIVER_MODE_STABLE,
                }
            )
            else SC_CATCH_UP_MAX_SPEED_KPH / 3.6
        )
        leader = self._on_track_leader()
        if (
            self._sc_leader_acquisition_wait_active()
            and leader is not None
            and state.driver_id == leader.driver_id
        ):
            # The SC itself may be waiting at 5 km/h, but the leader must be
            # allowed to close the gap instead of being capped to the SC's
            # current speed.  The normal SC phase factor and catch-up ceiling
            # still apply to the leader.
            return base_cap_mps
        correction = self._sc_order_correction
        if (
            correction is not None
            and correction.get("yielding_driver_id") == state.driver_id
            and correction.get("phase")
            in {"WAIT_SAFE_ZONE", "MOVE_ASIDE", "YIELDING"}
        ):
            predecessor_id = correction.get("predecessor_driver_id")
            if predecessor_id is None:
                predecessor_speed_mps = max(0.0, self._safety_car_speed_mps)
            else:
                predecessor_state = self.driver_states.get(predecessor_id)
                predecessor_speed_mps = (
                    max(0.0, predecessor_state.speed_kph / 3.6)
                    if predecessor_state is not None
                    else 0.0
                )
            phase = correction.get("phase")
            if phase == "WAIT_SAFE_ZONE":
                # Before the safe corridor opens, keep the yielding car from
                # driving away from its sporting predecessor.  This is a
                # single-pair cap; it is deliberately not propagated through
                # the rest of the queue.
                return min(
                    base_cap_mps,
                    predecessor_speed_mps
                    + SC_ORDER_RESTORE_APPROACH_MARGIN_KPH / 3.6,
                )
            if phase == "MOVE_ASIDE":
                # First open a little longitudinal room while moving into the
                # reserved corridor.  Braking before lateral clearance makes
                # the following predecessor compress onto this car, after
                # which neither body has room to complete the pass.
                return min(
                    base_cap_mps,
                    predecessor_speed_mps
                    + SC_QUEUE_MAX_OPENING_SPEED_KPH / 3.6,
                )
            # During the physical pass the yielding car rolls below its
            # predecessor.  The normal longitudinal model supplies braking;
            # no progress or world pose is snapped.
            return min(
                base_cap_mps,
                max(
                    0.0,
                    max(predecessor_speed_mps, self._safety_car_speed_mps)
                    - SC_ORDER_RESTORE_SPEED_DELTA_KPH / 3.6,
                ),
            )
        if (
            correction is not None
            and correction.get("phase") in {"WAIT_SAFE_ZONE", "MOVE_ASIDE", "YIELDING", "CONFIRM_ORDER", "MERGE_BACK"}
            and state.driver_id != correction.get("yielding_driver_id")
            and state.driver_id not in self._sc_unlap_driver_ids
        ):
            # While one pair is being restored, hold the remaining field at
            # the actual SC pace.  This is a neutral bounded cap, not a
            # predecessor-relative chain, and stops a later inverted pair
            # from driving away before it becomes the active pair.
            neutral_cap_mps = max(0.0, self._safety_car_speed_mps)
            if self._sc_leader_acquisition_wait_active():
                # Do not propagate the SC's temporary 5 km/h acquisition
                # speed to the rest of the field while the leader is closing.
                neutral_cap_mps = max(
                    neutral_cap_mps,
                    SC_CAUGHT_MAX_SPEED_KPH / 3.6,
                )
            return min(base_cap_mps, neutral_cap_mps)
        predecessor = self._sc_queue_predecessor(state)
        if (
            predecessor is not None
            and state.total_progress
            > predecessor.total_progress
            + SC_ORDER_RESTORE_TRIGGER_M / self.track_length_m
        ):
            # A non-active inverted pair is held at the ordinary caught-car
            # SC ceiling.  It is not given a predecessor-relative low cap and
            # therefore cannot form a chain of 65 km/h decisions while the
            # single active pair waits for a safe corridor.
            return min(base_cap_mps, SC_CAUGHT_MAX_SPEED_KPH / 3.6)
        if predecessor is None:
            ahead_total = self._safety_car_total_progress
            ahead_is_queued = True
            ahead_speed_mps = self._safety_car_speed_mps
        else:
            ahead_total = predecessor.total_progress
            predecessor_is_caught = (
                predecessor.driver_id in self._sc_caught_driver_ids
            )
            ahead_is_queued = (
                predecessor_is_caught
                or self._sc_queue_control_reaches(predecessor)
            )
            ahead_speed_mps = max(0.0, predecessor.speed_kph / 3.6)
            if ahead_is_queued and not predecessor_is_caught:
                predecessor_cap_mps = self._race_control_speed_cap_mps(
                    predecessor,
                    seen,
                )
                if predecessor_cap_mps is not None:
                    # Propagate the speed the uncaught predecessor may use
                    # this tick, not only its previous-frame speed.  Otherwise
                    # a long rear train copies 65 km/h one car at a time and
                    # the acceleration wave can take several corners to reach
                    # the tail even though the queue is still far ahead.
                    ahead_speed_mps = max(
                        ahead_speed_mps,
                        predecessor_cap_mps,
                    )
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
            queue_join_gap_m = SC_CAR_LENGTH_M * SC_MAX_GAP_CAR_LENGTHS
            if (
                state.driver_id not in self._sc_caught_driver_ids
                and gap_m > queue_join_gap_m
            ):
                # A high closing speed can consume the whole nominal reaction
                # distance and collapse the hard cap to the predecessor's
                # current speed (often about 65 km/h).  Outside the actual
                # queue-join gap that creates an accelerate/brake oscillation:
                # the car matches 65, accelerates again, then gets clamped
                # back to 65 before it is close enough to be marked caught.
                # Keep a small positive closing margin until the join window;
                # the normal physical following constraint still owns
                # collision avoidance and braking to the local predecessor.
                closing_margin_kph = min(
                    SC_UNCAUGHT_MAX_CLOSING_SPEED_KPH,
                    max(
                        SC_UNCAUGHT_MIN_CLOSING_SPEED_KPH,
                        (gap_m - queue_join_gap_m)
                        * SC_UNCAUGHT_CLOSING_GAIN_KPH_PER_M,
                    ),
                )
                approach_cap_mps = max(
                    approach_cap_mps,
                    min(
                        base_cap_mps,
                        ahead_speed_mps + closing_margin_kph / 3.6,
                    ),
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
        if self._sc_leader_acquisition_wait_active():
            leader = self._on_track_leader()
            if leader is not None and leader.driver_id == state.driver_id:
                # During the special SC acquisition wait, the leader must
                # catch the SC. Applying this constraint here would make the
                # leader match the SC's 5 km/h wait speed and prevent the
                # caught state from ever being reached.
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
            if self._sc_queue_plan is not None:
                control = self._sc_driver_states.get(state.driver_id)
                if control is not None:
                    predecessor = self._sc_queue_predecessor(state)
                    ahead_total = (
                        predecessor.total_progress
                        if predecessor is not None
                        else self._safety_car_total_progress
                    )
                    gap_m = (
                        (ahead_total - state.total_progress) * self.track_length_m
                        if ahead_total is not None
                        else float("inf")
                    )
                    if gap_m <= 0.0 or gap_m > SC_QUEUE_CATCH_UP_THRESHOLD_M:
                        return SC_CATCH_UP_FAST_LAP_TIME_FACTOR
                    if gap_m > SC_QUEUE_APPROACH_THRESHOLD_M:
                        return SC_CATCH_UP_NEAR_LAP_TIME_FACTOR
                    if control.mode in {
                        SC_DRIVER_MODE_FOLLOWING,
                        SC_DRIVER_MODE_STABLE,
                        SC_DRIVER_MODE_ORDER_RECOVERY,
                    }:
                        return SC_LAP_TIME_FACTOR
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
        self._sc_order_correction = None
        self._sc_order_correction_retry_after = 0.0
        self._sc_withdraw_target = None
        self._sc_restart_target = None
        self._sc_restart_accel_progress = None
        self._sc_restart_entered_at = 0.0
        self._sc_restart_hold_ticks = 0

    def _sc_max_gap_progress(self) -> float:
        return (SC_CAR_LENGTH_M * SC_MAX_GAP_CAR_LENGTHS) / self.track_length_m

    def _sc_leader_acquisition_wait_active(self) -> bool:
        """Return whether the SC is waiting for a leader behind it.

        The SC may temporarily travel at ``SC_LEADER_ACQUISITION_SPEED_KPH``
        when it joins the track ahead of the leader.  That speed is only a
        Safety Car control speed: the leader must still be allowed to close
        the gap, otherwise the SC-following constraint creates a deadlock in
        which both vehicles remain at the acquisition speed forever.
        """
        if (
            self.race_phase != "sc"
            or self.safety_car_stage != "collecting"
            or self._safety_car_total_progress is None
        ):
            return False
        leader = self._on_track_leader()
        return (
            leader is not None
            and self._safety_car_total_progress - leader.total_progress
            > 2.0 * self._sc_max_gap_progress()
        )

    def _sc_slot_speed_cap_mps(
        self,
        state: DriverRaceState,
        visited: set[int] | None = None,
    ) -> float | None:
        """Return a continuous gap-controller cap for one QueuePlan slot."""
        plan = self._sc_queue_plan
        if plan is None or state.driver_id not in self._sc_plan_driver_ids():
            return None
        seen = set() if visited is None else visited
        if state.driver_id in seen:
            return None
        seen.add(state.driver_id)
        if state.driver_id in self._sc_unlap_driver_ids:
            return SC_UNLAP_MAX_SPEED_KPH / 3.6

        control = self._sc_driver_states.get(state.driver_id)
        base_cap_mps = (
            SC_CAUGHT_MAX_SPEED_KPH / 3.6
            if self.safety_car_stage in {"queued", "in_this_lap"}
            or (
                control is not None
                and control.mode in {
                    SC_DRIVER_MODE_FOLLOWING,
                    SC_DRIVER_MODE_STABLE,
                }
            )
            else SC_CATCH_UP_MAX_SPEED_KPH / 3.6
        )
        leader = self._on_track_leader()
        if (
            self._sc_leader_acquisition_wait_active()
            and leader is not None
            and state.driver_id == leader.driver_id
        ):
            # The SC may wait at 5 km/h, but Slot 0 must be free to catch it.
            return base_cap_mps

        correction = self._sc_order_correction
        active_correction_phases = {
            "WAIT_SAFE_ZONE",
            "MOVE_ASIDE",
            "YIELDING",
            "CONFIRM_ORDER",
            "MERGE_BACK",
        }
        if (
            correction is not None
            and correction.get("phase") in active_correction_phases
        ):
            yielding_id = correction.get("yielding_driver_id")
            if state.driver_id == yielding_id and correction.get("phase") in {
                "WAIT_SAFE_ZONE",
                "MOVE_ASIDE",
                "YIELDING",
            }:
                predecessor_id = correction.get("predecessor_driver_id")
                predecessor = (
                    self.driver_states.get(predecessor_id)
                    if isinstance(predecessor_id, int)
                    else None
                )
                predecessor_speed_mps = (
                    max(0.0, predecessor.speed_kph / 3.6)
                    if predecessor is not None
                    else max(0.0, self._safety_car_speed_mps)
                )
                phase = correction.get("phase")
                if phase == "WAIT_SAFE_ZONE":
                    return min(
                        base_cap_mps,
                        predecessor_speed_mps
                        + SC_ORDER_RESTORE_APPROACH_MARGIN_KPH / 3.6,
                    )
                if phase == "MOVE_ASIDE":
                    return min(
                        base_cap_mps,
                        predecessor_speed_mps
                        + SC_QUEUE_MAX_OPENING_SPEED_KPH / 3.6,
                    )
                return min(
                    base_cap_mps,
                    max(
                        0.0,
                        max(predecessor_speed_mps, self._safety_car_speed_mps)
                        - SC_ORDER_RESTORE_SPEED_DELTA_KPH / 3.6,
                    ),
                )

            # QueuePlan slot caps for the remaining cars may point to another
            # still-inverted sporting predecessor.  While one adjacent pass is
            # active, overriding those chained targets with the actual SC pace
            # prevents the rest of the field from propagating a zero-speed cap.
            neutral_cap_mps = max(0.0, self._safety_car_speed_mps)
            if self._sc_leader_acquisition_wait_active():
                neutral_cap_mps = max(
                    neutral_cap_mps,
                    SC_CAUGHT_MAX_SPEED_KPH / 3.6,
                )
            return min(base_cap_mps, neutral_cap_mps)

        predecessor = self._sc_queue_predecessor(state)
        if predecessor is None:
            ahead_total = self._safety_car_total_progress
            ahead_speed_mps = max(0.0, self._safety_car_speed_mps)
        else:
            ahead_total = predecessor.total_progress
            ahead_speed_mps = max(0.0, predecessor.speed_kph / 3.6)
        if ahead_total is None:
            return base_cap_mps

        gap_m = (ahead_total - state.total_progress) * self.track_length_m
        if predecessor is None and gap_m <= 0.0:
            # The SC has not reached Slot 0 yet; do not make the leader match
            # a slower SC that is physically behind it.
            return base_cap_mps
        if predecessor is not None and gap_m <= 0.0:
            # A hazard can temporarily put the sporting predecessor behind
            # this body.  Its gap error is not a usable forward speed target:
            # clamping it to zero would trap the car and create a multi-car
            # 5 km/h tail.  Physical collision following and the single-pair
            # order-recovery corridor remain authoritative until the order is
            # valid again.
            order_recovery_active = (
                self._sc_order_yield_targets.get(state.driver_id)
                == predecessor.driver_id
                or (
                    isinstance(self._sc_order_correction, dict)
                    and self._sc_order_correction.get("yielding_driver_id")
                    == state.driver_id
                    and self._sc_order_correction.get("predecessor_driver_id")
                    == predecessor.driver_id
                )
            )
            hazard_recovery_active = getattr(self, "_hazard_recovery_active", None)
            if (
                callable(hazard_recovery_active)
                and hazard_recovery_active(state)
            ):
                return base_cap_mps
            inversion_timeout_reached = (
                control is not None
                and control.mode == SC_DRIVER_MODE_CATCH_UP
                and self.race_elapsed - control.mode_entered_at
                >= SC_ORDER_RESTORE_TIMEOUT_SECONDS
            )
            if inversion_timeout_reached:
                # A corridor that made no measurable progress has timed out;
                # release its low cap and let the physical controller find a
                # safe route while the sporting order remains frozen.
                return base_cap_mps
            suspended_order_recovery = (
                isinstance(self._sc_order_correction, dict)
                and self._sc_order_correction.get("phase") == "SUSPENDED"
                and self._sc_order_correction.get("yielding_driver_id")
                == state.driver_id
            )
            if suspended_order_recovery and predecessor.speed_kph <= 5.0:
                # Do not let a timed-out pair inherit a stopped predecessor's
                # zero target forever.  Once the physical head is moving, the
                # normal opening correction is restored below.
                return base_cap_mps
            waiting_without_corridor = (
                isinstance(self._sc_order_correction, dict)
                and self._sc_order_correction.get("phase") == "WAIT_SAFE_ZONE"
                and self._sc_order_correction.get("lateral_target_m") is None
                and self._sc_order_correction.get("yielding_driver_id")
                == state.driver_id
            )
            if waiting_without_corridor:
                # There is no legal lateral corridor at this location.  Do
                # not hold the physical head at a zero target while waiting
                # for a corridor that cannot exist; the retry/timeout state
                # machine will revisit the pair at a later safe zone.
                return base_cap_mps
            if order_recovery_active:
                recovery_cap_mps = max(
                    0.0,
                    predecessor.speed_kph / 3.6
                    - SC_QUEUE_MAX_OPENING_SPEED_KPH / 3.6,
                )
                return min(base_cap_mps, recovery_cap_mps)
            # Outside a hazard recovery window, retain a bounded opening
            # correction so a single inverted pair still gives its place back
            # instead of being released into an unconstrained physical race.
            return min(
                base_cap_mps,
                max(
                    0.0,
                    predecessor.speed_kph / 3.6
                    - SC_QUEUE_MAX_OPENING_SPEED_KPH / 3.6,
                ),
            )
        predecessor_control = (
            self._sc_driver_states.get(predecessor.driver_id)
            if predecessor is not None
            else None
        )
        predecessor_has_upstream_slot = (
            predecessor is not None
            and self._sc_queue_predecessor(predecessor) is not None
        )
        should_propagate_predecessor_target = (
            predecessor is not None
            and (
                gap_m - SC_SLOT_TARGET_GAP_M > SC_QUEUE_APPROACH_THRESHOLD_M
                or (
                    predecessor_has_upstream_slot
                    and predecessor_control is not None
                    and predecessor_control.mode
                    in {SC_DRIVER_MODE_CATCH_UP, SC_DRIVER_MODE_APPROACHING}
                )
            )
        )
        if should_propagate_predecessor_target:
            predecessor_cap_mps = self._sc_slot_speed_cap_mps(predecessor, seen)
            if predecessor_cap_mps is not None:
                # Propagate a catch-up wave only while this slot is itself
                # materially open. At the target gap, use the predecessor's
                # measured speed; otherwise a temporary cap from an earlier
                # slot can make a stable car accelerate for no reason.
                ahead_speed_mps = max(ahead_speed_mps, predecessor_cap_mps)
        gap_error_m = gap_m - SC_SLOT_TARGET_GAP_M
        if control is not None:
            control.gap_error_m = round(gap_error_m, 3)
        correction_mps = max(
            -SC_QUEUE_MAX_OPENING_SPEED_KPH / 3.6,
            min(
                SC_QUEUE_MAX_CLOSING_SPEED_KPH / 3.6,
                gap_error_m * SC_QUEUE_GAP_CONTROLLER_GAIN,
            ),
        )
        target_speed_mps = max(0.0, ahead_speed_mps + correction_mps)
        return min(base_cap_mps, target_speed_mps)

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
        ordered_ids = [
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
        self._sc_queue_plan = SafetyCarQueuePlan(
            revision=1,
            created_at=self.race_elapsed,
            slots=[
                SafetyCarQueueSlot(
                    driver_id=driver_id,
                    slot_index=index,
                    revision=1,
                )
                for index, driver_id in enumerate(ordered_ids)
            ],
        )
        self._sc_driver_states = {
            driver_id: DriverSafetyCarState(
                mode=SC_DRIVER_MODE_CATCH_UP,
                slot_index=index,
                predecessor_id=(ordered_ids[index - 1] if index else None),
                mode_entered_at=self.race_elapsed,
            )
            for index, driver_id in enumerate(ordered_ids)
        }
        self._sc_running_order = list(ordered_ids)
        self._sc_pit_exit_order_targets.clear()
        self._sc_queue_state = SC_QUEUE_STATE_DEPLOYING
        self._sc_queue_stable_seconds = 0.0
        self._sc_last_queue_sync_elapsed = self.race_elapsed

    def _sc_plan_driver_ids(self, *, include_unlapping: bool = False) -> list[int]:
        plan = self._sc_queue_plan
        if plan is None:
            return list(self._sc_running_order)
        ids = [
            slot.driver_id
            for slot in plan.slots
            if slot.status == "ACTIVE"
            and (
                include_unlapping
                or slot.driver_id not in plan.unlapping_ids
                and slot.driver_id not in self._sc_unlap_driver_ids
            )
        ]
        return ids

    def _reindex_sc_queue_plan(self) -> None:
        plan = self._sc_queue_plan
        if plan is None:
            return
        for index, slot in enumerate(plan.slots):
            slot.slot_index = index
            slot.revision = plan.revision
        self._sc_running_order = [slot.driver_id for slot in plan.slots]
        active_ids = set(self._sc_running_order)
        for index, driver_id in enumerate(self._sc_running_order):
            control = self._sc_driver_states.get(driver_id)
            if control is None:
                control = DriverSafetyCarState(
                    mode=SC_DRIVER_MODE_APPROACHING,
                    slot_index=index,
                    predecessor_id=(
                        self._sc_running_order[index - 1] if index else None
                    ),
                    mode_entered_at=self.race_elapsed,
                )
                self._sc_driver_states[driver_id] = control
            control.slot_index = index
            control.predecessor_id = (
                self._sc_running_order[index - 1] if index else None
            )
        for driver_id in list(self._sc_driver_states):
            if (
                driver_id not in active_ids
                and driver_id not in plan.pending_pit_exit_ids
                and driver_id not in plan.unlapping_ids
            ):
                self._sc_driver_states.pop(driver_id, None)

    def _sc_update_queue_plan_revision(self, reason: str) -> None:
        del reason  # The reason is carried by the inserted slot/event caller.
        plan = self._sc_queue_plan
        if plan is None:
            return
        plan.revision += 1
        self._reindex_sc_queue_plan()

    def _remove_from_sc_running_order(self, driver_id: int) -> None:
        plan = self._sc_queue_plan
        if plan is None:
            self._sc_running_order = [
                item for item in self._sc_running_order if item != driver_id
            ]
        else:
            before = len(plan.slots)
            plan.slots = [
                slot for slot in plan.slots if slot.driver_id != driver_id
            ]
            plan.pending_pit_exit_ids.discard(driver_id)
            plan.unlapping_ids.discard(driver_id)
            if len(plan.slots) != before:
                self._sc_update_queue_plan_revision("EVENT")
        self._sc_pit_exit_order_targets.pop(driver_id, None)
        self._sc_caught_driver_ids.discard(driver_id)
        self._sc_order_yield_targets.pop(driver_id, None)
        if plan is not None:
            self._sc_driver_states[driver_id] = DriverSafetyCarState(
                mode=(
                    SC_DRIVER_MODE_PIT_TRANSIT
                    if self.driver_states.get(driver_id) is not None
                    and self.driver_states[driver_id].in_pit
                    else SC_DRIVER_MODE_PIT_EXIT_PENDING
                ),
                slot_index=None,
                predecessor_id=None,
                mode_entered_at=self.race_elapsed,
            )
        correction = self._sc_order_correction
        if correction is not None and driver_id in {
            correction.get("yielding_driver_id"),
            correction.get("predecessor_driver_id"),
        }:
            self._sc_order_correction = None
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
        """Mirror the event-owned QueuePlan without live-distance reordering."""
        if self._sc_queue_plan is not None:
            stale_ids = [
                slot.driver_id
                for slot in self._sc_queue_plan.slots
                if slot.driver_id in self.driver_states
                and (
                    self.driver_states[slot.driver_id].retired
                    or self.driver_states[slot.driver_id].finished
                    or self.driver_states[slot.driver_id].in_pit
                )
            ]
            for driver_id in stale_ids:
                self._remove_from_sc_running_order(driver_id)
            self._reindex_sc_queue_plan()
            return
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
            pending_ids = set(self._sc_pit_exit_order_targets)
            if self._sc_queue_plan is not None:
                pending_ids.update(self._sc_queue_plan.unlapping_ids)
            pending = sorted(
                (
                    self.driver_states[driver_id]
                    for driver_id in pending_ids
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
        if self._sc_queue_plan is not None:
            self._sc_queue_plan.pending_pit_exit_ids.add(state.driver_id)
            self._sc_driver_states[state.driver_id] = DriverSafetyCarState(
                mode=SC_DRIVER_MODE_PIT_EXIT_PENDING,
                slot_index=None,
                predecessor_id=None,
                mode_entered_at=self.race_elapsed,
            )

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
            if self._sc_queue_plan is not None:
                self._sc_queue_plan.pending_pit_exit_ids.discard(state.driver_id)
            if state.retired or state.finished or state.in_pit:
                continue
            if self._sc_queue_plan is None:
                ordered = [
                    self.driver_states[driver_id]
                    for driver_id in self._sc_running_order
                    if driver_id in self.driver_states
                ]
                self._insert_by_live_race_distance(ordered, state)
                self._sc_running_order = [item.driver_id for item in ordered]
            else:
                ordered_ids = list(self._sc_running_order)
                insert_at = len(ordered_ids)
                for index, driver_id in enumerate(ordered_ids):
                    candidate = self.driver_states.get(driver_id)
                    if candidate is not None and state.total_progress > candidate.total_progress:
                        insert_at = index
                        break
                ordered_ids.insert(insert_at, state.driver_id)
                self._sc_queue_plan.slots = [
                    SafetyCarQueueSlot(
                        driver_id=driver_id,
                        slot_index=index,
                        inserted_reason="PIT_EXIT_SC2",
                        revision=self._sc_queue_plan.revision + 1,
                    )
                    for index, driver_id in enumerate(ordered_ids)
                ]
                self._sc_update_queue_plan_revision("PIT_EXIT_SC2")
                self._sc_driver_states[state.driver_id] = DriverSafetyCarState(
                    mode=SC_DRIVER_MODE_APPROACHING,
                    slot_index=insert_at,
                    predecessor_id=(
                        ordered_ids[insert_at - 1] if insert_at else None
                    ),
                    mode_entered_at=self.race_elapsed,
                )
            self._sc_caught_driver_ids.discard(state.driver_id)

    def _sc_queue_predecessor(self, state: DriverRaceState) -> DriverRaceState | None:
        """Return the predecessor from the fixed QueuePlan, never live order."""
        ordered_ids = self._sc_plan_driver_ids()
        for index, driver_id in enumerate(ordered_ids):
            if driver_id == state.driver_id and index > 0:
                return self.driver_states.get(ordered_ids[index - 1])
        return None

    def _refresh_sc_order_yield_targets(
        self,
        running: list[DriverRaceState],
    ) -> None:
        """Advance one bounded physical restoration pair.

        ``_sc_running_order`` is the sporting order and remains frozen.  This
        controller is the only place that uses the sporting predecessor to
        make a physical car give a place back.  The regular physics controller
        continues to use the nearest body in the actual corridor.
        """
        if self.safety_car_stage in {"deploying", "restart", "unlapping"}:
            self._sc_order_yield_targets.clear()
            self._sc_order_correction = None
            return

        active_ids = {state.driver_id for state in running}
        correction = self._sc_order_correction
        if correction is not None:
            yielding_id = correction.get("yielding_driver_id")
            predecessor_id = correction.get("predecessor_driver_id")
            yielding = self.driver_states.get(yielding_id) if isinstance(yielding_id, int) else None
            predecessor = (
                self.driver_states.get(predecessor_id)
                if isinstance(predecessor_id, int)
                else None
            )
            if (
                yielding is None
                or predecessor is None
                or yielding.driver_id not in active_ids
                or predecessor.driver_id not in active_ids
                or not self._sc_order_correction_is_eligible(yielding, predecessor)
            ):
                self._suspend_sc_order_correction()
                return

            signed_gap_m = self._sc_order_signed_gap_m(predecessor, yielding)
            self._update_sc_order_correction_progress(correction, signed_gap_m)
            phase = correction.get("phase")
            release_gap_m = SC_CAR_LENGTH_M * SC_ORDER_RESTORE_RELEASE_CAR_LENGTHS

            if phase == "SUSPENDED":
                self._sc_order_yield_targets.clear()
                if self.race_elapsed < self._sc_order_correction_retry_after:
                    return
                self._sc_order_correction = None
                correction = None
            elif (
                phase in {"WAIT_SAFE_ZONE", "MOVE_ASIDE"}
                and signed_gap_m >= release_gap_m
            ):
                # A manually resolved or already-safe pair needs no lateral
                # command.  Remove it before another pair can be selected.
                correction["completion_confirmed"] = True
                correction["phase"] = "COMPLETE"
                self._clear_completed_sc_order_correction()
                return
            elif phase == "WAIT_SAFE_ZONE":
                target = self._sc_order_correction_lateral_target(yielding, predecessor)
                if target is not None:
                    correction["lateral_target_m"] = target
                    correction["phase"] = "MOVE_ASIDE"
            elif phase == "MOVE_ASIDE":
                target = correction.get("lateral_target_m")
                if not isinstance(target, (int, float)):
                    target = self._sc_order_correction_lateral_target(yielding, predecessor)
                    correction["lateral_target_m"] = target
                elif abs(float(target) - predecessor.lateral_offset_m) < (
                    self._sc_order_required_lateral_clearance(yielding, predecessor) - 0.10
                ):
                    refreshed_target = self._sc_order_correction_lateral_target(
                        yielding,
                        predecessor,
                    )
                    if refreshed_target is not None:
                        correction["lateral_target_m"] = refreshed_target
                        target = refreshed_target
                if not isinstance(target, (int, float)) or abs(
                    float(target) - predecessor.lateral_offset_m
                ) < self._sc_order_required_lateral_clearance(yielding, predecessor) - 0.10:
                    self._suspend_sc_order_correction()
                    return
                if (
                    isinstance(target, (int, float))
                    and abs(yielding.lateral_offset_m - predecessor.lateral_offset_m)
                    >= self._sc_order_required_lateral_clearance(yielding, predecessor)
                ):
                    correction["phase"] = "YIELDING"
            elif phase == "YIELDING":
                target = correction.get("lateral_target_m")
                if (
                    isinstance(target, (int, float))
                    and abs(float(target) - predecessor.lateral_offset_m)
                    < self._sc_order_required_lateral_clearance(yielding, predecessor) - 0.10
                ):
                    refreshed_target = self._sc_order_correction_lateral_target(
                        yielding,
                        predecessor,
                    )
                    if refreshed_target is not None:
                        correction["lateral_target_m"] = refreshed_target
                    else:
                        self._suspend_sc_order_correction()
                        return
                if signed_gap_m >= release_gap_m:
                    correction["completion_confirmed"] = True
                    correction["phase"] = "CONFIRM_ORDER"
            elif phase == "CONFIRM_ORDER":
                if signed_gap_m >= release_gap_m:
                    correction["phase"] = "MERGE_BACK"
            elif phase == "MERGE_BACK":
                line_offset = self._track_physics_for_driver(yielding).line_offset_at_progress(
                    DRIVING_LINE_RACING,
                    yielding.progress,
                )
                if abs(yielding.lateral_offset_m - line_offset) <= 0.35:
                    correction["phase"] = "COMPLETE"
            elif phase == "COMPLETE":
                self._clear_completed_sc_order_correction()
                return

            if correction is not None and correction.get("phase") not in {"SUSPENDED", "COMPLETE"}:
                self._sc_order_yield_targets = {
                    yielding.driver_id: predecessor.driver_id,
                }
                if float(correction.get("no_progress_time_s", 0.0)) >= SC_ORDER_RESTORE_TIMEOUT_SECONDS:
                    self._suspend_sc_order_correction()
                    return

        if correction is not None:
            return
        if self.race_elapsed < self._sc_order_correction_retry_after:
            self._sc_order_yield_targets.clear()
            return

        # Restore one *physically adjacent* inverted pair at a time.  Selecting
        # adjacent slots in sporting order is unsafe when more than two cars
        # are inverted: another body can sit between that pair, so the yielding
        # car brakes in front of the intervening train while the sporting
        # predecessor can never reach the reserved corridor.  The result is a
        # stationary SC tail.  Bubble the physical order toward QueuePlan order
        # instead; every authorized pass then involves neighbouring bodies.
        sporting_rank = {
            state.driver_id: index for index, state in enumerate(running)
        }
        physical_order = sorted(
            running,
            key=lambda state: (-state.total_progress, state.position),
        )
        candidates: list[tuple[float, DriverRaceState, DriverRaceState, float]] = []
        for yielding, predecessor in zip(physical_order, physical_order[1:]):
            if sporting_rank[yielding.driver_id] < sporting_rank[predecessor.driver_id]:
                continue
            if not self._sc_order_correction_is_eligible(yielding, predecessor):
                continue
            signed_gap_m = self._sc_order_signed_gap_m(predecessor, yielding)
            if signed_gap_m >= -SC_ORDER_RESTORE_TRIGGER_M:
                continue
            candidates.append(
                (yielding.total_progress, predecessor, yielding, signed_gap_m)
            )

        if candidates:
            _, predecessor, yielding, signed_gap_m = max(
                candidates,
                key=lambda item: item[0],
            )
            self._sc_order_correction = {
                "phase": "WAIT_SAFE_ZONE",
                "yielding_driver_id": yielding.driver_id,
                "predecessor_driver_id": predecessor.driver_id,
                "started_at_s": self.race_elapsed,
                "last_progress_at_s": self.race_elapsed,
                "last_update_elapsed_s": self.race_elapsed,
                "signed_gap_m": round(signed_gap_m, 3),
                "previous_gap_m": round(signed_gap_m, 3),
                "no_progress_time_s": 0.0,
                "lateral_target_m": None,
                "corridor": None,
                "completion_confirmed": False,
                "retry_count": 0,
            }
            target = self._sc_order_correction_lateral_target(yielding, predecessor)
            if target is not None:
                self._sc_order_correction["phase"] = "MOVE_ASIDE"
                self._sc_order_correction["lateral_target_m"] = target
            self._sc_order_yield_targets = {
                yielding.driver_id: predecessor.driver_id,
            }
            return

        self._sc_order_yield_targets.clear()

    def _sc_order_signed_gap_m(
        self,
        predecessor: DriverRaceState,
        yielding: DriverRaceState,
    ) -> float:
        return (predecessor.total_progress - yielding.total_progress) * self.track_length_m

    def _sc_order_correction_is_eligible(
        self,
        yielding: DriverRaceState,
        predecessor: DriverRaceState,
    ) -> bool:
        if self.race_phase != "sc" or self.safety_car_stage in {
            "deploying",
            "restart",
            "unlapping",
        }:
            return False
        if any(
            state.retired
            or state.finished
            or state.in_pit
            or state.driver_id in self._sc_unlap_driver_ids
            or state.driver_id in self._sc_pit_exit_order_targets
            or state.hazard_active
            or state.avoidance_active
            for state in (yielding, predecessor)
        ):
            return False
        active_hazards = getattr(self, "_active_stopped_hazards", None)
        if callable(active_hazards) and active_hazards():
            return False
        return True

    def _sc_order_safe_zone(self, state: DriverRaceState) -> bool:
        segment = segment_at_progress(self.circuit, state.progress)
        if segment is None or segment.type != TrackSegmentType.STRAIGHT:
            return False
        if not getattr(segment, "side_by_side_allowed", True):
            return False
        minimum, maximum = self._track_surface.trajectory_body_lateral_bounds(
            state.progress,
            body_width_m=state.car_width_m,
            edge_margin_m=0.20,
        )
        return maximum - minimum >= state.car_width_m * 2.0 + 0.60

    def _sc_order_required_lateral_clearance(
        self,
        yielding: DriverRaceState,
        predecessor: DriverRaceState,
    ) -> float:
        return max(
            SC_ORDER_RESTORE_MIN_CLEARANCE_M,
            0.5 * (yielding.car_width_m + predecessor.car_width_m) + 0.45,
        )

    def _sc_order_correction_lateral_target(
        self,
        yielding: DriverRaceState,
        predecessor: DriverRaceState,
    ) -> float | None:
        if not self._sc_order_safe_zone(yielding):
            return None
        profile = self._track_physics_for_driver(yielding)
        racing_offset = profile.line_offset_at_progress(
            DRIVING_LINE_RACING,
            yielding.progress,
        )
        minimum, maximum = self._track_surface.trajectory_body_lateral_bounds(
            yielding.progress,
            body_width_m=yielding.car_width_m,
            edge_margin_m=0.20,
        )
        clearance = self._sc_order_required_lateral_clearance(yielding, predecessor)
        correction = self._sc_order_correction
        corridor = correction.get("corridor") if correction is not None else None
        if corridor == "left":
            candidates = [max(minimum, min(maximum, racing_offset - clearance))]
        elif corridor == "right":
            candidates = [max(minimum, min(maximum, racing_offset + clearance))]
        else:
            candidates = [
                max(minimum, min(maximum, racing_offset - clearance)),
                max(minimum, min(maximum, racing_offset + clearance)),
            ]
        other_states = [
            state
            for state in self.driver_states.values()
            if state.driver_id not in {yielding.driver_id, predecessor.driver_id}
            and not state.retired
            and not state.finished
            and not state.in_pit
        ]
        valid: list[float] = []
        for candidate in candidates:
            if abs(candidate - predecessor.lateral_offset_m) < clearance:
                continue
            blocked = False
            for other in other_states:
                if abs(other.total_progress - yielding.total_progress) * self.track_length_m > 35.0:
                    continue
                required = 0.5 * (yielding.car_width_m + other.car_width_m) + 0.45
                if abs(candidate - other.lateral_offset_m) < required:
                    blocked = True
                    break
            if not blocked:
                valid.append(candidate)
        if not valid:
            return None
        selected = max(valid, key=lambda candidate: abs(candidate - racing_offset))
        if correction is not None and correction.get("corridor") is None:
            correction["corridor"] = "left" if selected < racing_offset else "right"
        return selected

    def _update_sc_order_correction_progress(
        self,
        correction: dict[str, object],
        signed_gap_m: float,
    ) -> None:
        previous = float(correction.get("signed_gap_m", signed_gap_m))
        last_update = float(correction.get("last_update_elapsed_s", self.race_elapsed))
        elapsed = max(0.0, self.race_elapsed - last_update)
        correction["last_update_elapsed_s"] = self.race_elapsed
        correction["previous_gap_m"] = round(previous, 3)
        correction["signed_gap_m"] = round(signed_gap_m, 3)
        if abs(signed_gap_m - previous) >= 0.10:
            correction["last_progress_at_s"] = self.race_elapsed
            correction["no_progress_time_s"] = 0.0
        else:
            correction["no_progress_time_s"] = round(
                float(correction.get("no_progress_time_s", 0.0)) + elapsed,
                3,
            )

    def _suspend_sc_order_correction(self) -> None:
        correction = self._sc_order_correction
        retry_count = (
            int(correction.get("retry_count", 0)) + 1
            if correction is not None
            else 1
        )
        if correction is not None:
            correction["phase"] = "SUSPENDED"
            correction["lateral_target_m"] = None
            correction["retry_count"] = min(3, retry_count)
        self._sc_order_yield_targets.clear()
        self._sc_order_correction_retry_after = (
            self.race_elapsed + SC_ORDER_RESTORE_RETRY_DELAY_SECONDS
        )

    def _clear_completed_sc_order_correction(self) -> None:
        self._sc_order_yield_targets.clear()
        self._sc_order_correction = None

    def safety_car_diagnostic_snapshot(self) -> dict[str, object]:
        """Return bounded SC control facts without pose or dashboard payloads."""
        running = self._sc_ordered_on_track_states() if self.race_phase == "sc" else []
        physical = sorted(running, key=lambda state: (-state.total_progress, state.position))
        sporting_index = {state.driver_id: index for index, state in enumerate(running)}
        physical_index = {state.driver_id: index for index, state in enumerate(physical)}
        correction = self._sc_order_correction
        correction_snapshot = None
        if correction is not None:
            correction_snapshot = {
                key: correction.get(key)
                for key in (
                    "phase",
                    "yielding_driver_id",
                    "predecessor_driver_id",
                    "started_at_s",
                    "signed_gap_m",
                    "previous_gap_m",
                    "no_progress_time_s",
                    "lateral_target_m",
                    "corridor",
                    "completion_confirmed",
                    "retry_count",
                )
            }

        drivers: list[dict[str, object]] = []
        for state in running[:64]:
            predecessor = self._sc_queue_predecessor(state)
            cap_mps = self._race_control_speed_cap_mps(state)
            control = self._sc_driver_states.get(state.driver_id)
            if self.safety_car_stage == "restart":
                cap_source = "restart_leader_control"
            elif state.driver_id in self._sc_unlap_driver_ids:
                cap_source = "unlap"
            elif (
                correction is not None
                and correction.get("yielding_driver_id") == state.driver_id
            ):
                cap_source = f"order_correction:{correction.get('phase')}"
            elif state.driver_id in self._sc_caught_driver_ids:
                cap_source = "caught_sc"
            elif self._sc_queue_control_reaches(state):
                cap_source = "queue_approach"
            else:
                cap_source = "catch_up"
            physical_leader = None
            nearest = getattr(self, "_nearest_physical_following_leader", None)
            if callable(nearest):
                physical_leader = nearest(
                    state,
                    {
                        item.driver_id: (
                            item.total_progress,
                            max(0.0, item.speed_kph / 3.6),
                        )
                        for item in running
                    },
                    None,
                )
            drivers.append(
                {
                    "driver_id": state.driver_id,
                    "slot_index": control.slot_index if control is not None else None,
                    "mode": control.mode if control is not None else None,
                    "gap_error_m": (
                        round(control.gap_error_m, 3) if control is not None else None
                    ),
                    "relative_speed_kph": (
                        round(control.relative_speed_kph, 3)
                        if control is not None
                        else None
                    ),
                    "relative_speed_filtered_kph": (
                        round(control.relative_speed_filtered_kph, 3)
                        if control is not None
                        and control.relative_speed_filtered_kph is not None
                        else None
                    ),
                    "relative_speed_violation_seconds": (
                        round(control.relative_speed_violation_seconds, 3)
                        if control is not None
                        else 0.0
                    ),
                    "stable_seconds": (
                        round(control.stable_seconds, 3)
                        if control is not None
                        else 0.0
                    ),
                    "sporting_index": sporting_index.get(state.driver_id),
                    "physical_index": physical_index.get(state.driver_id),
                    "speed_kph": round(state.speed_kph, 3),
                    "race_control_cap_kph": (
                        round(cap_mps * 3.6, 3) if cap_mps is not None else None
                    ),
                    "cap_source": cap_source,
                    "predecessor_driver_id": (
                        predecessor.driver_id if predecessor is not None else None
                    ),
                    "signed_gap_m": (
                        round(self._sc_order_signed_gap_m(predecessor, state), 3)
                        if predecessor is not None
                        else None
                    ),
                    "caught": state.driver_id in self._sc_caught_driver_ids,
                    "emergency": bool(getattr(state, "emergency_braking", False)),
                    "in_pit": bool(state.in_pit),
                    "pit_phase": getattr(state, "pit_phase", None),
                    "unlap": state.driver_id in self._sc_unlap_driver_ids,
                    "following_source": (
                        physical_leader.driver_id if physical_leader is not None else None
                    ),
                }
            )
        return {
            "race_phase": self.race_phase,
            "safety_car_stage": self.safety_car_stage,
            "queue_state": self._sc_queue_state,
            "queue_plan_revision": (
                self._sc_queue_plan.revision if self._sc_queue_plan is not None else None
            ),
            "queue_plan_slots": [
                {
                    "driver_id": slot.driver_id,
                    "slot_index": slot.slot_index,
                    "status": slot.status,
                    "inserted_reason": slot.inserted_reason,
                    "revision": slot.revision,
                }
                for slot in (
                    self._sc_queue_plan.slots[:64]
                    if self._sc_queue_plan is not None
                    else []
                )
            ],
            "queue_stable_seconds": round(self._sc_queue_stable_seconds, 3),
            "queue_formed": bool(self._safety_car_queue_formed),
            "running_order": [state.driver_id for state in running[:64]],
            "physical_order": [state.driver_id for state in physical[:64]],
            "caught_driver_ids": sorted(self._sc_caught_driver_ids)[:64],
            "unlap_driver_ids": sorted(self._sc_unlap_driver_ids)[:64],
            "pit_exit_order_target_ids": sorted(self._sc_pit_exit_order_targets)[:64],
            "order_yield_targets": {
                str(driver_id): predecessor_id
                for driver_id, predecessor_id in list(
                    self._sc_order_yield_targets.items()
                )[:64]
            },
            "active_correction": correction_snapshot,
            "drivers": drivers,
        }

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
                self._sc_queue_state = SC_QUEUE_STATE_PICKUP
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
        if self.safety_car_stage == "collecting" and self._sc_leader_acquisition_wait_active():
            # If the SC exits just after the leader has passed the pit exit,
            # it waits at a controlled pace to pick that leader up instead of
            # forcing the field to chase almost a full lap.
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
            self._sc_restart_entered_at = self.race_elapsed
            self._sc_queue_state = SC_QUEUE_STATE_RESTART
            # ``tick`` may consume several 50 Hz steps.  Keep restart visible
            # for the remainder of that outer tick so lifecycle diagnostics
            # observe the restart stage before green is published.
            self._sc_restart_hold_ticks = 5
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
        self._sc_order_correction = None
        self._sc_order_correction_retry_after = 0.0
        self._sc_queue_plan = None
        self._sc_driver_states.clear()
        self._sc_queue_state = SC_QUEUE_STATE_GREEN
        self._sc_queue_stable_seconds = 0.0
        self._sc_last_queue_sync_elapsed = 0.0
        self._sc_withdraw_target = None
        self._sc_restart_target = None
        self._sc_restart_accel_progress = None
        self._sc_restart_entered_at = 0.0
        self._sc_restart_hold_ticks = 0
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

        if self.safety_car_stage == "collecting":
            self._sc_queue_state = SC_QUEUE_STATE_FORMING

        self._refresh_sc_running_order()
        running = self._sc_ordered_on_track_states(include_pending=False)
        self._refresh_sc_order_yield_targets(running)
        plan = self._sc_queue_plan
        if plan is None:
            return

        elapsed = max(0.0, self.race_elapsed - self._sc_last_queue_sync_elapsed)
        self._sc_last_queue_sync_elapsed = self.race_elapsed
        active_ids = {state.driver_id for state in running}
        physical_ids = [
            state.driver_id
            for state in sorted(
                running,
                key=lambda state: (-state.total_progress, state.position),
            )
        ]
        plan_ids = [
            driver_id for driver_id in self._sc_plan_driver_ids()
            if driver_id in active_ids
        ]
        self._sc_caught_driver_ids.clear()
        for index, state in enumerate(running):
            control = self._sc_driver_states.setdefault(
                state.driver_id,
                DriverSafetyCarState(
                    mode=SC_DRIVER_MODE_CATCH_UP,
                    slot_index=index,
                    predecessor_id=(plan_ids[index - 1] if index else None),
                    mode_entered_at=self.race_elapsed,
                ),
            )
            predecessor = self._sc_queue_predecessor(state)
            if predecessor is None:
                ahead_total = self._safety_car_total_progress
                ahead_speed_mps = max(0.0, self._safety_car_speed_mps)
            else:
                ahead_total = predecessor.total_progress
                ahead_speed_mps = max(0.0, predecessor.speed_kph / 3.6)
            gap_m = (
                (ahead_total - state.total_progress) * self.track_length_m
                if ahead_total is not None
                else float("inf")
            )
            gap_error_m = gap_m - SC_SLOT_TARGET_GAP_M
            relative_speed_kph = state.speed_kph - ahead_speed_mps * 3.6
            previous_mode = control.mode
            previous_predecessor_id = control.predecessor_id
            predecessor_id = predecessor.driver_id if predecessor is not None else None
            control.relative_speed_kph = round(relative_speed_kph, 3)
            if (
                control.relative_speed_filtered_kph is None
                or previous_predecessor_id != predecessor_id
            ):
                control.relative_speed_filtered_kph = relative_speed_kph
            elif elapsed > 0.0:
                filter_alpha = min(
                    1.0,
                    elapsed
                    / (SC_QUEUE_RELATIVE_SPEED_FILTER_SECONDS + elapsed),
                )
                control.relative_speed_filtered_kph += (
                    filter_alpha
                    * (
                        relative_speed_kph
                        - control.relative_speed_filtered_kph
                    )
                )
            filtered_relative_speed_kph = control.relative_speed_filtered_kph
            if state.driver_id in self._sc_unlap_driver_ids:
                control.relative_speed_violation_seconds = 0.0
                mode = SC_DRIVER_MODE_UNLAPPING
            elif (
                self._sc_order_correction is not None
                and self._sc_order_correction.get("yielding_driver_id")
                == state.driver_id
            ):
                control.relative_speed_violation_seconds = 0.0
                mode = SC_DRIVER_MODE_ORDER_RECOVERY
            elif gap_m <= 0.0 or gap_m > SC_QUEUE_CATCH_UP_THRESHOLD_M:
                control.relative_speed_violation_seconds = 0.0
                mode = SC_DRIVER_MODE_CATCH_UP
            elif gap_m > SC_QUEUE_APPROACH_THRESHOLD_M:
                control.relative_speed_violation_seconds = 0.0
                mode = SC_DRIVER_MODE_APPROACHING
            elif abs(gap_error_m) > SC_QUEUE_STABLE_GAP_TOLERANCE_M:
                # A real gap excursion still breaks stability immediately;
                # only the corner-sensitive speed signal is debounced below.
                control.relative_speed_violation_seconds = 0.0
                mode = SC_DRIVER_MODE_FOLLOWING
            elif previous_mode == SC_DRIVER_MODE_STABLE:
                # Keep a formed pair stable through a short corner-induced
                # speed differential.  A wider exit threshold plus a grace
                # period prevents one noisy physics sample from resetting the
                # whole-field queue timer, while sustained separation still
                # returns the car to FOLLOWING.
                if (
                    abs(filtered_relative_speed_kph)
                    > SC_QUEUE_STABLE_RELATIVE_SPEED_EXIT_KPH
                ):
                    control.relative_speed_violation_seconds += elapsed
                else:
                    control.relative_speed_violation_seconds = 0.0
                mode = (
                    SC_DRIVER_MODE_FOLLOWING
                    if control.relative_speed_violation_seconds
                    >= SC_QUEUE_STABLE_RELATIVE_SPEED_GRACE_SECONDS
                    else SC_DRIVER_MODE_STABLE
                )
            else:
                control.relative_speed_violation_seconds = 0.0
                mode = (
                    SC_DRIVER_MODE_STABLE
                    if abs(filtered_relative_speed_kph)
                    <= SC_QUEUE_STABLE_RELATIVE_SPEED_KPH
                    else SC_DRIVER_MODE_FOLLOWING
                )
            if mode != previous_mode:
                control.mode_entered_at = self.race_elapsed
                control.stable_seconds = 0.0
            elif mode == SC_DRIVER_MODE_STABLE:
                control.stable_seconds += elapsed
            else:
                control.stable_seconds = 0.0
            control.mode = mode
            control.slot_index = plan_ids.index(state.driver_id) if state.driver_id in plan_ids else None
            control.predecessor_id = predecessor_id
            control.gap_error_m = round(gap_error_m, 3)
            if (
                gap_m >= 0.0
                and abs(gap_error_m) <= SC_QUEUE_STABLE_GAP_TOLERANCE_M
                and abs(relative_speed_kph) <= SC_QUEUE_JOIN_MAX_RELATIVE_SPEED_KPH
            ):
                self._sc_caught_driver_ids.add(state.driver_id)

        queue_conditions_stable = (
            bool(plan_ids)
            and len(plan_ids) == len(set(plan_ids))
            and physical_ids == plan_ids
            and not plan.pending_pit_exit_ids
            and not plan.unlapping_ids
            and not self._sc_pit_exit_order_targets
            and self._sc_order_correction is None
            and not self._active_stopped_hazards()
            and all(
                self._sc_driver_states.get(driver_id) is not None
                and self._sc_driver_states[driver_id].mode
                == SC_DRIVER_MODE_STABLE
                for driver_id in plan_ids
            )
        )
        if queue_conditions_stable:
            self._sc_queue_stable_seconds += elapsed
        else:
            self._sc_queue_stable_seconds = 0.0
        formed = self._sc_queue_stable_seconds >= SC_QUEUE_STABLE_SECONDS
        was_formed = self._safety_car_queue_formed
        self._safety_car_queue_formed = formed

        if formed and not was_formed and self.safety_car_stage == "collecting":
            self.safety_car_stage = "queued"
            self._sc_queue_state = SC_QUEUE_STATE_STABLE
            events.append(
                RaceEvent(
                    type="sc_queue",
                    message="The field is queued behind the Safety Car",
                    message_ko="전체 차량이 세이프티카 뒤에 대열을 형성했습니다",
                )
            )
        elif not formed and self.safety_car_stage == "queued":
            self.safety_car_stage = "collecting"
            self._sc_queue_state = SC_QUEUE_STATE_FORMING

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
        self._sc_queue_state = SC_QUEUE_STATE_UNLAPPING
        self._safety_car_queue_formed = False
        self._sc_unlap_driver_ids = {state.driver_id for state in drivers}
        self._sc_unlap_targets = {
            state.driver_id: state.total_progress + 1.0 for state in drivers
        }
        self._sc_caught_driver_ids.difference_update(self._sc_unlap_driver_ids)
        if self._sc_queue_plan is not None:
            for state in drivers:
                self._remove_from_sc_running_order(state.driver_id)
                self._sc_queue_plan.unlapping_ids.add(state.driver_id)
                self._sc_driver_states[state.driver_id] = DriverSafetyCarState(
                    mode=SC_DRIVER_MODE_UNLAPPING,
                    slot_index=None,
                    predecessor_id=None,
                    mode_entered_at=self.race_elapsed,
                )
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
        plan = self._sc_queue_plan
        if plan is not None:
            plan.unlapping_ids.difference_update(self._sc_unlap_driver_ids)
            ordered_ids = list(self._sc_running_order)
            for state in sorted(unlapping, key=lambda item: item.position):
                if state.driver_id not in ordered_ids:
                    ordered_ids.append(state.driver_id)
                self._sc_driver_states[state.driver_id] = DriverSafetyCarState(
                    mode=SC_DRIVER_MODE_APPROACHING,
                    slot_index=None,
                    predecessor_id=(ordered_ids[-2] if len(ordered_ids) > 1 else None),
                    mode_entered_at=self.race_elapsed,
                )
            plan.slots = [
                SafetyCarQueueSlot(
                    driver_id=driver_id,
                    slot_index=index,
                    inserted_reason="UNLAP_COMPLETED",
                    revision=plan.revision + 1,
                )
                for index, driver_id in enumerate(ordered_ids)
            ]
            self._sc_update_queue_plan_revision("UNLAP_COMPLETED")
        for state in unlapping:
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
        self._safety_car_queue_formed = False
        self._sc_queue_stable_seconds = 0.0
        self._sc_queue_state = SC_QUEUE_STATE_FORMING

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
        self._sc_queue_state = SC_QUEUE_STATE_IN_THIS_LAP
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
