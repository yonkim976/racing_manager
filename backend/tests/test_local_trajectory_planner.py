"""Tests for the first-stage short-horizon trajectory lattice."""

from __future__ import annotations

from dataclasses import replace
import unittest

from data_loader import load_circuits, load_drivers, load_teams
from simulation.local_trajectory_planner import (
    LocalTrajectoryPlanner,
    LocalTrajectoryPlannerConfig,
    LocalTrajectoryPlannerWeights,
    LocalTrajectoryPlanningRequest,
    NearbyVehiclePredictionInput,
)
from simulation.race_engine import RaceEngine
from simulation.track_physics import DRIVING_LINE_RACING


class LocalTrajectoryPlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        driver = load_drivers()[0]
        teams = {team.id: team for team in load_teams()}
        circuit = next(circuit for circuit in load_circuits() if circuit.id == 4)
        cls.engine = RaceEngine(
            circuit=circuit,
            drivers=[driver],
            teams=teams,
            player_team_id=driver.team_id,
            player_driver_ids=[driver.id],
            seed=17,
            start_sequence_enabled=False,
        )
        cls.state = cls.engine.driver_states[driver.id]

    def _planner(
        self,
        config: LocalTrajectoryPlannerConfig | None = None,
    ) -> LocalTrajectoryPlanner:
        return LocalTrajectoryPlanner(
            self.engine._track_physics_for_driver(self.state),
            self.engine._track_surface,
            self.engine._vehicle_physics_for_driver(self.state),
            track_length_m=self.engine.track_length_m,
            config=config,
        )

    def _request(
        self,
        *,
        total_progress: float | None = None,
        speed_mps: float | None = None,
        nearby_vehicles: tuple[NearbyVehiclePredictionInput, ...] = (),
    ) -> LocalTrajectoryPlanningRequest:
        progress = (
            self.state.total_progress
            if total_progress is None
            else total_progress
        )
        track = self.engine._track_physics_for_driver(self.state)
        return LocalTrajectoryPlanningRequest(
            total_progress=progress,
            speed_mps=(
                self.state.speed_kph / 3.6
                if speed_mps is None
                else speed_mps
            ),
            acceleration_mps2=self.state.acceleration_mps2,
            lateral_offset_m=track.line_offset_at_progress(
                DRIVING_LINE_RACING,
                progress,
            ),
            lateral_speed_mps=self.state.lateral_speed_mps,
            body_width_m=self.state.car_width_m,
            body_length_m=self.state.car_length_m,
            modifiers=self.engine._physics_v2_modifiers(self.state),
            nearby_vehicles=nearby_vehicles,
        )

    def _nearby_vehicle(
        self,
        *,
        driver_id: int = 99,
        gap_m: float,
        speed_mps: float = 35.0,
    ) -> NearbyVehiclePredictionInput:
        progress = 0.12
        track = self.engine._track_physics_for_driver(self.state)
        lateral_offset_m = track.line_offset_at_progress(
            DRIVING_LINE_RACING,
            progress,
        )
        return NearbyVehiclePredictionInput(
            driver_id=driver_id,
            total_progress=progress + gap_m / self.engine.track_length_m,
            speed_mps=speed_mps,
            acceleration_mps2=0.0,
            lateral_offset_m=lateral_offset_m,
            lateral_speed_mps=0.0,
            target_lateral_offset_m=lateral_offset_m,
            heading_offset_rad=0.0,
            body_width_m=self.state.car_width_m,
            body_length_m=self.state.car_length_m,
        )

    def test_lattice_supports_five_seven_or_nine_symmetric_candidates(self) -> None:
        for count in (5, 7, 9):
            planner = self._planner(
                LocalTrajectoryPlannerConfig(candidate_count=count)
            )
            biases = planner.lateral_biases_m()
            self.assertEqual(len(biases), count)
            self.assertEqual(biases, tuple(-value for value in reversed(biases)))
            self.assertIn(0.0, biases)

        with self.assertRaises(ValueError):
            LocalTrajectoryPlannerConfig(candidate_count=6)

    def test_clean_track_selects_center_and_builds_body_occupancy(self) -> None:
        plan = self._planner().plan(self._request())

        self.assertEqual(len(plan.candidates), 5)
        self.assertEqual(plan.selected_candidate_id, "center")
        self.assertTrue(plan.selected.viable)
        self.assertEqual(plan.selected.body_boundary_violations, 0)
        self.assertEqual(len(plan.selected.samples), 31)
        self.assertAlmostEqual(plan.selected.samples[-1].time_seconds, 3.0)
        self.assertGreater(plan.selected.forward_distance_m, 0.0)
        self.assertTrue(
            all(
                sample.body_pose.width_m == self.state.car_width_m
                and sample.body_pose.length_m == self.state.car_length_m
                for sample in plan.selected.samples
            )
        )

    def test_wide_lattice_marks_boundary_crossing_candidates_nonviable(self) -> None:
        planner = self._planner(
            LocalTrajectoryPlannerConfig(
                candidate_count=9,
                lateral_spacing_m=2.0,
            )
        )
        plan = planner.plan(self._request())

        self.assertEqual(plan.selected_candidate_id, "center")
        self.assertTrue(any(not candidate.viable for candidate in plan.candidates))
        self.assertTrue(plan.selected.viable)

    def test_prediction_contains_real_braking_target_before_a_corner(self) -> None:
        plan = self._planner().plan(
            self._request(total_progress=0.05, speed_mps=90.0)
        )

        self.assertGreater(max(sample.brake for sample in plan.selected.samples), 0.5)
        self.assertLess(
            min(sample.target_speed_mps for sample in plan.selected.samples),
            50.0,
        )

    def test_control_preview_grows_with_speed_and_exposes_lateral_feedforward(self) -> None:
        planner = self._planner()
        slow = planner.plan(self._request(speed_mps=20.0))
        fast = planner.plan(self._request(speed_mps=80.0))

        self.assertGreater(
            fast.selected.control_lookahead_seconds,
            slow.selected.control_lookahead_seconds,
        )
        self.assertLessEqual(fast.selected.control_lookahead_seconds, 0.95)
        self.assertIsInstance(
            fast.selected.control_target_lateral_speed_mps,
            float,
        )

    def test_same_state_produces_deterministic_plan(self) -> None:
        planner = self._planner()
        request = self._request()

        self.assertEqual(planner.plan(request), planner.plan(request))

    def test_track_boundary_cache_reuses_the_same_quantized_sample(self) -> None:
        planner = self._planner()

        first = planner._trajectory_bounds(0.12001, 1.90, 5.63)
        second = planner._trajectory_bounds(0.12002, 1.90, 5.63)

        self.assertIs(first, second)
        self.assertEqual(len(planner._boundary_cache), 1)

    def test_pace_weights_change_soft_cost_without_relaxing_hard_safety(self) -> None:
        planner = self._planner(
            LocalTrajectoryPlannerConfig(
                candidate_count=9,
                lateral_spacing_m=2.0,
            )
        )
        request = self._request()
        conserve = planner.plan(
            replace(
                request,
                weights=LocalTrajectoryPlannerWeights(
                    forward_progress=0.85,
                    extra_path_distance=1.20,
                    lateral_deviation=1.25,
                    surface=1.25,
                    tire_surface=1.40,
                    traffic=1.25,
                ),
            )
        )
        attack = planner.plan(
            replace(
                request,
                weights=LocalTrajectoryPlannerWeights(
                    forward_progress=1.15,
                    extra_path_distance=0.90,
                    lateral_deviation=0.80,
                    surface=0.90,
                    tire_surface=0.80,
                    traffic=0.90,
                ),
            )
        )
        conserve_by_id = {
            candidate.candidate_id: candidate for candidate in conserve.candidates
        }
        attack_by_id = {
            candidate.candidate_id: candidate for candidate in attack.candidates
        }

        self.assertNotEqual(
            conserve_by_id["center"].objective_cost,
            attack_by_id["center"].objective_cost,
        )
        self.assertEqual(
            {
                candidate_id: candidate.viable
                for candidate_id, candidate in conserve_by_id.items()
            },
            {
                candidate_id: candidate.viable
                for candidate_id, candidate in attack_by_id.items()
            },
        )
        self.assertTrue(any(not candidate.viable for candidate in attack.candidates))

    def test_traffic_prediction_uses_the_same_time_axis_and_body_size(self) -> None:
        nearby = self._nearby_vehicle(gap_m=60.0)
        plan = self._planner().plan(
            self._request(
                total_progress=0.12,
                speed_mps=70.0,
                nearby_vehicles=(nearby,),
            )
        )

        self.assertEqual(len(plan.candidates), 7)
        self.assertEqual(len(plan.opponent_occupancies), 1)
        occupancy = plan.opponent_occupancies[0]
        self.assertEqual(occupancy.driver_id, nearby.driver_id)
        self.assertEqual(len(occupancy.samples), len(plan.selected.samples))
        self.assertEqual(
            tuple(sample.time_seconds for sample in occupancy.samples),
            tuple(sample.time_seconds for sample in plan.selected.samples),
        )
        self.assertTrue(
            all(
                sample.body_pose.width_m == nearby.body_width_m
                and sample.body_pose.length_m == nearby.body_length_m
                for sample in occupancy.samples
            )
        )

    def test_slower_car_ahead_scores_ttc_and_selects_a_safe_offset(self) -> None:
        nearby = self._nearby_vehicle(gap_m=30.0)
        plan = self._planner().plan(
            self._request(
                total_progress=0.12,
                speed_mps=70.0,
                nearby_vehicles=(nearby,),
            )
        )
        center = next(
            candidate
            for candidate in plan.candidates
            if candidate.candidate_id == "center"
        )

        self.assertEqual(center.predicted_collision_count, 1)
        self.assertEqual(center.conflicting_driver_ids, (nearby.driver_id,))
        self.assertIsNotNone(center.first_collision_time_seconds)
        self.assertLess(center.first_collision_time_seconds or 99.0, 1.5)
        self.assertNotEqual(plan.selected_candidate_id, "center")
        self.assertTrue(plan.selected.viable)
        self.assertEqual(plan.selected.predicted_collision_count, 0)

    def test_unavoidable_close_traffic_returns_no_viable_candidate(self) -> None:
        nearby = self._nearby_vehicle(gap_m=20.0)
        plan = self._planner().plan(
            self._request(
                total_progress=0.12,
                speed_mps=70.0,
                nearby_vehicles=(nearby,),
            )
        )

        self.assertFalse(any(candidate.viable for candidate in plan.candidates))
        self.assertFalse(plan.selected.viable)

    def test_following_prediction_keeps_a_safe_hold_candidate_viable(self) -> None:
        nearby = self._nearby_vehicle(gap_m=20.0)
        plan = self._planner().plan(
            replace(
                self._request(
                    total_progress=0.12,
                    speed_mps=70.0,
                    nearby_vehicles=(nearby,),
                ),
                following_driver_id=nearby.driver_id,
                following_desired_gap_m=18.0,
                following_minimum_gap_m=7.0,
            )
        )

        self.assertTrue(any(candidate.viable for candidate in plan.candidates))
        self.assertTrue(plan.selected.viable)
        self.assertNotIn(
            nearby.driver_id,
            plan.selected.conflicting_driver_ids,
        )
        final_gap_m = (
            plan.opponent_occupancies[0].samples[-1].body_pose.longitudinal_m
            - plan.selected.samples[-1].body_pose.longitudinal_m
        )
        self.assertGreaterEqual(final_gap_m, 7.0 - 1e-6)

    def test_faster_same_corridor_follower_does_not_force_leader_to_evade(self) -> None:
        trailing = self._nearby_vehicle(gap_m=-20.0, speed_mps=70.0)
        plan = self._planner().plan(
            self._request(
                total_progress=0.12,
                speed_mps=35.0,
                nearby_vehicles=(trailing,),
            )
        )

        self.assertEqual(plan.selected_candidate_id, "center")
        self.assertTrue(plan.selected.viable)
        self.assertNotIn(
            trailing.driver_id,
            plan.selected.conflicting_driver_ids,
        )
        moving_candidates = [
            candidate
            for candidate in plan.candidates
            if abs(candidate.lateral_bias_m) > 1e-9
        ]
        self.assertTrue(
            any(
                trailing.driver_id in candidate.conflicting_driver_ids
                for candidate in moving_candidates
            )
        )

    def test_distant_traffic_keeps_the_racing_line(self) -> None:
        nearby = self._nearby_vehicle(gap_m=60.0, speed_mps=70.0)
        plan = self._planner().plan(
            self._request(
                total_progress=0.12,
                speed_mps=70.0,
                nearby_vehicles=(nearby,),
            )
        )

        self.assertEqual(plan.selected_candidate_id, "center")
        self.assertTrue(plan.selected.viable)
        self.assertGreater(plan.selected.minimum_opponent_clearance_m, 0.5)

    def test_worn_tire_adds_risk_cost_to_the_same_kerb_candidate(self) -> None:
        planner = self._planner(
            LocalTrajectoryPlannerConfig(
                candidate_count=9,
                lateral_spacing_m=1.0,
            )
        )
        request = self._request(total_progress=0.55, speed_mps=65.0)
        fresh = planner.plan(
            replace(
                request,
                tire_wear=0.0,
                tire_lateral_grip=1.025,
                tire_braking_grip=1.015,
                tire_traction_grip=1.025,
            )
        )
        worn = planner.plan(
            replace(
                request,
                tire_wear=0.90,
                tire_lateral_grip=0.78,
                tire_braking_grip=0.82,
                tire_traction_grip=0.75,
            )
        )
        fresh_kerb = max(
            fresh.candidates,
            key=lambda candidate: candidate.surface_cost_seconds,
        )
        worn_kerb = next(
            candidate
            for candidate in worn.candidates
            if candidate.candidate_id == fresh_kerb.candidate_id
        )

        self.assertGreater(fresh_kerb.surface_cost_seconds, 0.0)
        self.assertEqual(fresh_kerb.tire_surface_cost_seconds, 0.0)
        self.assertGreater(
            worn_kerb.tire_surface_cost_seconds,
            fresh_kerb.tire_surface_cost_seconds,
        )
        self.assertGreater(worn_kerb.objective_cost, fresh_kerb.objective_cost)

    def test_minimum_hold_prevents_immediate_safe_path_reversal(self) -> None:
        planner = self._planner()
        close_plan = planner.plan(
            self._request(
                total_progress=0.12,
                speed_mps=70.0,
                nearby_vehicles=(self._nearby_vehicle(gap_m=30.0),),
            )
        )
        distant_request = self._request(
            total_progress=0.12,
            speed_mps=70.0,
            nearby_vehicles=(
                self._nearby_vehicle(gap_m=60.0, speed_mps=70.0),
            ),
        )
        held = planner.plan(
            replace(
                distant_request,
                previous_selected_candidate_id=(
                    close_plan.selected_candidate_id
                ),
                previous_selected_age_seconds=0.20,
            )
        )
        released = planner.plan(
            replace(
                distant_request,
                previous_selected_candidate_id=(
                    close_plan.selected_candidate_id
                ),
                previous_selected_age_seconds=1.0,
            )
        )

        self.assertEqual(
            held.selected_candidate_id,
            close_plan.selected_candidate_id,
        )
        self.assertEqual(held.selection_reason, "minimum_hold")
        self.assertEqual(released.selected_candidate_id, "center")
        self.assertEqual(released.selection_reason, "lowest_cost")

    def test_hysteresis_holds_a_safe_path_for_a_small_cost_gain(self) -> None:
        planner = self._planner(
            LocalTrajectoryPlannerConfig(
                switch_cost_improvement_threshold=1.0,
            )
        )
        plan = planner.plan(
            replace(
                self._request(
                    total_progress=0.12,
                    speed_mps=70.0,
                    nearby_vehicles=(
                        self._nearby_vehicle(gap_m=60.0, speed_mps=70.0),
                    ),
                ),
                previous_selected_candidate_id="left_1",
                previous_selected_age_seconds=1.0,
            )
        )

        self.assertEqual(plan.selected_candidate_id, "left_1")
        self.assertEqual(plan.selection_reason, "hysteresis")

    def test_predicted_collision_overrides_minimum_path_hold(self) -> None:
        plan = self._planner().plan(
            replace(
                self._request(
                    total_progress=0.12,
                    speed_mps=70.0,
                    nearby_vehicles=(self._nearby_vehicle(gap_m=30.0),),
                ),
                previous_selected_candidate_id="center",
                previous_selected_age_seconds=0.10,
            )
        )

        self.assertNotEqual(plan.selected_candidate_id, "center")
        self.assertTrue(plan.selected.viable)
        self.assertEqual(plan.selection_reason, "lowest_cost")


if __name__ == "__main__":
    unittest.main()
