"""Stage 3 single-vehicle kinematics tests and Bahrain/RBR diagnostics."""

from __future__ import annotations

import unittest

from data_loader import load_circuits, load_drivers, load_teams
from simulation.abstract import (
    AbstractSessionSnapshot,
    BoundedKinematicPoseBuffer,
    LateralTrajectory,
    SingleProbeKinematicCursor,
    SingleVehiclePoseSynthesizer,
    pose_sequence_hash,
)


class AbstractStage3KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.circuits = {circuit.id: circuit for circuit in load_circuits()}
        cls.drivers = load_drivers()
        cls.teams = load_teams()

    def _snapshot(self, circuit_id: int):
        return AbstractSessionSnapshot.from_content(
            session_id=f"stage3-baseline-{circuit_id}",
            session_seed=42,
            circuit=self.circuits[circuit_id],
            drivers=self.drivers,
            teams=self.teams,
        )

    def test_pose_exposes_compiled_racing_line_distance(self) -> None:
        """Line distance, rather than circuit length, is the pose authority."""

        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                circuit = self.circuits[circuit_id]
                snapshot = self._snapshot(circuit_id)
                synthesizer = SingleVehiclePoseSynthesizer(snapshot)
                progress = 0.5
                pose = synthesizer.pose_at(1, progress)
                compiled_distance = synthesizer.distance_contract.distance_at_total_progress(progress)
                self.assertLess(
                    abs(pose.line_distance_m - compiled_distance),
                    0.10,
                    f"{circuit.name} still uses circuit distance as pose authority",
                )

    def test_kinematic_sequence_has_bounded_chord_acceleration(self) -> None:
        """The distance-integrated sequence keeps chord acceleration bounded."""

        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                synthesizer = SingleVehiclePoseSynthesizer(self._snapshot(circuit_id))
                frames = synthesizer.logical_sequence(
                    1,
                    start_logical_time_s=0.0,
                    end_logical_time_s=20.0,
                )
                accelerations = [frame.longitudinal_acceleration_mps2 for frame in frames]
                self.assertLessEqual(max(accelerations), 18.0)
                self.assertGreaterEqual(min(accelerations), -50.0)
                self.assertTrue(
                    all(
                        current.line_distance_m - previous.line_distance_m
                        <= previous.speed_mps * 0.10 + 0.5 + 1e-6
                        for previous, current in zip(frames, frames[1:])
                    )
                )

    def test_distance_derived_metrics_and_reported_identity(self) -> None:
        """Use pose distance differences, not only stored diagnostics, for approval."""

        dt = 0.10
        regions = {
            3: (0.42444, 0.13158),
            4: (0.29789, 0.29960),
        }
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                synthesizer = SingleVehiclePoseSynthesizer(self._snapshot(circuit_id))
                frames = synthesizer.logical_sequence(
                    1,
                    start_logical_time_s=0.0,
                    end_logical_time_s=180.0,
                )
                interval_speeds = [
                    (current.line_distance_m - previous.line_distance_m) / dt
                    for previous, current in zip(frames, frames[1:])
                ]
                derived_accelerations = [
                    (next_speed - previous_speed) / dt
                    for previous_speed, next_speed in zip(interval_speeds, interval_speeds[1:])
                ]
                self.assertLessEqual(max(interval_speeds) * 3.6, 370.0 + 1e-7)
                self.assertLessEqual(max(derived_accelerations), 18.0 + 1e-7)
                self.assertGreaterEqual(min(derived_accelerations), -50.0 - 1e-7)
                for previous, current, interval_speed in zip(
                    frames[1:],
                    frames[2:],
                    interval_speeds[1:],
                ):
                    self.assertAlmostEqual(
                        interval_speed,
                        (previous.speed_mps + current.speed_mps) / 2.0,
                        delta=1e-6,
                    )
                    self.assertAlmostEqual(
                        current.longitudinal_acceleration_mps2,
                        (current.speed_mps - previous.speed_mps) / dt,
                        delta=1e-6,
                    )
                for target_progress in regions[circuit_id]:
                    self.assertLess(
                        min(abs(frame.progress - target_progress) for frame in frames),
                        0.01,
                    )

    def test_atomic_integrator_step_exposes_one_accepted_identity(self) -> None:
        dt = 0.10
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                synthesizer = SingleVehiclePoseSynthesizer(self._snapshot(circuit_id))
                plan = synthesizer.build_plan(1)
                distance = synthesizer.distance_contract.distance_at_total_progress(0.0)
                speed = plan.target_speed_at_line_distance(distance)
                for _ in range(1800):
                    step = plan.integrate(distance, speed)
                    self.assertAlmostEqual(
                        step.distance_m - step.previous_distance_m,
                        (step.previous_speed_mps + step.speed_mps) / 2.0 * dt,
                        delta=1e-6,
                    )
                    self.assertAlmostEqual(
                        step.longitudinal_acceleration_mps2,
                        (step.speed_mps - step.previous_speed_mps) / dt,
                        delta=1e-6,
                    )
                    self.assertFalse(step.integrator_adjusted)
                    distance, speed = step.distance_m, step.speed_mps

    def test_single_probe_cursor_is_continuous_and_exposes_accepted_step(self) -> None:
        dt = 0.10
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                synthesizer = SingleVehiclePoseSynthesizer(self._snapshot(circuit_id))
                cursor = SingleProbeKinematicCursor(
                    synthesizer,
                    1,
                    grid_position=1,
                )
                initial = cursor.current_pose()
                self.assertEqual(initial.physics_frame, 0)
                self.assertEqual(initial.simulation_time_s, 0.0)
                frames = [initial]
                accepted_steps = []
                for _ in range(2000):
                    tick = cursor.advance_one_tick()
                    frames.append(tick.pose)
                    accepted_steps.append(tick.accepted_step)
                    self.assertIs(cursor.last_step, tick.accepted_step)
                    self.assertAlmostEqual(
                        tick.pose.line_distance_m,
                        tick.accepted_step.distance_m,
                        delta=1e-9,
                    )
                    self.assertAlmostEqual(
                        tick.pose.speed_mps,
                        tick.accepted_step.speed_mps,
                        delta=1e-9,
                    )
                    self.assertAlmostEqual(
                        tick.pose.longitudinal_acceleration_mps2,
                        tick.accepted_step.longitudinal_acceleration_mps2,
                        delta=1e-9,
                    )
                    if tick.pose.lap_number >= 2:
                        break
                self.assertGreaterEqual(frames[-1].lap_number, 2)
                self.assertEqual(
                    [frame.physics_frame for frame in frames],
                    list(range(len(frames))),
                )
                for previous, current, step in zip(frames, frames[1:], accepted_steps):
                    self.assertAlmostEqual(current.simulation_time_s - previous.simulation_time_s, dt, delta=1e-9)
                    self.assertAlmostEqual(
                        current.line_distance_m - previous.line_distance_m,
                        (step.previous_speed_mps + step.speed_mps) * 0.5 * dt,
                        delta=1e-6,
                    )
                    self.assertAlmostEqual(
                        step.longitudinal_acceleration_mps2,
                        (step.speed_mps - step.previous_speed_mps) / dt,
                        delta=1e-6,
                    )
                cursor.dispose()
                self.assertTrue(cursor.disposed)

    def test_baseline_lateral_offset_has_no_instant_transition_contract(self) -> None:
        """The old pose API has no duration/easing for a line transition."""

        from simulation.abstract.kinematics import LateralTrajectory

        trajectory = LateralTrajectory.create(
            start_offset_m=0.0,
            end_offset_m=2.0,
            start_time_s=0.0,
            duration_s=0.10,
        )
        self.assertLessEqual(trajectory.max_lateral_speed_mps, 8.0)
        self.assertLessEqual(trajectory.max_lateral_acceleration_mps2, 20.0)

    def test_line_distance_round_trip_and_wrap_are_compiled_profile_authority(self) -> None:
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                synthesizer = SingleVehiclePoseSynthesizer(self._snapshot(circuit_id))
                contract = synthesizer.distance_contract
                for progress in (0.0, 0.01, 0.25, 0.5, 0.99, 1.0, 1.25, 2.0):
                    distance = contract.distance_at_total_progress(progress)
                    restored = contract.total_progress_at_distance(distance)
                    self.assertAlmostEqual(restored, progress, places=7)
                start = synthesizer.pose_at(1, 0.0)
                lap = synthesizer.pose_at(1, 1.0)
                self.assertAlmostEqual(start.world_x_m, lap.world_x_m, places=7)
                self.assertAlmostEqual(start.world_y_m, lap.world_y_m, places=7)
                self.assertAlmostEqual(start.line_distance_m, 0.0, places=7)
                self.assertAlmostEqual(lap.line_distance_m, contract.line_length_m, places=7)

    def test_longitudinal_plan_meets_speed_acceleration_and_tick_displacement_guards(self) -> None:
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                synthesizer = SingleVehiclePoseSynthesizer(self._snapshot(circuit_id))
                frames = synthesizer.logical_sequence(
                    1,
                    start_logical_time_s=0.0,
                    end_logical_time_s=180.0,
                    start_total_progress=0.0,
                )
                self.assertGreater(max(frame.speed_mps for frame in frames), 1.0)
                self.assertLessEqual(max(frame.speed_mps for frame in frames) * 3.6, 370.0 + 1e-7)
                self.assertLessEqual(max(frame.longitudinal_acceleration_mps2 for frame in frames), 18.0 + 1e-7)
                self.assertGreaterEqual(min(frame.longitudinal_acceleration_mps2 for frame in frames), -50.0 - 1e-7)
                self.assertTrue(all(frame.speed_mps >= -1e-9 for frame in frames))
                self.assertTrue(
                    all(
                        current.line_distance_m + 1e-7 >= previous.line_distance_m
                        for previous, current in zip(frames, frames[1:])
                    )
                )
                for previous, current in zip(frames, frames[1:]):
                    displacement = current.line_distance_m - previous.line_distance_m
                    self.assertLessEqual(displacement, previous.speed_mps * 0.10 + 0.5 + 1e-6)
                    world_chord = ((current.world_x_m - previous.world_x_m) ** 2 +
                                   (current.world_y_m - previous.world_y_m) ** 2) ** 0.5
                    self.assertLessEqual(world_chord, previous.speed_mps * 0.10 + 0.5 + 1e-6)

    def test_lateral_trajectory_limits_and_boundary_envelope(self) -> None:
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                snapshot = self._snapshot(circuit_id)
                synthesizer = SingleVehiclePoseSynthesizer(snapshot)
                trajectory = LateralTrajectory.create(
                    start_offset_m=0.0,
                    end_offset_m=2.0,
                    start_time_s=0.0,
                    duration_s=0.10,
                )
                self.assertGreaterEqual(trajectory.duration_s, 0.10)
                self.assertLessEqual(trajectory.max_lateral_speed_mps, 8.0 + 1e-7)
                self.assertLessEqual(trajectory.max_lateral_acceleration_mps2, 20.0 + 1e-7)
                frames = synthesizer.logical_sequence(
                    1,
                    start_logical_time_s=0.0,
                    end_logical_time_s=4.0,
                    lateral_trajectory=trajectory,
                )
                for frame in frames:
                    minimum, maximum = synthesizer.distance_contract.lateral_limits_at_progress(
                        frame.total_progress,
                        car_width_m=snapshot.track.display_geometry.car_width_m,
                    )
                    self.assertGreaterEqual(frame.lateral_offset_m, minimum - 1e-7)
                    self.assertLessEqual(frame.lateral_offset_m, maximum + 1e-7)
                self.assertGreater(frames[-1].lateral_offset_m, frames[0].lateral_offset_m)

    def test_signed_lateral_transitions_are_smooth(self) -> None:
        synthesizer = SingleVehiclePoseSynthesizer(self._snapshot(4))
        for target in (2.0, -2.0):
            with self.subTest(target=target):
                trajectory = LateralTrajectory.create(
                    start_offset_m=0.0,
                    end_offset_m=target,
                    start_time_s=0.0,
                    duration_s=2.0,
                )
                samples = [trajectory.offset_at(index * 0.1) for index in range(21)]
                self.assertAlmostEqual(samples[0], 0.0, places=7)
                self.assertAlmostEqual(samples[-1], target, places=7)
                self.assertTrue(
                    all(
                        abs(samples[index + 1] - samples[index]) <= 0.8 + 1e-9
                        for index in range(len(samples) - 1)
                    )
                )

    def test_lateral_boundary_violation_is_rejected_without_snap(self) -> None:
        synthesizer = SingleVehiclePoseSynthesizer(self._snapshot(3))
        with self.assertRaises(ValueError):
            synthesizer.logical_sequence(
                1,
                start_logical_time_s=0.0,
                end_logical_time_s=0.1,
                lateral_offset_m=100.0,
            )

    def test_grid_hold_and_lights_out_launch_are_continuous(self) -> None:
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                synthesizer = SingleVehiclePoseSynthesizer(self._snapshot(circuit_id))
                slot = synthesizer.track.display_geometry.grid_slots[0]
                frames = synthesizer.grid_sequence(1, grid_position=1, logical_duration_s=3.0)
                hold_frames = [frame for frame in frames if frame.simulation_time_s <= 1.0 + 1e-9]
                self.assertTrue(all(abs(frame.speed_mps) <= 1e-9 for frame in hold_frames))
                self.assertTrue(all(frame.line_distance_m == hold_frames[0].line_distance_m for frame in hold_frames))
                self.assertAlmostEqual(hold_frames[0].lateral_offset_m, slot.lateral_offset_m, places=7)
                launch_frames = [frame for frame in frames if frame.simulation_time_s > 1.0]
                self.assertTrue(any(frame.speed_mps > 0.0 for frame in launch_frames))
                self.assertLessEqual(max(frame.longitudinal_acceleration_mps2 for frame in frames), 18.0 + 1e-7)
                self.assertTrue(
                    all(
                        current.line_distance_m >= previous.line_distance_m - 1e-7
                        for previous, current in zip(frames, frames[1:])
                    )
                )

    def test_logical_interval_pose_hash_is_independent_of_wall_speed(self) -> None:
        for circuit_id in (3, 4):
            with self.subTest(circuit_id=circuit_id):
                synthesizer = SingleVehiclePoseSynthesizer(self._snapshot(circuit_id))
                sequences = [
                    synthesizer.logical_sequence(
                        1,
                        start_logical_time_s=0.0,
                        end_logical_time_s=12.0,
                        speed_multiplier=speed,
                    )
                    for speed in (1.0, 2.0, 5.0)
                ]
                self.assertEqual(len(sequences[0]), len(sequences[1]))
                self.assertEqual(len(sequences[1]), len(sequences[2]))
                hashes = [pose_sequence_hash(sequence) for sequence in sequences]
                self.assertEqual(len(set(hashes)), 1)
                self.assertEqual(
                    [frame.to_dict() for frame in sequences[0]],
                    [frame.to_dict() for frame in sequences[2]],
                )

    def test_bounded_probe_buffer_does_not_retain_full_sequence(self) -> None:
        synthesizer = SingleVehiclePoseSynthesizer(self._snapshot(3))
        frames = synthesizer.logical_sequence(
            1,
            start_logical_time_s=0.0,
            end_logical_time_s=30.0,
        )
        buffer = BoundedKinematicPoseBuffer(max_frames=16)
        for frame in frames:
            buffer.append(frame)
        self.assertEqual(len(buffer), 16)
        self.assertEqual(buffer.latest, frames[-1])


if __name__ == "__main__":
    unittest.main()
