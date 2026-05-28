"""Offline unit tests for the analytic inverse kinematics.

Pure math (stdlib only). Correctness is checked by composing the real
forward_kinematics with inverse_kinematics (FK∘IK), never by a second hand-rolled
approximation. Branch *selection* lives here in the tests, not inside IK.
"""

import math
import random
import unittest

from robot_core.kinematics import (
    forward_kinematics,
    inverse_kinematics,
    load_calibration_pairs,
)


def _nearest_branch(solutions, ref_j2, ref_j3):
    """Pick the IK solution whose (J2, J3) is closest to a reference (test-side)."""
    return min(
        solutions,
        key=lambda s: abs(s[1] - ref_j2) + abs(s[2] - ref_j3),
    )


def _pose_error(pose_a, pose_b):
    return max(abs(a - b) for a, b in zip(pose_a, pose_b))


class FkIkIdentityTests(unittest.TestCase):
    def test_every_solution_reproduces_the_input_pose(self):
        # For each measured pose, every IK branch must FK back to that pose.
        for sample in load_calibration_pairs():
            pose = sample.measured_pose
            solutions = inverse_kinematics(*pose)
            self.assertGreaterEqual(len(solutions), 1, f"{sample.label}: no solution")
            for sol in solutions:
                self.assertLess(
                    _pose_error(forward_kinematics(*sol), pose),
                    1e-6,
                    f"{sample.label}: FK(IK(pose)) != pose for {sol}",
                )


class RoundTripCalibrationTests(unittest.TestCase):
    def test_recovers_recorded_joints_within_tolerance(self):
        # IK(measured pose), pick the branch nearest the recorded joints, and
        # check it recovers all four recorded joint angles.
        for sample in load_calibration_pairs():
            j1, j2, j3, j4 = sample.joints
            solutions = inverse_kinematics(*sample.measured_pose)
            sol = _nearest_branch(solutions, j2, j3)
            self.assertAlmostEqual(sol[0], j1, delta=0.01, msg=f"{sample.label} J1")
            self.assertAlmostEqual(sol[1], j2, delta=0.01, msg=f"{sample.label} J2")
            self.assertAlmostEqual(sol[2], j3, delta=0.01, msg=f"{sample.label} J3")
            self.assertAlmostEqual(sol[3], j4, delta=0.01, msg=f"{sample.label} J4")


class RandomRoundTripTests(unittest.TestCase):
    def test_random_reachable_interior_configs_round_trip(self):
        rng = random.Random(20260528)
        for _ in range(500):
            # Interior ranges: reachable, two distinct solutions, rho > 0, away
            # from the max-reach boundary (so the branches don't degenerate).
            j1 = rng.uniform(-150.0, 150.0)
            j2 = rng.uniform(0.0, 60.0)
            j3 = rng.uniform(10.0, 90.0)
            j4 = rng.uniform(-150.0, 150.0)
            pose = forward_kinematics(j1, j2, j3, j4)
            solutions = inverse_kinematics(*pose)
            self.assertGreaterEqual(len(solutions), 1)
            sol = _nearest_branch(solutions, j2, j3)
            self.assertAlmostEqual(sol[0], j1, delta=1e-6)
            self.assertAlmostEqual(sol[1], j2, delta=1e-6)
            self.assertAlmostEqual(sol[2], j3, delta=1e-6)
            self.assertAlmostEqual(sol[3], j4, delta=1e-6)


class UnreachableTests(unittest.TestCase):
    def test_far_point_returns_empty(self):
        self.assertEqual(inverse_kinematics(9999.0, 0.0, 0.0, 0.0), [])

    def test_origin_singularity_returns_empty(self):
        # The base reference (u=v=0) is unreachable for L1 != L2.
        from robot_core.kinematics import default_config

        cfg = default_config()
        self.assertEqual(inverse_kinematics(cfg.base_r_mm, 0.0, cfg.base_z_mm, 0.0), [])


class MultipleSolutionTests(unittest.TestCase):
    def test_interior_pose_has_two_solutions_both_valid(self):
        pose = forward_kinematics(10.0, 30.0, 50.0, 20.0)  # interior config
        solutions = inverse_kinematics(*pose)
        self.assertEqual(len(solutions), 2)
        # The two branches differ (elbow flip)...
        self.assertNotAlmostEqual(solutions[0][2], solutions[1][2], places=3)
        # ...but both reproduce the same pose.
        for sol in solutions:
            self.assertLess(_pose_error(forward_kinematics(*sol), pose), 1e-6)


if __name__ == "__main__":
    unittest.main()
