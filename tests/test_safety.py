"""Offline unit tests for the safety gate.

Pure decisions — synthetic state snapshots + target poses, no hardware, no
sockets. Targets are built with forward_kinematics so they are genuinely
reachable; gates are then triggered via controlled SafetyBounds.
"""

import unittest

from robot_core.kinematics import forward_kinematics, inverse_kinematics
from robot_core.safety import (
    CouplingConstraint,
    SafetyBounds,
    default_bounds,
    evaluate_control_action,
    evaluate_move,
)
from robot_core.safety.gate import (
    ACTIVE_ERROR,
    COUPLING_VIOLATED,
    JOINT_OUT_OF_RANGE,
    NOT_ENABLED,
    OK,
    OUTSIDE_WORKSPACE,
    UNREACHABLE,
)
from robot_core.state.robot_state import RobotStateSnapshot


def _snapshot(*, enabled=True, error=False, joints=(0.0, 0.0, 60.0, 0.0)):
    """A synthetic RobotStateSnapshot with the given enable/error/joint state."""
    q = (joints[0], joints[1], joints[2], joints[3], 0.0, 0.0)
    return RobotStateSnapshot(
        robot_mode=5,
        enable_status=1 if enabled else 0,
        error_status=1 if error else 0,
        tool_vector_actual=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        q_actual=q,
        seq=1,
        monotonic_ts=0.0,
    )


def _bounds(**overrides):
    """Permissive bounds by default; override one field to exercise a gate."""
    defaults = dict(
        annulus_inner_mm=50.0,
        annulus_outer_mm=500.0,
        z_min_mm=-300.0,
        z_max_mm=300.0,
        j1_rear_dead_zone_deg=40.0,
        joint_ranges_deg={
            "J1": (-160.0, 160.0),
            "J2": (-25.0, 85.0),
            "J3": (-25.0, 105.0),
            "J4": (-180.0, 180.0),
        },
        coupling=(),
    )
    defaults.update(overrides)
    return SafetyBounds(**defaults)


class GateRejectionTests(unittest.TestCase):
    def test_not_enabled_rejected(self):
        target = forward_kinematics(0, 0, 60, 0)
        decision = evaluate_move(target, _snapshot(enabled=False), bounds=_bounds())
        self.assertFalse(decision.approved)
        self.assertEqual(decision.code, NOT_ENABLED)
        self.assertIsNone(decision.chosen_joints)

    def test_active_error_rejected(self):
        target = forward_kinematics(0, 0, 60, 0)
        decision = evaluate_move(target, _snapshot(error=True), bounds=_bounds())
        self.assertFalse(decision.approved)
        self.assertEqual(decision.code, ACTIVE_ERROR)

    def test_unreachable_rejected(self):
        decision = evaluate_move((9999.0, 0.0, 0.0, 0.0), _snapshot(), bounds=_bounds())
        self.assertFalse(decision.approved)
        self.assertEqual(decision.code, UNREACHABLE)

    def test_outside_outer_radius_rejected(self):
        target = forward_kinematics(0, 80, 0, 0)  # rho ~456 mm, still IK-reachable
        decision = evaluate_move(target, _snapshot(), bounds=_bounds(annulus_outer_mm=400.0))
        self.assertFalse(decision.approved)
        self.assertEqual(decision.code, OUTSIDE_WORKSPACE)

    def test_rear_dead_zone_rejected(self):
        target = forward_kinematics(170, 20, 50, 0)  # azimuth 170 deg = behind base
        decision = evaluate_move(target, _snapshot(), bounds=_bounds())
        self.assertFalse(decision.approved)
        self.assertEqual(decision.code, OUTSIDE_WORKSPACE)
        self.assertIn("dead-zone", decision.reason)

    def test_chosen_solution_out_of_joint_range_rejected(self):
        target = forward_kinematics(10, 80, 50, 0)  # J2 = 80
        ranges = {
            "J1": (-160.0, 160.0),
            "J2": (-25.0, 70.0),  # 80 now exceeds this
            "J3": (-25.0, 105.0),
            "J4": (-180.0, 180.0),
        }
        decision = evaluate_move(
            target, _snapshot(joints=(10, 80, 50, 0)), bounds=_bounds(joint_ranges_deg=ranges)
        )
        self.assertFalse(decision.approved)
        self.assertEqual(decision.code, JOINT_OUT_OF_RANGE)

    def test_chosen_solution_violates_coupling_rejected(self):
        target = forward_kinematics(0, 0, 80, 0)  # J3 = 80
        coupling = (CouplingConstraint(j2_coeff=0.0, j3_coeff=1.0, max_value=60.0, label="J3<=60"),)
        decision = evaluate_move(
            target, _snapshot(joints=(0, 0, 80, 0)), bounds=_bounds(coupling=coupling)
        )
        self.assertFalse(decision.approved)
        self.assertEqual(decision.code, COUPLING_VIOLATED)


class GateApprovalTests(unittest.TestCase):
    def test_reachable_in_range_approved(self):
        joints = (10.0, 20.0, 50.0, 0.0)
        target = forward_kinematics(*joints)
        decision = evaluate_move(target, _snapshot(joints=joints), bounds=_bounds())
        self.assertTrue(decision.approved)
        self.assertEqual(decision.code, OK)
        for got, want in zip(decision.chosen_joints, joints):
            self.assertAlmostEqual(got, want, delta=1e-6)

    def test_approved_under_default_bounds(self):
        # Exercises default_bounds() / config/safety.json loading with a known-safe
        # pose. Uses the actual factory pose from PROGRESS finding 4 / calibration
        # pair A1 (J=(-0.007, -0.021, 59.903, 2.681)) — NOT a rounded (0,0,60,0)
        # which sits exactly on the J3-J2=60 coupling boundary deployed by T7B.
        factory_joints = (-0.007, -0.021, 59.903, 2.681)
        target = forward_kinematics(*factory_joints)
        decision = evaluate_move(target, _snapshot(joints=factory_joints))
        self.assertTrue(decision.approved, decision.reason)
        self.assertIsNotNone(default_bounds())


class NearestSolutionTests(unittest.TestCase):
    def test_picks_solution_nearest_to_current_joints(self):
        target = forward_kinematics(0, 20, 40, 0)
        solutions = inverse_kinematics(*target)
        self.assertEqual(len(solutions), 2)  # two-branch interior pose
        wide = _bounds(  # wide ranges so either branch is allowed -> isolates selection
            joint_ranges_deg={k: (-200.0, 200.0) for k in ("J1", "J2", "J3", "J4")}
        )
        for branch in solutions:
            decision = evaluate_move(target, _snapshot(joints=branch[:4]), bounds=wide)
            self.assertTrue(decision.approved)
            for got, want in zip(decision.chosen_joints, branch):
                self.assertAlmostEqual(got, want, delta=1e-6)


class EmergencyStopTests(unittest.TestCase):
    def test_estop_always_allowed(self):
        for action in ("EmergencyStop", "ClearError", "DisableRobot", "ResetRobot"):
            decision = evaluate_control_action(action)
            self.assertTrue(decision.approved, action)
            self.assertEqual(decision.code, OK)

    def test_motion_command_is_not_an_always_allowed_control_action(self):
        decision = evaluate_control_action("MovL")
        self.assertFalse(decision.approved)


if __name__ == "__main__":
    unittest.main()
