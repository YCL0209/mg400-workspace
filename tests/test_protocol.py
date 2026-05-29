"""Offline unit tests for the protocol layer.

Pure logic — no real arm, no sockets, no motion sent to hardware. Covers exact
command strings, static validation rejections, reply parsing (incl. error
codes), and client wiring against a fake connection.
"""

import unittest

from robot_core.protocol import builders
from robot_core.protocol.builders import CommandValidationError
from robot_core.protocol.client import DashboardClient, MoveClient
from robot_core.protocol.responses import (
    ProtocolResponseError,
    extract_responses,
    parse_response,
)


class BuilderStringTests(unittest.TestCase):
    def test_no_arg_dashboard_commands(self):
        self.assertEqual(builders.enable_robot(), "EnableRobot()")
        self.assertEqual(builders.disable_robot(), "DisableRobot()")
        self.assertEqual(builders.clear_error(), "ClearError()")
        self.assertEqual(builders.reset_robot(), "ResetRobot()")
        self.assertEqual(builders.emergency_stop(), "EmergencyStop()")
        self.assertEqual(builders.robot_mode(), "RobotMode()")
        self.assertEqual(builders.get_pose(), "GetPose()")
        self.assertEqual(builders.get_angle(), "GetAngle()")
        self.assertEqual(builders.get_error_id(), "GetErrorID()")

    def test_control_verb_commands(self):
        # Continue is the SDK-PDF capitalization (reference fork's lowercase
        # continue() is treated as fork staleness).
        self.assertEqual(builders.continue_(), "Continue()")
        self.assertEqual(builders.start_drag(), "StartDrag()")
        self.assertEqual(builders.stop_drag(), "StopDrag()")

    def test_speed_factor_integer_format(self):
        self.assertEqual(builders.speed_factor(50), "SpeedFactor(50)")
        self.assertEqual(builders.speed_factor(1), "SpeedFactor(1)")
        self.assertEqual(builders.speed_factor(100), "SpeedFactor(100)")

    def test_motion_commands_six_decimal_floats(self):
        # Matches the reference's "{:f}" formatting (6 decimals).
        self.assertEqual(
            builders.mov_l(100, 200, 50, 0), "MovL(100.000000,200.000000,50.000000,0.000000)"
        )
        self.assertEqual(
            builders.mov_j(1.5, -2, 3, 4), "MovJ(1.500000,-2.000000,3.000000,4.000000)"
        )
        self.assertEqual(
            builders.joint_mov_j(0, 0, 60, 0),
            "JointMovJ(0.000000,0.000000,60.000000,0.000000)",
        )


class StaticValidationTests(unittest.TestCase):
    def test_speed_factor_out_of_range(self):
        for bad in (0, 101, -5):
            with self.assertRaises(CommandValidationError):
                builders.speed_factor(bad)

    def test_speed_factor_non_integer(self):
        for bad in (1.5, "50", True):  # float, str, bool all rejected
            with self.assertRaises(CommandValidationError):
                builders.speed_factor(bad)

    def test_joint_mov_j_out_of_range(self):
        with self.assertRaises(CommandValidationError):
            builders.joint_mov_j(200, 0, 60, 0)  # J1 > 160
        with self.assertRaises(CommandValidationError):
            builders.joint_mov_j(0, 90, 60, 0)  # J2 > 85
        with self.assertRaises(CommandValidationError):
            builders.joint_mov_j(0, 0, 120, 0)  # J3 > 105

    def test_mov_l_rejects_non_numbers(self):
        for bad in ("x", None, float("nan"), float("inf"), True):
            with self.assertRaises(CommandValidationError):
                builders.mov_l(bad, 0, 0, 0)

    def test_mov_l_in_range_does_not_validate_cartesian(self):
        # Reachability is NOT this layer's job: a far-away point still builds.
        self.assertEqual(
            builders.mov_l(9999, 0, 0, 0), "MovL(9999.000000,0.000000,0.000000,0.000000)"
        )


class ResponseParsingTests(unittest.TestCase):
    def test_parse_success(self):
        resp = parse_response("0,{5},RobotMode()")
        self.assertEqual(resp.error_id, 0)
        self.assertTrue(resp.is_ok)
        self.assertEqual(resp.payload, "5")

    def test_parse_nonzero_error_code(self):
        resp = parse_response("-40000,{},MovL()")
        self.assertEqual(resp.error_id, -40000)
        self.assertFalse(resp.is_ok)
        self.assertEqual(resp.payload, "")

    def test_parse_nested_payload(self):
        resp = parse_response("0,{-30.26,0.00,197.23,2.67},GetPose()")
        self.assertEqual(resp.error_id, 0)
        self.assertEqual(resp.payload, "-30.26,0.00,197.23,2.67")

    def test_parse_malformed_raises(self):
        with self.assertRaises(ProtocolResponseError):
            parse_response("not-an-error-id,{},Foo()")

    def test_extract_responses_splits_stream(self):
        buffer = b"0,{5},RobotMode();-1,{},EnableRobot();0,{},Get"
        responses, remainder = extract_responses(buffer)
        self.assertEqual([r.error_id for r in responses], [0, -1])
        self.assertEqual([r.is_ok for r in responses], [True, False])
        self.assertEqual(remainder, b"0,{},Get")  # partial reply carried over


class _FakeConnection:
    """Records the last command sent and returns a scripted reply (no socket)."""

    def __init__(self, reply: str):
        self.reply = reply
        self.sent: "list[str]" = []

    def request(self, command: str, *, timeout_s=None) -> str:
        self.sent.append(command)
        return self.reply


class ClientWiringTests(unittest.TestCase):
    def test_dashboard_sends_exact_command_and_parses_reply(self):
        conn = _FakeConnection("0,{5},RobotMode()")
        client = DashboardClient(conn)
        resp = client.robot_mode()
        self.assertEqual(conn.sent, ["RobotMode()"])
        self.assertEqual(resp.error_id, 0)
        self.assertEqual(resp.payload, "5")

    def test_dashboard_get_pose_wiring(self):
        conn = _FakeConnection("0,{1.0,2.0,3.0,4.0},GetPose()")
        resp = DashboardClient(conn).get_pose()
        self.assertEqual(conn.sent, ["GetPose()"])
        self.assertEqual(resp.payload, "1.0,2.0,3.0,4.0")

    def test_dashboard_control_verbs_wiring(self):
        for method, command in [
            ("continue_", "Continue()"),
            ("start_drag", "StartDrag()"),
            ("stop_drag", "StopDrag()"),
        ]:
            conn = _FakeConnection(f"0,{{}},{command}")
            resp = getattr(DashboardClient(conn), method)()
            self.assertEqual(conn.sent, [command])
            self.assertTrue(resp.is_ok)

    def test_dashboard_control_verb_error_path(self):
        # A reject reply (-1) surfaces as error_id, never an exception.
        conn = _FakeConnection("-1,{},StartDrag()")
        resp = DashboardClient(conn).start_drag()
        self.assertEqual(resp.error_id, -1)
        self.assertFalse(resp.is_ok)

    def test_emergency_stop_only_on_dashboard_client(self):
        # Channel separation: E-stop is a dashboard command, not on MoveClient.
        self.assertTrue(hasattr(DashboardClient, "emergency_stop"))
        self.assertFalse(hasattr(MoveClient, "emergency_stop"))


if __name__ == "__main__":
    unittest.main()
