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
    parse_angle,
    parse_error_id,
    parse_pose,
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

    def test_sync_command(self):
        self.assertEqual(builders.sync(), "Sync()")

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

    def test_enable_robot_signatures_0_1_4(self):
        # 0 / 1 / 4 params per official EnableRobot(load,centerX,centerY,centerZ).
        self.assertEqual(builders.enable_robot(), "EnableRobot()")
        self.assertEqual(builders.enable_robot(0.5), "EnableRobot(0.500000)")
        self.assertEqual(
            builders.enable_robot(0.5, 0, 0, 5.5),
            "EnableRobot(0.500000,0.000000,0.000000,5.500000)",
        )

    def test_mov_optional_kwargs_keyvalue_format(self):
        # Optional params appended as ",Key=value" in official order.
        self.assertEqual(
            builders.mov_l(100, 200, 50, 0, speed_l=60),
            "MovL(100.000000,200.000000,50.000000,0.000000,SpeedL=60)",
        )
        self.assertEqual(
            builders.mov_l(1, 2, 3, 4, user=1, tool=2, speed_l=60, acc_l=50, cp=10),
            "MovL(1.000000,2.000000,3.000000,4.000000,User=1,Tool=2,SpeedL=60,AccL=50,CP=10)",
        )
        self.assertEqual(
            builders.mov_j(1, 2, 3, 4, speed_j=80, acc_j=50),
            "MovJ(1.000000,2.000000,3.000000,4.000000,SpeedJ=80,AccJ=50)",
        )
        self.assertEqual(
            builders.joint_mov_j(0, 0, 60, 0, speed_j=60, acc_j=50, cp=0),
            "JointMovJ(0.000000,0.000000,60.000000,0.000000,SpeedJ=60,AccJ=50,CP=0)",
        )

    def test_mov_without_kwargs_unchanged(self):
        # Omitting all optional params yields the bare 4-arg form (back-compat).
        self.assertEqual(
            builders.mov_l(100, 200, 50, 0), "MovL(100.000000,200.000000,50.000000,0.000000)"
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

    def test_enable_robot_rejects_invalid_param_counts(self):
        # Firmware accepts only 0/1/4: partial centre-of-mass is rejected.
        with self.assertRaises(CommandValidationError):
            builders.enable_robot(0.5, 0)  # 2 params
        with self.assertRaises(CommandValidationError):
            builders.enable_robot(0.5, 0, 0)  # 3 params
        with self.assertRaises(CommandValidationError):
            builders.enable_robot(center_x=0, center_y=0, center_z=0)  # centres w/o load

    def test_enable_robot_rejects_out_of_range(self):
        with self.assertRaises(CommandValidationError):
            builders.enable_robot(-0.1)  # negative load
        with self.assertRaises(CommandValidationError):
            builders.enable_robot(0.5, 600, 0, 0)  # centerX > 500
        with self.assertRaises(CommandValidationError):
            builders.enable_robot(0.5, 0, -501, 0)  # centerY < -500

    def test_mov_optional_kwargs_out_of_range(self):
        with self.assertRaises(CommandValidationError):
            builders.mov_l(0, 0, 0, 0, speed_l=0)  # ratio < 1
        with self.assertRaises(CommandValidationError):
            builders.mov_l(0, 0, 0, 0, acc_l=101)  # ratio > 100
        with self.assertRaises(CommandValidationError):
            builders.mov_l(0, 0, 0, 0, cp=101)  # CP > 100
        with self.assertRaises(CommandValidationError):
            builders.mov_l(0, 0, 0, 0, user=10)  # index > 9
        with self.assertRaises(CommandValidationError):
            builders.mov_j(0, 0, 0, 0, speed_j=1.5)  # ratio must be int
        with self.assertRaises(CommandValidationError):
            builders.joint_mov_j(0, 0, 0, 0, cp=-1)  # CP < 0


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

    def test_parse_pose_typed(self):
        result = parse_pose(parse_response("0,{197.23,-0.02,-30.26,2.67},GetPose()"))
        self.assertTrue(result.is_ok)
        self.assertAlmostEqual(result.x, 197.23)
        self.assertAlmostEqual(result.y, -0.02)
        self.assertAlmostEqual(result.z, -30.26)
        self.assertAlmostEqual(result.r, 2.67)

    def test_parse_pose_error_returns_no_values(self):
        result = parse_pose(parse_response("-1,{},GetPose()"))
        self.assertEqual(result.error_id, -1)
        self.assertFalse(result.is_ok)
        self.assertIsNone(result.x)

    def test_parse_angle_typed(self):
        result = parse_angle(parse_response("0,{0.0,20.0,60.0,0.0},GetAngle()"))
        self.assertTrue(result.is_ok)
        self.assertEqual((result.j1, result.j2, result.j3, result.j4), (0.0, 20.0, 60.0, 0.0))

    def test_parse_angle_accepts_six_values_from_4axis_firmware(self):
        # Real hardware (4-axis MG400, controller 1.7.0.0): GetAngle returns 6
        # comma-separated values with J5,J6 padded as 0. The firmware shares the
        # 6-axis SDK protocol; we take the first 4 and discard the padding.
        # See PROGRESS finding (post-2026-06-02 hardware verification).
        raw = "0,{1.568910,43.076424,31.149742,15.652771,0.000000,0.000000},GetAngle()"
        result = parse_angle(parse_response(raw))
        self.assertTrue(result.is_ok)
        self.assertAlmostEqual(result.j1, 1.568910)
        self.assertAlmostEqual(result.j2, 43.076424)
        self.assertAlmostEqual(result.j3, 31.149742)
        self.assertAlmostEqual(result.j4, 15.652771)

    def test_parse_pose_wrong_arity_raises(self):
        # Fewer than 4 values is a real error (truncated payload).
        with self.assertRaises(ProtocolResponseError):
            parse_pose(parse_response("0,{1.0,2.0,3.0},GetPose()"))

    def test_extract_responses_splits_stream(self):
        buffer = b"0,{5},RobotMode();-1,{},EnableRobot();0,{},Get"
        responses, remainder = extract_responses(buffer)
        self.assertEqual([r.error_id for r in responses], [0, -1])
        self.assertEqual([r.is_ok for r in responses], [True, False])
        self.assertEqual(remainder, b"0,{},Get")  # partial reply carried over


class GetErrorIDParsingTests(unittest.TestCase):
    def test_parses_nested_controller_and_servo_groups(self):
        # Realistic firmware reply: [controller, servo1..6]; only ctrl + s1-4 kept.
        result = parse_error_id(
            parse_response("0,{[[112,114],[0],[0],[0],[0],[0],[0]]},GetErrorID()")
        )
        self.assertTrue(result.is_ok)
        self.assertEqual(result.controller_errors, (112, 114))
        self.assertEqual(result.servo_errors, ((0,), (0,), (0,), (0,)))
        self.assertTrue(result.has_active_errors)

    def test_no_active_errors(self):
        result = parse_error_id(parse_response("0,{[[],[],[],[],[]]},GetErrorID()"))
        self.assertTrue(result.is_ok)
        self.assertEqual(result.controller_errors, ())
        self.assertFalse(result.has_active_errors)

    def test_error_reply_yields_empty_result(self):
        result = parse_error_id(parse_response("-1,{},GetErrorID()"))
        self.assertEqual(result.error_id, -1)
        self.assertFalse(result.is_ok)
        self.assertEqual(result.controller_errors, ())
        self.assertEqual(result.servo_errors, ())

    def test_malformed_payload_raises(self):
        with self.assertRaises(ProtocolResponseError):
            parse_error_id(parse_response("0,{not-a-list},GetErrorID()"))

    def test_flat_list_not_nested_raises(self):
        with self.assertRaises(ProtocolResponseError):
            parse_error_id(parse_response("0,{[1,2,3]},GetErrorID()"))


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
        result = DashboardClient(conn).get_pose()
        self.assertEqual(conn.sent, ["GetPose()"])
        self.assertTrue(result.is_ok)
        self.assertEqual((result.x, result.y, result.z, result.r), (1.0, 2.0, 3.0, 4.0))

    def test_dashboard_get_angle_wiring(self):
        conn = _FakeConnection("0,{0.0,20.0,60.0,0.0},GetAngle()")
        result = DashboardClient(conn).get_angle()
        self.assertEqual(conn.sent, ["GetAngle()"])
        self.assertEqual((result.j1, result.j2, result.j3, result.j4), (0.0, 20.0, 60.0, 0.0))

    def test_dashboard_get_error_id_wiring(self):
        conn = _FakeConnection("0,{[[112],[0],[0],[0],[0]]},GetErrorID()")
        result = DashboardClient(conn).get_error_id()
        self.assertEqual(conn.sent, ["GetErrorID()"])
        self.assertEqual(result.controller_errors, (112,))
        self.assertTrue(result.has_active_errors)

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

    def test_move_client_sync_wiring(self):
        conn = _FakeConnection("0,{},Sync()")
        resp = MoveClient(conn).sync()
        self.assertEqual(conn.sent, ["Sync()"])
        self.assertTrue(resp.is_ok)

    def test_sync_only_on_move_client(self):
        # Channel separation: Sync drains the move queue (30003), not a dashboard cmd.
        self.assertTrue(hasattr(MoveClient, "sync"))
        self.assertFalse(hasattr(DashboardClient, "sync"))

    def test_dashboard_enable_robot_with_load_wiring(self):
        conn = _FakeConnection("0,{},EnableRobot()")
        DashboardClient(conn).enable_robot(0.5)
        self.assertEqual(conn.sent, ["EnableRobot(0.500000)"])

    def test_dashboard_enable_robot_no_args_wiring(self):
        conn = _FakeConnection("0,{},EnableRobot()")
        DashboardClient(conn).enable_robot()
        self.assertEqual(conn.sent, ["EnableRobot()"])

    def test_move_client_mov_l_kwargs_wiring(self):
        conn = _FakeConnection("0,{},MovL()")
        MoveClient(conn).mov_l(1, 2, 3, 4, speed_l=60, cp=10)
        self.assertEqual(
            conn.sent, ["MovL(1.000000,2.000000,3.000000,4.000000,SpeedL=60,CP=10)"]
        )

    def test_move_client_joint_mov_j_kwargs_wiring(self):
        conn = _FakeConnection("0,{},JointMovJ()")
        MoveClient(conn).joint_mov_j(0, 0, 60, 0, speed_j=60, acc_j=50)
        self.assertEqual(
            conn.sent, ["JointMovJ(0.000000,0.000000,60.000000,0.000000,SpeedJ=60,AccJ=50)"]
        )


class CoordinateBuilderTests(unittest.TestCase):
    """Phase 3.2 coordinate-system & kinematics command strings + validation."""

    def test_user_wire(self):
        self.assertEqual(builders.user(1), "User(1)")
        self.assertEqual(builders.user(0), "User(0)")
        self.assertEqual(builders.user(9), "User(9)")

    def test_user_rejects_bad_index(self):
        for bad in (-1, 10, 100):  # outside [0, 9]
            with self.assertRaises(CommandValidationError):
                builders.user(bad)
        for bad in (1.5, "1", True, None):  # float, str, bool, None all rejected
            with self.assertRaises(CommandValidationError):
                builders.user(bad)

    def test_tool_wire(self):
        self.assertEqual(builders.tool(1), "Tool(1)")
        self.assertEqual(builders.tool(0), "Tool(0)")
        self.assertEqual(builders.tool(9), "Tool(9)")

    def test_tool_rejects_bad_index(self):
        for bad in (-1, 10, 100):
            with self.assertRaises(CommandValidationError):
                builders.tool(bad)

    def test_tool_rejects_non_integer(self):
        for bad in (1.5, "1", True, None):  # float, str, bool, None all rejected
            with self.assertRaises(CommandValidationError):
                builders.tool(bad)

    def test_set_user_wire(self):
        self.assertEqual(
            builders.set_user(1, (10, 10, 10, 0)),
            "SetUser(1,{10.000000,10.000000,10.000000,0.000000})",
        )

    def test_set_user_rejects_bad_index(self):
        for bad in (-1, 10, 1.5, True, "3"):
            with self.assertRaises(CommandValidationError):
                builders.set_user(bad, (10, 10, 10, 0))

    def test_set_user_rejects_bad_table(self):
        # _coord_table requires exactly 4 finite numbers.
        for bad_table in ((10, 10, 10), (10, 10, 10, 0, 0), (10, 10, 10, float("nan"))):
            with self.assertRaises(CommandValidationError):
                builders.set_user(1, bad_table)

    def test_set_tool_wire(self):
        self.assertEqual(
            builders.set_tool(1, (10, 10, 10, 0)),
            "SetTool(1,{10.000000,10.000000,10.000000,0.000000})",
        )

    def test_set_tool_rejects_bad_index(self):
        for bad in (-1, 10, 1.0, "1", True):
            with self.assertRaises(CommandValidationError):
                builders.set_tool(bad, (10, 10, 10, 0))

    def test_set_tool_rejects_bad_table(self):
        for bad in ((10, 10, 10), (10, 10, 10, 0, 0), (10, 10, 10, "r"),
                    (10, 10, 10, float("nan")), (10, 10, 10, float("inf"))):
            with self.assertRaises(CommandValidationError):
                builders.set_tool(1, bad)

    def test_calc_user_wire(self):
        self.assertEqual(
            builders.calc_user(1, 1, (10, 10, 10, 10)),
            "CalcUser(1,1,{10.000000,10.000000,10.000000,10.000000})",
        )

    def test_calc_user_rejects_bad_index(self):
        for bad in (-1, 10, 1.0, True, "1"):
            with self.assertRaises(CommandValidationError):
                builders.calc_user(bad, 1, (10, 10, 10, 10))

    def test_calc_user_rejects_bad_matrix_direction(self):
        for bad in (-1, 2, 1.0, True, "1"):
            with self.assertRaises(CommandValidationError):
                builders.calc_user(1, bad, (10, 10, 10, 10))

    def test_calc_user_rejects_bad_table(self):
        for bad in ((10, 10, 10), (10, 10, 10, 10, 10), ("x", 10, 10, 10), (10, 10, 10, float("inf"))):
            with self.assertRaises(CommandValidationError):
                builders.calc_user(1, 1, bad)

    def test_calc_tool_wire(self):
        self.assertEqual(
            builders.calc_tool(1, 1, (10, 10, 10, 10)),
            "CalcTool(1,1,{10.000000,10.000000,10.000000,10.000000})",
        )

    def test_calc_tool_rejects_bad_matrix_direction(self):
        for bad in (-1, 2, 0.0, True, "0"):
            with self.assertRaises(CommandValidationError):
                builders.calc_tool(1, bad, (10, 10, 10, 10))

    def test_positive_solution_wire(self):
        self.assertEqual(
            builders.positive_solution(0, 0, 90, 0, 1, 1),
            "PositiveSolution(0.000000,0.000000,90.000000,0.000000,1,1)",
        )

    def test_positive_solution_rejects_bad_index(self):
        # user / tool must be ints in [0, 9].
        with self.assertRaises(CommandValidationError):
            builders.positive_solution(0, 0, 90, 0, -1, 1)  # user < 0
        with self.assertRaises(CommandValidationError):
            builders.positive_solution(0, 0, 90, 0, 1, 10)  # tool > 9
        with self.assertRaises(CommandValidationError):
            builders.positive_solution(0, 0, 90, 0, 1.5, 1)  # user not int
        with self.assertRaises(CommandValidationError):
            builders.positive_solution(0, 0, 90, 0, True, 1)  # bool rejected

    def test_positive_solution_rejects_bad_joint(self):
        with self.assertRaises(CommandValidationError):
            builders.positive_solution(200, 0, 90, 0, 1, 1)  # J1 > 160
        with self.assertRaises(CommandValidationError):
            builders.positive_solution(0, 90, 90, 0, 1, 1)  # J2 > 85
        with self.assertRaises(CommandValidationError):
            builders.positive_solution(0, 0, 120, 0, 1, 1)  # J3 > 105

    def test_inverse_solution_wire(self):
        self.assertEqual(
            builders.inverse_solution(473, -141, 469, -180, 0, 0),
            "InverseSolution(473.000000,-141.000000,469.000000,-180.000000,0,0)",
        )
        self.assertEqual(
            builders.inverse_solution(473, -141, 469, -180, 0, 0, joint_near=(0, 0, 90, 0)),
            "InverseSolution(473.000000,-141.000000,469.000000,-180.000000,0,0,1,"
            "{0.000000,0.000000,90.000000,0.000000})",
        )

    def test_inverse_solution_rejects_bad_index(self):
        for bad in (-1, 10, 1.0, "0", True):
            with self.assertRaises(CommandValidationError):
                builders.inverse_solution(473, -141, 469, -180, bad, 0)
            with self.assertRaises(CommandValidationError):
                builders.inverse_solution(473, -141, 469, -180, 0, bad)

    def test_inverse_solution_rejects_non_number_cartesian(self):
        for bad in ("x", None, float("nan"), float("inf"), True):
            with self.assertRaises(CommandValidationError):
                builders.inverse_solution(bad, 0, 0, 0, 0, 0)

    def test_inverse_solution_rejects_bad_joint_near(self):
        with self.assertRaises(CommandValidationError):  # wrong arity
            builders.inverse_solution(473, -141, 469, -180, 0, 0, joint_near=(0, 0, 0))
        with self.assertRaises(CommandValidationError):  # J2 > 85
            builders.inverse_solution(473, -141, 469, -180, 0, 0, joint_near=(0, 90, 0, 0))
        with self.assertRaises(CommandValidationError):  # non-number element
            builders.inverse_solution(473, -141, 469, -180, 0, 0, joint_near=(0, 0, "x", 0))

    def test_inverse_solution_does_not_validate_reachability(self):
        # A far-away point still builds: reachability is not this layer's job.
        self.assertEqual(
            builders.inverse_solution(9999, 0, 0, 0, 0, 0),
            "InverseSolution(9999.000000,0.000000,0.000000,0.000000,0,0)",
        )

    def test_get_pose_no_args(self):
        # `GetPose(User=,Tool=)` was removed after hardware proved firmware
        # 1.7.0.0 rejects keyword syntax with -30001 and silently ignores
        # positional args (returns base pose regardless). See PROGRESS
        # finding 22 — per-frame query is a client-side transform job now.
        self.assertEqual(builders.get_pose(), "GetPose()")


class CoordinateClientWiringTests(unittest.TestCase):
    """Coordinate/kinematics commands route through DashboardClient (29999)."""

    def test_user_client_wiring(self):
        conn = _FakeConnection("0,{},User(1)")
        resp = DashboardClient(conn).user(1)
        self.assertEqual(conn.sent, ["User(1)"])
        self.assertTrue(resp.is_ok)

    def test_tool_client_wiring(self):
        conn = _FakeConnection("0,{},Tool(1)")
        resp = DashboardClient(conn).tool(1)
        self.assertEqual(conn.sent, ["Tool(1)"])
        self.assertTrue(resp.is_ok)

    def test_set_user_client_wiring(self):
        conn = _FakeConnection("0,{},SetUser()")
        resp = DashboardClient(conn).set_user(1, (10, 10, 10, 0))
        self.assertEqual(conn.sent, ["SetUser(1,{10.000000,10.000000,10.000000,0.000000})"])
        self.assertTrue(resp.is_ok)

    def test_set_tool_client_wiring(self):
        conn = _FakeConnection("0,{},SetTool()")
        resp = DashboardClient(conn).set_tool(1, (10, 10, 10, 0))
        self.assertEqual(conn.sent, ["SetTool(1,{10.000000,10.000000,10.000000,0.000000})"])
        self.assertTrue(resp.is_ok)

    def test_calc_user_client_wiring_tags_user_index(self):
        conn = _FakeConnection("0,{197.23,-0.02,-30.26,2.67},CalcUser()")
        result = DashboardClient(conn).calc_user(1, 1, (10, 10, 10, 10))
        self.assertEqual(conn.sent, ["CalcUser(1,1,{10.000000,10.000000,10.000000,10.000000})"])
        self.assertEqual((result.x, result.y, result.z, result.r), (197.23, -0.02, -30.26, 2.67))
        self.assertEqual(result.user_index, 1)

    def test_calc_tool_client_wiring_tags_tool_index(self):
        conn = _FakeConnection("0,{1.0,2.0,3.0,4.0},CalcTool()")
        result = DashboardClient(conn).calc_tool(2, 1, (10, 10, 10, 10))
        self.assertEqual(conn.sent, ["CalcTool(2,1,{10.000000,10.000000,10.000000,10.000000})"])
        self.assertEqual((result.x, result.y, result.z, result.r), (1.0, 2.0, 3.0, 4.0))
        self.assertEqual(result.tool_index, 2)

    def test_positive_solution_client_wiring(self):
        conn = _FakeConnection("0,{1.0,2.0,3.0,4.0},PositiveSolution()")
        result = DashboardClient(conn).positive_solution(0, 0, 90, 0, 1, 1)
        self.assertEqual(conn.sent, ["PositiveSolution(0.000000,0.000000,90.000000,0.000000,1,1)"])
        self.assertEqual((result.x, result.y, result.z, result.r), (1.0, 2.0, 3.0, 4.0))
        self.assertEqual((result.user_index, result.tool_index), (1, 1))

    def test_inverse_solution_client_wiring(self):
        conn = _FakeConnection("0,{0.0,20.0,60.0,0.0},InverseSolution()")
        result = DashboardClient(conn).inverse_solution(473, -141, 469, -180, 0, 0)
        self.assertEqual(
            conn.sent, ["InverseSolution(473.000000,-141.000000,469.000000,-180.000000,0,0)"]
        )
        self.assertEqual((result.j1, result.j2, result.j3, result.j4), (0.0, 20.0, 60.0, 0.0))

    def test_get_pose_no_args(self):
        # `DashboardClient.get_pose()` is no-args after the hardware-verified
        # firmware 1.7.0.0 limitation; user_index/tool_index stay None on the
        # result. Per-frame queries are a client-side transform problem now
        # (PROGRESS finding 22 + Phase 6.1 transform.py).
        conn = _FakeConnection("0,{1.0,2.0,3.0,4.0},GetPose()")
        result = DashboardClient(conn).get_pose()
        self.assertEqual(conn.sent, ["GetPose()"])
        self.assertEqual((result.x, result.y, result.z, result.r), (1.0, 2.0, 3.0, 4.0))
        self.assertIsNone(result.user_index)
        self.assertIsNone(result.tool_index)


if __name__ == "__main__":
    unittest.main()
