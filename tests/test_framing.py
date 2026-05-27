"""Unit tests for protocol framing and the framed request/response loop.

Pure logic — no real sockets, no robot. A scripted ``FakeSocket`` lets us
verify that message framing survives arbitrary ``recv`` boundaries.
"""

import unittest

from robot_core.config import RobotConfig
from robot_core.transport.connection import (
    ConnectionClosedError,
    FramedConnection,
    NotConnectedError,
    TcpConnection,
    TransportError,
)
from robot_core.transport.framing import extract_frames


class ExtractFramesTests(unittest.TestCase):
    def test_single_complete_message(self):
        messages, remainder = extract_frames(b"RobotMode();")
        self.assertEqual(messages, ["RobotMode()"])
        self.assertEqual(remainder, b"")

    def test_partial_message_is_held_as_remainder(self):
        messages, remainder = extract_frames(b"0,{5},Robot")
        self.assertEqual(messages, [])
        self.assertEqual(remainder, b"0,{5},Robot")

    def test_multiple_messages_in_one_buffer(self):
        messages, remainder = extract_frames(b"resp1;resp2;resp3;")
        self.assertEqual(messages, ["resp1", "resp2", "resp3"])
        self.assertEqual(remainder, b"")

    def test_trailing_fragment_after_complete_messages(self):
        messages, remainder = extract_frames(b"resp1;resp2;par")
        self.assertEqual(messages, ["resp1", "resp2"])
        self.assertEqual(remainder, b"par")

    def test_reassembly_across_two_chunks(self):
        # Simulate a message split by a recv boundary: feed the remainder back.
        messages, remainder = extract_frames(b"0,{1,2,")
        self.assertEqual(messages, [])
        messages, remainder = extract_frames(remainder + b"3},GetPose();")
        self.assertEqual(messages, ["0,{1,2,3},GetPose()"])
        self.assertEqual(remainder, b"")

    def test_whitespace_is_stripped_and_empty_frames_dropped(self):
        messages, remainder = extract_frames(b"  resp1  ;;resp2;")
        self.assertEqual(messages, ["resp1", "resp2"])  # the empty ;; is dropped
        self.assertEqual(remainder, b"")

    def test_empty_buffer(self):
        messages, remainder = extract_frames(b"")
        self.assertEqual(messages, [])
        self.assertEqual(remainder, b"")

    def test_utf8_payload(self):
        messages, _ = extract_frames("錯誤;".encode("utf-8"))
        self.assertEqual(messages, ["錯誤"])

    def test_empty_terminator_rejected(self):
        with self.assertRaises(ValueError):
            extract_frames(b"x", terminator=b"")


class FakeSocket:
    """Minimal scripted stand-in for ``socket.socket`` used by the transport.

    ``recv_chunks`` is consumed one entry per ``recv`` call; an exhausted list
    returns ``b""`` to model the peer closing the connection. ``connect_errors``
    raises the given exception on the first calls to exercise retry logic.
    """

    def __init__(self, recv_chunks=(), connect_error=None):
        self._recv_chunks = list(recv_chunks)
        self._connect_error = connect_error
        self.sent = b""
        self.timeout = None
        self.connected_to = None
        self.closed = False

    def settimeout(self, value):
        self.timeout = value

    def gettimeout(self):
        return self.timeout

    def connect(self, address):
        if self._connect_error is not None:
            raise self._connect_error
        self.connected_to = address

    def sendall(self, data):
        self.sent += data

    def recv(self, _max_bytes):
        if not self._recv_chunks:
            return b""
        return self._recv_chunks.pop(0)

    def close(self):
        self.closed = True


class FramedConnectionTests(unittest.TestCase):
    def _framed(self, recv_chunks):
        sock = FakeSocket(recv_chunks=recv_chunks)
        conn = FramedConnection(
            "192.0.2.1", 29999, sock_factory=lambda: sock, retry_backoff_s=0
        )
        conn.connect()
        return conn, sock

    def test_request_sends_command_with_terminator(self):
        conn, sock = self._framed([b"0,{},EnableRobot();"])
        conn.request("EnableRobot()")
        self.assertEqual(sock.sent, b"EnableRobot();")

    def test_request_returns_single_reply(self):
        conn, _ = self._framed([b"0,{5},RobotMode();"])
        self.assertEqual(conn.request("RobotMode()"), "0,{5},RobotMode()")

    def test_request_reassembles_reply_split_across_recvs(self):
        conn, _ = self._framed([b"0,{1,2,", b"3,4},GetPo", b"se();"])
        self.assertEqual(conn.request("GetPose()"), "0,{1,2,3,4},GetPose()")

    def test_over_read_reply_is_buffered_for_next_request(self):
        # One recv delivers two replies; the second must be served from buffer.
        conn, sock = self._framed([b"0,{a},GetPose();0,{b},GetAngle();"])
        self.assertEqual(conn.request("GetPose()"), "0,{a},GetPose()")
        # Second request should not need any further recv (chunks exhausted).
        self.assertEqual(conn.request("GetAngle()"), "0,{b},GetAngle()")

    def test_peer_close_midframe_raises(self):
        conn, _ = self._framed([b"0,{partial"])  # then recv() -> b""
        with self.assertRaises(ConnectionClosedError):
            conn.request("RobotMode()")

    def test_request_without_connect_raises(self):
        conn = FramedConnection("192.0.2.1", 29999, sock_factory=FakeSocket)
        with self.assertRaises(NotConnectedError):
            conn.request("RobotMode()")

    def test_context_manager_connects_and_closes(self):
        sock = FakeSocket(recv_chunks=[b"ok;"])
        with FramedConnection(
            "192.0.2.1", 29999, sock_factory=lambda: sock
        ) as conn:
            self.assertTrue(conn.is_connected)
            self.assertEqual(conn.request("RobotMode()"), "ok")
        self.assertTrue(sock.closed)
        self.assertFalse(conn.is_connected)


class ConnectRetryTests(unittest.TestCase):
    def test_connect_retries_then_raises_transport_error(self):
        attempts = []

        def failing_factory():
            sock = FakeSocket(connect_error=OSError("refused"))
            attempts.append(sock)
            return sock

        conn = FramedConnection(
            "192.0.2.1",
            29999,
            sock_factory=failing_factory,
            max_retries=2,
            retry_backoff_s=0,
        )
        with self.assertRaises(TransportError):
            conn.connect()
        # max_retries=2 means 3 total attempts.
        self.assertEqual(len(attempts), 3)

    def test_connect_succeeds_after_transient_failures(self):
        calls = {"n": 0}

        def flaky_factory():
            calls["n"] += 1
            if calls["n"] < 3:
                return FakeSocket(connect_error=OSError("refused"))
            return FakeSocket(recv_chunks=[b"ok;"])

        conn = FramedConnection(
            "192.0.2.1",
            29999,
            sock_factory=flaky_factory,
            max_retries=5,
            retry_backoff_s=0,
        )
        conn.connect()
        self.assertTrue(conn.is_connected)
        self.assertEqual(calls["n"], 3)


class RecvExactTests(unittest.TestCase):
    """Cover TcpConnection.recv_exact's multi-recv accumulation loop directly.

    Feedback frames are a fixed 1440 bytes that the OS may hand over in several
    chunks; on real hardware at high frequency this loop is always exercised.
    The earlier feedback test used a FakeConn that returned the whole payload at
    once, which bypassed this path.
    """

    def _conn(self, recv_chunks):
        sock = FakeSocket(recv_chunks=recv_chunks)
        conn = TcpConnection(
            "192.0.2.1", 30004, sock_factory=lambda: sock, retry_backoff_s=0
        )
        conn.connect()
        return conn, sock

    def test_reassembles_fixed_size_payload_across_chunks(self):
        payload = bytes((i % 256) for i in range(1440))
        # 1440 split as 500 + 500 + 440, delivered one chunk per recv().
        chunks = [payload[:500], payload[500:1000], payload[1000:]]
        conn, _ = self._conn(chunks)
        result = conn.recv_exact(1440)
        self.assertEqual(len(result), 1440)
        self.assertEqual(result, payload)

    def test_raises_when_peer_closes_midway(self):
        # recv yields 3 bytes, then b"" (peer closed) before 10 are collected.
        conn, _ = self._conn([b"abc"])
        with self.assertRaises(ConnectionClosedError):
            conn.recv_exact(10)


class ConfigTests(unittest.TestCase):
    BASE = {
        "ip": "192.168.1.6",
        "ports": {"dashboard": 29999, "move": 30003, "feedback": 30004},
        "transport": {},
    }

    def test_defaults_from_dict(self):
        cfg = RobotConfig.from_dict(self.BASE)
        self.assertEqual(cfg.ip, "192.168.1.6")
        self.assertEqual(cfg.dashboard_port, 29999)
        self.assertEqual(cfg.move_port, 30003)
        self.assertEqual(cfg.feedback_port, 30004)

    def test_env_overrides(self):
        import os
        from unittest import mock

        with mock.patch.dict(
            os.environ, {"MG400_IP": "10.0.0.5", "MG400_DASHBOARD_PORT": "40000"}
        ):
            cfg = RobotConfig.from_dict(self.BASE)
        self.assertEqual(cfg.ip, "10.0.0.5")
        self.assertEqual(cfg.dashboard_port, 40000)
        self.assertEqual(cfg.feedback_port, 30004)  # untouched


if __name__ == "__main__":
    unittest.main()
