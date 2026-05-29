"""Socket transport for a single MG400 TCP port.

This layer knows about *sockets*, not about *robots*: it does connect / retry /
send / receive / framing, and nothing about command semantics. Higher layers
(protocol, state) sit on top. Accordingly it imports no UI and never calls
``print`` — it logs via the stdlib ``logging`` module.

Two classes:

* :class:`TcpConnection` — lifecycle (connect with retry, close, context
  manager) plus raw ``send`` / ``recv_exact``. Used directly for the 30004
  feedback port, which streams fixed 1440-byte binary frames.
* :class:`FramedConnection` — adds ``;``-delimited request/response framing on
  top, for the 29999 dashboard and 30003 move ports.

Phase 0 is synchronous on purpose: the dashboard/move protocol is inherently
request-response, so async buys no concurrency here. The streaming feedback /
state layer is where async lands (CLAUDE.md). Sockets are managed by explicit
``close()`` / context manager — never ``__del__``.
"""

from __future__ import annotations

import logging
import socket
import time
from collections import deque
from typing import Callable, Optional

from .framing import DEFAULT_TERMINATOR, extract_frames

logger = logging.getLogger(__name__)

# Factory matching ``socket.socket()``; injectable so tests can supply a fake.
SocketFactory = Callable[[], socket.socket]


class TransportError(Exception):
    """Base class for transport-layer failures."""


class NotConnectedError(TransportError):
    """Raised when an operation needs an open socket but none is connected."""


class ConnectionClosedError(TransportError):
    """Raised when the peer closes the connection mid-exchange."""


class TcpConnection:
    """A retrying, explicitly-managed TCP connection to one host:port.

    The connection is *not* opened in the constructor — call :meth:`connect`,
    or use the object as a context manager. This is what makes reconnect and
    retry possible (contrast the reference fork, which wires the socket up in
    ``__init__`` and can never recover).
    """

    def __init__(
        self,
        host: str,
        port: int,
        *,
        connect_timeout_s: float = 3.0,
        recv_timeout_s: float = 5.0,
        max_retries: int = 3,
        retry_backoff_s: float = 0.5,
        recv_chunk_size: int = 4096,
        sock_factory: SocketFactory = socket.socket,
    ) -> None:
        self.host = host
        self.port = port
        self.connect_timeout_s = connect_timeout_s
        self.recv_timeout_s = recv_timeout_s
        self.max_retries = max_retries
        self.retry_backoff_s = retry_backoff_s
        self.recv_chunk_size = recv_chunk_size
        self._sock_factory = sock_factory
        self._sock: Optional[socket.socket] = None

    # -- lifecycle ---------------------------------------------------------

    def connect(self) -> "TcpConnection":
        """Open the socket, retrying up to ``max_retries`` times with backoff.

        Returns ``self`` so it chains in a context manager. Raises
        :class:`TransportError` if every attempt fails.
        """
        if self._sock is not None:
            return self  # already connected; idempotent.

        attempts = self.max_retries + 1
        last_error: Optional[Exception] = None
        for attempt in range(1, attempts + 1):
            try:
                sock = self._sock_factory()
                sock.settimeout(self.connect_timeout_s)
                sock.connect((self.host, self.port))
                sock.settimeout(self.recv_timeout_s)
                self._sock = sock
                logger.info("Connected to %s:%d", self.host, self.port)
                return self
            except OSError as exc:
                last_error = exc
                logger.warning(
                    "Connect attempt %d/%d to %s:%d failed: %s",
                    attempt,
                    attempts,
                    self.host,
                    self.port,
                    exc,
                )
                if attempt < attempts:
                    time.sleep(self.retry_backoff_s * attempt)

        raise TransportError(
            f"Could not connect to {self.host}:{self.port} "
            f"after {attempts} attempt(s)"
        ) from last_error

    def close(self) -> None:
        """Close the socket if open. Safe to call repeatedly."""
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
                logger.info("Closed connection to %s:%d", self.host, self.port)

    @property
    def is_connected(self) -> bool:
        return self._sock is not None

    def __enter__(self) -> "TcpConnection":
        return self.connect()

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    # -- raw I/O -----------------------------------------------------------

    def _require_socket(self) -> socket.socket:
        if self._sock is None:
            raise NotConnectedError(
                f"Not connected to {self.host}:{self.port}; call connect() first"
            )
        return self._sock

    def send(self, data: bytes) -> None:
        """Send all bytes, blocking until the OS buffer accepts them."""
        sock = self._require_socket()
        sock.sendall(data)

    def recv_exact(self, size: int) -> bytes:
        """Read exactly ``size`` bytes, looping until the frame is complete.

        Used for the 30004 feedback port, where each status frame is a fixed
        1440 bytes and a single ``recv`` may return fewer. Raises
        :class:`ConnectionClosedError` if the peer closes before ``size`` bytes
        arrive.
        """
        sock = self._require_socket()
        chunks: list[bytes] = []
        remaining = size
        while remaining > 0:
            chunk = sock.recv(remaining)
            if not chunk:
                raise ConnectionClosedError(
                    f"Peer {self.host}:{self.port} closed after "
                    f"{size - remaining}/{size} bytes"
                )
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _recv_chunk(self) -> bytes:
        """Read one chunk of up to ``recv_chunk_size`` bytes."""
        sock = self._require_socket()
        chunk = sock.recv(self.recv_chunk_size)
        if not chunk:
            raise ConnectionClosedError(f"Peer {self.host}:{self.port} closed connection")
        return chunk


class FramedConnection(TcpConnection):
    """Request/response over a ``;``-delimited text protocol.

    Adds a receive buffer and :meth:`request` on top of :class:`TcpConnection`.
    Framing is delegated to the pure :func:`extract_frames`; this class only
    owns the buffer and the socket reads, so the parsing logic stays testable
    offline and reusable from an async reader later.
    """

    def __init__(self, *args, terminator: bytes = DEFAULT_TERMINATOR, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._terminator = terminator
        self._rx_buffer = b""
        self._pending: deque[str] = deque()  # complete frames not yet consumed.

    def close(self) -> None:
        super().close()
        self._rx_buffer = b""
        self._pending.clear()

    def request(self, message: str, *, timeout_s: Optional[float] = None) -> str:
        """Send ``message`` and return the next complete framed reply.

        Args:
            message: Command text *without* the trailing ``;`` terminator.
            timeout_s: Optional per-request receive timeout, overriding the
                connection default for slow commands (e.g. ``EnableRobot``,
                which can take several seconds). Restored afterwards.

        Returns the reply with terminator and surrounding whitespace stripped.
        """
        sock = self._require_socket()
        # Drop over-read residue from the previous request: this firmware answers
        # rejected commands with a non-standard double-';' frame
        # (e.g. b"-1,{},;EnableRobot();") whose 2nd fragment is an echo, not the
        # next reply. Left in _pending it would desync every later request.
        self._pending.clear()
        self._rx_buffer = b""
        self.send(message.encode("utf-8") + self._terminator)
        return self._read_frame(sock, timeout_s)

    def _read_frame(self, sock: socket.socket, timeout_s: Optional[float]) -> str:
        # Serve a buffered frame first if the previous read over-read.
        if self._pending:
            return self._pending.popleft()

        previous_timeout = sock.gettimeout()
        if timeout_s is not None:
            sock.settimeout(timeout_s)
        try:
            while not self._pending:
                self._rx_buffer += self._recv_chunk()
                messages, self._rx_buffer = extract_frames(
                    self._rx_buffer, self._terminator
                )
                self._pending.extend(messages)
        finally:
            if timeout_s is not None:
                sock.settimeout(previous_timeout)
        return self._pending.popleft()
