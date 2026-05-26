"""Typed configuration loaded from ``config/robot.json``.

IP and port numbers are never hard-coded in the code base (see CLAUDE.md). They
live in ``config/robot.json`` and can be overridden per-run via environment
variables, which is handy for testing against a simulator or a second arm:

    MG400_IP=192.168.1.20 python -m robot_core.scripts.connect_test

Recognised overrides: ``MG400_IP``, ``MG400_DASHBOARD_PORT``,
``MG400_MOVE_PORT``, ``MG400_FEEDBACK_PORT``.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path

# config/robot.json sits at the repo root, one level above robot_core/.
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "robot.json"


@dataclass(frozen=True)
class TransportSettings:
    """Tunables for the socket transport layer (timeouts and reconnect policy)."""

    connect_timeout_s: float = 3.0
    recv_timeout_s: float = 5.0
    max_retries: int = 3
    retry_backoff_s: float = 0.5


@dataclass(frozen=True)
class RobotConfig:
    """Connection settings for a single MG400 controller.

    Build with :meth:`load` to read from JSON; the constructor itself stays
    pure data so tests can instantiate it directly without touching the disk.
    """

    ip: str
    dashboard_port: int
    move_port: int
    feedback_port: int
    transport: TransportSettings

    @classmethod
    def load(cls, path: str | os.PathLike[str] | None = None) -> "RobotConfig":
        """Load configuration from JSON, then apply environment overrides."""
        config_path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
        with open(config_path, encoding="utf-8") as fh:
            raw = json.load(fh)
        return cls.from_dict(raw)

    @classmethod
    def from_dict(cls, raw: dict) -> "RobotConfig":
        """Build a config from an already-parsed mapping, applying env overrides.

        Kept separate from :meth:`load` so the override logic is unit-testable
        without a file on disk.
        """
        ports = raw.get("ports", {})
        transport_raw = raw.get("transport", {})

        ip = os.environ.get("MG400_IP", raw["ip"])
        dashboard_port = _env_int("MG400_DASHBOARD_PORT", ports["dashboard"])
        move_port = _env_int("MG400_MOVE_PORT", ports["move"])
        feedback_port = _env_int("MG400_FEEDBACK_PORT", ports["feedback"])

        transport = TransportSettings(
            connect_timeout_s=float(transport_raw.get("connect_timeout_s", 3.0)),
            recv_timeout_s=float(transport_raw.get("recv_timeout_s", 5.0)),
            max_retries=int(transport_raw.get("max_retries", 3)),
            retry_backoff_s=float(transport_raw.get("retry_backoff_s", 0.5)),
        )
        return cls(
            ip=ip,
            dashboard_port=dashboard_port,
            move_port=move_port,
            feedback_port=feedback_port,
            transport=transport,
        )


def _env_int(name: str, default: int) -> int:
    """Read an integer environment override, falling back to ``default``."""
    value = os.environ.get(name)
    return int(value) if value is not None else int(default)
