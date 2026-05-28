"""Typed kinematics configuration loaded from ``config/kinematics.json``.

Mechanism parameters (link lengths, base offsets, the r-axis definition) are
data, not code: they live in JSON and were back-fitted from real measurements
(see the file's ``provenance``). Keeping them here means re-calibration is a JSON
edit, and the forward-kinematics code stays a pure function of inputs + config.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

# config/kinematics.json at the repo root (two levels above this file's package).
DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parent.parent.parent / "config" / "kinematics.json"
)


@dataclass(frozen=True)
class KinematicsConfig:
    """MG400 parallelogram mechanism parameters (units: mm, deg).

    See ``config/kinematics.json`` ``model_notes`` for the exact equations the
    forward kinematics applies.
    """

    l1_rear_arm_mm: float
    l2_forearm_mm: float
    base_r_mm: float
    base_z_mm: float
    # r = j1_coeff*J1 + j4_coeff*J4 + offset_deg, optionally wrapped to +/-180.
    r_j1_coeff: float
    r_j4_coeff: float
    r_offset_deg: float
    r_wrap: bool

    @classmethod
    def from_dict(cls, raw: dict) -> "KinematicsConfig":
        links = raw["links"]
        r_axis = raw["r_axis"]
        return cls(
            l1_rear_arm_mm=float(links["L1_rear_arm_mm"]),
            l2_forearm_mm=float(links["L2_forearm_mm"]),
            base_r_mm=float(links["base_r_mm"]),
            base_z_mm=float(links["base_z_mm"]),
            r_j1_coeff=float(r_axis["j1_coeff"]),
            r_j4_coeff=float(r_axis["j4_coeff"]),
            r_offset_deg=float(r_axis.get("offset_deg", 0.0)),
            r_wrap=bool(r_axis.get("wrap", False)),
        )

    @classmethod
    def load(cls, path: "str | os.PathLike[str] | None" = None) -> "KinematicsConfig":
        config_path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
        with open(config_path, encoding="utf-8") as fh:
            return cls.from_dict(json.load(fh))


@lru_cache(maxsize=1)
def default_config() -> KinematicsConfig:
    """The default config, loaded once from :data:`DEFAULT_CONFIG_PATH` and cached."""
    return KinematicsConfig.load()
