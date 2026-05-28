"""Calibration / verification interface for the forward kinematics.

Feed it ``(joint angles, measured pose)`` pairs — e.g. captured with
``scripts/collect_pairs.py`` — and it reports the per-axis error between the FK
prediction and the real measurement, plus max/mean summaries. This is how the
mechanism parameters in ``config/kinematics.json`` are validated against the
physical arm (and how you'd judge a re-fit). Pure math, no hardware.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

from .config import KinematicsConfig
from .forward import forward_kinematics

DEFAULT_PAIRS_PATH = (
    Path(__file__).resolve().parent.parent.parent / "config" / "calibration_pairs.json"
)


@dataclass(frozen=True)
class CalibrationSample:
    """One measured posture: joint angles (deg) and the observed pose."""

    joints: tuple[float, float, float, float]  # J1..J4, deg
    measured_pose: tuple[float, float, float, float]  # x, y, z (mm), r (deg)
    label: str = ""


@dataclass(frozen=True)
class PoseError:
    """Signed per-axis error (FK prediction minus measurement) for one sample."""

    label: str
    dx: float
    dy: float
    dz: float
    dr: float

    @property
    def position_error_mm(self) -> float:
        """Euclidean position error over x/y/z (mm)."""
        return math.sqrt(self.dx * self.dx + self.dy * self.dy + self.dz * self.dz)

    @property
    def abs_r_error_deg(self) -> float:
        return abs(self.dr)


@dataclass(frozen=True)
class CalibrationReport:
    """Aggregated errors over a set of samples."""

    errors: list[PoseError]

    @property
    def max_position_error_mm(self) -> float:
        return max((e.position_error_mm for e in self.errors), default=0.0)

    @property
    def mean_position_error_mm(self) -> float:
        return _mean(e.position_error_mm for e in self.errors)

    @property
    def max_r_error_deg(self) -> float:
        return max((e.abs_r_error_deg for e in self.errors), default=0.0)

    @property
    def mean_r_error_deg(self) -> float:
        return _mean(e.abs_r_error_deg for e in self.errors)

    def format(self) -> str:
        """A human-readable per-sample table plus the summary line."""
        lines = [
            f"{'label':10s}{'dx':>9}{'dy':>9}{'dz':>9}{'dr':>9}{'|pos|':>9}",
            "-" * 64,
        ]
        for e in self.errors:
            lines.append(
                f"{e.label:10s}{e.dx:9.3f}{e.dy:9.3f}{e.dz:9.3f}{e.dr:9.3f}"
                f"{e.position_error_mm:9.3f}"
            )
        lines.append("-" * 64)
        lines.append(
            f"position error  max={self.max_position_error_mm:.3f} mm  "
            f"mean={self.mean_position_error_mm:.3f} mm"
        )
        lines.append(
            f"r (yaw) error   max={self.max_r_error_deg:.3f} deg  "
            f"mean={self.mean_r_error_deg:.3f} deg"
        )
        return "\n".join(lines)


def evaluate(
    samples: Iterable[CalibrationSample],
    *,
    config: "KinematicsConfig | None" = None,
) -> CalibrationReport:
    """Compute FK for each sample and report its error against the measurement."""
    errors: list[PoseError] = []
    for sample in samples:
        px, py, pz, pr = forward_kinematics(*sample.joints, config=config)
        mx, my, mz, mr = sample.measured_pose
        errors.append(
            PoseError(
                label=sample.label,
                dx=px - mx,
                dy=py - my,
                dz=pz - mz,
                dr=pr - mr,
            )
        )
    return CalibrationReport(errors)


def fit_config(
    samples: "Sequence[CalibrationSample]",
    *,
    r_j1_coeff: float = 1.0,
    r_j4_coeff: float = 1.0,
    r_offset_deg: float = 0.0,
    r_wrap: bool = False,
) -> KinematicsConfig:
    """Least-squares fit the link parameters (L1, L2, base_r, base_z) from pairs.

    This is how ``config/kinematics.json``'s link values were produced; calling it
    on the bundled pairs reproduces them, so the JSON is not a magic constant. The
    fit is *linear* in the forward model::

        rho = L1*sin(theta2) + L2*cos(theta3) + base_r
        z   = L1*cos(theta2) - L2*sin(theta3) + base_z

    so it is solved directly (no iteration). The r-axis convention (r = J1 + J4)
    is not a position fit and is carried through from the arguments. numpy is
    imported here only — the rest of the kinematics layer stays numpy-free.
    """
    import numpy as np

    rows: list[list[float]] = []
    rhs: list[float] = []
    for s in samples:
        _, j2, j3, _ = s.joints
        x, y, z, _ = s.measured_pose
        rho = math.hypot(x, y)
        t2, t3 = math.radians(j2), math.radians(j3)
        # Unknowns ordered [L1, L2, base_r, base_z].
        rows.append([math.sin(t2), math.cos(t3), 1.0, 0.0]); rhs.append(rho)
        rows.append([math.cos(t2), -math.sin(t3), 0.0, 1.0]); rhs.append(z)
    solution, *_ = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)
    l1, l2, base_r, base_z = (float(v) for v in solution)
    return KinematicsConfig(
        l1_rear_arm_mm=l1,
        l2_forearm_mm=l2,
        base_r_mm=base_r,
        base_z_mm=base_z,
        r_j1_coeff=r_j1_coeff,
        r_j4_coeff=r_j4_coeff,
        r_offset_deg=r_offset_deg,
        r_wrap=r_wrap,
    )


def load_calibration_pairs(
    path: "str | os.PathLike[str] | None" = None,
) -> list[CalibrationSample]:
    """Load measured calibration pairs from JSON (defaults to the bundled set)."""
    pairs_path = Path(path) if path is not None else DEFAULT_PAIRS_PATH
    with open(pairs_path, encoding="utf-8") as fh:
        raw = json.load(fh)
    samples: list[CalibrationSample] = []
    for entry in raw["pairs"]:
        joints = tuple(float(v) for v in entry["joints"])
        pose = tuple(float(v) for v in entry["pose"])
        samples.append(
            CalibrationSample(joints=joints, measured_pose=pose, label=entry.get("label", ""))
        )
    return samples


def _mean(values: Iterable[float]) -> float:
    seq: Sequence[float] = list(values)
    return sum(seq) / len(seq) if seq else 0.0
