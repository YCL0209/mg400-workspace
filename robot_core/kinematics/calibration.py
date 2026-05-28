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
