"""Kinematics layer: pure-math joint<->Cartesian mapping for the MG400.

Standalone and parallel to transport/state — depends on neither, touches no
socket or hardware. Phase 2a provides forward kinematics + a calibration
(verification) interface. All mechanism parameters come from
``config/kinematics.json``.
"""

from .calibration import (
    CalibrationReport,
    CalibrationSample,
    PoseError,
    evaluate,
    load_calibration_pairs,
)
from .config import KinematicsConfig, default_config
from .forward import forward_kinematics

__all__ = [
    "forward_kinematics",
    "KinematicsConfig",
    "default_config",
    "CalibrationSample",
    "PoseError",
    "CalibrationReport",
    "evaluate",
    "load_calibration_pairs",
]
