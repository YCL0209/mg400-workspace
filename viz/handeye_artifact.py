"""Write the hand-eye calibration artifact JSON.

The artifact (``config/hand_eye.json``) is the durable handoff from the
M0c solver to M2 FOV reprojection (camera ray → base plane intersection)
and M3 AOI result lift (detection in image → detection in base frame).
Schema is defined in PHASE2 design §8.2.5 and re-validated by
``tests/test_handeye_artifact.py``.

Overwrites in place; operators who want history copy the file aside
before re-solving (same convention as ``viz/calib_artifact.py``).

Critical: writes use ``allow_nan=False`` per PROGRESS finding 27 -- a
NaN slipping into ``rms_residual_mm`` from a degenerate solve would emit
bare ``NaN`` JSON that breaks the browser-side reader. With this flag,
``json.dumps`` raises instead, surfacing the bug at write time.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np

_DEFAULT_CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
_DEFAULT_PATH = _DEFAULT_CONFIG_DIR / "hand_eye.json"
_TOOL_VERSION = "M0c-v1"

# Anchor for the eye-in-hand transform: ``T_tcp←cam`` -- camera frame
# expressed in TCP. Matches PHASE2 §8.2 design choice (use TCP because
# 30004 .pose directly gives TCP, no FK round-trip). If we ever switch
# to flange anchoring, bump this string and re-solve everything that
# depends on it.
_FRAME_ANCHOR = "tcp"


def write_artifact(
    *,
    R,
    t_mm,
    rms_residual_mm: float,
    n_samples: int,
    method: str,
    intrinsics_file: str,
    intrinsics_rms_px: Optional[float],
    camera_serial: Optional[str],
    target_path: Optional[Path] = None,
) -> Path:
    """Write hand-eye result to ``config/hand_eye.json`` (or override path).

    ``R`` is a 3x3 array-like, ``t_mm`` a 3-vec array-like already in
    millimetres (caller converts from cv2's metre output). Both get
    coerced to plain nested / flat lists so the JSON is operator-readable.

    Returns the path written. Caller (M0c solver) logs this so the
    operator sees where the file landed.
    """
    if target_path is None:
        target_path = _DEFAULT_PATH

    R_list = np.asarray(R, dtype=float).tolist()
    t_list = np.asarray(t_mm, dtype=float).flatten().tolist()
    if len(R_list) != 3 or any(len(row) != 3 for row in R_list):
        raise ValueError(f"R must be 3x3, got shape {np.asarray(R).shape}")
    if len(t_list) != 3:
        raise ValueError(f"t_mm must be 3-vec, got len {len(t_list)}")

    payload = {
        "T_tcp_cam": {
            "R": R_list,
            "t": t_list,
        },
        "frame": _FRAME_ANCHOR,
        "method": method,
        "n_samples": int(n_samples),
        "rms_residual_mm": float(rms_residual_mm),
        "intrinsics_file": intrinsics_file,
        "intrinsics_rms_px": (
            float(intrinsics_rms_px) if intrinsics_rms_px is not None else None
        ),
        "camera_serial": camera_serial,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "tool_version": _TOOL_VERSION,
    }

    target_path.parent.mkdir(parents=True, exist_ok=True)
    # allow_nan=False: finding 27. A NaN in any numeric field (typical
    # culprit: rms_residual_mm when the solver degenerates) would emit
    # bare 'NaN' which is invalid JSON per RFC 7159. Better to raise
    # than to write a file that browsers + json5/python strict can't
    # read back.
    target_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )
    return target_path


__all__ = ["write_artifact"]
