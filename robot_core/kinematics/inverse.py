"""Inverse kinematics for the MG400 — analytic, pure math, no hardware.

Maps a flange-centre Cartesian pose ``(x, y, z, r)`` back to the joint angles
``(J1, J2, J3, J4)``. This is the exact closed-form inverse of
:func:`~robot_core.kinematics.forward.forward_kinematics` (same parallelogram
model, same ``KinematicsConfig``: L1/L2/base_r/base_z and ``r = J1 + J4``). No
numerical solver, no iteration.

Derivation (forward model: ``u = L1·sinθ2 + L2·cosθ3``, ``v = L1·cosθ2 − L2·sinθ3``
where ``u = ρ − base_r``, ``v = z − base_z``)::

    J1 = atan2(y, x);  ρ = hypot(x, y)
    eliminate θ2:  u·cosθ3 − v·sinθ3 = (u² + v² + L2² − L1²) / (2·L2) = K
                   = R·cos(θ3 + atan2(v, u)),  R = hypot(u, v)
    => θ3 = −atan2(v, u) ± acos(K/R)               (two branches)
       θ2 = atan2(u − L2·cosθ3,  v + L2·sinθ3)
       J4 = r − J1

Returns 0, 1, or 2 geometric solutions (deg). Unreachable (no real solution,
``|K/R| > 1``) returns ``[]``; a boundary pose (``|K/R| ≈ 1``, the two branches
coincide) returns a single solution.

This layer is intentionally *only* geometry: it does NOT filter joint ranges or
the J2/J3 coupling limit (that is the safety layer's job), and it does NOT pick a
"best" solution — it returns every geometric branch and lets the caller choose.
"""

from __future__ import annotations

import math

from .config import KinematicsConfig, default_config

# Two branches whose acos terms differ by less than this (rad) are treated as the
# same solution (boundary / fully-extended pose) and returned once.
_BRANCH_EPS = 1e-9


def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    r: float,
    *,
    config: "KinematicsConfig | None" = None,
) -> "list[tuple[float, float, float, float]]":
    """Return the geometric joint solutions ``(J1, J2, J3, J4)`` (deg) for a pose.

    Args:
        x, y, z: Flange-centre position (mm). r: tool yaw (deg).
        config: Mechanism parameters; defaults to :func:`default_config`.

    Returns:
        A list of 0–2 solutions. ``[]`` if the pose is unreachable. Joint ranges
        and coupling limits are NOT applied here.
    """
    cfg = config or default_config()
    l1 = cfg.l1_rear_arm_mm
    l2 = cfg.l2_forearm_mm

    j1 = math.degrees(math.atan2(y, x))
    rho = math.hypot(x, y)

    u = rho - cfg.base_r_mm
    v = z - cfg.base_z_mm
    radius = math.hypot(u, v)
    if radius == 0.0:
        return []  # degenerate: flange at the base reference is unreachable.

    k = (u * u + v * v + l2 * l2 - l1 * l1) / (2.0 * l2)
    ratio = k / radius
    if abs(ratio) > 1.0:
        return []  # out of the two-link reach -> no real solution.

    delta = math.acos(max(-1.0, min(1.0, ratio)))  # clamp for float safety.
    base_angle = math.atan2(v, u)
    # r = j1_coeff*J1 + j4_coeff*J4 + offset  ->  solve for J4.
    j4 = (r - cfg.r_offset_deg - cfg.r_j1_coeff * j1) / cfg.r_j4_coeff

    branches = (delta,) if delta <= _BRANCH_EPS else (delta, -delta)
    solutions: list[tuple[float, float, float, float]] = []
    for signed in branches:
        theta3 = -base_angle + signed
        theta2 = math.atan2(u - l2 * math.cos(theta3), v + l2 * math.sin(theta3))
        solutions.append((j1, _wrap_deg(math.degrees(theta2)), _wrap_deg(math.degrees(theta3)), j4))
    return solutions


def _wrap_deg(angle: float) -> float:
    """Normalise an angle to (−180, 180] so distinct revolutions don't masquerade
    as different solutions (FK is periodic; this keeps J2/J3 in a canonical range)."""
    wrapped = (angle + 180.0) % 360.0 - 180.0
    # map the -180 endpoint to +180 so the interval is (−180, 180].
    return 180.0 if wrapped == -180.0 else wrapped
