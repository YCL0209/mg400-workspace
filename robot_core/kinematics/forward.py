"""Forward kinematics for the MG400 — pure math, no hardware.

Maps joint angles ``(J1, J2, J3, J4)`` to the flange-centre Cartesian pose
``(x, y, z, r)``. Standard library only (``math``) — no numpy, no sockets.

Mechanism model (MG400 is a *parallelogram* 4-axis arm, NOT a serial 6-DOF DH
chain): J2 and J3 each set their link's ABSOLUTE angle and the linkage keeps the
flange vertical, so the planar reach decouples into two independent links.

    theta2 = J2  (deg, measured from vertical:   J2=0 -> rear arm vertical)
    theta3 = J3  (deg, measured from horizontal: J3=0 -> forearm horizontal)

    rho = base_r + L1*sin(theta2) + L2*cos(theta3)   # radial distance from J1 axis
    z   = base_z + L1*cos(theta2) - L2*sin(theta3)   # height (z<0 = below origin)
    x   = rho*cos(J1)                                 # J1 yaws the arm plane
    y   = rho*sin(J1)
    r   = j1_coeff*J1 + j4_coeff*J4 + offset          # absolute tool yaw

Frame: origin on the J1 rotation axis; +X forward; +Z up; angles in degrees.
``r`` is the absolute tool yaw and is NOT wrapped by default, so it can exceed
+/-180 (verified against real data). All parameters come from
``config/kinematics.json`` (back-fitted from real measurements); nothing here is
hard-coded geometry. Joint limits are NOT enforced — that is the safety layer's
job (Phase 2b).
"""

from __future__ import annotations

import math

from .config import KinematicsConfig, default_config

Pose = "tuple[float, float, float, float]"


def forward_kinematics(
    j1: float,
    j2: float,
    j3: float,
    j4: float,
    *,
    config: "KinematicsConfig | None" = None,
) -> "tuple[float, float, float, float]":
    """Compute the flange-centre pose ``(x, y, z, r)`` for joint angles (deg).

    Args:
        j1, j2, j3, j4: Joint angles in degrees.
        config: Mechanism parameters; defaults to :func:`default_config`.

    Returns:
        ``(x, y, z, r)`` — position in mm, ``r`` (tool yaw) in degrees.

    Pure: the result depends only on the inputs and ``config``. Joint limits are
    not checked here.
    """
    cfg = config or default_config()

    theta2 = math.radians(j2)
    theta3 = math.radians(j3)
    rho = cfg.base_r_mm + cfg.l1_rear_arm_mm * math.sin(theta2) + cfg.l2_forearm_mm * math.cos(theta3)
    z = cfg.base_z_mm + cfg.l1_rear_arm_mm * math.cos(theta2) - cfg.l2_forearm_mm * math.sin(theta3)

    j1_rad = math.radians(j1)
    x = rho * math.cos(j1_rad)
    y = rho * math.sin(j1_rad)

    r = cfg.r_j1_coeff * j1 + cfg.r_j4_coeff * j4 + cfg.r_offset_deg
    if cfg.r_wrap:
        r = (r + 180.0) % 360.0 - 180.0

    return (x, y, z, r)
