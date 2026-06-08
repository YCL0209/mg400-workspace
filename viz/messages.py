"""JSON message schemas pushed to the inspection UI.

Defined as TypedDicts so the static type matches the over-the-wire JSON 1:1.
PHASE2_COORDINATE_INTERFACE_DESIGN.md §5 is the contract; this file is the
machine-checkable copy.

M1 only emits ``WorkspaceMessage``. ``StateMessage`` is defined here so M2 can
fill it in without revisiting the schema's shape.
"""

from __future__ import annotations

from typing import TypedDict


class WorkspaceMessage(TypedDict):
    """Static workspace geometry — pushed once on ws connect."""

    type: str  # always "workspace"
    annulus_inner_mm: float
    annulus_outer_mm: float
    z_min_mm: float
    z_max_mm: float
    j1_range_deg: list  # [min, max]
    j1_rear_dead_zone_deg: float
    origin: list  # [x, y] = [0, 0]
    grid_step_mm: float


class PoseDict(TypedDict):
    x: float
    y: float
    z: float
    r: float


class StateMessage(TypedDict, total=False):
    """Per-frame state — M2 wires this up. M1 never sends it."""

    type: str  # always "state"
    pose: PoseDict
    joints: list  # [j1, j2, j3, j4]
    fov_polygon: list  # [[x1,y1], ...] — empty until M2 has K + hand-eye
    flags: dict  # {"enabled": bool, "error": bool}
    detections: list  # M3 fills this from phase5-panel
