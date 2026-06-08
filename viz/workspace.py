"""Pure function: SafetyBounds → workspace JSON message.

Isolated from server / I/O so unit tests cover the schema contract cheaply.
"""

from __future__ import annotations

from robot_core.safety.bounds import SafetyBounds

from .messages import WorkspaceMessage


def build_workspace_message(
    bounds: SafetyBounds, *, grid_step_mm: float = 50.0
) -> WorkspaceMessage:
    """Pack the static workspace geometry per PHASE2 design §5(a).

    The frontend uses this once on connect to draw the reachable annulus,
    J1 rear dead-zone sector, and coordinate grid. Joint range only ships J1
    because that's the one the top-down view renders (J2/J3/J4 don't affect
    the planar reachable footprint at this milestone).
    """
    j1_min, j1_max = bounds.joint_ranges_deg["J1"]
    return WorkspaceMessage(
        type="workspace",
        annulus_inner_mm=bounds.annulus_inner_mm,
        annulus_outer_mm=bounds.annulus_outer_mm,
        z_min_mm=bounds.z_min_mm,
        z_max_mm=bounds.z_max_mm,
        j1_range_deg=[j1_min, j1_max],
        j1_rear_dead_zone_deg=bounds.j1_rear_dead_zone_deg,
        origin=[0.0, 0.0],
        grid_step_mm=float(grid_step_mm),
    )
