"""Verify build_workspace_message produces the PHASE2 §5(a) schema from SafetyBounds."""

import unittest

from robot_core.safety.bounds import CouplingConstraint, SafetyBounds
from viz.workspace import build_workspace_message


def _sample_bounds() -> SafetyBounds:
    """Build a SafetyBounds with the actual Phase 2b v2 values (config/safety.json)."""
    return SafetyBounds(
        annulus_inner_mm=123.83,
        annulus_outer_mm=440.0,
        z_min_mm=-197.05,
        z_max_mm=116.21,
        j1_rear_dead_zone_deg=40.21,
        joint_ranges_deg={
            "J1": (-159.9, 157.0),
            "J2": (-25.0, 85.0),
            "J3": (-25.0, 105.0),
            "J4": (-180.0, 180.0),
        },
        coupling=(
            CouplingConstraint(
                j2_coeff=-1.0,
                j3_coeff=1.0,
                max_value=59.95,
                label="j3_minus_j2_le_60",
            ),
        ),
    )


class TestBuildWorkspaceMessage(unittest.TestCase):
    """Pure function: SafetyBounds → workspace JSON dict."""

    def test_packs_all_required_fields_with_correct_types(self):
        msg = build_workspace_message(_sample_bounds())

        # Type tag is the dispatcher key on the frontend.
        self.assertEqual(msg["type"], "workspace")

        # Annulus + z geometry passed through 1:1.
        self.assertAlmostEqual(msg["annulus_inner_mm"], 123.83)
        self.assertAlmostEqual(msg["annulus_outer_mm"], 440.0)
        self.assertAlmostEqual(msg["z_min_mm"], -197.05)
        self.assertAlmostEqual(msg["z_max_mm"], 116.21)

        # J1 range is the only joint shipped (others don't affect top-down geom).
        self.assertEqual(msg["j1_range_deg"], [-159.9, 157.0])
        self.assertAlmostEqual(msg["j1_rear_dead_zone_deg"], 40.21)

        self.assertEqual(msg["origin"], [0.0, 0.0])
        self.assertEqual(msg["grid_step_mm"], 50.0)

    def test_grid_step_override(self):
        msg = build_workspace_message(_sample_bounds(), grid_step_mm=25.0)
        self.assertEqual(msg["grid_step_mm"], 25.0)

    def test_message_is_json_serialisable(self):
        """No dataclass / tuple fields leak — frontend wire format must be plain JSON."""
        import json

        msg = build_workspace_message(_sample_bounds())
        roundtripped = json.loads(json.dumps(msg))
        self.assertEqual(roundtripped, msg)


if __name__ == "__main__":
    unittest.main()
