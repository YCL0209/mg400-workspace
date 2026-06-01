"""Test the calibrate_bounds offline fitting logic."""

import json
import tempfile
import unittest
from pathlib import Path

from robot_core.safety.bounds import CouplingConstraint
from robot_core.safety.calibrate_bounds import (
    CalibrationResult,
    calibrate_from_file,
    compute_workspace_limits,
    derive_j1_dead_zone,
    derive_joint_ranges,
    detect_z_floor,
    filter_masquerading_points,
    fit_j2_j3_coupling,
    fit_piecewise_envelope,
    load_limit_points,
    select_coupling_points,
)


class TestJointRangeDerivation(unittest.TestCase):
    """Test deriving joint ranges from limit observations."""

    def test_derive_ranges_from_points(self):
        """Joint ranges correctly extracted from observations."""
        points = [
            {"j1": -160.0, "j2": 0, "j3": 0, "j4": -180.0, "label": "j1-min"},
            {"j1": 160.0, "j2": 0, "j3": 0, "j4": 0, "label": "j1-max"},
            {"j1": 0, "j2": -25.0, "j3": 0, "j4": 0, "label": "j2-min"},
            {"j1": 0, "j2": 85.0, "j3": 60.0, "j4": 0, "label": "j2-max"},
            {"j1": 0, "j2": 0, "j3": -25.0, "j4": 0, "label": "j3-min"},
            {"j1": 0, "j2": 0, "j3": 105.0, "j4": 0, "label": "j3-max"},
            {"j1": 0, "j2": 0, "j3": 0, "j4": 180.0, "label": "j4-max"},
        ]

        ranges = derive_joint_ranges(points)

        self.assertEqual(ranges["J1"], (-160.0, 160.0))
        self.assertEqual(ranges["J2"], (-25.0, 85.0))
        self.assertEqual(ranges["J3"], (-25.0, 105.0))
        self.assertEqual(ranges["J4"], (-180.0, 180.0))

    def test_empty_points_returns_defaults(self):
        """Empty point list returns default ranges."""
        ranges = derive_joint_ranges([])

        # Returns tuples as per the type hint
        self.assertEqual(ranges["J1"], (-160.0, 160.0))
        self.assertEqual(ranges["J2"], (-25.0, 85.0))
        self.assertEqual(ranges["J3"], (-25.0, 105.0))
        self.assertEqual(ranges["J4"], (-180.0, 180.0))

    def test_spec_fallback_for_under_probed_side(self):
        """Side not probed close to spec → fall back to spec (don't encode random session samples)."""
        # J4 wandered in [-106, +159] during a J2/J3 coupling probe — neither side
        # was pushed to spec (±180). Both sides must fall back to spec.
        points = [
            {"j1": 0, "j2": 0, "j3": 60, "j4": -106.0, "label": "p1"},
            {"j1": 0, "j2": 0, "j3": 60, "j4":  159.0, "label": "p2"},
        ]
        ranges = derive_joint_ranges(points)
        self.assertEqual(ranges["J4"], (-180.0, 180.0))

    def test_mixed_probed_one_side_only(self):
        """One side at spec, the other under-probed → use observed for the probed side, spec for the other."""
        # J2 upper pushed to 82.8 (close to spec 85), lower observed only -14.7 (far from -25).
        points = [
            {"j1": 0, "j2": -14.7, "j3": 50, "j4": 0, "label": "coup_low"},
            {"j1": 0, "j2":  82.8, "j3": 60, "j4": 0, "label": "j2_high"},
        ]
        ranges = derive_joint_ranges(points)
        self.assertEqual(ranges["J2"], (-25.0, 82.8))  # spec low, observed high

    def test_observed_beyond_spec_is_trusted(self):
        """Real arm exceeding spec on a side → trust observed (real measurement wins)."""
        points = [{"j1": 165.0, "j2": 0, "j3": 0, "j4": 0, "label": "j1_beyond_spec"}]
        ranges = derive_joint_ranges(points)
        # observed_high 165 > spec 160 → use observed
        self.assertEqual(ranges["J1"][1], 165.0)


class TestJ2J3CouplingFit(unittest.TestCase):
    """Test fitting J2/J3 coupling constraints."""

    def test_fit_coupling_from_boundary_points(self):
        """Coupling constraints fitted from J2/J3 boundary observations."""
        points = [
            {"j1": 0, "j2": 0, "j3": 105.0, "j4": 0, "label": "j3-max-at-j2-0"},
            {"j1": 0, "j2": 40, "j3": 85.0, "j4": 0, "label": "j2-j3-coupled"},
            {"j1": 0, "j2": 85.0, "j3": 60.0, "j4": 0, "label": "j2-max-j3-limited"},
        ]

        constraints = fit_j2_j3_coupling(points)

        # Should find at least one constraint showing J3 decreases with J2
        if constraints:
            # Verify it's a reasonable half-plane
            c = constraints[0]
            # At J2=0, J3 should be allowed up to ~105
            j3_at_j2_0 = c.max_value / c.j3_coeff if c.j3_coeff != 0 else float('inf')
            # At J2=85, J3 should be limited to ~60
            if c.j3_coeff != 0:
                j3_at_j2_85 = (c.max_value - c.j2_coeff * 85) / c.j3_coeff
                # J3 should decrease as J2 increases
                self.assertGreater(j3_at_j2_0, j3_at_j2_85)

    def test_no_coupling_if_insufficient_points(self):
        """No coupling constraints if not enough boundary points."""
        points = [
            {"j1": 0, "j2": 0, "j3": 50, "j4": 0, "label": "single-point"},
        ]

        constraints = fit_j2_j3_coupling(points)
        self.assertEqual(len(constraints), 0)


class TestCouplingHelpers(unittest.TestCase):
    """Unit tests for the T7A coupling-fit helpers (C2 lands them dormant,
    C3 wires them into fit_j2_j3_coupling)."""

    def test_select_only_picks_coup_labeled(self):
        points = [
            {"j1": 0, "j2": 0, "j3": 60, "j4": 0, "label": "coup_j2_0"},
            {"j1": 0, "j2": 14, "j3": 55, "j4": 0, "label": "COUPLING_j2_15"},  # case-insensitive
            {"j1": 0, "j2": 0, "j3": 60, "j4": 0, "label": "outer_front"},
            {"j1": 0, "j2": 0, "j3": 60, "j4": 0, "label": "floor_0"},
            {"j1": 0, "j2": 0, "j3": 60, "j4": 0},  # no label key
        ]
        picked = select_coupling_points(points)
        self.assertEqual(len(picked), 2)
        self.assertEqual({p["label"] for p in picked}, {"coup_j2_0", "COUPLING_j2_15"})

    def test_detect_z_floor_prefers_floor_label(self):
        # A high-J2 floor-labelled point + a low-J2 non-floor point. The
        # floor-label takes precedence even though the other has a lower FK z.
        points = [
            {"j1": 0, "j2": 82.8, "j3": 77.3, "j4": 0, "label": "floor_0"},  # FK z ≈ -201
            {"j1": 0, "j2": -25.0, "j3": 105.0, "j4": 0, "label": "coup_low"},
        ]
        z = detect_z_floor(points)
        self.assertIsNotNone(z)
        self.assertAlmostEqual(z, -201.7, delta=2.0)  # FK of floor_0

    def test_detect_z_floor_falls_back_to_min(self):
        # No floor-labelled point → min FK z across all points.
        points = [
            {"j1": 0, "j2": 0, "j3": 60, "j4": 0, "label": "p1"},
            {"j1": 0, "j2": 82.8, "j3": 77.3, "j4": 0, "label": "p2"},  # lowest z
        ]
        z = detect_z_floor(points)
        self.assertIsNotNone(z)
        self.assertLess(z, -100)  # well below origin

    def test_detect_z_floor_empty(self):
        self.assertIsNone(detect_z_floor([]))

    def test_filter_drops_high_j2(self):
        coupling = [
            {"j1": 0, "j2": 14.1, "j3": 55.3, "j4": 0, "label": "coup_j2_14"},
            {"j1": 0, "j2": 63.9, "j3": 35.7, "j4": 0, "label": "coup_j2_64"},  # > cutoff 50
        ]
        kept = filter_masquerading_points(coupling, z_floor=None, j2_cutoff=50.0)
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0]["label"], "coup_j2_14")

    def test_filter_drops_near_floor(self):
        # Synthesize a coupling point whose FK z lands near a known z_floor.
        coupling = [
            {"j1": 0, "j2": -7.4, "j3": 50.4, "j4": 0, "label": "coup_clean"},   # FK z far above floor
            {"j1": 0, "j2": 82.0, "j3": 76.0, "j4": 0, "label": "coup_at_floor"}, # FK z ≈ floor (but |J2|>50 will catch first)
        ]
        # Disable j2_cutoff (very high) so the z-proximity filter is what does the work
        kept = filter_masquerading_points(
            coupling, z_floor=-200.0, j2_cutoff=180.0, z_proximity=30.0
        )
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0]["label"], "coup_clean")

    def test_filter_no_z_floor_skips_z_check(self):
        coupling = [{"j1": 0, "j2": 14.1, "j3": 55.3, "j4": 0, "label": "coup"}]
        kept = filter_masquerading_points(coupling, z_floor=None, j2_cutoff=50.0)
        self.assertEqual(len(kept), 1)

    def test_piecewise_fits_finding13_shape(self):
        # PROGRESS finding 13's clean 4 points (after dropping the contaminated J2=+63.9 one).
        coupling = [
            {"j1": 0, "j2": -14.7, "j3": 44.5, "j4": 0, "label": "coup"},
            {"j1": 0, "j2":  -7.4, "j3": 50.4, "j4": 0, "label": "coup"},
            {"j1": 0, "j2":  14.1, "j3": 55.3, "j4": 0, "label": "coup"},
            {"j1": 0, "j2":  29.3, "j3": 55.5, "j4": 0, "label": "coup"},
        ]
        constraints = fit_piecewise_envelope(coupling)
        # Expect two constraints: rising + flat
        self.assertEqual(len(constraints), 2)
        labels = {c.label for c in constraints}
        self.assertEqual(labels, {"coup_rising", "coup_flat"})

        # Flat: J3 <= ~55 (min of flat region 55.3/55.5 minus margin 3) ≈ 52.3
        flat = next(c for c in constraints if c.label == "coup_flat")
        self.assertEqual(flat.j2_coeff, 0.0)
        self.assertEqual(flat.j3_coeff, 1.0)
        self.assertAlmostEqual(flat.max_value, 52.3, delta=1.0)

        # Rising: a positive coefficient on J2 (negative j2_coeff in -a*J2+J3<=b form)
        rising = next(c for c in constraints if c.label == "coup_rising")
        self.assertLess(rising.j2_coeff, 0)  # because original a > 0 (J3 rises with J2)
        self.assertEqual(rising.j3_coeff, 1.0)

    def test_piecewise_returns_empty_when_too_few(self):
        coupling = [
            {"j1": 0, "j2": 0, "j3": 50, "j4": 0, "label": "coup"},
            {"j1": 0, "j2": 10, "j3": 55, "j4": 0, "label": "coup"},
        ]
        self.assertEqual(fit_piecewise_envelope(coupling), [])

    def test_fit_j2_j3_coupling_e2e_finding13(self):
        """End-to-end: PROGRESS finding 13's 5 coupling points + a floor point.

        The contaminated J2=+63.9 point must be excluded by the J2-cutoff filter,
        and the remaining 4 must yield a 2-segment envelope (rising + flat).
        """
        points = [
            # The 5 coupling-labelled points from the v1 capture
            {"j1": 0, "j2": -14.7, "j3": 44.5, "j4": -106, "label": "coup_j2_-10"},
            {"j1": 0, "j2":  -7.4, "j3": 50.4, "j4": -106, "label": "coup_j2_0"},
            {"j1": 0, "j2":  14.1, "j3": 55.3, "j4": -106, "label": "coup_j2_20"},
            {"j1": 0, "j2":  29.3, "j3": 55.5, "j4": -106, "label": "coup_j2_40"},
            {"j1": 0, "j2":  63.9, "j3": 35.7, "j4": -106, "label": "coup_j2_70"},  # masquerading
            # A separately-labelled floor point (so detect_z_floor anchors on it)
            {"j1": 0, "j2": 82.8, "j3": 77.3, "j4":  81, "label": "floor_0"},
        ]
        constraints = fit_j2_j3_coupling(points)
        # Must produce both rising and flat segments (NOT empty like v1)
        self.assertEqual(len(constraints), 2)
        labels = {c.label for c in constraints}
        self.assertEqual(labels, {"coup_rising", "coup_flat"})

        # Flat cap is min(55.3, 55.5) - 3 margin ≈ 52.3
        flat = next(c for c in constraints if c.label == "coup_flat")
        self.assertAlmostEqual(flat.max_value, 52.3, delta=1.0)

        # At J2=0, evaluating both segments — minimum is what gate enforces.
        # Rising at J2=0 ≈ b - margin. Flat at J2=0 = 52.3. The smaller wins.
        rising = next(c for c in constraints if c.label == "coup_rising")
        rising_at_j2_0 = -rising.j2_coeff * 0 + rising.max_value
        # Effective envelope at J2=0 = min of the two
        envelope_at_0 = min(rising_at_j2_0, flat.max_value)
        # Should be close to observed rising peak (~50) minus margin
        self.assertGreater(envelope_at_0, 40)
        self.assertLess(envelope_at_0, 60)


class TestWorkspaceLimits(unittest.TestCase):
    """Test workspace limit computation via FK."""

    def test_compute_workspace_from_points(self):
        """Workspace limits derived from FK of observed joints."""
        points = [
            {"j1": 0, "j2": 0, "j3": 0, "j4": 0},  # Near home
            {"j1": 0, "j2": 85, "j3": 0, "j4": 0},  # J2 extended
            {"j1": 0, "j2": 0, "j3": 105, "j4": 0}, # J3 extended
            {"j1": 90, "j2": 45, "j3": 45, "j4": 0}, # Side reach
        ]

        inner_r, outer_r, z_min, z_max = compute_workspace_limits(points)

        # Should get reasonable bounds
        self.assertGreater(inner_r, 50)   # Some minimum distance
        self.assertLess(inner_r, 200)     # Not too far
        self.assertGreater(outer_r, 300)  # Can reach out
        self.assertLess(outer_r, 450)     # Within max spec
        self.assertLess(z_min, 0)         # Can go below origin
        self.assertGreater(z_max, 100)    # Can reach up

    def test_workspace_with_safety_margins(self):
        """Safety margins properly applied to workspace."""
        points = [{"j1": 0, "j2": 0, "j3": 0, "j4": 0}]

        # Without margins
        inner1, outer1, z_min1, z_max1 = compute_workspace_limits(
            points, inner_margin=0, outer_margin=0, z_margin=0
        )

        # With margins
        inner2, outer2, z_min2, z_max2 = compute_workspace_limits(
            points, inner_margin=20, outer_margin=10, z_margin=5
        )

        # Inner increases with margin (pushes away from center)
        self.assertGreater(inner2, inner1)
        # Outer decreases with margin (pulls back from edge)
        self.assertLess(outer2, outer1)
        # Z limits shrink with margins
        self.assertGreater(z_min2, z_min1)
        self.assertLess(z_max2, z_max1)


class TestJ1DeadZone(unittest.TestCase):
    """Test J1 rear dead zone derivation."""

    def test_derive_dead_zone_from_rear_limits(self):
        """Dead zone computed from points near J1=±180°."""
        points = [
            {"j1": -160, "j2": 0, "j3": 0, "j4": 0, "label": "j1-min"},
            {"j1": 160, "j2": 0, "j3": 0, "j4": 0, "label": "j1-max"},
            {"j1": 165, "j2": 0, "j3": 0, "j4": 0, "label": "j1-rear-limit"},
        ]

        dead_zone = derive_j1_dead_zone(points)

        # 165° means 15° from 180°, so dead zone is 2*15 = 30°
        self.assertAlmostEqual(dead_zone, 30.0, places=1)

    def test_default_dead_zone_without_rear_points(self):
        """Default dead zone used when no rear observations."""
        points = [
            {"j1": 0, "j2": 0, "j3": 0, "j4": 0, "label": "home"},
            {"j1": 90, "j2": 0, "j3": 0, "j4": 0, "label": "side"},
        ]

        dead_zone = derive_j1_dead_zone(points)
        self.assertEqual(dead_zone, 40.0)  # Default value

    def test_dead_zone_no_artificial_cap(self):
        """Real arm with larger rear gap than spec → return the measured value, not capped at 40."""
        # J1 reached only ±150° → 2*(180-150) = 60° dead zone (well above the old 40° cap).
        points = [
            {"j1": -150.0, "j2": 0, "j3": 0, "j4": 0, "label": "j1_rear_left"},
            {"j1":  150.0, "j2": 0, "j3": 0, "j4": 0, "label": "j1_rear_right"},
        ]
        dead_zone = derive_j1_dead_zone(points)
        self.assertAlmostEqual(dead_zone, 60.0, places=1)


class TestCalibrationResult(unittest.TestCase):
    """Test the CalibrationResult data structure."""

    def test_to_safety_json_format(self):
        """CalibrationResult outputs correct config/safety.json format."""
        result = CalibrationResult(
            annulus_inner_mm=150.0,
            annulus_outer_mm=420.0,
            z_min_mm=-160.0,
            z_max_mm=180.0,
            j1_rear_dead_zone_deg=40.0,
            joint_ranges_deg={
                "J1": (-160.0, 160.0),
                "J2": (-25.0, 85.0),
                "J3": (-25.0, 105.0),
                "J4": (-180.0, 180.0),
            },
            coupling_constraints=[
                CouplingConstraint(
                    j2_coeff=0.5,
                    j3_coeff=1.0,
                    max_value=130.0,
                    label="upper_bound"
                )
            ],
            provenance="Test calibration",
            source_file="test.json",
            point_count=10,
        )

        output = result.to_safety_json()

        # Check structure
        self.assertIn("workspace", output)
        self.assertIn("joint_ranges_deg", output)
        self.assertIn("j2_j3_coupling", output)

        # Check values
        self.assertEqual(output["workspace"]["annulus_inner_radius_mm"], 150.0)
        self.assertEqual(output["workspace"]["annulus_outer_radius_mm"], 420.0)
        self.assertEqual(output["joint_ranges_deg"]["J2"], (-25.0, 85.0))

        # Check coupling
        self.assertEqual(len(output["j2_j3_coupling"]), 1)
        self.assertEqual(output["j2_j3_coupling"][0]["j2_coeff"], 0.5)
        self.assertEqual(output["j2_j3_coupling"][0]["label"], "upper_bound")


class TestEndToEndCalibration(unittest.TestCase):
    """Test the full calibration pipeline."""

    def test_calibrate_from_file(self):
        """Full calibration from a limits file."""
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create a test limits file
            limits_file = Path(tmpdir) / "limits_test.json"
            limits_data = {
                "captured_at": "2024-01-01T12:00:00",
                "count": 6,
                "note": "test limits",
                "points": [
                    {"j1": -160.0, "j2": 0, "j3": 0, "j4": 0, "label": "j1-min",
                     "robot_mode": 5, "error_status": 0, "has_error": False,
                     "captured_at": "2024-01-01T12:00:00", "seq": 1, "q_actual": [-160, 0, 0, 0, 0, 0]},
                    {"j1": 160.0, "j2": 0, "j3": 0, "j4": 0, "label": "j1-max",
                     "robot_mode": 5, "error_status": 0, "has_error": False,
                     "captured_at": "2024-01-01T12:00:01", "seq": 2, "q_actual": [160, 0, 0, 0, 0, 0]},
                    {"j1": 0, "j2": 85.0, "j3": 60.0, "j4": 0, "label": "j2-max-coupled",
                     "robot_mode": 5, "error_status": 0, "has_error": False,
                     "captured_at": "2024-01-01T12:00:02", "seq": 3, "q_actual": [0, 85, 60, 0, 0, 0]},
                    {"j1": 0, "j2": 0, "j3": 105.0, "j4": 0, "label": "j3-max",
                     "robot_mode": 5, "error_status": 0, "has_error": False,
                     "captured_at": "2024-01-01T12:00:03", "seq": 4, "q_actual": [0, 0, 105, 0, 0, 0]},
                    {"j1": 0, "j2": -25.0, "j3": -25.0, "j4": 0, "label": "j2-j3-min",
                     "robot_mode": 5, "error_status": 0, "has_error": False,
                     "captured_at": "2024-01-01T12:00:04", "seq": 5, "q_actual": [0, -25, -25, 0, 0, 0]},
                    {"j1": 165, "j2": 0, "j3": 0, "j4": 0, "label": "j1-rear-approach",
                     "robot_mode": 5, "error_status": 0, "has_error": False,
                     "captured_at": "2024-01-01T12:00:05", "seq": 6, "q_actual": [165, 0, 0, 0, 0, 0]},
                ],
            }
            limits_file.write_text(json.dumps(limits_data, indent=2))

            # Run calibration
            result = calibrate_from_file(limits_file, inner_margin=10, outer_margin=10, z_margin=5)

            # Verify result
            self.assertEqual(result.point_count, 6)
            self.assertEqual(result.source_file, "limits_test.json")
            
            # Check joint ranges derived (J1 max is 165 in the test data)
            self.assertEqual(result.joint_ranges_deg["J1"], (-160.0, 165.0))
            self.assertEqual(result.joint_ranges_deg["J2"], (-25.0, 85.0))
            self.assertEqual(result.joint_ranges_deg["J3"], (-25.0, 105.0))
            
            # Check workspace (rough check - exact values depend on FK)
            self.assertGreater(result.annulus_inner_mm, 50)
            self.assertLess(result.annulus_outer_mm, 450)
            
            # Check dead zone (165° -> 15° from 180° -> 30° dead zone)
            self.assertAlmostEqual(result.j1_rear_dead_zone_deg, 30.0, places=1)

            # Verify JSON output is valid
            json_output = result.to_safety_json()
            json_str = json.dumps(json_output)  # Should not raise
            self.assertIn("workspace", json_output)
            self.assertIn("provenance", json_output)


if __name__ == "__main__":
    unittest.main()