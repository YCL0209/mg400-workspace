"""Offline unit tests for the MG400 forward kinematics + calibration interface.

Pure math (stdlib only, no numpy, no hardware). The 10 real measured pairs in
config/calibration_pairs.json are the acceptance set: FK must reproduce them
within tolerance, which is what validates the parallelogram model + parameters.
"""

import unittest

from robot_core.kinematics import (
    CalibrationSample,
    default_config,
    evaluate,
    fit_config,
    forward_kinematics,
    load_calibration_pairs,
)

try:
    import numpy  # noqa: F401

    HAVE_NUMPY = True
except ImportError:  # pragma: no cover - depends on environment
    HAVE_NUMPY = False

# Real fit residual is ~0.003 mm / 0.001 deg; these are deliberately looser
# thresholds that still meaningfully fail if the model/params drift.
POSITION_TOLERANCE_MM = 1.0
R_TOLERANCE_DEG = 0.5


class CalibrationSetTests(unittest.TestCase):
    def setUp(self):
        self.report = evaluate(load_calibration_pairs())

    def test_all_pairs_within_tolerance(self):
        self.assertEqual(len(self.report.errors), 10)
        self.assertLess(self.report.max_position_error_mm, POSITION_TOLERANCE_MM)
        self.assertLess(self.report.max_r_error_deg, R_TOLERANCE_DEG)

    def test_mean_error_is_small(self):
        self.assertLess(self.report.mean_position_error_mm, POSITION_TOLERANCE_MM)


class FactoryPointTests(unittest.TestCase):
    def test_factory_point_pose(self):
        # J=(0,0,60,0) should land near the documented factory pose.
        x, y, z, r = forward_kinematics(0.0, 0.0, 60.0, 0.0)
        self.assertAlmostEqual(x, 197.2, delta=1.0)
        self.assertAlmostEqual(y, 0.0, delta=1.0)
        self.assertAlmostEqual(z, -30.3, delta=1.0)
        self.assertAlmostEqual(r, 0.0, delta=0.01)


class RAxisTests(unittest.TestCase):
    def test_r_is_j1_plus_j4(self):
        _, _, _, r = forward_kinematics(30.0, 0.0, 60.0, 15.0)
        self.assertAlmostEqual(r, 45.0, delta=1e-9)

    def test_r_does_not_wrap_past_180(self):
        # Real pair p9: J1=87.31, J4=159.76 -> r ~= 247, must NOT wrap to ~-113.
        _, _, _, r = forward_kinematics(87.310, -1.396, 38.689, 159.760)
        self.assertGreater(r, 180.0)
        self.assertAlmostEqual(r, 247.07, delta=R_TOLERANCE_DEG)


class SymmetryTests(unittest.TestCase):
    def test_j1_zero_gives_y_near_zero(self):
        # +X forward, left/right symmetric: J1=0 -> y ~ 0, x = rho > 0.
        x, y, _, _ = forward_kinematics(0.0, 10.0, 30.0, 0.0)
        self.assertAlmostEqual(y, 0.0, delta=1e-9)
        self.assertGreater(x, 0.0)

    def test_j1_rotation_preserves_radius(self):
        # Rotating J1 only must keep sqrt(x^2+y^2) (the planar reach) constant.
        import math

        x0, y0, _, _ = forward_kinematics(0.0, 20.0, 40.0, 0.0)
        x1, y1, _, _ = forward_kinematics(73.0, 20.0, 40.0, 0.0)
        self.assertAlmostEqual(math.hypot(x0, y0), math.hypot(x1, y1), delta=1e-6)


class VerificationInterfaceTests(unittest.TestCase):
    def test_correct_pair_reports_near_zero_error(self):
        sample = CalibrationSample(
            joints=(0.0, 0.0, 60.0, 0.0),
            measured_pose=forward_kinematics(0.0, 0.0, 60.0, 0.0),
            label="self",
        )
        report = evaluate([sample])
        self.assertLess(report.max_position_error_mm, 1e-6)

    def test_deliberately_wrong_pair_reports_the_error(self):
        # Measured pose is off by +10 mm in x and +5 deg in r; the interface
        # must surface those magnitudes rather than hide them.
        x, y, z, r = forward_kinematics(0.0, 0.0, 60.0, 0.0)
        wrong = CalibrationSample(
            joints=(0.0, 0.0, 60.0, 0.0),
            measured_pose=(x - 10.0, y, z, r - 5.0),
            label="wrong",
        )
        report = evaluate([wrong])
        self.assertAlmostEqual(report.errors[0].dx, 10.0, delta=1e-6)
        self.assertAlmostEqual(report.max_position_error_mm, 10.0, delta=1e-6)
        self.assertAlmostEqual(report.max_r_error_deg, 5.0, delta=1e-6)


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class FitConfigTests(unittest.TestCase):
    def test_fit_reproduces_shipped_link_parameters(self):
        # Re-fitting from the bundled pairs must reproduce config/kinematics.json's
        # link values — i.e. the shipped params are not a magic constant.
        fitted = fit_config(load_calibration_pairs())
        shipped = default_config()
        self.assertAlmostEqual(fitted.l1_rear_arm_mm, shipped.l1_rear_arm_mm, delta=0.05)
        self.assertAlmostEqual(fitted.l2_forearm_mm, shipped.l2_forearm_mm, delta=0.05)
        self.assertAlmostEqual(fitted.base_r_mm, shipped.base_r_mm, delta=0.05)
        self.assertAlmostEqual(fitted.base_z_mm, shipped.base_z_mm, delta=0.05)

    def test_fitted_config_has_tiny_residual(self):
        fitted = fit_config(load_calibration_pairs())
        report = evaluate(load_calibration_pairs(), config=fitted)
        self.assertLess(report.max_position_error_mm, 0.05)


if __name__ == "__main__":
    unittest.main()
