"""Camera adapter smoke tests — runnable on Mac dev without DmvSDK.

These do NOT exercise the real SDK (impossible without Windows + Delta
installer). They verify that ``import robot_core.camera`` succeeds, the
public surface exists, and ``.open()`` raises a clear error when the SDK
is absent so callers don't hit a surprise ``ImportError`` at the wrong
moment.
"""

import unittest

from robot_core.camera import HAS_DMV_SDK, DeltaCamera


class TestCameraImport(unittest.TestCase):
    """Import + public surface check."""

    def test_module_imports_without_dmv_sdk(self):
        """Mac dev must be able to import the camera module."""
        import robot_core.camera as camera

        self.assertTrue(hasattr(camera, "DeltaCamera"))
        self.assertTrue(hasattr(camera, "HAS_DMV_SDK"))

    def test_delta_camera_has_expected_methods(self):
        """All operator-facing methods present so static callers don't break."""
        for method in (
            "open",
            "close",
            "grab_one_rgb",
            "start_continuous",
            "grab_continuous_rgb",
            "stop_continuous",
            "__enter__",
            "__exit__",
        ):
            self.assertTrue(
                callable(getattr(DeltaCamera, method)),
                f"DeltaCamera missing method: {method}",
            )

    def test_construction_does_not_touch_sdk(self):
        """Object construction must be SDK-free so tests + lint work cross-platform."""
        cam = DeltaCamera()
        self.assertFalse(cam.is_open)
        # Field defaults — guards against silent rename of internal state.
        self.assertIsNone(cam.system)
        self.assertIsNone(cam.device)


@unittest.skipIf(HAS_DMV_SDK, "DmvSDK present — error-path test only meaningful when absent")
class TestCameraWithoutSdk(unittest.TestCase):
    """When DmvSDK isn't installed, ``open()`` should raise a clear error."""

    def test_open_raises_clear_runtime_error(self):
        cam = DeltaCamera()
        with self.assertRaises(RuntimeError) as ctx:
            cam.open()
        self.assertIn("DmvSDK", str(ctx.exception))
        self.assertIn("docs/dmv_sdk.md", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
