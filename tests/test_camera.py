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

    def test_construction_with_serial_stores_request(self):
        """serial= kwarg recorded for later open() lookup."""
        cam = DeltaCamera(serial="ABC123")
        self.assertEqual(cam._requested_serial, "ABC123")
        self.assertIsNone(cam._requested_index)

    def test_construction_with_device_index_stores_request(self):
        cam = DeltaCamera(device_index=1)
        self.assertIsNone(cam._requested_serial)
        self.assertEqual(cam._requested_index, 1)

    def test_construction_rejects_both_serial_and_index(self):
        """API contract: pick one selection method, not both."""
        with self.assertRaises(ValueError) as ctx:
            DeltaCamera(serial="X", device_index=0)
        self.assertIn("serial OR device_index", str(ctx.exception))

    def test_list_devices_is_classmethod(self):
        """list_devices() callable on the class itself."""
        self.assertTrue(callable(DeltaCamera.list_devices))


@unittest.skipIf(HAS_DMV_SDK, "DmvSDK present — error-path test only meaningful when absent")
class TestCameraWithoutSdk(unittest.TestCase):
    """When DmvSDK isn't installed, ``open()`` and ``list_devices()`` raise clear errors."""

    def test_open_raises_clear_runtime_error(self):
        cam = DeltaCamera()
        with self.assertRaises(RuntimeError) as ctx:
            cam.open()
        self.assertIn("DmvSDK", str(ctx.exception))
        self.assertIn("docs/dmv_sdk.md", str(ctx.exception))

    def test_list_devices_raises_clear_runtime_error(self):
        with self.assertRaises(RuntimeError) as ctx:
            DeltaCamera.list_devices()
        self.assertIn("DmvSDK", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
