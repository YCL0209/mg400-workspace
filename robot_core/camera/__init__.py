"""Delta industrial camera adapter — read-only consumer of DmvSDK.

Forked from ``phase5-panel/camera.py`` so this repo has its own copy (CLAUDE.md
rule: data and binary protocols may be migrated from reference projects;
program code is rewritten or — as here — vendored with attribution). Converted
``print`` calls to :mod:`logging` per the layering rule (library code uses
logging; only ``scripts/`` CLI talks to the operator with ``print``).

The native ``DmvSDK`` module is **Windows-only** and ships with Delta's DMV
installer; on Mac dev machines it is absent. We import it lazily under a
``try``/``except`` so ``import robot_core.camera`` never crashes — callers who
actually need hardware get a clear :class:`RuntimeError` from
:meth:`DeltaCamera.open`, gated by :data:`HAS_DMV_SDK`. See ``docs/dmv_sdk.md``
for the full 7-step acquisition flow this class wraps.
"""

from __future__ import annotations

import logging

import numpy as np

logger = logging.getLogger("robot_core.camera")

try:
    import DmvSDK  # type: ignore[import-not-found]

    HAS_DMV_SDK = True
except ImportError:
    DmvSDK = None  # type: ignore[assignment]
    HAS_DMV_SDK = False


_DMV_SDK_MISSING_MSG = (
    "DmvSDK not importable. The Delta DMV machine vision SDK ships with the "
    "DMV Windows installer; this code path only runs on a Win machine with "
    "DMV installed and the DmvSDK module on PYTHONPATH. See docs/dmv_sdk.md."
)


class DeltaCamera:
    """Delta industrial camera wrapper around DmvSDK's 7-step acquisition flow.

    Returns RGB ``(H, W, 3) uint8`` ndarrays from both single and continuous
    modes regardless of the native pixel format (Mono8 frames are broadcast to
    three channels for downstream consistency).

    Single mode raises ``RuntimeError`` on failure; continuous mode returns
    ``None`` so the caller's frame loop survives transient drops — a
    deliberate split (see docs/dmv_sdk.md §6).

    Multi-camera selection: pass ``serial=...`` to pick a specific device when
    multiple cameras share a hub (same model is hard to distinguish otherwise).
    Pass ``device_index=...`` as a debug fallback if serial reading is broken.
    With neither, opens the first enumerated device and logs a warning if more
    than one is present. Use :meth:`list_devices` to discover serials.
    """

    def __init__(
        self,
        *,
        serial: str | None = None,
        device_index: int | None = None,
    ) -> None:
        if serial is not None and device_index is not None:
            raise ValueError("specify serial OR device_index, not both")
        self._requested_serial = serial
        self._requested_index = device_index
        self.system = None
        self.device = None
        self.data_stream = None
        self.buffer = None
        self.is_open = False

    @classmethod
    def list_devices(cls) -> list[dict]:
        """Enumerate all connected DMV cameras.

        Returns a list of ``{index, display_name, serial, user_id, model}``
        dicts so the operator can pick the right one by serial when several
        same-model cameras share a hub.

        SDK API names are best-guesses from GenICam conventions (the public
        docs only show ``DcSystemGetDevice(system, None)``). If the installed
        DMV exposes different names, the per-field lookups fall back to
        ``"<unavailable: ...>"`` rather than crashing — fix the names + log
        a finding once the real SDK surface is observed.
        """
        if not HAS_DMV_SDK:
            raise RuntimeError(_DMV_SDK_MISSING_MSG)

        system = DmvSDK.DcSystemCreate()
        try:
            try:
                count = DmvSDK.DcSystemGetDeviceCount(system)
            except AttributeError:
                raise RuntimeError(
                    "DcSystemGetDeviceCount not in DmvSDK — enumeration API "
                    "name is a guess. Inspect `dir(DmvSDK)` on the install "
                    "and update list_devices() accordingly."
                )

            devices: list[dict] = []
            for i in range(count):
                try:
                    dev = DmvSDK.DcSystemGetDevice(system, i)
                except Exception as e:
                    devices.append({"index": i, "error": f"{type(e).__name__}: {e}"})
                    continue
                if dev is None:
                    devices.append({"index": i, "error": "DcSystemGetDevice returned None"})
                    continue

                info: dict = {"index": i}
                _attr_map = [
                    ("display_name", "DC_DEVICE_INFO_DISPLAY_NAME"),
                    ("serial", "DC_DEVICE_INFO_SERIAL_NUMBER"),
                    ("user_id", "DC_DEVICE_INFO_USER_ID"),
                    ("model", "DC_DEVICE_INFO_MODEL"),
                ]
                for key, sdk_attr in _attr_map:
                    try:
                        const = getattr(DmvSDK, sdk_attr)
                        info[key] = DmvSDK.DcDeviceGetInfo(dev, const)
                    except (AttributeError, RuntimeError) as e:
                        info[key] = f"<unavailable: {type(e).__name__}>"
                devices.append(info)
            return devices
        finally:
            DmvSDK.DcSystemDestroy(system)

    def _select_device(self):
        """Resolve which device to open per __init__ requested_serial/index."""
        if self._requested_serial is not None:
            try:
                count = DmvSDK.DcSystemGetDeviceCount(self.system)
            except AttributeError:
                raise RuntimeError(
                    "cannot select by serial: DcSystemGetDeviceCount unavailable; "
                    "fall back to device_index= or fix list_devices() API names"
                )
            for i in range(count):
                dev = DmvSDK.DcSystemGetDevice(self.system, i)
                if dev is None:
                    continue
                try:
                    serial = DmvSDK.DcDeviceGetInfo(
                        dev, DmvSDK.DC_DEVICE_INFO_SERIAL_NUMBER
                    )
                except (AttributeError, RuntimeError):
                    continue
                if str(serial) == str(self._requested_serial):
                    logger.info("matched camera serial %s at index %d", serial, i)
                    return dev
            raise RuntimeError(
                f"no camera matching serial {self._requested_serial!r}; "
                "run `python -m robot_core.scripts.list_cameras` to see all"
            )

        if self._requested_index is not None:
            dev = DmvSDK.DcSystemGetDevice(self.system, self._requested_index)
            if dev is None:
                raise RuntimeError(f"no camera at index {self._requested_index}")
            return dev

        # No selection → first device + warn if multiple present.
        dev = DmvSDK.DcSystemGetDevice(self.system, None)
        if dev is None:
            raise RuntimeError("No camera found — check power + cable")
        try:
            count = DmvSDK.DcSystemGetDeviceCount(self.system)
            if count > 1:
                logger.warning(
                    "multiple cameras detected (%d); opened the first. "
                    "Pass serial=... to pick a specific one; "
                    "use scripts/list_cameras.py to enumerate.",
                    count,
                )
        except AttributeError:
            pass  # No count API; can't warn but default works
        return dev

    def open(self) -> None:
        """Connect to the requested camera (DmvSDK steps 1–5)."""
        if not HAS_DMV_SDK:
            raise RuntimeError(_DMV_SDK_MISSING_MSG)

        # Step 1: create SDK system instance
        self.system = DmvSDK.DcSystemCreate()

        # Step 2: enumerate + select device per __init__ params
        try:
            self.device = self._select_device()
        except RuntimeError:
            self._cleanup()
            raise

        # Step 3: open device in exclusive control mode
        DmvSDK.DcDeviceOpen(self.device, DmvSDK.DC_DEVICE_ACCESS_TYPE_CONTROL)

        device_name = DmvSDK.DcDeviceGetInfo(
            self.device, DmvSDK.DC_DEVICE_INFO_DISPLAY_NAME
        )
        logger.info("camera connected: %s", device_name)

        # Step 4: switch to single-frame acquisition + disable trigger
        nodelist = DmvSDK.DcDeviceGetRemoteNodeList(self.device)
        DmvSDK.DcNodeListSetValue(nodelist, "AcquisitionMode", "SingleFrame")
        DmvSDK.DcNodeListSetSelectedValue(
            nodelist, "TriggerSelector", "", "TriggerMode", "Off"
        )

        # Step 5: prepare data stream + allocate one buffer
        self.data_stream = DmvSDK.DcDeviceGetDataStream(self.device, 0)
        buf = DmvSDK.DcDataStreamAllocAndAnnounceBuffer(self.data_stream)
        DmvSDK.DcDataStreamQueueBuffer(self.data_stream, buf)

        self.is_open = True

    def grab_one_rgb(self, timeout_ms: int = 3000) -> np.ndarray:
        """Capture a single frame; return ``(H, W, 3) uint8`` RGB ndarray."""
        if not self.is_open:
            raise RuntimeError("camera not open — call .open() first")

        # Step 6: start → wait for filled buffer → stop
        DmvSDK.DcDataStreamStartAcquisition(self.data_stream)

        try:
            buffer = DmvSDK.DcDataStreamGetFilledBuffer(
                self.data_stream, timeout_ms
            )
        except RuntimeError as e:
            DmvSDK.DcDataStreamStopAcquisition(self.data_stream, True)
            raise RuntimeError(f"acquisition failed: {e}") from e

        DmvSDK.DcDataStreamStopAcquisition(self.data_stream, True)

        if not DmvSDK.DcBufferIsComplete(buffer):
            raise RuntimeError("frame incomplete (dropped packets in transit)")

        # Step 7: pull image and convert to RGB ndarray
        return self._image_to_rgb(DmvSDK.DcBufferGetImage(buffer))

    def start_continuous(self) -> None:
        """Switch to continuous acquisition mode and start the stream."""
        if not self.is_open:
            raise RuntimeError("camera not open — call .open() first")

        nodelist = DmvSDK.DcDeviceGetRemoteNodeList(self.device)
        DmvSDK.DcNodeListSetValue(nodelist, "AcquisitionMode", "Continuous")
        DmvSDK.DcNodeListSetSelectedValue(
            nodelist, "TriggerSelector", "", "TriggerMode", "Off"
        )

        # 4 buffers is the DMV-recommended sweet spot for continuous capture.
        for _ in range(4):
            buf = DmvSDK.DcDataStreamAllocAndAnnounceBuffer(self.data_stream)
            DmvSDK.DcDataStreamQueueBuffer(self.data_stream, buf)

        DmvSDK.DcDataStreamStartAcquisition(self.data_stream)
        logger.info("continuous stream started")

    def grab_continuous_rgb(self, timeout_ms: int = 1000) -> np.ndarray | None:
        """Continuous mode: grab one frame, requeue buffer, return RGB ndarray.

        Returns ``None`` on timeout or incomplete frame — the caller's loop
        skips and tries again rather than raising.
        """
        try:
            buffer = DmvSDK.DcDataStreamGetFilledBuffer(
                self.data_stream, timeout_ms
            )
        except RuntimeError:
            return None

        if not DmvSDK.DcBufferIsComplete(buffer):
            DmvSDK.DcDataStreamQueueBuffer(self.data_stream, buffer)
            return None

        image = DmvSDK.DcBufferGetImage(buffer)
        rgb = self._image_to_rgb(image)

        # Critical: requeue so SDK reuses the buffer; otherwise the pool
        # exhausts and the stream stalls (docs/dmv_sdk.md §6 warning).
        DmvSDK.DcDataStreamQueueBuffer(self.data_stream, buffer)

        return rgb

    def stop_continuous(self) -> None:
        """Stop the continuous stream."""
        DmvSDK.DcDataStreamStopAcquisition(self.data_stream, True)
        logger.info("continuous stream stopped")

    def close(self) -> None:
        """Close the camera and release the SDK system."""
        self._cleanup()

    def _cleanup(self) -> None:
        if self.system is not None:
            DmvSDK.DcSystemDestroy(self.system)
        self.system = None
        self.device = None
        self.data_stream = None
        self.buffer = None
        self.is_open = False

    def _image_to_rgb(self, image) -> np.ndarray:
        """Convert a DMV image handle to ``(H, W, 3) uint8`` RGB.

        Mono8 → broadcast to three channels for downstream consistency.
        Anything else → DMV converts to BGR8 in-place via a scratch image,
        which we destroy before returning (Color path docstring §5 in
        docs/dmv_sdk.md flags this leak risk).
        """
        width = DmvSDK.DcImageGetWidth(image)
        height = DmvSDK.DcImageGetHeight(image)
        pixel_format = DmvSDK.DcImageGetPixelFormat(image)
        logger.debug(
            "frame %dx%d %s",
            width,
            height,
            DmvSDK.DcPixelFormatToString(pixel_format),
        )

        if pixel_format == DmvSDK.Mono8:
            byte_array = DmvSDK.DcImageGetData(image)
            arr = np.array(byte_array, dtype=np.uint8).reshape(height, width)
            return np.stack([arr, arr, arr], axis=-1)

        image2 = DmvSDK.DcImageCreate()
        try:
            DmvSDK.DcImageConvertFormat(image, image2, DmvSDK.BGR8)
            bgr_bytes = DmvSDK.DcImageGetData(image2)
            arr_bgr = np.array(bgr_bytes, dtype=np.uint8).reshape(height, width, 3)
            return arr_bgr[:, :, ::-1].copy()
        finally:
            DmvSDK.DcImageDestroy(image2)

    def __enter__(self) -> "DeltaCamera":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()


__all__ = ["DeltaCamera", "HAS_DMV_SDK"]
