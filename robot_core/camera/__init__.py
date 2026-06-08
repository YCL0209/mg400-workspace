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

# DcSystemUpdateInterfaceList / DcInterfaceUpdateDeviceList both take a
# ``timeout`` int in milliseconds. Local USB / GigE enumeration usually
# completes in <100ms; 1000 leaves headroom without freezing the UI on a
# stuck device. Override per call only if a slow camera shows up.
_ENUMERATION_TIMEOUT_MS = 1000


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

        Walks the GenTL hierarchy ``System → Interface → Device``: for each
        interface (USB controller / GigE NIC), refresh its device list and
        emit one dict per device with ``interface_index``, ``device_index``,
        ``display_name``, ``serial``, ``user_defined_name``, ``model``,
        ``vendor``, ``version``. Each info lookup is guarded so a single
        unknown constant doesn't abort the whole enumeration.

        ``device_index`` here is the per-interface index expected by
        :func:`DcInterfaceGetDevice`; it is *not* the flat index callers pass
        to :class:`DeltaCamera` via ``device_index=``. The flat index is the
        sequence number across all interfaces, which :meth:`_select_device`
        re-walks the same way to resolve.
        """
        if not HAS_DMV_SDK:
            raise RuntimeError(_DMV_SDK_MISSING_MSG)

        system = DmvSDK.DcSystemCreate()
        try:
            DmvSDK.DcSystemUpdateInterfaceList(system, _ENUMERATION_TIMEOUT_MS)
            n_interfaces = DmvSDK.DcSystemGetInterfaceCount(system)

            devices: list[dict] = []
            flat_index = 0
            for iface_idx in range(n_interfaces):
                try:
                    interface = DmvSDK.DcSystemGetInterface(system, iface_idx)
                except RuntimeError as e:
                    devices.append({
                        "interface_index": iface_idx,
                        "error": f"DcSystemGetInterface: {e}",
                    })
                    continue

                try:
                    DmvSDK.DcInterfaceOpen(interface)
                except RuntimeError as e:
                    devices.append({
                        "interface_index": iface_idx,
                        "error": f"DcInterfaceOpen: {e}",
                    })
                    continue

                # All info reads + iteration must happen inside this try —
                # DcInterfaceClose destroys the underlying devices so we
                # snapshot to plain dicts before closing.
                try:
                    DmvSDK.DcInterfaceUpdateDeviceList(interface, _ENUMERATION_TIMEOUT_MS)
                    n_devs = DmvSDK.DcInterfaceGetDeviceCount(interface)

                    for dev_idx in range(n_devs):
                        try:
                            device = DmvSDK.DcInterfaceGetDevice(interface, dev_idx)
                        except RuntimeError as e:
                            devices.append({
                                "interface_index": iface_idx,
                                "device_index": dev_idx,
                                "flat_index": flat_index,
                                "error": f"DcInterfaceGetDevice: {e}",
                            })
                            flat_index += 1
                            continue

                        info: dict = {
                            "interface_index": iface_idx,
                            "device_index": dev_idx,
                            "flat_index": flat_index,
                        }
                        for key, sdk_attr in [
                            ("display_name", "DC_DEVICE_INFO_DISPLAY_NAME"),
                            ("serial", "DC_DEVICE_INFO_SERIAL_NUMBER"),
                            ("user_defined_name", "DC_DEVICE_INFO_USER_DEFINED_NAME"),
                            ("model", "DC_DEVICE_INFO_MODEL"),
                            ("vendor", "DC_DEVICE_INFO_VENDOR"),
                            ("version", "DC_DEVICE_INFO_VERSION"),
                        ]:
                            try:
                                const = getattr(DmvSDK, sdk_attr)
                                info[key] = DmvSDK.DcDeviceGetInfo(device, const)
                            except (AttributeError, RuntimeError) as e:
                                info[key] = f"<unavailable: {type(e).__name__}>"
                        devices.append(info)
                        flat_index += 1
                finally:
                    DmvSDK.DcInterfaceClose(interface)
            return devices
        finally:
            DmvSDK.DcSystemDestroy(system)

    def _select_device(self):
        """Resolve which device to open per __init__ requested serial/index.

        Three paths:

        * ``serial`` set → build a ``DcDeviceSerialNumberHint`` and let
          :func:`DcSystemGetDevice` find the match natively; cheap because
          the SDK does the enumeration internally.
        * ``device_index`` set → walk ``System → Interface → Device`` and
          stop at the Nth device across all interfaces (a *flat* index).
        * Neither → ``DcSystemGetDevice(system, None)`` for the first
          available device; if a quick walk turns up >1 device anywhere,
          log a warning telling the operator to pin a serial.
        """
        if self._requested_serial is not None:
            hint = DmvSDK.DcDeviceSerialNumberHint(str(self._requested_serial))
            dev = DmvSDK.DcSystemGetDevice(self.system, hint)
            if dev is None:
                raise RuntimeError(
                    f"no camera matching serial {self._requested_serial!r}; "
                    "run `python -m robot_core.scripts.list_cameras` to enumerate"
                )
            logger.info("matched camera by serial %s", self._requested_serial)
            return dev

        if self._requested_index is not None:
            DmvSDK.DcSystemUpdateInterfaceList(self.system, _ENUMERATION_TIMEOUT_MS)
            n_ifaces = DmvSDK.DcSystemGetInterfaceCount(self.system)
            flat = 0
            for iface_idx in range(n_ifaces):
                iface = DmvSDK.DcSystemGetInterface(self.system, iface_idx)
                DmvSDK.DcInterfaceOpen(iface)
                keep_open = False
                try:
                    DmvSDK.DcInterfaceUpdateDeviceList(iface, _ENUMERATION_TIMEOUT_MS)
                    n_devs = DmvSDK.DcInterfaceGetDeviceCount(iface)
                    for dev_idx in range(n_devs):
                        if flat == self._requested_index:
                            keep_open = True
                            return DmvSDK.DcInterfaceGetDevice(iface, dev_idx)
                        flat += 1
                finally:
                    # Closing destroys the device we returned, so only
                    # close interfaces that don't hold our target.
                    if not keep_open:
                        DmvSDK.DcInterfaceClose(iface)
            raise RuntimeError(
                f"no camera at flat index {self._requested_index} "
                f"(only {flat} cameras total)"
            )

        # Default: count first (warn if >1), THEN ask the SDK for the
        # first device. Counting happens via interface walk; closing those
        # interfaces is safe because we haven't requested a device yet —
        # DcSystemGetDevice(system, None) does its own interface lifecycle
        # internally for the hint=None path.
        total = 0
        try:
            DmvSDK.DcSystemUpdateInterfaceList(self.system, _ENUMERATION_TIMEOUT_MS)
            n_ifaces = DmvSDK.DcSystemGetInterfaceCount(self.system)
            for iface_idx in range(n_ifaces):
                iface = DmvSDK.DcSystemGetInterface(self.system, iface_idx)
                DmvSDK.DcInterfaceOpen(iface)
                try:
                    DmvSDK.DcInterfaceUpdateDeviceList(iface, _ENUMERATION_TIMEOUT_MS)
                    total += DmvSDK.DcInterfaceGetDeviceCount(iface)
                finally:
                    DmvSDK.DcInterfaceClose(iface)
        except (AttributeError, RuntimeError):
            total = 0  # walk failed; skip warning, fall through

        dev = DmvSDK.DcSystemGetDevice(self.system, None)
        if dev is None:
            raise RuntimeError("No camera found — check power + cable")
        if total > 1:
            logger.warning(
                "multiple cameras detected (%d total across interfaces); "
                "opened the first. Pass serial=... to pin a specific one; "
                "see `python -m robot_core.scripts.list_cameras`.",
                total,
            )
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
