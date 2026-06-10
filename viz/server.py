"""FastAPI ws server for the inspection UI.

M1 endpoint: ``GET /ws`` accepts a WebSocket, sends one ``WorkspaceMessage``,
then keeps the connection open until the client disconnects. No state/FOV/
detection traffic until M2/M3.

M0c-2 adds an application lifespan handler that opens an
:class:`AsyncFeedbackStream` to the arm's 30004 port and drives a
:class:`RobotState` via :class:`RobotStateMonitor`. The resolved state
flows into ``/ws/handeye`` so the operator sees ``ARM: ONLINE`` and
SPACE captures pair frames with live TCP poses. Mac dev without an arm
gets ``ARM: OFFLINE`` (connect fails fast at the configured timeout, or
disable entirely with ``MG400_VIZ_ARM=0``).

Configuration: ``SafetyBounds`` loads from ``config/safety.json`` (default
path); host/port/grid_step come from ``config/robot.json`` ``viz`` section
with ``MG400_VIZ_HOST`` / ``MG400_VIZ_PORT`` / ``MG400_VIZ_GRID_STEP_MM`` env
overrides. Arm endpoint comes from the same JSON's top-level ``ip`` and
``ports.feedback`` (+ ``transport.connect_timeout_s``) with the existing
``MG400_IP`` / ``MG400_FEEDBACK_PORT`` overrides honoured.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Callable, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from robot_core.safety.bounds import SafetyBounds
from robot_core.state import RobotState
from robot_core.state.monitor import RobotStateMonitor
from robot_core.transport import AsyncFeedbackStream

from .workspace import build_workspace_message

logger = logging.getLogger("viz.server")

_DEFAULT_ROBOT_JSON = Path(__file__).resolve().parent.parent / "config" / "robot.json"

# Time between successive calib frame pushes. ~10 fps is enough for the
# operator to see live preview + ChArUco overlay without saturating the ws
# channel with base64-encoded JPEGs. M2 high-throughput stream can drop this.
_CALIB_FRAME_INTERVAL_S = 0.1


def _load_viz_config() -> dict:
    """Read ``viz`` section from config/robot.json, fall back to defaults.

    Env vars override the file. Missing ``viz`` section is fine — defaults win.
    """
    cfg = {
        "host": "localhost",
        "port": 8765,
        "grid_step_mm": 50.0,
        "camera_serial": None,
    }
    try:
        with open(_DEFAULT_ROBOT_JSON, encoding="utf-8") as fh:
            file_cfg = json.load(fh).get("viz", {})
        cfg.update(file_cfg)
    except FileNotFoundError:
        logger.warning("config/robot.json missing — using viz defaults")

    if env_host := os.environ.get("MG400_VIZ_HOST"):
        cfg["host"] = env_host
    if env_port := os.environ.get("MG400_VIZ_PORT"):
        cfg["port"] = int(env_port)
    if env_step := os.environ.get("MG400_VIZ_GRID_STEP_MM"):
        cfg["grid_step_mm"] = float(env_step)
    return cfg


def _default_calib_session_factory():
    """Production CalibSession: open DeltaCamera w/ configured serial, start continuous.

    Threads the configured ``camera_serial`` through to CalibSession so the
    eventual ``config/camera_intrinsics.json`` artifact records WHICH camera
    its K matrix belongs to. Operators sharing a hub with multiple cameras
    need this to know which calibration applies to which lens setup.
    """
    from robot_core.camera import DeltaCamera

    from .calib_session import CalibSession

    cfg = _load_viz_config()
    serial = cfg.get("camera_serial")
    cam = DeltaCamera(serial=serial)
    cam.open()
    cam.start_continuous()
    return CalibSession(camera=cam, camera_serial=serial), cam


def _default_handeye_session_factory_with_arm(arm_state_holder: dict) -> Callable:
    """Build a factory that injects the lifespan-resolved RobotState.

    Returns a zero-arg callable producing ``(HandeyeSession, camera)``.
    Reading ``arm_state_holder["state"]`` at call time (per ws connect)
    means a delayed startup or future hot-reconnect could be picked up
    by new clients without recreating the factory.
    """
    from robot_core.camera import DeltaCamera

    from .handeye_session import HandeyeSession

    def factory():
        cfg = _load_viz_config()
        serial = cfg.get("camera_serial")
        cam = DeltaCamera(serial=serial)
        cam.open()
        cam.start_continuous()
        session = HandeyeSession(
            camera=cam,
            arm_state=arm_state_holder["state"],
            camera_serial=serial,
        )
        return session, cam

    return factory


def _load_arm_endpoint_config() -> dict:
    """Read arm host/feedback_port/connect_timeout for the M0c-2 lifespan.

    Reads ``config/robot.json`` top-level ``ip``, ``ports.feedback``, and
    ``transport.connect_timeout_s``. Env vars override the file values
    using the same names as the rest of the project (``MG400_IP`` /
    ``MG400_FEEDBACK_PORT`` per CLAUDE.md build/test section).
    """
    cfg = {"ip": "192.168.1.6", "feedback_port": 30004, "connect_timeout_s": 3.0}
    try:
        with open(_DEFAULT_ROBOT_JSON, encoding="utf-8") as fh:
            data = json.load(fh)
        cfg["ip"] = data.get("ip", cfg["ip"])
        cfg["feedback_port"] = (
            data.get("ports", {}).get("feedback", cfg["feedback_port"])
        )
        cfg["connect_timeout_s"] = (
            data.get("transport", {}).get("connect_timeout_s", cfg["connect_timeout_s"])
        )
    except FileNotFoundError:
        logger.warning("config/robot.json missing -- using arm endpoint defaults")

    if v := os.environ.get("MG400_IP"):
        cfg["ip"] = v
    if v := os.environ.get("MG400_FEEDBACK_PORT"):
        cfg["feedback_port"] = int(v)
    return cfg


def _arm_lifespan_enabled() -> bool:
    """``MG400_VIZ_ARM=0/false/no/off`` disables the lifespan connect attempt.

    Useful for Mac dev when no arm is reachable -- skips the timeout-bound
    wait at startup and goes straight to ARM: OFFLINE. Default on.
    """
    val = os.environ.get("MG400_VIZ_ARM", "1").lower()
    return val not in ("0", "false", "no", "off")


def create_app(
    *,
    bounds: SafetyBounds | None = None,
    grid_step_mm: float = 50.0,
    calib_session_factory: Optional[Callable] = None,
    handeye_session_factory: Optional[Callable] = None,
    enable_arm_lifespan: bool = True,
) -> FastAPI:
    """Factory — accepts pre-loaded bounds for tests; production uses defaults.

    ``calib_session_factory`` returns ``(CalibSession, cleanup_target)`` where
    cleanup_target is whatever needs ``stop_continuous() + close()`` on
    disconnect (the camera, in production). Tests pass fakes here so unit
    tests run without DmvSDK.

    ``enable_arm_lifespan`` toggles the M0c-2 startup hook that opens the
    arm feedback stream + RobotState. Tests almost always pass ``False``
    so unit suites don't try to reach 192.168.1.6 during ``TestClient``
    boot (TestClient runs lifespan by default; without the toggle that
    would hang for ``connect_timeout_s`` per test). The dedicated
    lifespan tests pass ``True`` and inject a fake stream opener.
    """
    # Closure shared by lifespan + the default handeye factory. Lifespan
    # writes the resolved arm state into ``holder["state"]`` on startup
    # success; the factory reads it when a ws client connects. Using a
    # dict rather than a nonlocal var lets the factory close cleanly over
    # a mutable cell (Python doesn't allow reassigning closed-over names
    # without ``nonlocal``, and the factory may be passed around / tested).
    arm_state_holder: dict = {"state": None, "monitor": None, "stream": None}

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        """Open arm feedback + RobotState on startup; tear down on shutdown.

        Graceful degradation: any failure (timeout, connection refused,
        env-var off) logs a warning and lets the app come up with
        ``arm_state_holder["state"] = None``. Operators see ARM: OFFLINE
        rather than the server refusing to boot.
        """
        if not enable_arm_lifespan:
            logger.info(
                "arm lifespan disabled (enable_arm_lifespan=False) -- ARM: OFFLINE"
            )
            yield
            return
        if not _arm_lifespan_enabled():
            logger.info("MG400_VIZ_ARM disabled in env -- ARM: OFFLINE")
            yield
            return

        arm_cfg = _load_arm_endpoint_config()
        stream = AsyncFeedbackStream(
            arm_cfg["ip"],
            arm_cfg["feedback_port"],
            connect_timeout_s=arm_cfg["connect_timeout_s"],
        )
        try:
            await stream.connect()
        except (OSError, asyncio.TimeoutError) as e:
            # Note: asyncio.wait_for wraps the inner OpenError as
            # TimeoutError on timeout; bare OSError covers refused/no-route.
            logger.warning(
                "arm feedback connect to %s:%d failed (%s) -- ARM: OFFLINE",
                arm_cfg["ip"],
                arm_cfg["feedback_port"],
                e,
            )
            await stream.close()
            yield
            return

        state = RobotState()
        monitor = RobotStateMonitor(stream, state)
        monitor.start()
        arm_state_holder["state"] = state
        arm_state_holder["monitor"] = monitor
        arm_state_holder["stream"] = stream
        logger.info(
            "arm feedback stream started at %s:%d -- ARM: ONLINE",
            arm_cfg["ip"],
            arm_cfg["feedback_port"],
        )
        try:
            yield
        finally:
            try:
                await monitor.stop()
            except Exception as e:
                logger.warning("arm monitor stop failed: %s", e)
            arm_state_holder["state"] = None
            arm_state_holder["monitor"] = None
            arm_state_holder["stream"] = None
            logger.info("arm feedback stream stopped")

    app = FastAPI(
        title="MG400 inspection viz", version="0.3.0-m0c-2", lifespan=lifespan
    )
    # Expose the holder via app.state so tests + future debugging endpoints
    # can introspect whether the arm came up cleanly. Production code
    # doesn't need this -- the handeye factory closes over the same
    # holder via the closure above.
    app.state.arm_state_holder = arm_state_holder
    # Vite dev server runs on :5173; ws upgrades aren't CORS-checked by
    # browsers, but having permissive CORS lets future REST endpoints (M3
    # phase5-panel POST detections) work without per-route headers.
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    resolved_bounds = bounds if bounds is not None else SafetyBounds.load()
    resolved_calib_factory = calib_session_factory or _default_calib_session_factory
    resolved_handeye_factory = handeye_session_factory or (
        _default_handeye_session_factory_with_arm(arm_state_holder)
    )

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        message = build_workspace_message(resolved_bounds, grid_step_mm=grid_step_mm)
        await ws.send_json(message)
        logger.info("ws client connected — workspace sent")
        try:
            # M1 keeps the connection open; M2 will push state here.
            while True:
                # Block on client-side messages to detect disconnect cleanly.
                # We don't act on the payload yet.
                await ws.receive_text()
        except WebSocketDisconnect:
            logger.info("ws client disconnected")

    @app.websocket("/ws/calib")
    async def ws_calib(ws: WebSocket) -> None:
        """Live ChArUco capture session for M0b-3 frontend.

        Spawns two concurrent tasks for one client:

        - sender: ~10 fps stream of (jpeg + detection) messages
        - receiver: pulls action messages (capture/discard/reset/solve)
          and applies them to the shared CalibSession

        Either task ending (sender error, receiver disconnect) cancels the
        other and triggers camera cleanup. Only one ws client at a time --
        the SDK's exclusive mode on the camera prevents anything else
        anyway.
        """
        await ws.accept()
        try:
            session, cleanup_target = resolved_calib_factory()
        except Exception as e:
            logger.exception("calib session init failed: %s", e)
            await ws.send_json({
                "type": "calib_result",
                "success": False,
                "n_views": 0,
                "rms_px": float("nan"),
                "error": f"session init failed: {e}",
            })
            await ws.close()
            return

        logger.info("calib client connected — streaming frames")

        async def sender() -> None:
            while True:
                msg = await session.stream_frame()
                if msg is not None:
                    await ws.send_json(msg)
                await asyncio.sleep(_CALIB_FRAME_INTERVAL_S)

        async def receiver() -> None:
            while True:
                raw = await ws.receive_text()
                try:
                    payload = json.loads(raw)
                except json.JSONDecodeError:
                    logger.warning("calib: bad JSON from client: %r", raw)
                    continue
                result = session.apply_action(payload)
                if result is not None:
                    await ws.send_json(result)

        tasks = [asyncio.create_task(sender()), asyncio.create_task(receiver())]
        try:
            done, pending = await asyncio.wait(
                tasks, return_when=asyncio.FIRST_EXCEPTION
            )
            for t in pending:
                t.cancel()
            for t in done:
                exc = t.exception()
                if exc and not isinstance(exc, WebSocketDisconnect):
                    logger.exception("calib task crashed: %s", exc)
        finally:
            try:
                cleanup_target.stop_continuous()
            except Exception:
                pass
            try:
                cleanup_target.close()
            except Exception:
                pass
            logger.info("calib client disconnected, camera released")

    @app.websocket("/ws/handeye")
    async def ws_handeye(ws: WebSocket) -> None:
        """Live hand-eye capture session for M0c-1 frontend.

        Identical task structure to ``/ws/calib`` -- two coroutines (sender
        + receiver), failure on either side cancels the other and releases
        the camera. The only schema difference is each frame carries an
        ``arm`` payload (``available=False`` until M0c-2 wires a live
        ``RobotState`` into the factory).
        """
        await ws.accept()
        try:
            session, cleanup_target = resolved_handeye_factory()
        except Exception as e:
            logger.exception("handeye session init failed: %s", e)
            await ws.send_json({
                "type": "handeye_result",
                "success": False,
                "n_samples": 0,
                "error": f"session init failed: {e}",
            })
            await ws.close()
            return

        logger.info("handeye client connected -- streaming frames")

        async def sender() -> None:
            while True:
                msg = await session.stream_frame()
                if msg is not None:
                    await ws.send_json(msg)
                await asyncio.sleep(_CALIB_FRAME_INTERVAL_S)

        async def receiver() -> None:
            while True:
                raw = await ws.receive_text()
                try:
                    payload = json.loads(raw)
                except json.JSONDecodeError:
                    logger.warning("handeye: bad JSON from client: %r", raw)
                    continue
                result = session.apply_action(payload)
                if result is not None:
                    await ws.send_json(result)

        tasks = [asyncio.create_task(sender()), asyncio.create_task(receiver())]
        try:
            done, pending = await asyncio.wait(
                tasks, return_when=asyncio.FIRST_EXCEPTION
            )
            for t in pending:
                t.cancel()
            for t in done:
                exc = t.exception()
                if exc and not isinstance(exc, WebSocketDisconnect):
                    logger.exception("handeye task crashed: %s", exc)
        finally:
            try:
                cleanup_target.stop_continuous()
            except Exception:
                pass
            try:
                cleanup_target.close()
            except Exception:
                pass
            logger.info("handeye client disconnected, camera released")

    return app


def main() -> None:  # pragma: no cover — runs uvicorn, not unit-tested
    import uvicorn

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    cfg = _load_viz_config()
    app = create_app(grid_step_mm=cfg["grid_step_mm"])
    uvicorn.run(app, host=cfg["host"], port=cfg["port"], log_level="info")
