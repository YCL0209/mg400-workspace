"""FastAPI ws server for the inspection UI.

M1 endpoint: ``GET /ws`` accepts a WebSocket, sends one ``WorkspaceMessage``,
then keeps the connection open until the client disconnects. No state/FOV/
detection traffic until M2/M3.

Configuration: ``SafetyBounds`` loads from ``config/safety.json`` (default
path); host/port/grid_step come from ``config/robot.json`` ``viz`` section
with ``MG400_VIZ_HOST`` / ``MG400_VIZ_PORT`` / ``MG400_VIZ_GRID_STEP_MM`` env
overrides (same convention as transport/feedback ports — CLAUDE.md line 103).
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
from pathlib import Path
from typing import Callable, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from robot_core.safety.bounds import SafetyBounds

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
    """Production CalibSession: open DeltaCamera w/ configured serial, start continuous."""
    from robot_core.camera import DeltaCamera

    from .calib_session import CalibSession

    cfg = _load_viz_config()
    cam = DeltaCamera(serial=cfg.get("camera_serial"))
    cam.open()
    cam.start_continuous()
    return CalibSession(camera=cam), cam


def create_app(
    *,
    bounds: SafetyBounds | None = None,
    grid_step_mm: float = 50.0,
    calib_session_factory: Optional[Callable] = None,
) -> FastAPI:
    """Factory — accepts pre-loaded bounds for tests; production uses defaults.

    ``calib_session_factory`` returns ``(CalibSession, cleanup_target)`` where
    cleanup_target is whatever needs ``stop_continuous() + close()`` on
    disconnect (the camera, in production). Tests pass fakes here so unit
    tests run without DmvSDK.
    """
    app = FastAPI(title="MG400 inspection viz", version="0.2.0-m0b")
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

    return app


def main() -> None:  # pragma: no cover — runs uvicorn, not unit-tested
    import uvicorn

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    cfg = _load_viz_config()
    app = create_app(grid_step_mm=cfg["grid_step_mm"])
    uvicorn.run(app, host=cfg["host"], port=cfg["port"], log_level="info")
