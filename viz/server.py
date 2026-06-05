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

import json
import logging
import os
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from robot_core.safety.bounds import SafetyBounds

from .workspace import build_workspace_message

logger = logging.getLogger("viz.server")

_DEFAULT_ROBOT_JSON = Path(__file__).resolve().parent.parent / "config" / "robot.json"


def _load_viz_config() -> dict:
    """Read ``viz`` section from config/robot.json, fall back to defaults.

    Env vars override the file. Missing ``viz`` section is fine — defaults win.
    """
    cfg = {"host": "localhost", "port": 8765, "grid_step_mm": 50.0}
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


def create_app(*, bounds: SafetyBounds | None = None, grid_step_mm: float = 50.0) -> FastAPI:
    """Factory — accepts pre-loaded bounds for tests; production uses defaults."""
    app = FastAPI(title="MG400 inspection viz", version="0.1.0-m1")
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

    return app


def main() -> None:  # pragma: no cover — runs uvicorn, not unit-tested
    import uvicorn

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    cfg = _load_viz_config()
    app = create_app(grid_step_mm=cfg["grid_step_mm"])
    uvicorn.run(app, host=cfg["host"], port=cfg["port"], log_level="info")
