/**
 * Bootstrap for the M0b calibration page.
 *
 * Mirror of main.js but targets /ws/calib instead of /ws and wires up the
 * calib-specific dispatch + keyboard shortcuts. The workspace 3D scene
 * does NOT run on this page (it has its own index.html) -- this page is
 * a single-purpose tool for one-off intrinsics calibration setup.
 */

import { connectWs } from "./ws.js";
import {
  applyCalibFrame,
  applyCalibResult,
  bindKeys,
  logLine,
  setWsStatus,
} from "./calib.js";

const WS_URL = "ws://localhost:8765/ws/calib";

const wsClient = connectWs(WS_URL, {
  onOpen: () => {
    setWsStatus("connected", true);
    logLine("ws connected to " + WS_URL);
  },
  onClose: () => {
    setWsStatus("disconnected", false);
    logLine("ws disconnected", "err");
  },
  onError: () => setWsStatus("error", false),
  onMessage: (msg) => {
    if (msg.type === "calib_frame") {
      applyCalibFrame(msg);
    } else if (msg.type === "calib_result") {
      applyCalibResult(msg);
    } else {
      console.warn("[calib] unknown message type", msg);
    }
  },
});

bindKeys((action) => {
  wsClient.send(JSON.stringify({ action }));
  logLine(`-> ${action}`, "action");
});
