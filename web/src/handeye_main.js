/**
 * Bootstrap for the M0c hand-eye page.
 *
 * Mirror of calib_main.js but targets /ws/handeye and dispatches
 * handeye_frame / handeye_result messages.
 */

import { connectWs } from "./ws.js";
import {
  applyHandeyeFrame,
  applyHandeyeResult,
  bindKeys,
  logLine,
  setWsStatus,
} from "./handeye.js";

const WS_URL = "ws://localhost:8765/ws/handeye";

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
    if (msg.type === "handeye_frame") {
      applyHandeyeFrame(msg);
    } else if (msg.type === "handeye_result") {
      applyHandeyeResult(msg);
    } else {
      console.warn("[handeye] unknown message type", msg);
    }
  },
});

bindKeys((action) => {
  wsClient.send(JSON.stringify({ action }));
  logLine(`-> ${action}`, "action");
});
