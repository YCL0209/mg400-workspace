/**
 * M0b calibration page view + key handler.
 *
 * Decodes the base64 JPEG into an <img>, updates the stats panel from
 * the per-frame detection summary, drives a log pane, and maps SPACE / D /
 * R / Enter to capture / discard / reset / solve actions on the ws channel.
 *
 * Threshold colouring is intentionally loose: corner counts below 30 turn
 * warn-orange (calibration solver tolerates fewer but accuracy drops);
 * board-visible false turns warn (the operator needs to reposition the
 * board into frame); rms < 1.0 px after solve turns ok-green.
 */

const $ = (id) => document.getElementById(id);
const frameEl = $("frame");
const overlayEl = $("overlay");
const wsStateEl = $("ws-state");
const boardStatEl = $("board-stat");
const cornersStatEl = $("corners-stat");
const markersStatEl = $("markers-stat");
const capturesStatEl = $("captures-stat");
const distanceStatEl = $("distance-stat");
const rmsStatEl = $("rms-stat");
const logEl = $("log");

const CORNER_OK_THRESHOLD = 30;
const RMS_OK_THRESHOLD = 1.0;

export function setWsStatus(text, ok) {
  wsStateEl.textContent = text;
  wsStateEl.className = "stat-value " + (ok ? "ok" : "warn");
}

export function applyCalibFrame(msg) {
  frameEl.src = "data:image/jpeg;base64," + msg.jpeg_b64;
  frameEl.alt = `frame @ ${msg.timestamp_ms} ms`;

  const det = msg.detection || {};
  const cap = msg.captures || {};

  // Top-left HUD over the frame -- compact.
  overlayEl.textContent =
    `corners ${det.charuco_corners_found}/${det.charuco_corners_total}` +
    `  |  captures ${cap.collected}/${cap.target}`;

  // Side panel detail.
  boardStatEl.textContent = det.board_visible ? "yes" : "no";
  boardStatEl.className =
    "stat-value " + (det.board_visible ? "ok" : "warn");

  cornersStatEl.textContent =
    `${det.charuco_corners_found} / ${det.charuco_corners_total}`;
  cornersStatEl.className =
    "stat-value " +
    (det.charuco_corners_found >= CORNER_OK_THRESHOLD ? "ok" : "warn");

  const markerCount = (det.marker_ids || []).length;
  markersStatEl.textContent = String(markerCount);
  markersStatEl.className = "stat-value " + (markerCount > 0 ? "ok" : "warn");

  capturesStatEl.textContent = `${cap.collected} / ${cap.target}`;
  capturesStatEl.className =
    "stat-value " + (cap.collected >= cap.target ? "ok" : "");

  // board_pose is omitted (a) before calibration (no K loaded) or (b)
  // when cv2 couldn't solve pose this frame. Render "--" in both cases
  // so the operator doesn't see flicker between valid + missing frames.
  const pose = det.board_pose;
  if (pose && typeof pose.tz_mm === "number") {
    distanceStatEl.textContent = `${pose.tz_mm.toFixed(1)} mm`;
    distanceStatEl.className = "stat-value ok";
  } else {
    distanceStatEl.textContent = "—";
    distanceStatEl.className = "stat-value";
  }
}

export function applyCalibResult(msg) {
  if (msg.success) {
    const rms = msg.rms_px.toFixed(3);
    rmsStatEl.textContent = `${rms} px`;
    rmsStatEl.className =
      "stat-value " + (msg.rms_px < RMS_OK_THRESHOLD ? "ok" : "warn");
    logLine(`solve OK: rms=${rms} n_views=${msg.n_views}`, "ok");
  } else {
    rmsStatEl.textContent = "failed";
    rmsStatEl.className = "stat-value bad";
    logLine(`solve failed: ${msg.error}`, "err");
  }
}

export function logLine(text, cls = "") {
  const entry = document.createElement("div");
  entry.className = "entry " + cls;
  const ts = new Date().toLocaleTimeString();
  entry.textContent = `[${ts}] ${text}`;
  logEl.appendChild(entry);
  logEl.scrollTop = logEl.scrollHeight;
  // Cap log length so a long session doesn't blow up the DOM.
  while (logEl.children.length > 200) {
    logEl.removeChild(logEl.firstChild);
  }
}

const KEY_TO_ACTION = {
  " ": "capture",
  "d": "discard",
  "D": "discard",
  "r": "reset",
  "R": "reset",
  "Enter": "solve",
};

export function bindKeys(sendAction) {
  window.addEventListener("keydown", (e) => {
    // Allow normal typing in any future <input> elements.
    if (e.target.tagName === "INPUT" || e.target.tagName === "TEXTAREA") return;
    const action = KEY_TO_ACTION[e.key];
    if (!action) return;
    e.preventDefault();
    sendAction(action);
  });
}
