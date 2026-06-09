/**
 * M0c-1 hand-eye page view + key handler.
 *
 * Mirror of calib.js with an extra arm panel. Decodes the base64 JPEG,
 * updates the ChArUco stats (identical to calib), and renders the arm
 * status block from `msg.arm`. SPACE / D / R / Enter map to capture /
 * discard / reset / solve actions on the ws channel.
 *
 * When `arm.available === false`, the panel reads `ARM: OFFLINE` and
 * SPACE still works -- backend will record the sample with arm_pose=None
 * and M0c-3's solver will drop it. We don't block SPACE so Mac devs can
 * exercise the UI without a running arm.
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

const armLinkEl = $("arm-link");
const armModeEl = $("arm-mode");
const armEnabledEl = $("arm-enabled");
const armErrEl = $("arm-err");
const armXyEl = $("arm-xy");
const armZrEl = $("arm-zr");
const armJointsEl = $("arm-joints");

const residualStatEl = $("residual-stat");
const artifactStatEl = $("artifact-stat");
const logEl = $("log");

const CORNER_OK_THRESHOLD = 30;
const RESIDUAL_OK_THRESHOLD_MM = 2.0;

export function setWsStatus(text, ok) {
  wsStateEl.textContent = text;
  wsStateEl.className = "stat-value " + (ok ? "ok" : "warn");
}

export function applyHandeyeFrame(msg) {
  frameEl.src = "data:image/jpeg;base64," + msg.jpeg_b64;
  frameEl.alt = `frame @ ${msg.timestamp_ms} ms`;

  const det = msg.detection || {};
  const cap = msg.captures || {};
  const arm = msg.arm || { available: false };

  overlayEl.textContent =
    `corners ${det.charuco_corners_found}/${det.charuco_corners_total}` +
    `  |  captures ${cap.collected}/${cap.target}` +
    `  |  arm ${arm.available ? "ONLINE" : "OFFLINE"}`;

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

  const pose = det.board_pose;
  if (pose && typeof pose.tz_mm === "number") {
    distanceStatEl.textContent = `${pose.tz_mm.toFixed(1)} mm`;
    distanceStatEl.className = "stat-value ok";
  } else {
    distanceStatEl.textContent = "—";
    distanceStatEl.className = "stat-value";
  }

  renderArm(arm);
}

function renderArm(arm) {
  if (!arm.available) {
    armLinkEl.textContent = "OFFLINE";
    armLinkEl.className = "stat-value offline";
    for (const el of [armModeEl, armEnabledEl, armErrEl, armXyEl, armZrEl, armJointsEl]) {
      el.textContent = "—";
      el.className = "stat-value";
    }
    return;
  }
  armLinkEl.textContent = "ONLINE";
  armLinkEl.className = "stat-value ok";

  armModeEl.textContent = String(arm.mode);
  // Mode 5 = ENABLE (idle, ready for motion). Anything else means the operator
  // shouldn't be pressing SPACE yet -- we don't block, just colour-warn.
  armModeEl.className = "stat-value " + (arm.mode === 5 ? "ok" : "warn");

  armEnabledEl.textContent = arm.enabled ? "yes" : "no";
  armEnabledEl.className = "stat-value " + (arm.enabled ? "ok" : "bad");

  armErrEl.textContent = arm.has_error ? "yes" : "no";
  armErrEl.className = "stat-value " + (arm.has_error ? "bad" : "ok");

  const p = arm.pose || {};
  armXyEl.textContent =
    `${(p.x ?? 0).toFixed(1)} / ${(p.y ?? 0).toFixed(1)} mm`;
  armXyEl.className = "stat-value";
  armZrEl.textContent =
    `${(p.z ?? 0).toFixed(1)} mm / ${(p.r ?? 0).toFixed(1)}°`;
  armZrEl.className = "stat-value";

  const j = arm.joints || [];
  armJointsEl.textContent =
    j.length === 4
      ? j.map((v) => v.toFixed(1)).join(" ")
      : "—";
  armJointsEl.className = "stat-value";
}

export function applyHandeyeResult(msg) {
  if (msg.success) {
    const rms = (msg.rms_residual_mm ?? NaN).toFixed(3);
    residualStatEl.textContent = `${rms} mm`;
    residualStatEl.className =
      "stat-value " + (msg.rms_residual_mm < RESIDUAL_OK_THRESHOLD_MM ? "ok" : "warn");
    artifactStatEl.textContent = msg.artifact_path || "—";
    artifactStatEl.className = "stat-value ok";
    logLine(
      `solve OK: residual=${rms} mm n_samples=${msg.n_samples} -> ${msg.artifact_path}`,
      "ok",
    );
  } else {
    residualStatEl.textContent = "failed";
    residualStatEl.className = "stat-value bad";
    artifactStatEl.textContent = "—";
    artifactStatEl.className = "stat-value";
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
    if (e.target.tagName === "INPUT" || e.target.tagName === "TEXTAREA") return;
    const action = KEY_TO_ACTION[e.key];
    if (!action) return;
    e.preventDefault();
    sendAction(action);
  });
}
