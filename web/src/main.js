import { connectWs } from "./ws.js";
import { initScene, applyWorkspace } from "./scene.js";

const WS_URL = "ws://localhost:8765/ws";

const scene = initScene(document.getElementById("canvas-root"));

const statusEl = document.getElementById("ws-status");
const infoEl = document.getElementById("ws-info");

function setStatus(text, ok) {
  statusEl.textContent = text;
  statusEl.className = ok ? "ok" : "bad";
}

connectWs(WS_URL, {
  onOpen: () => setStatus("connected", true),
  onClose: () => setStatus("disconnected", false),
  onError: (e) => {
    setStatus("error", false);
    infoEl.textContent = String(e);
  },
  onMessage: (msg) => {
    console.log("[ws]", msg);
    if (msg.type === "workspace") {
      applyWorkspace(scene, msg);
      infoEl.textContent =
        `annulus ${msg.annulus_inner_mm}–${msg.annulus_outer_mm} mm · ` +
        `J1 dead-zone ${msg.j1_rear_dead_zone_deg}° · grid ${msg.grid_step_mm} mm`;
    }
    // M2: state message goes here.
  },
});
