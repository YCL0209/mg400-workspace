/**
 * Minimal WebSocket client with auto-reconnect.
 *
 * The backend (viz/server.py) sends ``workspace`` on connect and stays silent
 * after that in M1 (M2 will push ``state``). We dispatch on ``msg.type`` so
 * adding new message types is a one-line addition in main.js.
 */
export function connectWs(url, { onOpen, onClose, onError, onMessage } = {}) {
  let ws;
  let reconnectDelayMs = 1000;

  const open = () => {
    ws = new WebSocket(url);
    ws.addEventListener("open", () => {
      reconnectDelayMs = 1000;
      onOpen && onOpen();
    });
    ws.addEventListener("close", () => {
      onClose && onClose();
      setTimeout(open, reconnectDelayMs);
      reconnectDelayMs = Math.min(reconnectDelayMs * 2, 10_000);
    });
    ws.addEventListener("error", (e) => {
      onError && onError(e);
    });
    ws.addEventListener("message", (e) => {
      let msg;
      try {
        msg = JSON.parse(e.data);
      } catch (err) {
        console.warn("[ws] non-JSON message", e.data);
        return;
      }
      onMessage && onMessage(msg);
    });
  };

  open();
  return {
    close: () => ws && ws.close(),
    send: (data) => ws && ws.readyState === WebSocket.OPEN && ws.send(data),
  };
}
