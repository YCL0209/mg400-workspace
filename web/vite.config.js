import { defineConfig } from "vite";

// M1 frontend: pure Vite dev server, no plugins, no proxy.
// The browser opens the ws connection directly to localhost:8765; CORS on the
// FastAPI side allows http://localhost:5173 (this dev origin).
export default defineConfig({
  server: { port: 5173, strictPort: true },
});
