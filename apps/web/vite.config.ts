import { defineConfig } from "vite";

export default defineConfig({
  root: new URL(".", import.meta.url).pathname,
  base: "/",
  server: { host: "127.0.0.1", port: 57000, strictPort: true },
  preview: { host: "127.0.0.1", port: 57000, strictPort: true },
  build: {
    target: "es2022",
    outDir: "../../dist/web",
    emptyOutDir: true,
  },
});
