// Restart-on-change for the long-lived stdio MCP child.
// Hermes keeps an MCP child alive for the life of its process (the desktop app runs for days), so new or
// changed tools never appear until that child is replaced. When RoboClaw's source changes, this verifies the
// new code loads, waits until no request is in flight (exiting mid-call would leave a robot command's outcome
// unknown), then exits. Hermes' stdio keepalive notices, respawns from the new code and re-lists the tools.
import { spawn } from "node:child_process";
import { watch, type FSWatcher } from "node:fs";
import { pathToFileURL } from "node:url";
import { join } from "node:path";
import type { Transport } from "@modelcontextprotocol/sdk/shared/transport.js";

type Id = string | number;

/** Counts JSON-RPC requests received and not yet answered on a connected transport. */
export function trackInflight(transport: Transport): { readonly count: number } {
  const open = new Set<Id>();
  const onmessage = transport.onmessage;
  transport.onmessage = (message, extra) => {
    const m = message as { id?: Id; method?: string };
    if (m.method !== undefined && m.id !== undefined) open.add(m.id);
    onmessage?.(message, extra);
  };
  const send = transport.send.bind(transport);
  transport.send = async (message, options) => {
    const m = message as { id?: Id; method?: string };
    try { return await send(message, options); }
    finally { if (m.method === undefined && m.id !== undefined) open.delete(m.id); }
  };
  return { get count() { return open.size; } };
}

/** True when the server module imports cleanly in a fresh Node process. */
export function sourceLoads(dir: string, timeoutMs = 20_000): Promise<boolean> {
  return new Promise(resolve => {
    const url = pathToFileURL(join(dir, "mcp-server.ts")).href;
    const child = spawn(process.execPath, ["--input-type=module", "-e", `await import(${JSON.stringify(url)})`],
      { stdio: "ignore" });
    const timer = setTimeout(() => { child.kill(); resolve(false); }, timeoutMs);
    child.on("exit", code => { clearTimeout(timer); resolve(code === 0); });
    child.on("error", () => { clearTimeout(timer); resolve(false); });
  });
}

export interface ReloadOptions {
  dir: string;
  inflight: { readonly count: number };
  exit?: () => void;
  debounceMs?: number;
  verify?: (dir: string) => Promise<boolean>;
  log?: (line: string) => void;
}

export function exitOnSourceChange(o: ReloadOptions): FSWatcher {
  const exit = o.exit ?? (() => process.exit(0));
  const log = o.log ?? (line => console.error(line));        // stderr only: stdout carries MCP protocol
  const verify = o.verify ?? sourceLoads;
  let timer: NodeJS.Timeout | undefined, pending = false;
  const watcher = watch(o.dir, (_event, file) => {
    if (!file || !file.endsWith(".ts") || file.endsWith(".test.ts")) return;
    clearTimeout(timer);
    timer = setTimeout(async () => {
      if (pending) return;
      if (!(await verify(o.dir))) { log("RoboClaw MCP: source changed but does not load; keeping the running server."); return; }
      pending = true;
      log("RoboClaw MCP: source changed; restarting once idle so Hermes picks up the new tools.");
      const poll = setInterval(() => { if (o.inflight.count === 0) { clearInterval(poll); watcher.close(); exit(); } }, 200);
    }, o.debounceMs ?? 1500);
  });
  watcher.unref();
  return watcher;
}
