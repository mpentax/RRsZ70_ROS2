// Restart-on-change for the long-lived stdio MCP child.
import { watch, type FSWatcher } from "node:fs";
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

export interface ReloadOptions {
  dir: string;
  inflight: { readonly count: number };
  exit?: () => void;
  debounceMs?: number;
  log?: (line: string) => void;
}

export function exitOnSourceChange(o: ReloadOptions): FSWatcher {
  const exit = o.exit ?? (() => process.exit(0));
  const log = o.log ?? (line => console.error(line));
  let timer: NodeJS.Timeout | undefined, pending = false;
  const watcher = watch(o.dir, (_event, file) => {
    if (!file || !file.endsWith(".ts") || file.endsWith(".test.ts")) return;
    clearTimeout(timer);
    timer = setTimeout(() => {
      if (pending) return;
      pending = true;
      log("RoboClaw MCP: source changed; restarting once idle.");
      const poll = setInterval(() => {
        if (o.inflight.count === 0) { clearInterval(poll); watcher.close(); exit(); }
      }, 50);
    }, o.debounceMs ?? 250);
  });
  watcher.unref();
  return watcher;
}
