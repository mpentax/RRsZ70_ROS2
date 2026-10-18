import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtemp, realpath, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { Client } from "@modelcontextprotocol/sdk/client/index.js";
import { StdioClientTransport } from "@modelcontextprotocol/sdk/client/stdio.js";
import { exitOnSourceChange, trackInflight } from "./mcp-reload.ts";

async function scratch(t: { after(fn: () => unknown): void }) {
  const dir = await realpath(await mkdtemp(join(tmpdir(), "mcp-reload-")));
  t.after(() => rm(dir, { recursive: true, force: true }));
  return dir;
}

async function spawnServer(t: { after(fn: () => unknown): void }, watchDir: string) {
  const tokenFile = join(watchDir, "token");
  await writeFile(tokenFile, "x".repeat(43), { mode: 0o600 });
  const transport = new StdioClientTransport({
    command: process.execPath, args: [join(import.meta.dirname, "mcp-main.ts")], stderr: "pipe",
    env: { ...process.env as Record<string, string>, ROBOCLAW_MCP_TOKEN_FILE: tokenFile, ROBOCLAW_MCP_WATCH_DIR: watchDir },
  });
  const client = new Client({ name: "reload-test", version: "0" });
  await client.connect(transport);
  const closed = new Promise<void>(resolve => { transport.onclose = () => resolve(); });
  t.after(() => client.close().catch(() => {}));
  return { client, closed };
}

const within = (p: Promise<unknown>, ms: number) =>
  Promise.race([p.then(() => true), new Promise(r => setTimeout(() => r(false), ms))]);

test("the stdio server exits after its source changes so the host respawns it with the new tools", async t => {
  const dir = await scratch(t);
  await writeFile(join(dir, "mcp-server.ts"), "export {};\n");
  const { client, closed } = await spawnServer(t, dir);
  assert.ok((await client.listTools()).tools.some(tool => tool.name === "set_arm_anchor"));
  await writeFile(join(dir, "arm-estimator.ts"), "export const changed = 1;\n");
  assert.equal(await within(closed, 10_000), true);
});

test("a source change that does not load keeps the running server", async t => {
  const dir = await scratch(t);
  await writeFile(join(dir, "mcp-server.ts"), "throw new Error('broken');\n");
  const { client, closed } = await spawnServer(t, dir);
  await writeFile(join(dir, "other.ts"), "export {};\n");
  assert.equal(await within(closed, 4_000), false);
  assert.ok((await client.listTools()).tools.length > 0);
});

test("restart waits for in-flight requests to be answered", async t => {
  const dir = await scratch(t);
  const sent: unknown[] = [];
  const transport = { onmessage: (_m: unknown) => {}, send: async (m: unknown) => { sent.push(m); }, start: async () => {}, close: async () => {} };
  const inflight = trackInflight(transport as never);
  transport.onmessage({ jsonrpc: "2.0", id: 7, method: "tools/call", params: {} });
  assert.equal(inflight.count, 1);
  let exited = false;
  const watcher = exitOnSourceChange({ dir, inflight, exit: () => { exited = true; }, debounceMs: 50, verify: async () => true, log: () => {} });
  t.after(() => watcher.close());
  await writeFile(join(dir, "x.ts"), "1");
  await new Promise(r => setTimeout(r, 600));
  assert.equal(exited, false);
  await transport.send({ jsonrpc: "2.0", id: 7, result: {} });
  assert.equal(inflight.count, 0);
  await new Promise(r => setTimeout(r, 400));
  assert.equal(exited, true);
});
