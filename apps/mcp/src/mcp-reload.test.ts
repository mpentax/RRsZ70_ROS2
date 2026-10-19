import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtemp, realpath, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { exitOnSourceChange, trackInflight } from "./mcp-reload.ts";

async function scratch(t: { after(fn: () => unknown): void }) {
  const dir = await realpath(await mkdtemp(join(tmpdir(), "mcp-reload-")));
  t.after(() => rm(dir, { recursive: true, force: true }));
  return dir;
}

test("exitOnSourceChange ignores test files and exits once in-flight requests finish", async t => {
  const dir = await scratch(t);
  const sent: unknown[] = [];
  const transport = {
    onmessage: (_m: unknown) => {},
    send: async (m: unknown) => { sent.push(m); },
    start: async () => {},
    close: async () => {},
  };
  const inflight = trackInflight(transport as never);
  transport.onmessage({ jsonrpc: "2.0", id: 7, method: "tools/call", params: {} });
  assert.equal(inflight.count, 1);

  let exited = false;
  const watcher = exitOnSourceChange({
    dir,
    inflight,
    exit: () => { exited = true; },
    debounceMs: 25,
    log: () => {},
  });
  t.after(() => watcher.close());

  await writeFile(join(dir, "ignored.test.ts"), "1");
  await new Promise(r => setTimeout(r, 80));
  assert.equal(exited, false);

  await writeFile(join(dir, "mcp-server.ts"), "export {};\n");
  await new Promise(r => setTimeout(r, 80));
  assert.equal(exited, false);

  await transport.send({ jsonrpc: "2.0", id: 7, result: {} });
  assert.equal(inflight.count, 0);
  await new Promise(r => setTimeout(r, 80));
  assert.equal(exited, true);
});
