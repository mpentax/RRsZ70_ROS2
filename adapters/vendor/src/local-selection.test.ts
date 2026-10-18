import { test } from "node:test";
import assert from "node:assert/strict";
import { selectedLocalProtocol } from "./credentials.ts";
import { SecretStore } from "./secret-store.ts";
import { mkdtemp, realpath, rm } from "node:fs/promises";
import { join } from "node:path";
import { tmpdir } from "node:os";
const device = { host: "192.0.2.1", duid: "fixture", name: "Z70", model: "a179", localKey: "0123456789abcdef", protocolVersion: "1.0", localProtocol: "L01" as const };
test("explicit local L01 persists independently of cloud pv 1.0", async t => {
  const dir = await realpath(await mkdtemp(join(tmpdir(), "local-selection-")));  // macOS tmpdir is under the /var symlink
  t.after(() => rm(dir, { recursive: true, force: true }));
  const store = new SecretStore(dir);
  await store.saveDevice(device);
  const saved = await store.loadDevice();
  assert.ok(saved); assert.equal(saved.protocolVersion, "1.0");
  assert.equal(selectedLocalProtocol(saved), "L01");
  assert.equal(selectedLocalProtocol({ ...device, protocolVersion: "L01", localProtocol: undefined }), "1.0");
  await assert.rejects(store.saveDevice({ ...device, localProtocol: "unknown" as "L01" }));
  assert.equal((await store.loadDevice())?.localProtocol, "L01");
});
