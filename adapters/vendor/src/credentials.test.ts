import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtemp, realpath, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { selectedLocalProtocol, summarizeStatus } from "./credentials.ts";
import { SecretStore } from "./secret-store.ts";

const device = { host: "192.0.2.1", duid: "fixture", name: "Z70", model: "a179", localKey: "0123456789abcdef", protocolVersion: "1.0", localProtocol: "L01" as const };
const camera = { account: { user: "u", secret: "s", key: "k", mqttUrl: "ssl://mqtt-eu.roborock.com:8883" }, pin: "1234" };
const home = { deviceId: "fixture", homeId: 1, realApi: "https://api-eu.roborock.com", user: "u", secret: "s", hawkKey: "k", rooms: [] };

test("status summary permits only selected scalar fields", () => {
  assert.deepEqual(summarizeStatus([{ battery: 90, state: 8, localKey: "secret", fw_ver: {} }]), {
    battery: 90, state: 8,
  });
  assert.deepEqual(summarizeStatus(null), {});
  assert.deepEqual(summarizeStatus([]), {});
});

test("explicit local L01 persists independently of cloud pv 1.0", async t => {
  const dir = await realpath(await mkdtemp(join(tmpdir(), "local-selection-")));
  t.after(() => rm(dir, { recursive: true, force: true }));
  const store = new SecretStore(dir);
  await store.savePairing({ device, camera, home });
  const saved = (await store.loadPairing())?.device;
  assert.ok(saved); assert.equal(saved.protocolVersion, "1.0");
  assert.equal(selectedLocalProtocol(saved), "L01");
  assert.equal(selectedLocalProtocol({ ...device, protocolVersion: "L01", localProtocol: undefined }), "1.0");
  await assert.rejects(store.savePairing({ device: { ...device, localProtocol: "unknown" as "L01" }, camera, home }));
  assert.equal((await store.loadPairing())?.device.localProtocol, "L01");
});
