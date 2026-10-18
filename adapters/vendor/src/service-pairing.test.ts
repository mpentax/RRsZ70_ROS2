import { test } from "node:test";
import assert from "node:assert/strict";
import { createService, type ServiceOptions, type ServiceStore } from "./service.ts";
import { loadServiceConfig } from "./service-config.ts";
import type { CameraLogin } from "./cloud.ts";
const token = "x".repeat(43);
const device = { duid: "robot1", host: "127.0.0.1", localKey: "1234567890abcdef", name: "test", model: "a179", protocolVersion: "L01" };
const camera = { account: { user: "old", secret: "s", key: "k", mqttUrl: "mqtts://mqtt.roborock.com:8883" }, pin: "1234" };
const home = { deviceId: "robot1", homeId: 1, realApi: "https://example.invalid", user: "old", secret: "s", hawkKey: "k", rooms: [] };
const login: CameraLogin = { devices: [device], account: { ...camera.account, user: "new" }, home: { ...home, user: "new" } };
function deferred<T>() { return Promise.withResolvers<T>(); }
async function fixture(t: import("node:test").TestContext, overrides: Partial<ServiceOptions>, savePairing: NonNullable<ServiceStore["savePairing"]>) {
  const store: ServiceStore = { async initialize() {}, async serviceToken() { return token; }, async loadPairing() { return { device, camera, home }; }, async loadDevice() { return null; }, async saveDevice() {}, async loadCamera() { return null; }, async saveCamera() {}, savePairing };
  const config = { ...loadServiceConfig(), port: 0 };
  const service = await createService({ store, config, robotFactory: () => ({ connection: { state: "connected", connectedAt: 1 }, close() {}, async request() { return ["ok"]; } }), ...overrides });
  await service.start(); t.after(() => service.stop());
  const address = service.server.address(); assert.ok(address && typeof address === "object");
  const origin = `http://127.0.0.1:${address.port}`; config.allowedHosts.push(`127.0.0.1:${address.port}`); config.publicOrigin = origin;
  return async (path: string, body: unknown) => {
    const response = await fetch(origin + path, { method: "POST", headers: { Authorization: `Bearer ${token}`, "Content-Type": "application/json" }, body: JSON.stringify(body) });
    return { status: response.status, body: await response.json() };
  };
}
test("stale room refresh cannot persist credentials after same-device account selection", async t => {
  const rooms = deferred<[]>(), started = deferred<void>(); const writes: unknown[] = [];
  const post = await fixture(t, { roomRefresher: async () => { started.resolve(); return rooms.promise; }, cloudFactory: () => ({ async requestCode() {}, async loginCameraWithCode() { return login; } }) }, async p => { writes.push(p); });
  const refresh = post("/api/home/rooms/refresh", {}); await started.promise;
  assert.equal((await post("/api/account/code", { email: "user@example.com", region: "eu" })).status, 200);
  assert.equal((await post("/api/account/login", { code: "123456" })).status, 200);
  assert.equal((await post("/api/account/select", { id: "robot1", host: "127.0.0.1" })).status, 200);
  rooms.resolve([]); assert.equal((await refresh).status, 409); assert.equal(writes.length, 1);
});
test("concurrent login completion does not dereference or overwrite cleared attempt", async t => {
  const delayed = deferred<CameraLogin>(), entered = deferred<void>(); let count = 0;
  const post = await fixture(t, { cloudFactory: () => ({ async requestCode() {}, async loginCameraWithCode() { if (++count === 1) { entered.resolve(); return delayed.promise; } return login; } }) }, async () => {});
  await post("/api/account/code", { email: "user@example.com", region: "eu" });
  const first = post("/api/account/login", { code: "123456" }); await entered.promise;
  assert.equal((await post("/api/account/login", { code: "234567" })).status, 200);
  assert.equal((await post("/api/account/select", { id: "robot1", host: "127.0.0.1" })).status, 200);
  delayed.resolve(login); assert.equal((await first).status, 409);
});
test("concurrent selections serialize without publishing the same consumed attempt twice", async t => {
  const saving = deferred<void>(), entered = deferred<void>(); let writes = 0;
  const post = await fixture(t, { cloudFactory: () => ({ async requestCode() {}, async loginCameraWithCode() { return login; } }) }, async () => { writes++; entered.resolve(); await saving.promise; });
  await post("/api/account/code", { email: "user@example.com", region: "eu" }); await post("/api/account/login", { code: "123456" });
  const first = post("/api/account/select", { id: "robot1", host: "127.0.0.1" }); await entered.promise;
  const second = post("/api/account/select", { id: "robot1", host: "127.0.0.2" });
  // The second handler must run before the first persistence finishes.
  await new Promise(resolve => setTimeout(resolve, 25)); saving.resolve();
  assert.equal((await first).status, 200); assert.equal((await second).status, 409); assert.equal(writes, 1);
});
test("room refresh cannot overwrite a concurrently published camera PIN pairing", async t => {
  const rooms = deferred<[]>(), entered = deferred<void>(); const writes: { camera: { pin: string } }[] = [];
  const post = await fixture(t, { roomRefresher: async () => { entered.resolve(); return rooms.promise; } }, async p => { writes.push(p); });
  const refresh = post("/api/home/rooms/refresh", {}); await entered.promise;
  assert.equal((await post("/api/camera/pin", { pin: "2345" })).status, 200);
  rooms.resolve([]); assert.equal((await refresh).status, 409);
  assert.equal(writes.length, 1); assert.equal(writes[0].camera.pin, "2345");
});
