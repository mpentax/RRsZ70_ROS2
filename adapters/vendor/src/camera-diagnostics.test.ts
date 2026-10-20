import test from "node:test";
import assert from "node:assert/strict";
import { Go2Rtc } from "./go2rtc.ts";
import { startService, type ServiceStore } from "./service.ts";
import { loadServiceConfig } from "./service-config.ts";

test("private acquisition reports an empty go2rtc startup response without private text", async t => {
  const realFetch = globalThis.fetch;
  let captures = 0;
  t.mock.method(globalThis, "fetch", async (input: Parameters<typeof fetch>[0], init?: RequestInit) => {
    const url = String(input);
    if (!url.startsWith("http://127.0.0.1:57001/")) return realFetch(input, init);
    if (url.includes("/api/streams")) return new Response(null);
    captures++;
    return new Response(null, { status: 200 });
  });
  const token = "t".repeat(43);
  const store: ServiceStore = { async initialize() {}, async serviceToken() { return token; },
    async loadPairing() {
      return {
        device: { duid: "test-device", host: "127.0.0.1", localKey: "1234567890abcdef", name: "test", model: "a179", protocolVersion: "L01" },
        camera: { account: { user: "synthetic-user", secret: "synthetic-secret", key: "synthetic-account-key", mqttUrl: "ssl://mqtt-eu.roborock.com:8883" }, pin: "1234" },
        home: { deviceId: "test-device", homeId: 1, realApi: "https://api-eu.roborock.com", user: "synthetic-user", secret: "s", hawkKey: "k", rooms: [] },
      };
    },
    async savePairing() {} };
  const service = await startService({ config: { ...loadServiceConfig(), port: 0 }, store, driver: new Go2Rtc(), robotFactory: () => ({
    connection: { state: "connected", connectedAt: 1 }, close() {}, async request(method) {
      assert.equal(method, "get_camera_status"); return [11685];
    },
  }) });
  t.after(() => service.stop());
  const response = await realFetch(service.origin+'/api/camera/frame',{method:'POST',headers:{Authorization:['Bearer',token].join(' '),'Content-Type':'application/json'},body:'{}'});
  assert.equal(response.status,502);
  const error=await response.json() as Record<string,unknown>;
  assert.equal(error.cameraFailure,'no_camera_image');assert.equal(error.upstreamStatus,200);
  assert.equal(JSON.stringify(error).includes('synthetic-secret'),false);
  assert.equal(captures, 1);
});
