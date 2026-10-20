import test from "node:test";
import assert from "node:assert/strict";
import { setTimeout as delay } from "node:timers/promises";
import { CameraSelector } from "./camera-selection.ts";
import { TransportError } from "./rpc-outcome.ts";
import { createService } from "./service.ts";
import { loadServiceConfig } from "./service-config.ts";

const jpeg = (camera: number) => Buffer.from([255,216,255,192,0,11,8,...(camera ? [3,128] : [4,176]),6,64,1,1,17,0,255,218,0,2,255,217]);

test("selection polls for a delayed bit flip after missing acknowledgement", async () => {
  let selected = 0, readsAfterWrite = 0;
  const writes: unknown[] = [];
  const selector = new CameraSelector(async (method, params) => {
    if (method === "app_change_camera") {
      writes.push(params); throw new TransportError("RPC timed out", "unknown");
    }
    if (writes.length && ++readsAfterWrite >= 3) selected = 1;
    return [{camera_status: selected << 15}];
  });
  assert.equal(await selector.select("gripper", async () => jpeg(0)), "gripper");
  assert.deepEqual(writes, [{flag: 1}]);
  assert.ok(readsAfterWrite >= 3);
});

test("default reads queued behind a switch do not flip it back", async () => {
  let selected = 0;
  const writes: unknown[] = [];
  const selector = new CameraSelector(async (method, params) => {
    if (method === "app_change_camera") {
      writes.push(params); selected = (params as {flag: number}).flag;
      throw new TransportError("RPC timed out", "unknown");
    }
    return [selected << 15];
  });
  const capture = async () => jpeg(selected);
  const results = await Promise.all([selector.run("gripper", capture), selector.run(undefined, capture), selector.run(undefined, capture)]);
  assert.deepEqual(results.map(result => result.camera), ["gripper", "gripper", "gripper"]);
  assert.deepEqual(writes, [{flag: 1}]);
});

test("selection does not swallow a non-timeout transport failure", async () => {
  const failure = new TransportError("connection failed", "unknown");
  let reads = 0;
  const selector = new CameraSelector(async method => {
    if (method === "app_change_camera") throw failure;
    reads++; return [0];
  });
  await assert.rejects(selector.select("gripper", async () => jpeg(0)), error => error === failure);
  assert.equal(reads, 2);
});

test("cached live status expires even when frame dimensions are unknown", async () => {
  let selected = 0, reads = 0;
  const selector = new CameraSelector(async () => { reads++; return [selected << 15]; });
  const unknownShape = async () => Buffer.from([255,216,255,192,0,11,8,0,12,0,16,1,1,17,0,255,218,0,2,255,217]);
  assert.equal((await selector.run(undefined, unknownShape)).camera, "front");
  selected = 1;
  await selector.run(undefined, unknownShape);
  assert.equal(reads, 1);
  await delay(1050);
  assert.equal((await selector.run(undefined, unknownShape)).camera, "gripper");
  assert.equal(reads, 2);
});

test("stale pre-switch JPEGs are drained, never tagged as gripper", async () => {
  let selected = 0, captures = 0;
  const selector = new CameraSelector(async (method, params) => {
    if (method === "app_change_camera") selected = (params as {flag:number}).flag;
    return [selected << 15];
  });
  const result = await selector.run("gripper", async () => jpeg(++captures < 3 ? 0 : selected));
  assert.equal(result.camera, "gripper");
  assert.equal(result.sizeCamera, "gripper");
  assert.equal(captures, 3);
});

test("a confirmed switch passes the selected camera without restarting the warm receivers", async () => {
  let bit = 0;
  const switched: string[] = [];
  const warmed: string[] = [];
  const selector = new CameraSelector(async (method, params) => {
    if (method === "get_camera_status") return [bit << 15];
    if (method === "app_change_camera") { bit = (params as { flag: number }).flag; throw new TransportError("Camera RPC timed out", "unknown"); }
    return ["ok"];
  }, (camera) => { switched.push(camera); });
  assert.equal(await selector.select("gripper", async (current) => { warmed.push(current); }), "gripper");
  assert.deepEqual(warmed, ["front"]);
  assert.deepEqual(switched, ["gripper"]);
  assert.equal(await selector.select("gripper", async (current) => { warmed.push(current); }), "gripper");
  assert.deepEqual(switched, ["gripper"]);
});

test("service switches between front and gripper using pre-warmed per-camera streams without closing go2rtc", async t => {
  let selected = 0, closed = 0;
  const framesRequested: string[] = [];
  const token = "c".repeat(43);
  const robot = {
    connection: { state: "connected", connectedAt: 1 },
    close() {},
    request(method: string, params?: unknown) { return this.rpc(method, params); },
    async rpc(method: string, params?: unknown) {
      if (method === "get_status") return [{ common_status: 49154 | (2 << 10), state: 39 }];
      if (method === "get_camera_status") return [selected << 15];
      if (method === "app_change_camera") { selected = (params as { flag: number }).flag; return ["ok"]; }
      return ["ok"];
    },
  };
  const store = {
    async initialize() {}, async serviceToken() { return token; },
    async loadPairing() {
      return {
        device: { duid: "robot1", host: "127.0.0.1", localKey: "0123456789abcdef", name: "Z70", model: "roborock.vacuum.a179", protocolVersion: "L01" as const },
        camera: { account: { user: "u", secret: "mock-long-camera-secret", key: "mock-long-camera-key", mqttUrl: "ssl://mqtt-eu.roborock.com:8883" }, pin: "1234" },
        home: { deviceId: "robot1", homeId: 1, realApi: "https://api-eu.roborock.com", user: "u", secret: "s", hawkKey: "k", rooms: [] },
      };
    },
    async savePairing() {},
  };
  const config = { ...loadServiceConfig(), port: 0 };
  const service = await createService({
    store, config, robotFactory: () => robot,
    driver: {
      async configure() {},
      async offer() { return { type: "answer" as const, sdp: "v=0" }; },
      async frame(camera = "front") { framesRequested.push(camera); return jpeg(camera === "gripper" ? 1 : 0); },
      close() { closed++; },
    },
  });
  await service.start();
  t.after(() => service.stop());
  const origin = service.origin;
  const headers = { Authorization: ["Bearer", token].join(" "), "Content-Type": "application/json" };

  const f1 = await fetch(origin + "/api/camera/frame", { method: "POST", headers, body: "{}" });
  assert.equal(f1.status, 200);
  assert.equal(f1.headers.get("x-roboclaw-camera"), "front");
  assert.equal(f1.headers.get("x-roboclaw-image-height"), "1200");

  const sel = await fetch(origin + "/api/driver/camera/select", { method: "POST", headers, body: JSON.stringify({ camera: "gripper" }) });
  assert.equal(sel.status, 200);
  assert.equal(closed, 0);

  const g1 = await fetch(origin + "/api/camera/frame", { method: "POST", headers, body: "{}" });
  assert.equal(g1.status, 200);
  assert.equal(g1.headers.get("x-roboclaw-camera"), "gripper");
  assert.equal(g1.headers.get("x-roboclaw-image-height"), "896");
  assert.equal(closed, 0);
  assert.deepEqual(framesRequested, ["front", "front", "gripper"]);
});
