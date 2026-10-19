import test from "node:test";
import assert from "node:assert/strict";
import { createService, type ServiceRobot, type ServiceStore } from "./service.ts";
import { loadServiceConfig } from "./service-config.ts";
import { RpcError } from "./robot.ts";
import { TransportError } from "./rpc-outcome.ts";

const jpeg = (selected: number) => Buffer.from([255,216,255,192,0,11,8,...(selected ? [3,128] : [4,176]),6,64,1,1,17,0,255,218,0,2,255,217]);
async function fixture(t: import("node:test").TestContext) {
  const calls: string[] = [], timeouts: number[] = [];
  let selected = 0, common = 49154, captureFailure = false, noFlip = false, selectionUnknown = false, wrongShape = false;
  const token = "c".repeat(43);
  const robot: ServiceRobot = { connection: { state: "connected", connectedAt: 1 }, close() {}, request(method, params) { return this.rpc!(method, params); }, async rpc(method, params, timeoutMs) {
    calls.push(method + (method === "app_change_camera" ? `:${(params as {flag:number}).flag}` : ""));
    if (method === "get_status") return [{ common_status: common, state: 39 }];
    if (method === "get_camera_status") return [selected << 15];
    if (method === "app_change_camera") {
      const flag = (params as {flag:number}).flag;
      timeouts.push(timeoutMs!);
      if (flag === 1 && ((common >> 10) & 3) === 0) throw new RpcError(-10007);
      if (!noFlip) selected = flag;
      if (selectionUnknown || noFlip) throw new TransportError("RPC timed out", "unknown");
    }
    return ["ok"];
  }};
  const store: ServiceStore = { async initialize() {}, async serviceToken() { return token; },
    async loadPairing() {
      return {
        device: { duid: "robot1", host: "127.0.0.1", localKey: "0123456789abcdef", name: "Z70", model: "roborock.vacuum.a179", protocolVersion: "L01" },
        camera: { account: { user: "camera-user", secret: "mock-long-camera-secret", key: "mock-long-camera-key", mqttUrl: "ssl://mqtt-eu.roborock.com:8883" }, pin: "1234" },
        home: { deviceId: "robot1", homeId: 1, realApi: "https://api-eu.roborock.com", user: "camera-user", secret: "s", hawkKey: "k", rooms: [] },
      };
    },
    async savePairing() {} };
  const config = { ...loadServiceConfig(), port: 0 };
  const service = await createService({ store, config, robotFactory: () => robot,
    driver: { async configure() { calls.push("configure"); }, async offer() { return { type: "answer", sdp: "v=0" }; }, async frame() {
      calls.push(`frame:${selected}`); if (captureFailure && selected === 1) throw Error("failed"); return jpeg(wrongShape ? 1 : selected);
    }} });
  await service.start(); t.after(() => service.stop());
  const addr = service.server.address(); assert.ok(addr && typeof addr === "object");
  const origin = `http://127.0.0.1:${addr.port}`; config.allowedHosts.push(`127.0.0.1:${addr.port}`); config.publicOrigin = origin;
  return { calls, timeouts, appSwitch(value: number) { selected = value; }, wrongShape() { wrongShape = true; }, noFlip() { noFlip = true; }, setCommon(value: number) { common = value; }, failCapture() { captureFailure = true; }, unknownSelection() { selectionUnknown = true; }, selected: () => selected,
    select: (body: unknown) => fetch(origin + "/api/driver/camera/select", { method: "POST", headers: { Authorization: `Bearer ${token}`, "Content-Type": "application/json" }, body: JSON.stringify(body) }),
    post: (body: unknown) => fetch(origin + "/api/camera/frame", { method: "POST", headers: { Authorization: `Bearer ${token}`, "Content-Type": "application/json" }, body: JSON.stringify(body) }) };
}

test("unmapped camera cannot silently return a front image or start preview", async t => {
  const f = await fixture(t);
  for (const camera of ["left", "right", "back", "depth", "ir", "structured-light"]) {
    assert.equal((await f.post({ camera })).status, 422);
  }
  assert.deepEqual(f.calls, []);
});

test("outside pickup the robot refuses the gripper camera: clear 409, nothing else sent, front kept", async t => {
  const f = await fixture(t);
  const res = await f.select({ camera: "gripper" });
  assert.equal(res.status, 409);
  assert.match(((await res.json()) as { error: string }).error, /pickup mode/);
  assert.ok(!f.calls.some(c => /pick_up|rc_|arm/.test(c)));
  assert.equal(f.selected(), 0);
});

test("gripper switches inside existing preview and stays through default polling", async t => {
  const f = await fixture(t); f.setCommon(49154 | (2 << 10));
  assert.equal((await f.select({camera:'gripper'})).status,200);
  const res = await f.post({});
  assert.equal(res.status, 200);
  assert.equal(res.headers.get("x-roboclaw-camera"), "gripper");
  assert.equal(res.headers.get("x-roboclaw-lens"), "unknown");
  assert.ok(f.calls.indexOf("frame:0") < f.calls.indexOf("app_change_camera:1"));
  assert.ok(f.calls.indexOf("app_change_camera:1") < f.calls.lastIndexOf("frame:1"));
  assert.equal(f.calls.includes("app_change_camera:0"), false);
  assert.equal(f.selected(), 1);
  const poll = await f.post({});
  assert.equal(poll.status, 200);
  assert.equal(poll.headers.get("x-roboclaw-camera"), "gripper");
  assert.equal(f.calls.at(-1), "frame:1");
});

test("gripper capture failure never restores front", async t => {
  const f = await fixture(t); f.setCommon(49154 | (2 << 10)); f.failCapture();
  assert.equal((await f.select({camera:'gripper'})).status,200);
  assert.equal((await f.post({})).status, 502);
  assert.equal(f.selected(), 1);
  assert.equal(f.calls.includes("app_change_camera:0"), false);
});

test("selector reply timeout succeeds when live status flips", async t => {
  const f = await fixture(t); f.setCommon(49154 | (2 << 10)); f.unknownSelection();
  const response = await f.select({camera: "gripper"});
  assert.equal(response.status, 200);
  const selected=await response.json() as {camera:string;outcomes:{method:string;outcome:{dispatch:string;response:string;commandId:string}}[]};
  assert.equal(selected.camera,'gripper');
  const switchOutcome=selected.outcomes.find(o=>o.method==='app_change_camera')!.outcome;
  assert.equal(switchOutcome.dispatch,'unknown'); assert.equal(switchOutcome.response,'transport_error');
  assert.match(switchOutcome.commandId,/^[0-9a-f-]{36}$/);
  assert.equal(f.timeouts.length,1); assert.ok(f.timeouts[0]>0 && f.timeouts[0]<=500);
  assert.equal(f.calls.filter(c => c === "app_change_camera:1").length, 1);
  assert.equal(f.selected(), 1);
});

test("selector timeout without a bit flip returns actionable error", async t => {
  const f = await fixture(t); f.setCommon(49154 | (2 << 10)); f.noFlip();
  const response = await f.select({camera: "gripper"});
  assert.equal(response.status, 409);
  assert.match((await response.json() as {error:string}).error, /not confirmed.*pickup mode.*arm out/);
  assert.equal(f.selected(), 0);
  assert.equal(f.calls.filter(c => c === "app_change_camera:1").length, 1);
});

test("official app selection is reflected in correctly tagged frame dimensions", async t => {
  const f = await fixture(t);
  await f.post({});
  f.appSwitch(1);
  const response = await f.post({});
  assert.equal(response.status, 200);
  assert.equal(response.headers.get("x-roboclaw-camera"), "gripper");
  assert.equal(response.headers.get("x-roboclaw-size-camera"), "gripper");
  assert.equal(response.headers.get("x-roboclaw-image-width"), "1600");
  assert.equal(response.headers.get("x-roboclaw-image-height"), "896");
  assert.equal(f.calls.some(c => c.startsWith("app_change_camera")), false);
});

test("persistent frame/status disagreement fails instead of mislabeling pixels", async t => {
  const f = await fixture(t); f.wrongShape();
  const response = await f.post({});
  assert.equal(response.status, 502);
  assert.match((await response.json() as {error:string}).error, /dimensions disagree/);
});

test("select validates camera and redundant selections do not write", async t => {
  const f = await fixture(t);
  assert.equal((await f.select({})).status, 400);
  assert.equal((await f.select({camera: "left"})).status, 422);
  assert.equal((await f.select({camera: "front"})).status, 200);
  assert.equal(f.calls.some(c => c.startsWith("app_change_camera")), false);
});

test("ordinary front requests do not issue rejected redundant selector commands", async t => {
  const f = await fixture(t);
  assert.equal((await f.post({})).status, 200);
  assert.equal((await f.post({ camera: "front" })).status, 503);
  assert.equal(f.calls.some(c => c.startsWith("app_change_camera")), false);
});

test("invalid camera names are rejected without RPCs", async t => {
  const f = await fixture(t);
  for (const camera of [1, null, "Front", "anything"]) assert.equal((await f.post({ camera })).status, 400);
  assert.deepEqual(f.calls, []);
});
