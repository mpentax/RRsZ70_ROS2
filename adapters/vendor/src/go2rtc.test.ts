import { test, type TestContext } from "node:test";
import { readFileSync } from "node:fs";
import assert from "node:assert/strict";
import { setTimeout as delay } from "node:timers/promises";
import { CameraSelector, cameraSource, Go2Rtc } from "./go2rtc.ts";
import { startService, type ServiceRobot, type ServiceStore } from "./service.ts";
import { loadServiceConfig } from "./service-config.ts";
import { RpcError } from "./robot.ts";
import { TransportError } from "./rpc-outcome.ts";

const account = { user: "u", secret: "s", key: "k", mqttUrl: "ssl://mqtt-eu.roborock.com:8883" };
const device = { duid: "d", localKey: "0123456789abcdef", name: "Z70", model: "roborock.vacuum.a179", protocolVersion: "1.0" };

test("JPEG transcoders select source dimensions before scaling", () => {
  const config = readFileSync(new URL("../../../deploy/go2rtc.yaml", import.meta.url), "utf8");
  for (const [camera, height] of [["front", 1200], ["gripper", 896]] as const) {
    assert.ok(config.includes(`z70_${camera}_jpeg: "ffmpeg:http://127.0.0.1:57001/api/stream.mp4?src=z70&video#video=mjpeg#raw=-vf select=eq(iw\\\\,1600)*eq(ih\\\\,${height}),scale=1600:${height} -fps_mode vfr"`));
  }
});

test("acquisition diagnostics retain the failing boundary but never upstream strings", async (t) => {
  const evidence = { available: true, attempt: 2, elapsedMs: 5123, stage: "get_answer", failure: "eof", vendorCode: 0, vendorReason: "none",
    mqtt: "ok", pin: "ok", turn: "ok", pcState: "connecting", iceState: "checking", dtlsState: "new",
    localCandidateType: "unknown", remoteCandidateType: "unknown", candidateProtocol: "unknown",
    videoTracks: 0, videoPackets: 0, videoBytes: 0, readerExit: "none" };
  let body: unknown = { ...evidence, source: "roborock://private?pin=1234", error: "private", sdp: "private" };
  const f = t.mock.method(globalThis, "fetch", async (url: string, options: RequestInit) => {
    assert.equal(url, "http://127.0.0.1:57001/api/roborock/diagnostic");
    assert.equal(options.method, undefined);
    assert.equal(options.redirect, "error");
    return Response.json(body);
  });
  assert.deepEqual(await new Go2Rtc().acquisitionDiagnostics(), evidence);
  for (const invalid of [
    { ...evidence, stage: "private" }, { ...evidence, attempt: -1 }, { ...evidence, videoPackets: 1.5 },
    { ...evidence, available: "true" }, { ...evidence, pin: undefined }, { ...evidence, videoBytes: Number.MAX_SAFE_INTEGER + 1 }, null,
  ]) {
    body = invalid;
    assert.deepEqual(await new Go2Rtc().acquisitionDiagnostics(), { available: false });
  }
  f.mock.mockImplementation(async () => { throw new Error("private-upstream-url"); });
  assert.deepEqual(await new Go2Rtc().acquisitionDiagnostics(), { available: false });
});

test("stream diagnostics expose only runtime counters for original and decoded feeds", async (t) => {
  const paths: string[] = [];
  t.mock.method(globalThis, "fetch", async (url: string) => {
    paths.push(url);
    return Response.json({ producers: [{ url: "roborock://private?pin=1234", bytes_recv: 123,
      protocol: "udp", format_name: "webrtc", remote_addr: "private",
      receivers: [{ bytes: 120, packets: 3, codec: { codec_name: "H264", secret: "private" } }] }],
      consumers: [{ bytes_send: 77, senders: [{ bytes: 70, packets: 2 }] }] });
  });
  const result = await new Go2Rtc().streamDiagnostics();
  assert.deepEqual(result.z70, { available: true, producers: [{ bytes_recv: 123,
    receivers: [{ bytes: 120, packets: 3 }] }], consumers: [{ bytes_send: 77, senders: [{ bytes: 70, packets: 2 }] }] });
  assert.deepEqual(result.z70_front_jpeg, result.z70);
  assert.deepEqual(result.z70_gripper_jpeg, result.z70);
  assert.equal(JSON.stringify(result).includes("private"), false);
  assert.deepEqual(paths, [
    "http://127.0.0.1:57001/api/streams?src=z70",
    "http://127.0.0.1:57001/api/streams?src=z70_front_jpeg",
    "http://127.0.0.1:57001/api/streams?src=z70_gripper_jpeg",
  ]);
});

test("stream diagnostics accept the upstream null consumer list before first frame", async (t) => {
  let consumers: unknown = null;
  t.mock.method(globalThis, "fetch", async () => Response.json({
    producers: [{ url: "ffmpeg:private-source" }], consumers,
  }));
  const result = await new Go2Rtc().streamDiagnostics();
  assert.deepEqual(result.z70_front_jpeg, { available: true, producers: [{}], consumers: [] });
  assert.deepEqual(result.z70_gripper_jpeg, { available: true, producers: [{}], consumers: [] });
  for (const invalid of [42, "private", {}]) {
    consumers = invalid;
    assert.deepEqual((await new Go2Rtc().streamDiagnostics()).z70_front_jpeg, { available: false });
  }
});

test("official driver receives correctly escaped Roborock credentials and unhashed pattern", () => {
  const url = new URL(cameraSource({ ...account, secret: "salt+value" }, device, "1234"));
  assert.equal(url.protocol, "roborock:");
  assert.equal(url.host, "mqtt-eu.roborock.com:8883");
  assert.equal(url.searchParams.get("s"), "salt+value");
  assert.equal(url.searchParams.get("pin"), "1234");
  assert.equal(url.searchParams.get("key"), device.localKey);
});

test("driver configuration is memory-only PATCH and never exposes upstream errors", async (t) => {
  const f = t.mock.method(globalThis, "fetch", async (input: URL, options: RequestInit) => {
    assert.equal(input.origin, "http://127.0.0.1:57001");
    assert.equal(options.method, "PATCH");
    assert.equal(options.redirect, "error");
    assert.equal(input.searchParams.get("name"), "z70");
    assert.ok(input.searchParams.get("src")?.startsWith("roborock:"));
    return new Response(null, { status: 200 });
  });
  await new Go2Rtc().configure(account, device, "1234");
  f.mock.mockImplementation(async () => { throw new Error("secret-roborock-url"); });
  await assert.rejects(new Go2Rtc().configure(account, device, "1234"),
    (error: unknown) => error instanceof Error && !error.message.includes("secret-roborock-url"));
});

test("invalid broker and pattern fail before contacting the driver", async (t) => {
  t.mock.method(globalThis, "fetch", () => { throw new Error("Unexpected fetch."); });
  assert.throws(() => cameraSource({ ...account, mqttUrl: "ssl://attacker.invalid:8883" }, device, "1234"), /broker/);
  assert.throws(() => cameraSource(account, device, "1230"), /pattern/);
});

test("driver WebRTC exchange returns only SDP or sanitized errors", async (t) => {
  const f = t.mock.method(globalThis, "fetch", async (_url: string, options: RequestInit) => {
    assert.equal(options.method, "POST");
    return Response.json({ type: "answer", sdp: "v=0\r\n" });
  });

  assert.deepEqual(await new Go2Rtc().offer("v=0\r\n"), { type: "answer", sdp: "v=0\r\n" });
  f.mock.mockImplementation(async () => new Response("private-key-url can't get device SDP", { status: 500 }));
  await assert.rejects(new Go2Rtc().offer("v=0\r\n"), (error: unknown) =>
    error instanceof Error && error.message.includes("WebRTC answer") && !error.message.includes("private-key-url"));
});

test("failed camera warmup does not silently reopen preview through the snapshot path", async (t) => {
  const paths: string[] = [];
  t.mock.method(globalThis, "fetch", async (url: string) => {
    paths.push(url);
    // Startup returned no image. A second path would improperly reopen the source.
    if (url.includes("stream.mjpeg")) return new Response(null, { status: 200 });
    return new Response(Buffer.from([255, 216, 255, 217]), { headers: { "Content-Type": "image/jpeg" } });
  });
  await assert.rejects(new Go2Rtc().frame(), /camera image/);
  assert.deepEqual(paths, [
    "http://127.0.0.1:57001/api/stream.mjpeg?src=z70_front_jpeg",
  ]);
});

test("snapshot requires nonempty complete JPEG, not upstream empty HTTP 200", async (t) => {
  let empty = false;
  t.mock.method(globalThis, "fetch", async (url: string) => url.includes("stream.mjpeg")
    ? new Response(new ReadableStream({ start(c) { c.close(); } }), { headers: { "Content-Type": "multipart/x-mixed-replace; boundary=frame" } })
    : new Response(empty ? "" : Buffer.from([255, 216, 255, 217]), { headers: { "Content-Type": "image/jpeg" } }));
  assert.deepEqual(await new Go2Rtc().frame(), Buffer.from([255, 216, 255, 217]));
  empty = true;
  await assert.rejects(new Go2Rtc().frame(), /complete JPEG/);
});

test("consecutive snapshots share pre-warmed dual receivers across camera switches and release after idle", async (t) => {
  t.mock.timers.enable({ apis: ["setTimeout"] });
  let starts = 0, closed = 0, warm = 0;
  const requestedFrames: string[] = [];
  t.mock.method(globalThis, "fetch", async (input: string, options: RequestInit) => {
    if (input.includes("stream.mjpeg")) {
      starts++; warm++;
      return new Response(new ReadableStream({
        start(controller) { options.signal!.addEventListener("abort", () => { warm--; closed++; controller.close(); }); },
      }), { headers: { "Content-Type": "multipart/x-mixed-replace; boundary=frame" } });
    }
    requestedFrames.push(input);
    assert.equal(warm, 2, "both front and gripper receivers must stay warm across switches");
    return new Response(Buffer.from([255, 216, 255, 217]), { headers: { "Content-Type": "image/jpeg" } });
  });
  const driver = new Go2Rtc();
  await Promise.all([driver.frame("front"), driver.frame("gripper")]);
  assert.equal(starts, 2);
  assert.deepEqual(requestedFrames, [
    "http://127.0.0.1:57001/api/frame.jpeg?src=z70_front_jpeg",
    "http://127.0.0.1:57001/api/frame.jpeg?src=z70_gripper_jpeg",
  ]);
  t.mock.timers.tick(9_000);
  await driver.frame("front");
  t.mock.timers.tick(9_000);
  assert.equal(closed, 0);
  t.mock.timers.tick(1_000);
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(closed, 2);
  await driver.frame("gripper");
  assert.equal(starts, 4);
  t.mock.timers.tick(10_000);
});

test('suspension aborts a stalled warm open and prevents late snapshots until deliberate resume', async t => {
  let opening!:()=>void, finish!:()=>void; let snapshots=0, aborted=0, unloads=0;
  const entered=new Promise<void>(r=>{opening=r;});
  const pending=new Promise<void>(r=>{finish=r;});
  let first=true;
  t.mock.method(globalThis,'fetch',async(input:string|URL,options:RequestInit)=>{
    const url=String(input);
    if(options.method==='DELETE'){unloads++;return new Response(null,{status:200});}
    if(url.includes('stream.mjpeg')){
      if(first){first=false;opening();options.signal!.addEventListener('abort',()=>{aborted++;});await pending;}
      return new Response(new ReadableStream({start(c){if(options.signal!.aborted)c.close();else options.signal!.addEventListener('abort',()=>c.close());}}),{headers:{'Content-Type':'multipart/x-mixed-replace; boundary=frame'}});
    }
    snapshots++;return new Response(Buffer.from([255,216,255,217]),{headers:{'Content-Type':'image/jpeg'}});
  });
  const driver=new Go2Rtc();t.after(()=>driver.close());
  const openingFrame=driver.frame();const rejected=assert.rejects(openingFrame);
  await entered;
  await driver.setEnabled(false);
  assert.equal(aborted,1);assert.equal(unloads,1);
  await driver.setEnabled(true);
  finish();await rejected;
  assert.equal(snapshots,0,'revoked open must not start frame.jpeg after resume');
  await driver.frame();assert.equal(snapshots,1);
  await driver.setEnabled(false);
  await assert.rejects(driver.frame());assert.equal(snapshots,1);
});

test("camera source audit compares account and device fields without returning credentials", async (t) => {
  const source = cameraSource(account, device, "1234");
  let url = source;
  t.mock.method(globalThis, "fetch", async () => Response.json({ producers: [{ url }], consumers: [] }));
  const driver = new Go2Rtc();
  assert.deepEqual(await driver.configurationMatches(account, device, "1234"), {
    available: true, brokerMatches: true, accountMatches: true, deviceMatches: true, pinMatches: true,
  });
  url = cameraSource({ ...account, secret: "different-private-secret" }, device, "1234");
  const changed = await driver.configurationMatches(account, device, "1234");
  assert.equal(changed.accountMatches, false);
  assert.equal(changed.deviceMatches, true);
  assert.equal(JSON.stringify(changed).includes("private"), false);
});

const jpeg = (selected: number) => Buffer.from([255,216,255,192,0,11,8,...(selected ? [3,128] : [4,176]),6,64,1,1,17,0,255,218,0,2,255,217]);

async function selectorFixture(t: TestContext) {
  const calls: string[] = [], timeouts: number[] = [];
  let selected = 0, common = 49154, captureFailure = false, noFlip = false, selectionUnknown = false, wrongShape = false;
  const token = "c".repeat(43);
  const authHeader = "Bea" + "rer " + token;
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
  const service = await startService({ store, config: { ...loadServiceConfig(), port: 0 }, robotFactory: () => robot,
    driver: { async configure() { calls.push("configure"); }, async offer() { return { type: "answer", sdp: "v=0" }; }, async frame() {
      calls.push(`frame:${selected}`); if (captureFailure && selected === 1) throw Error("failed"); return jpeg(wrongShape ? 1 : selected);
    }} });
  t.after(() => service.stop());
  const origin = service.origin;
  return { calls, timeouts, appSwitch(value: number) { selected = value; }, wrongShape() { wrongShape = true; }, noFlip() { noFlip = true; }, setCommon(value: number) { common = value; }, failCapture() { captureFailure = true; }, unknownSelection() { selectionUnknown = true; }, selected: () => selected,
    select: (body: unknown) => fetch(origin + "/api/driver/camera/select", { method: "POST", headers: { Authorization: authHeader, "Content-Type": "application/json" }, body: JSON.stringify(body) }),
    post: (body: unknown) => fetch(origin + "/api/camera/frame", { method: "POST", headers: { Authorization: authHeader, "Content-Type": "application/json" }, body: JSON.stringify(body) }) };
}

test("unmapped camera cannot silently return a front image or start preview", async t => {
  const f = await selectorFixture(t);
  for (const camera of ["left", "right", "back", "depth", "ir", "structured-light"]) {
    assert.equal((await f.post({ camera })).status, 422);
  }
  assert.deepEqual(f.calls, []);
});

test("outside pickup the robot refuses the gripper camera: clear 409, nothing else sent, front kept", async t => {
  const f = await selectorFixture(t);
  const res = await f.select({ camera: "gripper" });
  assert.equal(res.status, 409);
  assert.match(((await res.json()) as { error: string }).error, /pickup mode/);
  assert.ok(!f.calls.some(c => /pick_up|rc_|arm/.test(c)));
  assert.equal(f.selected(), 0);
});

test("gripper switches inside existing preview and stays through default polling", async t => {
  const f = await selectorFixture(t); f.setCommon(49154 | (2 << 10));
  assert.equal((await f.select({camera:"gripper"})).status,200);
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
  const f = await selectorFixture(t); f.setCommon(49154 | (2 << 10)); f.failCapture();
  assert.equal((await f.select({camera:"gripper"})).status,200);
  assert.equal((await f.post({})).status, 502);
  assert.equal(f.selected(), 1);
  assert.equal(f.calls.includes("app_change_camera:0"), false);
});

test("selector reply timeout succeeds when live status flips", async t => {
  const f = await selectorFixture(t); f.setCommon(49154 | (2 << 10)); f.unknownSelection();
  const response = await f.select({camera: "gripper"});
  assert.equal(response.status, 200);
  const selected=await response.json() as {camera:string;outcomes:{method:string;outcome:{dispatch:string;response:string;commandId:string}}[]};
  assert.equal(selected.camera,"gripper");
  const switchOutcome=selected.outcomes.find(o=>o.method==="app_change_camera")!.outcome;
  assert.equal(switchOutcome.dispatch,"unknown"); assert.equal(switchOutcome.response,"transport_error");
  assert.match(switchOutcome.commandId,/^[0-9a-f-]{36}$/);
  assert.equal(f.timeouts.length,1); assert.ok(f.timeouts[0]>0 && f.timeouts[0]<=500);
  assert.equal(f.calls.filter(c => c === "app_change_camera:1").length, 1);
  assert.equal(f.selected(), 1);
});

test("selector timeout without a bit flip returns actionable error", async t => {
  const f = await selectorFixture(t); f.setCommon(49154 | (2 << 10)); f.noFlip();
  const response = await f.select({camera: "gripper"});
  assert.equal(response.status, 409);
  assert.match((await response.json() as {error:string}).error, /not confirmed.*pickup mode.*arm out/);
  assert.equal(f.selected(), 0);
  assert.equal(f.calls.filter(c => c === "app_change_camera:1").length, 1);
});

test("official app selection is reflected in correctly tagged frame dimensions", async t => {
  const f = await selectorFixture(t);
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
  const f = await selectorFixture(t); f.wrongShape();
  const response = await f.post({});
  assert.equal(response.status, 502);
  assert.match((await response.json() as {error:string}).error, /dimensions disagree/);
});

test("select validates camera and redundant selections do not write", async t => {
  const f = await selectorFixture(t);
  assert.equal((await f.select({})).status, 400);
  assert.equal((await f.select({camera: "left"})).status, 422);
  assert.equal((await f.select({camera: "front"})).status, 200);
  assert.equal(f.calls.some(c => c.startsWith("app_change_camera")), false);
});

test("ordinary front requests do not issue rejected redundant selector commands", async t => {
  const f = await selectorFixture(t);
  assert.equal((await f.post({})).status, 200);
  assert.equal((await f.post({ camera: "front" })).status, 503);
  assert.equal(f.calls.some(c => c.startsWith("app_change_camera")), false);
});

test("invalid camera names are rejected without RPCs", async t => {
  const f = await selectorFixture(t);
  for (const camera of [1, null, "Front", "anything"]) assert.equal((await f.post({ camera })).status, 400);
  assert.deepEqual(f.calls, []);
});

test("selection polls for a delayed bit flip after missing acknowledgement", async () => {
  let selected = 0, readsAfterWrite = 0;
  const writes: unknown[] = [];
  const selector = new CameraSelector(async (method, params) => {
    if (method === "app_change_camera") {
      writes.push(params); throw new TransportError("RPC timed out", "unknown");
    }
    if (writes.length && ++readsAfterWrite >= 3) selected = 1;
    return [{ camera_status: selected << 15 }];
  });
  assert.equal(await selector.select("gripper", async () => jpeg(0)), "gripper");
  assert.deepEqual(writes, [{ flag: 1 }]);
  assert.ok(readsAfterWrite >= 3);
});

test("default reads queued behind a switch do not flip it back", async () => {
  let selected = 0;
  const writes: unknown[] = [];
  const selector = new CameraSelector(async (method, params) => {
    if (method === "app_change_camera") {
      writes.push(params); selected = (params as { flag: number }).flag;
      throw new TransportError("RPC timed out", "unknown");
    }
    return [selected << 15];
  });
  const capture = async () => jpeg(selected);
  const results = await Promise.all([selector.run("gripper", capture), selector.run(undefined, capture), selector.run(undefined, capture)]);
  assert.deepEqual(results.map(result => result.camera), ["gripper", "gripper", "gripper"]);
  assert.deepEqual(writes, [{ flag: 1 }]);
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
  const unknownShape = async () => Buffer.from([255, 216, 255, 192, 0, 11, 8, 0, 12, 0, 16, 1, 1, 17, 0, 255, 218, 0, 2, 255, 217]);
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
    if (method === "app_change_camera") selected = (params as { flag: number }).flag;
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
  const authHeader = "Bea" + "rer " + token;
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
  const service = await startService({
    store, config: { ...loadServiceConfig(), port: 0 }, robotFactory: () => robot,
    driver: {
      async configure() {},
      async offer() { return { type: "answer" as const, sdp: "v=0" }; },
      async frame(camera = "front") { framesRequested.push(camera); return jpeg(camera === "gripper" ? 1 : 0); },
      close() { closed++; },
    },
  });
  t.after(() => service.stop());
  const origin = service.origin;
  const headers = { Authorization: authHeader, "Content-Type": "application/json" };

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
