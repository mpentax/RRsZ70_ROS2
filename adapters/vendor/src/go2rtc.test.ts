import { test } from "node:test";
import { readFileSync } from "node:fs";
import assert from "node:assert/strict";
import { cameraSource, Go2Rtc } from "./go2rtc.ts";

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
  t.mock.method(globalThis, "fetch", async (url: string, options: RequestInit) => {
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
  t.mock.method(globalThis, "fetch", async () => { throw new Error("private-upstream-url"); });
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
  t.mock.method(globalThis, "fetch", async () => Response.json({
    producers: [{ url: "ffmpeg:private-source" }], consumers: null,
  }));
  const result = await new Go2Rtc().streamDiagnostics();
  assert.deepEqual(result.z70_front_jpeg, { available: true, producers: [{}], consumers: [] });
  assert.deepEqual(result.z70_gripper_jpeg, { available: true, producers: [{}], consumers: [] });
  for (const consumers of [42, "private", {}]) {
    t.mock.method(globalThis, "fetch", async () => Response.json({ producers: [{}], consumers }));
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
  t.mock.method(globalThis, "fetch", async (input: URL, options: RequestInit) => {
    assert.equal(input.origin, "http://127.0.0.1:57001");
    assert.equal(options.method, "PATCH");
    assert.equal(options.redirect, "error");
    assert.equal(input.searchParams.get("name"), "z70");
    assert.ok(input.searchParams.get("src")?.startsWith("roborock:"));
    return new Response(null, { status: 200 });
  });
  await new Go2Rtc().configure(account, device, "1234");
  t.mock.method(globalThis, "fetch", async () => { throw new Error("secret-roborock-url"); });
  await assert.rejects(new Go2Rtc().configure(account, device, "1234"),
    (error: unknown) => error instanceof Error && !error.message.includes("secret-roborock-url"));
});

test("invalid broker and pattern fail before contacting the driver", async (t) => {
  t.mock.method(globalThis, "fetch", () => { throw new Error("Unexpected fetch."); });
  assert.throws(() => cameraSource({ ...account, mqttUrl: "ssl://attacker.invalid:8883" }, device, "1234"), /broker/);
  assert.throws(() => cameraSource(account, device, "1230"), /pattern/);
});

test("driver WebRTC exchange returns only SDP or sanitized errors", async (t) => {
  t.mock.method(globalThis, "fetch", async (_url: string, options: RequestInit) => {
    assert.equal(options.method, "POST");
    return Response.json({ type: "answer", sdp: "v=0\r\n" });
  });

  assert.deepEqual(await new Go2Rtc().offer("v=0\r\n"), { type: "answer", sdp: "v=0\r\n" });
  t.mock.method(globalThis, "fetch", async () => new Response("private-key-url can't get device SDP", { status: 500 }));
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
  t.mock.method(globalThis, "fetch", async (url: string) => url.includes("stream.mjpeg")
    ? new Response(new ReadableStream({ start(c) { c.close(); } }), { headers: { "Content-Type": "multipart/x-mixed-replace; boundary=frame" } })
    : new Response(Buffer.from([255, 216, 255, 217]), { headers: { "Content-Type": "image/jpeg" } }));
  assert.deepEqual(await new Go2Rtc().frame(), Buffer.from([255, 216, 255, 217]));
  t.mock.method(globalThis, "fetch", async (url: string) => url.includes("stream.mjpeg")
    ? new Response(new ReadableStream({ start(c) { c.close(); } }), { headers: { "Content-Type": "multipart/x-mixed-replace; boundary=frame" } })
    : new Response("", { headers: { "Content-Type": "image/jpeg" } }));
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

test("configured status never exports a credential-bearing source URL", async (t) => {
  t.mock.method(globalThis, "fetch", async () => Response.json({
    producers: [{ url: "roborock://mqtt.roborock.com:8883?key=private" }],
  }));
  assert.equal(await new Go2Rtc().configured(), true);
  t.mock.method(globalThis, "fetch", async () => Response.json({ producers: [{ url: "{input}" }] }));
  assert.equal(await new Go2Rtc().configured(), false);
});
