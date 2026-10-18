import assert from "node:assert/strict";
import { test } from "node:test";
import { Socket } from "node:net";
import { spawnSync } from "node:child_process";
import { localProtocolCodec } from "./local.ts";
import { RpcError, RobotClient } from "./robot.ts";
import { probeMapList, summarizeMapList, MAP_LIST_METHOD } from "./map-list-probe.ts";
import type { SavedDevice } from "./credentials.ts";

const KEY = "b8Hj5mFk3QzT7rLp";
const device: SavedDevice = { host: "127.0.0.1", localKey: KEY, duid: "fixture",
  model: "roborock.vacuum.a179", name: "fixture", protocolVersion: "1.0", localProtocol: "L01" };

test("bounded redacted summary never echoes map IDs, geometry, or unknown fields", () => {
  const secret = { mapFlag: 987654, mapName: "PRIVATE-FLOOR", geometry: "PRIVATE-GEOMETRY" };
  assert.deepEqual(summarizeMapList({ map_info: { map: [secret, { mapFlag: 22, mapName: "SECRET" }] } }), {
    method: MAP_LIST_METHOD, transport: "local-L01", supported: true,
    response: "structured-list", itemCount: 2, normalizedFlagCount: 2,
  });
  assert.equal(summarizeMapList({ map_info: { map: [] } }).response, "empty-list");
  for (const raw of [null, ["ok"], { result: "ok" }, { mapList: [] },
    { map_info: { map: Array(65).fill({ mapFlag: 1 }) } }, [null],
    { map_info: { map: [{ mapFlag: "12" }] } },
    { map_info: { map: [{ mapFlag: 2 }, { mapFlag: 2 }] } }]) {
    const summary = summarizeMapList(raw);
    assert.equal(summary.supported, false);
    assert.equal(summary.response, "unrecognized-shape");
    assert.ok(!JSON.stringify(summary).includes("PRIVATE"));
  }
  assert.ok(!JSON.stringify(summarizeMapList({ map_info: { map: [secret] } })).includes("987654"));
  assert.deepEqual(summarizeMapList([{ map_info: { map: [secret] } }]).response, "structured-list");
});

test("opt-in preflight rejects a different device/protocol before transport", async () => {
  let attempts = 0;
  const fake = () => { attempts++; throw Error("must not connect"); };
  assert.equal((await probeMapList({ ...device, model: "other" }, fake)).response, "preflight-failed");
  assert.equal((await probeMapList({ ...device, localProtocol: "1.0" }, fake)).response, "preflight-failed");
  assert.equal(attempts, 0);
});

test("one explicitly correlated L01 request, other ID and unsolicited frames ignored; close on result", async (t) => {
  let requests = 0;
  let acknowledgments = 0;
  const nonce = 432123;
  let socket: Socket | undefined;
  const frame = (type: number, sequence: number, random: number, timestamp: number, extra = Buffer.alloc(0)) => {
    const head = Buffer.alloc(17);
    head.write("L01"); head.writeUInt32BE(sequence, 3); head.writeUInt32BE(random, 7);
    head.writeUInt32BE(timestamp, 11); head.writeUInt16BE(type, 15);
    const body = Buffer.concat([head, extra]); const length = Buffer.alloc(4);
    length.writeUInt32BE(body.length); return Buffer.concat([length, body]);
  };
  const sendReply = (id: number, result: unknown, sequence: number, connectNonce: number) => {
    const timestamp = 1753606905, random = 304251;
    const payload = localProtocolCodec.encryptPayload(Buffer.from(JSON.stringify({ dps: {
      "102": JSON.stringify({ id, result }),
    }, t: timestamp })), { version: "L01", localKey: KEY, sequence, timestamp, random,
      connectNonce, ackNonce: nonce });
    const length = Buffer.alloc(2); length.writeUInt16BE(payload.length);
    const head = frame(4, sequence, random, timestamp).subarray(4);
    const body = Buffer.concat([head, length, payload]);
    const crc = Buffer.alloc(4); crc.writeUInt32BE(localProtocolCodec.crc32(body));
    const size = Buffer.alloc(4); size.writeUInt32BE(body.length + crc.length);
    socket!.emit("data", Buffer.concat([size, body, crc]));
  };
  t.mock.method(Socket.prototype, "connect", function(this: Socket, address: unknown) {
    socket = this;
    assert.deepEqual(address, { host: "127.0.0.1", port: 58867 });
    t.mock.method(this, "write", writeFrame);
    queueMicrotask(() => this.emit("connect"));
    return this;
  });
  const writeFrame = (bytes: Buffer, callback?: () => void) => {
    const body = bytes.subarray(4), type = body.readUInt16BE(15);
    if (type === 0) {
      (socket as Socket & { fixtureNonce?: number }).fixtureNonce = body.readUInt32BE(7);
      const ack = Buffer.alloc(4);
      queueMicrotask(() => { if (!socket!.destroyed) socket!.emit("data", frame(1, 0, nonce, 0, ack)); callback?.(); });
    } else if (type === 4) {
      requests++;
      const connectNonce = (socket as Socket & { fixtureNonce?: number }).fixtureNonce ?? 0;
      const sequence = body.readUInt32BE(3), random = body.readUInt32BE(7), timestamp = body.readUInt32BE(11);
      const payload = localProtocolCodec.decryptPayload(body.subarray(19, -4), {
        version: "L01", localKey: KEY, sequence, random, timestamp, connectNonce, ackNonce: nonce,
      });
      const rpc = JSON.parse(JSON.parse(payload.toString()).dps["101"]);
      assert.equal(rpc.method, MAP_LIST_METHOD);
      assert.deepEqual(rpc.params, []);
      assert.equal(rpc.security, undefined);
      queueMicrotask(() => {
        sendReply(rpc.id + 1, [{ mapFlag: 99999 }], 71, connectNonce);
        sendReply(rpc.id, { map_info: { map: [{ mapFlag: 1234, geometry: "PRIVATE" }] } }, 72, connectNonce);
        callback?.();
      });
    } else if (type === 5) { acknowledgments++; callback?.(); }
    else callback?.();
    return true;
  };
  // Cloud inventory may advertise 1.0 even though the verified local wire path is L01.
  const result = await probeMapList({ ...device, protocolVersion: "1.0" }, (options) => new RobotClient(options));
  assert.deepEqual(result, { method: MAP_LIST_METHOD, transport: "local-L01", supported: true,
    response: "structured-list", itemCount: 1, normalizedFlagCount: 1 });
  assert.equal(requests, 1);
  assert.equal(acknowledgments, 2);
  assert.equal(socket?.destroyed, true);
});

test("probe reports sanitized errors and always closes", async () => {
  let closed = 0, requested = 0;
  const fake = (result: unknown, failure?: Error) => ({
    connect: async () => { if (failure?.message === "connect") throw failure; },
    request: async (method: string, params: unknown) => { requested++; assert.equal(method, MAP_LIST_METHOD);
      assert.deepEqual(params, []); if (failure && failure.message !== "connect") throw failure; return result; },
    close: () => { closed++; },
  });
  assert.equal((await probeMapList(device, () => fake({ map_info: { map: [{ mapFlag: 3 }] } }))).response, "structured-list");
  assert.equal((await probeMapList(device, () => fake([], new RpcError(42)))).response, "request-rejected");
  assert.equal((await probeMapList(device, () => fake([], new Error("SECRET")))).response, "no-correlated-response");
  assert.equal((await probeMapList(device, () => fake([], new Error("connect")))).response, "no-correlated-response");
  assert.equal(requested, 3);
  assert.equal(closed, 4);
});

test("CLI always fails closed before loading secrets or making a robot request", () => {
  for (const args of [[], ["--one-read-only-request"], ["--unexpected"]]) {
    const child = spawnSync(process.execPath, [new URL("./map-list-probe.ts", import.meta.url).pathname, ...args], {
      encoding: "utf8", timeout: 5000,
      env: { ...process.env, ROBOCLAW_STATE_DIR: "/nonexistent/map-list-preflight-must-not-read" },
    });
    assert.equal(child.error, undefined);
    assert.equal(child.status, 1);
    assert.equal(child.stderr, "");
    assert.deepEqual(JSON.parse(child.stdout), { method: MAP_LIST_METHOD, transport: "local-L01",
      supported: false, response: "preflight-failed", reason: "live-safety-preflight-unavailable" });
  }
});
