import assert from "node:assert/strict";
import { test } from "node:test";
import { localProtocolCodec } from "../adapters/vendor/src/local.ts";
import { inspectCloudMapListFrame } from "./legacy-ts/cloud-map-list-v1-research.ts";
import { classifySplitMapList, MAX_LIST_FRAME_BYTES } from "./legacy-ts/map-list-split-correlation-research.ts";

const key = "0123456789abcdef", id = 712345, time = 1753606905;
function frame(inner: unknown, options: { protocol?: number; key?: string; raw?: Buffer } = {}): Buffer {
  const plain = options.raw ?? Buffer.from(JSON.stringify({ dps: { "102": JSON.stringify(inner) }, t: time }));
  const encrypted = localProtocolCodec.encryptPayload(plain, {
    version: "1.0", localKey: options.key ?? key, timestamp: time,
    sequence: 4, random: 99, connectNonce: 0, ackNonce: 0,
  });
  const result = Buffer.alloc(23 + encrypted.length);
  result.write("1.0"); result.writeUInt32BE(4, 3); result.writeUInt32BE(99, 7);
  result.writeUInt32BE(time, 11); result.writeUInt16BE(options.protocol ?? 102, 15);
  result.writeUInt16BE(encrypted.length, 17); encrypted.copy(result, 19);
  result.writeUInt32BE(localProtocolCodec.crc32(result.subarray(0, -4)), result.length - 4);
  return result;
}
const inspect = (bytes: Buffer, expected = id) => inspectCloudMapListFrame(bytes, key, expected);
const list = { map_info: { map: [{ mapFlag: 77, name: "PRIVATE_FLOOR_SENTINEL" }] } };

test("bound V1 protocol-102 JSON only after CRC and local-key ECB; list is categorical downstream", () => {
  const bytes = frame({ id, result: list });
  assert.equal(bytes.subarray(19).includes(Buffer.from("PRIVATE_FLOOR_SENTINEL")), false);
  const result = inspect(bytes);
  assert.equal(result.kind, "bound");
  if (result.kind !== "bound") return;
  const assessment = classifySplitMapList({ requestId: id, incomingTopic: "test/selected", sentAt: 100,
    setupReady: true, observations: [
      { source: "lan", at: 101, correlated: true, reply: { requestId: id, result: ["ok"] } },
      { source: "cloud", at: 102, topic: "test/selected", protocol: 102, frameBytes: bytes.length,
        frameValid: true, outerDecrypted: true, reply: result.reply },
    ] });
  assert.equal(assessment.outcome, "LAN_ACK + cloud_list");
  assert.equal(assessment.itemCount, 1);
  assert.ok(!JSON.stringify(assessment).includes("PRIVATE_FLOOR_SENTINEL"));
  assert.ok(!JSON.stringify(assessment).includes("77"));
});

test("request binding, ACK and RPC error are distinct; reply IDs cannot be inferred from frame sequence", () => {
  assert.deepEqual(inspect(frame({ id: id + 1, result: list })), { kind: "unrelated" });
  assert.deepEqual(inspect(frame({ id, result: list }), id + 1), { kind: "unrelated" });
  assert.deepEqual(inspect(frame({ id, result: ["ok"] })),
    { kind: "bound", reply: { requestId: id, result: ["ok"] } });
  assert.deepEqual(inspect(frame({ id, error: { code: -10001, secret: "PRIVATE" } })),
    { kind: "bound", reply: { requestId: id, errorCode: -10001 } });
  assert.deepEqual(inspect(frame({ id, error: "PRIVATE" })),
    { kind: "bound", reply: { requestId: id, errorCode: -1 } });
  assert.deepEqual(inspect(frame({ result: list })), { kind: "opaque" });
});

test("framing, type, encrypted length, key and UTF-8/JSON failures fail closed", () => {
  const good = frame({ id, result: list });
  const crc = Buffer.from(good); crc[20] ^= 1;
  assert.deepEqual(inspect(crc), { kind: "malformed" });
  const length = Buffer.from(good); length.writeUInt16BE(16, 17);
  assert.deepEqual(inspect(length), { kind: "malformed" });
  assert.deepEqual(inspect(Buffer.alloc(22)), { kind: "malformed" });
  assert.deepEqual(inspect(Buffer.alloc(MAX_LIST_FRAME_BYTES + 1)), { kind: "malformed" });
  assert.deepEqual(inspect(frame({ id, result: list }, { protocol: 301 })), { kind: "unrelated" });
  assert.deepEqual(inspect(frame({ id, result: list }, { key: "fedcba9876543210" })), { kind: "opaque" });
  assert.deepEqual(inspect(frame(null, { raw: Buffer.from([0xff]) })), { kind: "opaque" });
  assert.deepEqual(inspect(frame(null, { raw: Buffer.from("not-json") })), { kind: "opaque" });
  const highBitVersion = Buffer.from(good);
  highBitVersion[0] |= 0x80; // Node's 'ascii' decode masks this bit; V1 header must be exact bytes.
  highBitVersion.writeUInt32BE(localProtocolCodec.crc32(highBitVersion.subarray(0, -4)), highBitVersion.length - 4);
  assert.deepEqual(inspect(highBitVersion), { kind: "malformed" });
  assert.deepEqual(inspect(frame(null, { raw: Buffer.from(JSON.stringify({ dps: { "101": "{}" } })) })), { kind: "opaque" });
});

test("bounded nested or excessive JSON never becomes a bound reply", () => {
  let nested: unknown = 1;
  for (let i = 0; i < 25; i++) nested = [nested];
  assert.deepEqual(inspect(frame({ id, result: nested })), { kind: "opaque" });
  const crowd = Array.from({ length: 4100 }, (_, i) => i);
  assert.deepEqual(inspect(frame({ id, result: crowd })), { kind: "opaque" });
  const oversized = Buffer.alloc(MAX_LIST_FRAME_BYTES + 1);
  assert.deepEqual(inspect(oversized), { kind: "malformed" });
  assert.deepEqual(inspect(frame({ id, result: list }), -1), { kind: "malformed" });
});
