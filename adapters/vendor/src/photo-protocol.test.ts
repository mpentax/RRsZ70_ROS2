import assert from "node:assert/strict";
import { createCipheriv, createHash } from "node:crypto";
import { test } from "node:test";
import { encodeV1MqttFrame } from "./cloud-control.ts";
import { localProtocolCodec } from "./local.ts";
import { encodeMapResponse } from "./map-raster-fixture.ts";
import {
  createBlobSecurity,
  decodeCloudMapResponse,
  decodeMapResponse,
  inspectMapFrame,
  MAX_CLOUD_MAP_FRAME_BYTES,
} from "./photo-protocol.ts";

const LOCAL_KEY = "0123456789abcdef";
function response(data: Buffer, security = createBlobSecurity("map-test-key"), requestId = 20009): Buffer {
  return encodeV1MqttFrame(LOCAL_KEY, 301, encodeMapResponse(data, security, requestId), 1753606905);
}

test("blob security uses account-key hash bytes 8..13 and a fresh 16-byte nonce", () => {
  const first = createBlobSecurity("synthetic-account-key");
  const second = createBlobSecurity("synthetic-account-key");
  assert.equal(first.endpoint, createHash("md5").update("synthetic-account-key")
    .digest().subarray(8, 14).toString("base64"));
  assert.equal(first.nonce.length, 16);
  assert.notDeepEqual(first.nonce, second.nonce);
  assert.throws(() => createBlobSecurity(""), /account key/);
});

test("map envelope checks endpoint and request, then decrypts CBC and decompresses", () => {
  const security = createBlobSecurity("synthetic-key");
  const data = Buffer.from("synthetic-decompressed-map");
  const payload = encodeMapResponse(data, security);
  assert.deepEqual(decodeMapResponse(301, payload, security, 20009), { requestId: 20009, data });
  assert.equal(decodeMapResponse(102, payload, security), undefined);
  assert.equal(decodeMapResponse(301, payload, security, 3), undefined);
  assert.equal(decodeMapResponse(301, payload, createBlobSecurity("different-key")), undefined);
  assert.throws(() => decodeMapResponse(301, payload.subarray(0, -1), security), /frame size/);
  const damaged = Buffer.from(payload);
  damaged.fill(0, 24);
  assert.throws(() => decodeMapResponse(301, damaged, security), /decrypt|compressed/);
  assert.throws(() => decodeMapResponse(301, payload, { ...security, nonce: Buffer.alloc(1) }), /security/);
});

test("cloud map probe accepts only a CRC-verified protocol-301 response bound to its nonce and request", () => {
  const security = createBlobSecurity("map-test-key");
  const data = Buffer.from("private map bytes");
  const frame = response(data, security);
  assert.deepEqual(decodeCloudMapResponse(frame, LOCAL_KEY, security, 20009), { requestId: 20009, data });
  assert.equal(decodeCloudMapResponse(frame, LOCAL_KEY, security, 20010), undefined);
  assert.equal(decodeCloudMapResponse(frame, LOCAL_KEY, createBlobSecurity("another-key"), 20009), undefined);
  frame[20] ^= 1;
  assert.throws(() => decodeCloudMapResponse(frame, LOCAL_KEY, security, 20009), /Invalid cloud map/);
});

test("cloud map probe refuses malformed and oversize outer frames before parsing", () => {
  const security = createBlobSecurity("map-test-key");
  assert.throws(() => decodeCloudMapResponse(Buffer.alloc(22), LOCAL_KEY, security, 1), /Invalid cloud map/);
  assert.throws(() => decodeCloudMapResponse(Buffer.alloc(MAX_CLOUD_MAP_FRAME_BYTES + 1), LOCAL_KEY, security, 1), /Invalid cloud map/);
});

test("map-frame inspection never calls an opaque 301 frame correlated", () => {
  const security = createBlobSecurity("synthetic-map-key");
  const frame = response(Buffer.from("synthetic map"), security, 15001);
  assert.notDeepEqual(frame.subarray(19, 27), Buffer.from(security.endpoint));
  assert.equal(inspectMapFrame(frame, LOCAL_KEY, security, 15001), "correlated-v1");
  assert.equal(inspectMapFrame(frame, LOCAL_KEY, security, 15002), "unrelated");
  const opaque = Buffer.from(frame);
  opaque.fill(0x91, 19, 27);
  opaque.writeUInt32BE(localProtocolCodec.crc32(opaque.subarray(0, -4)), opaque.length - 4);
  assert.equal(inspectMapFrame(opaque, LOCAL_KEY, security, 15001), "opaque-301");
  assert.equal(inspectMapFrame(Buffer.alloc(23), LOCAL_KEY, security, 15001), "malformed");
});

const L01_KEY = "b8Hj5mFk3QzT7rLp";
const ACK_NONCE = 485592656;

test("CRC32 matches the standard check vector", () => {
  assert.equal(localProtocolCodec.crc32(Buffer.from("123456789")), 0xcbf43926);
  assert.equal(localProtocolCodec.crc32(Buffer.alloc(0)), 0);
});

test("L01 matches upstream cryptoEngine.test.ts official vector", () => {
  const context = {
    version: "L01" as const, localKey: L01_KEY, timestamp: 1753606905, sequence: 1,
    random: 304251, connectNonce: 893563, ackNonce: ACK_NONCE,
  };
  const plain = Buffer.from('{"dps":{"101":"{\\"id\\":1806,\\"method\\":\\"get_prop\\",\\"params\\":[\\"get_status\\"]}"},"t":1753606905}');
  const encrypted = Buffer.from([
    253,96,200,218,202,28,202,230,127,96,119,71,123,250,157,55,24,154,56,215,91,60,74,144,124,36,53,211,193,70,238,132,216,249,149,151,
    227,225,87,26,1,89,97,206,170,77,100,188,54,149,250,224,36,195,65,103,55,215,113,80,52,29,226,156,173,47,149,191,175,83,35,88,241,
    43,191,248,159,20,15,239,91,30,226,132,195,171,254,59,131,165,119,145,10,114,5,109,171,77,90,117,177,130,209,160,203,161,69,227,
    228,80,243,146,116,67,
  ]);
  assert.deepEqual(localProtocolCodec.encryptPayload(plain, context), encrypted);
  assert.deepEqual(localProtocolCodec.decryptPayload(encrypted, context), plain);
  assert.throws(() => localProtocolCodec.decryptPayload(encrypted, { ...context, ackNonce: ACK_NONCE + 1 }));
  assert.throws(() => localProtocolCodec.decryptPayload(encrypted, { ...context, connectNonce: context.connectNonce + 1 }));
});

test("V1 follows the independent upstream MD5 timestamp permutation and AES-ECB specification", () => {
  const context = {
    version: "1.0" as const, localKey: L01_KEY, timestamp: 0x12345678,
    sequence: 1, random: 2, connectNonce: 3, ackNonce: 4,
  };
  const key = createHash("md5").update("67482315" + L01_KEY + "TXdfu$jyZ#TZHsg4").digest();
  const cipher = createCipheriv("aes-128-ecb", key, null);
  const plain = Buffer.from("read-only status");
  const expected = Buffer.concat([cipher.update(plain), cipher.final()]);
  assert.deepEqual(localProtocolCodec.encryptPayload(plain, context), expected);
  assert.deepEqual(localProtocolCodec.decryptPayload(expected, context), plain);
});
