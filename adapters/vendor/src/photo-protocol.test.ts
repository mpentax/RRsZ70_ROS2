import assert from "node:assert/strict";
import { createHash } from "node:crypto";
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
