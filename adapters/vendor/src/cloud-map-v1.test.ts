import assert from "node:assert/strict";
import { test } from "node:test";
import { localProtocolCodec } from "./local.ts";
import { encodeV1MqttFrame } from "./cloud-control.ts";
import { decodeCloudMapResponse, inspectMapFrame, MAX_CLOUD_MAP_FRAME_BYTES } from "./cloud-map-v1.ts";
import { createBlobSecurity } from "./photo-protocol.ts";
import { encodeMapResponse } from "./map-raster-fixture.ts";

const LOCAL_KEY = "0123456789abcdef";
function response(data: Buffer, security = createBlobSecurity("map-test-key"), requestId = 20009): Buffer {
  return encodeV1MqttFrame(LOCAL_KEY, 301, encodeMapResponse(data, security, requestId), 1753606905);
}

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
  // The same bytes looked opaque when inspected without the required V1 outer
  // ECB layer; this is the concrete protocol difference behind the live retry.
  assert.notDeepEqual(frame.subarray(19, 27), Buffer.from(security.endpoint));
  assert.equal(inspectMapFrame(frame, LOCAL_KEY, security, 15001), "correlated-v1");
  assert.equal(inspectMapFrame(frame, LOCAL_KEY, security, 15002), "unrelated");
  const opaque = Buffer.from(frame);
  opaque.fill(0x91, 19, 27);
  opaque.writeUInt32BE(localProtocolCodec.crc32(opaque.subarray(0, -4)), opaque.length - 4);
  assert.equal(inspectMapFrame(opaque, LOCAL_KEY, security, 15001), "opaque-301");
  assert.equal(inspectMapFrame(Buffer.alloc(23), LOCAL_KEY, security, 15001), "malformed");
});
