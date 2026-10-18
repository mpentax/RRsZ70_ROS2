import assert from "node:assert/strict";
import { createCipheriv, createHash } from "node:crypto";
import { test } from "node:test";
import { gzipSync } from "node:zlib";
import {
  createBlobSecurity, decodeMapResponse, decodePhotoBlob, MAX_PHOTO_BYTES,
} from "./photo-protocol.ts";
import type { BlobSecurity } from "./photo-protocol.ts";

function blob(data: Buffer, requestId = 20008): Buffer {
  const compressed = gzipSync(data);
  const header = Buffer.alloc(56);
  header.write("ROBOROCK");
  header.writeUInt32LE(requestId, 8);
  header.writeUInt16LE(header.length, 16);
  header.writeUInt32LE(compressed.length, 20);
  return Buffer.concat([header, compressed]);
}

function mapResponse(data: Buffer, security: BlobSecurity, requestId = 20009): Buffer {
  const cipher = createCipheriv("aes-128-cbc", security.nonce, Buffer.alloc(16));
  const encrypted = Buffer.concat([cipher.update(gzipSync(data)), cipher.final()]);
  const header = Buffer.alloc(24);
  header.write(security.endpoint, 0, "ascii");
  header.writeUInt16LE(requestId, 16);
  return Buffer.concat([header, encrypted]);
}

test("protocol-301 ROBOROCK blob decoding matches upstream 56-byte-header fixture", () => {
  const data = Buffer.from([3, 0, 8, 0, 4, 0, 0, 0, 255, 216, 255, 224]);
  const payload = blob(data);
  assert.deepEqual(decodePhotoBlob(301, payload, 20008), { requestId: 20008, data });
  assert.equal(decodePhotoBlob(102, payload), undefined);
  assert.equal(decodePhotoBlob(301, payload, 20009), undefined);
  assert.equal(decodePhotoBlob(301, Buffer.from('["ok"]')), undefined);
});

test("blob decoder rejects truncated, inconsistent, corrupt and decompression-bomb frames", () => {
  const badHeader = blob(Buffer.from("x"));
  badHeader.writeUInt16LE(23, 16);
  const badLength = blob(Buffer.from("x"));
  badLength.writeUInt32LE(0xffffffff, 20);
  const corrupt = blob(Buffer.from("x"));
  corrupt.fill(0, 56);
  const bomb = blob(Buffer.alloc(MAX_PHOTO_BYTES + 1));
  for (const payload of [
    Buffer.from("ROBOROCK"), badHeader, badLength, corrupt, bomb,
    blob(Buffer.from("x")).subarray(0, -1),
    Buffer.concat([blob(Buffer.from("x")), Buffer.from([1])]),
  ]) assert.throws(() => decodePhotoBlob(301, payload));
  // Ignore another request before decompressing its potentially huge payload.
  assert.equal(decodePhotoBlob(301, bomb, 123), undefined);
});

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
  const payload = mapResponse(data, security);
  assert.deepEqual(decodeMapResponse(301, payload, security, 20009), { requestId: 20009, data });
  assert.equal(decodeMapResponse(102, payload, security), undefined);
  assert.equal(decodeMapResponse(301, payload, security, 3), undefined);
  assert.equal(decodeMapResponse(301, payload, createBlobSecurity("different-key")), undefined);
  assert.equal(decodeMapResponse(301, blob(data), security), undefined);
  assert.throws(() => decodeMapResponse(301, payload.subarray(0, -1), security), /frame size/);
  const damaged = Buffer.from(payload);
  damaged.fill(0, 24);
  assert.throws(() => decodeMapResponse(301, damaged, security), /decrypt|compressed/);
  assert.throws(() => decodeMapResponse(301, payload, { ...security, nonce: Buffer.alloc(1) }), /security/);
});
