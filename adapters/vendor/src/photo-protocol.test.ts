import assert from "node:assert/strict";
import { createCipheriv, createHash } from "node:crypto";
import { test } from "node:test";
import { gzipSync } from "node:zlib";
import { createBlobSecurity, decodeMapResponse, type BlobSecurity } from "./photo-protocol.ts";

export function mapResponse(data: Buffer, security: BlobSecurity, requestId = 20009): Buffer {
  const cipher = createCipheriv("aes-128-cbc", security.nonce, Buffer.alloc(16));
  const encrypted = Buffer.concat([cipher.update(gzipSync(data)), cipher.final()]);
  const header = Buffer.alloc(24);
  header.write(security.endpoint, 0, "ascii");
  header.writeUInt16LE(requestId, 16);
  return Buffer.concat([header, encrypted]);
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
  const payload = mapResponse(data, security);
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
