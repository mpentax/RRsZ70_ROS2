import { createCipheriv, createHash } from "node:crypto";
import { gzipSync } from "node:zlib";
import type { BlobSecurity } from "./photo-protocol.ts";

/** Synthetic-only V1 bytes for raster/cache tests; never loads a private map. */
export function syntheticV1Image(pixels = Buffer.from([1, 31, 255, 0]), width = 2, height = 2): Buffer {
  const header = Buffer.alloc(20); header.write("rr"); header.writeUInt16LE(20, 2);
  const image = Buffer.alloc(24 + pixels.length); image.writeUInt16LE(2); image.writeUInt16LE(24, 2);
  image.writeUInt32LE(pixels.length, 4); image.writeUInt32LE(height, 16); image.writeUInt32LE(width, 20);
  pixels.copy(image, 24);
  const position = Buffer.alloc(20); position.writeUInt16LE(8); position.writeUInt16LE(8, 2);
  position.writeUInt32LE(12, 4); position.writeInt32LE(123456, 8); position.writeInt32LE(-7777, 12);
  const digest = Buffer.alloc(28); digest.writeUInt16LE(1024); digest.writeUInt16LE(8, 2); digest.writeUInt32LE(20, 4);
  const raw = Buffer.concat([header, image, position, digest]); raw.writeUInt32LE(raw.length - 20, 4);
  createHash("sha1").update(raw.subarray(0, -20)).digest().copy(raw, raw.length - 20);
  return raw;
}

export function encodeMapResponse(data: Buffer, security: BlobSecurity, requestId = 20009): Buffer {
  const cipher = createCipheriv("aes-128-cbc", security.nonce, Buffer.alloc(16));
  const encrypted = Buffer.concat([cipher.update(gzipSync(data)), cipher.final()]);
  const header = Buffer.alloc(24);
  header.write(security.endpoint, 0, "ascii");
  header.writeUInt16LE(requestId, 16);
  return Buffer.concat([header, encrypted]);
}
