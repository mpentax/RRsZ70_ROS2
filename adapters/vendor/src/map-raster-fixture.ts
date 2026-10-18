import { createHash } from "node:crypto";

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
