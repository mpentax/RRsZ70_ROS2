import assert from "node:assert/strict";
import { test } from "node:test";
import { inflateSync } from "node:zlib";
import { renderPrivateV1Map } from "./private-map-raster.ts";
import { localProtocolCodec } from "./local.ts";
import { syntheticV1Image } from "./map-raster-fixture.ts";

test("V1 raster is a bounded RGB PNG with only occupancy pixels and no private chunks", () => {
  const png = renderPrivateV1Map(syntheticV1Image());
  assert.deepEqual(png.subarray(0, 8), Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]));
  const chunks: { name: string; data: Buffer }[] = [];
  for (let at = 8; at < png.length;) {
    const length = png.readUInt32BE(at), name = png.toString("ascii", at + 4, at + 8);
    assert.equal(png.readUInt32BE(at + 8 + length), localProtocolCodec.crc32(png.subarray(at + 4, at + 8 + length)));
    chunks.push({ name, data: png.subarray(at + 8, at + 8 + length) }); at += 12 + length;
  }
  assert.deepEqual(chunks.map(c => c.name), ["IHDR", "IDAT", "IEND"]);
  assert.equal(chunks[0].data.readUInt32BE(0), 2); assert.equal(chunks[0].data.readUInt32BE(4), 2);
  assert.equal(chunks[0].data[9], 2);
  const rows = inflateSync(chunks[1].data);
  assert.deepEqual([...rows], [0, 224, 231, 234, 24, 30, 37, 0, 67, 76, 86, 194, 219, 200]);
  assert.equal(png.includes(Buffer.from("123456")), false);
});

test("raster refuses corrupted content and oversized dimensions before encoding", () => {
  const damaged = syntheticV1Image(); damaged[30] ^= 1;
  assert.throws(() => renderPrivateV1Map(damaged), /integrity/);
  const huge = syntheticV1Image(Buffer.alloc(1024 * 1024 + 1), 1025, 1024);
  assert.throws(() => renderPrivateV1Map(huge), /image size/);
});
