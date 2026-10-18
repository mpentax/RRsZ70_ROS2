import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { test } from "node:test";
import { summarizeV1Map } from "./v1-map-summary.ts";

function block(type: number, headerSize: number, data: Buffer, image = false): Buffer {
  const result = Buffer.alloc(headerSize + data.length);
  result.writeUInt16LE(type); result.writeUInt16LE(headerSize, 2); result.writeUInt32LE(data.length, 4);
  if (image) {
    result.writeInt32LE(2, headerSize - 16); result.writeInt32LE(4, headerSize - 12);
    result.writeUInt32LE(2, headerSize - 8); result.writeUInt32LE(2, headerSize - 4);
  }
  data.copy(result, headerSize); return result;
}
function sample(vacuumX = 200, vacuumY = 100): Buffer {
  const header = Buffer.alloc(20); header.write("rr"); header.writeUInt16LE(20, 2);
  const image = block(2, 24, Buffer.from([1, 31, 0, 255]), true);
  const position = Buffer.alloc(12); position.writeInt32LE(vacuumX); position.writeInt32LE(vacuumY, 4);
  const digest = block(1024, 8, Buffer.alloc(20));
  const raw = Buffer.concat([header, image, block(8, 8, position), digest]);
  raw.writeUInt32LE(raw.length - 20, 4);
  createHash("sha1").update(raw.subarray(0, -20)).digest().copy(raw, raw.length - 20);
  return raw;
}
test("bounded V1 summary extracts image extent, segment and embedded position without claiming fresh pose", () => {
  const result = summarizeV1Map(sample());
  assert.deepEqual(result.image.roomSegments, [3]);
  assert.deepEqual(result.vacuumPosition, { x: 200, y: 100, angle: 0 });
  assert.equal(result.embeddedVacuumRoomSegment, null);
  assert.equal(result.positionIsIndependentFreshTelemetry, false);
  assert.equal(result.image.pixels, 4);
});
test("fractional embedded position uses Android plugin's floor grid index, not rounding", () => {
  // Image origin is (4, 2), size 2×2. Cell (1, 0) is segment 3;
  // rounding 4.6 - 4 would incorrectly index that column instead of (0, 0).
  assert.equal(summarizeV1Map(sample(230, 100)).embeddedVacuumRoomSegment, null);
  // Cell (1, 0) is the second image byte, and remains in bounds.
  assert.equal(summarizeV1Map(sample(250, 100)).embeddedVacuumRoomSegment, 3);
  assert.equal(summarizeV1Map(sample(300, 100)).embeddedVacuumRoomSegment, null);
});
test("V1 summary fails closed on altered digest or invalid image geometry", () => {
  const damaged = sample(); damaged[27] ^= 1;
  assert.throws(() => summarizeV1Map(damaged), /integrity/);
  const bad = sample(); bad.writeUInt32LE(3, 20 + 24 - 4);
  createHash("sha1").update(bad.subarray(0, -20)).digest().copy(bad, bad.length - 20);
  assert.throws(() => summarizeV1Map(bad), /image size/);
});
