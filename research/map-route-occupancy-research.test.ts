import assert from "node:assert/strict";
import { test } from "node:test";
import { createHash } from "node:crypto";
import { syntheticV1Image } from "../adapters/vendor/src/map-raster-fixture.ts";
import { researchStaticSegment, ROUTE_OCCUPANCY_STATUS } from "./legacy-ts/map-route-occupancy-research.ts";

// Fictional bottom-first V1 grid, deliberately non-square and with a nonzero
// negative/fractional origin in separate transform research tests.
function fixture(rowsTopFirst: number[][]): Buffer {
  const h = rowsTopFirst.length, w = rowsTopFirst[0].length;
  const raw = syntheticV1Image(Buffer.from(rowsTopFirst.toReversed().flat()), w, h);
  // A synthetic fixture can vary origins without changing occupancy lookup.
  raw.writeInt32LE(-17, 20 + 8); raw.writeInt32LE(-9, 20 + 12);
  createHash("sha1").update(raw.subarray(0, -20)).digest().copy(raw, raw.length - 20);
  return raw;
}
const F = 255, W = 1, U = 0, S = 31;
const line = fixture([[F, F, F, F, F], [F, F, F, F, F], [F, F, F, F, F]]);
test("fictional classified floor is symmetric, top-first despite V1 bottom-first", () => {
  for (const [from, to] of [[{ x: 0, y: 0 }, { x: 4, y: 2 }], [{ x: 4, y: 2 }, { x: 0, y: 0 }]] as const)
    assert.deepEqual(researchStaticSegment(line, from, to), { status: ROUTE_OCCUPANCY_STATUS, clear: true, reason: "clear" });
  assert.equal(researchStaticSegment(line, { x: -1, y: 0 }, { x: 2, y: 0 }), null);
  assert.equal(researchStaticSegment(line, { x: 0.5, y: 0 }, { x: 2, y: 0 }), null);
});
test("supercover checks intermediate wall, unknown, and exact corner sides", () => {
  const middle = fixture([[F, F, F, F, F], [F, W, U, S, F], [F, F, F, F, F]]);
  assert.equal(researchStaticSegment(middle, { x: 0, y: 1 }, { x: 4, y: 1 })?.reason, "unknown-or-wall");
  assert.equal(researchStaticSegment(fixture([[F, W], [F, F]]), { x: 0, y: 0 }, { x: 1, y: 1 })?.reason, "corner-cut");
  assert.equal(researchStaticSegment(middle, { x: 2, y: 1 }, { x: 2, y: 1 })?.reason, "unknown-or-wall");
  assert.equal(researchStaticSegment(middle, { x: 3, y: 1 }, { x: 3, y: 1 })?.clear, true);
  // A row asymmetry catches accidentally reading V1 bytes as top-first.
  assert.equal(researchStaticSegment(middle, { x: 1, y: 2 }, { x: 3, y: 2 })?.clear, true);
});
test("tampered V1 digest cannot be interpreted as occupancy", () => {
  const damaged = Buffer.from(line); damaged[45] ^= 1;
  assert.throws(() => researchStaticSegment(damaged, { x: 0, y: 0 }, { x: 1, y: 0 }), /integrity/);
});
