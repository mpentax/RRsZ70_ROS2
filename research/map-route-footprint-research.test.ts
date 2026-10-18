import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { test } from "node:test";
import { syntheticV1Image } from "../adapters/vendor/src/map-raster-fixture.ts";
import { FOOTPRINT_ROUTE_STATUS, researchFootprintCorridor } from "./legacy-ts/map-route-footprint-research.ts";

const F = 255, W = 1, U = 0, S = 31;
function fixture(rows: number[][]): Buffer {
  const raw = syntheticV1Image(Buffer.from(rows.toReversed().flat()), rows[0].length, rows.length);
  raw.writeInt32LE(-17, 28); raw.writeInt32LE(-9, 32);
  createHash("sha1").update(raw.subarray(0, -20)).digest().copy(raw, raw.length - 20);
  return raw;
}
function mask(width: number, height: number, x = -1, y = -1, value = 1) {
  const pixels = new Uint8Array(width * height);
  if (x >= 0) pixels[x + width * y] = value;
  return { width, height, pixels };
}
const check = (raw: Buffer, from: { x: number; y: number }, to = from, radiusCells = 1,
  dynamicMask = mask(7, 7)) => researchFootprintCorridor(raw, { from, to, radiusCells, dynamicMask });
const open = fixture(Array.from({ length: 7 }, () => Array(7).fill(F)));
test("interior footprint result is explicitly historical synthetic classification, not safe route", () => {
  assert.deepEqual(check(open, { x: 2, y: 3 }, { x: 4, y: 3 }),
    { status: FOOTPRINT_ROUTE_STATUS, accepted: true, reason: "static-classification-only" });
  assert.equal(check(open, { x: 3, y: 3 }, { x: 3, y: 3 }, 0).accepted, true);
});
test("one-cell floor corridor rejects footprint although center-only passes", () => {
  const narrow = fixture(Array.from({ length: 7 }, (_, y) => Array(7).fill(y === 3 ? F : W)));
  assert.equal(check(narrow, { x: 2, y: 3 }, { x: 4, y: 3 }, 0).accepted, true);
  assert.equal(check(narrow, { x: 2, y: 3 }, { x: 4, y: 3 }).reason, "unknown-or-wall-or-edge");
});
test("edge inflation rejects, even for an otherwise floor center", () => {
  assert.equal(check(open, { x: 0, y: 3 }).reason, "unknown-or-wall-or-edge");
  assert.equal(check(open, { x: 2, y: 0 }).reason, "unknown-or-wall-or-edge");
});
test("diagonal crossing checks side cells and inflated flank in both directions", () => {
  const corner = fixture([[F, W, F], [F, F, F], [F, F, F]]);
  assert.equal(check(corner, { x: 0, y: 0 }, { x: 1, y: 1 }, 0, mask(3, 3)).reason, "unknown-or-wall-or-edge");
  assert.equal(check(corner, { x: 1, y: 1 }, { x: 0, y: 0 }, 0, mask(3, 3)).reason, "unknown-or-wall-or-edge");
  const flank = fixture(Array.from({ length: 7 }, (_, y) => Array.from({ length: 7 }, (_, x) => x === 2 && y === 3 ? W : F)));
  assert.equal(check(flank, { x: 3, y: 2 }, { x: 4, y: 3 }, 0).accepted, true);
  assert.equal(check(flank, { x: 3, y: 2 }, { x: 4, y: 3 }, 1).reason, "unknown-or-wall-or-edge");
});
test("holes, unknowns and walls are rejected in top-first interpretation", () => {
  for (const pixel of [U, W]) {
    const rows = Array.from({ length: 7 }, () => Array(7).fill(F)); rows[2][3] = pixel;
    assert.equal(check(fixture(rows), { x: 3, y: 3 }).reason, "unknown-or-wall-or-edge");
  }
  const hole = Array.from({ length: 7 }, () => Array(7).fill(F)); hole[4][4] = U;
  assert.equal(check(fixture(hole), { x: 2, y: 2 }, { x: 5, y: 5 }).reason, "unknown-or-wall-or-edge");
  hole[4][4] = S;
  assert.equal(check(fixture(hole), { x: 3, y: 4 }, { x: 4, y: 4 }, 0).accepted, true);
});
test("dynamic mask absent, wrong size, obstructed and unknown are all rejected", () => {
  const from = { x: 3, y: 3 };
  assert.equal(researchFootprintCorridor(open, { from, to: from, radiusCells: 1 }).reason, "dynamic-mask-unavailable");
  assert.equal(check(open, from, from, 1, mask(2, 2)).reason, "dynamic-mask-unavailable");
  assert.equal(check(open, from, from, 1, mask(7, 7, 4, 3)).reason, "dynamic-obstacle");
  assert.equal(check(open, from, from, 1, mask(7, 7, 3, 3, 2)).reason, "dynamic-obstacle");
});
test("invalid coordinates or radius fail closed; integrity tampering throws", () => {
  assert.equal(check(open, { x: -1, y: 3 }).reason, "invalid-input");
  assert.equal(check(open, { x: 2.5, y: 3 }).reason, "invalid-input");
  assert.equal(check(open, { x: 3, y: 3 }, { x: 3, y: 3 }, -1).reason, "invalid-input");
  const damaged = Buffer.from(open); damaged[50] ^= 1;
  assert.throws(() => check(damaged, { x: 3, y: 3 }), /integrity/);
});
