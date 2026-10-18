import assert from "node:assert/strict";
import { test } from "node:test";
import { syntheticHermesButtonScalar, syntheticHermesDisplayPoint, syntheticHermesScreenToOrigin, HERMES_PIN_STATUS } from "./legacy-ts/map-click-hermes-fixtures-research.ts";
import { syntheticRasterCellToScreen, syntheticScreenToRasterCell, type SyntheticRaster, type SyntheticViewport } from "./legacy-ts/map-click-transform-research.ts";

const map = { left: -13.25, top: -7.75, width: 11, height: 7, data: { left: -15.5, top: -9.5 } };
test("Hermes #9572 floors before fractional crop translation and inverts y", () => {
  assert.deepEqual(syntheticHermesScreenToOrigin(map, { x: 2.999, y: 1.999 })?.origin, { x: 4.75, y: 7.25 });
  assert.deepEqual(syntheticHermesScreenToOrigin(map, { x: 3, y: 2 })?.origin, { x: 5.75, y: 6.25 });
  assert.equal(syntheticHermesScreenToOrigin(map, { x: -0.001, y: 1 }), null);
  assert.equal(syntheticHermesScreenToOrigin(map, { x: 1, y: 7 }), null);
  assert.equal(syntheticHermesScreenToOrigin({ ...map, width: 0 }, { x: 1, y: 1 }), null);
});
test("#17368/#17372 bottom-edge origin, inverted offset, two-decimal display precision", () => {
  const o = { x: -13.25, y: -0.75 };
  assert.deepEqual(syntheticHermesDisplayPoint(o, { x: 2.125, y: -1.235 })?.displayPoint,
    { x: -11.13, y: 0.49 });
  assert.equal(syntheticHermesDisplayPoint(o, { x: NaN, y: 0 }), null);
});
test("#26335 multiplies display coordinate by 50, rounds integer; negative and fractional cases", () => {
  assert.equal(syntheticHermesButtonScalar(1.24)?.roundedScalar, 62);
  assert.equal(syntheticHermesButtonScalar(-1.24)?.roundedScalar, -62);
  assert.equal(syntheticHermesButtonScalar(0.011)?.roundedScalar, 1);
  assert.equal(syntheticHermesButtonScalar(Number.POSITIVE_INFINITY), null);
  assert.equal(syntheticHermesButtonScalar(1)?.status, HERMES_PIN_STATUS);
});
test("existing PNG crop/pan/scale/rotation inverse differs from Hermes floor/crop path", () => {
  const raster: SyntheticRaster = { left: -13, top: -7, width: 11, height: 7 };
  const base: SyntheticViewport = { width: 480, height: 300, scale: 2.5, pan: { x: 19, y: -27 },
    crop: { left: 2, top: 1, width: 7, height: 5 }, clockwiseDegrees: 0 };
  for (const clockwiseDegrees of [0, 90, 180, 270] as const) {
    const view = { ...base, clockwiseDegrees };
    const screen = syntheticRasterCellToScreen({ x: 4, y: 3 }, raster, view);
    assert.ok(screen);
    assert.deepEqual(syntheticScreenToRasterCell(screen, raster, view)?.rasterCell, { x: 4, y: 3 });
  }
  assert.notDeepEqual(syntheticHermesScreenToOrigin(map, { x: 4.5, y: 3.5 })?.origin, { x: 4, y: 3 });
});
