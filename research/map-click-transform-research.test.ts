import assert from "node:assert/strict";
import { test } from "node:test";
import {
  MAP_CLICK_TRANSFORM_STATUS, syntheticEmbeddedMmToRawIndex, syntheticRasterCellToScreen,
  syntheticRasterToRawIndex, syntheticScreenToRasterCell,
  type SyntheticRaster, type SyntheticViewport,
} from "./legacy-ts/map-click-transform-research.ts";

// Fictional geometry only. Distinct axes, negative V1 origin, partial crop.
const raster: SyntheticRaster = { left: -13, top: -7, width: 11, height: 7 };
const base: SyntheticViewport = {
  width: 480, height: 300, scale: 2.5, pan: { x: 19, y: -27 },
  crop: { left: 2, top: 1, width: 7, height: 5 }, clockwiseDegrees: 0,
};
test("synthetic non-square negative-origin raster survives crop, zoom, pan and four rotations", () => {
  for (const clockwiseDegrees of [0, 90, 180, 270] as const) {
    const view = { ...base, clockwiseDegrees };
    for (let x = 2; x < 9; x++) for (let y = 1; y < 6; y++) {
      const screen = syntheticRasterCellToScreen({ x, y }, raster, view);
      assert.ok(screen);
      assert.deepEqual(syntheticScreenToRasterCell(screen, raster, view), {
        status: MAP_CLICK_TRANSFORM_STATUS, rasterCell: { x, y },
      });
    }
  }
});
test("fractional boundaries floor after inverse, reject crop and viewport exterior", () => {
  const view = { ...base, width: 700, height: 400, scale: 1, pan: { x: 0, y: 0 } };
  // Crop starts at raster x=2. Local x in [0, 1) remains cell 2.
  const left = view.width / 2 - view.crop.width / 2;
  assert.deepEqual(syntheticScreenToRasterCell({ x: left + 0.999, y: view.height / 2 }, raster, view)?.rasterCell.x, 2);
  assert.deepEqual(syntheticScreenToRasterCell({ x: left + 1, y: view.height / 2 }, raster, view)?.rasterCell.x, 3);
  assert.equal(syntheticScreenToRasterCell({ x: left - 0.001, y: view.height / 2 }, raster, view), null);
  assert.equal(syntheticScreenToRasterCell({ x: left + 7, y: view.height / 2 }, raster, view), null);
  assert.equal(syntheticRasterCellToScreen({ x: 1, y: 2 }, raster, view), null);
});
test("V1 raw-image indexing differs from top-first PNG and preserves floor after origin", () => {
  assert.equal(syntheticRasterToRawIndex({ x: 0, y: 0 }, raster), 66);
  assert.equal(syntheticRasterToRawIndex({ x: 0, y: 6 }, raster), 0);
  assert.equal(syntheticEmbeddedMmToRawIndex({ x: -625, y: -350 }, raster), 0);
  assert.equal(syntheticEmbeddedMmToRawIndex({ x: -600.01, y: -350 }, raster), 0);
  assert.equal(syntheticEmbeddedMmToRawIndex({ x: -600, y: -350 }, raster), 1);
  assert.equal(syntheticEmbeddedMmToRawIndex({ x: -650.01, y: -350 }, raster), null);
});
test("fail closed for missing, degenerate, NaN, oversize, unsupported rotation or inverted crop", () => {
  const center = { x: 240, y: 150 };
  for (const bad of [
    { ...base, scale: 0 }, { ...base, pan: { x: NaN, y: 0 } },
    { ...base, clockwiseDegrees: 45 as 0 },
    { ...base, crop: { left: 9, top: 1, width: 7, height: 5 } },
    { ...base, crop: { left: 2, top: 1, width: -7, height: 5 } },
  ]) assert.equal(syntheticScreenToRasterCell(center, raster, bad), null);
  assert.equal(syntheticScreenToRasterCell({ x: Infinity, y: 1 }, raster, base), null);
  assert.equal(syntheticScreenToRasterCell(center, { ...raster, width: 0 }, base), null);
  assert.equal(syntheticScreenToRasterCell(center, { ...raster, left: -13.5 }, base), null);
  assert.equal(syntheticRasterToRawIndex({ x: 11, y: 0 }, raster), null);
});
test("research output is labelled, no command payload or millimetre coordinate API", () => {
  const result = syntheticScreenToRasterCell({ x: 259, y: 123 }, raster, base);
  assert.ok(result);
  assert.deepEqual(Object.keys(result).sort(), ["rasterCell", "status"]);
  assert.equal(result.status, "UNVERIFIED_RESEARCH_ONLY");
});
