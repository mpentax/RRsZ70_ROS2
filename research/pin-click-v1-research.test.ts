import assert from "node:assert/strict";
import { test } from "node:test";
import { inflateSync } from "node:zlib";
import { syntheticV1Image } from "../adapters/vendor/src/map-raster-fixture.ts";
import { syntheticDraftToV1Lookup } from "./legacy-ts/pin-click-v1-research.ts";
import { renderPrivateV1Map } from "../adapters/vendor/src/private-map-raster.ts";
import { syntheticEmbeddedMmToRawIndex, syntheticRasterToRawIndex } from "./legacy-ts/map-click-transform-research.ts";

// Synthetic non-square raster and fictional negative origin; no private map.
const image = { width: 5, height: 3, left: -13, top: -7 };
test("all synthetic PNG cell centres map to V1 bottom-first raw cells and embedded lookup", () => {
  for (let y = 0; y < image.height; y++) for (let x = 0; x < image.width; x++) {
    const result = syntheticDraftToV1Lookup({ x: (x + .5) / image.width, y: (y + .5) / image.height }, image);
    assert.ok(result);
    assert.equal(result.status, "UNVERIFIED_RESEARCH_ONLY");
    assert.deepEqual(result.pngCell, { column: x, row: y });
    assert.deepEqual(result.rawCell, { column: x, row: image.height - 1 - y, index: x + image.width * (image.height - 1 - y) });
    assert.equal(syntheticRasterToRawIndex({ x, y }, image), result.rawCell.index);
    assert.equal(syntheticEmbeddedMmToRawIndex({ x: result.embeddedLookupCellCenterMm.horizontal,
      y: result.embeddedLookupCellCenterMm.vertical }, image), result.rawCell.index);
  }
});
test("fractional draft boundaries floor to one pixel; right and bottom edges fail closed", () => {
  assert.equal(syntheticDraftToV1Lookup({ x: .199999, y: .333332 }, image)?.rawCell.index, 10);
  assert.equal(syntheticDraftToV1Lookup({ x: .2, y: 1 / 3 }, image)?.rawCell.index, 6);
  assert.equal(syntheticDraftToV1Lookup({ x: 0, y: 0 }, image)?.rawCell.index, 10);
  assert.equal(syntheticDraftToV1Lookup({ x: .999999, y: .999999 }, image)?.rawCell.index, 4);
  for (const draft of [{ x: 1, y: .5 }, { x: 0, y: 1 }, { x: -1e-10, y: 0 }, { x: NaN, y: 0 }, { x: Infinity, y: 0 }])
    assert.equal(syntheticDraftToV1Lookup(draft, image), null);
  for (const bad of [{ ...image, width: 0 }, { ...image, height: 1025 }, { ...image, left: .5 }, { ...image, top: Infinity }])
    assert.equal(syntheticDraftToV1Lookup({ x: .5, y: .5 }, bad), null);
});
test("synthetic V1 raster rendering picks the same cell in the top-first PNG", () => {
  const pixels = Buffer.from([1, 7, 255, 0, 1, 255]); // bottom-first, 3 x 2
  const raw = syntheticV1Image(pixels, 3, 2);
  const png = renderPrivateV1Map(raw);
  const width = png.readUInt32BE(16), height = png.readUInt32BE(20);
  assert.deepEqual([width, height], [3, 2]);
  const idatOffset = 8 + 25;
  assert.equal(png.toString("ascii", idatOffset + 4, idatOffset + 8), "IDAT");
  const rows = inflateSync(png.subarray(idatOffset + 8, idatOffset + 8 + png.readUInt32BE(idatOffset)));
  const expected = [[24, 30, 37], [67, 76, 86], [224, 231, 234], [67, 76, 86], [224, 231, 234], [224, 231, 234]];
  for (let y = 0; y < height; y++) for (let x = 0; x < width; x++) {
    const result = syntheticDraftToV1Lookup({ x: (x + .5) / width, y: (y + .5) / height }, { width, height, left: -3, top: 4 });
    assert.ok(result);
    assert.equal(pixels[result.rawCell.index], pixels[x + width * (height - 1 - y)]);
    assert.deepEqual([...rows.subarray(y * (width * 3 + 1) + 1 + x * 3, y * (width * 3 + 1) + 4 + x * 3)], expected[y * width + x]);
  }
});
