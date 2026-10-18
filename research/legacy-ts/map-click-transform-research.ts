/**
 * OFFLINE RESEARCH ONLY — never a navigation target or verified robot frame.
 * This module accepts synthetic geometry, not a V1 map, and has no IO/RPC.
 * No production caller may interpret its output as app_goto_target input.
 *
 * Source trace: extracted Android Hermes v96 index.android.hasm, functions
 * #17322 (pan end), #17368 (map rectangle origin), #17370 (inverse transform),
 * #17372 (target display, rounding), #9572 (screen -> origin), #9605 (point
 * normalization). The raster in private-map-raster.ts flips V1 row-major
 * bottom-first storage to PNG top-first; v1-map-summary.ts indexes raw cells
 * with floor(position_mm / 50 - origin). App #9572 instead floors image-screen
 * coordinates before adding its map/data crop origins and a half-cell offset.
 * These are different paths; their equivalence is NOT established.
 */
export const MAP_CLICK_TRANSFORM_STATUS = "UNVERIFIED_RESEARCH_ONLY" as const;
export type ResearchPoint = Readonly<{ x: number; y: number }>;
export type SyntheticRaster = Readonly<{ left: number; top: number; width: number; height: number }>;
export type SyntheticViewport = Readonly<{
  width: number; height: number; scale: number; pan: ResearchPoint;
  /** Explicit synthetic crop in PNG top-left pixels, NOT a V1 image origin. */
  crop: Readonly<{ left: number; top: number; width: number; height: number }>;
  /** Mathematical clockwise screen rotation; not an assertion about app mapDeg. */
  clockwiseDegrees: 0 | 90 | 180 | 270;
}>;

function finite(value: number): boolean { return Number.isFinite(value) && Math.abs(value) <= 1_000_000; }
function validPoint(point: ResearchPoint): boolean { return finite(point.x) && finite(point.y); }
function validRaster(raster: SyntheticRaster): boolean {
  return finite(raster.left) && finite(raster.top) && Number.isInteger(raster.left) && Number.isInteger(raster.top) &&
    Number.isInteger(raster.width) && Number.isInteger(raster.height) && raster.width > 0 && raster.height > 0 &&
    raster.width <= 1024 && raster.height <= 1024;
}
function validCrop(crop: SyntheticViewport["crop"], raster: SyntheticRaster): boolean {
  return [crop.left, crop.top, crop.width, crop.height].every(Number.isInteger) &&
    crop.left >= 0 && crop.top >= 0 && crop.width > 0 && crop.height > 0 &&
    crop.left + crop.width <= raster.width && crop.top + crop.height <= raster.height;
}
function validViewport(view: SyntheticViewport, raster: SyntheticRaster): boolean {
  return finite(view.width) && finite(view.height) && view.width > 0 && view.height > 0 &&
    finite(view.scale) && view.scale >= 0.1 && view.scale <= 20 && validPoint(view.pan) &&
    [0, 90, 180, 270].includes(view.clockwiseDegrees) && validCrop(view.crop, raster);
}
function rotate(point: ResearchPoint, deg: SyntheticViewport["clockwiseDegrees"]): ResearchPoint {
  switch (deg) {
    case 0: return point;
    case 90: return { x: -point.y, y: point.x };
    case 180: return { x: -point.x, y: -point.y };
    case 270: return { x: point.y, y: -point.x };
  }
}

/** Synthetic viewport only: invert pan, scale, screen rotation and crop.
 * Returns a labelled, non-command-shaped raster cell, never mm or a target pair.
 */
export function syntheticScreenToRasterCell(
  screen: ResearchPoint, raster: SyntheticRaster, view: SyntheticViewport,
): Readonly<{ status: typeof MAP_CLICK_TRANSFORM_STATUS; rasterCell: ResearchPoint }> | null {
  if (!validPoint(screen) || !validRaster(raster) || !validViewport(view, raster)) return null;
  const centered = { x: (screen.x - view.width / 2 - view.pan.x) / view.scale,
    y: (screen.y - view.height / 2 - view.pan.y) / view.scale };
  const unrotated = rotate(centered, ((360 - view.clockwiseDegrees) % 360) as SyntheticViewport["clockwiseDegrees"]);
  const cropX = unrotated.x + view.crop.width / 2;
  const cropY = unrotated.y + view.crop.height / 2;
  if (!finite(cropX) || !finite(cropY) || cropX < 0 || cropY < 0 || cropX >= view.crop.width || cropY >= view.crop.height) return null;
  return { status: MAP_CLICK_TRANSFORM_STATUS,
    rasterCell: { x: Math.floor(view.crop.left + cropX), y: Math.floor(view.crop.top + cropY) } };
}

/** Synthetic forward companion for fixture construction; pixel CENTER only. */
export function syntheticRasterCellToScreen(
  rasterCell: ResearchPoint, raster: SyntheticRaster, view: SyntheticViewport,
): ResearchPoint | null {
  if (!validRaster(raster) || !validViewport(view, raster) || !validPoint(rasterCell) ||
      !Number.isInteger(rasterCell.x) || !Number.isInteger(rasterCell.y) ||
      rasterCell.x < view.crop.left || rasterCell.y < view.crop.top ||
      rasterCell.x >= view.crop.left + view.crop.width || rasterCell.y >= view.crop.top + view.crop.height) return null;
  const local = { x: rasterCell.x - view.crop.left + 0.5 - view.crop.width / 2,
    y: rasterCell.y - view.crop.top + 0.5 - view.crop.height / 2 };
  const rotated = rotate(local, view.clockwiseDegrees);
  return { x: view.width / 2 + view.pan.x + rotated.x * view.scale,
    y: view.height / 2 + view.pan.y + rotated.y * view.scale };
}

/** Compare parser bottom-first index and PNG top-first cell, for synthetic samples only. */
export function syntheticRasterToRawIndex(
  rasterCell: ResearchPoint, raster: SyntheticRaster,
): number | null {
  if (!validRaster(raster) || !validPoint(rasterCell) || !Number.isInteger(rasterCell.x) ||
      !Number.isInteger(rasterCell.y) || rasterCell.x < 0 || rasterCell.y < 0 ||
      rasterCell.x >= raster.width || rasterCell.y >= raster.height) return null;
  return rasterCell.x + raster.width * (raster.height - 1 - rasterCell.y);
}

/** Parser's embedded mm-position cell lookup, retained as an independent synthetic check.
 * No screen or app target semantics can be inferred from this operation.
 */
export function syntheticEmbeddedMmToRawIndex(mm: ResearchPoint, raster: SyntheticRaster): number | null {
  if (!validRaster(raster) || !validPoint(mm)) return null;
  const x = Math.floor(mm.x / 50 - raster.left), y = Math.floor(mm.y / 50 - raster.top);
  return x < 0 || y < 0 || x >= raster.width || y >= raster.height ? null : x + raster.width * y;
}
