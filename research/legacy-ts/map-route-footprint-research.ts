import { verifiedV1Image } from "../../adapters/vendor/src/v1-map-summary.ts";

/** Offline historical-classification research only. An accepted static corridor
 * is NOT a safe route, active-map binding, current pose, or dynamic clearance.
 * Coordinates are top-first PNG cell centers, not robot/navigation coordinates.
 */
export const FOOTPRINT_ROUTE_STATUS = "UNVERIFIED_RESEARCH_ONLY" as const;
type Cell = Readonly<{ x: number; y: number }>;
type Assessment = Readonly<{ status: typeof FOOTPRINT_ROUTE_STATUS; accepted: boolean;
  reason: "static-classification-only" | "invalid-input" | "unknown-or-wall-or-edge" | "dynamic-mask-unavailable" | "dynamic-obstacle" }>;
type Mask = Readonly<{ width: number; height: number; pixels: Uint8Array }>;
export type FootprintResearchInput = Readonly<{
  from: Cell; to: Cell;
  /** Integer cell radius, conservatively covers every cell whose square is
   * within this radius of any traversed cell center (including corner touch).
   * Not a calibrated physical robot radius or an mm conversion. */
  radiusCells: number;
  /** Required, independently supplied synthetic mask: 0 = known unoccupied,
   * 1 = obstacle. Any other value is unknown and rejected. This is not telemetry. */
  dynamicMask?: Mask;
}>;

const result = (reason: Assessment["reason"]): Assessment => ({
  status: FOOTPRINT_ROUTE_STATUS, accepted: reason === "static-classification-only", reason,
});
const validPoint = (p: Cell, w: number, h: number) => p != null &&
  Number.isInteger(p.x) && Number.isInteger(p.y) && p.x >= 0 && p.y >= 0 && p.x < w && p.y < h;
const floor = (pixel: number) => pixel === 255 || pixel === 7 || (pixel & 7) === 7;

/** Pure, fail-closed straight-line cell corridor check over integrity-verified
 * synthetic V1 bytes. Inflate every supercovered line cell by the supplied
 * discrete footprint radius, rejecting map boundaries, unknowns, walls and
 * unavailable/unknown/obstructed dynamic-mask cells. At radius zero exact
 * corner crossings still inspect both adjacent cells. No RPC or coordinates.
 */
export function researchFootprintCorridor(raw: Buffer, input: FootprintResearchInput): Assessment {
  const { width: w, height: h, pixels } = verifiedV1Image(raw);
  const { from, to, radiusCells: radius, dynamicMask: mask } = input;
  if (!validPoint(from, w, h) || !validPoint(to, w, h) ||
      !Number.isSafeInteger(radius) || radius < 0 || radius > Math.max(w, h)) return result("invalid-input");
  if (!mask || mask.width !== w || mask.height !== h || mask.pixels?.length !== w * h)
    return result("dynamic-mask-unavailable");
  const visited = new Set<number>();
  const check = (cx: number, cy: number): Assessment | null => {
    // Radius-0 still checks the exact traversed cell. For radius r>0,
    // circle-to-cell-square contact is inclusive and uses integer arithmetic.
    for (let oy = -radius; oy <= radius; oy++) for (let ox = -radius; ox <= radius; ox++) {
      const closestX = Math.max(0, 2 * Math.abs(ox) - 1);
      const closestY = Math.max(0, 2 * Math.abs(oy) - 1);
      if (closestX * closestX + closestY * closestY > 4 * radius * radius) continue;
      const x = cx + ox, y = cy + oy;
      if (x < 0 || x >= w || y < 0 || y >= h) return result("unknown-or-wall-or-edge");
      const index = x + w * y;
      if (visited.has(index)) continue;
      visited.add(index);
      if (!floor(pixels[x + w * (h - 1 - y)])) return result("unknown-or-wall-or-edge");
      if (mask.pixels[index] !== 0) return result("dynamic-obstacle");
    }
    return null;
  };
  let x = from.x, y = from.y;
  const dx = to.x - x, dy = to.y - y;
  const sx = Math.sign(dx), sy = Math.sign(dy), ax = Math.abs(dx), ay = Math.abs(dy);
  let nx = 0, ny = 0;
  const first = check(x, y);
  if (first) return first;
  while (nx < ax || ny < ay) {
    const compare = nx === ax ? 1 : ny === ay ? -1 :
      (2 * nx + 1) * ay - (2 * ny + 1) * ax;
    if (compare === 0) {
      const sideX = check(x + sx, y), sideY = check(x, y + sy);
      if (sideX) return sideX;
      if (sideY) return sideY;
      x += sx; y += sy; nx++; ny++;
    } else if (compare < 0) { x += sx; nx++; }
    else { y += sy; ny++; }
    const rejected = check(x, y);
    if (rejected) return rejected;
  }
  return result("static-classification-only");
}
