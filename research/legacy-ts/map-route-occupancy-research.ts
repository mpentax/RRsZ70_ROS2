import { verifiedV1Image } from "../../adapters/vendor/src/v1-map-summary.ts";

/** OFFLINE SYNTHETIC RESEARCH ONLY. Neither result nor input is a robot target,
 * navigable route, or independently verified obstacle observation. The V1
 * classified floor is historical and omits dynamic obstacles and robot footprint.
 */
export const ROUTE_OCCUPANCY_STATUS = "UNVERIFIED_RESEARCH_ONLY" as const;
type Cell = Readonly<{ x: number; y: number }>;
type Result = Readonly<{ status: typeof ROUTE_OCCUPANCY_STATUS; clear: boolean; reason: "clear" | "unknown-or-wall" | "corner-cut" }>;
const cell = (p: Cell, width: number, height: number): boolean => p != null &&
  Number.isInteger(p.x) && Number.isInteger(p.y) && p.x >= 0 && p.y >= 0 && p.x < width && p.y < height;
const floor = (pixel: number): boolean => pixel === 255 || pixel === 7 || (pixel & 7) === 7;

/** Check a supercover of a straight segment between PNG cell centers.
 * PNG y is top-first, V1 bytes bottom-first; no mm or viewport conversion.
 * At exact corner crossings, both side-adjacent cells must also be floor.
 * This is a necessary static-classification check, never a sufficient route.
 */
export function researchStaticSegment(raw: Buffer, from: Cell, to: Cell): Result | null {
  const { width, height, pixels } = verifiedV1Image(raw);
  if (!cell(from, width, height) || !cell(to, width, height)) return null;
  const free = (x: number, y: number) => floor(pixels[x + width * (height - 1 - y)]);
  const reject = (reason: "unknown-or-wall" | "corner-cut"): Result =>
    ({ status: ROUTE_OCCUPANCY_STATUS, clear: false, reason });
  if (!free(from.x, from.y)) return reject("unknown-or-wall");
  let x = from.x, y = from.y;
  const dx = to.x - x, dy = to.y - y;
  const sx = Math.sign(dx), sy = Math.sign(dy), ax = Math.abs(dx), ay = Math.abs(dy);
  // Compare exact integer crossing times instead of floating point steps.
  let nx = 0, ny = 0;
  while (nx < ax || ny < ay) {
    const compare = nx === ax ? 1 : ny === ay ? -1 :
      (2 * nx + 1) * ay - (2 * ny + 1) * ax;
    if (compare === 0) {
      if (!free(x + sx, y) || !free(x, y + sy)) return reject("corner-cut");
      x += sx; y += sy; nx++; ny++;
    } else if (compare < 0) { x += sx; nx++; }
    else { y += sy; ny++; }
    if (!free(x, y)) return reject("unknown-or-wall");
  }
  return { status: ROUTE_OCCUPANCY_STATUS, clear: true, reason: "clear" };
}
