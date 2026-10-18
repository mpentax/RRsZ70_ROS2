/** Offline synthetic Hermes v96 coordinate fixtures only; NOT a navigation command. No IO. */
export const HERMES_PIN_STATUS = "UNVERIFIED_RESEARCH_ONLY" as const;
type Point = Readonly<{ x: number; y: number }>;
type Rect = Readonly<{ left: number; top: number; width: number; height: number }>;
/** App map rectangle and nested data rectangle are separate, not interchangeable with PNG pixels. */
export type SyntheticHermesMap = Rect & Readonly<{ data: Readonly<{ left: number; top: number }> }>;
const finite = (n: number) => Number.isFinite(n) && Math.abs(n) < 1_000_000;
/** #9572: floor screen pixel before adding map-data crop, invert vertical pixel axis. */
export function syntheticHermesScreenToOrigin(map: SyntheticHermesMap, screen: Point):
  Readonly<{ status: typeof HERMES_PIN_STATUS; origin: Point }> | null {
  if (![map.left, map.top, map.width, map.height, map.data.left, map.data.top, screen.x, screen.y].every(finite) ||
    map.width <= 0 || map.height <= 0 || !Number.isInteger(map.width) || !Number.isInteger(map.height)) return null;
  if (screen.x < 0 || screen.y < 0 || screen.x >= map.width || screen.y >= map.height) return null;
  return { status: HERMES_PIN_STATUS, origin: {
    x: Math.floor(screen.x) + map.left - map.data.left + 0.5,
    y: map.height - Math.floor(screen.y) - 0.5 + map.top - map.data.top,
  } };
}
/** #17368 and #17372: bottom-edge map origin + rotated/unscaled map offset,
 * then Number((value).toFixed(2)); this is not mm and not an RPC payload.
 */
export function syntheticHermesDisplayPoint(origin: Point, inverseMapPoint: Point):
  Readonly<{ status: typeof HERMES_PIN_STATUS; displayPoint: Point }> | null {
  if (![origin.x, origin.y, inverseMapPoint.x, inverseMapPoint.y].every(finite)) return null;
  return { status: HERMES_PIN_STATUS, displayPoint: {
    x: Number((origin.x + inverseMapPoint.x).toFixed(2)),
    y: Number((origin.y - inverseMapPoint.y).toFixed(2)),
  } };
}
/** #26335: button code multiplies coordinate by 50 and rounds to integer;
 * deliberately returns a labelled scalar instead of a target array.
 */
export function syntheticHermesButtonScalar(displayCoordinate: number):
  Readonly<{ status: typeof HERMES_PIN_STATUS; roundedScalar: number }> | null {
  if (!finite(displayCoordinate)) return null;
  return { status: HERMES_PIN_STATUS, roundedScalar: Number((50 * displayCoordinate).toFixed(0)) };
}
