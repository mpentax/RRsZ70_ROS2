/**
 * OFFLINE SYNTHETIC RESEARCH ONLY. No IO, map cache, UI, navigation route or robot target.
 * Connects RoboClaw's normalized image draft to its top-first PNG and bottom-first
 * V1 image storage. The *hypothetical* embedded-position lookup is NOT the
 * Android click transform or an app_goto_target frame. Never use this output
 * as PinGoEvidence.transform or as an RPC argument.
 */
export const PIN_CLICK_V1_STATUS = "UNVERIFIED_RESEARCH_ONLY" as const;
export type NormalizedDraft = Readonly<{ x: number; y: number }>;
export type SyntheticImage = Readonly<{ width: number; height: number; left: number; top: number }>;
export type PinClickV1Research = Readonly<{
  status: typeof PIN_CLICK_V1_STATUS;
  pngCell: Readonly<{ column: number; row: number }>;
  rawCell: Readonly<{ column: number; row: number; index: number }>;
  /** Cell centre in the V1 embedded-position lookup convention, not an app target. */
  embeddedLookupCellCenterMm: Readonly<{ horizontal: number; vertical: number }>;
}>;

/** A click at the image's right/bottom edge is outside its half-open pixel area.
 * Fractional/negative V1 origins are retained; no rounding precedes the
 * embedded lookup convention floor(position / 50 - origin).
 */
export function syntheticDraftToV1Lookup(draft: NormalizedDraft, image: SyntheticImage): PinClickV1Research | null {
  if (!draft || !image || ![draft.x, draft.y, image.left, image.top].every(Number.isFinite) ||
    !Number.isSafeInteger(image.width) || !Number.isSafeInteger(image.height) ||
    image.width < 1 || image.height < 1 || image.width > 1024 || image.height > 1024 ||
    !Number.isInteger(image.left) || !Number.isInteger(image.top) ||
    Math.abs(image.left) > 1_000_000 || Math.abs(image.top) > 1_000_000 ||
    draft.x < 0 || draft.y < 0 || draft.x >= 1 || draft.y >= 1) return null;
  const column = Math.floor(draft.x * image.width);
  const pngRow = Math.floor(draft.y * image.height);
  const rawRow = image.height - 1 - pngRow;
  const horizontal = (image.left + column + .5) * 50;
  const vertical = (image.top + rawRow + .5) * 50;
  if (!Number.isSafeInteger(column + image.width * rawRow) ||
      Math.floor(horizontal / 50 - image.left) !== column ||
      Math.floor(vertical / 50 - image.top) !== rawRow) return null;
  return { status: PIN_CLICK_V1_STATUS,
    pngCell: { column, row: pngRow },
    rawCell: { column, row: rawRow, index: column + image.width * rawRow },
    embeddedLookupCellCenterMm: { horizontal, vertical } };
}
