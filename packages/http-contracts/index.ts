/** Transport-free HTTP/media contracts shared by the vendor facade and MCP. */
export const CAMERA_NAMES = ["front", "left", "right", "back", "gripper", "depth", "ir", "structured-light"] as const;
const ID = "[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}";
const GET = new RegExp(`^/api/ros/(state|map|goals|tasks(?:/${ID})?)$`);
const POST = new RegExp(`^/api/ros/(frame|stop|control/session|rpc|camera/select|arm/anchor|goals/validate|tasks(?:/${ID}/cancel)?)$`);
export function rosRoute(method: string, path: string): boolean {
  return (method === "GET" ? GET : method === "POST" ? POST : /$a/).test(path);
}
export type ImageDimensions = Readonly<{ width: number; height: number }>;
function requireValue(condition: boolean, message: string): asserts condition {
  if (!condition) throw new Error(`Camera observation: ${message}.`);
}
export function jpegImageDimensions(jpeg: Uint8Array): ImageDimensions {
  requireValue(jpeg instanceof Uint8Array && jpeg.length >= 4 && jpeg[0] === 0xff && jpeg[1] === 0xd8 &&
    jpeg[jpeg.length - 2] === 0xff && jpeg[jpeg.length - 1] === 0xd9, "incomplete JPEG");
  let offset = 2;
  let found: ImageDimensions | null = null;
  while (offset < jpeg.length - 2) {
    requireValue(jpeg[offset++] === 0xff, "invalid JPEG marker");
    while (jpeg[offset] === 0xff) offset++;
    const marker = jpeg[offset++];
    requireValue(marker !== undefined && marker !== 0 && marker !== 0xd8 && marker !== 0xd9, "invalid JPEG marker");
    if (marker === 0x01 || (marker >= 0xd0 && marker <= 0xd7)) continue;
    requireValue(offset + 2 <= jpeg.length - 2, "truncated JPEG segment");
    const length = jpeg[offset] * 256 + jpeg[offset + 1];
    requireValue(length >= 2 && offset + length <= jpeg.length - 2, "invalid JPEG segment length");
    if ([0xc0, 0xc1, 0xc2].includes(marker)) {
      requireValue(!found && length >= 8 && jpeg[offset + 2] === 8 &&
        jpeg[offset + 7] > 0 && length === 8 + 3 * jpeg[offset + 7], "invalid or unsupported JPEG dimensions header");
      const height = jpeg[offset + 3] * 256 + jpeg[offset + 4], width = jpeg[offset + 5] * 256 + jpeg[offset + 6];
      requireValue(height > 0 && width > 0, "image dimensions must be positive finite integers");
      found = Object.freeze({ height, width });
    }
    if (marker === 0xda) {
      requireValue(found !== null, "JPEG has no supported dimensions header"); return found;
    }
    offset += length;
  }
  throw new Error("Camera observation: JPEG has no scan header.");
}
