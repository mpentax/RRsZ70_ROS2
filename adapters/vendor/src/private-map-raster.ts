import { deflateSync } from "node:zlib";
import { localProtocolCodec } from "./local.ts";
import { verifiedV1Image } from "./v1-map-summary.ts";

const SIGNATURE = Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]);
const MAX_PIXELS = 1024 * 1024;
const MAX_PNG_BYTES = 5 * 1024 * 1024;

function chunk(type: string, data: Buffer): Buffer {
  const label = Buffer.from(type, "ascii");
  const result = Buffer.alloc(12 + data.length);
  result.writeUInt32BE(data.length, 0);
  label.copy(result, 4);
  data.copy(result, 8);
  result.writeUInt32BE(localProtocolCodec.crc32(result.subarray(4, -4)), result.length - 4);
  return result;
}

/** Render only classified occupancy/segment color, never V1 position, metadata, or IDs. */
export function renderPrivateV1Map(raw: Buffer): Buffer {
  const { width, height, pixels } = verifiedV1Image(raw);
  if (width * height > MAX_PIXELS) throw new Error("Invalid private map raster.");
  const rowStride = 1 + width * 3;
  const scanlines = Buffer.alloc(rowStride * height);
  for (let y = 0; y < height; y++) {
    // V1 image storage is row-major with the map's bottom row first.
    const output = (height - 1 - y) * rowStride;
    for (let x = 0; x < width; x++) {
      const pixel = pixels[y * width + x];
      const target = output + 1 + x * 3;
      // Unknown/unmapped is dark; walls are distinct. Segment IDs are never
      // serialized: a fixed, small palette conveys coarse room separation.
      const color = pixel === 1 || (pixel !== 0 && pixel !== 255 && pixel !== 7 && (pixel & 7) === 1)
        ? [67, 76, 86] : pixel === 255 || pixel === 7
          ? [224, 231, 234] : (pixel & 7) === 7
            ? [[181, 213, 224], [206, 207, 234], [220, 211, 184], [194, 219, 200]][(pixel >> 3) % 4]
            : [24, 30, 37];
      scanlines[target] = color[0]; scanlines[target + 1] = color[1]; scanlines[target + 2] = color[2];
    }
  }
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(width); ihdr.writeUInt32BE(height, 4);
  ihdr[8] = 8; ihdr[9] = 2; // 8-bit RGB, no metadata-bearing ancillary chunks
  const png = Buffer.concat([SIGNATURE, chunk("IHDR", ihdr), chunk("IDAT", deflateSync(scanlines)), chunk("IEND", Buffer.alloc(0))]);
  if (png.length > MAX_PNG_BYTES) throw new Error("Private map raster is too large.");
  return png;
}
