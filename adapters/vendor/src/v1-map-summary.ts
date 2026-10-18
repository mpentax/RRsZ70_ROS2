import { createHash } from "node:crypto";

/** Bounded, read-only V1 map geometry summary. No planner coordinates are emitted. */
export function summarizeV1Map(raw: Buffer) {
  if (raw.length < 40 || raw.length > 32 * 1024 * 1024 || raw.toString("ascii", 0, 2) !== "rr" ||
      raw.readUInt16LE(2) !== 20 || raw.readUInt32LE(4) !== raw.length - 20 ||
      !createHash("sha1").update(raw.subarray(0, -20)).digest().equals(raw.subarray(-20))) {
    throw new Error("Invalid V1 map integrity");
  }
  // The trailing digest is itself block 1024 (8-byte header + 20 bytes).
  const end = raw.length;
  const blocks: Record<string, number> = {};
  const rooms = new Set<number>();
  let image: { left: number; top: number; width: number; height: number; metersPerPixel: number; walls: number; floor: number; pixels: number; roomSegments: number[] } | undefined;
  let vacuumPosition: { x: number; y: number; angle?: number } | undefined;
  let chargerPosition: { x: number; y: number; angle?: number } | undefined;
  const position = (start: number, length: number) => {
    if (length !== 8 && length !== 12) throw new Error("Invalid V1 position block");
    return { x: raw.readInt32LE(start), y: raw.readInt32LE(start + 4),
      ...(length === 12 ? { angle: raw.readInt32LE(start + 8) } : {}) };
  };
  let blockCount = 0;
  let digestSeen = false;
  let imagePixelData: Buffer | undefined;
  for (let offset = 20; offset < end;) {
    if (++blockCount > 1024 || offset + 8 > end) throw new Error("Invalid V1 map block count");
    const type = raw.readUInt16LE(offset), headerSize = raw.readUInt16LE(offset + 2), length = raw.readUInt32LE(offset + 4);
    if (headerSize < 8 || headerSize > 256 || offset + headerSize + length > end) throw new Error("Invalid V1 map block extent");
    const data = offset + headerSize;
    blocks[String(type)] = (blocks[String(type)] ?? 0) + 1;
    if (type === 1024) {
      if (length !== 20 || data + length !== end) throw new Error("Invalid V1 digest block");
      digestSeen = true;
      break;
    }
    if (type === 1) chargerPosition = position(data, length);
    if (type === 8) vacuumPosition = position(data, length);
    if (type === 2) {
      if (image) throw new Error("Duplicate V1 image block");
      if (headerSize < 24) throw new Error("Invalid V1 image header");
      const left = raw.readInt32LE(offset + headerSize - 12), top = raw.readInt32LE(offset + headerSize - 16);
      const height = raw.readUInt32LE(offset + headerSize - 8), width = raw.readUInt32LE(offset + headerSize - 4);
      if (!width || !height || width > 1024 || height > 1024 || width * height !== length) throw new Error("Invalid V1 image size");
      let walls = 0, floor = 0;
      for (let i = data; i < data + length; i++) {
        const pixel = raw[i];
        if (pixel === 1 || (pixel !== 0 && pixel !== 255 && pixel !== 7 && (pixel & 7) === 1)) walls++;
        if (pixel === 255 || pixel === 7 || (pixel & 7) === 7) floor++;
        if (pixel !== 255 && pixel !== 7 && (pixel & 7) === 7) rooms.add(pixel >> 3);
      }
      image = { left, top, width, height, metersPerPixel: 0.05, walls, floor, pixels: length,
        roomSegments: [...rooms].sort((a, b) => a - b) };
      imagePixelData = raw.subarray(data, data + length);
    }
    offset += headerSize + length;
  }
  if (!digestSeen || !image || !imagePixelData) throw new Error("Incomplete V1 map block sequence");
  const getVacuumRoom = (): number | null => {
    if (!vacuumPosition || !image || !imagePixelData) return null;
    // The extracted Android plugin indexes raw row-major image data using
    // Math.floor(grid coordinate minus image origin), not the flipped PNG.
    // Keep the floor operation AFTER subtracting the origin: for fractional
    // origins, floor(x) - origin need not itself be an integer image index.
    const x = Math.floor(vacuumPosition.x / 50 - image.left);
    const y = Math.floor(vacuumPosition.y / 50 - image.top);
    if (x < 0 || y < 0 || x >= image.width || y >= image.height) return null;
    const pixel = imagePixelData[x + image.width * y];
    return pixel !== 255 && pixel !== 7 && (pixel & 7) === 7 ? pixel >> 3 : null;
  };
  return { blocks, image, vacuumPosition, chargerPosition,
    embeddedVacuumRoomSegment: getVacuumRoom(),
    positionIsIndependentFreshTelemetry: false,
    coordinateConvention: "Android plugin raw-image indexing: floor(position / 50 - image origin), row-major; orientation and independent localization require validation",
  };
}

/** A verified V1 image block, without interpreting embedded positions as live telemetry.
 * The caller must keep both this buffer and its pixels private.
 */
export function verifiedV1Image(raw: Buffer): { width: number; height: number; pixels: Buffer } {
  // Reuse the strict digest, block extent and image-size checks above; do not
  // accept a merely plausible header from an unverified cache file.
  const summary = summarizeV1Map(raw);
  let offset = 20;
  while (offset < raw.length) {
    const type = raw.readUInt16LE(offset);
    const headerSize = raw.readUInt16LE(offset + 2);
    const length = raw.readUInt32LE(offset + 4);
    if (type === 2) return {
      width: summary.image.width, height: summary.image.height,
      pixels: raw.subarray(offset + headerSize, offset + headerSize + length),
    };
    offset += headerSize + length;
  }
  throw new Error("Incomplete V1 map block sequence");
}
