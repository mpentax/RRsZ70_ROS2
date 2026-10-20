import { createHash, randomUUID } from "node:crypto";
import { constants } from "node:fs";
import { link, open, unlink } from "node:fs/promises";
import { join } from "node:path";
import { MAX_MAP_BYTES } from "./photo-protocol.ts";
import { ensurePrivateDir, hasCode, resolveStateDir, syncPrivateDir, writePrivateTempFile } from "./secret-store.ts";

/** Bounded, read-only V1 map geometry summary. */
export function summarizeV1Map(raw: Buffer) {
  if (raw.length < 40 || raw.length > MAX_MAP_BYTES || raw.toString("ascii", 0, 2) !== "rr" ||
      raw.readUInt16LE(2) !== 20 || raw.readUInt32LE(4) !== raw.length - 20 ||
      !createHash("sha1").update(raw.subarray(0, -20)).digest().equals(raw.subarray(-20))) {
    throw new Error("Invalid V1 map integrity");
  }
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
  const roomSegment = (pixel: number) => pixel !== 255 && pixel !== 7 && (pixel & 7) === 7 ? pixel >> 3 : null;
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
        if ((pixel & 7) === 1) walls++;
        if ((pixel & 7) === 7) floor++;
        const seg = roomSegment(pixel);
        if (seg !== null) rooms.add(seg);
      }
      image = { left, top, width, height, metersPerPixel: 0.05, walls, floor, pixels: length,
        roomSegments: [...rooms].sort((a, b) => a - b) };
      imagePixelData = raw.subarray(data, data + length);
    }
    offset += headerSize + length;
  }
  if (!digestSeen || !image || !imagePixelData) throw new Error("Incomplete V1 map block sequence");
  let embeddedVacuumRoomSegment: number | null = null;
  if (vacuumPosition) {
    const x = Math.floor(vacuumPosition.x / 50 - image.left);
    const y = Math.floor(vacuumPosition.y / 50 - image.top);
    if (x >= 0 && y >= 0 && x < image.width && y < image.height) {
      embeddedVacuumRoomSegment = roomSegment(imagePixelData[x + image.width * y]);
    }
  }
  return { blocks, image, vacuumPosition, chargerPosition, embeddedVacuumRoomSegment };
}

export interface PrivateMapMetadata {
  version: 1;
  deviceId: string;
  provenance: "get_map_v1 / local L01 request + correlated MQTT reply";
  savedAt: number;
  mapIndex: number;
  mapSequence: number;
  bytes: number;
  sha256: string;
}

export function mapMetadataFromRead(deviceId: string, read: {
  bytes: number; sha256: string; mapIndex: number; mapSequence: number;
}, savedAt = Date.now()): PrivateMapMetadata {
  return validatedMapMetadata({
    version: 1, deviceId, sha256: read.sha256,
    provenance: "get_map_v1 / local L01 request + correlated MQTT reply",
    savedAt, mapIndex: read.mapIndex, mapSequence: read.mapSequence, bytes: read.bytes,
  });
}

export function validatedMapMetadata(value: unknown): PrivateMapMetadata {
  if (!value || typeof value !== "object" || Array.isArray(value)) throw new Error("Invalid private map metadata.");
  const item = value as Record<string, unknown>;
  if (Object.keys(item).sort().join(",") !== "bytes,deviceId,mapIndex,mapSequence,provenance,savedAt,sha256,version" ||
    item.version !== 1 || typeof item.sha256 !== "string" || !/^[a-f0-9]{64}$/.test(item.sha256) || typeof item.deviceId !== "string" || !item.deviceId || item.deviceId.length > 128 ||
    item.provenance !== "get_map_v1 / local L01 request + correlated MQTT reply" ||
    ![item.savedAt, item.mapIndex, item.mapSequence, item.bytes].every(Number.isSafeInteger) ||
    (item.savedAt as number) <= 0 || (item.mapIndex as number) < 0 || (item.mapSequence as number) < 0 ||
    (item.bytes as number) <= 0 || (item.bytes as number) > MAX_MAP_BYTES) throw new Error("Invalid private map metadata.");
  return item as unknown as PrivateMapMetadata;
}

export async function readPrivateMapFile(path: string, expectedBytes: number): Promise<Buffer> {
  const file = await open(path, constants.O_RDONLY | constants.O_NOFOLLOW | constants.O_NONBLOCK);
  try {
    const stat = await file.stat();
    if (!stat.isFile() || stat.nlink !== 1 || (stat.mode & 0o7777) !== 0o600 || stat.size !== expectedBytes ||
        stat.size > MAX_MAP_BYTES) throw new Error("Unsafe private map file.");
    const bytes = Buffer.alloc(stat.size);
    let total = 0;
    while (total < bytes.length) {
      const { bytesRead } = await file.read(bytes, total, bytes.length - total, null);
      if (!bytesRead) throw new Error("Truncated private map file.");
      total += bytesRead;
    }
    return bytes;
  } finally { await file.close(); }
}

/** Save a verified V1 map binary into the private state directory. */
export async function savePrivateV1Map(raw: Buffer, stateDirectory: string): Promise<{ sha256: string; bytes: number; saved: boolean }> {
  // Validate before touching disk; never print raw bytes, geometry, filesystem errors, or private paths.
  if (!Buffer.isBuffer(raw) || raw.length > MAX_MAP_BYTES) throw new Error("Invalid private V1 map.");
  try { summarizeV1Map(raw); } catch { throw new Error("Invalid private V1 map."); }
  const sha256 = createHash("sha256").update(raw).digest("hex");
  const root = resolveStateDir(stateDirectory, "Cannot save private V1 map.");
  const directory = join(root, "maps");
  const destination = join(directory, `v1-${sha256}.bin`);
  const temporary = join(directory, `.v1-${randomUUID()}.tmp`);
  try {
    await ensurePrivateDir(root, "Unsafe private map directory.");
    await ensurePrivateDir(directory, "Unsafe private map directory.");
    let saved = false;
    await writePrivateTempFile(temporary, raw);
    try { await link(temporary, destination); saved = true; }
    catch (error) {
      if (!hasCode(error, "EEXIST")) throw error;
      const bytes = await readPrivateMapFile(destination, raw.length);
      if (!bytes.equals(raw)) throw new Error("Private map hash collision or changed file.");
    }
    await unlink(temporary);
    await syncPrivateDir(directory);
    return { sha256, bytes: raw.length, saved };
  } catch { throw new Error("Cannot save private V1 map."); }
  finally { try { await unlink(temporary); } catch { /* absent or cleanup failure cannot expose map bytes */ } }
}

export function syntheticV1Image(pixels = Buffer.from([1, 31, 255, 0]), width = 2, height = 2): Buffer {
  const header = Buffer.alloc(20); header.write("rr"); header.writeUInt16LE(20, 2);
  const image = Buffer.alloc(24 + pixels.length); image.writeUInt16LE(2); image.writeUInt16LE(24, 2);
  image.writeUInt32LE(pixels.length, 4); image.writeUInt32LE(height, 16); image.writeUInt32LE(width, 20);
  pixels.copy(image, 24);
  const position = Buffer.alloc(20); position.writeUInt16LE(8); position.writeUInt16LE(8, 2);
  position.writeUInt32LE(12, 4); position.writeInt32LE(123456, 8); position.writeInt32LE(-7777, 12);
  const digest = Buffer.alloc(28); digest.writeUInt16LE(1024); digest.writeUInt16LE(8, 2); digest.writeUInt32LE(20, 4);
  const raw = Buffer.concat([header, image, position, digest]); raw.writeUInt32LE(raw.length - 20, 4);
  createHash("sha1").update(raw.subarray(0, -20)).digest().copy(raw, raw.length - 20);
  return raw;
}
