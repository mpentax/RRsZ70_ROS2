import { createHash, randomUUID } from "node:crypto";
import { constants } from "node:fs";
import { link, open, unlink } from "node:fs/promises";
import { join } from "node:path";
import { MAX_MAP_BYTES } from "./photo-protocol.ts";
import { ensurePrivateDir, hasCode, resolveStateDir, syncPrivateDir, writePrivateTempFile } from "./secret-store.ts";
import { summarizeV1Map } from "./v1-map-summary.ts";

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
