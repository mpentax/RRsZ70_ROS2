import { MAX_MAP_BYTES } from "./photo-protocol.ts";

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
  return validatedMapMetadata({ version: 1, deviceId, sha256: read.sha256,
    provenance: "get_map_v1 / local L01 request + correlated MQTT reply",
    savedAt, mapIndex: read.mapIndex, mapSequence: read.mapSequence, bytes: read.bytes });
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
