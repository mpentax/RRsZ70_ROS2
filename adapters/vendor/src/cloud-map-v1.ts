import { type BlobSecurity, decodeMapResponse, MAX_MAP_BYTES } from "./photo-protocol.ts";
import { decryptV1MqttPayload, validV1MqttFrame } from "./cloud-control.ts";

/** Largest complete MQTT V1 frame accepted by the map probe. */
export const MAX_CLOUD_MAP_FRAME_BYTES = MAX_MAP_BYTES + 47;

/**
 * Validates the V1 protocol-301 envelope, decrypts its outer ECB payload
 * with the device local key, then delegates nonce-bound CBC/gzip map decoding.
 */
export function decodeCloudMapResponse(
  frame: Buffer, localKey: string, security: BlobSecurity, expectedRequestId: number,
): { requestId: number; data: Buffer } | undefined {
  if (!validV1MqttFrame(frame, MAX_CLOUD_MAP_FRAME_BYTES)) throw new Error("Invalid cloud map response frame.");
  if (frame.readUInt16BE(15) !== 301) return undefined;
  return decodeMapResponse(301, decryptedPayload(frame, localKey), security, expectedRequestId);
}

function decryptedPayload(frame: Buffer, localKey: string): Buffer {
  try { return decryptV1MqttPayload(frame, localKey); }
  catch { throw new Error("Unable to decrypt outer cloud map payload."); }
}

export function inspectMapFrame(frame: Buffer, localKey: string, security: BlobSecurity, expectedRequestId: number):
  "unrelated" | "correlated-v1" | "opaque-301" | "malformed" {
  if (!validV1MqttFrame(frame, MAX_CLOUD_MAP_FRAME_BYTES)) return "malformed";
  if (frame.readUInt16BE(15) !== 301) return "unrelated";
  let payload: Buffer;
  try { payload = decryptedPayload(frame, localKey); }
  catch { return "opaque-301"; }
  if (payload.length < 24 || !payload.subarray(0, 8).equals(Buffer.from(security.endpoint, "ascii"))) return "opaque-301";
  return payload.readUInt16LE(16) === expectedRequestId ? "correlated-v1" : "unrelated";
}
