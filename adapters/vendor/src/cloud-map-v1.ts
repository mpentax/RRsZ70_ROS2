import { type BlobSecurity, decodeMapResponse, MAX_MAP_BYTES } from "./photo-protocol.ts";
import { localProtocolCodec } from "./local.ts";

/** Largest complete MQTT V1 frame accepted by the map probe. */
export const MAX_CLOUD_MAP_FRAME_BYTES = MAX_MAP_BYTES + 47;

function validV1Frame(frame: Buffer): boolean {
  return Buffer.isBuffer(frame) && frame.length >= 23 && frame.length <= MAX_CLOUD_MAP_FRAME_BYTES &&
    frame.toString("ascii", 0, 3) === "1.0" &&
    frame.readUInt16BE(17) + 23 === frame.length &&
    localProtocolCodec.crc32(frame.subarray(0, -4)) === frame.readUInt32BE(frame.length - 4);
}

/**
 * Validates the V1 protocol-301 envelope, decrypts its outer ECB payload
 * with the device local key, then delegates nonce-bound CBC/gzip map decoding.
 */
export function decodeCloudMapResponse(
  frame: Buffer, localKey: string, security: BlobSecurity, expectedRequestId: number,
): { requestId: number; data: Buffer } | undefined {
  if (!validV1Frame(frame)) throw new Error("Invalid cloud map response frame.");
  if (frame.readUInt16BE(15) !== 301) return undefined;
  return decodeMapResponse(301, decryptedPayload(frame, localKey), security, expectedRequestId);
}

function decryptedPayload(frame: Buffer, localKey: string): Buffer {
  if (frame.readUInt16BE(17) % 16 !== 0) throw new Error("Invalid encrypted cloud map payload length.");
  try {
    return localProtocolCodec.decryptPayload(frame.subarray(19, -4), {
      version: "1.0", localKey, timestamp: frame.readUInt32BE(11),
      sequence: frame.readUInt32BE(3), random: frame.readUInt32BE(7),
      connectNonce: 0, ackNonce: 0,
    });
  } catch {
    throw new Error("Unable to decrypt outer cloud map payload.");
  }
}

export function inspectMapFrame(frame: Buffer, localKey: string, security: BlobSecurity, expectedRequestId: number):
  "unrelated" | "correlated-v1" | "opaque-301" | "malformed" {
  if (!validV1Frame(frame)) return "malformed";
  if (frame.readUInt16BE(15) !== 301) return "unrelated";
  let payload: Buffer;
  try { payload = decryptedPayload(frame, localKey); }
  catch { return "opaque-301"; }
  if (payload.length < 24 || !payload.subarray(0, 8).equals(Buffer.from(security.endpoint, "ascii"))) return "opaque-301";
  return payload.readUInt16LE(16) === expectedRequestId ? "correlated-v1" : "unrelated";
}
