// Wire format references (independent, bounded TypeScript implementation):
// https://github.com/Python-roborock/python-roborock/blob/6397ace9f5d533ffb82230289a51c13d82853121/roborock/protocols/v1_protocol.py
import { createDecipheriv, createHash, randomBytes } from "node:crypto";
import { gunzipSync } from "node:zlib";

export const MAX_MAP_BYTES = 32 * 1024 * 1024;

export class PhotoProtocolError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "PhotoProtocolError";
  }
}

export interface BlobSecurity {
  endpoint: string;
  nonce: Buffer;
}

/** Account RRiot.k, not the robot local key. Do not log the returned security data. */
export function createBlobSecurity(accountKey: string): BlobSecurity {
  if (typeof accountKey !== "string" || !accountKey || Buffer.byteLength(accountKey) > 8192) {
    throw new PhotoProtocolError("Invalid blob account key");
  }
  return {
    endpoint: createHash("md5").update(accountKey).digest().subarray(8, 14).toString("base64"),
    nonce: randomBytes(16),
  };
}

function validSecurity(security: BlobSecurity): void {
  if (!/^[A-Za-z0-9+/]{8}$/.test(security.endpoint) ||
      !Buffer.isBuffer(security.nonce) || security.nonce.length !== 16) {
    throw new PhotoProtocolError("Invalid blob security context");
  }
}

function decompress(bytes: Buffer, limit: number): Buffer {
  try {
    return gunzipSync(bytes, { maxOutputLength: limit });
  } catch (error: unknown) {
    if (error instanceof Error && "code" in error &&
        ["Z_DATA_ERROR", "Z_BUF_ERROR", "Z_MEM_ERROR", "ERR_BUFFER_TOO_LARGE"].includes(String(error.code))) {
      throw new PhotoProtocolError("Invalid or oversized compressed binary response");
    }
    throw error;
  }
}

/** Map protocol-301 envelope: AES-128-CBC using the request nonce and zero IV, followed by gzip. */
export function decodeMapResponse(
  protocol: number, payload: Buffer, security: BlobSecurity, expectedRequestId?: number,
): { requestId: number; data: Buffer } | undefined {
  validSecurity(security);
  if (protocol !== 301 || !Buffer.isBuffer(payload) ||
      !payload.subarray(0, 8).equals(Buffer.from(security.endpoint, "ascii"))) return undefined;
  if (payload.length < 40 || payload.length > MAX_MAP_BYTES + 24 ||
      (payload.length - 24) % 16 !== 0) {
    throw new PhotoProtocolError("Invalid map response frame size");
  }
  const requestId = payload.readUInt16LE(16);
  if (expectedRequestId !== undefined && requestId !== expectedRequestId) return undefined;
  let compressed: Buffer;
  try {
    const decipher = createDecipheriv("aes-128-cbc", security.nonce, Buffer.alloc(16));
    compressed = Buffer.concat([decipher.update(payload.subarray(24)), decipher.final()]);
  } catch (error: unknown) {
    if (error instanceof Error && "code" in error && error.code === "ERR_OSSL_BAD_DECRYPT") {
      throw new PhotoProtocolError("Unable to decrypt map response");
    }
    throw error;
  }
  return { requestId, data: decompress(compressed, MAX_MAP_BYTES) };
}
