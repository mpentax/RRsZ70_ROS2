// Wire format references (independent, bounded TypeScript implementation):
// https://github.com/Python-roborock/python-roborock/blob/6397ace9f5d533ffb82230289a51c13d82853121/roborock/protocols/v1_protocol.py
import { createDecipheriv, createHash, randomBytes, randomUUID } from "node:crypto";
import { gunzipSync } from "node:zlib";
import { connectAsync } from "mqtt";
import { cloudControlCredentials, decryptV1MqttPayload, mqttOptions, validV1MqttFrame } from "./cloud-control.ts";
import type { CameraAccount } from "./cloud.ts";
import { summarizeV1Map } from "./private-map-cache.ts";
import { RobotClient } from "./robot.ts";
import type { SavedDevice } from "./secret-store.ts";

export const MAX_MAP_BYTES = 32 * 1024 * 1024;

/** Largest complete MQTT V1 frame accepted by the map probe. */
export const MAX_CLOUD_MAP_FRAME_BYTES = MAX_MAP_BYTES + 47;

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

const TIMEOUT_MS = 12_000;

/** Read the V1 LiDAR map via local L01 request and correlated MQTT 301 reply. */
export async function readSplitMap(device: SavedDevice, account: CameraAccount): Promise<{
  raw: Buffer; bytes: number; sha256: string; mapIndex: number; mapSequence: number; receivedAt: number;
  geometry: ReturnType<typeof summarizeV1Map>;
}> {
  if (device.model !== "roborock.vacuum.a179") throw new Error("Unsupported map device.");
  const credentials = cloudControlCredentials(account, device);
  const auth = mqttOptions(credentials);
  const security = createBlobSecurity(credentials.accountKey);
  const client = await connectAsync(credentials.broker, {
    username: auth.username, password: auth.password,
    clientId: `roboclaw-map-split-${randomUUID()}`,
    protocolVersion: 4, clean: true, reconnectPeriod: 0, queueQoSZero: false,
    resubscribe: false, connectTimeout: 4000, keepalive: 20,
  });
  const robot = new RobotClient({ host: device.host, localKey: device.localKey, protocol: "L01" });
  let reason: string = "no-map-frame";
  let requestId: number | null = null;
  let receivedFrames = 0;
  let opaqueFrames = 0;
  let undecodableFrames = 0;
  let map: Buffer | null = null;
  let receivedAt = 0;
  let changed: () => void = () => {};
  let active = true;
  const wake = () => changed();
  client.on("error", wake);
  client.on("close", wake);
  client.on("message", (topic, frame) => {
    if (!active || topic !== auth.incoming || requestId === null) return;
    if (++receivedFrames > 128) { reason = "setup-failed"; client.end(true); return; }
    if (frame.length > MAX_CLOUD_MAP_FRAME_BYTES) { reason = "setup-failed"; client.end(true); return; }
    const classification = inspectMapFrame(frame, credentials.localKey, security, requestId);
    if (classification === "opaque-301") {
      if (reason === "no-map-frame") reason = "opaque-301-unbound";
      opaqueFrames++;
    } else if (classification === "correlated-v1") {
      try {
        const decoded = decodeCloudMapResponse(frame, credentials.localKey, security, requestId);
        if (decoded) { map = decoded.data; receivedAt = Date.now(); }
      } catch { reason = "correlated-map-undecodable"; undecodableFrames++; }
    }
    wake();
  });
  try {
    await new Promise<void>((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error("setup-failed")), 4000);
      client.subscribe(auth.incoming, { qos: 0 }, (error, granted) => {
        clearTimeout(timer);
        if (error || granted?.length !== 1 || granted[0]?.qos === 128) reject(new Error("setup-failed"));
        else resolve();
      });
    });
    if (!client.connected) throw new Error("setup-failed");
    await robot.connect();
    const read = robot.requestMapWithSecurity(security);
    requestId = read.requestId;
    const deadline = Date.now() + TIMEOUT_MS;
    let response: { id: number; response: unknown };
    try { response = await read.result; }
    catch { throw new Error("local-not-acknowledged"); }
    if (!Array.isArray(response.response) || response.response.length !== 1 || response.response[0] !== "ok") {
      throw new Error("local-not-acknowledged");
    }
    while (!map && client.connected && Date.now() < deadline) {
      await new Promise<void>((resolve) => {
        const timer = setTimeout(resolve, Math.max(1, deadline - Date.now()));
        changed = () => { clearTimeout(timer); resolve(); };
      });
      changed = () => {};
    }
    if (!map) {
      if (undecodableFrames > 0) throw new Error("correlated-map-undecodable");
      if (opaqueFrames > 0) throw new Error(`opaque-301-unbound:${opaqueFrames}`);
      throw new Error(reason);
    }
    const bytes: Buffer = map;
    let geometry: ReturnType<typeof summarizeV1Map>;
    try { geometry = summarizeV1Map(bytes); }
    catch { throw new Error("invalid-map-format"); }
    return {
      bytes: bytes.length, sha256: createHash("sha256").update(bytes).digest("hex"),
      mapIndex: bytes.readUInt32LE(12), mapSequence: bytes.readUInt32LE(16), receivedAt,
      geometry, raw: bytes,
    };
  } finally {
    active = false; robot.close(); client.end(true);
  }
}
