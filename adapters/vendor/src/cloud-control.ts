// MQTT auth, V1 envelopes, and protocol-301 map reads follow go2rtc/python-roborock; see THIRD_PARTY_NOTICES.
import { createDecipheriv, createHash, randomBytes, randomInt, randomUUID } from "node:crypto";
import { gunzipSync } from "node:zlib";
import { connectAsync, type IClientOptions } from "mqtt";
import { defaultRpcTimeoutMs, localProtocolCodec, PRIORITY_STOPS, RobotClient, RpcError, TransportError } from "./robot.ts";
import type { CameraAccount } from "./cloud.ts";
import { MAX_MAP_BYTES, summarizeV1Map, type SavedDevice } from "./secret-store.ts";

export { MAX_MAP_BYTES } from "./secret-store.ts";

export interface CloudControlCredentials {
  broker: string; user: string; secret: string; accountKey: string; deviceId: string; localKey: string;
}

function record(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value);
}

/** Credentials originate from the authenticated service account, never go2rtc. */
export function cloudControlCredentials(account: CameraAccount, device: SavedDevice): CloudControlCredentials {
  const endpoint = new URL(account.mqttUrl);
  if (!["ssl:", "mqtts:"].includes(endpoint.protocol)) throw new TransportError("Invalid authenticated broker", "not_sent");
  const credentials = { broker: account.mqttUrl.replace(/^(ssl|mqtts):/, "mqtts:"),
    user: account.user, secret: account.secret, accountKey: account.key, deviceId: device.duid, localKey: device.localKey };
  mqttOptions(credentials);
  return credentials;
}

export function mqttOptions(credentials: CloudControlCredentials) {
  let endpoint: URL;
  try { endpoint = new URL(credentials.broker); } catch { throw new Error("Invalid cloud broker."); }
  if (endpoint.protocol !== "mqtts:" || !/^mqtt(?:-[a-z0-9]+(?:-[0-9]+)?)?\.roborock\.com$/.test(endpoint.hostname) ||
    !endpoint.port || endpoint.username || endpoint.password || endpoint.search || endpoint.hash ||
    (endpoint.pathname !== "" && endpoint.pathname !== "/") ||
    !/^[a-zA-Z0-9_-]+$/.test(credentials.user) || !/^[a-zA-Z0-9_-]+$/.test(credentials.deviceId) ||
    !credentials.secret || !credentials.accountKey || Buffer.byteLength(credentials.localKey) !== 16) {
    throw new Error("Invalid or untrusted cloud control credentials.");
  }
  const username = createHash("md5").update(`${credentials.user}:${credentials.accountKey}`).digest().subarray(1, 5).toString("hex");
  const password = createHash("md5").update(`${credentials.secret}:${credentials.accountKey}`).digest().subarray(8).toString("hex");
  return {
    username, password,
    incoming: `rr/m/o/${credentials.user}/${username}/${credentials.deviceId}`,
    outgoing: `rr/m/i/${credentials.user}/${username}/${credentials.deviceId}`,
  };
}

export interface CloudRpcSecurity {
  endpoint: string;
  nonce: Buffer;
}

export function encodeV1MqttFrame(localKey: string, protocol: number, plain: Buffer, timestamp: number, sequence = 0, random = 0): Buffer {
  const encrypted = localProtocolCodec.encryptPayload(plain, {
    version: "1.0", localKey, timestamp, sequence, random, connectNonce: 0, ackNonce: 0,
  });
  const frame = Buffer.alloc(23 + encrypted.length);
  frame.write("1.0");
  frame.writeUInt32BE(sequence, 3); frame.writeUInt32BE(random, 7); frame.writeUInt32BE(timestamp, 11);
  frame.writeUInt16BE(protocol, 15); frame.writeUInt16BE(encrypted.length, 17); encrypted.copy(frame, 19);
  frame.writeUInt32BE(localProtocolCodec.crc32(frame.subarray(0, -4)), frame.length - 4);
  return frame;
}

export function validV1MqttFrame(frame: Buffer, maxBytes = 65558): boolean {
  return Buffer.isBuffer(frame) && frame.length >= 23 && frame.length <= maxBytes &&
    frame.toString("ascii", 0, 3) === "1.0" &&
    frame.readUInt16BE(17) + 23 === frame.length &&
    localProtocolCodec.crc32(frame.subarray(0, -4)) === frame.readUInt32BE(frame.length - 4);
}

export function decryptV1MqttPayload(frame: Buffer, localKey: string): Buffer {
  return localProtocolCodec.decryptPayload(frame.subarray(19, -4), {
    version: "1.0", localKey, timestamp: frame.readUInt32BE(11),
    sequence: frame.readUInt32BE(3), random: frame.readUInt32BE(7),
    connectNonce: 0, ackNonce: 0,
  });
}

/** Optional security data is only for V1 binary-response reads such as get_map_v1. */
export function encodeCloudRpc(
  localKey: string, id: number, method: string, params: unknown, timestamp: number, security?: CloudRpcSecurity,
): Buffer {
  if (security && (!/^[A-Za-z0-9+/]{8}$/.test(security.endpoint) ||
      !Buffer.isBuffer(security.nonce) || security.nonce.length !== 16)) {
    throw new Error("Invalid cloud RPC security.");
  }
  const request = { id, method, params, ...(security
    ? { security: { endpoint: security.endpoint, nonce: security.nonce.toString("hex") } }
    : {}) };
  const plain = Buffer.from(JSON.stringify({ dps: { "101": JSON.stringify(request) }, t: timestamp }));
  if (plain.length > 16000) throw new Error("Cloud RPC payload is too large.");
  return encodeV1MqttFrame(localKey, 101, plain, timestamp, id, timestamp);
}

export function decodeCloudRpc(localKey: string, frame: Buffer): { id: number; result: unknown; errorCode?: number } | undefined {
  if (!validV1MqttFrame(frame)) throw new Error("Invalid cloud response frame.");
  if (frame.readUInt16BE(15) !== 102) return undefined;
  try {
    const plain = decryptV1MqttPayload(frame, localKey);
    const outer: unknown = JSON.parse(new TextDecoder("utf-8", { fatal: true }).decode(plain));
    if (!record(outer) || !record(outer.dps) || typeof outer.dps["102"] !== "string") return undefined;
    const inner: unknown = JSON.parse(outer.dps["102"]);
    if (!record(inner) || typeof inner.id !== "number" || !Number.isSafeInteger(inner.id)) return undefined;
    if (inner.error !== undefined && inner.error !== null) {
      return { id: inner.id, result: undefined,
        errorCode: record(inner.error) && typeof inner.error.code === "number" && Number.isSafeInteger(inner.error.code) ? inner.error.code : -1 };
    }
    if (!Object.hasOwn(inner, "result")) throw new Error("Missing result.");
    return { id: inner.id, result: inner.result };
  } catch { throw new Error("Cloud response could not be decrypted or decoded."); }
}

type Pending = {
  resolve: (value: unknown) => void; reject: (error: Error) => void; timer: ReturnType<typeof setTimeout>;
};

export interface CloudMqttPeer {
  readonly connected: boolean;
  on(event: "error" | "close", listener: () => void): unknown;
  on(event: "message", listener: (topic: string, frame: Buffer) => void): unknown;
  subscribe(topic: string, options: { qos: 0 }, callback: (error?: Error | null, granted?: { qos: number }[]) => void): unknown;
  publish(topic: string, payload: Buffer, options: { qos: 0; retain: false }, callback: (error?: Error) => void): unknown;
  end(force: boolean): unknown;
}

export class CloudControlClient {
  #credentials: CloudControlCredentials;
  #auth: ReturnType<typeof mqttOptions>;
  #mqtt?: CloudMqttPeer;
  #connector: (url: string, options: IClientOptions) => Promise<CloudMqttPeer>;
  #pending = new Map<number, Pending>();
  #id = randomInt(100_000, 0x3fffffff);
  #connectedAt: number | null = null;
  #connecting?: Promise<void>;
  #closed = false;

  constructor(credentials: CloudControlCredentials,
    connector: (url: string, options: IClientOptions) => Promise<CloudMqttPeer> = connectAsync) {
    this.#auth = mqttOptions(credentials);
    this.#credentials = { ...credentials };
    this.#connector = connector;
  }

  get connection() {
    return { state: this.#connectedAt !== null ? "connected" : "disconnected", connectedAt: this.#connectedAt,
      protocol: "cloud-mqtt-v1" };
  }

  connect(): Promise<void> {
    if (this.#closed) return Promise.reject(new Error("Cloud client is closed."));
    if (this.#connecting) return this.#connecting;
    this.#connecting = this.#connect().finally(() => { this.#connecting = undefined; });
    return this.#connecting;
  }

  async #connect(): Promise<void> {
    if (this.#connectedAt !== null) return;
    const auth = this.#auth;
    try {
      const client = await this.#connector(this.#credentials.broker, {
        username: auth.username, password: auth.password,
        clientId: `roboclaw-${randomUUID()}`, protocolVersion: 4, clean: true,
        connectTimeout: 5000, reconnectPeriod: 0, queueQoSZero: false, resubscribe: false, keepalive: 20,
      });
      this.#mqtt = client;
      client.on("error", () => this.#mqtt === client && this.#fail(new Error("Roborock cloud connection failed.")));
      client.on("close", () => this.#mqtt === client && this.#fail(new Error("Roborock cloud connection closed; commands were not replayed.")));
      client.on("message", (topic, frame) => {
        if (this.#mqtt !== client || topic !== auth.incoming || this.#closed) return;
        try {
          const response = decodeCloudRpc(this.#credentials.localKey, frame);
          if (!response) return;
          const pending = this.#pending.get(response.id);
          if (!pending) return;
          this.#pending.delete(response.id); clearTimeout(pending.timer);
          if (response.errorCode !== undefined) pending.reject(new RpcError(response.errorCode));
          else pending.resolve(response.result);
        } catch { this.#fail(new Error("Cloud RPC decoding failed; outcome is unknown.")); }
      });
      await new Promise<void>((resolve, reject) => {
        const timer = setTimeout(() => reject(new Error("Cloud subscription timed out.")), 5000);
        client.subscribe(auth.incoming, { qos: 0 }, (error, granted) => {
          clearTimeout(timer);
          if (error || !granted?.length || granted.some((item) => item.qos === 128)) reject(new Error("Cloud subscription rejected."));
          else resolve();
        });
      });
      if (this.#closed || !client.connected) { client.end(true); throw new Error("Cloud connection unavailable."); }
      this.#connectedAt = Date.now();
    } catch {
      this.#fail(new Error("Could not connect to Roborock cloud control."));
      throw new Error("Could not connect to Roborock cloud control.");
    }
  }

  request(method: string, params: unknown = []): Promise<unknown> { return this.rpc(method, params); }
  async rpc(method: string, params: unknown = [], timeoutMs: number = defaultRpcTimeoutMs(method)): Promise<unknown> {
    if (!/^[A-Za-z][A-Za-z0-9_]{0,127}$/.test(method) || !Number.isInteger(timeoutMs) || timeoutMs < 1 || timeoutMs > 120000)
      throw new TransportError("Invalid RPC input", "not_sent");
    const deadline=performance.now()+timeoutMs;
    try { await this.connect(); } catch { throw new TransportError("Cloud connection failed", "not_sent"); }
    const remaining=Math.floor(deadline-performance.now());
    if (remaining<=0) throw new TransportError('Cloud RPC timed out before dispatch','not_sent');
    return this.#send(method, params, remaining);
  }

  #send(method: string, params: unknown, timeoutMs: number): Promise<unknown> {
    const client = this.#mqtt;
    if (this.#closed || this.#connectedAt === null || !client?.connected) {
      return Promise.reject(new TransportError("Cloud connection unavailable", "not_sent"));
    }
    const priority = PRIORITY_STOPS.has(method);
    if (this.#pending.size >= (priority ? 8 : 4)) return Promise.reject(new TransportError("Cloud transport capacity exceeded", "not_sent"));
    const id = ++this.#id;
    let frame: Buffer;
    try { frame = encodeCloudRpc(this.#credentials.localKey, id, method, params, Math.floor(Date.now() / 1000)); }
    catch { return Promise.reject(new TransportError("Invalid cloud RPC payload", "not_sent")); }
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => this.#fail(new Error("Cloud RPC timed out; command outcome is unknown.")), timeoutMs);
      this.#pending.set(id, { resolve, reject, timer });
      try { client.publish(this.#auth.outgoing, frame, { qos: 0, retain: false }, (error) => {
        if (error && this.#mqtt === client) this.#fail(new Error("Cloud publish failed; command outcome is unknown."));
      }); } catch { this.#fail(new Error("Cloud publish failed; command outcome is unknown.")); }
    });
  }

  #fail(error: Error): void {
    this.#connectedAt = null;
    for (const pending of this.#pending.values()) { clearTimeout(pending.timer); pending.reject(new TransportError(error.message, "unknown")); }
    this.#pending.clear();
    const client = this.#mqtt; this.#mqtt = undefined;
    client?.end(true);
  }

  close(): void {
    this.#closed = true;
    this.#fail(new Error("Cloud control client closed."));
  }
}

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
