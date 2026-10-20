// MQTT auth and V1 envelopes follow go2rtc's Roborock driver; see THIRD_PARTY_NOTICES.
import { createHash, randomInt, randomUUID } from "node:crypto";
import { connectAsync, type IClientOptions } from "mqtt";
import { localProtocolCodec } from "./local.ts";
import { RpcError } from "./robot.ts";
import { defaultRpcTimeoutMs, PRIORITY_STOPS, TransportError } from "./rpc-outcome.ts";
import type { CameraAccount } from "./cloud.ts";
import type { SavedDevice } from "./secret-store.ts";

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
