// Cloud flow, Hawk signing, MQTT auth, V1 envelopes, and protocol-301 map reads; see THIRD_PARTY_NOTICES.
import { createDecipheriv, createHash, createHmac, randomBytes, randomInt, randomUUID } from "node:crypto";
import { gunzipSync } from "node:zlib";
import { connectAsync, type IClientOptions } from "mqtt";
import { defaultRpcTimeoutMs, localProtocolCodec, PRIORITY_STOPS, RobotClient, RpcError, TransportError } from "./robot.ts";
import { summarizeV1Map, type SavedDevice } from "./secret-store.ts";

export const MAX_MAP_BYTES = 32 * 1024 * 1024;

export interface DeviceCredentials {
  duid: string;
  name: string;
  model: string;
  localKey: string;
  protocolVersion: string;
}

export interface CameraAccount {
  user: string;
  secret: string;
  key: string;
  mqttUrl: string;
}

export interface AccountRoom { id: string; name: string }

/** One selected home/account; never return these secrets through an HTTP response. */
export interface HomeAuthorization {
  deviceId: string;
  homeId: number;
  realApi: string;
  user: string;
  secret: string;
  hawkKey: string;
  rooms: AccountRoom[];
}

export interface CameraLogin {
  devices: DeviceCredentials[];
  account: CameraAccount;
  home: Omit<HomeAuthorization, "deviceId">;
}

const REGIONS = {
  eu: { origin: "https://euiot.roborock.com", country: "DE", countryCode: "49" },
  us: { origin: "https://usiot.roborock.com", country: "US", countryCode: "1" },
  cn: { origin: "https://cniot.roborock.com", country: "CN", countryCode: "86" },
  asia: { origin: "https://api.roborock.com", country: "SG", countryCode: "65" },
};
const REQUEST_TIMEOUT_MS = 12_000;
const MAX_RESPONSE_BYTES = 2 * 1024 * 1024;

class CloudError extends Error {}

function record(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value);
}

function text(value: unknown): value is string {
  return typeof value === "string" && value.length > 0 && value.length <= 8192;
}

function headerText(value: unknown): value is string {
  return text(value) && /^[\x20-\x7e]+$/.test(value);
}

function hawkField(value: unknown): value is string {
  return text(value) && /^[a-zA-Z0-9_-]+$/.test(value);
}

function invalid(stage: string): never {
  throw new CloudError(`Roborock returned incomplete or invalid ${stage}. Please check the account region and retry.`);
}

function identifier(value: unknown): string {
  if (text(value)) return value;
  if (typeof value === "number" && Number.isSafeInteger(value) && value >= 0) return String(value);
  return invalid("product metadata");
}

function emailAddress(email: string): string {
  const normalized = email.trim();
  if (normalized.length > 254 || !/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(normalized)) {
    throw new CloudError("Enter a valid Roborock account email address.");
  }
  return normalized;
}

function trustedUrl(value: string): URL {
  let url: URL;
  try {
    url = new URL(value);
  } catch {
    throw new CloudError("Roborock returned an invalid cloud endpoint.");
  }
  // Use an explicit domain boundary, not a substring match or arbitrary login URL.
  if (
    url.protocol !== "https:" || !url.hostname.endsWith(".roborock.com") ||
    url.username || url.password || url.port || url.hash
  ) {
    throw new CloudError("Refusing an untrusted Roborock cloud endpoint.");
  }
  return url;
}

async function request(url: URL, init: RequestInit, stage: string): Promise<unknown> {
  trustedUrl(url.href);
  const signal = AbortSignal.timeout(REQUEST_TIMEOUT_MS);
  try {
    const response = await fetch(url, { ...init, redirect: "error", signal });
    if (!response.ok) {
      await response.body?.cancel();
      if (response.status === 429) {
        throw new CloudError("Roborock rate limit reached. Wait before trying again.");
      }
      if (response.status === 401 || response.status === 403) {
        throw new CloudError(`Roborock rejected ${stage}. Check the region, verification code, and computer clock.`);
      }
      throw new CloudError(`Roborock ${stage} failed (HTTP ${response.status}). Please retry later.`);
    }
    if (!response.body) return invalid(stage);
    const reader = response.body.getReader();
    const chunks: Uint8Array[] = [];
    let size = 0;
    try {
      while (true) {
        const { done, value } = await reader.read();
        if (done) break;
        size += value.byteLength;
        if (size > MAX_RESPONSE_BYTES) {
          await reader.cancel();
          throw new CloudError("Roborock response exceeded the safe size limit.");
        }
        chunks.push(value);
      }
    } finally {
      reader.releaseLock();
    }
    const body: unknown = JSON.parse(Buffer.concat(chunks).toString("utf8"));
    return body;
  } catch (error: unknown) {
    if (error instanceof CloudError) throw error;
    if (signal.aborted) {
      throw new CloudError(`Roborock ${stage} timed out. Check your connection and retry.`);
    }
    if (error instanceof SyntaxError) return invalid(stage);
    // Never retain an upstream cause: it can contain email, tokens, or response bodies.
    throw new CloudError(`Unable to complete Roborock ${stage}. Check your connection and account region; redirects are not allowed.`);
  }
}

function apiData(body: unknown, stage: string): unknown {
  if (!record(body) || typeof body.code !== "number" || !Number.isSafeInteger(body.code)) {
    return invalid(stage);
  }
  if (body.code !== 200) {
    throw new CloudError(`Roborock rejected ${stage}. Check the account region and verification code, or wait before retrying.`);
  }
  return body.data;
}

export function hawkHeaders(url: URL, user: string, secret: string, hawkKey: string): Record<string, string> {
  const timestamp = Math.floor(Date.now() / 1000);
  const nonce = randomBytes(6).toString("base64").slice(0, 6).replace(/\+/g, "X").replace(/\//g, "Y");
  const pathHash = createHash("md5").update(url.pathname + url.search).digest("hex");
  const mac = createHmac("sha256", hawkKey).update([user, secret, nonce, timestamp, pathHash, "", ""].join(":")).digest("base64");
  return {
    Authorization: ["Hawk", `id="${user}", s="${secret}", ts="${timestamp}", nonce="${nonce}", mac="${mac}"`].join(" "),
    "x-iotsdk-version": "1.0.1",
    "x-app-name": "com.roborock.smart",
    "x-app-version-code": "100834",
    "x-app-version-name": "4.57.02",
    "x-uid": user,
    "User-Agent": "UA=RRSDKAndroid/1.0.1",
  };
}

export function roomsFromHome(body: unknown): AccountRoom[] {
  if (!record(body) || body.success !== true || !record(body.result)) return invalid("home data");
  const rooms: unknown = body.result.rooms ?? [];
  if (!Array.isArray(rooms) || rooms.length > 128) return invalid("home room metadata");
  const seen = new Set<string>();
  return rooms.map((room): AccountRoom => {
    if (!record(room) || Object.keys(room).length > 16 ||
      !(typeof room.id === "number" && Number.isSafeInteger(room.id) && room.id >= 0 ||
        typeof room.id === "string" && /^[0-9]{1,24}$/.test(room.id)) ||
      typeof room.name !== "string" || room.name.length > 120 ||
      /[\u0000-\u001f\u007f]/.test(room.name)) return invalid("home room metadata");
    const id = String(room.id);
    if (seen.has(id)) return invalid("home room metadata");
    seen.add(id);
    return { id, name: room.name };
  });
}

export function validateHomeAuthorization(value: unknown): HomeAuthorization {
  if (!record(value)) throw new Error("Invalid private home authorization.");
  const home = value;
  if (Object.keys(home).sort().join(",") !== "deviceId,hawkKey,homeId,realApi,rooms,secret,user" ||
    typeof home.deviceId !== "string" || !/^[\x20-\x7e]{1,128}$/.test(home.deviceId) ||
    typeof home.homeId !== "number" || !Number.isSafeInteger(home.homeId) || home.homeId <= 0 ||
    typeof home.realApi !== "string" ||
    ![home.user, home.secret, home.hawkKey].every(field => typeof field === "string" &&
      field.length > 0 && field.length <= 4096 && /^[\x20-\x7e]+$/.test(field)) ||
    !Array.isArray(home.rooms) || home.rooms.length > 128) throw new Error("Invalid private home authorization.");
  let url: URL;
  try { url = new URL(home.realApi); } catch { throw new Error("Invalid private home authorization."); }
  if (url.origin !== home.realApi || url.protocol !== "https:" || !url.hostname.endsWith(".roborock.com") ||
    url.username || url.password || url.port || url.pathname !== "/" || url.search || url.hash ||
    !/^(?:[a-z0-9](?:[a-z0-9-]*[a-z0-9])?\.)+roborock\.com$/.test(url.hostname)) {
    throw new Error("Invalid private home authorization.");
  }
  const rooms = new Set<string>();
  for (const room of home.rooms) {
    if (!record(room) || Object.keys(room).sort().join(",") !== "id,name" ||
      typeof room.id !== "string" || !/^[0-9]{1,24}$/.test(room.id) ||
      typeof room.name !== "string" || room.name.length > 120 || /[\u0000-\u001f\u007f]/.test(room.name) ||
      rooms.has(room.id)) throw new Error("Invalid private home authorization.");
    rooms.add(room.id);
  }
  return {
    deviceId: home.deviceId, homeId: home.homeId, realApi: home.realApi,
    user: home.user as string, secret: home.secret as string, hawkKey: home.hawkKey as string,
    rooms: home.rooms as AccountRoom[],
  };
}

export function selectedRooms(home: HomeAuthorization): { source: string; rooms: AccountRoom[] } {
  return { source: "roborock-account-home", rooms: home.rooms.map(room => ({ id: room.id, name: room.name })) };
}

export async function refreshRooms(home: HomeAuthorization): Promise<AccountRoom[]> {
  home = validateHomeAuthorization(home);
  const url = new URL(`/v3/user/homes/${home.homeId}`, home.realApi);
  return roomsFromHome(await request(url, {
    method: "GET",
    headers: hawkHeaders(url, home.user, home.secret, home.hawkKey),
  }, "home room refresh"));
}

function devicesFromHome(body: unknown): DeviceCredentials[] {
  if (!record(body) || body.success !== true || !record(body.result)) {
    return invalid("home data");
  }
  const home = body.result;
  const products: unknown = home.products ?? [];
  const owned: unknown = home.devices ?? [];
  const shared: unknown = home.receivedDevices ?? [];
  if (!Array.isArray(products) || !Array.isArray(owned) || !Array.isArray(shared)) {
    return invalid("home data");
  }
  const metadata = new Map<string, { model: string; name?: string }>();
  for (const item of products) {
    if (!record(item) || !text(item.model) || (item.name !== undefined && !text(item.name))) {
      return invalid("product metadata");
    }
    metadata.set(identifier(item.id), { model: item.model, name: item.name });
  }
  return [...owned, ...shared].map((item: unknown): DeviceCredentials => {
    if (
      !record(item) || !text(item.duid) || !text(item.pv) ||
      !text(item.localKey) || !/^[\x20-\x7e]{16}$/.test(item.localKey) ||
      (item.name !== undefined && typeof item.name !== "string")
    ) {
      return invalid("device credentials");
    }
    const product = metadata.get(identifier(item.productId));
    if (!product) return invalid("device product metadata");
    return {
      duid: item.duid,
      name: item.name || product.name || product.model,
      model: product.model,
      localKey: item.localKey,
      protocolVersion: item.pv,
    };
  });
}

export class RoborockCloud {
  #region: (typeof REGIONS)[keyof typeof REGIONS];
  #clientId = randomBytes(16).toString("hex");

  constructor(region: "eu" | "us" | "cn" | "asia") {
    if (!Object.hasOwn(REGIONS, region)) throw new CloudError("Select a supported Roborock account region.");
    this.#region = REGIONS[region];
  }

  #headers(email: string): Record<string, string> {
    return {
      header_clientid: createHash("md5").update(email).update(this.#clientId).digest("base64"),
      header_appversion: "4.57.02",
      header_clientlang: "de",
      header_phonemodel: "Pixel 9 Pro XL",
      header_phonesystem: "Android",
    };
  }

  async requestCode(email: string): Promise<void> {
    email = emailAddress(email);
    const body = await request(new URL("/api/v4/email/code/send", this.#region.origin), {
      method: "POST",
      headers: this.#headers(email),
      body: new URLSearchParams({ type: "login", email, platform: "" }),
    }, "email code request");
    apiData(body, "email code request");
  }

  async loginWithCode(email: string, code: string): Promise<DeviceCredentials[]> {
    return (await this.#login(email, code, false)).devices;
  }

  async loginCameraWithCode(email: string, code: string): Promise<CameraLogin> {
    const result = await this.#login(email, code, true);
    if (!result.account) return invalid("camera account");
    return { devices: result.devices, account: result.account, home: result.home! };
  }

  async #login(email: string, code: string, camera: boolean): Promise<{
    devices: DeviceCredentials[]; account?: CameraAccount; home?: Omit<HomeAuthorization, "deviceId">;
  }> {
    email = emailAddress(email);
    if (!/^\d{6}$/.test(code)) throw new CloudError("Enter the six-digit verification code.");
    const headers = this.#headers(email);
    const salt = randomBytes(12).toString("base64").replace(/\+/g, "X").replace(/\//g, "Y");
    const signUrl = new URL("/api/v3/key/sign", this.#region.origin);
    signUrl.searchParams.set("s", salt);
    const sign = apiData(await request(signUrl, { method: "POST", headers }, "login signature"), "login signature");
    if (!record(sign) || !headerText(sign.k)) return invalid("login signature");
    const login = apiData(await request(new URL("/api/v4/auth/email/login/code", this.#region.origin), {
      method: "POST",
      headers: { ...headers, "x-mercy-k": sign.k, "x-mercy-ks": salt },
      body: new URLSearchParams({
        country: this.#region.country,
        countryCode: this.#region.countryCode,
        email,
        code,
        majorVersion: "14",
        minorVersion: "0",
      }),
    }, "code login"), "code login");
    if (!record(login) || !headerText(login.token) || !record(login.rriot)) return invalid("login data");
    const rriot = login.rriot;
    if (
      !hawkField(rriot.u) || !hawkField(rriot.s) || !headerText(rriot.h) ||
      !record(rriot.r) || !text(rriot.r.a)
    ) return invalid("login data");
    let account: CameraAccount | undefined;
    if (camera) {
      if (!headerText(rriot.k) || !text(rriot.r.m)) return invalid("camera account");
      let mqtt: URL;
      try { mqtt = new URL(rriot.r.m); }
      catch { return invalid("camera broker"); }
      if (mqtt.protocol !== "ssl:" || !mqtt.hostname.endsWith(".roborock.com") ||
        mqtt.username || mqtt.password || mqtt.search || mqtt.hash ||
        (mqtt.pathname !== "" && mqtt.pathname !== "/") ||
        !mqtt.port || Number(mqtt.port) < 1 || Number(mqtt.port) > 65535) {
        throw new CloudError("Refusing an untrusted Roborock camera broker.");
      }
      account = { user: rriot.u, secret: rriot.s, key: rriot.k, mqttUrl: mqtt.href };
    }
    const realApi = trustedUrl(rriot.r.a);
    if (realApi.pathname !== "/" || realApi.search) {
      throw new CloudError("Roborock returned an unexpected cloud endpoint path.");
    }
    const home = apiData(await request(new URL("/api/v1/getHomeDetail", this.#region.origin), {
      method: "GET",
      headers: { ...headers, Authorization: login.token },
    }, "home lookup"), "home lookup");
    if (!record(home) || typeof home.rrHomeId !== "number" || !Number.isSafeInteger(home.rrHomeId) || home.rrHomeId <= 0) {
      return invalid("home identifier");
    }
    const homeUrl = new URL(`/v3/user/homes/${home.rrHomeId}`, realApi);
    const homeResponse = await request(homeUrl, {
      method: "GET",
      headers: hawkHeaders(homeUrl, rriot.u, rriot.s, rriot.h),
    }, "signed home lookup");
    const devices = devicesFromHome(homeResponse);
    const rooms = camera ? roomsFromHome(homeResponse) : [];
    return { devices, ...(account ? { account, home: {
      homeId: home.rrHomeId, realApi: realApi.origin, user: rriot.u,
      secret: rriot.s, hawkKey: rriot.h, rooms,
    } } : {}) };

  }
}

export interface CloudControlCredentials {
  broker: string; user: string; secret: string; accountKey: string; deviceId: string; localKey: string;
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
    if (remaining<=0) throw new TransportError("Cloud RPC timed out before dispatch","not_sent");
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
