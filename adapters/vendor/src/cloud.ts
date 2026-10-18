// Cloud flow and Hawk signing adapted from copystring/ioBroker.roborock (MIT).
// Copyright (c) 2025-2026 copystring. See THIRD_PARTY_NOTICES.
import { createHash, createHmac, randomBytes } from "node:crypto";

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
    const timestamp = Math.floor(Date.now() / 1000);
    const nonce = randomBytes(6).toString("base64").slice(0, 6).replace(/\+/g, "X").replace(/\//g, "Y");
    const pathHash = createHash("md5").update(homeUrl.pathname + homeUrl.search).digest("hex");
    const prestr = [rriot.u, rriot.s, nonce, timestamp, pathHash, "", ""].join(":");
    const mac = createHmac("sha256", rriot.h).update(prestr).digest("base64");
    const homeResponse = await request(homeUrl, {
      method: "GET",
      headers: {
        Authorization: `Hawk id="${rriot.u}", s="${rriot.s}", ts="${timestamp}", nonce="${nonce}", mac="${mac}"`,
        "x-iotsdk-version": "1.0.1",
        "x-app-name": "com.roborock.smart",
        "x-app-version-code": "100834",
        "x-app-version-name": "4.57.02",
        "x-uid": rriot.u,
        "User-Agent": "UA=RRSDKAndroid/1.0.1",
      },
    }, "signed home lookup");
    const devices = devicesFromHome(homeResponse);
    const rooms = camera ? roomsFromHome(homeResponse) : [];
    return { devices, ...(account ? { account, home: {
      homeId: home.rrHomeId, realApi: realApi.origin, user: rriot.u,
      secret: rriot.s, hawkKey: rriot.h, rooms,
    } } : {}) };

  }
}
