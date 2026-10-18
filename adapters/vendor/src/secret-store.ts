import { constants, type Stats } from "node:fs";
import { link, lstat, mkdir, open, rename, unlink } from "node:fs/promises";
import { createHash, randomBytes, randomUUID } from "node:crypto";
import { validatedMapMetadata } from "./map-cache-status.ts";
import { summarizeV1Map } from "./v1-map-summary.ts";
import { MAX_MAP_BYTES } from "./photo-protocol.ts";
import { isIP } from "node:net";
import { join, parse, resolve, sep } from "node:path";
import type { CameraAccount, HomeAuthorization } from "./cloud.ts";
import { validateHomeAuthorization } from "./home-account.ts";
import { localProtocol, type SavedDevice } from "./credentials.ts";

export interface PersistentCamera {
  account: CameraAccount;
  pin: string;
}

const MAX_JSON_BYTES = 16 * 1024;
const FILES = ["device.json", "camera.json", "home.json", "service-token", "map-cache.json", "pairing.json"] as const;
type StateFile = typeof FILES[number];

function hasCode(error: unknown, code: string): boolean {
  return error instanceof Error && "code" in error && error.code === code;
}

function shape(value: unknown, keys: readonly string[]): value is Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value) &&
    Object.keys(value).length === keys.length &&
    keys.every((key) => Object.hasOwn(value, key));
}

function text(value: unknown): value is string {
  return typeof value === "string" && value.trim().length > 0 &&
    value.length <= 4096 && !/[\u0000-\u001f\u007f]/.test(value);
}

function deviceValue(value: unknown): SavedDevice {
  if (!(shape(value, ["host", "duid", "name", "model", "localKey", "protocolVersion"]) ||
    shape(value, ["host", "duid", "name", "model", "localKey", "protocolVersion", "localProtocol"])) ||
    !text(value.host) || !isIP(value.host) || !text(value.duid) ||
    !text(value.name) || !text(value.model) || !text(value.localKey) ||
    !text(value.protocolVersion) || !/^[\x20-\x7e]{16}$/.test(value.localKey)) {
    throw new Error("Invalid saved device state.");
  }
  return {
    host: value.host, duid: value.duid, name: value.name, model: value.model,
    localKey: value.localKey, protocolVersion: value.protocolVersion,
    ...(Object.hasOwn(value, "localProtocol") ? { localProtocol: localProtocol(value.localProtocol) } : {}),
  };
}

function cameraValue(value: unknown): PersistentCamera {
  if (!shape(value, ["account", "pin"]) || !shape(value.account, ["user", "secret", "key", "mqttUrl"]) ||
    !text(value.account.user) || !text(value.account.secret) || !text(value.account.key) ||
    !text(value.account.mqttUrl) || typeof value.pin !== "string" || (value.pin !== "" && !/^[1-9]{4,12}$/.test(value.pin))) {
    throw new Error("Invalid saved camera state.");
  }
  const account = {
    user: value.account.user, secret: value.account.secret,
    key: value.account.key, mqttUrl: value.account.mqttUrl,
  };
  const broker = new URL(account.mqttUrl);
  if (broker.protocol !== "ssl:" || !broker.hostname.endsWith(".roborock.com") ||
    !/^(?:[a-z0-9](?:[a-z0-9-]*[a-z0-9])?\.)+roborock\.com$/.test(broker.hostname) ||
    broker.username || broker.password || broker.search || broker.hash ||
    (broker.pathname !== "" && broker.pathname !== "/") ||
    !broker.port || Number(broker.port) < 1 || Number(broker.port) > 65535) {
    throw new Error("Invalid saved camera broker.");
  }
  return {
    account: { user: account.user, secret: account.secret, key: account.key, mqttUrl: account.mqttUrl },
    pin: value.pin,
  };
}

function json(value: string): unknown {
  try { return JSON.parse(value); }
  catch { throw new Error("Invalid private state JSON."); }
}

/** Internal fixed messages and errno codes carry no secrets or paths; anything else is reduced to its class name. */
const SAFE_REASON = /^(Invalid|Unsafe|Private|Truncated)[ A-Za-z.]{0,80}$/;
export function safeReason(error: unknown): string {
  const code = typeof error === "object" && error !== null && "code" in error ? (error as { code: unknown }).code : undefined;
  if (typeof code === "string" && /^E[A-Z0-9]{2,20}$/.test(code)) return code;
  if (error instanceof Error && SAFE_REASON.test(error.message)) return error.message;
  return error instanceof Error ? error.name : "unknown";
}

async function sanitized<T>(operation: () => Promise<T>, message: string): Promise<T> {
  try { return await operation(); }
  // Filesystem and URL/JSON parser errors can contain credentials or private paths: keep only a safe reason.
  catch (error) { throw new Error(message, { cause: safeReason(error) }); }
}

/**
 * Plaintext private volume storage, NOT encryption. Only the selected state directory
 * is chmod'ed; ancestors are inspected, never chmod'ed. Symlink paths are refused.
 * Set ROBOCLAW_STATE_DIR=/data for a persistent container volume.
 */
export class SecretStore {
  #directory: string;

  constructor(directory = process.env.ROBOCLAW_STATE_DIR ?? ".local") {
    if (typeof directory !== "string" || !directory.trim() || directory.includes("\0")) {
      throw new Error("Invalid private state directory.");
    }
    this.#directory = resolve(directory);
    if (this.#directory === parse(this.#directory).root) {
      throw new Error("Invalid private state directory.");
    }
  }

  async initialize(): Promise<void> {
    await sanitized(async () => {
      await this.#prepare();
      for (const file of FILES) await this.#checkFile(file);
      await this.serviceToken();
    }, "Cannot initialize private state.");
  }

  async loadPairing(): Promise<{ device: SavedDevice; camera: PersistentCamera; home: HomeAuthorization } | null> {
    return sanitized(async () => {
      await this.#prepare();
      const contents = await this.#read("pairing.json", MAX_JSON_BYTES);
      if (contents === null) return null;
      const value = json(contents);
      if (!shape(value, ["device", "camera", "home"])) throw new Error("Invalid account pairing");
      const pairing = { device: deviceValue(value.device), camera: cameraValue(value.camera), home: validateHomeAuthorization(value.home) };
      if (pairing.home.deviceId !== pairing.device.duid) throw new Error("Invalid account pairing");
      return pairing;
    }, "Cannot load authenticated account pairing.");
  }
  async savePairing(pairing: { device: SavedDevice; camera: PersistentCamera; home: HomeAuthorization }): Promise<void> {
    await sanitized(async () => {
      const value = { device: deviceValue(pairing.device), camera: cameraValue(pairing.camera), home: validateHomeAuthorization(pairing.home) };
      if (value.home.deviceId !== value.device.duid) throw new Error("Invalid account pairing");
      await this.#write("pairing.json", JSON.stringify(value), false);
    }, "Cannot save authenticated account pairing.");
  }

  async loadDevice(): Promise<SavedDevice | null> {
    return sanitized(async () => {
      await this.#prepare();
      const contents = await this.#read("device.json", MAX_JSON_BYTES);
      return contents === null ? null : deviceValue(json(contents));
    }, "Cannot load private device state.");
  }

  async saveDevice(device: SavedDevice): Promise<void> {
    await sanitized(async () => {
      const contents = JSON.stringify(deviceValue(device));
      await this.#write("device.json", contents, false);
    }, "Cannot save private device state.");
  }

  async loadCamera(): Promise<PersistentCamera | null> {
    return sanitized(async () => {
      await this.#prepare();
      const contents = await this.#read("camera.json", MAX_JSON_BYTES);
      return contents === null ? null : cameraValue(json(contents));
    }, "Cannot load private camera state.");
  }

  async saveCamera(camera: PersistentCamera): Promise<void> {
    await sanitized(async () => {
      const contents = JSON.stringify(cameraValue(camera));
      await this.#write("camera.json", contents, false);
    }, "Cannot save private camera state.");
  }

  async loadHome(): Promise<HomeAuthorization | null> {
    return sanitized(async () => {
      await this.#prepare();
      const contents = await this.#read("home.json", MAX_JSON_BYTES);
      return contents === null ? null : validateHomeAuthorization(json(contents));
    }, "Cannot load private home authorization.");
  }

  async saveHome(home: HomeAuthorization): Promise<void> {
    await sanitized(async () => {
      let validated: HomeAuthorization;
      try { validated = validateHomeAuthorization(home); }
      catch {
        // Boolean schema checks only; no private values or exception text.
        const url = (() => { try { return new URL(home.realApi); } catch { return null; } })();
        console.error("Home authorization validation failed:", JSON.stringify({
          shape: Object.keys(home).sort().join(",") === "deviceId,hawkKey,homeId,realApi,rooms,secret,user",
          deviceId: typeof home.deviceId === "string" && /^[\x20-\x7e]{1,128}$/.test(home.deviceId),
          homeId: Number.isSafeInteger(home.homeId) && home.homeId > 0,
          origin: !!url && url.origin === home.realApi,
          trustedHost: !!url && /^(?:[a-z0-9](?:[a-z0-9-]*[a-z0-9])?\.)+roborock\.com$/.test(url.hostname),
          credentialShape: [home.user, home.secret, home.hawkKey].every(v => typeof v === "string" && v.length > 0 && v.length <= 4096 && /^[\x20-\x7e]+$/.test(v)),
          roomsBounded: Array.isArray(home.rooms) && home.rooms.length <= 128,
          roomsUnique: Array.isArray(home.rooms) && new Set(home.rooms.map(r => r.id)).size === home.rooms.length,
          roomShape: Array.isArray(home.rooms) && home.rooms.every(r => Object.keys(r).sort().join(",") === "id,name" && typeof r.id === "string" && /^[0-9]{1,24}$/.test(r.id) && typeof r.name === "string" && r.name.length <= 120 && !/[\u0000-\u001f\u007f]/.test(r.name)),
        }));
        throw new Error("Invalid private home authorization.");
      }
      await this.#write("home.json", JSON.stringify(validated), false);
    }, "Cannot save private home authorization.");
  }

  async clearHome(): Promise<void> {
    await sanitized(async () => {
      await this.#prepare();
      // Do not unlink an attacker-swapped symlink or silently delete corrupt state.
      const contents = await this.#read("home.json", MAX_JSON_BYTES);
      if (contents !== null) validateHomeAuthorization(json(contents));
      try { await unlink(join(this.#directory, "home.json")); }
      catch (error) { if (!hasCode(error, "ENOENT")) throw error; }
    }, "Cannot clear private home authorization.");
  }

  async loadMapCache(): Promise<unknown | null> {
    return sanitized(async () => {
      const entry = await this.#readVerifiedMapCache();
      return entry?.metadata ?? null;
    }, "Cannot load private map metadata.");
  }

  /** Re-open the same verified metadata and private file for rasterization; no
   * caller-controlled path, and no byte-only legacy cache fallback.
   */
  async loadMapRasterSource(): Promise<{ metadata: import("./map-cache-status.ts").PrivateMapMetadata; raw: Buffer } | null> {
    return sanitized(() => this.#readVerifiedMapCache(), "Cannot load private map raster source.");
  }

  async #readVerifiedMapCache() {
    await this.#prepare();
    const contents = await this.#read("map-cache.json", MAX_JSON_BYTES);
    if (contents === null) return null;
    const metadata = validatedMapMetadata(json(contents));
    const path = join(this.#directory, "maps", `v1-${metadata.sha256}.bin`);
    const dir = await lstat(join(this.#directory, "maps"));
    if (!dir.isDirectory() || dir.isSymbolicLink() || (dir.mode & 0o777) !== 0o700) throw new Error("Unsafe private map directory.");
    const file = await open(path, constants.O_RDONLY | constants.O_NOFOLLOW | constants.O_NONBLOCK);
    try {
      const stat = await file.stat();
      if (!stat.isFile() || stat.nlink !== 1 || (stat.mode & 0o7777) !== 0o600 || stat.size !== metadata.bytes ||
          stat.size > MAX_MAP_BYTES) throw new Error("Unsafe private map file.");
      const bytes = Buffer.alloc(stat.size);
      let total = 0;
      while (total < bytes.length) {
        const { bytesRead } = await file.read(bytes, total, bytes.length - total, null);
        if (!bytesRead) throw new Error("Truncated private map file.");
        total += bytesRead;
      }
      // Verify both the published content address and the underlying V1 bytes.
      if (createHash("sha256").update(bytes).digest("hex") !== metadata.sha256) throw new Error("Invalid cached map digest.");
      summarizeV1Map(bytes);
      if (bytes.readUInt32LE(12) !== metadata.mapIndex || bytes.readUInt32LE(16) !== metadata.mapSequence)
        throw new Error("Private map header differs from published metadata.");
      return { metadata, raw: bytes };
    } finally { await file.close(); }
  }

  async saveMapCache(metadata: unknown): Promise<void> {
    await sanitized(async () => {
      await this.#write("map-cache.json", JSON.stringify(validatedMapMetadata(metadata)), false);
    }, "Cannot save private map metadata.");
  }

  async saveMapBytes(raw: Buffer): Promise<void> {
    const { savePrivateV1Map } = await import("./private-map-cache.ts");
    await savePrivateV1Map(raw, this.#directory);
  }

  async serviceToken(): Promise<string> {
    return sanitized(async () => {
      await this.#prepare();
      let token = await this.#read("service-token", 43);
      if (token === null) {
        await this.#write("service-token", randomBytes(32).toString("base64url"), true);
        token = await this.#read("service-token", 43);
      }
      if (token === null || !/^[A-Za-z0-9_-]{43}$/.test(token) ||
        Buffer.from(token, "base64url").toString("base64url") !== token) {
        throw new Error("Invalid private service token.");
      }
      return token;
    }, "Cannot load private service token.");
  }

  async #prepare(): Promise<void> {
    const root = parse(this.#directory).root;
    let current = root;
    for (const component of this.#directory.slice(root.length).split(sep)) {
      current = join(current, component);
      try { await mkdir(current, { mode: 0o700 }); }
      catch (error) { if (!hasCode(error, "EEXIST")) throw error; }
      const info = await lstat(current);
      if (!info.isDirectory() || info.isSymbolicLink()) throw new Error("Unsafe state directory.");
    }
    const handle = await open(this.#directory, constants.O_RDONLY | constants.O_DIRECTORY | constants.O_NOFOLLOW);
    try {
      await handle.chmod(0o700);
    } finally { await handle.close(); }
  }

  async #checkFile(file: StateFile): Promise<boolean> {
    try {
      const info = await lstat(join(this.#directory, file));
      this.#assertFile(info);
      return true;
    } catch (error) {
      if (hasCode(error, "ENOENT")) return false;
      throw error;
    }
  }

  #assertFile(info: Stats): void {
    if (!info.isFile() || info.isSymbolicLink() || (info.mode & 0o7777) !== 0o600) {
      throw new Error("Unsafe private state file.");
    }
  }

  async #read(file: StateFile, limit: number): Promise<string | null> {
    let handle;
    try {
      handle = await open(join(this.#directory, file),
        constants.O_RDONLY | constants.O_NOFOLLOW | constants.O_NONBLOCK);
    } catch (error) {
      if (hasCode(error, "ENOENT")) {
        // A dangling symlink is unsafe, not an absent file.
        if (await this.#checkFile(file)) throw new Error("Unsafe private state file.");
        return null;
      }
      throw error;
    }
    try {
      const info = await handle.stat();
      this.#assertFile(info);
      if (info.size > limit) throw new Error("Private state is too large.");
      const buffer = Buffer.alloc(limit + 1);
      let total = 0;
      while (total < buffer.length) {
        const { bytesRead } = await handle.read(buffer, total, buffer.length - total, null);
        if (!bytesRead) break;
        total += bytesRead;
      }
      if (total > limit) throw new Error("Private state is too large.");
      return new TextDecoder("utf-8", { fatal: true }).decode(buffer.subarray(0, total));
    } finally { await handle.close(); }
  }

  async #write(file: StateFile, contents: string, exclusive: boolean): Promise<void> {
    if (Buffer.byteLength(contents) > MAX_JSON_BYTES) throw new Error("Private state is too large.");
    await this.#prepare();
    await this.#checkFile(file);
    const temporary = join(this.#directory, `.${file}-${randomUUID()}.tmp`);
    const handle = await open(temporary,
      constants.O_WRONLY | constants.O_CREAT | constants.O_EXCL | constants.O_NOFOLLOW, 0o600);
    try {
      try {
        await handle.chmod(0o600);
        await handle.writeFile(contents, "utf8");
        await handle.sync();
      } finally { await handle.close(); }
      await this.#prepare();
      await this.#checkFile(file);
      if (exclusive) {
        // Publish complete bytes without replacing a token another process won.
        try { await link(temporary, join(this.#directory, file)); }
        catch (error) { if (!hasCode(error, "EEXIST")) throw error; }
      } else {
        await rename(temporary, join(this.#directory, file));
      }
    } finally {
      try { await unlink(temporary); }
      catch (error) { if (!hasCode(error, "ENOENT")) throw error; }
    }
    const directory = await open(this.#directory, constants.O_RDONLY | constants.O_DIRECTORY | constants.O_NOFOLLOW);
    try { await directory.sync(); }
    finally { await directory.close(); }
  }
}
