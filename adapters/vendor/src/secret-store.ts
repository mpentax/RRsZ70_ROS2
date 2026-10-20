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

export interface PersistentPairing {
  device: SavedDevice;
  camera: PersistentCamera;
  home: HomeAuthorization;
}

const MAX_JSON_BYTES = 16 * 1024;
const FILES = ["pairing.json", "service-token", "map-cache.json"] as const;
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
  return { account, pin: value.pin };
}

function pairingValue(value: unknown): PersistentPairing {
  if (!shape(value, ["device", "camera", "home"])) throw new Error("Invalid account pairing.");
  const pairing = {
    device: deviceValue(value.device),
    camera: cameraValue(value.camera),
    home: validateHomeAuthorization(value.home),
  };
  if (pairing.home.deviceId !== pairing.device.duid) throw new Error("Invalid account pairing.");
  return pairing;
}

function json(value: string): unknown {
  try { return JSON.parse(value); }
  catch { throw new Error("Invalid private state JSON."); }
}

/** Internal fixed messages and errno codes carry no secrets or paths; anything else is reduced to its class name. */
export function safeReason(error: unknown): string {
  const code = typeof error === "object" && error !== null && "code" in error ? (error as { code: unknown }).code : undefined;
  if (typeof code === "string" && /^E[A-Z0-9]{2,20}$/.test(code)) return code;
  if (error instanceof Error && /^(Invalid|Unsafe|Private|Truncated)[ A-Za-z.]{0,80}$/.test(error.message)) return error.message;
  return error instanceof Error ? error.name : "unknown";
}

async function sanitized<T>(operation: () => Promise<T>, message: string): Promise<T> {
  try { return await operation(); }
  catch (error) { throw new Error(message, { cause: safeReason(error) }); }
}

/** Private state directory storage for pairing, service token, and map cache. */
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

  async loadPairing(): Promise<PersistentPairing | null> {
    return sanitized(async () => {
      await this.#prepare();
      const contents = await this.#read("pairing.json", MAX_JSON_BYTES);
      return contents === null ? null : pairingValue(json(contents));
    }, "Cannot load authenticated account pairing.");
  }

  async savePairing(pairing: PersistentPairing): Promise<void> {
    await sanitized(async () => {
      await this.#write("pairing.json", JSON.stringify(pairingValue(pairing)), false);
    }, "Cannot save authenticated account pairing.");
  }

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
    try { await handle.chmod(0o700); }
    finally { await handle.close(); }
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
