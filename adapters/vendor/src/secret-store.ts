import { constants, type Stats } from "node:fs";
import { link, lstat, mkdir, open, rename, unlink } from "node:fs/promises";
import { createHash, randomBytes, randomUUID } from "node:crypto";
import { isIP } from "node:net";
import { join, parse, resolve, sep } from "node:path";
import { validateHomeAuthorization, type CameraAccount, type DeviceCredentials, type HomeAuthorization } from "./cloud.ts";

export const MAX_MAP_BYTES = 32 * 1024 * 1024;
export type LocalProtocol = "1.0" | "L01";
export function localProtocol(value: unknown): LocalProtocol {
  if (value !== "1.0" && value !== "L01") throw new Error("Unsupported local protocol");
  return value;
}
export function selectedLocalProtocol(device: SavedDevice): LocalProtocol {
  return localProtocol(device.localProtocol ?? "1.0");
}

export interface SavedDevice extends DeviceCredentials {
  host: string;
  localProtocol?: LocalProtocol;
}

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

export function hasCode(error: unknown, code: string): boolean {
  return error instanceof Error && "code" in error && error.code === code;
}

export function resolveStateDir(directory: string, message = "Invalid private state directory."): string {
  if (typeof directory !== "string" || !directory.trim() || directory.includes("\0")) throw new Error(message);
  const resolved = resolve(directory);
  if (resolved === parse(resolved).root) throw new Error(message);
  return resolved;
}

export async function writePrivateTempFile(path: string, contents: string | Buffer): Promise<void> {
  const handle = await open(path, constants.O_WRONLY | constants.O_CREAT | constants.O_EXCL | constants.O_NOFOLLOW, 0o600);
  try {
    await handle.chmod(0o600);
    await handle.writeFile(contents);
    await handle.sync();
  } finally { await handle.close(); }
}

export async function ensurePrivateDir(directory: string, message = "Unsafe state directory."): Promise<void> {
  const root = parse(directory).root;
  let current = root;
  for (const component of directory.slice(root.length).split(sep)) {
    current = join(current, component);
    try { await mkdir(current, { mode: 0o700 }); }
    catch (error) { if (!hasCode(error, "EEXIST")) throw error; }
    const info = await lstat(current);
    if (!info.isDirectory() || info.isSymbolicLink()) throw new Error(message);
  }
  const handle = await open(directory, constants.O_RDONLY | constants.O_DIRECTORY | constants.O_NOFOLLOW);
  try { await handle.chmod(0o700); }
  finally { await handle.close(); }
}

export async function syncPrivateDir(directory: string): Promise<void> {
  const handle = await open(directory, constants.O_RDONLY | constants.O_DIRECTORY | constants.O_NOFOLLOW);
  try { await handle.sync(); }
  finally { await handle.close(); }
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
    this.#directory = resolveStateDir(directory);
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

  async loadMapRasterSource(): Promise<{ metadata: PrivateMapMetadata; raw: Buffer } | null> {
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
    const bytes = await readPrivateMapFile(path, metadata.bytes);
    if (createHash("sha256").update(bytes).digest("hex") !== metadata.sha256) throw new Error("Invalid cached map digest.");
    summarizeV1Map(bytes);
    if (bytes.readUInt32LE(12) !== metadata.mapIndex || bytes.readUInt32LE(16) !== metadata.mapSequence)
      throw new Error("Private map header differs from published metadata.");
    return { metadata, raw: bytes };
  }

  async saveMapCache(metadata: unknown): Promise<void> {
    await sanitized(async () => {
      await this.#write("map-cache.json", JSON.stringify(validatedMapMetadata(metadata)), false);
    }, "Cannot save private map metadata.");
  }

  async saveMapBytes(raw: Buffer): Promise<void> {
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
    await ensurePrivateDir(this.#directory);
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
    try {
      await writePrivateTempFile(temporary, contents);
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
    await syncPrivateDir(this.#directory);
  }
}

/** Bounded, read-only V1 map geometry summary. */
export function summarizeV1Map(raw: Buffer) {
  if (raw.length < 40 || raw.length > MAX_MAP_BYTES || raw.toString("ascii", 0, 2) !== "rr" ||
      raw.readUInt16LE(2) !== 20 || raw.readUInt32LE(4) !== raw.length - 20 ||
      !createHash("sha1").update(raw.subarray(0, -20)).digest().equals(raw.subarray(-20))) {
    throw new Error("Invalid V1 map integrity");
  }
  const end = raw.length;
  const blocks: Record<string, number> = {};
  const rooms = new Set<number>();
  let image: { left: number; top: number; width: number; height: number; metersPerPixel: number; walls: number; floor: number; pixels: number; roomSegments: number[] } | undefined;
  let vacuumPosition: { x: number; y: number; angle?: number } | undefined;
  let chargerPosition: { x: number; y: number; angle?: number } | undefined;
  const position = (start: number, length: number) => {
    if (length !== 8 && length !== 12) throw new Error("Invalid V1 position block");
    return { x: raw.readInt32LE(start), y: raw.readInt32LE(start + 4),
      ...(length === 12 ? { angle: raw.readInt32LE(start + 8) } : {}) };
  };
  const roomSegment = (pixel: number) => pixel !== 255 && pixel !== 7 && (pixel & 7) === 7 ? pixel >> 3 : null;
  let blockCount = 0;
  let digestSeen = false;
  let imagePixelData: Buffer | undefined;
  for (let offset = 20; offset < end;) {
    if (++blockCount > 1024 || offset + 8 > end) throw new Error("Invalid V1 map block count");
    const type = raw.readUInt16LE(offset), headerSize = raw.readUInt16LE(offset + 2), length = raw.readUInt32LE(offset + 4);
    if (headerSize < 8 || headerSize > 256 || offset + headerSize + length > end) throw new Error("Invalid V1 map block extent");
    const data = offset + headerSize;
    blocks[String(type)] = (blocks[String(type)] ?? 0) + 1;
    if (type === 1024) {
      if (length !== 20 || data + length !== end) throw new Error("Invalid V1 digest block");
      digestSeen = true;
      break;
    }
    if (type === 1) chargerPosition = position(data, length);
    if (type === 8) vacuumPosition = position(data, length);
    if (type === 2) {
      if (image) throw new Error("Duplicate V1 image block");
      if (headerSize < 24) throw new Error("Invalid V1 image header");
      const left = raw.readInt32LE(offset + headerSize - 12), top = raw.readInt32LE(offset + headerSize - 16);
      const height = raw.readUInt32LE(offset + headerSize - 8), width = raw.readUInt32LE(offset + headerSize - 4);
      if (!width || !height || width > 1024 || height > 1024 || width * height !== length) throw new Error("Invalid V1 image size");
      let walls = 0, floor = 0;
      for (let i = data; i < data + length; i++) {
        const pixel = raw[i];
        if ((pixel & 7) === 1) walls++;
        if ((pixel & 7) === 7) floor++;
        const seg = roomSegment(pixel);
        if (seg !== null) rooms.add(seg);
      }
      image = { left, top, width, height, metersPerPixel: 0.05, walls, floor, pixels: length,
        roomSegments: [...rooms].sort((a, b) => a - b) };
      imagePixelData = raw.subarray(data, data + length);
    }
    offset += headerSize + length;
  }
  if (!digestSeen || !image || !imagePixelData) throw new Error("Incomplete V1 map block sequence");
  let embeddedVacuumRoomSegment: number | null = null;
  if (vacuumPosition) {
    const x = Math.floor(vacuumPosition.x / 50 - image.left);
    const y = Math.floor(vacuumPosition.y / 50 - image.top);
    if (x >= 0 && y >= 0 && x < image.width && y < image.height) {
      embeddedVacuumRoomSegment = roomSegment(imagePixelData[x + image.width * y]);
    }
  }
  return { blocks, image, vacuumPosition, chargerPosition, embeddedVacuumRoomSegment };
}

export interface PrivateMapMetadata {
  version: 1;
  deviceId: string;
  provenance: "get_map_v1 / local L01 request + correlated MQTT reply";
  savedAt: number;
  mapIndex: number;
  mapSequence: number;
  bytes: number;
  sha256: string;
}

export function mapMetadataFromRead(deviceId: string, read: {
  bytes: number; sha256: string; mapIndex: number; mapSequence: number;
}, savedAt = Date.now()): PrivateMapMetadata {
  return validatedMapMetadata({
    version: 1, deviceId, sha256: read.sha256,
    provenance: "get_map_v1 / local L01 request + correlated MQTT reply",
    savedAt, mapIndex: read.mapIndex, mapSequence: read.mapSequence, bytes: read.bytes,
  });
}

export function validatedMapMetadata(value: unknown): PrivateMapMetadata {
  if (!value || typeof value !== "object" || Array.isArray(value)) throw new Error("Invalid private map metadata.");
  const item = value as Record<string, unknown>;
  if (Object.keys(item).sort().join(",") !== "bytes,deviceId,mapIndex,mapSequence,provenance,savedAt,sha256,version" ||
    item.version !== 1 || typeof item.sha256 !== "string" || !/^[a-f0-9]{64}$/.test(item.sha256) || typeof item.deviceId !== "string" || !item.deviceId || item.deviceId.length > 128 ||
    item.provenance !== "get_map_v1 / local L01 request + correlated MQTT reply" ||
    ![item.savedAt, item.mapIndex, item.mapSequence, item.bytes].every(Number.isSafeInteger) ||
    (item.savedAt as number) <= 0 || (item.mapIndex as number) < 0 || (item.mapSequence as number) < 0 ||
    (item.bytes as number) <= 0 || (item.bytes as number) > MAX_MAP_BYTES) throw new Error("Invalid private map metadata.");
  return item as unknown as PrivateMapMetadata;
}

export async function readPrivateMapFile(path: string, expectedBytes: number): Promise<Buffer> {
  const file = await open(path, constants.O_RDONLY | constants.O_NOFOLLOW | constants.O_NONBLOCK);
  try {
    const stat = await file.stat();
    if (!stat.isFile() || stat.nlink !== 1 || (stat.mode & 0o7777) !== 0o600 || stat.size !== expectedBytes ||
        stat.size > MAX_MAP_BYTES) throw new Error("Unsafe private map file.");
    const bytes = Buffer.alloc(stat.size);
    let total = 0;
    while (total < bytes.length) {
      const { bytesRead } = await file.read(bytes, total, bytes.length - total, null);
      if (!bytesRead) throw new Error("Truncated private map file.");
      total += bytesRead;
    }
    return bytes;
  } finally { await file.close(); }
}

/** Save a verified V1 map binary into the private state directory. */
export async function savePrivateV1Map(raw: Buffer, stateDirectory: string): Promise<{ sha256: string; bytes: number; saved: boolean }> {
  if (!Buffer.isBuffer(raw) || raw.length > MAX_MAP_BYTES) throw new Error("Invalid private V1 map.");
  try { summarizeV1Map(raw); } catch { throw new Error("Invalid private V1 map."); }
  const sha256 = createHash("sha256").update(raw).digest("hex");
  const root = resolveStateDir(stateDirectory, "Cannot save private V1 map.");
  const directory = join(root, "maps");
  const destination = join(directory, `v1-${sha256}.bin`);
  const temporary = join(directory, `.v1-${randomUUID()}.tmp`);
  try {
    await ensurePrivateDir(root, "Unsafe private map directory.");
    await ensurePrivateDir(directory, "Unsafe private map directory.");
    let saved = false;
    await writePrivateTempFile(temporary, raw);
    try { await link(temporary, destination); saved = true; }
    catch (error) {
      if (!hasCode(error, "EEXIST")) throw error;
      const bytes = await readPrivateMapFile(destination, raw.length);
      if (!bytes.equals(raw)) throw new Error("Private map hash collision or changed file.");
    }
    await unlink(temporary);
    await syncPrivateDir(directory);
    return { sha256, bytes: raw.length, saved };
  } catch { throw new Error("Cannot save private V1 map."); }
  finally { try { await unlink(temporary); } catch { /* absent or cleanup failure cannot expose map bytes */ } }
}

export function syntheticV1Image(pixels = Buffer.from([1, 31, 255, 0]), width = 2, height = 2): Buffer {
  const header = Buffer.alloc(20); header.write("rr"); header.writeUInt16LE(20, 2);
  const image = Buffer.alloc(24 + pixels.length); image.writeUInt16LE(2); image.writeUInt16LE(24, 2);
  image.writeUInt32LE(pixels.length, 4); image.writeUInt32LE(height, 16); image.writeUInt32LE(width, 20);
  pixels.copy(image, 24);
  const position = Buffer.alloc(20); position.writeUInt16LE(8); position.writeUInt16LE(8, 2);
  position.writeUInt32LE(12, 4); position.writeInt32LE(123456, 8); position.writeInt32LE(-7777, 12);
  const digest = Buffer.alloc(28); digest.writeUInt16LE(1024); digest.writeUInt16LE(8, 2); digest.writeUInt32LE(20, 4);
  const raw = Buffer.concat([header, image, position, digest]); raw.writeUInt32LE(raw.length - 20, 4);
  createHash("sha1").update(raw.subarray(0, -20)).digest().copy(raw, raw.length - 20);
  return raw;
}
