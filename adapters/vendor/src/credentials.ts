import { mkdir, lstat, chmod, open, rename, unlink, readFile } from "node:fs/promises";
import { join } from "node:path";
import { randomUUID } from "node:crypto";
import type { DeviceCredentials } from "./cloud.ts";

export type LocalProtocol = "1.0" | "L01";
export function localProtocol(value: unknown): LocalProtocol {
  if (value !== "1.0" && value !== "L01") throw new Error("Unsupported local protocol");
  return value;
}
export function selectedLocalProtocol(device: SavedDevice): LocalProtocol {
  // Legacy records retain the historical TCP default, never infer from cloud pv.
  return localProtocol(device.localProtocol ?? "1.0");
}

export interface SavedDevice extends DeviceCredentials {
  host: string;
  /** Explicit local TCP setting; protocolVersion is cloud inventory metadata only. */
  localProtocol?: LocalProtocol;
}

export async function saveDevice(directory: string, device: SavedDevice): Promise<void> {
  selectedLocalProtocol(device);
  await mkdir(directory, { recursive: true, mode: 0o700 });
  const info = await lstat(directory);
  if (!info.isDirectory() || info.isSymbolicLink()) {
    throw new Error("Credential storage must be a real directory.");
  }
  await chmod(directory, 0o700);
  const temporary = join(directory, `.device-${randomUUID()}.tmp`);
  const handle = await open(temporary, "wx", 0o600);
  try {
    await handle.writeFile(JSON.stringify(device));
    await handle.sync();
  } catch (error) {
    await handle.close();
    await unlink(temporary);
    throw error;
  }
  await handle.close();
  try {
    await rename(temporary, join(directory, "device.json"));
  } catch (error) {
    await unlink(temporary);
    throw error;
  }
}

export async function loadDevice(directory: string): Promise<SavedDevice> {
  const file = join(directory, "device.json");
  const info = await lstat(file);
  if (!info.isFile() || info.isSymbolicLink() || (info.mode & 0o077) !== 0) {
    throw new Error("Credential file must be a private regular file (mode 0600).");
  }
  const contents = await readFile(file, "utf8");
  let value: unknown;
  try {
    value = JSON.parse(contents);
  } catch (error) {
    if (error instanceof SyntaxError) {
      throw new Error("Saved device credentials are invalid JSON; run npm run auth again.");
    }
    throw error;
  }
  if (
    !value || typeof value !== "object" ||
    !("host" in value) || typeof value.host !== "string" ||
    !("duid" in value) || typeof value.duid !== "string" ||
    !("name" in value) || typeof value.name !== "string" ||
    !("model" in value) || typeof value.model !== "string" ||
    !("localKey" in value) || typeof value.localKey !== "string" ||
    !("protocolVersion" in value) || typeof value.protocolVersion !== "string"
  ) {
    throw new Error("Invalid saved device credentials; run npm run auth again.");
  }
  return {
    host: value.host,
    duid: value.duid,
    name: value.name,
    model: value.model,
    localKey: value.localKey,
    protocolVersion: value.protocolVersion,
    ...("localProtocol" in value ? { localProtocol: localProtocol(value.localProtocol) } : {}),
  };
}

export function summarizeStatus(status: unknown): Record<string, string | number | boolean | null> {
  const first: unknown = Array.isArray(status) ? status[0] : status;
  const result: Record<string, string | number | boolean | null> = {};
  if (!first || typeof first !== "object") return result;
  for (const key of ["battery", "state", "error_code", "in_cleaning", "in_returning", "fw_ver", "common_status", "grab_status"]) {
    const value: unknown = Reflect.get(first, key);
    if (value === null || typeof value === "string" || typeof value === "number" || typeof value === "boolean") {
      result[key] = value;
    }
  }
  return result;
}
