import type { DeviceCredentials } from "./cloud.ts";

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
