type Timestamp = number | string;

export const FRAME_MAX_AGE_MS = 30_000;
export const STATE_STALE_MS = 15_000;
export const PREVIEW_LEASE_MS = 60_000;

export function timestamp(value: Timestamp | null | undefined): number | null {
  if (value === null || value === undefined) return null;
  const result = typeof value === "number" ? value : Date.parse(value);
  return Number.isFinite(result) ? result : null;
}

export function isFresh(at: Timestamp | null | undefined, now: number, maxAge: number): boolean {
  const time = timestamp(at);
  return time !== null && now >= time && now - time <= maxAge;
}

export function ageLabel(at: Timestamp | null | undefined, now: number): string {
  const time = timestamp(at);
  if (time === null) return "Not observed";
  const age = Math.max(0, Math.floor((now - time) / 1000));
  if (age < 60) return `${age}s ago`;
  if (age < 3600) return `${Math.floor(age / 60)}m ago`;
  return `${Math.floor(age / 3600)}h ago`;
}

export function timeLabel(at: Timestamp | null | undefined): string {
  const time = timestamp(at);
  return time === null ? "Not observed" : new Date(time).toLocaleTimeString();
}

export function record(value: unknown): Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value)
    ? value as Record<string, unknown> : {};
}

export function scalar(value: unknown): string {
  return typeof value === "string" || typeof value === "number" || typeof value === "boolean"
    ? String(value) : "--";
}

export function batteryPercent(value: unknown): number | null {
  return typeof value === "number" && Number.isFinite(value) && value >= 0 && value <= 100
    ? value : null;
}

export function errorMessage(error: unknown): string {
  return error instanceof Error ? error.message : "The request could not be completed.";
}

export function pollDelay(failures: number): number {
  return Math.min(30_000, 3_000 * 2 ** Math.min(failures, 4));
}

export function takeFragmentToken(location: Pick<Location, "hash" | "pathname" | "search">,
  history: Pick<History, "replaceState">): string | null {
  const params = new URLSearchParams(location.hash.slice(1));
  if (!params.has("token")) return null;
  const token = params.get("token");
  history.replaceState(null, "", `${location.pathname}${location.search}`);
  return token || null;
}
