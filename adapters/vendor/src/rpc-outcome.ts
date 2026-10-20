/** Dispatch evidence is factual and per request; it never admits or blocks another request. */
export type DispatchEvidence = "not_sent" | "sent" | "unknown";
export class TransportError extends Error {
  readonly dispatch: DispatchEvidence;
  readonly reason?: 'cancelled';
  constructor(message: string, dispatch: DispatchEvidence, reason?: 'cancelled') { super(message); this.name = "TransportError"; this.dispatch = dispatch; this.reason = reason; }
}
export interface RpcOutcome {
  requestId: string;
  commandId?: string;
  attemptId?: string;
  attemptIndex?: number;
  dispatch: DispatchEvidence;
  response: "received" | "vendor_error" | "transport_error";
  result?: unknown;
  error?: string;
  vendorCode?: number;
  transportReason?: "timeout" | "capacity" | "invalid-input" | "unavailable" | "transport" | "cancelled";
  startedAt: number;
  finishedAt: number;
}
/** Do not forward credential-bearing values even from authenticated vendor payloads. */
export function publicRpcValue(value: unknown, secrets: readonly string[] = [], depth = 0): unknown {
  if (depth > 32) return null;
  if (typeof value === "string") return secrets.reduce((text, secret) => secret ? text.split(secret).join("[redacted]") : text, value);
  if (Array.isArray(value)) return value.map(item => publicRpcValue(item, secrets, depth + 1));
  if (value && typeof value === "object") return Object.fromEntries(Object.entries(value)
    .filter(([key]) => !/(password|passwd|token|secret|local.?key|hawk.?key|nonce|credential)/i.test(key))
    .map(([key, item]) => [publicRpcValue(key, secrets, depth + 1) as string, publicRpcValue(item, secrets, depth + 1)]));
  return value;
}

const SLOW_METHODS = new Set([
  "app_arm_out_compartment", "app_arm_in_compartment", "app_open_gripper", "app_grip",
  "app_pick_up", "app_put_down", "app_pick_up_enter_exit", "app_arm_move", "app_arm_direction_move",
  "app_stop_grasp", "app_start_program", "app_program_enter_exit",
]);

export const PRIORITY_STOPS = new Set([
  "app_rc_stop", "app_rc_end", "app_stop_grasp", "app_stop", "stop_goto_target",
]);

/** Response deadline used when a caller does not pass one. */
export function defaultRpcTimeoutMs(method: string): number {
  return SLOW_METHODS.has(method) ? 30_000 : 5_000;
}

/** Numeric transport sanitization only. ROS owns polling and semantic telemetry. */
export type NumericTelemetry = number | boolean | null | NumericTelemetry[] | { [key: string]: NumericTelemetry };
export function numericTelemetry(value: unknown, depth = 0): NumericTelemetry {
  if (depth > 5) return null;
  if (typeof value === "number") return Number.isFinite(value) ? value : null;
  if (typeof value === "boolean") return value;
  if (Array.isArray(value)) return value.slice(0, 64).map((item) => numericTelemetry(item, depth + 1));
  if (!value || typeof value !== "object") return null;
  const result: Record<string, NumericTelemetry> = {};
  for (const [key, item] of Object.entries(value).slice(0, 64)) {
    if (!/^[a-zA-Z][a-zA-Z0-9_]{0,63}$/.test(key) ||
      /key|token|secret|password|ssid|serial|duid|(^|_)(mac|ip|url)($|_)/i.test(key)) continue;
    const filtered = numericTelemetry(item, depth + 1);
    if (filtered !== null) result[key] = filtered;
  }
  return result;
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
