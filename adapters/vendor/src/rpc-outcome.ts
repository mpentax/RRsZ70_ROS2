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
