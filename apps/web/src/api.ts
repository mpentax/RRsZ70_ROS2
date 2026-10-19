import { record } from "./helpers.ts";
import { actionOutcome, outcomeMessage, UNKNOWN_ACTION_MESSAGE, type UiOutcome } from "./outcomes.ts";

export const SESSION_EXPIRED = "roboclaw:session-expired";

export class ApiError extends Error {
  readonly status: number;
  readonly outcome: UiOutcome;
  evidence?: Record<string, unknown>;

  constructor(message: string, status: number, outcome: UiOutcome = "unknown") {
    super(message);
    this.name = "ApiError";
    this.status = status;
    this.outcome = outcome;
  }
}

interface RequestOptions {
  method?: "GET" | "POST" | "DELETE";
  body?: unknown;
  signal?: AbortSignal;
  timeout?: number;
  keepalive?: boolean;
  /** Mark actuation explicitly; no request is automatically retried. */
  action?: boolean;
}

async function request<T>(path: string, options: RequestOptions,
  consume: (response: Response) => Promise<T>): Promise<T> {
  const isAction = options.action ?? false;
  if (options.signal?.aborted && isAction) {
    throw new ApiError("Request cancelled before dispatch; no command was sent.", 0, "rejected");
  }
  const controller = new AbortController();
  const abort = () => controller.abort();
  options.signal?.addEventListener("abort", abort, { once: true });
  if (options.signal?.aborted) controller.abort();
  const timer = setTimeout(abort, options.timeout ?? 15_000);
  try {
    const response = await fetch(path, {
      method: options.method ?? (options.body === undefined ? "GET" : "POST"),
      credentials: "same-origin",
      cache: "no-store",
      redirect: "error",
      headers: options.body === undefined ? undefined : { "Content-Type": "application/json" },
      body: options.body === undefined ? undefined : JSON.stringify(options.body),
      signal: controller.signal,
      keepalive: options.keepalive ?? false,
    });
    if (response.status === 401 && path !== "/api/session") {
      window.dispatchEvent(new Event(SESSION_EXPIRED));
    }
    if (!response.ok) {
      let message = `Request failed (HTTP ${response.status}).`;
      let outcome: UiOutcome = "unknown";
      let evidence: Record<string, unknown> | undefined;
      if (response.headers.get("content-type")?.includes("application/json")) {
        try {
          const data = record(await response.json());
          if (path !== "/api/session") evidence = data;
          if (typeof data.error === "string") message = data.error;
          outcome = actionOutcome(data);
        } catch { /* An unreadable rejection body cannot establish execution evidence. */ }
      }
      // Authentication errors must never echo submitted credentials.
      if (path === "/api/session") message = record(options.body).setup === true
        ? `Local setup unavailable (HTTP ${response.status}). Open the configured localhost URL; if setup is complete, sign in with your camera PIN.`
        : `Service access failed (HTTP ${response.status}). Check your camera PIN.`;
      const error = new ApiError(isAction ? `${message} ${outcomeMessage(outcome)}` : message, response.status, outcome);
      error.evidence = evidence;
      throw error;
    }
    return await consume(response);
  } catch (error) {
    if (error instanceof ApiError) throw error;
    if (isAction) {
      const reason = controller.signal.aborted ? "Response wait was cancelled or timed out." : "The service response could not be obtained or read.";
      throw new ApiError(`${reason} ${UNKNOWN_ACTION_MESSAGE}`, 0);
    }
    if (controller.signal.aborted && !options.signal?.aborted) {
      throw new Error("The service took too long to respond. Outcome unknown; refresh state and verify physically before deciding whether to retry an action.");
    }
    throw error;
  } finally {
    clearTimeout(timer);
    options.signal?.removeEventListener("abort", abort);
  }
}

export function json<T>(path: string, options: RequestOptions = {}): Promise<T> {
  return request(path, options, async (response) => {
    if (!response.headers.get("content-type")?.includes("application/json")) {
      throw new Error("The service returned an unexpected response instead of JSON.");
    }
    return await response.json() as T;
  });
}

export function deleteSession(signal?: AbortSignal): Promise<void> {
  return request("/api/session", { method: "DELETE", signal }, async (response) => {
    await response.body?.cancel();
  });
}

export function snapshot(signal: AbortSignal): Promise<Blob> {
  return request("/api/ros/frame", { body: {}, signal, timeout: 5_000 }, async (response) => {
    if (response.headers.get("content-type")?.split(";")[0]?.trim() !== "image/jpeg") {
      if (response.headers.get("content-type")?.includes("application/json")) {
        const data = record(await response.json());
        throw new Error(typeof data.error === "string" ? data.error : "No JPEG image was returned.");
      }
      throw new Error("The camera returned an unexpected image format.");
    }
    const image = await response.blob();
    if (!image.size) throw new Error("The camera returned an empty image.");
    if (image.size > 5 * 1024 * 1024) throw new Error("The camera image exceeds the 5 MiB limit.");
    return image;
  });
}

export interface SessionStatus {
  authenticated: boolean;
  scope?: "setup" | "control" | null;
  setupAvailable?: boolean;
}

export async function initialSession(pin?: string | null): Promise<SessionStatus> {
  try {
    return await json<SessionStatus>("/api/session", pin ? { body: { pin } } : {});
  } finally {
    pin = null;
  }
}
