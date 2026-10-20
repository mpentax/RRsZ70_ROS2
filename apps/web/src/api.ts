export function record(value: unknown): Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value)
    ? value as Record<string, unknown> : {};
}

export type UiOutcome = "rejected" | "failed" | "accepted" | "in-progress" | "acknowledged" | "unknown";
export const UNKNOWN_ACTION_MESSAGE = "Outcome unknown.";

export function actionOutcome(value: unknown): UiOutcome {
  const data = record(value);
  const evidence = record(data.actionOutcome);
  const envelope = Object.keys(evidence).length ? evidence : data;
  const kind = envelope.outcome;
  if (["unknown", "rejected", "failed", "accepted", "in-progress", "acknowledged"].includes(String(kind))) {
    return kind as UiOutcome;
  }
  const rosOutcome = record(data.outcome);
  if (rosOutcome.dispatch === 2 || data.dispatch === "not_sent") return "rejected";
  if (rosOutcome.response === 2) return "acknowledged";
  if (rosOutcome.response === 3 || rosOutcome.response === 4) return "failed";
  if (rosOutcome.dispatch === 1 && rosOutcome.response === 1) return "accepted";
  if (data.status === "accepted" || data.status === "pending") return "accepted";
  if (data.status === "executing") return "in-progress";
  if (data.status === "succeeded") return "acknowledged";
  if (data.status === "failed" || data.status === "cancelled") return "failed";
  if (data.status === "rejected") return "rejected";
  if (data.acknowledged === true) return "acknowledged";
  if (data.accepted === true) return "accepted";
  const state = record(data.state);
  if (data.inProgress === true || state.busy === true ||
    ["preparing", "moving", "braking", "stopping", "running"].includes(String(state.phase))) return "in-progress";
  return "unknown";
}

export function outcomeMessage(outcome: UiOutcome): string {
  switch (outcome) {
    case "rejected": return "Request rejected.";
    case "failed": return "Execution failed.";
    case "accepted": return "Request accepted.";
    case "in-progress": return "Request in progress.";
    case "acknowledged": return "Command acknowledged.";
    case "unknown": return UNKNOWN_ACTION_MESSAGE;
  }
}

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
      throw new Error("The service request timed out.");
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

export type Task = { id: string; kind: string; status: string; phase: number; feedback: unknown; result: unknown };
export type RosState = { available: boolean; session_enabled?: boolean | null; state: { connected: boolean; battery_known: boolean; battery_fraction: number; evidence: { valid: boolean; stale: boolean } } | null; camera: unknown; arm: unknown; calibration: unknown };
export type Setup = { device: { id: string; name: string; model: string; host: string; localProtocol: string } | null; accountConfigured: boolean; pinConfigured: boolean };
export type TaskRequest = { kind: string; goal: Record<string, unknown> };
export type GoalCatalog = { goals: { kind: string; name: string; topic: string; action: string; fields: string[]; navigation: boolean; description: string }[]; context_primitives: { name: string; signature: string; summary: string }[] };
export interface RpcInput { method: string; params: unknown; transport?: "local" | "cloud"; timeoutMs?: number }
export interface RpcOutcome { outcome: { command_id: unknown; dispatch: number; response: number; error_code: string }; result_json: string }
export const robotRpc = (input: RpcInput) => json<RpcOutcome>("/api/ros/rpc", { body: input, action: true, timeout: (input.timeoutMs ?? 12000) + 18000 });
export const terminal = (task: Task) => ["succeeded", "failed", "cancelled", "rejected"].includes(task.status);
export const taskPath = (id: string) => {
  if (!/^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/.test(id)) throw new Error("Invalid task ID.");
  return `/api/ros/tasks/${id}`;
};
export const listGoals = (signal?: AbortSignal) => json<GoalCatalog>("/api/ros/goals", { signal });
export const validateGoal = (input: { kind: string; goal: Record<string, unknown>; simulate?: boolean; context?: Record<string, unknown> }) => json("/api/ros/goals/validate", { body: input });
export const startTask = (task: TaskRequest) => json<Task>("/api/ros/tasks", { body: task, action: true, timeout: 5000 });
export const listTasks = (signal?: AbortSignal) => json<{ tasks: Task[] }>("/api/ros/tasks", { signal });
export const cancelTask = (id: string) => json<{ id: string; cancel_requested: boolean }>(`${taskPath(id)}/cancel`, { body: {}, action: true, timeout: 5000 });
export const stopRobot = () => json("/api/ros/stop", { body: { scope: 3 }, action: true, timeout: 18000 });
export type ControlSession = { enabled: boolean; streams_released: boolean; cessation_observed: boolean; outcomes: { dispatch: number; response: number; error_code: string }[] };
export const setControlSession = (enabled: boolean) => json<ControlSession>("/api/ros/control/session", { body: { enabled }, action: true, timeout: 38000 });
