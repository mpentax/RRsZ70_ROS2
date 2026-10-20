import { isIP } from "node:net";
import { open, mkdtemp, writeFile, unlink } from "node:fs/promises";
import { constants, rmSync } from "node:fs";
import { resolve, join } from "node:path";
import { tmpdir } from "node:os";
import { randomUUID } from "node:crypto";
import { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import type { CallToolResult } from "@modelcontextprotocol/sdk/types.js";
import { z } from "zod/v4";
import { jpegImageDimensions, CAMERA_NAMES, rosRoute, type GoalCatalog as GoalCatalogResponse, type GoalValidationResponse } from "@rrsz70/http-contracts";

export type TaskInput = { kind: string; goal?: Record<string, unknown>; [key: string]: unknown };
export type { GoalCatalogResponse, GoalValidationResponse };

let frameDirectory: string | undefined;
let previousFrame: string | undefined;
export const OPERATOR_INSTRUCTIONS = "Local MCP interface for the Roborock Saros Z70 via the ROS 2 web gateway. Never include credentials, tokens, or PINs in tool arguments or chat.";

export async function privateMcpFrame(frame: Buffer): Promise<string> {
  if (!frameDirectory) {
    frameDirectory = await mkdtemp(join(tmpdir(), "roboclaw-mcp-frame-"));
    process.once("exit", () => { if (frameDirectory) rmSync(frameDirectory, { recursive: true, force: true }); });
  }
  const path = join(frameDirectory, `${randomUUID()}.jpg`);
  await writeFile(path, frame, { mode: 0o600, flag: "wx" });
  const old = previousFrame;
  previousFrame = path;
  if (old) await unlink(old).catch(() => {});
  return path;
}
const rpcSchema = {
  method: z.string().regex(/^[A-Za-z][A-Za-z0-9_]{0,127}$/),
  params: z.any().optional(),
  transport: z.enum(["local", "cloud"]).optional(),
  timeoutMs: z.number().int().min(1).max(30000).optional(),
  takeover: z.boolean().optional(),
};
const MAX_JSON = 16 * 1024 * 1024;
const MAX_ERROR_JSON = 16 * 1024;
const MAX_JPEG = 5 * 1024 * 1024;
const READONLY = { readOnlyHint: true, destructiveHint: false, openWorldHint: false } as const;
const ACTUATION = { readOnlyHint: false, destructiveHint: true, idempotentHint: false, openWorldHint: false } as const;
/** Refuse remote origins and injected URLs; the service credential must stay on this host. */
export function mcpServiceOrigin(value: string): URL {
  let url: URL;
  try { url = new URL(value); } catch { throw new Error("Invalid local RoboClaw origin."); }
  if (url.protocol !== "http:" || !["127.0.0.1", "[::1]"].includes(url.hostname) ||
    url.username || url.password || url.pathname !== "/" || url.search || url.hash ||
    !Number.isSafeInteger(Number(url.port)) || !url.port || Number(url.port) < 57000 || Number(url.port) > 57100) {
    throw new Error("MCP requires a loopback RoboClaw HTTP origin on port 57000–57100.");
  }
  // No DNS names (including localhost) and no redirects with credentials attached.
  if (isIP(url.hostname.replace(/^\[|\]$/g, "")) === 0) throw new Error("MCP requires a numeric loopback address.");
  return url;
}

export async function readMcpToken(path: string): Promise<string> {
  const file = await open(resolve(path), constants.O_RDONLY | constants.O_NOFOLLOW);
  try {
    const stats = await file.stat();
    if (!stats.isFile() || stats.size !== 43 || (stats.mode & 0o077) !== 0 ||
      (typeof process.getuid === "function" && stats.uid !== process.getuid())) {
      throw new Error("Unsafe local service token file.");
    }
    const token = await file.readFile("utf8");
    if (!/^[A-Za-z0-9_-]{43}$/.test(token)) throw new Error("Invalid local service token.");
    return token;
  } finally { await file.close(); }
}

export class RoboClawMcpClient {
  readonly frameMetadata = new WeakMap<Buffer, Record<string, unknown>>();
  private readonly origin: URL;
  private readonly token: string;
  private readonly transport: typeof fetch;
  constructor(origin: URL, token: string, transport: typeof fetch = fetch) {
    this.origin = origin; this.token = token; this.transport = transport;
  }

  async request(path: string, timeoutMs = 12000, payload?: Record<string, unknown>): Promise<unknown> {
    const get = rosRoute("GET", path) && (path !== "/api/ros/tasks" || payload === undefined);
    const action = !get && path !== '/api/ros/frame';
    if (!rosRoute(get ? 'GET' : 'POST', path)) throw new Error("Unsupported RoboClaw MCP route.");
    if (action && payload === undefined) throw new Error("RPC payload is required.");
    if (get && payload !== undefined) throw new Error("Read routes do not accept RPC payloads.");
    if (path === "/api/ros/frame" && payload !== undefined &&
        (Object.keys(payload).some(key => key !== "camera") ||
         (payload.camera !== undefined && !CAMERA_NAMES.includes(payload.camera as typeof CAMERA_NAMES[number]))))
      throw new Error("Invalid camera read payload.");
    const response = await this.transport(new URL(path, this.origin), {
      method: get ? "GET" : "POST",
      headers: { Authorization: `Bearer ${this.token}`, ...(get ? {} : { "Content-Type": "application/json" }) },
      ...(get ? {} : { body: JSON.stringify(payload ?? {}) }),
      redirect: "error", signal: AbortSignal.timeout(timeoutMs),
    });
    if (!response.ok) {
      const evidence = rosErrorEvidence(await boundedErrorEvidence(response));
      throw new RoboClawHttpError(response.status, evidence);
    }
    const jpeg = path === "/api/ros/frame";
    const contentType = response.headers.get("content-type")?.split(";")[0]?.trim().toLowerCase();
    const maximum = jpeg ? MAX_JPEG : MAX_JSON;
    const declared = Number(response.headers.get("content-length"));
    if (response.headers.has("content-length") && (!Number.isSafeInteger(declared) || declared > maximum || declared < 0)) {
      await response.body?.cancel(); throw new Error("RoboClaw read response exceeds limit.");
    }
    const chunks: Uint8Array[] = [];
    let length = 0;
    if (!response.body) throw new Error("RoboClaw read returned no body.");
    for await (const chunk of response.body) {
      length += chunk.byteLength;
      if (length > maximum) { await response.body.cancel().catch(() => {}); throw new Error("RoboClaw read response exceeds limit."); }
      chunks.push(chunk);
    }
    const bytes = Buffer.concat(chunks, length);
    if (jpeg) {
      if (contentType !== "image/jpeg" || bytes.length < 4 || bytes.length > MAX_JPEG ||
        bytes[0] !== 0xff || bytes[1] !== 0xd8 || bytes.at(-2) !== 0xff || bytes.at(-1) !== 0xd9) {
        throw new Error("RoboClaw did not return a bounded JPEG frame.");
      }
      const frameId = response.headers.get("x-roboclaw-frame-id");
      const sessionId = response.headers.get("x-roboclaw-camera-session");
      const opaque = (value: string | null) => value !== null && /^[A-Za-z0-9_-]{1,128}$/.test(value);
      const camera = response.headers.get("x-roboclaw-camera");
      const sizeCamera = response.headers.get("x-roboclaw-size-camera");
      const sampleReceivedAt = Number(response.headers.get('x-roboclaw-received-at'));
      const metadata: Record<string, unknown> = { receivedAt: response.headers.has('x-roboclaw-received-at') && Number.isSafeInteger(sampleReceivedAt) && sampleReceivedAt > 0 ? sampleReceivedAt : null,
        ...(camera === "front" || camera === "gripper" ? { camera } : {}),
        ...(sizeCamera === "front" || sizeCamera === "gripper" ? { sizeCamera } : {}),
        mimeType: "image/jpeg",
        ...(opaque(frameId) && opaque(sessionId) ? { frameId, cameraSessionId: sessionId } : {}) };
      try { metadata.dimensions = jpegImageDimensions(bytes); } catch { /* No dimensions claim from a byte-only JPEG. */ }
      this.frameMetadata.set(bytes, metadata);
      return bytes;
    }
    if (contentType !== "application/json" || bytes.length > MAX_JSON) throw new Error("Invalid RoboClaw JSON response.");
    return JSON.parse(bytes.toString("utf8")) as unknown;
  }

  listGoals(timeoutMs = 12000): Promise<GoalCatalogResponse> {
    return this.request("/api/ros/goals", timeoutMs) as Promise<GoalCatalogResponse>;
  }

  validateGoal(task: TaskInput, timeoutMs = 5000): Promise<GoalValidationResponse> {
    return this.request("/api/ros/goals/validate", timeoutMs, task) as Promise<GoalValidationResponse>;
  }

  startTask(task: TaskInput, timeoutMs = 5000): Promise<unknown> {
    return this.request("/api/ros/tasks", timeoutMs, task);
  }
}

export class RoboClawHttpError extends Error {
  readonly status: number;
  readonly evidence?: Record<string, unknown>;
  constructor(status: number, evidence?: Record<string, unknown>) {
    super(`RoboClaw returned HTTP ${status}.`); this.status = status; this.evidence = evidence;
  }
}

function rosErrorEvidence(data: Record<string, unknown> | undefined): Record<string, unknown> | undefined {
  if (!data) return undefined;
  const codes = ['ROS_UNAVAILABLE', 'RESPONSE_TIMEOUT', 'INVALID_INPUT', 'REJECTED', 'TASK_NOT_FOUND', 'TASK_FINISHED', 'CANCEL_REJECTED', 'OBSERVATION_GAP', 'CAPACITY'];
  return { ...(typeof data.error === 'string' && codes.includes(data.error) ? { error: data.error } : {}),
    ...(['unknown', 'not_sent', 'sent'].includes(String(data.dispatch)) ? { dispatch: data.dispatch } : {}),
    ...(typeof data.id === 'string' && /^[0-9a-f-]{36}$/.test(data.id) ? { id: data.id } : {}) };
}

/** Non-2xx bodies are untrusted: bounded bytes, JSON only, closed evidence only. */
async function boundedErrorEvidence(response: Response): Promise<Record<string, unknown> | undefined> {
  try {
    if (response.headers.get("content-type")?.split(";")[0]?.trim().toLowerCase() !== "application/json" ||
      (response.headers.has("content-length") && (!Number.isSafeInteger(Number(response.headers.get("content-length"))) ||
        Number(response.headers.get("content-length")) < 0 || Number(response.headers.get("content-length")) > MAX_ERROR_JSON))) {
      await response.body?.cancel(); return undefined;
    }
    if (!response.body) return undefined;
    const reader = response.body.getReader();
    const chunks: Uint8Array[] = []; let size = 0;
    try {
      while (true) {
        const { value, done } = await reader.read(); if (done) break;
        size += value.byteLength;
        if (size > MAX_ERROR_JSON) { await reader.cancel(); return undefined; }
        chunks.push(value);
      }
    } finally { reader.releaseLock(); }
    const data: unknown = JSON.parse(Buffer.concat(chunks, size).toString("utf8"));
    return data !== null && typeof data === "object" && !Array.isArray(data) ? data as Record<string, unknown> : undefined;
  } catch { return undefined; }
}
const textResult = (value: unknown): CallToolResult => ({ content: [{ type: "text", text: JSON.stringify(value) }] });
const toolError = (error: unknown): CallToolResult => ({ isError: true, content: [{ type: "text", text: JSON.stringify(
  error instanceof RoboClawHttpError ? { status: error.status, ...(error.evidence ?? {}) }
    : { dispatch: "unknown", response: "transport_error", error: "Local service response unavailable." }) }] });
const jsonTool = <T = unknown>(fn: (arg: T) => Promise<unknown>) => async (arg: T): Promise<CallToolResult> => {
  try { return textResult(await fn(arg)); } catch (error) { return toolError(error); }
};

export function createRoboClawMcp(client: RoboClawMcpClient): McpServer {
  const server = new McpServer({ name: "roboclaw-local", version: "0.1.0" }, { instructions: OPERATOR_INSTRUCTIONS });
  const readState = jsonTool(() => client.request("/api/ros/state"));
  for (const [name, title, description] of [
    ["get_state", "Get ROS robot state", "Read shared ROS robot state, battery, camera status, arm estimate, and chassis calibration."],
    ["refresh_status", "Read shared status", "Read cached ROS robot status and telemetry."],
    ["get_camera_diagnostics", "Read camera status", "Read ROS camera and stream connection status."],
  ] as const) {
    server.registerTool(name, { title, description, annotations: READONLY }, readState);
  }
  server.registerTool("select_camera", {
    title: "Select shared camera", description: "Switch the active camera (`front` or `gripper`). Gripper camera requires pickup mode with the arm deployed.",
    inputSchema: { camera: z.enum(["front", "gripper"]) },
    annotations: { readOnlyHint: false, destructiveHint: false, idempotentHint: true, openWorldHint: false },
  }, jsonTool((payload) => client.request("/api/ros/camera/select", 23000, payload)));
  server.registerTool("get_camera_frame", {
    title: "Get shared camera frame", description: "Read the latest cached JPEG frame and metadata from the active or specified camera.",
    inputSchema: { camera: z.enum(CAMERA_NAMES).optional() },
    annotations: READONLY,
  }, async ({ camera }) => {
    try {
      const frame = await client.request("/api/ros/frame", 5000, camera === undefined ? {} : { camera });
      if (!Buffer.isBuffer(frame)) throw new Error("No frame.");
      const metadata = client.frameMetadata.get(frame) ?? { receivedAt: Date.now(), mimeType: "image/jpeg" };
      if (process.env.ROBOCLAW_MCP_IMAGE_HANDOFF === "private-file") {
        return textResult({ ...metadata,
          imagePath: await privateMcpFrame(frame), temporaryPrivateFile: true,
          note: "Read imagePath with the local image reader." });
      }
      return {
        content: [
          { type: "text", text: JSON.stringify(metadata) },
          { type: "image", data: frame.toString("base64"), mimeType: "image/jpeg" },
        ],
      };
    } catch (error) { return toolError(error); }
  });
  server.registerTool("get_arm_estimate", {
    title: "Get arm pose estimate",
    description: "Read ROS ArmEstimate (joint radians, REP-103 wrist coordinates, and validity).",
    inputSchema: {}, annotations: READONLY,
  }, jsonTool(async () => ((await client.request("/api/ros/state")) as { arm: unknown }).arm));
  server.registerTool("set_arm_anchor", {
    title: "Set arm anchor",
    description: "Set the arm estimator anchor (`stowed`, `deployed`, or `unknown`) without moving the robot.",
    inputSchema: { anchor: z.enum(["stowed", "deployed", "unknown"]), reason: z.string().regex(/^[A-Za-z0-9 _.-]{0,64}$/).optional() },
    annotations: { readOnlyHint: false, destructiveHint: false, idempotentHint: true, openWorldHint: false },
  }, jsonTool((payload) => client.request("/api/ros/arm/anchor", 5000, payload)));
  server.registerTool("robot_rpc", {
    title: "Advanced robot RPC", description: "Send a vendor RPC method and params through the ROS scheduler.",
    inputSchema: rpcSchema, annotations: ACTUATION,
  }, jsonTool((payload) => client.request('/api/ros/rpc', (payload.timeoutMs ?? 12000) + 18000, payload)));
  const idSchema = z.string().uuid();
  const taskSchema = z.object({
    kind: z.string().regex(/^[A-Za-z_][A-Za-z0-9_]{0,63}$/),
    goal: z.record(z.string(), z.unknown()),
  }).strict();
  server.registerTool('list_goals', {title: 'List ROS goals and GoalContext primitives', description: 'List all registered ROS 2 action goals, their required fields, and GoalContext authoring primitives.', annotations: READONLY},
    jsonTool(() => client.listGoals()));
  server.registerTool('start_task', {title: 'Start bounded ROS action', description: 'Start a ROS action or manipulation primitive and return its goal UUID (or validate/simulate without actuation).', inputSchema: {task: taskSchema, validateOnly: z.boolean().optional(), simulate: z.boolean().optional(), context: z.record(z.string(), z.unknown()).optional()}, annotations: ACTUATION},
    jsonTool(({task, validateOnly, simulate, context}) => validateOnly === true || simulate === true || context !== undefined ? client.validateGoal({...task, ...(simulate === true || context !== undefined ? {simulate: true} : {}), ...(context !== undefined ? {context} : {})}) : client.startTask(task)));
  server.registerTool('list_tasks', {title:'List shared tasks', description:'List active and recent ROS tasks.', annotations:READONLY},
    jsonTool(() => client.request('/api/ros/tasks')));
  server.registerTool('get_task', {title:'Get shared task', description:'Read status, phase, and result for a ROS task UUID.', inputSchema:{id:idSchema}, annotations:READONLY},
    jsonTool(({id}) => client.request(`/api/ros/tasks/${id}`)));
  server.registerTool('cancel_task', {title:'Cancel ROS task', description:'Request cancellation of a ROS task by UUID.', inputSchema:{id:idSchema}, annotations:ACTUATION},
    jsonTool(({id}) => client.request(`/api/ros/tasks/${id}/cancel`, 5000, {})));
  server.registerTool('stop', {title:'Priority stop', description:'Send a priority stop through ROS (scope: 1=BASE, 2=ARM, 3=ALL).', inputSchema:{scope:z.number().int().min(1).max(3).default(3)}, annotations:ACTUATION},
    jsonTool((payload) => client.request('/api/ros/stop', 18000, payload)));
  server.registerTool('control_session', {title:'Stop or resume control and streams', description:'Enable or suspend robot control and shared streams.', inputSchema:{enabled:z.boolean()}, annotations:ACTUATION},
    jsonTool((payload) => client.request('/api/ros/control/session', 40000, payload)));
  server.registerTool('get_map', {title:'Read shared ROS map', description:'Read the current ROS OccupancyGrid map along with session-scoped dynamic obstacles and tracked world objects when published.', annotations:READONLY},
    jsonTool(() => client.request('/api/ros/map')));
  return server;
}
