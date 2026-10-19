import { isIP } from "node:net";
import { open, mkdtemp, writeFile, unlink } from "node:fs/promises";
import { constants, rmSync } from "node:fs";
import { resolve, join } from "node:path";
import { tmpdir } from "node:os";
import { randomUUID } from "node:crypto";
import { jpegImageDimensions, CAMERA_NAMES, rosRoute } from "@rrsz70/http-contracts";

export type TaskInput = { kind: string; goal?: Record<string, unknown>; [key: string]: unknown };
export interface GoalCatalogResponse {
  goals: { kind: string; topic: string; action: string; fields: string[] }[];
  context_primitives?: { name: string; signature: string; summary: string }[];
}
export interface GoalValidationResponse {
  valid: boolean;
  kind?: string;
  topic?: string;
  action?: string;
  error?: string;
}

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
import { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import type { CallToolResult } from "@modelcontextprotocol/sdk/types.js";
import { z } from "zod/v4";
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
const GET_PATHS = new Set(["/api/ros/state", "/api/ros/map", "/api/ros/goals", "/api/ros/tasks"]);
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
    const get = (GET_PATHS.has(path) && (path !== '/api/ros/tasks' || payload === undefined)) || /^\/api\/ros\/tasks\/[0-9a-f-]{36}$/.test(path);
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
      const metadata: Record<string, unknown> = { receivedAt: response.headers.has('x-roboclaw-received-at') && Number.isSafeInteger(sampleReceivedAt) && sampleReceivedAt > 0 ? sampleReceivedAt : null, capturedAt: null,
        ...(camera === "front" || camera === "gripper" ? { camera } : {}),
        ...(sizeCamera === "front" || sizeCamera === "gripper" ? { sizeCamera } : {}),
        captureProvenance: null, lensVerification: "unknown", mimeType: "image/jpeg", feedLensVerified: false,
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

export function createRoboClawMcp(client: RoboClawMcpClient): McpServer {
  const server = new McpServer({ name: "roboclaw-local", version: "0.1.0" }, { instructions: OPERATOR_INSTRUCTIONS });
  server.registerTool("get_state", {
    title: "Get ROS robot state", description: "Read shared ROS robot state, battery, camera status, arm estimate, and chassis calibration.",
    annotations: READONLY,
  }, async () => {
    try {
      return textResult(await client.request('/api/ros/state'));
    } catch (error) { return toolError(error); }
  });
  server.registerTool("refresh_status", {
    title: "Read shared status", description: "Read cached ROS robot status and telemetry.",
    annotations: READONLY,
  }, async () => {
    try { return textResult(await client.request("/api/ros/state")); } catch (error) { return toolError(error); }
  });
  server.registerTool("get_camera_diagnostics", {
    title: "Read camera status", description: "Read ROS camera and stream connection status.",
    inputSchema: {}, annotations: READONLY,
  }, async () => {
    try { return textResult(await client.request("/api/ros/state")); } catch (error) { return toolError(error); }
  });
  server.registerTool("select_camera", {
    title: "Select shared camera", description: "Switch the active camera (`front` or `gripper`). Gripper camera requires pickup mode with the arm deployed.",
    inputSchema: { camera: z.enum(["front", "gripper"]) },
    annotations: { readOnlyHint: false, destructiveHint: false, idempotentHint: true, openWorldHint: false },
  }, async (payload) => {
    try { return textResult(await client.request("/api/ros/camera/select", 23000, payload)); } catch (error) { return toolError(error); }
  });
  server.registerTool("get_camera_frame", {
    title: "Get shared camera frame", description: "Read the latest cached JPEG frame and metadata from the active or specified camera.",
    inputSchema: { camera: z.enum(CAMERA_NAMES).optional() },
    annotations: READONLY,
  }, async ({ camera }) => {
    try {
      const frame = await client.request("/api/ros/frame", 5000, camera === undefined ? {} : { camera });
      if (!Buffer.isBuffer(frame)) throw new Error("No frame.");
      const metadata = client.frameMetadata.get(frame) ?? { receivedAt: Date.now(), capturedAt: null,
        captureProvenance: null, mimeType: "image/jpeg", feedLensVerified: false, lensVerification: "unknown" };
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
  }, async () => {
    try { const state = await client.request("/api/ros/state") as { arm: unknown }; return textResult(state.arm); } catch (error) { return toolError(error); }
  });
  server.registerTool("set_arm_anchor", {
    title: "Set arm anchor",
    description: "Set the arm estimator anchor (`stowed`, `deployed`, or `unknown`) without moving the robot.",
    inputSchema: { anchor: z.enum(["stowed", "deployed", "unknown"]), reason: z.string().regex(/^[A-Za-z0-9 _.-]{0,64}$/).optional() },
    annotations: { readOnlyHint: false, destructiveHint: false, idempotentHint: true, openWorldHint: false },
  }, async (payload) => {
    try { return textResult(await client.request("/api/ros/arm/anchor", 5000, payload)); } catch (error) { return toolError(error); }
  });
  server.registerTool("robot_rpc", {
    title: "Advanced robot RPC", description: "Send a vendor RPC method and params through the ROS scheduler.",
    inputSchema: rpcSchema, annotations: ACTUATION,
  }, async (payload) => {
    try {
      return textResult(await client.request('/api/ros/rpc', (payload.timeoutMs ?? 12000) + 18000, payload));
    } catch (error) { return toolError(error); }
  });
  const idSchema = z.string().uuid();
  const taskSchema = z.discriminatedUnion('kind', [
    z.object({kind: z.literal('deliver_garment'), goal: z.object({category: z.string().max(32), drop_x_m: z.number().min(-100).max(100), drop_y_m: z.number().min(-100).max(100), max_duration_s: z.number().min(1).max(3600), allow_towing: z.boolean(), dry_run: z.boolean()}).strict()}).strict(),
    z.object({kind: z.literal('patrol_collect'), goal: z.object({drop_x_m: z.number().min(-100).max(100), drop_y_m: z.number().min(-100).max(100), max_duration_s: z.number().min(1).max(3600), max_items: z.number().int().min(1).max(1000), allow_towing: z.boolean(), dry_run: z.boolean()}).strict()}).strict(),
    z.object({kind: z.literal('pin_and_go'), goal: z.object({target_x_m: z.number().min(-100).max(100), target_y_m: z.number().min(-100).max(100), max_duration_s: z.number().min(1).max(600), dry_run: z.boolean()}).strict()}).strict(),
    z.object({kind: z.literal('pick'), goal: z.object({color: z.string().regex(/^[A-Za-z0-9_-]{1,32}$/), dry_run: z.boolean()}).strict()}).strict(),
    z.object({kind: z.literal('stack'), goal: z.object({base_color: z.string().regex(/^[A-Za-z0-9_-]{1,32}$/), middle_color: z.string().regex(/^[A-Za-z0-9_-]{1,32}$/), top_color: z.string().regex(/^[A-Za-z0-9_-]{1,32}$/), dry_run: z.boolean()}).strict()}).strict(),
    z.object({kind: z.literal('base_pulse'), goal: z.object({linear_mps: z.number().min(-.1).max(.1), angular_radps: z.number().min(-.5).max(.5), duration: z.object({sec: z.literal(0), nanosec: z.number().int().min(1).max(800000000)}).strict()}).strict()}).strict(),
    z.object({kind: z.literal('arm_step'), goal: z.object({joint: z.union([z.literal(3), z.literal(4), z.literal(5), z.literal(34), z.literal(43)]), direction: z.union([z.literal(-1), z.literal(1)])}).strict()}).strict(),
    z.object({kind: z.literal('hand_step'), goal: z.object({direction: z.number().int().min(1).max(4)}).strict()}).strict(),
    z.object({kind: z.literal('arm_operation'), goal: z.object({operation: z.number().int().min(1).max(9)}).strict()}).strict(),
    z.object({kind: z.literal('deploy_arm'), goal: z.object({settle: z.boolean()}).strict()}).strict(),
    z.object({kind: z.literal('align_target'), goal: z.object({color: z.string().regex(/^[A-Za-z0-9_-]{1,32}$/)}).strict()}).strict(),
    z.object({kind: z.literal('move_arm'), goal: z.object({m3_deg: z.number().min(90).max(270), m4_deg: z.number().min(35).max(170), m5_deg: z.number().min(0).max(155)}).strict()}).strict(),
    z.object({kind: z.literal('grasp_and_lift'), goal: z.object({lift_steps: z.number().int().min(0).max(10), verify_color: z.string().regex(/^[A-Za-z0-9_-]{0,32}$/)}).strict()}).strict(),
    z.object({kind: z.literal('release_and_stow'), goal: z.object({put_down: z.boolean()}).strict()}).strict(),
  ]).or(z.object({ kind: z.string().min(1) }).passthrough());
  const goalSchema = taskSchema;
  server.registerTool('start_task', {title: 'Start bounded ROS action', description: 'Start a ROS action or manipulation primitive and return its goal UUID.', inputSchema: {task: goalSchema, validateOnly: z.boolean().optional()}, annotations: ACTUATION}, async ({task, validateOnly}) => {
    try {return textResult(validateOnly === true ? await client.validateGoal(task) : await client.startTask(task));} catch (error) {return toolError(error);}
  });
  server.registerTool('list_tasks', {title:'List shared tasks', description:'List active and recent ROS tasks.', annotations:READONLY}, async () => {
    try {return textResult(await client.request('/api/ros/tasks'));} catch (error) {return toolError(error);}
  });
  server.registerTool('get_task', {title:'Get shared task', description:'Read status, phase, and result for a ROS task UUID.', inputSchema:{id:idSchema}, annotations:READONLY}, async ({id}) => {
    try {return textResult(await client.request(`/api/ros/tasks/${id}`));} catch (error) {return toolError(error);}
  });
  server.registerTool('cancel_task', {title:'Cancel ROS task', description:'Request cancellation of a ROS task by UUID.', inputSchema:{id:idSchema}, annotations:ACTUATION}, async ({id}) => {
    try {return textResult(await client.request(`/api/ros/tasks/${id}/cancel`, 5000, {}));} catch (error) {return toolError(error);}
  });
  server.registerTool('stop', {title:'Priority stop', description:'Send a priority stop through ROS (scope: 1=BASE, 2=ARM, 3=ALL).', inputSchema:{scope:z.number().int().min(1).max(3).default(3)}, annotations:ACTUATION}, async (payload) => {
    try {return textResult(await client.request('/api/ros/stop', 18000, payload));} catch (error) {return toolError(error);}
  });
  server.registerTool('control_session', {title:'Stop or resume control and streams', description:'Enable or suspend robot control and shared streams.', inputSchema:{enabled:z.boolean()}, annotations:ACTUATION}, async (payload) => {
    try {return textResult(await client.request('/api/ros/control/session', 40000, payload));} catch (error) {return toolError(error);}
  });
  server.registerTool('get_map', {title:'Read shared ROS map', description:'Read the current ROS OccupancyGrid map.', annotations:READONLY}, async () => {
    try {return textResult(await client.request('/api/ros/map'));} catch (error) {return toolError(error);}
  });
  return server;
}
