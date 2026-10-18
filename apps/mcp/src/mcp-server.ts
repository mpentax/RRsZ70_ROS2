import { isIP } from "node:net";
import { open, mkdtemp, writeFile, unlink } from "node:fs/promises";
import { constants, rmSync } from "node:fs";
import { resolve, join } from "node:path";
import { tmpdir } from "node:os";
import { randomUUID } from "node:crypto";
import { jpegImageDimensions, CAMERA_NAMES, rosRoute } from "@rrsz70/http-contracts";

let frameDirectory: string | undefined;
let previousFrame: string | undefined;
/** Operating doctrine every connected model receives, independent of the host's prompts or skills. */
export const OPERATOR_INSTRUCTIONS = `You are operating a real Roborock Saros Z70 for Michael. Keep the task moving; be careful, not rigid.

Stop motion only for: a robot error_code != 0, motion you did not command or much larger than commanded, a person/pet/obstacle entering the path, the robot starting to clean or dock unexpectedly, or Michael saying stop. Stopping means app_rc_stop (and app_rc_end when done) and telling Michael why.

NOT reasons to stop or abort a task: a rejected or noisy odometry/VO measurement, a missing or slow camera frame, a single transport error, robot state 39 after RC (normal remote mode), or "release unverified". Record it, then continue with the next step.

Ambiguous outcome (dispatch "unknown"): the command may have run. Resolve it by observing (odometry delta, a fresh camera frame, status), not by giving up. If it ran, count it and continue. If it clearly did not run, send it again. If you still can't tell, skip that step and continue. Never re-send a move blindly without looking first.

Retries: reads, app_rc_stop/end/start are retried once by RoboClaw already. Retry them again yourself if needed. Moves: retry after you have observed that the previous attempt did not run.

Calibration is ongoing: every RC move is paired with odometry by ROS to refine turn/forward gains and camera offset. Prefer many small natural moves over special trials. Use deterministic signals (odometry, status, estimates) first and vision for meaning or anomalies, not for every step.

Never put credentials, tokens or PINs in arguments or chat.`;

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
import { rpcSchema } from "./mcp-actions.ts";
import { z } from "zod/v4";
const MAX_JSON = 16 * 1024 * 1024;
const MAX_ERROR_JSON = 16 * 1024;
const MAX_JPEG = 5 * 1024 * 1024;
const READONLY = { readOnlyHint: true, destructiveHint: false, openWorldHint: false } as const;
const ACTUATION = { readOnlyHint: false, destructiveHint: true, idempotentHint: false, openWorldHint: false } as const;
const GET_PATHS = new Set(["/api/ros/state", "/api/ros/map", "/api/ros/tasks"]);
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
  error instanceof RoboClawHttpError ? { status: error.status, ...(error.evidence ?? {}), physicalCompletion: "unverified" }
    : { dispatch: "unknown", response: "transport_error", error: "Local service response unavailable; no automatic retry.", physicalCompletion: "unverified" }) }] });

export function createRoboClawMcp(client: RoboClawMcpClient): McpServer {
  const server = new McpServer({ name: "roboclaw-local", version: "0.1.0" }, { instructions: OPERATOR_INSTRUCTIONS });
  server.registerTool("get_state", {
    title: "Get ROS robot state", description: "Read shared ROS state, connection/battery, camera, arm estimate and typed chassis calibration. Preserves source evidence and stale flags; no extra robot RPC.",
    annotations: READONLY,
  }, async () => {
    try {
      return textResult(await client.request('/api/ros/state'));
    } catch (error) { return toolError(error); }
  });
  server.registerTool("refresh_status", {
    title: "Read shared status", description: "Read the driver-owned ROS observation cache; check evidence age and validity. Acquisition belongs to the ROS driver, not this tool.",
    annotations: READONLY,
  }, async () => {
    try { return textResult(await client.request("/api/ros/state")); } catch (error) { return toolError(error); }
  });
  server.registerTool("get_camera_diagnostics", {
    title: "Read camera status", description: "Read shared ROS camera/connection evidence. Required account/PIN input and recovery belong in web setup; this never opens a diagnostic session.",
    inputSchema: {}, annotations: READONLY,
  }, async () => {
    try { return textResult(await client.request("/api/ros/state")); } catch (error) { return toolError(error); }
  });
  server.registerTool("select_camera", {
    title: "Select shared camera", description: "Switch the shared active camera and leave it selected for all consumers. Confirms get_camera_status bit 15 even when app_change_camera never replies. Gripper needs pickup mode (app_pick_up_enter_exit {flag:2}) with the arm out; this tool does not enter pickup or move the arm.",
    inputSchema: { camera: z.enum(["front", "gripper"]) },
    annotations: { readOnlyHint: false, destructiveHint: false, idempotentHint: true, openWorldHint: false },
  }, async (payload) => {
    try { return textResult(await client.request("/api/ros/camera/select", 23000, payload)); } catch (error) { return toolError(error); }
  });
  server.registerTool("get_camera_frame", {
    title: "Get shared camera frame", description: "Read latest paired ROS JPEG/metadata (maximum age 5 seconds), never opens a preview or selects a lens. Optional camera filters the shared cache; use select_camera explicitly to switch. Retains acquisition identity; receipt time is not exposure or physical lens verification.",
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
          note: "Read imagePath with the local image reader. Receipt time is not exposure time; lens and arm pose remain unverified." });
      }
      return {
        content: [
          { type: "text", text: JSON.stringify({ ...metadata, note: "Receipt time is not camera exposure time; lens identity and arm pose are unverified." }) },
          { type: "image", data: frame.toString("base64"), mimeType: "image/jpeg" },
        ],
      };
    } catch (error) { return toolError(error); }
  });
  server.registerTool("get_arm_estimate", {
    title: "Get arm pose estimate",
    description: "Read ROS ArmEstimate: named joint radians, REP-103 wrist metres, explicit validity/source/uncertainty. Command-derived estimates are not sensed joints; UNKNOWN must not authorize planning.",
    inputSchema: {}, annotations: READONLY,
  }, async () => {
    try { const state = await client.request("/api/ros/state") as { arm: unknown }; return textResult(state.arm); } catch (error) { return toolError(error); }
  });
  server.registerTool("set_arm_anchor", {
    title: "Set arm anchor",
    description: "Re-anchor the arm estimate after confirming the pose by looking (camera or operator): stowed, deployed (vendor working pose right after arm-out), or unknown. Does not move the robot.",
    inputSchema: { anchor: z.enum(["stowed", "deployed", "unknown"]), reason: z.string().regex(/^[A-Za-z0-9 _.-]{0,64}$/).optional() },
    annotations: { readOnlyHint: false, destructiveHint: false, idempotentHint: true, openWorldHint: false },
  }, async (payload) => {
    try { return textResult(await client.request("/api/ros/arm/anchor", 5000, payload)); } catch (error) { return toolError(error); }
  });
  server.registerTool("robot_rpc", {
    title: "Advanced robot RPC", description: `Advanced vendor method/params through the SAME ROS scheduler and estimator as web/actions. No task bypass, no gateway retries. Explicit takeover cancels/stops the old task before dispatch. Outcomes preserve command/attempt IDs and UNKNOWN dispatch; replies do not establish physical completion. Device selection and credentials belong in web setup.\n\nOPERATING RULES:\n${OPERATOR_INSTRUCTIONS}`,
    inputSchema: rpcSchema, annotations: ACTUATION,
  }, async (payload) => {
    try {
      return textResult(await client.request('/api/ros/rpc', (payload.timeoutMs ?? 12000) + 18000, payload));
    } catch (error) { return toolError(error); }
  });
  const idSchema = z.string().uuid();
  const goalSchema = z.discriminatedUnion('kind', [
    z.object({kind: z.literal('pick'), goal: z.object({color: z.string().regex(/^[A-Za-z0-9_-]{1,32}$/), dry_run: z.boolean()}).strict()}).strict(),
    z.object({kind: z.literal('stack'), goal: z.object({base_color: z.string().regex(/^[A-Za-z0-9_-]{1,32}$/), middle_color: z.string().regex(/^[A-Za-z0-9_-]{1,32}$/), top_color: z.string().regex(/^[A-Za-z0-9_-]{1,32}$/), dry_run: z.boolean()}).strict()}).strict(),
    z.object({kind: z.literal('base_pulse'), goal: z.object({linear_mps: z.number().min(-.1).max(.1), angular_radps: z.number().min(-.5).max(.5), duration: z.object({sec: z.literal(0), nanosec: z.number().int().min(1).max(800000000)}).strict()}).strict()}).strict(),
    z.object({kind: z.literal('arm_step'), goal: z.object({joint: z.union([z.literal(3), z.literal(4), z.literal(5)]), direction: z.union([z.literal(-1), z.literal(1)])}).strict()}).strict(),
    z.object({kind: z.literal('hand_step'), goal: z.object({direction: z.number().int().min(1).max(4)}).strict()}).strict(),
    z.object({kind: z.literal('arm_operation'), goal: z.object({operation: z.number().int().min(1).max(5)}).strict()}).strict(),
    z.object({kind: z.literal('deploy_arm'), goal: z.object({settle: z.boolean()}).strict()}).strict(),
    z.object({kind: z.literal('align_target'), goal: z.object({color: z.string().regex(/^[A-Za-z0-9_-]{1,32}$/)}).strict()}).strict(),
    z.object({kind: z.literal('move_arm'), goal: z.object({m3_deg: z.number().min(90).max(270), m4_deg: z.number().min(35).max(170), m5_deg: z.number().min(0).max(155)}).strict()}).strict(),
    z.object({kind: z.literal('grasp_and_lift'), goal: z.object({lift_steps: z.number().int().min(0).max(10), verify_color: z.string().regex(/^[A-Za-z0-9_-]{0,32}$/)}).strict()}).strict(),
    z.object({kind: z.literal('release_and_stow'), goal: z.object({put_down: z.boolean()}).strict()}).strict()]);
  server.registerTool('start_task', {title: 'Start bounded ROS action', description: 'Start Pick or one bounded primitive; returns the ROS goal UUID immediately. Web and MCP share this ID, numeric phase, feedback and result. Pick dry_run=true is observation-only. No motion is replayed after an uncertain response; get_task/list_tasks reconcile it.', inputSchema: {task: goalSchema}, annotations: ACTUATION}, async ({task}) => {
    try {return textResult(await client.request('/api/ros/tasks', 5000, task));} catch (error) {return toolError(error);}
  });
  server.registerTool('list_tasks', {title:'List shared tasks', description:'Read shared ROS task handles including tasks started in web. UNKNOWN is not a completed or replayable task.', annotations:READONLY}, async () => {
    try {return textResult(await client.request('/api/ros/tasks'));} catch (error) {return toolError(error);}
  });
  server.registerTool('get_task', {title:'Get shared task', description:'Read the identical ROS goal ID/phase/result used by the web UI.', inputSchema:{id:idSchema}, annotations:READONLY}, async ({id}) => {
    try {return textResult(await client.request(`/api/ros/tasks/${id}`));} catch (error) {return toolError(error);}
  });
  server.registerTool('cancel_task', {title:'Cancel ROS task', description:'Request standard action cancellation and bounded stop through ROS. Poll get_task for terminal result; cancellation/stop ACK does not prove motor cessation.', inputSchema:{id:idSchema}, annotations:ACTUATION}, async ({id}) => {
    try {return textResult(await client.request(`/api/ros/tasks/${id}/cancel`, 5000, {}));} catch (error) {return toolError(error);}
  });
  server.registerTool('stop', {title:'Priority stop', description:'ROS priority stop independent of long tasks and camera waits. Scope BASE=1 ARM=2 ALL=3. No automatic stow/open. Cessation remains unverified without independent observation.', inputSchema:{scope:z.number().int().min(1).max(3).default(3)}, annotations:ACTUATION}, async (payload) => {
    try {return textResult(await client.request('/api/ros/stop', 18000, payload));} catch (error) {return toolError(error);}
  });
  server.registerTool('control_session', {title:'Stop or resume control and streams', description:'enabled=false stops motion, exits remote viewing and suspends shared acquisition until explicit enabled=true. Inspect streams_released and outcomes; cessation is not physically observed.', inputSchema:{enabled:z.boolean()}, annotations:ACTUATION}, async (payload) => {
    try {return textResult(await client.request('/api/ros/control/session', 40000, payload));} catch (error) {return toolError(error);}
  });
  server.registerTool('get_map', {title:'Read shared ROS map', description:'Read standard ROS OccupancyGrid. Cache receipt does not prove source freshness or current localisation.', annotations:READONLY}, async () => {
    try {return textResult(await client.request('/api/ros/map'));} catch (error) {return toolError(error);}
  });
  return server;
}
