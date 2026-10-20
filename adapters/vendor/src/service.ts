import { localProtocol, mapMetadataFromRead, selectedLocalProtocol, type PrivateMapMetadata, type SavedDevice } from "./secret-store.ts";
import { createServer, type IncomingMessage, type ServerResponse } from "node:http";
import { createHash, randomBytes, randomUUID, timingSafeEqual } from "node:crypto";
import { AsyncLocalStorage } from "node:async_hooks";
import { readFile } from "node:fs/promises";
import { extname, resolve, sep } from "node:path";
import { isIP } from "node:net";
import { fileURLToPath } from "node:url";
import { refreshRooms, RoborockCloud, selectedRooms, type CameraAccount, type CameraLogin, type HomeAuthorization } from "./cloud.ts";
import { CommandEvents, numericTelemetry, publicRpcValue, pulseParams, RobotClient, RpcError, summarizeStatus, TransportError, type RobotDiagnostic, type RpcOutcome } from "./robot.ts";
import { CloudControlClient, cloudControlCredentials, readSplitMap } from "./cloud-control.ts";
import { CameraSelector, Go2Rtc, localServiceOrigin, publicCameraFailure, requestedCamera, ServiceHttpError } from "./go2rtc.ts";
import { jpegImageDimensions, rosRoute } from "@rrsz70/http-contracts";

export { localServiceOrigin, rosRoute, ServiceHttpError };

const COOKIE = "roboclaw_session";
const SESSION_MS = 12 * 60 * 60_000;
function digest(value: string): Buffer { return createHash("sha256").update(value).digest(); }

export class ServiceAuth {
  #token: Buffer;
  #config: ServiceConfig;
  #sessions = new Map<string, { expires: number; scope: "control" | "setup" }>();
  #attempts = 0;
  #window = 0;

  constructor(token: string, config: ServiceConfig) {
    if (!token || token.length < 32 || token.length > 4096) throw new Error("Invalid persisted service token.");
    this.#token = digest(token);
    this.#config = config;
  }

  checkHost(req: IncomingMessage): void {
    const hosts = req.rawHeaders.filter((_, index) => index % 2 === 0)
      .filter((name) => name.toLowerCase() === "host");
    if (hosts.length !== 1 || !req.headers.host ||
      !this.#config.allowedHosts.includes(req.headers.host.toLowerCase())) {
      throw new ServiceHttpError(403, "Untrusted request host.");
    }
    if (req.url?.startsWith("//") || !req.url?.startsWith("/")) {
      throw new ServiceHttpError(400, "Invalid request target.");
    }
  }

  checkOrigin(req: IncomingMessage): void {
    if (req.headers.origin !== this.#config.publicOrigin ||
      req.headers["sec-fetch-site"] === "cross-site") {
      throw new ServiceHttpError(403, "A same-origin request is required.");
    }
  }

  #cookie(req: IncomingMessage): string | undefined {
    const values = (req.headers.cookie ?? "").split(";").map((value) => value.trim())
      .filter((value) => value.startsWith(`${COOKIE}=`));
    if (values.length !== 1) return undefined;
    const value = values[0].slice(COOKIE.length + 1);
    return /^[a-f0-9]{64}$/.test(value) ? digest(value).toString("hex") : undefined;
  }

  authenticate(req: IncomingMessage): { kind: "bearer" | "cookie"; id: string; scope: "control" | "setup" } | null {
    if (req.headers.authorization !== undefined) {
      const value = /^Bearer ([^\s]{1,4096})$/.exec(req.headers.authorization)?.[1];
      return value && timingSafeEqual(digest(value), this.#token) ? { kind: "bearer", id: "bearer", scope: "control" } : null;
    }
    const id = this.#cookie(req);
    if (!id) return null;
    const session = this.#sessions.get(id);
    if (!session || session.expires <= Date.now()) {
      this.#sessions.delete(id);
      return null;
    }
    return { kind: "cookie", id, scope: session.scope };
  }

  require(req: IncomingMessage): NonNullable<ReturnType<ServiceAuth["authenticate"]>> {
    const auth = this.authenticate(req);
    if (!auth) throw new ServiceHttpError(401, "Authentication required.");
    if (auth.kind === "cookie" && !["GET", "HEAD"].includes(req.method ?? "")) this.checkOrigin(req);
    return auth;
  }

  login(req: IncomingMessage, res: ServerResponse, pin: unknown, expectedPin: string | null | undefined): void {
    this.checkOrigin(req);
    const now = Date.now();
    if (now - this.#window >= 60_000) { this.#window = now; this.#attempts = 0; }
    if (++this.#attempts > 10) throw new ServiceHttpError(429, "Too many authentication attempts.");
    if (!expectedPin || !/^[1-9]{4,12}$/.test(expectedPin)) {
      throw new ServiceHttpError(503, "Camera PIN is not configured.");
    }
    if (typeof pin !== "string" || !/^[1-9]{4,12}$/.test(pin) ||
      !timingSafeEqual(digest(pin), digest(expectedPin))) {
      throw new ServiceHttpError(401, "Invalid camera PIN.");
    }
    this.#issue(req, res, "control", SESSION_MS);
  }

  setup(req: IncomingMessage, res: ServerResponse): void {
    this.checkOrigin(req);
    const hostname = new URL(`http://${req.headers.host}`).hostname;
    if (!["localhost", "127.0.0.1", "[::1]"].includes(hostname) ||
      !["localhost", "127.0.0.1", "[::1]"].includes(new URL(this.#config.publicOrigin).hostname)) {
      throw new ServiceHttpError(403, "Initial setup requires localhost.");
    }
    this.#issue(req, res, "setup", 20 * 60_000);
  }

  revokeSetup(): void {
    for (const [id, session] of this.#sessions) if (session.scope === "setup") this.#sessions.delete(id);
  }

  #issue(req: IncomingMessage, res: ServerResponse, scope: "control" | "setup", duration: number): void {
    const now = Date.now();
    for (const [id, session] of this.#sessions) if (session.expires <= now) this.#sessions.delete(id);
    const old = this.#cookie(req);
    if (old) this.#sessions.delete(old);
    if (this.#sessions.size >= 64) throw new ServiceHttpError(429, "Too many active sessions.");
    const value = randomBytes(32).toString("hex");
    this.#sessions.set(digest(value).toString("hex"), { expires: now + duration, scope });
    res.setHeader("Set-Cookie", this.#header(value, duration / 1000));
  }

  logout(req: IncomingMessage, res: ServerResponse): void {
    this.checkOrigin(req);
    const id = this.#cookie(req);
    if (id) this.#sessions.delete(id);
    res.setHeader("Set-Cookie", this.#header("", 0));
  }

  clear(): void { this.#sessions.clear(); }

  #header(value: string, maxAge: number): string {
    return `${COOKIE}=${value}; Path=/; HttpOnly; SameSite=Strict; Max-Age=${maxAge}${this.#config.secureCookie ? "; Secure" : ""}`;
  }
}

const ROOT = fileURLToPath(new URL("../../../", import.meta.url));

export interface ServiceConfig {
  host: string;
  port: number;
  publicOrigin: string;
  allowedHosts: string[];
  secureCookie: boolean;
  stateDir: string;
  webRoot: string;
  go2rtcOrigin: string;
}

function flag(value: string | undefined): boolean {
  if (value === undefined || value === "false" || value === "0") return false;
  if (value === "true" || value === "1") return true;
  throw new Error("Boolean service configuration must be true or false.");
}

function origin(value: string): URL {
  let url: URL;
  try { url = new URL(value); }
  catch { throw new Error("Invalid service origin configuration."); }
  if (!["http:", "https:"].includes(url.protocol) || url.username || url.password ||
    url.pathname !== "/" || url.search || url.hash || url.origin !== value) {
    throw new Error("Service origins must be canonical HTTP origins without paths or credentials.");
  }
  return url;
}

export function localOrigin(value: string): string {
  const url = origin(value);
  localServiceOrigin(value, "go2rtc");
  const port = Number(url.port || (url.protocol === "https:" ? 443 : 80));
  if (port < 57000 || port > 57100) {
    throw new Error("The camera service port must be between 57000 and 57100.");
  }
  return url.origin;
}

export function loadServiceConfig(env: NodeJS.ProcessEnv = process.env): ServiceConfig {
  const host = env.ROBOCLAW_HOST ?? "127.0.0.1";
  if (!isIP(host) && host !== "localhost") throw new Error("ROBOCLAW_HOST must be a literal bind address or localhost.");
  const portText = env.ROBOCLAW_PORT ?? "57000";
  const port = Number(portText);
  if (!/^\d{5}$/.test(portText) || !Number.isInteger(port) || port < 57000 || port > 57100) {
    throw new Error("ROBOCLAW_PORT must be between 57000 and 57100.");
  }
  const publicOrigin = env.ROBOCLAW_PUBLIC_ORIGIN ?? `http://127.0.0.1:${port}`;
  const publicUrl = origin(publicOrigin);
  const secureCookie = flag(env.ROBOCLAW_SECURE_COOKIE);
  if ((publicUrl.protocol === "https:") !== secureCookie) {
    throw new Error("HTTPS public origins require ROBOCLAW_SECURE_COOKIE=true; HTTP origins require false.");
  }
  const allowedHosts = [publicUrl.host];
  for (const item of (env.ROBOCLAW_ALLOWED_HOSTS ?? "").split(",").filter(Boolean)) {
    const authority = item.trim().toLowerCase();
    const candidate = origin(`http://${authority}`);
    if (candidate.host !== authority || authority.includes("*")) throw new Error("Invalid exact Host allowlist.");
    allowedHosts.push(authority);
  }
  return {
    host, port, publicOrigin, allowedHosts: [...new Set(allowedHosts)], secureCookie,
    stateDir: resolve(env.ROBOCLAW_STATE_DIR ?? resolve(ROOT, ".local")),
    webRoot: resolve(env.ROBOCLAW_WEB_ROOT ?? resolve(ROOT, "dist/web")),
    go2rtcOrigin: localOrigin(env.GO2RTC_ORIGIN ?? "http://127.0.0.1:57001"),
  };
}

export function rosGatewayOrigin(value: string): URL {
  const url = new URL(value);
  if (url.protocol !== "http:" || !["127.0.0.1", "[::1]", "localhost"].includes(url.hostname) || !url.port ||
      url.username || url.password || url.pathname !== "/" || url.search || url.hash)
    throw new Error("Invalid ROS gateway origin.");
  return url;
}

/** One authenticated request, never retried or replaced with a vendor command. */
export async function proxyRos(res: ServerResponse, origin: URL, token: string, method: string, path: string,
  body?: Record<string, unknown>, transport: typeof fetch = fetch): Promise<void> {
  const action = method === "POST" && path !== "/api/ros/frame";
  try {
    const response = await transport(new URL(path, origin), {
      method, headers: { Authorization: ["Bearer", token].join(" "), ...(body ? { "Content-Type": "application/json" } : {}) },
      ...(body ? { body: JSON.stringify(body) } : {}), redirect: "error", signal: AbortSignal.timeout(48_000),
    });
    const jpeg = response.ok && path === "/api/ros/frame";
    const contentType = response.headers.get("content-type")?.split(";")[0];
    if (contentType !== (jpeg ? "image/jpeg" : "application/json")) throw new Error("Unexpected gateway response");
    const limit = jpeg ? 5 * 1024 * 1024 : 16 * 1024 * 1024;
    const parts: Uint8Array[] = []; let size = 0;
    if (!response.body) throw new Error("Empty gateway response");
    for await (const chunk of response.body) {
      size += chunk.byteLength;
      if (size > limit) { await response.body.cancel().catch(() => {}); throw new Error("Gateway response limit"); }
      parts.push(chunk);
    }
    const bytes = Buffer.concat(parts, size);
    if (!jpeg) JSON.parse(bytes.toString());
    const headers: Record<string, string> = { "Content-Type": contentType!, "Content-Length": String(bytes.length), "Cache-Control": "no-store" };
    for (const key of ["x-roboclaw-camera", "x-roboclaw-frame-id", "x-roboclaw-camera-session", "x-roboclaw-received-at"]) {
      const value = response.headers.get(key);
      if (value && /^[A-Za-z0-9_.-]{1,128}$/.test(value)) headers[key] = value;
    }
    res.writeHead(response.status, headers); res.end(bytes);
  } catch {
    res.writeHead(503, { "Content-Type": "application/json" });
    res.end(JSON.stringify({ error: "ROS_UNAVAILABLE", dispatch: action ? "unknown" : "not_sent" }));
  }
}

/** Connection-level events only; messages are fixed internal labels (no payloads, keys or addresses). */
function logRobotDiagnostic(e: RobotDiagnostic): void {
  if (e.kind === "disconnected" || e.kind === "ignored-frame") console.error(`robot link ${e.kind}: ${e.message ?? ""}`);
}

export interface ServiceStore {
  initialize(): Promise<void>;
  loadPairing(): Promise<{ device: SavedDevice; camera: { account: CameraAccount; pin: string }; home: HomeAuthorization } | null>;
  savePairing(pairing: { device: SavedDevice; camera: { account: CameraAccount; pin: string }; home: HomeAuthorization }): Promise<void>;
  serviceToken(): Promise<string>;
  loadMapRasterSource?(): Promise<{ metadata: PrivateMapMetadata; raw: Buffer } | null>;
  saveMapCache?(metadata: PrivateMapMetadata): Promise<void>;
  saveMapBytes?(raw: Buffer): Promise<void>;
}
export interface ServiceRobot {
  readonly connection: { state: string; protocol?: string; connectedAt: number | null };
  request(method: string, params?: unknown): Promise<unknown>;
  rpc?(method: string, params?: unknown, timeoutMs?: number): Promise<unknown>;
  close(): void;
}
export interface ServiceOptions {
  store: ServiceStore;
  config?: ServiceConfig;

  robotFactory?: (device: SavedDevice, diagnostic: (event: RobotDiagnostic) => void) => ServiceRobot;
  cloudRobotFactory?: (account: CameraAccount, device: SavedDevice) => ServiceRobot;
  driver?: Pick<Go2Rtc, "configure" | "offer" | "frame"> & Partial<Pick<Go2Rtc, "preload" | "unload" | "preloaded" | "configurationMatches" | "streamDiagnostics" | "acquisitionDiagnostics" | "close" | "setEnabled">>;
  cloudFactory?: (region: "eu" | "us" | "cn" | "asia") => Pick<RoborockCloud, "requestCode" | "loginCameraWithCode">;
  mapReader?: typeof readSplitMap;
  roomRefresher?: typeof refreshRooms;
  rosOrigin?: string;
  rosTransport?: typeof fetch;
}
const MAX_JSON = 192 * 1024;
const MIME: Record<string, string> = { ".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8", ".css": "text/css; charset=utf-8", ".svg": "image/svg+xml", ".png": "image/png", ".jpg": "image/jpeg", ".ico": "image/x-icon", ".woff": "font/woff", ".woff2": "font/woff2" };
function record(value: unknown): value is Record<string, unknown> { return value !== null && typeof value === "object" && !Array.isArray(value); }
function fail(status: number, message: string): never { throw new ServiceHttpError(status, message); }
function reply(res: ServerResponse, status: number, value: unknown): void {
  if (res.destroyed || res.writableEnded) return;
  res.writeHead(status, { "Content-Type": "application/json; charset=utf-8" }); res.end(JSON.stringify(value));
}
function pinValue(value: unknown): string {
  if (typeof value !== "string" || !/^[1-9]{4,12}$/.test(value)) return fail(400, "Enter the existing camera pattern as 4-12 digits from 1-9.");
  return value;
}

async function readJson(req: IncomingMessage, limit = MAX_JSON): Promise<Record<string, unknown>> {
  if (!/^application\/json(?:\s*;\s*charset=utf-8)?$/i.test(req.headers["content-type"] ?? "")) {
    return fail(415, "Content-Type must be application/json.");
  }
  if (req.headers["content-encoding"] && req.headers["content-encoding"] !== "identity") {
    return fail(415, "Compressed request bodies are not accepted.");
  }
  const length = req.headers["content-length"];
  if (length && (!/^\d+$/.test(length) || Number(length) > limit)) return fail(413, "Request body is too large.");
  const chunks: Buffer[] = [];
  let size = 0;
  // Readable iteration destroys the socket on early return; data events allow a bounded error response.
  await new Promise<void>((resolve, reject) => {
    const timer = setTimeout(() => finish(new ServiceHttpError(408, "Request body timed out.")), 10_000);
    const onData = (chunk: Buffer) => {
      size += chunk.length;
      if (size > limit) { finish(new ServiceHttpError(413, "Request body is too large.")); return; }
      chunks.push(chunk);
    };
    const onEnd = () => finish();
    const onError = () => finish(new ServiceHttpError(400, "Request body was interrupted."));
    function finish(error?: Error) {
      clearTimeout(timer);
      req.off("data", onData); req.off("end", onEnd); req.off("error", onError); req.off("aborted", onError);
      if (error) { req.pause(); reject(error); } else resolve();
    }
    req.on("data", onData); req.once("end", onEnd); req.once("error", onError); req.once("aborted", onError);
  });
  let value: unknown;
  try { value = JSON.parse(Buffer.concat(chunks).toString("utf8")); }
  catch { return fail(400, "Invalid JSON request body."); }
  if (!record(value)) return fail(400, "A JSON object is required.");
  return value;
}


async function staticFile(req: IncomingMessage, res: ServerResponse, webRoot: string, path: string): Promise<void> {
  let decoded: string;
  try { decoded = decodeURIComponent(path); }
  catch { return fail(400, "Invalid path."); }
  if (decoded.includes("\\") || decoded.includes("\0") || decoded.split("/").some((part) => part.startsWith("."))) {
    return fail(404, "Not found.");
  }
  const root = resolve(webRoot);
  let relative = decoded.slice(1) || "index.html";
  let target = resolve(root, relative);
  if (target !== root && !target.startsWith(root + sep)) return fail(404, "Not found.");
  let body = await readFile(target).catch(() => null);
  if (!body) {
    if (extname(relative) || !req.headers.accept?.includes("text/html")) return fail(404, "Not found.");
    relative = "index.html";
    target = resolve(root, relative);
    body = await readFile(target).catch(() => null);
    if (!body) return fail(503, "The frontend build is unavailable.");
  }
  const mime = MIME[extname(relative)];
  if (!mime) return fail(404, "Not found.");
  res.writeHead(200, { "Content-Type": mime, "Content-Length": body.length });
  res.end(req.method === "HEAD" ? undefined : body);
}

/** Owns the robot connection and timers, but never spawns or stops the independent sidecar. */

/** Single authenticated transport boundary. No history, leases or physical-state admission. */
export async function createService(options: ServiceOptions) {
  const config = options.config ?? loadServiceConfig();
  const store = options.store;
  await store.initialize();
  const serviceToken = await store.serviceToken();
  const auth = new ServiceAuth(serviceToken, config);
  const rosOrigin = rosGatewayOrigin(options.rosOrigin ?? process.env.Z70_ROS_GATEWAY_ORIGIN ?? "http://127.0.0.1:57020");
  const savedPairing = await store.loadPairing();
  let device = savedPairing?.device ?? null;
  let camera = savedPairing?.camera ?? null;
  let pinConfigured = !!camera?.pin && /^[1-9]{4,12}$/.test(camera.pin);
  let home = savedPairing?.home ?? null;
  let robot: ServiceRobot | null = null;
  let cloudRobot: ServiceRobot | null = null;
  let stopped = false;
  let configured = false;
  let cameraEnabled = true, cameraGeneration = 0;
  const cameraContext = new AsyncLocalStorage<number>();
  function checkCamera(generation = cameraGeneration) {
    if (!cameraEnabled || generation !== cameraGeneration) return fail(409, "SESSION_STOPPED");
  }
  const driver = options.driver ?? new Go2Rtc(config.go2rtcOrigin);
  const jobs = new Set<Promise<void>>();
  const cameraSession = randomUUID();
  const commandEvents = new CommandEvents();
  function replaceRobot() {
    robot?.close(); cloudRobot?.close(); cloudRobot = null; configured = false;
    robot = device ? (options.robotFactory?.(device, () => {}) ?? new RobotClient({ host: device.host, localKey: device.localKey, protocol: selectedLocalProtocol(device), onDiagnostic: logRobotDiagnostic })) : null;
  }
  replaceRobot();
  function secrets(): string[] { return [serviceToken, device?.localKey, camera?.pin, camera?.account.secret, camera?.account.key, home?.hawkKey, home?.secret].filter((value): value is string => !!value); }

  async function dispatch(input: Record<string, unknown>): Promise<RpcOutcome> {
    if (Object.keys(input).some(key => !["method", "params", "transport", "timeoutMs", "commandId"].includes(key)) ||
      typeof input.method !== "string" || !/^[A-Za-z][A-Za-z0-9_]{0,127}$/.test(input.method) ||
      (input.commandId !== undefined && (typeof input.commandId !== 'string' || !/^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$/i.test(input.commandId))) ||
      (input.transport !== undefined && input.transport !== "local" && input.transport !== "cloud") ||
      (input.timeoutMs !== undefined && (typeof input.timeoutMs !== "number" || !Number.isInteger(input.timeoutMs) || input.timeoutMs < 1 || input.timeoutMs > 120000)))
      return fail(400, "Invalid vendor RPC envelope.");
    const requestId = typeof input.commandId === 'string' ? input.commandId : randomUUID(), startedAt = Date.now();
    let attemptIndex = -1, attemptId = requestId, attemptedAt: number | null = null;
    const event = (phase: 'accepted' | 'dispatched' | 'replied' | 'finished', dispatch: 'not_sent' | 'sent' | 'unknown', response: 'none' | 'received' | 'vendor_error' | 'transport_error', errorCode = 'NONE', vendorCode?: number) => {
      commandEvents.emit({commandId: requestId, attemptId, attemptIndex: Math.max(0, attemptIndex), phase,
        operation: input.method as string, requestedAt: startedAt, attemptedAt,
        finishedAt: phase === 'replied' || phase === 'finished' ? Date.now() : null,
        dispatch, response, errorCode, ...(vendorCode !== undefined ? {vendorCode} : {}),
        ...(pulseParams(input.method as string, input.params) ? {params: pulseParams(input.method as string, input.params)} : {})});
    };
    const emitError = (phase: 'replied' | 'finished', error: unknown) => {
      const vendor = error instanceof RpcError;
      event(phase, vendor ? 'sent' : error instanceof TransportError ? error.dispatch : 'unknown',
        vendor ? 'vendor_error' : 'transport_error',
        vendor ? 'VENDOR_REJECTED' : error instanceof TransportError && error.reason === 'cancelled' ? 'CANCELLED' : error instanceof TransportError && /timeout|timed out/i.test(error.message) ? 'TIMEOUT' : 'TRANSPORT',
        vendor ? error.code : undefined);
    };
    event('accepted', 'not_sent', 'none');
    const requestSecrets = secrets();
    const base = { requestId, startedAt, deviceId: device?.duid ?? null, transport: input.transport ?? "local" };
    let target: ServiceRobot | null = robot;
    try {
      if (!device) throw new TransportError("No selected device", "not_sent");
      if (input.transport === "cloud") {
        if (!camera) throw new TransportError("No authenticated cloud account", "not_sent");
        try { cloudRobot ??= options.cloudRobotFactory?.(camera.account, device) ?? new CloudControlClient(cloudControlCredentials(camera.account, device)); }
        catch { throw new TransportError("Invalid authenticated cloud credentials", "not_sent"); }
        target = cloudRobot;
      }
      if (!target) throw new TransportError("No device transport", "not_sent");
      const params = Object.hasOwn(input, "params") ? input.params : [];
      const deadline = typeof input.timeoutMs === 'number' ? performance.now() + input.timeoutMs : null;
      const send = async (t: ServiceRobot) => {
        const remaining = deadline === null ? undefined : Math.max(0, Math.floor(deadline-performance.now()));
        if (remaining === 0) throw new TransportError('RPC total reply budget timed out', 'not_sent');
        const id=randomUUID(), sentAt=Date.now();
        attemptId = id; attemptIndex++; attemptedAt = sentAt;
        event('dispatched', 'unknown', 'none');
        try {
          const value=await (t.rpc ? t.rpc(input.method as string,params,remaining) : t.request(input.method as string,params));
          event('replied', 'sent', 'received');
          return value;
        } catch (error) {
          emitError('replied', error);
          throw error;
        }
      };
      let result: unknown, retried = 0;
      try { result = await send(target); }
      catch (first) {
        // One automatic retry when repeating cannot cause extra motion: reads/stop/end/start, or a command that never left us.
        const repeatable = input.method !== "app_goto_target" && (/^(get_|stop_goto_target$|app_rc_(stop|end|start)$)/.test(input.method) || (first instanceof TransportError && first.dispatch === "not_sent"));
        if (first instanceof RpcError || (first instanceof TransportError && first.reason === 'cancelled') || !repeatable || (deadline !== null && performance.now() >= deadline)) throw first;
        retried = 1; result = await send(target);
      }
      event('finished', 'sent', 'received');
      return { ...base, commandId: requestId, attemptId, attemptIndex: Math.max(0, attemptIndex), finishedAt: Date.now(), dispatch: "sent", response: "received", ...(retried ? { retried } : {}), result: publicRpcValue(result, requestSecrets) };
    } catch (error) {
      const vendor = error instanceof RpcError;
      emitError('finished', error);
      return { ...base, commandId: requestId, attemptId, attemptIndex: Math.max(0, attemptIndex), finishedAt: Date.now(), dispatch: vendor ? "sent" : error instanceof TransportError ? error.dispatch : "unknown",
        response: vendor ? "vendor_error" : "transport_error",
        ...(!vendor ? { transportReason: error instanceof TransportError ? (error.reason === 'cancelled' ? 'cancelled' as const : /timed out|timeout/i.test(error.message) ? "timeout" as const : /capacity|queue is full|buffer limit/i.test(error.message) ? "capacity" as const : /Invalid|serializable/i.test(error.message) ? "invalid-input" as const : /connection|selected|account|transport/i.test(error.message) ? "unavailable" as const : "transport" as const) : "transport" as const } : {}), error: vendor ? "Robot rejected RPC" : "Device transport failed", ...(vendor && error.code !== undefined ? { vendorCode: error.code } : {}) };
    }
  }
  async function telemetry() {
    async function read(method: string, params: unknown) {
      const response = await dispatch({ method, params });
      return { value: response.response === "received" ? (method === "get_status" ? summarizeStatus(response.result) : numericTelemetry(response.result)) : null,
        observedAt: response.response === "received" ? response.finishedAt : null, requestedAt: response.startedAt, error: response.error ?? null };
    }
    const [status, camera] = await Promise.all([read("get_status", []), read("get_camera_status", [])]);
    return { status, camera };
  }
  const selectorOutcomes = new AsyncLocalStorage<{ method: string; outcome: RpcOutcome }[]>();
  const cameraSelector = new CameraSelector(async (method, params, timeoutMs) => {
    checkCamera(cameraContext.getStore());
    const outcome = await dispatch({ method, params, timeoutMs });
    selectorOutcomes.getStore()?.push({ method, outcome });
    if (outcome.response === "vendor_error") throw new RpcError(typeof outcome.vendorCode === "number" ? outcome.vendorCode : -1);
    if (outcome.response !== "received") throw new TransportError(outcome.transportReason === "timeout" ? "Camera RPC timed out" : "Camera selection/status RPC failed", outcome.dispatch);
    return outcome.result;
  });
  async function configureCamera(generation = cameraGeneration) {
    checkCamera(generation);
    if (!device || !camera) return fail(503, "Authenticated device and camera credentials required.");
    if (!configured) { await driver.configure(camera.account, device, camera.pin); checkCamera(generation); configured = true; }
  }
  const captureFrame = (generation: number) => async (active: "front" | "gripper") => {
    await configureCamera(generation);
    checkCamera(generation);
    return driver.frame(active);
  };
  let account: { owner: string; email: string; expiresAt: number; cloud: Pick<RoborockCloud, "requestCode" | "loginCameraWithCode">; login?: CameraLogin; attempts: number } | null = null;
  let lastCode = 0;
  let pairingWrite = Promise.resolve();
  function persistPairing(operation: () => Promise<void>): Promise<void> {
    const update = pairingWrite.then(operation);
    pairingWrite = update.catch(() => {});
    return update;
  }
  async function handle(req: IncomingMessage, res: ServerResponse) {
    auth.checkHost(req);
    const path = req.url!.split("?")[0];
    if (req.url !== path) return fail(400, "Query parameters are not accepted.");
    if (path === "/healthz" && req.method === "GET") return reply(res, stopped ? 503 : 200, { available: !stopped });
    if (path === "/api/session") {
      const sessionStatus = () => {
        const session = auth.authenticate(req);
        return { authenticated: session?.scope === "control", scope: session?.scope ?? null, setupAvailable: !pinConfigured };
      };
      if (req.method === "GET") return reply(res, 200, sessionStatus());
      if (req.method === "POST") {
        const body = await readJson(req);
        if (body.setup === true) {
          auth.checkOrigin(req);
          if (pinConfigured) return fail(409, "Initial setup is complete. Sign in with the camera PIN.");
          auth.setup(req, res);
          return reply(res, 200, { authenticated: false, scope: "setup", setupAvailable: true });
        }
        auth.login(req, res, body.pin, camera?.pin);
        return reply(res, 200, { authenticated: true, scope: "control", setupAvailable: false });
      }
      if (req.method === "DELETE") { auth.logout(req, res); return reply(res, 200, { authenticated: false }); }
      return fail(405, "Method not allowed.");
    }
    if (!path.startsWith("/api/")) {
      if (!["GET", "HEAD"].includes(req.method ?? "")) return fail(405, "Method not allowed.");
      return staticFile(req, res, config.webRoot, path);
    }
    const caller = auth.require(req);
    if (stopped) return fail(503, "Service is stopping.");
    // Explicit allowlist: setup authority never reaches ROS, telemetry or private transports.
    if (caller.scope === "setup" && !(req.method === "GET" && path === "/api/setup") &&
      !(req.method === "POST" && ["/api/account/code", "/api/account/login", "/api/account/select", "/api/camera/pin"].includes(path)))
      return fail(403, "Initial setup session cannot access control or observation APIs.");
    if (path.startsWith('/api/ros/')) {
      if (!rosRoute(req.method ?? '', path)) return fail(404, 'Unsupported ROS route.');
      const body = req.method === 'POST' ? await readJson(req) : undefined;
      return proxyRos(res, rosOrigin, serviceToken, req.method!, path, body, options.rosTransport);
    }
    if (path === '/api/setup' && req.method === 'GET')
      return reply(res, 200, { device: device ? { id: device.duid, name: device.name, model: device.model, host: device.host, localProtocol: selectedLocalProtocol(device) } : null,
        accountConfigured: !!camera, pinConfigured, regions: ['eu', 'us', 'cn', 'asia'], protocols: ['1.0', 'L01'] });
    // Acquisition transport endpoints are private to the ROS driver. Browser consumers
    // use the shared ROS cache and cannot open additional previews or workflows.
    if (caller.kind === 'cookie' && !path.startsWith('/api/account/') && path !== '/api/camera/pin')
      return fail(410, 'Use the shared /api/ros interfaces; account and camera input remain in web setup.');
    const requireBearer = () => {
      if (!req.headers.authorization?.startsWith("Bearer ")) fail(403, "Private driver bearer authentication required.");
    };
    if (path === '/api/driver/control/session' && req.method === 'POST') {
      requireBearer();
      const body = await readJson(req);
      if (Object.keys(body).length !== 1 || typeof body.enabled !== 'boolean') return fail(400, 'Boolean enabled required.');
      // Fence queued captures and in-flight configuration before awaiting cleanup.
      cameraEnabled = body.enabled; cameraGeneration++; configured = false;
      if (driver.setEnabled) await driver.setEnabled(body.enabled);
      else if (!body.enabled) {
        driver.close?.();
        if (!driver.unload) return fail(503, 'Preview release unsupported.');
        await driver.unload();
      }
      return reply(res, 200, {enabled: cameraEnabled, streams_released: !cameraEnabled});
    }
    if (path === "/api/driver/rpc") {
      requireBearer();
      if (req.method !== "POST") return fail(405, "Method not allowed.");
      const outcome = await dispatch(await readJson(req));
      return reply(res, outcome.response === "received" ? 200 : 502, outcome);
    }
    if (path === '/api/commands/events') {
      if (req.method !== 'GET') return fail(405, 'Method not allowed.');
      commandEvents.subscribe(res); return;
    }
    if (path === "/api/state") {
      if (req.method !== "GET") return fail(405, "Method not allowed.");
      return reply(res, 200, { observationSession: cameraSession, device: device ? { id: device.duid, name: device.name, model: device.model, cloudProtocolVersion: device.protocolVersion, localProtocol: selectedLocalProtocol(device) } : null,
        connection: robot?.connection ?? { state: "disconnected", connectedAt: null }, telemetry: await telemetry(),
        api: { rpc: null, commandOwner: "ROS", rosRawRpc: "/z70/driver/raw_rpc", genericVendorRpc: true }, camera: { configured, pinCached: !!camera?.pin, feedLensVerified: false,
          acquisition: await driver.acquisitionDiagnostics?.() ?? { available: false },
          streamDiagnostics: await driver.streamDiagnostics?.() ?? null,
          sourceAudit: device && camera && driver.configurationMatches
            ? await driver.configurationMatches(camera.account, device, camera.pin) : { available: false } } });
    }
    if (path === "/api/home/rooms") return reply(res, 200, home && home.deviceId === device?.duid ? selectedRooms(home) : { source: "unavailable", rooms: [] });
    if (path === "/api/home/rooms/refresh" && req.method === "POST") {
      await readJson(req); if (!device || !camera || !home || home.deviceId !== device.duid) return fail(503, "Account authorization unavailable.");
      const selected = device, authorization = home, selectedCamera = camera;
      const updated = { ...authorization, rooms: await (options.roomRefresher ?? refreshRooms)(authorization) };
      await persistPairing(async () => {
        if (device !== selected || home !== authorization || camera !== selectedCamera) return fail(409, "Selected account changed.");
        await store.savePairing({ device: selected, camera: selectedCamera, home: updated });
        home = updated;
      });
      return reply(res, 200, selectedRooms(updated));
    }
    // Authenticated geometry export for local ROS observers; never public or fresh-pose telemetry.
    if (path === "/api/map/cache/raw" && req.method === "GET") {
      const source = await store.loadMapRasterSource?.();
      if (!source || source.metadata.deviceId !== device?.duid) return fail(404, "No selected-device map.");
      res.writeHead(200, { "Content-Type": "application/octet-stream", "Content-Length": source.raw.length,
        "X-Roboclaw-Observation-Session": cameraSession,
        "Cache-Control": "no-store", "X-Map-Saved-At": String(source.metadata.savedAt),
        "X-Map-Index": String(source.metadata.mapIndex), "X-Map-Sequence": String(source.metadata.mapSequence) });
      res.end(source.raw); return;
    }
    if (path === "/api/map/cache/refresh" && req.method === "POST") {
      await readJson(req); if (!device || !camera) return fail(503, "Authenticated device/account required.");
      const selected = device;
      const read = await (options.mapReader ?? readSplitMap)(selected, camera.account);
      const metadata = mapMetadataFromRead(selected.duid, read);
      await store.saveMapBytes?.(read.raw); await store.saveMapCache?.(metadata);
      return reply(res, 200, { cached: true, savedAt: metadata.savedAt, mapIndex: metadata.mapIndex, mapSequence: metadata.mapSequence, bytes: metadata.bytes });
    }
    if (path.startsWith("/api/account/") && req.method === "POST") {
      const body = await readJson(req);
      auth.require(req);
      if (path === "/api/account/code") {
        if (typeof body.email !== "string" || body.email.length > 254 || !/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(body.email) || !["eu", "us", "cn", "asia"].includes(String(body.region))) return fail(400, "Invalid email or region.");
        if (Date.now() - lastCode < 60000) return fail(429, "Wait before requesting another code.");
        lastCode = Date.now();
        const cloud = options.cloudFactory?.(body.region as "eu") ?? new RoborockCloud(body.region as "eu");
        try { await cloud.requestCode(body.email); }
        catch { return fail(502, 'Sign-in code request failed. Check account email, region and network; wait before requesting another code.'); }
        auth.require(req);
        account = { owner: caller.id, email: body.email, cloud, expiresAt: Date.now() + 1200000, attempts: 0 };
        return reply(res, 200, { sent: true });
      }
      if (!account || account.owner !== caller.id || account.expiresAt < Date.now()) return fail(410, "Account login expired. Request a new code in web setup.");
      if (path === "/api/account/login") {
        if (typeof body.code !== "string" || !/^\d{6}$/.test(body.code)) return fail(400, "Invalid verification code.");
        if (++account.attempts > 5) return fail(429, "Too many login attempts.");
        const attempt = account;
        let login: CameraLogin;
        try { login = await attempt.cloud.loginCameraWithCode(attempt.email, body.code); }
        catch { return fail(502, 'Account verification failed. Check region, verification code, network and computer clock; request a new code if it expired.'); }
        auth.require(req);
        if (account !== attempt || attempt.expiresAt < Date.now() || attempt.login) return fail(409, "Account login changed.");
        attempt.login = login; attempt.email = "";
        return reply(res, 200, { devices: login.devices.map(item => ({ id: item.duid, name: item.name, model: item.model })) });
      }
      if (path === "/api/account/select") {
        const attempt = account, login = attempt.login, selected = login?.devices.find(item => item.duid === body.id);
        if (!login || !selected || typeof body.host !== "string" || !isIP(body.host)) return fail(400, "Select an authenticated device and literal IP.");
        if (body.localProtocol !== undefined && body.localProtocol !== "1.0" && body.localProtocol !== "L01") return fail(400, "Unsupported local protocol.");
        const nextDevice = { ...selected, host: body.host, localProtocol: localProtocol(body.localProtocol ?? "1.0") }, nextCamera = { account: login.account, pin: body.pin === undefined ? camera?.pin ?? "" : pinValue(body.pin) }, nextHome = { ...login.home, deviceId: selected.duid };
        // Stop using the old account pairing before changing credentials. Failed persistence remains disconnected.
        await persistPairing(async () => {
          auth.require(req);
          if (account !== attempt || attempt.login !== login) return fail(409, "Account login changed.");
          robot?.close(); cloudRobot?.close(); robot = null; cloudRobot = null; device = null; camera = null; home = null; configured = false;
          await store.savePairing({ device: nextDevice, camera: nextCamera, home: nextHome });
          device = nextDevice; camera = nextCamera; home = nextHome; if (account === attempt) account = null;
          pinConfigured = /^[1-9]{4,12}$/.test(nextCamera.pin);
          if (pinConfigured) { auth.revokeSetup(); account = null; }
          replaceRobot();
        });

        return reply(res, 200, { selected: true });
      }
    }
    if (path === "/api/camera/pin" && req.method === "POST") {
      const body = await readJson(req); auth.require(req); if (!device || !camera || !home) return fail(503, "Authenticated account required.");
      const selected = device, selectedCamera = camera, authorization = home;
      const next = { ...selectedCamera, pin: pinValue(body.pin) };
      await persistPairing(async () => {
        auth.require(req);
        if (device !== selected || camera !== selectedCamera || home !== authorization) return fail(409, "Selected account changed.");
        await store.savePairing({ device: selected, camera: next, home: authorization });
        camera = next; configured = false;
        if (!pinConfigured) account = null;
        pinConfigured = true; auth.revokeSetup();
      });
      return reply(res, 200, { pinCached: true });
    }
    if (path === "/api/driver/camera/select" && req.method === "POST") {
      requireBearer();
      const generation = cameraGeneration; checkCamera(generation);
      const selected = requestedCamera((await readJson(req)).camera);
      if (selected === undefined) return fail(400, "Camera is required.");
      const outcomes: { method: string; outcome: RpcOutcome }[] = [];
      const confirmed = await cameraContext.run(generation, () => selectorOutcomes.run(outcomes, () => cameraSelector.select(selected, captureFrame(generation))));
      checkCamera(generation);
      return reply(res, 200, { camera: confirmed, outcomes });
    }
    if ((path === "/api/camera/frame" || path === "/api/camera/offer") && req.method === "POST") {
      const generation = cameraGeneration; checkCamera(generation);
      const body = await readJson(req);
      const selectedCamera = requestedCamera(body.camera);
      if (selectedCamera !== undefined) return fail(503, "Camera selection moved to ROS /z70/camera/select; frame reads follow the shared lens.");
      if (path.endsWith("/offer")) {
        await configureCamera(generation);
        if (typeof body.sdp !== "string" || !body.sdp.startsWith("v=0") || body.sdp.length > 131072) return fail(400, "Invalid SDP.");
        return reply(res, 200, await driver.offer(body.sdp));
      }
      const { frame, camera: activeCamera, sizeCamera } = await cameraContext.run(generation, () => cameraSelector.run(selectedCamera, captureFrame(generation)));
      checkCamera(generation);
      const dimensions = jpegImageDimensions(frame);
      res.writeHead(200, { "Content-Type": "image/jpeg", "Content-Length": frame.length, "X-Roboclaw-Frame-Id": randomUUID(), "X-Roboclaw-Camera-Session": cameraSession,
        "X-Roboclaw-Received-At": String(Date.now()), "X-Roboclaw-Camera": activeCamera, "X-Roboclaw-Size-Camera": sizeCamera ?? "unknown", "X-Roboclaw-Capture-Time": "unknown", "X-Roboclaw-Lens": "unknown",
        "X-Roboclaw-Image-Width": String(dimensions.width), "X-Roboclaw-Image-Height": String(dimensions.height) }); res.end(frame); return;
    }
    if (path === "/api/agent/camera/hold" && req.method === "POST") {
      const generation = cameraGeneration; checkCamera(generation);
      const body = await readJson(req); if (typeof body.enabled !== "boolean") return fail(400, "Boolean enabled required.");
      await configureCamera(generation);
      if (body.enabled) { if (!driver.preload) return fail(503, "Preload unsupported."); await driver.preload(true); }
      else { if (!driver.unload) return fail(503, "Unload unsupported."); await driver.unload(); }
      return reply(res, 200, { persistentStream: await driver.preloaded?.() ?? null });
    }
    return fail(404, "Not found.");
  }
  const server = createServer((req, res) => {
    res.setHeader("Cache-Control", "no-store"); res.setHeader("X-Content-Type-Options", "nosniff");
    res.setHeader("Content-Security-Policy", "default-src 'self'; img-src 'self' blob: data:; media-src 'self' blob:; connect-src 'self'; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'");
    const job = handle(req, res).catch(error => {
      if (res.headersSent) { res.destroy(); return; }
      reply(res, error instanceof ServiceHttpError ? error.status : 502, { error: error instanceof ServiceHttpError ? error.message : "Service operation failed", dispatch: error instanceof ServiceHttpError ? "not_sent" : "unknown", ...publicCameraFailure(error) });
    }); jobs.add(job); void job.finally(() => jobs.delete(job));
  });
  server.requestTimeout = 15000; server.headersTimeout = 10000; server.keepAliveTimeout = 5000; server.maxHeadersCount = 64;
  async function start() {
    if (stopped) throw new Error("Service stopped");
    await new Promise<void>((resolve, reject) => { server.once("error", reject); server.listen(config.port, config.host, () => { server.off("error", reject); resolve(); }); });
    const addr = server.address();
    if (config.port === 0 && addr && typeof addr === "object") {
      const host = `${config.host}:${addr.port}`;
      if (!config.allowedHosts.includes(host)) config.allowedHosts.push(host);
      config.publicOrigin = `http://${host}`;
    }
  }
  let stopping: Promise<void> | undefined;
  function stop(): Promise<void> {
    if (stopping) return stopping;
    stopped = true; commandEvents.close(); auth.clear(); account = null; driver.close?.(); robot?.close(); cloudRobot?.close();
    stopping = (async () => { await new Promise<void>(resolve => { if (!server.listening) return resolve(); server.close(() => resolve()); server.closeAllConnections(); }); await Promise.allSettled([...jobs]); })();
    return stopping;
  }
  return {
    config, server, start, stop,
    get origin() {
      const addr = server.address();
      return addr && typeof addr === "object" ? `http://${config.host}:${addr.port}` : config.publicOrigin;
    },
  };
}
export async function startService(options: ServiceOptions) { const service = await createService(options); await service.start(); return service; }
