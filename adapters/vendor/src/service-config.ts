import { createHash, randomBytes, timingSafeEqual } from "node:crypto";
import type { IncomingMessage, ServerResponse } from "node:http";
import { isIP } from "node:net";
import { resolve } from "node:path";
import { fileURLToPath } from "node:url";

export { rosRoute } from "@rrsz70/http-contracts";

export class ServiceHttpError extends Error {
  readonly status: number;
  constructor(status: number, message: string) {
    super(message);
    this.status = status;
  }
}

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

/** Explicit local-service destinations, never arbitrary model-provider URLs. */
export function localServiceOrigin(value: string, service: "go2rtc" = "go2rtc"): string {
  let url: URL;
  try { url = new URL(value); }
  catch { throw new Error(`Invalid ${service} service origin.`); }
  const host = url.hostname.replace(/^\[|\]$/g, "");
  const ipv4 = isIP(host) === 4 ? host.split(".").map(Number) : [];
  const privateV4 = ipv4.length === 4 && (ipv4[0] === 127 || ipv4[0] === 10 ||
    (ipv4[0] === 192 && ipv4[1] === 168) || (ipv4[0] === 172 && ipv4[1] >= 16 && ipv4[1] <= 31));
  const allowedHost = privateV4 || host === "::1" || host === "localhost";
  if (url.protocol !== "http:" || !allowedHost || url.username || url.password ||
    url.pathname !== "/" || url.search || url.hash) {
    throw new Error(`${service} must use an explicit local HTTP service origin.`);
  }
  return url.origin;
}

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
