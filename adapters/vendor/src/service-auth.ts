import { createHash, randomBytes, timingSafeEqual } from "node:crypto";
import type { IncomingMessage, ServerResponse } from "node:http";
import type { ServiceConfig } from "./service-config.ts";

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
    // Session bootstrap still needs Origin: otherwise a supplied secret can force a victim into a session.
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
    // Trust the exact localhost Host + configured Origin, not forwarded headers or
    // socket peer: Docker's loopback-published port has a bridge peer inside the container.
    // Deployment must keep the published port loopback-bound during initial setup.
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
