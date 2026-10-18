import { isIP } from "node:net";
import { resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { localServiceOrigin } from "./local-origin.ts";

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
  visionOrigin: string;
  /** @deprecated Use `visionOrigin`; retained for compatibility with older Ollama-only configs. */
  ollamaOrigin: string;
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

export function localOrigin(value: string, kind: "camera" | "vision"): string {
  const url = origin(value);
  localServiceOrigin(value, kind === "camera" ? "go2rtc" : "vision");
  const port = Number(url.port || (url.protocol === "https:" ? 443 : 80));
  if (kind === "camera" && (port < 57000 || port > 57100)) {
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
  const visionOrigin = env.VISION_ORIGIN ?? env.LMSTUDIO_ORIGIN ?? env.OLLAMA_ORIGIN ?? "http://127.0.0.1:57003";
  const resolvedVisionOrigin = localOrigin(visionOrigin, "vision");
  return {
    host, port, publicOrigin, allowedHosts: [...new Set(allowedHosts)], secureCookie,
    stateDir: resolve(env.ROBOCLAW_STATE_DIR ?? resolve(ROOT, ".local")),
    webRoot: resolve(env.ROBOCLAW_WEB_ROOT ?? resolve(ROOT, "dist/web")),
    go2rtcOrigin: localOrigin(env.GO2RTC_ORIGIN ?? "http://127.0.0.1:57001", "camera"),
    visionOrigin: resolvedVisionOrigin,
    ollamaOrigin: resolvedVisionOrigin,
  };
}
