import { isIP } from "node:net";
import { resolve } from "node:path";
import { fileURLToPath } from "node:url";

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
