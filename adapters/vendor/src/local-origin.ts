import { isIP } from "node:net";

/** Explicit local-service destinations, never arbitrary model-provider URLs. */
export function localServiceOrigin(value: string, service: "go2rtc" = "go2rtc"): string {
  let url: URL;
  try { url = new URL(value); }
  catch { throw new Error(`Invalid ${service} service origin.`); }
  const host = url.hostname.replace(/^\[|\]$/g, "");
  const ipv4 = isIP(host) === 4 ? host.split(".").map(Number) : [];
  const privateV4 = ipv4.length === 4 && (ipv4[0] === 127 || ipv4[0] === 10 ||
    (ipv4[0] === 192 && ipv4[1] === 168) || (ipv4[0] === 172 && ipv4[1] >= 16 && ipv4[1] <= 31));
  const allowedHost = privateV4 || host === "::1" || host === "localhost" ||
    host === "host.docker.internal" || host === service;
  if (url.protocol !== "http:" || !allowedHost || url.username || url.password ||
    url.pathname !== "/" || url.search || url.hash) {
    throw new Error(`${service} must use an explicit local HTTP service origin.`);
  }
  return url.origin;
}
