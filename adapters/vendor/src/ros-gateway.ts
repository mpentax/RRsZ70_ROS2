import type { ServerResponse } from "node:http";

export { rosRoute } from "@rrsz70/http-contracts";
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
      method, headers: { Authorization: `Bearer ${token}`, ...(body ? { "Content-Type": "application/json" } : {}) },
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
