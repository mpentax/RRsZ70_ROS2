import test from "node:test";
import assert from "node:assert/strict";
import { mkdtemp, writeFile, rm, chmod, symlink } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { Client } from "@modelcontextprotocol/sdk/client/index.js";
import { InMemoryTransport } from "@modelcontextprotocol/sdk/inMemory.js";
import { createRoboClawMcp, mcpServiceOrigin, readMcpToken, RoboClawMcpClient, RoboClawHttpError } from "./mcp-server.ts";

const token = "a".repeat(43);
const origin = mcpServiceOrigin("http://127.0.0.1:57010");
const frame = Buffer.from([0xff, 0xd8, 0xff, 0xd9]);

test("MCP camera selection forwards its camera argument without treating it as an action", async () => {
  let payload: unknown;
  const transport: typeof fetch = async (_url, options) => {
    payload = JSON.parse(options?.body as string);
    return new Response(frame, { headers: { "Content-Type": "image/jpeg", "X-Roboclaw-Camera": "gripper" } });
  };
  const server = createRoboClawMcp(new RoboClawMcpClient(origin, token, transport));
  const client = new Client({ name: "camera-selector-test", version: "1" });
  const [a, b] = InMemoryTransport.createLinkedPair();
  try {
    await server.connect(b); await client.connect(a);
    const reply = await client.callTool({ name: "get_camera_frame", arguments: { camera: "gripper" } });
    assert.notEqual(reply.isError, true);
    assert.deepEqual(payload, { camera: "gripper" });
    const metadata = JSON.parse((reply.content as {text:string}[])[0]!.text);
    assert.equal(metadata.camera, "gripper");
    assert.equal(metadata.defaultRestored, undefined);
    assert.equal(metadata.mimeType, "image/jpeg");
  } finally { await client.close(); await server.close(); }
});

test("shared camera tool reports bounded observation gaps without opening a preview", async (t) => {
  // Scale the real cancellation deadline to keep this regression fast.
  const timeout = AbortSignal.timeout.bind(AbortSignal);
  t.mock.method(AbortSignal, "timeout", (ms: number) => timeout(Math.ceil(ms / 1000)));
  const transport: typeof fetch = async (_url, options) => {
    await new Promise<void>((resolve, reject) => {
      const timer = setTimeout(resolve, 1);
      options?.signal?.addEventListener("abort", () => {
        clearTimeout(timer); reject(options.signal?.reason);
      }, { once: true });
    });
    return Response.json({ error: "OBSERVATION_GAP", dispatch:'not_sent' }, { status: 503 });
  };
  const server = createRoboClawMcp(new RoboClawMcpClient(origin, token, transport));
  const client = new Client({ name: "camera-timeout-test", version: "1" });
  const [a, b] = InMemoryTransport.createLinkedPair();
  try {
    await server.connect(b); await client.connect(a);
    const reply = await client.callTool({ name: "get_camera_frame", arguments: {} });
    assert.equal(reply.isError, true);
    assert.deepEqual(JSON.parse((reply.content as { text: string }[])[0]!.text), {
      status: 503, error:'OBSERVATION_GAP', dispatch:'not_sent',
    });
  } finally { await client.close(); await server.close(); }
});

test("MCP origin only accepts numeric loopback and local service ports; token stays in file", async () => {
  assert.equal(origin.href, "http://127.0.0.1:57010/");
  assert.equal(mcpServiceOrigin("http://[::1]:57000").hostname, "[::1]");
  for (const bad of ["http://localhost:57010", "http://192.168.50.2:57010", "https://127.0.0.1:57010",
    "http://127.0.0.1:1234", "http://a:b@127.0.0.1:57010", "http://127.0.0.1:57010/x",
    "http://127.0.0.1:57010/?secret=1"]) assert.throws(() => mcpServiceOrigin(bad));
  const dir = await mkdtemp(join(tmpdir(), "roboclaw-mcp-test-"));
  try {
    const file = join(dir, "service-token");
    await writeFile(file, token, { mode: 0o600 });
    assert.equal(await readMcpToken(file), token);
    await chmod(file, 0o644);
    await assert.rejects(readMcpToken(file), /Unsafe local service token file/);
    await chmod(file, 0o600);
    await writeFile(file, "not-a-token");
    await assert.rejects(readMcpToken(file), /Unsafe local service token file/);
    await symlink(file, join(dir, "linked-token"));
    await assert.rejects(readMcpToken(join(dir, "linked-token")));
  } finally { await rm(dir, { recursive: true, force: true }); }
});


test("unknown routes cannot receive service bearer", async () => {
  let sends = 0;
  const client = new RoboClawMcpClient(origin, token, async () => { sends++; throw new Error("unexpected"); });
  for (const path of ["/api/session", "/api/commands/recovery", "/api/agent/stop", "/api/v1/devices/../rpc"]) await assert.rejects(client.request(path, 100, {}), /Unsupported/);
  assert.equal(sends, 0);
});

test("camera failures retain bounded diagnostic evidence without exposing upstream secrets", async () => {
  const client = new RoboClawMcpClient(origin, token, async () => Response.json({
    error: "credential-bearing upstream message", dispatch: "unknown",
    cameraFailure: "no_camera_image", upstreamStatus: 200,
    source: "roborock://private?pin=1234", extra: "private",
  }, { status: 502 }));
  await assert.rejects(client.request("/api/ros/frame"), (error: unknown) => {
    assert.ok(error instanceof RoboClawHttpError);
    assert.deepEqual(error.evidence, { dispatch:'unknown' });
    assert.equal(JSON.stringify(error).includes("private"), false);
    return true;
  });
});

test("camera diagnostics reject unknown reasons, invalid status values and unbounded error bodies", async () => {
  for (const body of [
    { cameraFailure: "private-secret", upstreamStatus: 200 },
    { cameraFailure: "no_camera_image", upstreamStatus: "private-secret" },
    { cameraFailure: "no_camera_image", upstreamStatus: 999 },
    { cameraFailure: "no_camera_image", upstreamStatus: 200, extra: "x".repeat(100_000) },
  ]) {
    const client = new RoboClawMcpClient(origin, token, async () => Response.json(body, { status: 502 }));
    await assert.rejects(client.request("/api/ros/frame"), (error: unknown) => {
      assert.ok(error instanceof RoboClawHttpError);
      assert.equal(JSON.stringify(error.evidence ?? {}).includes("private-secret"), false);
      assert.equal(error.evidence?.upstreamStatus, undefined);
      return true;
    });
  }
});

test("bounded camera JPEG and private image handoff retain metadata", async () => {
  const client = new RoboClawMcpClient(origin, token, async () => new Response(frame, { headers: { "content-type": "image/jpeg", "x-roboclaw-frame-id": "frame_1", "x-roboclaw-camera-session": "session_1" } }));
  const value = await client.request("/api/ros/frame");
  assert.ok(Buffer.isBuffer(value));
  assert.equal(client.frameMetadata.get(value)?.frameId, "frame_1");
  assert.equal(client.frameMetadata.get(value)?.mimeType, "image/jpeg");
  const { privateMcpFrame } = await import("./mcp-server.ts");
  const { readFile, stat } = await import("node:fs/promises");
  const path = await privateMcpFrame(value);
  assert.deepEqual(await readFile(path), frame);
  assert.equal((await stat(path)).mode & 0o077, 0);
});

test("camera frame errors retain fixed ROS codes only", async () => {
  const error = "OBSERVATION_GAP";
  const client = new RoboClawMcpClient(origin, token, async () => Response.json({error}, {status: 409}));
  await assert.rejects(client.request("/api/ros/frame", 1000, {camera: "gripper"}), (failure: unknown) => {
    assert.ok(failure instanceof RoboClawHttpError);
    assert.deepEqual(failure.evidence, {error});
    return true;
  });
});

test("MCP select_camera confirms shared mode and default frame reads omit camera", async () => {
  const requests: {path: string; payload: unknown}[] = [];
  const transport: typeof fetch = async (url, options) => {
    const path = new URL(String(url)).pathname;
    requests.push({path, payload: JSON.parse(options?.body as string)});
    return path.endsWith("/select") ? Response.json({camera: "gripper"})
      : new Response(frame, {headers: {"Content-Type": "image/jpeg", "X-Roboclaw-Camera": "gripper"}});
  };
  const server = createRoboClawMcp(new RoboClawMcpClient(origin, token, transport));
  const client = new Client({name: "shared-camera-test", version: "1"});
  const [a,b] = InMemoryTransport.createLinkedPair();
  try {
    await server.connect(b); await client.connect(a);
    const selection = await client.callTool({name: "select_camera", arguments: {camera: "gripper"}});
    assert.notEqual(selection.isError, true);
    assert.deepEqual(JSON.parse((selection.content as {text:string}[])[0]!.text), {camera: "gripper"});
    const image = await client.callTool({name: "get_camera_frame", arguments: {}});
    assert.notEqual(image.isError, true);
    assert.equal(JSON.parse((image.content as {text:string}[])[0]!.text).camera, "gripper");
    assert.deepEqual(requests, [{path: "/api/ros/camera/select", payload: {camera: "gripper"}}, {path: "/api/ros/frame", payload: {}}]);
  } finally {await client.close(); await server.close();}
});
