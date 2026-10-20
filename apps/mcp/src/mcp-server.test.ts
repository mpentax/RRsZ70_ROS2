import test from "node:test";
import assert from "node:assert/strict";
import { mkdtemp, writeFile, rm, chmod, symlink, readFile, stat } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { Client } from "@modelcontextprotocol/sdk/client/index.js";
import { InMemoryTransport } from "@modelcontextprotocol/sdk/inMemory.js";
import { createRoboClawMcp, mcpServiceOrigin, privateMcpFrame, readMcpToken, RoboClawMcpClient, RoboClawHttpError } from "./mcp-server.ts";

const token = "a".repeat(43);
const origin = mcpServiceOrigin("http://127.0.0.1:57010");
const frame = Buffer.from([0xff, 0xd8, 0xff, 0xd9]);

async function connectClient(t: { after(fn: () => unknown): void }, transport: typeof fetch) {
  const server = createRoboClawMcp(new RoboClawMcpClient(origin, token, transport));
  const client = new Client({ name: "test", version: "1" });
  const [a, b] = InMemoryTransport.createLinkedPair();
  await server.connect(b);
  await client.connect(a);
  t.after(async () => { await client.close(); await server.close(); });
  return client;
}

test("MCP camera selection forwards its camera argument without treating it as an action", async (t) => {
  let payload: unknown;
  const client = await connectClient(t, async (_url, options) => {
    payload = JSON.parse(options?.body as string);
    return new Response(frame, { headers: { "Content-Type": "image/jpeg", "X-Roboclaw-Camera": "gripper" } });
  });
  const reply = await client.callTool({ name: "get_camera_frame", arguments: { camera: "gripper" } });
  assert.notEqual(reply.isError, true);
  assert.deepEqual(payload, { camera: "gripper" });
  const metadata = JSON.parse((reply.content as {text:string}[])[0]!.text);
  assert.equal(metadata.camera, "gripper");
  assert.equal(metadata.defaultRestored, undefined);
  assert.equal(metadata.mimeType, "image/jpeg");
});

test("shared camera tool reports bounded observation gaps without opening a preview", async (t) => {
  // Scale the real cancellation deadline to keep this regression fast.
  const timeout = AbortSignal.timeout.bind(AbortSignal);
  t.mock.method(AbortSignal, "timeout", (ms: number) => timeout(Math.ceil(ms / 1000)));
  const client = await connectClient(t, async (_url, options) => {
    await new Promise<void>((resolve, reject) => {
      const timer = setTimeout(resolve, 1);
      options?.signal?.addEventListener("abort", () => {
        clearTimeout(timer); reject(options.signal?.reason);
      }, { once: true });
    });
    return Response.json({ error: "OBSERVATION_GAP", dispatch:'not_sent' }, { status: 503 });
  });
  const reply = await client.callTool({ name: "get_camera_frame", arguments: {} });
  assert.equal(reply.isError, true);
  assert.deepEqual(JSON.parse((reply.content as { text: string }[])[0]!.text), {
    status: 503, error:'OBSERVATION_GAP', dispatch:'not_sent',
  });
});

test("MCP origin only accepts numeric loopback and local service ports; token stays in file", async (t) => {
  assert.equal(origin.href, "http://127.0.0.1:57010/");
  assert.equal(mcpServiceOrigin("http://[::1]:57000").hostname, "[::1]");
  for (const bad of ["http://localhost:57010", "http://192.168.50.2:57010", "https://127.0.0.1:57010",
    "http://127.0.0.1:1234", "http://a:b@127.0.0.1:57010", "http://127.0.0.1:57010/x",
    "http://127.0.0.1:57010/?secret=1"]) assert.throws(() => mcpServiceOrigin(bad));
  const dir = await mkdtemp(join(tmpdir(), "roboclaw-mcp-test-"));
  t.after(() => rm(dir, { recursive: true, force: true }));
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

test("MCP select_camera confirms shared mode and default frame reads omit camera", async (t) => {
  const requests: {path: string; payload: unknown}[] = [];
  const client = await connectClient(t, async (url, options) => {
    const path = new URL(String(url)).pathname;
    requests.push({path, payload: JSON.parse(options?.body as string)});
    return path.endsWith("/select") ? Response.json({camera: "gripper"})
      : new Response(frame, {headers: {"Content-Type": "image/jpeg", "X-Roboclaw-Camera": "gripper"}});
  });
  const selection = await client.callTool({name: "select_camera", arguments: {camera: "gripper"}});
  assert.notEqual(selection.isError, true);
  assert.deepEqual(JSON.parse((selection.content as {text:string}[])[0]!.text), {camera: "gripper"});
  const image = await client.callTool({name: "get_camera_frame", arguments: {}});
  assert.notEqual(image.isError, true);
  assert.equal(JSON.parse((image.content as {text:string}[])[0]!.text).camera, "gripper");
  assert.deepEqual(requests, [{path: "/api/ros/camera/select", payload: {camera: "gripper"}}, {path: "/api/ros/frame", payload: {}}]);
});

test("MCP exposes generic RPC and observation tools without challenges or prior latch", async (t) => {
  const calls: { path: string; payload: unknown }[] = [];
  const outcome = { requestId: "r1", dispatch: "sent", response: "vendor_error", vendorCode: -1, result: { raw: true } };
  const client = await connectClient(t, async (url, options) => {
    assert.equal(new Headers(options?.headers).get("authorization"), `Bearer ${token}`);
    assert.equal(options?.redirect, "error");
    calls.push({ path: new URL(String(url)).pathname, payload: JSON.parse(String(options?.body ?? "{}")) });
    return Response.json(outcome);
  });
  assert.deepEqual((await client.listTools()).tools.map(t => t.name).sort(), ["cancel_task", "control_session", "get_arm_estimate", "get_camera_diagnostics", "get_camera_frame", "get_map", "get_state", "get_task", "list_goals", "list_tasks", "refresh_status", "robot_rpc", "select_camera", "set_arm_anchor", "start_task", "stop"]);
  const params = { arbitrary: [1, false, { raw: "value" }] };
  for (const takeover of [false, true]) {
    const reply = await client.callTool({ name: "robot_rpc", arguments: { method: "any_vendor_method", params, transport: "cloud", timeoutMs: 3456, takeover } });
    assert.deepEqual(JSON.parse((reply.content as { text: string }[])[0]!.text), outcome);
  }
  assert.deepEqual(calls, [false, true].map(takeover => ({ path: "/api/ros/rpc", payload: { method: "any_vendor_method", params, transport: "cloud", timeoutMs: 3456, takeover } })));
  for (const enabled of [false, true]) {
    const reply = await client.callTool({ name: "control_session", arguments: { enabled } });
    assert.deepEqual(JSON.parse((reply.content as { text: string }[])[0]!.text), outcome);
    assert.deepEqual(calls.at(-1), { path: "/api/ros/control/session", payload: { enabled } });
  }
  for (const task of [
    { kind: "deliver_garment", goal: { category: "sock", drop_x_m: 1, drop_y_m: 2, max_duration_s: 60, allow_towing: false, dry_run: true } },
    { kind: "patrol_collect", goal: { drop_x_m: 1, drop_y_m: 2, max_duration_s: 600, max_items: 10, allow_towing: false, dry_run: true } },
    { kind: "pin_and_go", goal: { target_x_m: 24.5, target_y_m: 26.0, max_duration_s: 60, dry_run: true } },
    { kind: "deploy_arm", goal: { settle: true } },
    { kind: "align_target", goal: { color: "yellow" } },
    { kind: "move_arm", goal: { m3_deg: 190, m4_deg: 65, m5_deg: 0 } },
    { kind: "grasp_and_lift", goal: { lift_steps: 2, verify_color: "yellow" } },
    { kind: "release_and_stow", goal: { put_down: true } },
    { kind: "arm_operation", goal: { operation: 6 } },
    { kind: "arm_operation", goal: { operation: 7 } },
    { kind: "arm_operation", goal: { operation: 8 } },
    { kind: "arm_operation", goal: { operation: 9 } },
    { kind: "stack", goal: { base_color: "yellow", middle_color: "blue", top_color: "purple", dry_run: true } },
  ] as const) {
    const reply = await client.callTool({ name: "start_task", arguments: { task } });
    assert.notEqual(reply.isError, true);
    assert.deepEqual(calls.at(-1), { path: "/api/ros/tasks", payload: task });
  }
});

const catalogFixture = {
  goals: [{ kind: "pick", topic: "/z70/pick", action: "Pick", fields: ["color", "dry_run"] }],
  context_primitives: [{ name: "status", signature: "status() -> dict", summary: "Read robot status." }],
};

function mockGateway(action = "CustomGoal") {
  const calls: { method: string; path: string; payload: unknown }[] = [];
  const transport: typeof fetch = async (url, options) => {
    const path = new URL(String(url)).pathname;
    const method = options?.method ?? "GET";
    const payload = options?.body ? JSON.parse(String(options.body)) : undefined;
    calls.push({ method, path, payload });
    if (path === "/api/ros/goals") return Response.json(catalogFixture);
    if (path === "/api/ros/goals/validate") {
      return Response.json({ valid: true, kind: (payload as { kind: string }).kind, topic: "/z70/custom", action });
    }
    return Response.json({ id: "00000000-0000-0000-0000-000000000001", kind: (payload as { kind?: string })?.kind, status: "accepted" });
  };
  return { calls, transport };
}

test("RoboClawMcpClient supports listGoals, validateGoal, and custom TaskInput in startTask", async () => {
  const { calls, transport } = mockGateway("CustomGoal");
  const gatewayClient = new RoboClawMcpClient(origin, token, transport);

  const catalog = await gatewayClient.listGoals();
  assert.equal(catalog.goals[0]?.kind, "pick");
  assert.equal(catalog.context_primitives?.[0]?.name, "status");

  const customTask = { kind: "custom_inspect", goal: { target: "yellow", dry_run: true } };
  const validation = await gatewayClient.validateGoal(customTask);
  assert.deepEqual(validation, { valid: true, kind: "custom_inspect", topic: "/z70/custom", action: "CustomGoal" });

  const started = await gatewayClient.startTask(customTask) as { id: string; kind: string };
  assert.equal(started.kind, "custom_inspect");

  assert.deepEqual(calls, [
    { method: "GET", path: "/api/ros/goals", payload: undefined },
    { method: "POST", path: "/api/ros/goals/validate", payload: customTask },
    { method: "POST", path: "/api/ros/tasks", payload: customTask },
  ]);
});

test("MCP start_task accepts custom goal kinds and supports validateOnly and simulate/context dry-run validation", async (t) => {
  const { calls, transport } = mockGateway("Custom");
  const client = await connectClient(t, transport);

  const listed = await client.callTool({ name: "list_goals", arguments: {} });
  assert.notEqual(listed.isError, true);
  assert.deepEqual(JSON.parse((listed.content as { text: string }[])[0]!.text), catalogFixture);

  const customTask = { kind: "custom_sweep", goal: { radius_m: 1.5, dry_run: true } };
  const validated = await client.callTool({ name: "start_task", arguments: { task: customTask, validateOnly: true } });
  assert.notEqual(validated.isError, true);
  assert.deepEqual(JSON.parse((validated.content as { text: string }[])[0]!.text), {
    valid: true,
    kind: "custom_sweep",
    topic: "/z70/custom",
    action: "Custom",
  });

  const dispatched = await client.callTool({ name: "start_task", arguments: { task: customTask } });
  assert.notEqual(dispatched.isError, true);

  const simulated = await client.callTool({
    name: "start_task",
    arguments: { task: customTask, simulate: true, context: { status: { state: 39 } } },
  });
  assert.notEqual(simulated.isError, true);

  assert.deepEqual(calls.map(({ path, payload }) => ({ path, payload })), [
    { path: "/api/ros/goals", payload: undefined },
    { path: "/api/ros/goals/validate", payload: customTask },
    { path: "/api/ros/tasks", payload: customTask },
    { path: "/api/ros/goals/validate", payload: { ...customTask, simulate: true, context: { status: { state: 39 } } } },
  ]);
});
