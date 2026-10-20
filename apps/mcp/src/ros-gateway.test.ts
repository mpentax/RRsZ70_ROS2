import test from "node:test";
import assert from "node:assert/strict";
import { Client } from "@modelcontextprotocol/sdk/client/index.js";
import { InMemoryTransport } from "@modelcontextprotocol/sdk/inMemory.js";
import { createRoboClawMcp, mcpServiceOrigin, RoboClawMcpClient } from "./mcp-server.ts";

const token = "a".repeat(43);
const origin = mcpServiceOrigin("http://127.0.0.1:57010");

test("RoboClawMcpClient supports listGoals, validateGoal, and custom TaskInput in startTask", async () => {
  const calls: { method: string; path: string; payload: unknown }[] = [];
  const transport: typeof fetch = async (url, options) => {
    const path = new URL(String(url)).pathname;
    const method = options?.method ?? "GET";
    const payload = options?.body ? JSON.parse(String(options.body)) : undefined;
    calls.push({ method, path, payload });
    if (path === "/api/ros/goals") {
      return Response.json({
        goals: [{ kind: "pick", topic: "/z70/pick", action: "Pick", fields: ["color", "dry_run"] }],
        context_primitives: [{ name: "status", signature: "status() -> dict", summary: "Read robot status." }],
      });
    }
    if (path === "/api/ros/goals/validate") {
      return Response.json({ valid: true, kind: (payload as { kind: string }).kind, topic: "/z70/custom", action: "CustomGoal" });
    }
    return Response.json({ id: "00000000-0000-0000-0000-000000000001", kind: (payload as { kind: string }).kind, status: "accepted" });
  };

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

test("MCP start_task accepts custom goal kinds and supports validateOnly dry-run validation", async () => {
  const calls: { path: string; payload: unknown }[] = [];
  const transport: typeof fetch = async (url, options) => {
    const path = new URL(String(url)).pathname;
    const payload = options?.body ? JSON.parse(String(options.body)) : undefined;
    calls.push({ path, payload });
    if (path === "/api/ros/goals") {
      return Response.json({
        goals: [{ kind: "pick", topic: "/z70/pick", action: "Pick", fields: ["color", "dry_run"] }],
        context_primitives: [{ name: "status", signature: "status() -> dict", summary: "Read robot status." }],
      });
    }
    if (path === "/api/ros/goals/validate") {
      return Response.json({ valid: true, kind: (payload as { kind: string }).kind, topic: "/z70/custom", action: "Custom" });
    }
    return Response.json({ id: "00000000-0000-0000-0000-000000000002", status: "accepted" });
  };

  const server = createRoboClawMcp(new RoboClawMcpClient(origin, token, transport));
  const client = new Client({ name: "goal-validate-test", version: "1" });
  const [a, b] = InMemoryTransport.createLinkedPair();
  try {
    await server.connect(b);
    await client.connect(a);

    const listed = await client.callTool({ name: "list_goals", arguments: {} });
    assert.notEqual(listed.isError, true);
    assert.deepEqual(JSON.parse((listed.content as { text: string }[])[0]!.text), {
      goals: [{ kind: "pick", topic: "/z70/pick", action: "Pick", fields: ["color", "dry_run"] }],
      context_primitives: [{ name: "status", signature: "status() -> dict", summary: "Read robot status." }],
    });

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

    const simulated = await client.callTool({ name: "start_task", arguments: { task: customTask, simulate: true } });
    assert.notEqual(simulated.isError, true);

    assert.deepEqual(calls, [
      { path: "/api/ros/goals", payload: undefined },
      { path: "/api/ros/goals/validate", payload: customTask },
      { path: "/api/ros/tasks", payload: customTask },
      { path: "/api/ros/goals/validate", payload: { ...customTask, simulate: true } },
    ]);
  } finally {
    await client.close();
    await server.close();
  }
});
