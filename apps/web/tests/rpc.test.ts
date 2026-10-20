import test from "node:test";
import assert from "node:assert/strict";
import { listGoals, robotRpc, validateGoal } from "../src/ros.ts";


test("browser RPC uses canonical route, session cookie and unchanged raw outcome", async context => {
  const input = { method: "custom", params: [1, { x: true }], transport: "local" as const, timeoutMs: 1000 };
  const outcome = { requestId: "1", dispatch: "unknown", response: "transport_error", error: "timeout" };
  const paths: string[] = [];
  context.mock.method(globalThis, "fetch", async (path: string, options: RequestInit) => {
    paths.push(path); assert.equal(options.method, "POST"); assert.equal(options.credentials, "same-origin");
    assert.deepEqual(JSON.parse(String(options.body)), input); return Response.json(outcome);
  });
  assert.deepEqual(await robotRpc(input), outcome);
  assert.deepEqual(paths, ["/api/ros/rpc"]);
});

test("listGoals and validateGoal use canonical ROS goal endpoints", async context => {
  const calls: { path: string; method?: string; body?: unknown }[] = [];
  context.mock.method(globalThis, "fetch", async (path: string, options: RequestInit) => {
    calls.push({ path, method: options.method, body: options.body ? JSON.parse(String(options.body)) : undefined });
    if (path === "/api/ros/goals") return Response.json({ goals: [{ kind: "pick" }], context_primitives: [{ name: "status", signature: "status() -> dict", summary: "Read robot status." }] });
    return Response.json({ valid: true, kind: "pick", simulated: { ok: true, result: { verified_hold: true } } });
  });
  const catalog = await listGoals();
  assert.equal(catalog.goals[0].kind, "pick");
  assert.equal(catalog.context_primitives[0].summary, "Read robot status.");
  const validated = await validateGoal({ kind: "pick", goal: { color: "yellow", dry_run: true }, simulate: true }) as { valid: boolean };
  assert.equal(validated.valid, true);
  assert.deepEqual(calls.map(c => [c.method, c.path]), [["GET", "/api/ros/goals"], ["POST", "/api/ros/goals/validate"]]);
});
