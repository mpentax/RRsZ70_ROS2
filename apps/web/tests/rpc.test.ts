import test from "node:test";
import assert from "node:assert/strict";
import { robotRpc } from "../src/rpc.ts";


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
