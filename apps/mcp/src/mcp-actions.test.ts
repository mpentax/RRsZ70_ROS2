import test from "node:test";
import assert from "node:assert/strict";
import { Client } from "@modelcontextprotocol/sdk/client/index.js";
import { InMemoryTransport } from "@modelcontextprotocol/sdk/inMemory.js";
import { createRoboClawMcp, RoboClawMcpClient, mcpServiceOrigin } from "./mcp-server.ts";

test("MCP exposes generic RPC and observation tools without challenges or prior latch", async () => {
  const calls: { path: string; payload: unknown }[] = [];
  const outcome = { requestId: "r1", dispatch: "sent", response: "vendor_error", vendorCode: -1, result: { raw: true } };
  const server = createRoboClawMcp(new RoboClawMcpClient(mcpServiceOrigin("http://127.0.0.1:57010"), "a".repeat(43), async (url, options) => {
    assert.equal(new Headers(options?.headers).get("authorization"), `Bearer ${"a".repeat(43)}`);
    assert.equal(options?.redirect, "error");
    calls.push({ path: new URL(String(url)).pathname, payload: JSON.parse(String(options?.body ?? "{}")) });
    return Response.json(outcome);
  }));
  const client = new Client({ name: "test", version: "1" });
  const [a, b] = InMemoryTransport.createLinkedPair();
  try {
    await server.connect(b); await client.connect(a);
    assert.deepEqual((await client.listTools()).tools.map(t => t.name).sort(), ["cancel_task", "control_session", "get_arm_estimate", "get_camera_diagnostics", "get_camera_frame", "get_map", "get_state", "get_task", "list_tasks", "refresh_status", "robot_rpc", "select_camera", "set_arm_anchor", "start_task", "stop"]);
    const params = { arbitrary: [1, false, { raw: "value" }] };
    for (const takeover of [false, true]) {
      const reply = await client.callTool({ name: "robot_rpc", arguments: { method: "any_vendor_method", params, transport: "cloud", timeoutMs: 3456, takeover } });
      assert.deepEqual(JSON.parse((reply.content as { text: string }[])[0]!.text), outcome);
    }
    assert.deepEqual(calls, [false, true].map(takeover => ({ path:'/api/ros/rpc', payload: { method: "any_vendor_method", params, transport: "cloud", timeoutMs: 3456, takeover } })));
    for (const enabled of [false, true]) {
      const reply = await client.callTool({name:'control_session', arguments:{enabled}});
      assert.deepEqual(JSON.parse((reply.content as {text:string}[])[0]!.text), outcome);
      assert.deepEqual(calls.at(-1), {path:'/api/ros/control/session', payload:{enabled}});
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
  } finally { await client.close(); await server.close(); }
});
