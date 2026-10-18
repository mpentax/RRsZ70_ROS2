import { test } from "node:test";
import assert from "node:assert/strict";
import { namedRpc } from "./vendor-rpc.ts";
import { vendorCommandRpc } from "./vendor-commands.ts";
test("movement mapping keeps simultaneous translation/rotation and removes experimental policy limits", () => {
  const params = [{ velocity: 0.9, omega: 1.8, duration: 10000, seqnum: 12 }];
  assert.deepEqual(namedRpc("/api/chassis/input", { velocity: 0.9, omega: 1.8, durationMs: 10000, sequence: 12 }), { method: "app_rc_move", params });
  assert.deepEqual(vendorCommandRpc({ kind: "chassis-move", velocity: 0.9, omega: 1.8, durationMs: 10000, sequence: 12 }), { method: "app_rc_move", params });
});
test("aliases preserve vendor mappings and never sequence mode entry or docking", () => {
  assert.deepEqual(namedRpc("/api/agent/chassis/dock", {}), { method: "app_charge", params: [] });
  assert.deepEqual(namedRpc("/api/agent/arm/action", { action: "deploy" }), { method: "app_arm_out_compartment", params: {} });
  assert.deepEqual(namedRpc("/api/agent/camera/mechanical", { enabled: true }), { method: "app_change_camera", params: { flag: 1 } });
  assert.throws(() => namedRpc("/api/agent/chassis/pulse", { direction: "forward", durationMs: 500 }), /workflows retired/);
});

test("inherited mapping properties are never accepted as named vendor arguments", () => {
  for (const value of ["toString", "constructor", "__proto__"]) {
    for (const [path, body] of [["/api/agent/arm/action", { action: value }], ["/api/agent/arm/nudge", { direction: value }], ["/api/agent/arm/pickup-workflow", { operation: value }], ["/api/agent/arm/remote-compartment", { operation: value }]] as const)
      assert.throws(() => namedRpc(path, body), /Invalid named/);
  }
});
