import { MANUAL_JOINT_CONTROLS } from "./arm-joint-protocol.ts";
import { ServiceHttpError } from "./service-auth.ts";
/** Named aliases perform one mapping and one RPC. Modes/order/timing are caller responsibilities. */
export function namedRpc(path: string, body: Record<string, unknown>): { method: string; params: unknown } | null {
  const direct: Record<string, [string, unknown]> = {
    "/api/agent/stop": ["app_rc_stop", []], "/api/chassis/stop": ["app_rc_stop", []],
    "/api/chassis/enable": ["app_rc_start", []], "/api/chassis/end": ["app_rc_end", []],
    "/api/chassis/cancel-task": ["app_stop", []], "/api/agent/chassis/dock": ["app_charge", []],
    "/api/agent/arm/manual-pickup": ["app_pick_up", { flag: 1 }],
    "/api/agent/arm/remote-pickup-mode": ["app_pick_up_enter_exit", { flag: 2 }],
    "/api/agent/arm/remote-pickup-arm-out": ["app_arm_out_compartment", {}],
    "/api/agent/arm/remote-pickup-forward": ["app_arm_direction_move", { direction: 3 }],
    "/api/agent/arm/exit-manual": ["app_pick_up_enter_exit", { flag: 0 }],
    "/api/agent/arm/stop-grasp": ["app_stop_grasp", []], "/api/arm/cancel": ["app_stop_grasp", []],
  };
  const fixed = direct[path];
  if (fixed) return { method: fixed[0], params: fixed[1] };
  const invalid = (): never => { throw new ServiceHttpError(400, "Invalid named vendor RPC arguments."); };
  if (path === "/api/arm/action" || path === "/api/agent/arm/action") {
    const method = ({ deploy: "app_arm_out_compartment", stow: "app_arm_in_compartment", open: "app_open_gripper", close: "app_grip" } as Record<string, string>)[String(body.action)];
    if (typeof method !== "string") return invalid(); return { method, params: {} };
  }
  if (path === "/api/agent/arm/nudge") {
    const direction = ({ up: 1, down: 2, forward: 3, backward: 4 } as Record<string, number>)[String(body.direction)];
    if (typeof direction !== "number") return invalid(); return { method: "app_arm_direction_move", params: { direction } };
  }
  if (path === "/api/agent/arm/joint-step") {
    if (typeof body.control !== "string" || !Object.hasOwn(MANUAL_JOINT_CONTROLS, body.control)) return invalid();
    return { method: "app_arm_move", params: { m: [...MANUAL_JOINT_CONTROLS[body.control as keyof typeof MANUAL_JOINT_CONTROLS]] } };
  }
  if (path === "/api/agent/arm/remote-compartment") {
    const method = ({ stow: "app_arm_in_compartment", grip: "app_grip" } as Record<string, string>)[String(body.operation)];
    if (typeof method !== "string") return invalid(); return { method, params: {} };
  }
  if (path === "/api/agent/arm/pickup-workflow") {
    const op = ({ start: ["app_pick_up", 2], continue: ["app_pick_up", 3], pause: ["app_pick_up", 0], putDown: ["app_put_down", 0], continuePutDown: ["app_put_down", 1], pausePutDown: ["app_put_down", 2] } as Record<string, [string, number]>)[String(body.operation)];
    if (!Array.isArray(op)) return invalid(); return { method: op[0], params: { flag: op[1] } };
  }
  if (path === "/api/agent/camera/mechanical") {
    if (typeof body.enabled !== "boolean") return invalid();
    return { method: "app_change_camera", params: { flag: body.enabled ? 1 : 0 } };
  }
  if (path === "/api/chassis/input") {
    const { velocity, omega, durationMs, sequence } = body;
    if (typeof velocity !== "number" || !Number.isFinite(velocity) || typeof omega !== "number" || !Number.isFinite(omega) ||
      typeof durationMs !== "number" || !Number.isSafeInteger(durationMs) || durationMs < 0 || typeof sequence !== "number" || !Number.isInteger(sequence) || sequence < 1 || sequence > 0x7fffffff) return invalid();
    return { method: "app_rc_move", params: [{ velocity, omega, duration: durationMs, seqnum: sequence }] };
  }
  if (path === "/api/agent/chassis/pulse" || path === "/api/agent/chassis/undock")
    throw new ServiceHttpError(410, "Pulse/undock workflows retired; issue explicit app_rc_start/move/stop/end RPCs from the controller.");
  return null;
}
