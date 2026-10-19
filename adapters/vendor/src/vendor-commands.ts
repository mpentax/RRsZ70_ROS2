import { MANUAL_JOINT_CONTROLS, type ManualJointControl } from "./arm-joint-protocol.ts";
export type { ManualJointControl } from "./arm-joint-protocol.ts";

export type VendorCommand =
  | { kind: "goto-target"; xMm: number; yMm: number }
  | { kind: "goto-stop" }
  | { kind: "chassis-start" | "chassis-stop" | "chassis-end" | "cancel-cleaning" | "return-dock" }
  | { kind: "chassis-move"; velocity: number; omega: number; durationMs: number; sequence: number }
  | { kind: "arm-mode"; enabled: boolean }
  | { kind: "arm-deploy" | "arm-stow" | "gripper-open" | "gripper-close" | "arm-manual-pickup" | "arm-remote-pickup-mode"
    | "arm-pickup-start" | "arm-pickup-continue" | "arm-pickup-pause"
    | "arm-put-down-start" | "arm-put-down-continue" | "arm-put-down-pause" | "arm-stop-grasp" }
  | { kind: "arm-joint-step"; control: ManualJointControl }
  | { kind: "arm-direction"; direction: "up" | "down" | "forward" | "backward" };

export function vendorResponseTimeout(command: VendorCommand): number {
  return defaultRpcTimeoutMs(vendorCommandRpc(command).method);
}

// A179 arm/gripper/program actions reply only after the mechanism finishes,
// observed beyond 15 s. Everything else replies quickly. This is a ceiling, not a wait.
const SLOW_METHODS = new Set([
  "app_arm_out_compartment", "app_arm_in_compartment", "app_open_gripper", "app_grip",
  "app_pick_up", "app_put_down", "app_pick_up_enter_exit", "app_arm_move", "app_arm_direction_move",
  "app_stop_grasp", "app_start_program", "app_program_enter_exit",
]);

/** Response deadline used when a caller does not pass one. */
export function defaultRpcTimeoutMs(method: string): number {
  return SLOW_METHODS.has(method) ? 30_000 : 5_000;
}

/** Wire mappings only. Inputs are vendor values, not calibrated SI units. */
export function vendorCommandRpc(command: VendorCommand): { method: string; params: unknown } {
  switch (command.kind) {
    case "goto-target":
      if (!Number.isSafeInteger(command.xMm) || !Number.isSafeInteger(command.yMm)) throw new Error("Invalid goto target.");
      return { method: "app_goto_target", params: [command.xMm, command.yMm] };
    case "goto-stop": return { method: "stop_goto_target", params: [] };
    case "chassis-start": return { method: "app_rc_start", params: [] };
    case "chassis-stop": return { method: "app_rc_stop", params: [] };
    case "chassis-end": return { method: "app_rc_end", params: [] };
    case "cancel-cleaning": return { method: "app_stop", params: [] };
    // Native return-to-dock only; RoboClaw supplies no navigation, map, or path inputs.
    case "return-dock": return { method: "app_charge", params: [] };
    case "chassis-move":
      if (!Number.isFinite(command.velocity) || !Number.isFinite(command.omega) ||
        !Number.isSafeInteger(command.durationMs) || command.durationMs < 0 ||
        !Number.isInteger(command.sequence) || command.sequence < 1 || command.sequence > 0x7fffffff)
        throw new Error("Invalid vendor movement fields.");
      return { method: "app_rc_move", params: [{
        velocity: command.velocity, omega: command.omega, duration: command.durationMs, seqnum: command.sequence,
      }] };
    case "arm-mode":
      if (typeof command.enabled !== "boolean") throw new Error("Invalid arm mode.");
      return { method: "app_pick_up_enter_exit", params: { flag: command.enabled ? 1 : 0 } };
    // Plugin UI _setGrapMode(2): distinct remote-control-to-pick-up mode.
    case "arm-remote-pickup-mode": return { method: "app_pick_up_enter_exit", params: { flag: 2 } };
    case "arm-deploy": return { method: "app_arm_out_compartment", params: {} };
    case "arm-stow": return { method: "app_arm_in_compartment", params: {} };
    // Hermes #29618 -> #19907 -> #5505: one named three-input button selection.
    // These values are not angles, physical units, or a trajectory.
    case "arm-joint-step":
      if (typeof command.control !== "string" || !Object.hasOwn(MANUAL_JOINT_CONTROLS, command.control) ||
        Object.keys(command).sort().join(",") !== "control,kind") {
        throw new Error("Invalid manual joint control; only one named step is permitted.");
      }
      return { method: "app_arm_move", params: { m: [...MANUAL_JOINT_CONTROLS[command.control]] } };
    // Vendor cancellation request only; neither ACK nor feedback proves motor stop.
    case "arm-stop-grasp": return { method: "app_stop_grasp", params: [] };
    // Mapping independently published for the a179/Z70 profile. This remains a
    // one-shot discrete step, never a raw joint position or continuous command.
    case "arm-direction": return { method: "app_arm_direction_move", params: {
      direction: { up: 1, down: 2, forward: 3, backward: 4 }[command.direction],
    } };
    // Requests the vendor's manual pick-up workflow; it is a fixed one-shot
    // mode transition, not arbitrary arm motion.
    case "arm-manual-pickup": return { method: "app_pick_up", params: { flag: 1 } };
    // Android Hermes v96: remoteArmPickUp/remoteArmPutDown pass the button's
    // discrete flag through unchanged; never expose arbitrary flag values.
    case "arm-pickup-start": return { method: "app_pick_up", params: { flag: 2 } };
    case "arm-pickup-continue": return { method: "app_pick_up", params: { flag: 3 } };
    case "arm-pickup-pause": return { method: "app_pick_up", params: { flag: 0 } };
    case "arm-put-down-start": return { method: "app_put_down", params: { flag: 0 } };
    case "arm-put-down-continue": return { method: "app_put_down", params: { flag: 1 } };
    case "arm-put-down-pause": return { method: "app_put_down", params: { flag: 2 } };
    case "gripper-open": return { method: "app_open_gripper", params: {} };
    case "gripper-close": return { method: "app_grip", params: {} };
    default: throw new Error("Unknown named vendor command; use generic RPC for other vendor methods.");
  }
}
