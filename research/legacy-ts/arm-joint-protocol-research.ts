/** Primary Hermes trace: #29618 -> #19907 -> #5505. Offline protocol fixtures only.
 * These are directional selections, never calibrated positions or trajectories.
 * No RobotClient, service route or motion permission is attached to this module.
 */
import { MANUAL_JOINT_CONTROLS, type ManualJointControl } from "../../adapters/vendor/src/arm-joint-protocol.ts";

export function manualJointFixture(control: ManualJointControl) {
  if (typeof control !== "string" || !Object.hasOwn(MANUAL_JOINT_CONTROLS, control)) throw new TypeError("Unknown manual joint control");
  return { method: "app_arm_move" as const, params: { m: [...MANUAL_JOINT_CONTROLS[control]] },
    semantics: "directional-selection" as const, physicalUnits: "unverified" as const, executionAvailable: false as const };
}

/** Program enums have different labels from the manual live-RPC button labels.
 * #32758: arm_joints selects M3/M4/M5/M6; arm_move selects end-effector direction.
 */
export type ProgramArmFixture =
  | { cmd_id: "arm_joints"; value: [number, number, number, number]; step: number }
  | { cmd_id: "arm_move"; direction: number; step: number };

export function parseProgramArmFixture(value: unknown): ProgramArmFixture {
  if (!value || typeof value !== "object" || Array.isArray(value)) throw new TypeError("Invalid program arm fixture");
  const v = value as Record<string, unknown>;
  // Single iteration is a research scope bound, NOT a discovered firmware limit.
  if (v.step !== 1) throw new TypeError("Only one research iteration is represented");
  if (v.cmd_id === "arm_joints" && Object.keys(v).sort().join(",") === "cmd_id,step,value" &&
    Array.isArray(v.value) && v.value.length === 4 && v.value.every(n => Number.isInteger(n) && n >= 0 && n <= 2) &&
    v.value.filter(n => n !== 0).length === 1) {
    return { cmd_id: "arm_joints", value: [...v.value] as [number, number, number, number], step: 1 };
  }
  if (v.cmd_id === "arm_move" && Object.keys(v).sort().join(",") === "cmd_id,direction,step" &&
    Number.isInteger(v.direction) && Number(v.direction) >= 1 && Number(v.direction) <= 4) {
    return { cmd_id: "arm_move", direction: Number(v.direction), step: 1 };
  }
  throw new TypeError("Unsupported program arm fixture");
}

export const ARM_INTERFACE_RESEARCH = Object.freeze({
  fiveInputWrapper: { functionId: 5570, identifiedCaller: false, slotUnits: "unknown", absoluteTargetsVerified: false },
  jointQuery: { functionId: 5537, measuredFeedbackVerified: false },
  programModel: { measured: false, jointIncrementDegrees: 10, handIncrementModelUnits: 0.04 },
  cameraSelector: { functionId: 28055, requiresModeEntryInWrapper: false, settingsBitFallbackEquivalent: false },
});
