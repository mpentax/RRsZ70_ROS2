/** Proven directional wire values; not positions, feedback or trajectories. */
export type ManualJointControl = "m3-left" | "m3-right" | "m4-left" | "m4-right" | "m5-left" | "m5-right";
export const MANUAL_JOINT_CONTROLS = Object.freeze({
  "m3-left": Object.freeze([2, 0, 0]), "m3-right": Object.freeze([1, 0, 0]),
  "m4-left": Object.freeze([0, 1, 0]), "m4-right": Object.freeze([0, 2, 0]),
  "m5-left": Object.freeze([0, 0, 2]), "m5-right": Object.freeze([0, 0, 1]),
} as const);
