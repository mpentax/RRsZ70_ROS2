/**
 * Z70 arm kinematic model, ported from the Roborock Android plugin's arm checker
 * (Function #32735; see docs/z70-arm-model.md). Vendor-app predictive constants.
 * Validation 2026-10-13: wrist M5 10/20/30 deg commanded -> 6.1/19.3/30.2 deg measured
 * (calibration/arm-validation/LIMITED_POSE_RESULTS.md). M3/M4/hand steps unverified.
 *
 * Planar frame: x forward, y up, metres; y ~ 0 is the floor.
 */
export const SHOULDER = { x: 0.0475, y: 0.14 } as const;
export const L1 = 0.1775;
export const L2 = 0.205;
export const LIMITS = { m3: [90, 270], m4: [35, 170], m5: [0, 155] } as const;
export const JOINT_STEP_DEG = 10;
export const HAND_STEP_M = 0.04;

export type Joint = "m3" | "m4" | "m5" | "m6";
export interface Joints { m3: number; m4: number; m5: number; m6: number }
export interface Point { x: number; y: number }

export const POSES = {
  IN_ROBOT: { m3: 0, m4: 0, m5: 0, m6: 0 },
  WORKING_POS: { m3: 150, m4: 45, m5: 0, m6: 90 },
  TRANSFER_POS: { m3: 150, m4: 75, m5: 0, m6: 0 },
} as const satisfies Record<string, Joints>;

const rad = (d: number) => d * Math.PI / 180;
const deg = (r: number) => r * 180 / Math.PI;

export function forward(m3: number, m4: number): { elbow: Point; wrist: Point } {
  const a1 = rad(270 - m3), a2 = rad(m4 - m3 + 90);
  const elbow = { x: SHOULDER.x + L1 * Math.cos(a1), y: SHOULDER.y + L1 * Math.sin(a1) };
  return { elbow, wrist: { x: elbow.x + L2 * Math.cos(a2), y: elbow.y + L2 * Math.sin(a2) } };
}

/** 2-link IK for the wrist point, choosing the branch closest to `current`. */
export function inverse(target: Point, current: { m3: number; m4: number }): { m3: number; m4: number } | null {
  const dx = target.x - SHOULDER.x, dy = target.y - SHOULDER.y, d2 = dx * dx + dy * dy, d = Math.sqrt(d2);
  if (d > L1 + L2 || d < Math.abs(L1 - L2) || d < 1e-6) return null;
  const c = Math.max(-1, Math.min(1, (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)));
  let best: { m3: number; m4: number } | null = null, bestCost = Infinity;
  for (const s of [1, -1]) {
    const th2 = s * Math.acos(c);
    const th1 = Math.atan2(dy, dx) - Math.atan2(L2 * Math.sin(th2), L1 + L2 * Math.cos(th2));
    let m3 = 270 - deg(th1);
    m3 = ((m3 % 360) + 360) % 360;
    const m4 = deg(th1 + th2) + m3 - 90;
    const cost = Math.abs(m3 - current.m3) + Math.abs(m4 - current.m4);
    if (cost < bestCost) { best = { m3, m4 }; bestCost = cost; }
  }
  return best;
}

export type Fault = "SUCCESS" | "FAULT_JOINT_LIMIT" | "FAULT_ARM_HEIGHT" | "FAULT_ROBOT_COLLISION"
  | "FAULT_GROUND_COLLISION" | "FAULT_PROJECTION" | "FAULT_A4_ANGLE";

/** The app's checkArmInvalidAction geometry (j #32744). */
export function check(j: Joints): Fault {
  for (const k of ["m3", "m4", "m5"] as const) if (j[k] < LIMITS[k][0] || j[k] > LIMITS[k][1]) return "FAULT_JOINT_LIMIT";
  const { wrist: { x, y } } = forward(j.m3, j.m4);
  if (y > 0.37) return "FAULT_ARM_HEIGHT";
  if (Math.abs(x) < 0.24) { if (y < 0.14) return "FAULT_ROBOT_COLLISION"; }
  else { if (y < 0.01) return "FAULT_GROUND_COLLISION"; if (Math.abs(x) > 0.37) return "FAULT_PROJECTION"; }
  if (j.m4 - j.m3 + 90 > 15) return "FAULT_A4_ANGLE";
  return "SUCCESS";
}
