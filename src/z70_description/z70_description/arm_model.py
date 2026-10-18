"""Z70 arm kinematic model, as hard-coded in the Roborock Android plugin.

Source: Roborock 4.74.08 plugin `index.android.hasm` (Hermes bytecode disassembly), arm-checker module
Function #32735 (exports ForwardKinematics/InverseKinematics/checkArmInvalidAction), lines ~1428440-1429890.
These are the VENDOR APP'S PREDICTIVE CONSTANTS, not measured on this robot. See docs/z70-arm-model.md.

Frame (app 2-D, sagittal plane): x forward, y up, metres.
The app end_pos is not the CAD M5 wrist; physical FK is pincher_geometry.hand_pose.
y ~ 0 is the floor (ground-collision check y < 0.01); the robot body occupies |x| < 0.24, y < 0.14.
"""
import math
from dataclasses import dataclass, field
import yaml

from .resources import config_path

SHOULDER = (0.0475, 0.14)       # m2_pos: shoulder pivot (M3)           #32735 / W #32741
# Independent app endpoint geometry; never substitute the CAD wrist station.
VENDOR_L1, VENDOR_L2 = 0.1775, 0.205
_CAD = yaml.safe_load(config_path("cad_arm.yaml").read_text())
L1 = float(_CAD["L1_m"])        # shoulder (M3) -> elbow (M4), copied model
L2 = float(_CAD["L2_m"])        # elbow (M4) -> wrist roll (M5), copied model
LIMITS = {"m3": (90, 270), "m4": (35, 170), "m5": (0, 155)}            # #32735 arrays
JOINT_STEP_DEG = 10             # arm_joints: +-10 deg per repetition    q #32745
HAND_STEP_M = 0.04              # arm_move: end point +-0.04 m in x or y, then IK   q #32745
A4_MAX_DEG = 15                 # link-2 absolute angle (m4 - m3 + 90) must be <= 15
HEIGHT_MAX = 0.37               # end_pos.y > 0.37 -> FAULT_ARM_HEIGHT
BODY_HALF_X, BODY_TOP = 0.24, 0.14  # |x| < 0.24 and y < 0.14 -> FAULT_ROBOT_COLLISION
GROUND_Y = 0.01                 # y < 0.01 (outside body) -> FAULT_GROUND_COLLISION
PROJECTION_X = 0.37             # |x| > 0.37 -> FAULT_PROJECTION

# Named poses (state machine w #32743, b #32742, W #32741)
POSES = {
    "IN_ROBOT":     dict(m3=0,   m4=0,  m5=0, m6=0),   # stowed; app collapses positions to the shoulder point
    "WORKING_POS":  dict(m3=150, m4=45, m5=0, m6=90),  # after ARM_OUT (gripper open)
    "TRANSFER_POS": dict(m3=150, m4=75, m5=0, m6=0),   # after ARM_GRASP (gripper closed)
}
# Transitions: ARM_OUT from IN_ROBOT -> WORKING_POS; ARM_GRASP (m6==90) -> TRANSFER_POS;
# ARM_PUTDOWN (state != IN_ROBOT, m6==0) -> WORKING_POS; ARM_IN -> IN_ROBOT;
# ARM_JOINTS / ARM_MOVE -> IN_PROCESS_POS.


def vendor_forward(m3, m4):
    """Recovered app FK: (elbow_xy, end_pos_xy), not physical wrist TF."""
    a1 = math.radians(270 - m3)
    elbow = (SHOULDER[0] + VENDOR_L1 * math.cos(a1), SHOULDER[1] + VENDOR_L1 * math.sin(a1))
    a2 = math.radians(m4 - m3 + 90)
    end = (elbow[0] + VENDOR_L2 * math.cos(a2), elbow[1] + VENDOR_L2 * math.sin(a2))
    return elbow, end


def vendor_inverse(x, y, prefer_current=None):
    """IK of app end_pos; nearest requested branch, or None if unreachable."""
    L1, L2 = VENDOR_L1, VENDOR_L2
    dx, dy = x - SHOULDER[0], y - SHOULDER[1]
    d2 = dx * dx + dy * dy
    d = math.sqrt(d2)
    if d > L1 + L2 or d < abs(L1 - L2) or d < 1e-6:
        return None
    sols = []
    for s in (1, -1):
        c = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        th2 = s * math.acos(max(-1.0, min(1.0, c)))                  # link2 relative to link1
        th1 = math.atan2(dy, dx) - math.atan2(L2 * math.sin(th2), L1 + L2 * math.cos(th2))
        m3 = 270 - math.degrees(th1)
        m4 = math.degrees(th1 + th2) + m3 - 90
        m3 %= 360
        sols.append((m3, m4))
    if prefer_current:
        sols.sort(key=lambda s: abs(s[0] - prefer_current[0]) + abs(s[1] - prefer_current[1]))
    return sols[0]


# Compatibility API: these functions have always promised the recovered app model.
# L1/L2 remain CAD constants for existing physical consumers, not FK inputs.
forward = vendor_forward
inverse = vendor_inverse


def vendor_check(m3, m4, m5):
    """Replicates checkArmInvalidAction's geometric checks (j #32744). Returns fault name or 'SUCCESS'."""
    for k, v in (("m3", m3), ("m4", m4), ("m5", m5)):
        lo, hi = LIMITS[k]
        if v < lo or v > hi:
            return "FAULT_JOINT_LIMIT"
    _, (x, y) = vendor_forward(m3, m4)
    if y > HEIGHT_MAX:
        return "FAULT_ARM_HEIGHT"
    if abs(x) < BODY_HALF_X:
        if y < BODY_TOP:
            return "FAULT_ROBOT_COLLISION"
    else:
        if y < GROUND_Y:
            return "FAULT_GROUND_COLLISION"
        if abs(x) > PROJECTION_X:
            return "FAULT_PROJECTION"
    if m4 - m3 + 90 > A4_MAX_DEG:
        return "FAULT_A4_ANGLE"
    return "SUCCESS"


check = vendor_check  # Compatibility for executor/node firmware-admission callers.


@dataclass
class ArmState:
    """Historical app-preview simulation, not the ROS runtime estimator."""
    state: str = "IN_ROBOT"
    m3: float = 0
    m4: float = 0
    m5: float = 0
    m6: float = 0
    history: list = field(default_factory=list)

    def apply(self, inst, **kw):
        if inst == "ARM_OUT" and self.state == "IN_ROBOT":
            self._set("WORKING_POS")
        elif inst == "ARM_IN":
            self._set("IN_ROBOT")
        elif inst == "ARM_GRASP" and self.m6 == 90:
            self._set("TRANSFER_POS")
        elif inst == "ARM_PUTDOWN" and self.state != "IN_ROBOT" and self.m6 == 0:
            self._set("WORKING_POS")
        elif inst == "ARM_JOINTS":                     # kw: joint in m3..m6, direction +1/-1, n_rep
            j, sgn, n = kw["joint"], kw["direction"], kw.get("n_rep", 1)
            if j == "m6":
                self.m6 = 90 if sgn > 0 else 0
            else:
                setattr(self, j, getattr(self, j) + sgn * JOINT_STEP_DEG * n)
            self.state = "IN_PROCESS_POS"
        elif inst == "ARM_MOVE":                       # kw: axis 'x'|'y', direction +1/-1, n_rep
            for _ in range(kw.get("n_rep", 1)):
                _, (x, y) = forward(self.m3, self.m4)
                x += HAND_STEP_M * kw["direction"] if kw["axis"] == "x" else 0
                y += HAND_STEP_M * kw["direction"] if kw["axis"] == "y" else 0
                sol = inverse(x, y, (self.m3, self.m4))
                if sol:
                    self.m3, self.m4 = sol
            self.state = "IN_PROCESS_POS"
        self.history.append((inst, kw, (self.m3, self.m4, self.m5, self.m6)))
        return check(self.m3, self.m4, self.m5) if self.state != "IN_ROBOT" else "SUCCESS"

    def _set(self, name):
        self.state = name
        for k, v in POSES[name].items():
            setattr(self, k, v)


if __name__ == "__main__":
    for name in ("WORKING_POS", "TRANSFER_POS"):
        p = POSES[name]
        elbow, end = forward(p["m3"], p["m4"])
        print(f"{name:13s} elbow=({elbow[0]:+.3f},{elbow[1]:.3f})  end=({end[0]:+.3f},{end[1]:.3f})  "
              f"check={check(p['m3'], p['m4'], p['m5'])}")
    s = ArmState(); s.apply("ARM_OUT")
    for i in range(1, 7):
        r = s.apply("ARM_MOVE", axis="x", direction=+1)
        _, end = forward(s.m3, s.m4)
        print(f"after {i} x HAND_MOVE_FRONT: m3={s.m3:6.1f} m4={s.m4:6.1f} end=({end[0]:+.3f},{end[1]:.3f}) {r}")
    for (m3, m4) in [(150, 45), (200, 90), (240, 130)]:
        _, e = forward(m3, m4); ik = inverse(*e, (m3, m4))
        print(f"IK roundtrip ({m3},{m4}) -> ({ik[0]:.2f},{ik[1]:.2f})")
