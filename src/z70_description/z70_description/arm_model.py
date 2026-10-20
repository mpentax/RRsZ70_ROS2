"""Z70 sagittal 2-link arm kinematic model from the Roborock Android plugin.

Source: Roborock 4.74.08 plugin `index.android.hasm` Function #32735
(`ForwardKinematics`, `InverseKinematics`, `checkArmInvalidAction`).

Frame (app 2-D, sagittal plane): x forward, y up, metres.
y ~ 0 is the floor (ground-collision check y < 0.01); the robot body occupies |x| < 0.24, y < 0.14.
"""
import math
import yaml

from .resources import config_path

SHOULDER = (0.0475, 0.14)       # m2_pos: shoulder pivot (M3)           #32735 / W #32741
VENDOR_L1, VENDOR_L2 = 0.1775, 0.205
_CAD = yaml.safe_load(config_path("cad_arm.yaml").read_text())
L1 = float(_CAD["L1_m"])        # shoulder (M3) -> elbow (M4), CAD link length
L2 = float(_CAD["L2_m"])        # elbow (M4) -> wrist roll (M5), CAD link length
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
    """Sagittal 2-link forward kinematics returning (elbow_xy, end_pos_xy)."""
    a1 = math.radians(270 - m3)
    elbow = (SHOULDER[0] + VENDOR_L1 * math.cos(a1), SHOULDER[1] + VENDOR_L1 * math.sin(a1))
    a2 = math.radians(m4 - m3 + 90)
    end = (elbow[0] + VENDOR_L2 * math.cos(a2), elbow[1] + VENDOR_L2 * math.sin(a2))
    return elbow, end


def vendor_inverse(x, y, prefer_current=None):
    """Sagittal 2-link inverse kinematics for (x, y); returns (m3, m4) or None if unreachable."""
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


def vendor_check(m3, m4, m5):
    """Evaluate `checkArmInvalidAction` workspace and joint limits; returns fault code or 'SUCCESS'."""
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

