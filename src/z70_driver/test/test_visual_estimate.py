"""Service estimates must drive the same physical joints as the vendor model."""
import math

import pytest

from z70_driver import driver as bridge
from z70_driver.driver import estimate_to_app


@pytest.mark.parametrize("state,joints", [
    ("WORKING_POS", {"m3": 150, "m4": 45, "m5": 0, "m6": 90}),
    ("IN_PROCESS_POS", {"m3": 210, "m4": 110, "m5": 35, "m6": 0}),
])
def test_estimate_maps_vendor_angles_to_joint_rotations(state, joints):
    q = bridge.app_to_urdf(**estimate_to_app({"state": state, "joints": joints}))
    assert q["m3_joint"] == pytest.approx(math.radians(joints["m3"] - 270))
    assert q["m4_joint"] == pytest.approx(math.radians(180 - joints["m4"]))
    assert q["m5_joint"] == pytest.approx(-math.radians(joints["m5"]))
    assert q["m6_left_joint"] == q["m6_right_joint"] == pytest.approx(math.radians(joints["m6"]) / 2)


@pytest.mark.parametrize("state", ["UNKNOWN", "IN_ROBOT"])
def test_untrusted_or_stowed_estimate_folds_visual_arm(state):
    folded = estimate_to_app({"state": state, "joints": {"m3": 0, "m4": 0, "m5": 0, "m6": 0}})
    assert folded == bridge.STOW_VISUAL
    assert bridge.app_to_urdf(**folded)["m3_joint"] == pytest.approx(-math.pi)
    assert bridge.app_to_urdf(**folded)["m4_joint"] == pytest.approx(math.radians(10))


def test_unknown_estimate_with_deployed_joints_preserves_visual_arm_pose():
    deployed = {"m3": 150.0, "m4": 45.0, "m5": 0.0, "m6": 90.0}
    assert estimate_to_app({"state": "UNKNOWN", "joints": deployed}) == deployed
    assert estimate_to_app({"state": "IN_ROBOT", "joints": deployed}) == bridge.STOW_VISUAL


def test_gripper_visual_stays_within_finger_limits():
    assert bridge.app_to_urdf(150, 45, 0, -10)["m6_left_joint"] == 0
    assert bridge.app_to_urdf(150, 45, 0, 180)["m6_left_joint"] == pytest.approx(math.pi / 4)
