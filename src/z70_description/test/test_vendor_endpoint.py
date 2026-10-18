"""Recovered app arithmetic, not a physical calibration or robot trial."""
import math

import pytest

from z70_description import arm_model as model


def recovered_forward(m3, m4):
    a1, a2 = map(math.radians, (270 - m3, m4 - m3 + 90))
    elbow = (.0475 + .1775 * math.cos(a1), .14 + .1775 * math.sin(a1))
    return elbow, (elbow[0] + .205 * math.cos(a2), elbow[1] + .205 * math.sin(a2))


@pytest.mark.parametrize('angles', [(150, 45), (200, 90), (240, 130)])
def test_compatibility_fk_and_ik_use_recovered_endpoint_lengths(angles):
    expected = recovered_forward(*angles)
    elbow, endpoint = model.forward(*angles)
    assert model.vendor_forward(*angles) == model.forward(*angles)
    assert model.vendor_inverse(*expected[1], angles) == model.inverse(*expected[1], angles)
    assert elbow == pytest.approx(expected[0])
    assert endpoint == pytest.approx(expected[1])
    assert model.inverse(*expected[1], angles) == pytest.approx(angles)


@pytest.mark.parametrize('angles,fault', [
    ((89, 45, 0), 'FAULT_JOINT_LIMIT'),
    ((100, 90, 0), 'FAULT_ARM_HEIGHT'),
    ((240, 35, 0), 'FAULT_ROBOT_COLLISION'),
    ((250, 80, 0), 'FAULT_GROUND_COLLISION'),
    ((240, 130, 0), 'FAULT_PROJECTION'),
    ((100, 35, 0), 'FAULT_A4_ANGLE'),
    ((150, 45, 0), 'SUCCESS'),
])
def test_explicit_vendor_checker_keeps_recovered_fault_order(angles, fault):
    assert model.vendor_check(*angles) == fault
    assert model.check(*angles) == fault
