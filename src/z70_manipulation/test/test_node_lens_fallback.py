"""Construct the actual perception node without CameraInfo or any hardware bridge."""
import numpy as np
import pytest
import rclpy

from z70_manipulation import planner
from z70_manipulation.node import Perception


def test_node_before_camera_info_uses_shared_pincher_lens():
    rclpy.init(domain_id=91)
    node = None
    try:
        node = Perception(name='pincher_lens_fallback_test', subscribe=False)
        assert node.K == pytest.approx(planner.GRIPPER_MATRIX, abs=1e-12)
        assert node.D == pytest.approx(planner.GRIPPER_D, abs=1e-12)
        assert not np.shares_memory(node.K, planner.GRIPPER_MATRIX)
        assert not np.shares_memory(node.D, planner.GRIPPER_D)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
