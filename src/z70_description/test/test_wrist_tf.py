"""Wrist TF verification through robot_state_publisher."""
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
import xacro
import yaml

from z70_driver.driver import app_to_urdf
from z70_manipulation import planner
from z70_manipulation.node import rotation as quat_matrix


@pytest.fixture(scope="module")
def tf_robot(tmp_path_factory):
    directory = tmp_path_factory.mktemp("wrist_tf")
    description = xacro.process_file(str(Path(get_package_share_directory("z70_description")) / "urdf/z70.urdf.xacro")).toxml()
    params = directory / "robot.yaml"
    params.write_text(yaml.safe_dump({"robot_state_publisher": {"ros__parameters": {"robot_description": description}}}))
    executable = Path(get_package_prefix("robot_state_publisher")) / "lib/robot_state_publisher/robot_state_publisher"
    log = (directory / "publisher.log").open("w")
    process = subprocess.Popen([str(executable), "--ros-args", "--params-file", str(params)],
                               env={**os.environ, "ROS_DOMAIN_ID": "91"}, stdout=log, stderr=log)
    rclpy.init(domain_id=91)
    node = rclpy.create_node("wrist_fk_test")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(JointState, "/joint_states", 10)
    try:
        yield node, buffer, publisher, process
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()


def publish_and_lookup(tf_robot, angles, *frames):
    node, buffer, publisher, process = tf_robot
    msg = JointState(name=list(angles), position=list(angles.values()))
    started = node.get_clock().now()
    deadline = time.monotonic() + 8
    found = {}
    while time.monotonic() < deadline:
        assert process.poll() is None, "robot_state_publisher exited"
        msg.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        for frame in frames:
            try:
                candidate = buffer.lookup_transform("base_footprint", frame, Time())
                if Time.from_msg(candidate.header.stamp) >= started:
                    found[frame] = candidate
            except TransformException:
                pass
        if len(found) == len(frames):
            return found[frames[0]] if len(frames) == 1 else found
    raise AssertionError(f"Missing TF for {frames}")


@pytest.mark.parametrize("m3,m4,m5,m6", [
    (150, 45, 0, 90), (150, 75, 0, 0), (200, 90, 35, 90),
    (240, 130, 155, 0), (90, 170, 0, 0),
])
def test_nominal_wrist_tf_matches_effective_chain(tf_robot, m3, m4, m5, m6):
    transform = publish_and_lookup(tf_robot, app_to_urdf(m3, m4, m5, m6), "arm_wrist_link")
    wrist = transform.transform.translation
    expected = planner.measured_point((m3, m4), (planner.CAD_L2, 0., 0.))
    assert (wrist.x, wrist.y, wrist.z) == pytest.approx(expected, abs=1e-8)


@pytest.mark.parametrize('m3,m4', [(170, 65), (200, 95), (240, 105)])
@pytest.mark.parametrize('m5', [0, 35, 155])
def test_measured_gripper_tf(tf_robot, m3, m4, m5):
    _, buffer, _, _ = tf_robot
    tr = publish_and_lookup(tf_robot, app_to_urdf(m3, m4, m5, 90), 'gripper_camera_optical_frame')
    expected, R = planner.camera_pose((m3, m4, m5))
    p = tr.transform.translation; actual = quat_matrix(tr.transform.rotation)
    assert (p.x, p.y, p.z) == pytest.approx(expected, abs=1e-8)
    assert actual == pytest.approx(R, abs=1e-8)
    cfg = planner._CALIBRATION
    camera = np.array([planner.CAD_L2, 0, 0]) + np.array(planner._CAD['camera_from_wrist_m'], float)
    marker = np.array([planner.CAD_L2, 0, 0]) + np.array(planner._CAD['marker_from_wrist_m'], float)
    jaw = np.array(cfg['effective_jaw_from_elbow_m'])
    assert jaw == pytest.approx(np.array(cfg['marker_effective_from_elbow_m']) + np.array(cfg['marker_rotation_link2']) @ cfg['jaw_region_in_marker_m'], abs=1e-12)
    assert cfg['jaw_nominal_wrist_offset_m'] == pytest.approx(jaw - np.array([.205, 0, 0]), abs=1e-12)
    roll = np.deg2rad(m5); c, s = np.cos(roll), np.sin(roll)
    Q = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    tcp = buffer.lookup_transform('base_footprint', 'gripper_tcp', Time()).transform.translation
    expected_jaw = planner.measured_point((m3, m4), Q @ jaw)
    assert (tcp.x, tcp.y, tcp.z) == pytest.approx(expected_jaw, abs=1e-8)
    optical = planner.hand.rpy(planner._CAD['camera_rpy']) @ np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    marker_rotation = planner.hand.rpy(planner._CAD['marker_rpy'])
    marker_in_optical = optical.T @ (marker - camera)
    actual_marker = np.array([p.x, p.y, p.z]) + actual @ marker_in_optical
    expected_marker = planner.measured_point((m3, m4), Q @ marker)
    assert actual_marker == pytest.approx(expected_marker, abs=1e-8)
    marker_tf = buffer.lookup_transform('gripper_camera_optical_frame', 'pincher_marker_68', Time()).transform
    assert (marker_tf.translation.x, marker_tf.translation.y, marker_tf.translation.z) == pytest.approx(marker_in_optical, abs=1e-8)
    assert quat_matrix(marker_tf.rotation) == pytest.approx(optical.T @ marker_rotation, abs=1e-8)
    a = np.deg2rad(m4 - m3 + 90); ca, sa = np.cos(a), np.sin(a)
    cy, sy = np.cos(planner.YAW), np.sin(planner.YAW)
    link2_rotation = np.array([[cy * ca, -sy, -cy * sa], [sy * ca, cy, -sy * sa], [sa, 0, ca]])
    assert actual @ optical.T @ marker_rotation == pytest.approx(link2_rotation @ Q @ marker_rotation, abs=1e-8)
