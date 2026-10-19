"""Wrist TF verification through robot_state_publisher."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
import xacro
import yaml

from z70_driver.driver import app_to_urdf


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


@pytest.mark.parametrize("m3,m4,m5,m6", [
    (150, 45, 0, 90), (150, 75, 0, 0), (200, 90, 35, 90),
    (240, 130, 155, 0), (90, 170, 0, 0),
])
def test_nominal_wrist_tf_matches_effective_chain(tf_robot, m3, m4, m5, m6):
    node, buffer, publisher, process = tf_robot
    angles = app_to_urdf(m3, m4, m5, m6)
    msg = JointState(name=list(angles), position=list(angles.values()))
    pose_started = node.get_clock().now()
    deadline = time.monotonic() + 8
    transform = None
    while time.monotonic() < deadline:
        assert process.poll() is None, "robot_state_publisher exited"
        msg.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            candidate = buffer.lookup_transform("base_footprint", "arm_wrist_link", Time())
            if Time.from_msg(candidate.header.stamp) >= pose_started:
                transform = candidate
                break
        except TransformException:
            pass
    assert transform is not None, "No TF received for this joint pose"
    wrist = transform.transform.translation
    from z70_manipulation import planner
    expected=planner.measured_point((m3,m4),(planner.CAD_L2,0.,0.))
    assert (wrist.x,wrist.y,wrist.z) == pytest.approx(expected,abs=1e-8)
