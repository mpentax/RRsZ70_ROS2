"""Offline integration and unit tests for the MuJoCo Z70 simulator and ROS 2 driver bridge."""
from __future__ import annotations

import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from z70_driver.driver import Driver
from z70_manipulation import perception, placement
from z70_manipulation.executor import Executor
from z70_manipulation.observations import Observations
from z70_mapping.decoder import decode_map
from z70_mapping.node import MappingNode
from z70_odometry.floor_vo import FloorCamera, FloorOdometry
from z70_sim import SimWorld, create_sim_server, encode_v1_map


def wait_for(predicate, seconds: float = 5.0) -> None:
    deadline = time.monotonic() + seconds
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert predicate(), 'condition deadline'


def test_mujoco_cameras_marker_68_floor_vo_and_v1_map():
    world = SimWorld()

    # 1. Front camera HSV object detection and floor projection
    jpeg_f, cam_f, wf, hf = world.render_jpeg('front')
    assert (cam_f, wf, hf) == ('front', 1600, 1200)
    img_f = cv2.imdecode(np.frombuffer(jpeg_f, np.uint8), cv2.IMREAD_COLOR)
    objs = perception.front_objects(img_f)
    colors = {o['color']: o for o in objs}
    assert {'yellow', 'purple', 'blue'} <= set(colors)
    assert colors['yellow']['position'][0] == pytest.approx(0.64, abs=0.05)
    assert colors['yellow']['position'][1] == pytest.approx(0.0, abs=0.02)

    # 2. Deploy arm, move to grasp pose (250, 95, 0), and verify ArUco ID68 & joint fit
    world.handle_rpc('app_pick_up_enter_exit', {'flag': 1})
    world.handle_rpc('app_arm_out_compartment', {})
    world.joints['m3'], world.joints['m4'], world.joints['m5'] = 250.0, 95.0, 0.0
    jpeg_arm, _, _, _ = world.render_jpeg('front')
    img_arm = cv2.imdecode(np.frombuffer(jpeg_arm, np.uint8), cv2.IMREAD_COLOR)
    jaw = perception.front_jaw(img_arm)
    assert jaw['marker_id'] == 68 and jaw['rms_px'] < 0.5
    fit = placement.observed_joints(jaw['hand_pose_base'])
    assert fit is not None
    assert fit['joints'] == pytest.approx([250.0, 95.0, 0.0], abs=1.0)

    # 3. Switch to gripper camera at WORKING_POS and verify 1600x896 rendering
    world.joints['m3'], world.joints['m4'], world.joints['m5'] = 150.0, 45.0, 0.0
    kind, _ = world.handle_rpc('app_change_camera', {'flag': 1})
    assert kind == 'timeout' and world.camera == 'gripper'
    jpeg_g, cam_g, wg, hg = world.render_jpeg()
    assert (cam_g, wg, hg) == ('gripper', 1600, 896)
    img_g = cv2.imdecode(np.frombuffer(jpeg_g, np.uint8), cv2.IMREAD_COLOR)
    assert any(b['color'] == 'yellow' for b in perception.detect(img_g))

    # 4. Floor visual odometry over textured MuJoCo floor
    world.handle_rpc('app_change_camera', {'flag': 0})
    world.handle_rpc('app_arm_in_compartment', {})
    jpeg_0, _, _, _ = world.render_jpeg('front')
    world.handle_rpc('app_rc_move', [{'velocity': 0.06, 'omega': 0.0, 'duration': 500, 'seqnum': 1}])
    jpeg_1, _, _, _ = world.render_jpeg('front')
    vo = FloorOdometry(FloorCamera.load(), camera_x_m=0.275)
    motion = vo.estimate(
        cv2.imdecode(np.frombuffer(jpeg_0, np.uint8), cv2.IMREAD_COLOR),
        cv2.imdecode(np.frombuffer(jpeg_1, np.uint8), cv2.IMREAD_COLOR),
    )
    assert motion is not None
    assert motion.dx == pytest.approx(0.06 * 0.65 * 0.5, abs=0.003)
    assert motion.inliers >= 200

    # 5. V1 map encoding and decoding
    raw = encode_v1_map(robot_pose=(0.25, -0.10, 0.5))
    decoded = decode_map(raw)
    assert (decoded.width, decoded.height) == (120, 120)
    assert sorted(decoded.rooms) == [1, 2]
    assert decoded.robot[:2] == pytest.approx((0.25, -0.10), abs=0.005)


def test_closed_loop_pick_against_mujoco_world(tmp_path: Path):
    world = SimWorld()

    class DirectSimClient:
        def __init__(self, w: SimWorld):
            self.w = w
            self.commands = None
            self.frame_received_at = 0.0
            self.frame_exposure_known = False

        def status(self):
            return self.w.status_dict()

        def arm_estimate(self):
            return {
                'state': 'WORKING_POS' if self.w.deployed else 'IN_ROBOT',
                'joints': dict(self.w.joints),
            }

        def frame(self):
            jpeg, cam, _, _ = self.w.render_jpeg()
            self.frame_received_at = time.time()
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            return img, cam

        def select(self, camera: str):
            self.w.camera = camera
            return {'camera': camera}

        def rpc(self, method: str, params, timeout_ms: int = 5000):
            kind, val = self.w.handle_rpc(method, params)
            if kind == 'ok':
                return {'dispatch': 'sent', 'response': 'received', 'result': val}
            return {'dispatch': 'unknown', 'response': 'transport_error', 'transportReason': 'timeout'}

    client = DirectSimClient(world)

    def observe(camera: str, color: str):
        img, tag = client.frame()
        blobs = perception.front_objects(img) if tag == 'front' else perception.detect(img)
        return [b for b in blobs if b['color'] == color]

    ex = Executor(
        client,
        observe,
        tmp_path / 'pick_run',
        sleep=lambda _: None,
        odom=lambda: {'x': world.pose[0], 'y': world.pose[1], 'yaw': world.pose[2]},
    )
    result = ex.pick('yellow')
    assert result['verified'] is True
    assert not world.deployed


def test_sim_server_and_ros_driver_integration(tmp_path: Path):
    token_file = tmp_path / 'service-token'
    token_file.write_text('sim-test-bearer-token\n')
    server = create_sim_server(token='sim-test-bearer-token')
    rclpy.init(domain_id=97)
    driver = Driver(
        [
            Parameter('origin', value=server.origin),
            Parameter('token_file', value=str(token_file)),
            Parameter('arm_estimate_path', value=''),
            Parameter('camera_hz', value=5.0),
            Parameter('state_hz', value=5.0),
            Parameter('map_refresh_s', value=60.0),
        ]
    )
    mapping = MappingNode()
    consumer = rclpy.create_node('sim_test_consumer')
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (driver, mapping, consumer):
        executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    obs = None
    try:
        obs = Observations(consumer, timeout=4.0)
        wait_for(lambda: 'state' in driver.samples and 'camera' in driver.samples and 'map' in driver.samples)
        status = obs.status()
        assert status['state'] == 3 and status['battery'] == 100
        img, tag = obs.frame()
        assert tag == 'front' and img.shape == (1200, 1600, 3)
        nav_snap = obs.navigation_snapshot(refresh=False)
        assert nav_snap.active_floor_verified and nav_snap.current_pose_verified
        assert nav_snap.labels == {1: 'Living Room', 2: 'Study'}

        # Deploy arm via driver_rpc and switch to gripper camera via SelectCamera
        driver.driver_rpc('app_pick_up_enter_exit', {'flag': 2}, 3000)
        driver.driver_rpc('app_arm_out_compartment', {}, 3000)
        obs.select('gripper')
        wait_for(lambda: server.world.camera == 'gripper')
    finally:
        if obs is not None:
            obs.close()
        driver.close()
        executor.shutdown()
        for node in (consumer, mapping, driver):
            node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=3.0)
        server.stop()
