"""ROS image/TF perception and single-owner manipulation/navigation action servers."""
from copy import deepcopy
import json
import os
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from z70_interfaces.msg import ArmEstimate, CalibrationState, CommandOutcome
from z70_odometry.calibration_state import gains_from_state
from z70_driver.frame_subscription import MatchedFrames
from z70_mapping.boundary import BoundaryConfig
from z70_mapping.geometry import quat_yaw
from . import perception, planner
from .commands import Commands
from .executor import Client, log_directory
from .garment_delivery import TowingQualificationConfig
from .goals import GoalContext, GoalSpec, BUILTIN_GOALS
from .observations import Observations, LATCHED

RGB = {'yellow': (1., 1., 0.), 'orange': (1., .4, 0.), 'purple': (.7, 0., 1.), 'blue': (0., .2, 1.), 'green': (0., 1., .2)}


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Perception(Node):
    def __init__(self, name='z70_perception', parameter_overrides=None, subscribe=True):
        super().__init__(name, parameter_overrides=parameter_overrides or [])
        self.repo = Path(self.declare_parameter('repo', str(Path(os.environ.get('Z70_REPO', Path.cwd())))).value)
        config = self.declare_parameter('colors_file', '').value
        self.thresholds = yaml.safe_load(Path(config).read_text()) if config else perception.colors()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.arm_trusted = False
        self.create_subscription(ArmEstimate, '/z70/arm_estimate', self.arm, LATCHED)
        self.pub_json = self.create_publisher(String, '/z70/objects/json', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/z70/objects', 10)
        self.K = planner.GRIPPER_MATRIX.copy()
        self.D = planner.GRIPPER_D.copy()
        self.create_subscription(CameraInfo, '/camera/gripper/camera_info', self.info, 10)
        if subscribe:
            self.frames = [MatchedFrames(self, camera, lambda msg, c=camera: self.image(msg, c), compressed=True)
                           for camera in ('front', 'gripper')]

    def arm(self, msg):
        self.arm_trusted = msg.evidence.valid and not msg.evidence.stale and msg.validity.value in (1, 2) and not msg.restored

    def info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)

    def project(self, image, camera, stamp=None):
        if camera == 'front':
            objects = perception.front_objects(image, thresholds=self.thresholds)
        else:
            objects = perception.detect(image, self.thresholds)
            if self.arm_trusted:
                tr = self.tf.lookup_transform('base_footprint', 'gripper_camera_optical_frame', stamp or Time(), timeout=Duration(seconds=.5))
                p = tr.transform.translation
                C = np.array([p.x, p.y, p.z])
                R = rotation(tr.transform.rotation)
                K = self.K.copy()
                K[0, :] *= image.shape[1] / 1600
                K[1, :] *= image.shape[0] / 896
                for b in objects:
                    point = perception.ray_floor(b['contact_px'], K, self.D, C, R)
                    if point:
                        b['position'] = point
                        b['size_m'] = .05
                        b['camera_distance_m'] = float(np.linalg.norm(np.array(point) - C))
                        b['camera_height_m'] = float(C[2])
            for b in objects:
                b['camera'] = 'gripper'
        self.publish(objects, camera, stamp)
        return objects

    def publish(self, objects, camera, stamp=None):
        now = self.get_clock().now().to_msg()
        self.pub_json.publish(String(data=json.dumps(dict(camera=camera, frame_id='base_footprint', stamp_ns=self.get_clock().now().nanoseconds, objects=objects))))
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = 'base_footprint'
        clear.header.stamp = now
        markers = [clear]
        for index, b in enumerate(objects):
            if 'position' not in b:
                continue
            m = Marker()
            m.header.frame_id = 'base_footprint'
            m.header.stamp = now
            m.ns = camera
            m.id = index
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, b['position'])
            m.pose.position.z += b.get('size_m', .05) / 2
            m.pose.orientation.w = 1.
            m.scale.x = m.scale.y = m.scale.z = float(b.get('size_m', .05))
            m.color.r, m.color.g, m.color.b = RGB[b['color']]
            m.color.a = .9
            m.text = b['color']
            markers.append(m)
        self.pub_markers.publish(MarkerArray(markers=markers))

    def image(self, msg, camera):
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        try:
            self.project(image, camera, Time.from_msg(msg.header.stamp))
        except TransformException:
            self.get_logger().warning('camera TF unavailable; next observation will retry', throttle_duration_sec=5)


class PickServer(Perception):
    def __init__(self, parameter_overrides=None):
        super().__init__('z70_pick_server', parameter_overrides, subscribe=False)
        self.log_root = self.declare_parameter('log_root', '').value
        self.safety_stop = False
        self.odom_value = None
        self.odom_at = 0.
        self.lock = threading.Lock()
        self.active_handle = None
        self.navigation_active = False
        self.client = None
        self.gains = {}
        self.deadline = None
        self.create_subscription(Bool, '/z70/safety_stop', self.stop_signal, 10)
        self.create_subscription(Odometry, '/odom', self.odometry, 30)
        self.group = ReentrantCallbackGroup()
        self.observations = Observations(self)
        if hasattr(self.observations, 'node'):
            Commands(self.observations)
        self.observations.cancelled = lambda: self.client is not None and self.client.commands is not None and self.client.commands.cancelled.is_set()
        self.create_subscription(CalibrationState, '/z70/chassis_calibration', self.calibration, LATCHED)
        self.create_timer(.05, self.watchdog, callback_group=self.group)
        self.navigation_config = BoundaryConfig(allowed_segments=tuple(
            self.declare_parameter('pin_and_go_allowed_segments', [1, 2, 3, 4, 5, 6]).value))
        towing_qual = bool(self.declare_parameter('towing_qualified', False).value)
        towing_cats = tuple(self.declare_parameter('towing_qualified_categories', ['towel', 'shirt', 'pants', 'jeans', 'hoodie']).value)
        towing_uri = str(self.declare_parameter('towing_artifact_uri', '').value)
        self.towing_config = TowingQualificationConfig(
            towing_qualified=towing_qual, qualified_categories=towing_cats, artifact_uri=towing_uri)
        self.patrol_directory = Path(self.declare_parameter('patrol_ledger_directory', str(self.repo / '.local/patrol')).value)

        self.action_servers = []
        self.goal_specs = {}
        for spec in BUILTIN_GOALS.values():
            self.register_goal(spec)

    def register_goal(self, spec: GoalSpec) -> ActionServer:
        server = ActionServer(
            self,
            spec.action_cls,
            spec.topic,
            execute_callback=lambda handle, sp=spec: self._run_goal(handle, sp),
            goal_callback=lambda req, sp=spec: GoalResponse.ACCEPT if sp.validate(self, req) and self.lock.acquire(blocking=False) else GoalResponse.REJECT,
            cancel_callback=self.cancel,
            callback_group=self.group,
        )
        self.action_servers.append(server)
        self.goal_specs[spec.name] = spec
        return server

    def destroy_node(self):
        for server in self.action_servers:
            server.destroy()
        self.action_servers.clear()
        return super().destroy_node()

    def _run_goal(self, handle, spec: GoalSpec):
        self.active_handle = handle
        self.navigation_active = spec.navigation
        client = None
        try:
            req = handle.request
            client = Client(read_only=spec.read_only(req), observations=self.observations)
            self.client = client
            timeout = spec.timeout_s(req) if callable(spec.timeout_s) else float(spec.timeout_s)
            self.deadline = time.monotonic() + timeout
            base_dir = log_directory(self.repo)
            directory = (Path(self.log_root) if spec.navigation else Path(self.log_root) / base_dir.name) if self.log_root else base_dir
            ctx = GoalContext(self, handle, spec.action_cls, client, directory, create_executor=spec.needs_executor(req))
            try:
                if client.commands:
                    client.commands.begin()
                success, result = spec.run(ctx, req)
                if handle.is_cancel_requested:
                    handle.canceled()
                elif success:
                    handle.succeed()
                else:
                    handle.abort()
                return result
            finally:
                if client.commands:
                    try:
                        client.commands.end()
                    except (LookupError, TimeoutError, InterruptedError):
                        pass
                client.close()
        except Exception as exc:
            self.get_logger().warning(f'{spec.name} failed: {type(exc).__name__}: {exc}')
            if spec.navigation and client and client.commands:
                try:
                    client.commands.stop()
                except (LookupError, TimeoutError):
                    pass
            if handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
            res = spec.action_cls.Result()
            if hasattr(res, 'outcome'):
                res.outcome = CommandOutcome(error_code='OBSERVATION_GAP')
            if hasattr(res, 'status_code'):
                res.status_code = 'OBSERVATION_GAP'
            if hasattr(res, 'detail'):
                res.detail = 'Shared driver observation or command unavailable'
            return res
        finally:
            self.navigation_active = False
            self.active_handle = None
            self.client = None
            self.deadline = None
            self.lock.release()

    # --- Node Helpers & Subscriptions ---

    def calibration(self, msg):
        self.gains = gains_from_state(msg)

    def _revoke_commands(self):
        if self.client and self.client.commands and not self.client.commands.cancelled.is_set():
            self.client.commands.cancelled.set()
            threading.Thread(target=self.client.commands.stop, daemon=True).start()

    def watchdog(self):
        expired = self.deadline is not None and time.monotonic() >= self.deadline
        navigation_hazard = self.navigation_active and self.safety_stop
        if expired or navigation_hazard:
            self._revoke_commands()

    def cancel(self, handle):
        if self.active_handle is not None and handle != self.active_handle:
            return CancelResponse.REJECT
        self._revoke_commands()
        return CancelResponse.ACCEPT

    def stop_signal(self, msg):
        self.safety_stop = msg.data

    def odometry(self, msg):
        p = msg.pose.pose
        if msg.pose.covariance[0] > 1000:
            return
        self.odom_value = dict(x=p.position.x, y=p.position.y, yaw=quat_yaw(p.orientation))
        self.odom_at = time.monotonic()

    def odom(self):
        return self.odom_value if time.monotonic() - self.odom_at < 2. else None

    def hazard(self):
        if self.safety_stop:
            return 'person/pet/obstacle or operator stop signal'
        if self.active_handle and self.active_handle.is_cancel_requested:
            return 'operator cancelled task'
        if self.client and self.client.commands and self.client.commands.cancelled.is_set():
            return 'task revoked'
        if self.deadline is not None and time.monotonic() >= self.deadline:
            return 'task deadline'
        return None

    def _current_arm_msg(self):
        with getattr(self.observations, 'condition', threading.Lock()):
            arm = getattr(self.observations, 'arm', None)
            return deepcopy(arm) if arm is not None else ArmEstimate()

    def map_pose(self, source='base_footprint'):
        tr = self.tf.lookup_transform('map', source, Time(), timeout=Duration(seconds=.5))
        age = (self.get_clock().now().nanoseconds - Time.from_msg(tr.header.stamp).nanoseconds) / 1e9
        if not 0 <= age <= 2.:
            raise LookupError('stale map -> odom -> base TF')
        p = tr.transform.translation
        return (p.x, p.y, quat_yaw(tr.transform.rotation))


def run(node_type):
    rclpy.init()
    node = node_type()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def perception_main():
    run(Perception)


def server_main():
    run(PickServer)
