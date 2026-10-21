"""ROS image/TF perception, observation/command transport, action servers, and CLI entry points."""
import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
import requests
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from unique_identifier_msgs.msg import UUID
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from z70_interfaces.action import Pick, Stack
from z70_interfaces.msg import ArmEstimate, CalibrationState, CommandOutcome, FrameMetadata, GarmentObservations, MapSnapshot, RobotState
from z70_interfaces.srv import RawRpc, RefreshMap, SelectCamera, SetArmAnchor, Stop, TaskSession
from z70_odometry.node import gains_from_state
from z70_driver.driver import LATCHED, MatchedFrames, frame_key, urdf_to_app
from z70_mapping.boundary import BoundaryConfig, SessionWorldMap
from z70_mapping.node import decode_map, quat_yaw
from . import perception, planner
from .executor import Executor, UncertainMotion, log_directory, read_arm, read_status
from .garment_delivery import TowingQualificationConfig
from .goals import GoalContext, GoalSpec, BUILTIN_GOALS
from .perception import stamp_seconds

RGB = {'yellow': (1., 1., 0.), 'orange': (1., .4, 0.), 'purple': (.7, 0., 1.), 'blue': (0., .2, 1.), 'green': (0., 1., .2)}


def call_service(client, request, timeout_s, cancelled=lambda: False,
                 timeout_exc=TimeoutError('ROS_DEADLINE_UNKNOWN_DISPATCH')):
    future = client.call_async(request)
    done = threading.Event(); future.add_done_callback(lambda _: done.set())
    deadline = time.monotonic() + timeout_s
    while not done.wait(.02):
        if cancelled():
            raise InterruptedError('CANCELLED')
        if time.monotonic() >= deadline:
            raise timeout_exc
    return future.result()


class Observations:
    def __init__(self, node=None, timeout=5.):
        self.owned = node is None
        self.initialized = False
        if self.owned and not rclpy.ok():
            rclpy.init(); self.initialized = True
        self.node = node or rclpy.create_node('z70_pick_observations')
        self.timeout = timeout
        self.cancelled = lambda: False
        self.condition = threading.Condition()
        self.state = None; self.arm = None; self.image = None; self.used = None
        self.map_snapshot = None; self.garments = None
        self.dynamic_grid = None; self.world = SessionWorldMap()
        self.frames = {}; self.metadata = {}
        self.group = ReentrantCallbackGroup()
        self.world_pub = self.node.create_publisher(String, '/z70/world/update', 10)
        self.subscriptions = [
            self.node.create_subscription(RobotState, '/z70/state', self.on_state, LATCHED, callback_group=self.group),
            self.node.create_subscription(ArmEstimate, '/z70/arm_estimate', self.on_arm, LATCHED, callback_group=self.group),
            self.node.create_subscription(GarmentObservations, '/z70/garments/observations', self.on_garments, LATCHED, callback_group=self.group),
            self.node.create_subscription(MapSnapshot, '/z70/driver/map_snapshot', self.on_map, LATCHED, callback_group=self.group),
            self.node.create_subscription(OccupancyGrid, '/z70/map/dynamic_obstacles', self.on_dynamic_grid, LATCHED, callback_group=self.group),
            self.node.create_subscription(String, '/z70/world/objects', self.on_world_objects, LATCHED, callback_group=self.group)]
        self.map_refresher = self.node.create_client(RefreshMap, '/z70/map/refresh', callback_group=self.group)
        for camera in ('front', 'gripper'):
            self.subscriptions.extend([
                self.node.create_subscription(CompressedImage, f'/camera/{camera}/image_raw/compressed',
                                              lambda m, c=camera: self.on_image(m, c), 2, callback_group=MutuallyExclusiveCallbackGroup()),
                self.node.create_subscription(FrameMetadata, f'/camera/{camera}/metadata',
                                              lambda m, c=camera: self.on_metadata(m, c), 2, callback_group=MutuallyExclusiveCallbackGroup())])
        self.selector = self.node.create_client(SelectCamera, '/z70/camera/select', callback_group=self.group)
        if self.owned:
            self.executor = SingleThreadedExecutor(); self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin, name='pickup_observations')
            self.thread.start()

    @staticmethod
    def evidence_id(evidence):
        return (bytes(evidence.session_id.uuid), evidence.sequence)

    def on_garments(self, msg):
        with self.condition:
            self.garments = msg
            if self.map_snapshot is not None and getattr(msg, 'model_available', False) and getattr(msg, 'geofence_active', False):
                try:
                    m = decode_map(bytes(self.map_snapshot.raw))
                    self.world.bind_session(bytes(self.map_snapshot.evidence.session_id.uuid), self.map_snapshot.map_index, m.origin, m.width, m.height, m.resolution)
                    self.world.ingest_observations(msg, m.robot)
                except Exception:
                    pass
            self.condition.notify_all()

    def on_dynamic_grid(self, msg):
        with self.condition:
            self.dynamic_grid = msg

    def on_world_objects(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.condition:
            self.world.load_entries(payload)

    def dynamic_obstacles(self, pose_xy=None):
        with self.condition:
            if self.world.width > 0 and self.world.height > 0:
                return self.world.obstacle_mask(protect_xy=pose_xy)
            if self.dynamic_grid is not None:
                return deepcopy(self.dynamic_grid)
            return None

    def world_map(self) -> SessionWorldMap:
        return self.world

    def record_world_event(self, action: str, **fields):
        payload = dict(action=action, **fields)
        with self.condition:
            self.world.apply_event(payload)
        try:
            self.world_pub.publish(String(data=json.dumps(payload)))
        except Exception:
            pass

    def garment_barrier(self):
        with self.condition:
            meta = self.metadata.get('front')
            return dict(garment=self.evidence_id(self.garments.evidence) if self.garments else None,
                        image=self.evidence_id(meta.evidence) if meta else None)

    def garment_snapshot(self, after=None, max_age_s=6.):
        prior = after.get('garment') if isinstance(after, dict) else after
        def ready():
            m = self.garments
            return (m is not None and self.fresh(m.evidence, max_age_s)
                    and (prior is None or self.evidence_id(m.evidence) != prior))
        self.wait(ready)
        with self.condition:
            message = deepcopy(self.garments)
        if not message.model_available:
            raise LookupError('GARMENT_MODEL_UNAVAILABLE')
        return message

    def garment_frame(self, after=None):
        prior = after.get('image') if isinstance(after, dict) else after
        def ready():
            meta = self.metadata.get('front'); frame = self.frames.get('front')
            return (meta is not None and frame is not None and self.key(meta) == self.key(frame)
                    and self.fresh(meta.evidence, 6.) and self.evidence_id(meta.evidence) != prior
                    and self.evidence_id(meta.evidence) != self.used)
        self.wait(ready)
        with self.condition:
            frame = deepcopy(self.frames['front']); meta = deepcopy(self.metadata['front'])
            self.used = self.evidence_id(meta.evidence)
        image = cv2.imdecode(np.frombuffer(frame.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise LookupError('invalid garment evidence frame')
        visible = (image.shape[:2] == (meta.height, meta.width) and float(image.mean()) > 8.
                   and float(np.percentile(image, 95) - np.percentile(image, 5)) > 12.)
        return dict(image=image, source_id=(bytes(meta.evidence.session_id.uuid).hex(), meta.evidence.sequence),
                    evidence=meta.evidence, coverage_valid=visible)

    def on_map(self, msg):
        with self.condition:
            self.map_snapshot = msg; self.condition.notify_all()

    def _call_service(self, client, request, timeout_s, timeout_exc):
        return call_service(client, request, timeout_s, self.cancelled, timeout_exc)

    def navigation_snapshot(self, refresh=False, max_age_s=5.):
        expected = None
        if refresh:
            if not self.map_refresher.wait_for_service(timeout_sec=1.):
                raise LookupError('MAP_REFRESH_UNAVAILABLE')
            response = self._call_service(self.map_refresher, RefreshMap.Request(), 35., TimeoutError('MAP_REFRESH_TIMEOUT'))
            if response.outcome.error_code != 'NONE' or not response.snapshot.valid or response.snapshot.stale:
                raise LookupError('MAP_REFRESH_FAILED')
            expected = response.snapshot
        def ready():
            m = self.map_snapshot
            return (m is not None and m.evidence.valid and not m.evidence.stale
                    and (refresh or max_age_s is None or self.fresh(m.evidence, max_age_s)
                         or (m.header.stamp.sec > 0 and 0 <= time.time() - stamp_seconds(m.header.stamp) <= max_age_s))
                    and (expected is None or (m.evidence.session_id == expected.session_id
                         and m.evidence.sequence >= expected.sequence)))
        self.wait(ready)
        with self.condition:
            m = deepcopy(self.map_snapshot)
        raw_age = time.time() - stamp_seconds(m.evidence.received_at)
        hdr_age = time.time() - stamp_seconds(m.header.stamp) if m.header.stamp.sec > 0 else raw_age
        cache_age_s = 0. if refresh else min(raw_age, hdr_age) if hdr_age >= 0 else raw_age
        decoded = decode_map(bytes(m.raw))
        with self.condition:
            self.world.bind_session(bytes(m.evidence.session_id.uuid), m.map_index, decoded.origin, decoded.width, decoded.height, decoded.resolution)
        return SimpleNamespace(
            map=decoded, labels={r.segment_id: r.name for r in m.rooms},
            sequence=m.evidence.sequence, map_index=m.map_index, map_sequence=m.map_sequence,
            session_id=bytes(m.evidence.session_id.uuid),
            active_floor_verified=m.active_floor_verified, current_pose_verified=m.current_pose_verified,
            cache_age_s=cache_age_s, evidence=m.evidence,
            dynamic_obstacles=self.dynamic_obstacles(decoded.robot[:2] if decoded.robot else None),
            world=self.world,
        )

    def on_state(self, msg):
        with self.condition:
            self.state = msg; self.condition.notify_all()

    def on_arm(self, msg):
        with self.condition:
            self.arm = msg; self.condition.notify_all()

    key = staticmethod(frame_key)

    def match_frame(self, camera):
        frame, meta = self.frames.get(camera), self.metadata.get(camera)
        if frame is not None and meta is not None and self.key(frame) == self.key(meta):
            self.image = (frame, meta, camera)
            self.condition.notify_all()

    def on_image(self, msg, camera):
        with self.condition:
            self.frames[camera] = msg; self.match_frame(camera)

    def on_metadata(self, msg, camera):
        with self.condition:
            self.metadata[camera] = msg; self.match_frame(camera)

    @staticmethod
    def fresh(evidence, max_age):
        return (evidence.valid and not evidence.stale
                and 0 <= time.time() - stamp_seconds(evidence.received_at) <= max_age)

    def wait(self, predicate):
        deadline = time.monotonic() + self.timeout
        with self.condition:
            while not predicate():
                if self.cancelled():
                    raise InterruptedError('CANCELLED')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise LookupError('shared driver observation unavailable or stale')
                self.condition.wait(min(remaining, .05))

    def status(self) -> dict:
        self.wait(lambda: self.state is not None and self.fresh(self.state.evidence, 15.))
        with self.condition:
            m = deepcopy(self.state)
        if not m.connected:
            raise LookupError('driver disconnected')
        status = {}
        for field, key in [('robot_state', 'state'), ('error_code', 'error_code'), ('common_status', 'common_status'), ('grab_status', 'grab_status')]:
            if getattr(m, field + '_known'):
                status[key] = getattr(m, field)
        if m.battery_known:
            status['battery'] = m.battery_fraction * 100
        if m.cleaning_known:
            status['in_cleaning'] = m.in_cleaning
        if m.returning_known:
            status['in_returning'] = m.in_returning
        return status

    def arm_estimate(self) -> dict:
        self.wait(lambda: self.arm is not None and self.arm.evidence.valid and not self.arm.evidence.stale
                  and self.state is not None and self.fresh(self.state.evidence, 15.))
        with self.condition:
            m = deepcopy(self.arm)
        usable = m.validity.value in (1, 2) and not m.restored
        j = urdf_to_app(m.joints.name, m.joints.position) if usable else {}
        usable = bool(j)
        state = 'IN_PROCESS_POS' if usable else 'IN_ROBOT' if m.validity.value == 3 and m.validity.reason == 'COMMAND_DERIVED_STOW' else 'UNKNOWN'
        return {'state': state, 'joints': j, 'source': 'ROS_COMMAND_ESTIMATE',
                'updatedAt': stamp_seconds(m.evidence.source_at) * 1000, 'stepsSinceAnchor': m.steps_since_anchor}

    def get(self, path):
        if path == '/api/state':
            return {'telemetry': {'status': {'value': self.status()}}}
        if path == '/api/arm/estimate':
            return self.arm_estimate()
        raise ValueError('unsupported observation path')

    def frame(self):
        def available():
            if self.image is None:
                return False
            _, meta, _ = self.image
            return self.evidence_id(meta.evidence) != self.used and self.fresh(meta.evidence, 5.)
        self.wait(available)
        with self.condition:
            frame, meta, camera = self.image
            self.used = self.evidence_id(meta.evidence)
        image = cv2.imdecode(np.frombuffer(frame.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise LookupError('invalid shared frame')
        return image, camera

    def select(self, camera):
        if camera not in ('front', 'gripper'):
            raise ValueError('unsupported camera')
        if not self.selector.wait_for_service(timeout_sec=2.):
            raise LookupError('driver selector unavailable')
        response = self._call_service(self.selector, SelectCamera.Request(camera=1 if camera == 'front' else 2),
                                      20., LookupError('CAMERA_SELECT_TIMEOUT'))
        if not response.state.selection_confirmed or response.outcome.error_code not in ('NONE', 'TIMEOUT'):
            raise LookupError(response.outcome.error_code or 'selection unconfirmed')
        with self.condition:
            self.image = None; self.frames.clear(); self.metadata.clear()
        return {'camera': camera}

    def close(self):
        if self.owned:
            self.executor.shutdown(); self.thread.join(timeout=5)
            self.node.destroy_node()
            if self.initialized:
                rclpy.try_shutdown()
        else:
            for sub in self.subscriptions:
                self.node.destroy_subscription(sub)
            self.node.destroy_publisher(self.world_pub)
            self.node.destroy_client(self.selector)
            self.node.destroy_client(self.map_refresher)


class _Transport:
    """Node-owned entities outlive individual tasks and the executor wait set."""
    def __init__(self, observations):
        self.node = observations.node
        self.group = observations.group
        self.clients = {kind: self.node.create_client(kind, name, callback_group=self.group)
                        for kind, name in ((RawRpc, '/z70/driver/raw_rpc'), (Stop, '/z70/stop'),
                                           (TaskSession, '/z70/tasks/session'), (SetArmAnchor, '/z70/arm/anchor'))}
        self.active: 'Commands | None' = None
        self.subscription = self.node.create_subscription(UUID, '/z70/tasks/cancel', self.revoked, 10, callback_group=self.group)

    def revoked(self, token):
        active = self.active
        if active is not None:
            active.revoked(token)


class Commands:
    def __init__(self, observations):
        self.node = observations.node
        transport = getattr(observations, '_command_transport', None)
        if transport is None:
            transport = _Transport(observations)
            observations._command_transport = transport
        self.transport = transport
        self.clients = transport.clients
        self.task_id = UUID()
        self.cancelled = threading.Event()
        self.latest_outcome = None
        self.stop_lock = threading.Lock(); self.stop_future = None; self.driver_revoked = False
        transport.active = self

    def revoked(self, token):
        if any(self.task_id.uuid) and token == self.task_id:
            self.driver_revoked = True; self.cancelled.set()

    def call(self, kind, request, timeout, cancellable=True):
        client = self.clients[kind]
        if not client.wait_for_service(timeout_sec=1.):
            raise LookupError('ROS_DRIVER_UNAVAILABLE')
        return call_service(client, request, max(timeout, 10.), lambda: cancellable and self.cancelled.is_set())

    def begin(self):
        response = self.call(TaskSession, TaskSession.Request(begin=True), 3.)
        if not response.accepted:
            raise LookupError(response.reason)
        self.task_id = response.task_id

    def end(self):
        if any(self.task_id.uuid):
            try:
                self.call(TaskSession, TaskSession.Request(begin=False, task_id=self.task_id), 16., False)
            finally:
                self.task_id = UUID()

    def set_arm_anchor(self, anchor: str, reason: str = '') -> dict:
        codes = {'unknown': 0, 'stowed': 1, 'deployed': 2}
        if anchor not in codes:
            raise ValueError("anchor must be 'unknown', 'stowed', or 'deployed'")
        reply = self.call(SetArmAnchor, SetArmAnchor.Request(anchor=codes[anchor], observation_reason=str(reason)[:64]), 3.)
        est = getattr(reply, 'estimate', None)
        expected = {'unknown': 0, 'stowed': 3, 'deployed': 1}[anchor]
        accepted = bool(getattr(reply, 'accepted', est is not None and est.validity.value == expected))
        return {'accepted': accepted, 'anchor': anchor}

    def rpc(self, method, params, timeout_ms):
        if self.cancelled.is_set():
            raise InterruptedError('CANCELLED')
        response = self.call(RawRpc, RawRpc.Request(
            method=method, params_json=json.dumps(params, allow_nan=False),
            timeout_ms=timeout_ms, task_id=self.task_id,
        ), timeout_ms / 1000 + 1)
        self.latest_outcome = o = response.outcome
        if o.error_code in ('BUSY', 'CANCELLED', 'STOP_UNVERIFIED'):
            if any(self.task_id.uuid):
                self.driver_revoked = True
            self.cancelled.set(); raise InterruptedError(o.error_code)
        return dict(
            dispatch={0: 'not_sent', 1: 'sent', 2: 'unknown'}[o.dispatch],
            response={0: 'none', 1: 'received', 2: 'vendor_error', 3: 'transport_error'}[o.response],
            transportReason='timeout' if o.error_code in ('TIMEOUT', 'DEADLINE') else o.error_code,
            vendorCode=o.vendor_code if o.vendor_code_known else None,
            result=json.loads(response.result_json) if response.result_json else None,
        )

    def stop(self):
        self.cancelled.set()
        with self.stop_lock:
            if self.driver_revoked and self.stop_future is None:
                return None
            if self.stop_future is None:
                client = self.clients[Stop]
                if not client.wait_for_service(timeout_sec=1.):
                    raise LookupError('PHYSICAL_STOP_REQUIRED')
                self.stop_future = client.call_async(Stop.Request(scope=3))
            future = self.stop_future
        done = threading.Event(); future.add_done_callback(lambda _: done.set())
        if not done.wait(15.5):
            raise TimeoutError('PHYSICAL_STOP_REQUIRED')
        return future.result()

    def close(self):
        if self.transport.active is self:
            self.transport.active = None


class Client:
    def __init__(self, origin='', token_file='', read_only=False, observations=None):
        self.origin = origin.rstrip('/')
        self.read_only = read_only
        self.observations = observations if observations is not None else Observations()
        self.owns_observations = observations is None
        self.commands = None if read_only else Commands(self.observations)

    def get(self, path):
        return self.observations.get(path)

    def status(self) -> dict:
        return read_status(self.observations)

    def arm_estimate(self) -> dict:
        return read_arm(self.observations)

    def frame(self):
        obs = self.observations
        if not hasattr(obs, 'condition'):
            return obs.frame()
        with obs.condition:
            result = obs.frame()
            _, meta, _ = obs.image
            self.frame_received_at = stamp_seconds(meta.evidence.received_at)
            self.frame_exposure_known = meta.exposure_time_known
            return result

    def select(self, camera):
        if self.read_only:
            raise PermissionError('dry-run forbids camera selection')
        return self.observations.select(camera)

    def rpc(self, method, params, timeout_ms=5000):
        if self.read_only:
            raise PermissionError('dry-run forbids all RPCs')
        if method.startswith('get_') or method == 'app_get_arm_joints_data':
            raise ValueError('observations belong to the shared ROS driver')
        d = self.commands.rpc(method, params, timeout_ms)
        if d.get('response') == 'received':
            return d
        if method in ('app_grip', 'app_open_gripper') and d.get('dispatch') in ('sent', 'unknown') and d.get('transportReason') == 'timeout':
            return d
        if d.get('dispatch') == 'unknown':
            raise UncertainMotion('dispatch unknown: ' + method)
        if d.get('response') == 'vendor_error':
            raise RuntimeError(f'robot rejected {method} (vendor code {d.get("vendorCode")}): command format or mode problem, not a safety event')
        raise requests.RequestException('RPC transport failure: ' + method)

    def close(self):
        if self.commands:
            self.commands.close()
        if self.owns_observations:
            self.observations.close()


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
        self.pub_json = self.create_publisher(String, '/z70/objects/json', LATCHED)
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
            try:
                server.destroy()
            except ValueError:
                pass
        self.action_servers.clear()
        return super().destroy_node()

    def _run_goal(self, handle, spec: GoalSpec):
        self.active_handle = handle
        self.navigation_active = spec.navigation
        client = None
        ctx = None
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
                if ctx.trace:
                    ctx.log('goal_trace', goal=spec.name, steps=len(ctx.trace), trace=ctx.trace)
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
            if ctx and ctx.trace:
                try:
                    ctx.log('goal_trace', goal=spec.name, steps=len(ctx.trace), trace=ctx.trace, error=str(exc))
                except Exception:
                    pass
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


def _spin_node(node_type):
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
    _spin_node(Perception)


def server_main():
    _spin_node(PickServer)


COLORS = ['yellow', 'purple', 'blue', 'green', 'orange']


def _common_parser(description: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=description)
    p.add_argument('--dry-run', action='store_true')
    p.add_argument('--origin', default='http://127.0.0.1:57010')
    p.add_argument('--repo', type=Path, default=Path(os.environ.get('Z70_REPO', Path.cwd())))
    p.add_argument('--token-file', type=Path)
    p.add_argument('--log-dir', type=Path)
    return p


def _run_dry(args, execute_fn) -> dict:
    directory = args.log_dir or log_directory(args.repo)
    client = Client(args.origin, args.token_file or args.repo / '.local/service-token', read_only=True)
    cached_objects = None

    def observe(camera, color):
        nonlocal cached_objects
        if cached_objects is None:
            image, tag = client.frame()
            if tag != 'front':
                raise LookupError('dry-run requires an already-selected front camera; no selection was sent')
            directory.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(directory / 'front.jpg'), image)
            cached_objects = perception.front_objects(image)
        return [b for b in cached_objects if color is None or b.get('color') == color]

    executor = Executor(client, observe, directory)
    try:
        report = execute_fn(executor)
        (directory / 'dry-run.json').write_text(json.dumps(report, indent=2) + '\n')
        return report
    finally:
        client.close()


def _run_action(p, args, node_name, action_cls, topic, goal, reject_msg, format_result) -> dict:
    if args.origin != p.get_default('origin') or args.token_file or args.log_dir:
        p.error('connection/log options configure --dry-run only; configure the ROS pick server for live execution')
    rclpy.init()
    node = rclpy.create_node(node_name)
    action = ActionClient(node, action_cls, topic)
    handle = None
    try:
        if not action.wait_for_server(timeout_sec=10):
            raise RuntimeError(f'{topic} server unavailable; launch bringup manipulation:=true')
        future = action.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(reject_msg)
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, future)
        return format_result(future.result().result)
    except KeyboardInterrupt:
        if handle:
            future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        raise
    finally:
        action.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


def run(argv=None):
    p = _common_parser('pixi run pick COLOR [--dry-run]')
    p.add_argument('color', choices=COLORS)
    args = p.parse_args(argv)
    if args.dry_run:
        return _run_dry(args, lambda ex: ex.pick(args.color, dry_run=True))
    return _run_action(
        p, args, 'z70_pick_client', Pick, '/z70/pick',
        Pick.Goal(color=args.color, dry_run=False),
        'pickup goal rejected (another pickup active or unknown colour)',
        lambda r: dict(
            verified=r.verified_hold, dry_run_valid=r.dry_run_valid,
            success=r.verified_hold or r.dry_run_valid, evidence_uri=r.evidence_uri,
            error_code=r.outcome.error_code,
        ),
    )


def _cli_main(fn, dry_key: str):
    try:
        report = fn()
        print(json.dumps(report, indent=2))
        if (report.get('dry_run') and not report.get(dry_key)) or report.get('success') is False:
            raise SystemExit(1)
    except (RuntimeError, LookupError, ValueError) as e:
        raise SystemExit(str(e))


def main():
    _cli_main(run, 'target')


def run_stack(argv=None):
    p = _common_parser('pixi run stack BASE_COLOR MIDDLE_COLOR TOP_COLOR [--dry-run]')
    for name in ('base_color', 'middle_color', 'top_color'):
        p.add_argument(name, choices=COLORS)
    args = p.parse_args(argv)
    if len({args.base_color, args.middle_color, args.top_color}) != 3:
        p.error('stack requires 3 distinct colors')
    if args.dry_run:
        return _run_dry(args, lambda ex: ex.stack(args.base_color, args.middle_color, args.top_color, dry_run=True))
    return _run_action(
        p, args, 'z70_stack_client', Stack, '/z70/stack',
        Stack.Goal(base_color=args.base_color, middle_color=args.middle_color, top_color=args.top_color, dry_run=False),
        'stack goal rejected (another manipulation task active or invalid colors)',
        lambda r: dict(
            tiers_completed=int(r.tiers_completed), stack_verified=bool(r.stack_verified),
            dry_run_valid=bool(r.dry_run_valid), success=bool(r.stack_verified or r.dry_run_valid),
            evidence_uri=r.evidence_uri, error_code=r.outcome.error_code,
        ),
    )


def stack_main():
    _cli_main(run_stack, 'all_targets_visible')
