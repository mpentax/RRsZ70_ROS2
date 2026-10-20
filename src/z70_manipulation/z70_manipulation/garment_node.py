"""Front-camera garment perception ROS 2 node."""
from copy import deepcopy
import json
import math
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from z70_interfaces.msg import GarmentObservation, GarmentObservations
from z70_driver.frame_subscription import MatchedFrames, read_pipe_exact, stop_worker_process, worker_env
from .perception import front_camera, calibration
from .garment_geometry import observe, geofence, GarmentTracker
from .garment_worker import decode_result, MAX_PACKET, BACKENDS

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


def observations_message(items, meta, quality, geofence_active):
    msg = GarmentObservations()
    msg.header = deepcopy(meta.header); msg.header.frame_id = 'base_footprint'
    msg.evidence = deepcopy(meta.evidence)
    msg.model_id = str(quality.get('model_id', quality.get('backend', 'unavailable')))[:64]
    msg.model_available = bool(quality.get('model_available', False))
    msg.geofence_active = bool(geofence_active)
    scalar_fields = ('category', 'confidence', 'bbox_px', 'contact_px', 'grasp_px', 'contact_valid',
                     'grasp_valid', 'footprint_width_m', 'footprint_depth_m', 'position_uncertainty_m',
                     'inside_geofence', 'geofence_verified', 'reachable_for_grasp', 'transport_suitability',
                     'suitability_reason', 'calibration_id')
    for item in items:
        obs = GarmentObservation(); obs.header = deepcopy(msg.header); obs.evidence = deepcopy(msg.evidence)
        obs.track_id.uuid = list(uuid.UUID(item['track_id']).bytes)
        for field in scalar_fields:
            setattr(obs, field, item[field])
        for field in ('floor_contact_m', 'grasp_point_m'):
            x, y, z = map(float, item[field])
            setattr(obs, field, Point(x=x, y=y, z=z))
        msg.garments.append(obs)
    return msg


class GarmentNode(Node):
    def __init__(self):
        super().__init__('z70_garment_perception')
        python = self.declare_parameter('garment_python', str(Path.home()/'.venvs/roboclaw-depth/bin/python')).value
        backend = self.declare_parameter('backend', 'grounding_dino_clipseg').value
        device = self.declare_parameter('device', 'mps').value
        hz = float(self.declare_parameter('garment_hz', .5).value)
        self.max_age = float(self.declare_parameter('max_frame_age_s', 5.).value)
        qual_small = bool(self.declare_parameter('qualified_small_lift', backend == 'sim_hsv').value)
        allow_tow = bool(self.declare_parameter('allow_tow', backend == 'sim_hsv').value)
        self.qualified_policy = dict(qualified_small_lift=qual_small, allow_tow=allow_tow) if (qual_small or allow_tow) else None
        self.cam = front_camera(); self.camera_x = calibration()['front_camera_x_m']
        self.pub = self.create_publisher(GarmentObservations, '/z70/garments/observations', LATCHED)
        self.markers = self.create_publisher(MarkerArray, '/z70/garments', 1)
        self.json_pub = self.create_publisher(String, '/z70/garments/json', 1)
        self.status = self.create_publisher(String, '/z70/garments/status', LATCHED)
        self.grid = None; self.grid_received = None
        self.create_subscription(OccupancyGrid, '/z70/map/safe_grid', self.on_grid, LATCHED)
        self.tf = Buffer(cache_time=Duration(seconds=30.)); self.tf_listener = TransformListener(self.tf, self)
        self.tracker = GarmentTracker(); self.last_pose = None; self.last_session = None
        self.lock = threading.Lock(); self.stop_event = threading.Event()
        self.latest = None; self.result = None; self.process = None; self.worker = None
        self.frames = MatchedFrames(self, 'front', self.on_frame, compressed=True)
        self.set_status(dict(model_available=False, backend=backend, state='STARTING'))
        if backend not in BACKENDS:
            self.set_status(dict(model_available=False, error='UNKNOWN_BACKEND')); return
        env = worker_env(HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1', HF_HUB_DISABLE_TELEMETRY='1')
        try:
            self.process = subprocess.Popen([python, '-u', str(Path(__file__).with_name('garment_worker.py')),
                                             '--backend', backend, '--device', device], stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, env=env, bufsize=0)
            self.worker = threading.Thread(target=self.run_worker, args=(1/max(.01, hz),), daemon=True)
            self.worker.start()
        except OSError as error:
            self.set_status(dict(model_available=False, error=type(error).__name__))
        self.create_timer(.1, self.publish)

    def set_status(self, quality):
        self.status.publish(String(data=json.dumps(quality, allow_nan=False)))

    def on_grid(self, msg):
        self.grid = msg; self.grid_received = time.monotonic()

    def on_frame(self, msg):
        meta = self.frames.metadata
        if (msg.header.frame_id != 'camera_optical_frame' or meta is None
                or (meta.width, meta.height) != (self.cam.width, self.cam.height)
                or not 0 < len(msg.data) <= MAX_PACKET):
            return
        with self.lock:
            self.latest = (msg, deepcopy(meta))

    def read_exact(self, count):
        return read_pipe_exact(self.process, count, self.stop_event)

    def run_worker(self, period):
        proc = self.process
        while not self.stop_event.is_set():
            with self.lock:
                frame, self.latest = self.latest, None
            if frame is None:
                self.stop_event.wait(.05); continue
            msg, meta = frame
            if not 0 <= time.time()-seconds(meta.evidence.received_at) <= self.max_age:
                continue
            try:
                data = bytes(msg.data)
                # Raw pipes can short-write; use write-all, not one optimistic write.
                payload = memoryview(struct.pack('<I', len(data))+data)
                while payload and not self.stop_event.is_set():
                    written = proc.stdin.write(payload)
                    if not written:
                        raise EOFError('Garment stdin closed')
                    payload = payload[written:]
                size = struct.unpack('<I', self.read_exact(4))[0]
                if not 0 < size <= MAX_PACKET:
                    raise ValueError('Oversized worker response')
                detections, quality = decode_result(self.read_exact(size))
                with self.lock:
                    self.result = (meta, detections, quality)
            except (OSError, EOFError, TimeoutError, ValueError, KeyError) as error:
                with self.lock:
                    self.result = (meta, [], dict(model_available=False, error=type(error).__name__))
                break
            self.stop_event.wait(period)

    def pose_at(self, stamp):
        try:
            tf = self.tf.lookup_transform('map', 'base_footprint', Time.from_msg(stamp), timeout=Duration(seconds=0.))
            q = tf.transform.rotation; t = tf.transform.translation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            age = abs(seconds(tf.header.stamp)-seconds(stamp))
            return (t.x, t.y, yaw), age
        except TransformException:
            return None, float('inf')

    def publish(self):
        with self.lock:
            result, self.result = self.result, None
        if result is None:
            if self.process is not None and self.process.poll() is not None:
                self.set_status(dict(model_available=False, error='WORKER_EXITED'))
            return
        meta, detections, quality = result
        age = time.time()-seconds(meta.evidence.received_at)
        if not 0 <= age <= self.max_age:
            quality = dict(quality, model_available=False, error='STALE_INFERENCE', frame_age_s=age)
            detections = []
            meta = deepcopy(meta); meta.evidence.stale = True; meta.evidence.valid = False
            meta.evidence.reason = 'STALE_INFERENCE'
        pose, tf_age = self.pose_at(meta.header.stamp)
        grid = self.grid
        grid_age = (max(time.monotonic()-self.grid_received, time.time()-seconds(grid.header.stamp))
                    if grid is not None else float('inf'))
        session = bytes(meta.evidence.session_id.uuid)
        # Missing ego pose cannot be interpreted as zero motion.
        if session != self.last_session or pose is None or self.last_pose is None:
            self.tracker = GarmentTracker(); delta = (0., 0., 0.)
        else:
            old_x, old_y, old_yaw = self.last_pose
            dx, dy = pose[0]-old_x, pose[1]-old_y
            delta = (math.cos(old_yaw)*dx+math.sin(old_yaw)*dy,
                     -math.sin(old_yaw)*dx+math.cos(old_yaw)*dy, pose[2]-old_yaw)
        self.last_pose, self.last_session = pose, session
        items = []
        for detection in detections:
            try:
                item = observe(detection['mask'], self.cam, category=detection['category'],
                               confidence=detection['confidence'], camera_x_m=self.camera_x,
                               calibration_id=meta.calibration_id,
                               qualified_policy=getattr(self, 'qualified_policy', None))
            except (ValueError, KeyError):
                continue
            if item['contact_valid']:
                item['inside_geofence'], item['geofence_verified'] = geofence(
                    item['floor_contact_m'], pose, grid, tf_age_s=tf_age, grid_age_s=grid_age)
            # Contour diagnostic output is not a garment selection candidate.
            if not quality.get('model_available', False):
                item['grasp_valid'] = False; item['reachable_for_grasp'] = False
            items.append(item)
        items = self.tracker.update(items, delta)
        for item in items:
            item['reachable_for_grasp'] = bool(item['reachable_for_grasp'] and item['confirmed']
                                               and item['inside_geofence'] and item['geofence_verified'])
        _, active = geofence((0., 0., 0.), pose, grid, tf_age_s=tf_age, grid_age_s=grid_age)
        msg = observations_message(items, meta, quality, active)
        self.pub.publish(msg)
        self.json_pub.publish(String(data=json.dumps(dict(model=quality, geofence_active=active,
                                                          frame_age_s=age, garments=items), allow_nan=False)))
        markers = MarkerArray()
        clear = Marker(); clear.header = deepcopy(msg.header); clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, item in enumerate(items):
            if not item['contact_valid']:
                continue
            marker = Marker(); marker.header = deepcopy(msg.header); marker.ns = 'garments'; marker.id = i
            marker.type = Marker.SPHERE; marker.action = Marker.ADD
            marker.pose.position = Point(**dict(zip(('x', 'y', 'z'), item['floor_contact_m'])))
            marker.pose.orientation.w = 1.; marker.scale.x = marker.scale.y = marker.scale.z = .025
            marker.color.a = 1.; marker.color.g = .8 if item['inside_geofence'] else 0.; marker.color.r = .8
            marker.lifetime = Duration(seconds=5.).to_msg(); markers.markers.append(marker)
        self.markers.publish(markers)
        self.set_status(dict(quality, geofence_active=active, frame_age_s=age, tracks=len(self.tracker.tracks),
                             confirmed=sum(x['confirmed'] for x in items)))

    def close(self):
        stop_worker_process(self.process, self.worker, self.stop_event)


def main():
    rclpy.init(); node = GarmentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.close(); node.destroy_node(); rclpy.try_shutdown()
