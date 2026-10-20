"""ROS 2 driver node for robot state, camera frames, maps, and arm estimates."""
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

import cv2
import numpy as np
import requests
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
from unique_identifier_msgs.msg import UUID
from z70_interfaces.msg import (EvidenceStamp, RobotState, CameraState, FrameMetadata,
                                ArmEstimate, MapSnapshot, RoomLabel, CommandEvent, CommandOutcome)
from z70_interfaces.srv import SelectCamera, RefreshMap
from z70_description.arm_model import LIMITS, POSES
from z70_mapping.decoder import decode_map, room_names
from .commands import CommandSurfaces, LATCHED, host_ms, host_time, uid
from .dashboard import ArmEstimator, Dashboard

STOW_VISUAL = dict(m3=90., m4=170., m5=0., m6=0.)
CAMERAS = {'front': 1, 'gripper': 2}


def estimate_to_app(estimate):
    joints = estimate.get('joints') or {}
    if (estimate.get('state', 'UNKNOWN') == 'IN_ROBOT'
            or float(joints.get('m3', 0)) < LIMITS['m3'][0]
            or float(joints.get('m4', 0)) < LIMITS['m4'][0]):
        return dict(STOW_VISUAL)
    return {j: float(joints[j]) for j in STOW_VISUAL}


def app_to_urdf(m3, m4, m5, m6):
    f = max(0., min(math.radians(m6) / 2, math.pi / 4))
    return dict(m3_joint=math.radians(m3-270), m4_joint=math.radians(180-m4),
                m5_joint=-math.radians(m5), m6_left_joint=f, m6_right_joint=f)


class Driver(CommandSurfaces, Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('z70_driver', parameter_overrides=parameter_overrides or [])
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.origin = param('origin', 'http://127.0.0.1:57010').rstrip('/')
        token_file = param('token_file', '')
        self.auth = {'Authorization': 'Bearer ' + Path(token_file).read_text().strip()} if token_file else {}
        self.camera_hz = float(param('camera_hz', 3.))
        self.state_hz = float(param('state_hz', 1.))
        self.map_interval = float(param('map_refresh_s', 60.))
        self.state_stale = float(param('state_stale_after_s', 5.))
        self.camera_stale = float(param('camera_stale_after_s', 5.))
        config = Path(get_package_share_directory('z70_description')) / 'config'
        self.calibrations = {}
        for camera, default in [('front', config/'z70_front_camera_info.yaml'),
                                ('gripper', config/'z70_gripper_camera_info.yaml')]:
            path = param('camera_info' if camera == 'front' else 'gripper_camera_info', str(default))
            if path and Path(path).is_file():
                content = Path(path).read_bytes()
                self.calibrations[camera] = (self.load_camera_info(yaml.safe_load(content)), hashlib.sha256(content).hexdigest())
        self.driver_session = uuid.uuid4()
        self.session = uid(self.driver_session)
        self.backend_session = None
        self.sequences = {}
        self.samples = {}
        self.arm_payload = None
        self.display_sample = None
        self.estimate_source = 1
        self.good_at = {}
        self.stop_event = threading.Event()
        self.acquisition_generation = 0
        self.camera_lock = threading.Lock()
        self.map_lock = threading.Lock()
        self.sample_lock = threading.RLock()
        self.workers = []
        self.worker_lock = threading.Lock()
        self.stream_response = None
        self.event_session = None
        self.event_sequence = None
        self.pub_state = self.create_publisher(RobotState, '/z70/state', LATCHED)
        self.pub_camera_state = self.create_publisher(CameraState, '/z70/camera/state', LATCHED)
        self.pub_arm = self.create_publisher(ArmEstimate, '/z70/arm_estimate', LATCHED)
        self.pub_map = self.create_publisher(MapSnapshot, '/z70/driver/map_snapshot', LATCHED)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_display = self.create_publisher(JointState, '/z70/display/joint_states', 10)
        self.pub_events = self.create_publisher(CommandEvent, '/z70/commands', 256)
        self.pub_diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.dashboard = Dashboard(self)
        self.pub_img = {}; self.pub_raw = {}; self.pub_info = {}; self.pub_metadata = {}
        for camera in CAMERAS:
            prefix = '/camera/' + camera
            self.pub_img[camera] = self.create_publisher(CompressedImage, prefix+'/image_raw/compressed', LATCHED)
            self.pub_raw[camera] = self.create_publisher(Image, prefix+'/image_raw', 2)
            self.pub_info[camera] = self.create_publisher(CameraInfo, prefix+'/camera_info', LATCHED)
            self.pub_metadata[camera] = self.create_publisher(FrameMetadata, prefix+'/metadata', LATCHED)
        group = ReentrantCallbackGroup()
        self.create_service(SelectCamera, '/z70/camera/select', self.select, callback_group=group)
        self.create_service(RefreshMap, '/z70/map/refresh', self.refresh, callback_group=group)
        self.create_timer(.25, self.check_stale)
        estimate_default = str(Path(os.environ.get('XDG_DATA_HOME', str(Path.home()/'.local/share')))/'RRsZ70_ROS2/arm-estimate.json')
        self.estimator = ArmEstimator(path=param('arm_estimate_path', estimate_default))
        self.init_commands()
        self.publish_estimate(self.estimator.snapshot())
        self.start_worker('state', self.state_loop)
        self.start_worker('events', self.events_loop)
        if self.camera_hz > 0:
            self.start_worker('camera', self.camera_loop)
        if self.map_interval > 0:
            self.start_worker('map', self.map_loop)

    @staticmethod
    def load_camera_info(d):
        m = CameraInfo(width=d['image_width'], height=d['image_height'], distortion_model=d['distortion_model'])
        for field, key in [('k', 'camera_matrix'), ('d', 'distortion_coefficients'),
                           ('r', 'rectification_matrix'), ('p', 'projection_matrix')]:
            setattr(m, field, [float(x) for x in d[key]['data']])
        return m

    def acquisition_session(self, backend_epoch):
        # Source headers identify a backend PROCESS, not a device preview.
        # Namespace by this observer so driver restarts cannot reuse sequences.
        return uid(uuid.uuid5(self.driver_session, str(backend_epoch)))

    def start_worker(self, name, fn):
        worker = threading.Thread(target=fn, name='z70_driver_'+name)
        with self.worker_lock:
            self.workers = [w for w in self.workers if w.is_alive()]
            self.workers.append(worker)
            worker.start()

    def request(self, method, path, timeout=5, **kw):
        allow_error = kw.pop('allow_error', False)
        if (not self.dispatcher.session_enabled and path not in
                ('/api/driver/control/session', '/api/driver/rpc', '/api/commands/events')):
            raise requests.RequestException('SESSION_STOPPED')
        r = requests.request(method, self.origin+path, headers=self.auth, timeout=timeout, **kw)
        if not allow_error: r.raise_for_status()
        return r

    def evidence(self, stream, source_ms=None, kind=0, session=None, sequence=None):
        with self.sample_lock:
            selected_session = session or self.session
            key = (stream, bytes(selected_session.uuid))
            if sequence is None:
                self.sequences[key] = self.sequences.get(key, 0)+1
                sequence = self.sequences[key]
                while len(self.sequences)>128: self.sequences.pop(next(iter(self.sequences)))
            e = EvidenceStamp(received_at=host_time(time.time()), valid=True,
                              session_id=selected_session, sequence=sequence,
                              timestamp_kind=kind)
        if source_ms is not None:
            e.source_at = host_ms(source_ms)
            e.source_time_known = True; e.source_clock = 1
        return e

    def header(self, msg, frame='base_footprint'):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame

    def diagnostic(self, category, code, level=1):
        d = DiagnosticStatus(name='z70_driver/'+category, level=bytes([level]),
                             message=code, hardware_id='z70', values=[KeyValue(key='code', value=code)])
        m = DiagnosticArray(status=[d]); self.header(m, 'base_footprint')
        self.pub_diagnostics.publish(m)

    def remember(self, name, msg, pub):
        with self.sample_lock:
            self.samples[name] = (msg, pub)
            self.good_at[name] = time.monotonic()
            pub.publish(msg)

    def gap(self, name):
        with self.sample_lock:
            if name in self.samples:
                msg, pub = self.samples[name]
                msg.evidence.valid = False; msg.evidence.stale = True; msg.evidence.reason = 'OBSERVATION_GAP'
                if name == 'state':
                    msg.connected = False; msg.transport_state = 3
                if name == 'arm':
                    msg.validity.value = 4; msg.validity.reason = 'OBSERVATION_GAP'
                pub.publish(msg)
        self.diagnostic(name, 'OBSERVATION_GAP')

    def _publish_display(self, sample=None):
        with self.sample_lock:
            if sample is not None:
                self.display_sample = sample
                fresh = sample
            elif self.display_sample is not None:
                fresh = deepcopy(self.display_sample)
                fresh.header.stamp = self.get_clock().now().to_msg()
            else:
                return
        self.pub_js.publish(fresh)
        self.pub_display.publish(fresh)

    def check_stale(self):
        self._publish_display()
        with self.sample_lock:
            for name in list(self.samples):
                limit = self.camera_stale if name == 'camera' else self.state_stale
                if name == 'map':
                    continue  # Source freshness is unknown, independent of cache receipt age.
                if time.monotonic()-self.good_at[name] > limit:
                    self.gap(name)

    def state_loop(self):
        while not self.stop_event.is_set():
            state_available = False
            try:
                d = self.request('GET', '/api/state', timeout=8).json()
                self.dashboard.state = d
                self.dashboard.state_received = time.time()
                if d.get('observationSession') and d['observationSession'] != self.backend_session:
                    if self.backend_session is not None:
                        with self.estimator_lock: self.estimator.invalidate('SESSION_CHANGED')
                    with self.sample_lock:
                        if self.backend_session is not None:
                            self.diagnostic('state', 'SESSION_CHANGED')

                            for name in ('state','selection','arm','map'): self.gap(name)
                        self.backend_session = d['observationSession']
                        self.session = self.acquisition_session(self.backend_session)
                telemetry = d.get('telemetry', {})
                entry = telemetry.get('status', {})
                values = entry.get('value')
                m = RobotState(); self.header(m, 'base_footprint')
                m.evidence = self.evidence('state', entry.get('observedAt'))
                connection = d.get('connection', {}).get('state', 'disconnected')
                m.connected = connection == 'connected'
                m.transport_state = {'disconnected': 0, 'connecting': 1, 'connected': 2}.get(connection, 3)
                if (not isinstance(values, dict) or entry.get('observedAt') is None or
                    not 0 <= time.time()-entry['observedAt']/1000 <= self.state_stale):
                    self.gap('state')
                else:
                    for source, field in [('state','robot_state'), ('error_code','error_code'),
                                          ('common_status','common_status'), ('grab_status','grab_status')]:
                        v = values.get(source)
                        if isinstance(v, int) and not isinstance(v, bool) and 0 <= v <= 0xffffffff:
                            setattr(m, field, v); setattr(m, field+'_known', True)
                    if m.common_status_known and not m.grab_status_known:
                        m.grab_status = (m.common_status >> 6) & 0xf
                        m.grab_status_known = True
                    battery = values.get('battery')
                    if isinstance(battery, (float, int)) and 0 <= battery <= 100:
                        m.battery_known = True; m.battery_fraction = float(battery/100)
                    for source, known in [('in_cleaning','cleaning_known'),('in_returning','returning_known')]:
                        value=values.get(source)
                        if isinstance(value,(bool,int)) and value in (False,True,0,1):
                            setattr(m,known,True);setattr(m,source,bool(value))
                    self.remember('state', m, self.pub_state)
                    state_available = m.connected
                    with self.estimator_lock:
                        common = values.get('common_status')
                        if (isinstance(common, int)
                                and ((common & 256) or (((common >> 6) & 15) in (1, 2, 3, 4) and ((common >> 10) & 3) in (1, 2)))
                                and self.estimator.data.get('state') in ('UNKNOWN', 'IN_ROBOT')
                                and self.estimator.data.get('unknownReason') != 'GRIP_POSE_UNVERIFIED'):
                            if float(self.estimator.data.get('joints', {}).get('m3', 0)) < LIMITS['m3'][0]:
                                self.estimator._anchor('WORKING_POS', 'common_status_deployed')
                            else:
                                self.estimator.data.update(state='IN_PROCESS_POS', restored=False, unknownReason=None)
                                self.estimator._commit('common_status_deployed', 'RECONCILED_DEPLOYED')
                        self.estimator.observe_status(values, entry.get('requestedAt',entry['observedAt']))
                ce = telemetry.get('camera', {})
                cv = ce.get('value')
                # Status responses can be a one-element numeric array or a named numeric field.
                if isinstance(cv, list) and len(cv) == 1: cv = cv[0]
                if isinstance(cv, dict): cv = cv.get('camera_status')
                if (isinstance(cv, int) and ce.get('observedAt') is not None and
                    0 <= time.time()-ce['observedAt']/1000 <= self.state_stale):
                    c = CameraState(selected_camera=2 if cv & 32768 else 1, selection_confirmed=True)
                    self.header(c); c.evidence = self.evidence('camera_state', ce['observedAt'])
                    self.remember('selection', c, self.pub_camera_state)
                else: self.gap('selection')
            except (requests.RequestException, ValueError, TypeError, KeyError):
                self.gap('state'); self.gap('selection')
            try:
                if state_available:
                    with self.estimator_lock:
                        self.publish_estimate(self.estimator.snapshot())
                else:
                    self.gap('arm')
            except (requests.RequestException, ValueError, TypeError, KeyError):
                self.gap('arm')
            self.stop_event.wait(1/max(.1, self.state_hz))

    def publish_estimate(self, d):
        self.dashboard.arm = d
        self.dashboard.arm_received = time.time()
        if d.get('persistenceError'): self.diagnostic('arm','PERSISTENCE_FAILED',2)
        updated = d.get('updatedAt')
        with self.sample_lock:
            old = self.samples.get('arm')
            same_update = (old is not None and self.arm_payload == d and
                           old[0].evidence.session_id == self.session and
                           old[0].evidence.source_time_known and old[0].evidence.source_at == host_ms(updated or 0))
            if same_update and old[0].evidence.valid and not old[0].evidence.stale:
                # Polling an unchanged estimate must not mint a new update stamp/identity.
                self.good_at['arm'] = time.monotonic()
                self._publish_display()
                return
        m = ArmEstimate(source=self.estimate_source, model_id='vendor-app', restored=bool(d.get('restored')))
        self.header(m, 'vendor_arm_planar'); m.evidence = self.evidence('arm', updated, 3)
        if same_update:
            # Recovery of the same cached model update changes availability, not
            # its command-derived timestamp or identity.
            m.header = deepcopy(old[0].header); m.evidence = deepcopy(old[0].evidence)
            m.evidence.valid = True; m.evidence.stale = False; m.evidence.reason = ''
        usable = d.get('state') not in ('UNKNOWN','IN_ROBOT') and not m.restored
        m.validity.value = 1 if usable else 0
        m.validity.reason = 'COMMAND_DEAD_RECKONING' if usable else 'UNRECONCILED_OR_UNKNOWN'
        if d.get('state') == 'IN_ROBOT' and not m.restored:
            m.validity.value = 3; m.validity.reason = 'COMMAND_DERIVED_STOW'
        m.unknown_reason = '' if usable else 'UNRECONCILED_OR_UNKNOWN'
        q = app_to_urdf(**estimate_to_app(d))
        display = JointState(); display.header = deepcopy(m.header)
        display.name = [*q, 'shoulder_lift', 'shoulder_yaw', 'arm_lid']
        display.position = [*q.values(), 0., 0., 0.]
        if usable:
            # Only the three arm joints have even heuristic uncertainty in ArmEstimate.
            m.joints = JointState(header=deepcopy(m.header), name=list(q)[:3], position=list(q.values())[:3])
            sigma = d.get('sigmaDeg', {})
            if all(isinstance(sigma.get(j), (int,float)) and math.isfinite(sigma[j]) and sigma[j]>=0 for j in ('m3','m4','m5')):
                m.joint_uncertainty_known = True
                m.joint_sigma_rad = [math.radians(sigma[j]) for j in ('m3','m4','m5')]
        self._publish_display(display)
        m.steps_since_anchor = int(d.get('stepsSinceAnchor', 0))
        if d.get('anchoredAt') is not None:
            m.anchor_time_known = True; m.anchored_at = host_ms(d['anchoredAt'])
        # Legacy wrist is vendor 2-D app geometry, not a measured ROS point/covariance.
        self.arm_payload = deepcopy(d)
        self.remember('arm', m, self.pub_arm)

    def camera_loop(self):
        while not self.stop_event.is_set():
            start = time.monotonic()
            try:
                with self.camera_lock:
                    if self.stop_event.is_set(): break
                    generation = self.acquisition_generation
                    r = self.request('POST', '/api/camera/frame', json={}, timeout=20)
                    if self.dispatcher.session_enabled and generation == self.acquisition_generation:
                        self.publish_frame(r)
            except (requests.RequestException, ValueError, TypeError, KeyError):
                self.gap('camera')
            self.stop_event.wait(max(0., 1/self.camera_hz-(time.monotonic()-start)))

    def publish_frame(self, r):
        camera = r.headers.get('X-Roboclaw-Camera')
        if camera not in CAMERAS:
            raise ValueError('unlabelled frame')
        image = cv2.imdecode(np.frombuffer(r.content, np.uint8), cv2.IMREAD_COLOR)
        if image is None: raise ValueError('invalid JPEG')
        height, width = image.shape[:2]
        session = self.acquisition_session(r.headers['X-Roboclaw-Camera-Session']) if r.headers.get('X-Roboclaw-Camera-Session') else self.session
        meta = FrameMetadata(camera=CAMERAS[camera], width=width, height=height,
                             session_scope=1 if r.headers.get('X-Roboclaw-Camera-Session') else 0)
        self.header(meta, 'camera_optical_frame' if camera == 'front' else 'gripper_camera_optical_frame')
        meta.evidence = self.evidence('frame', session=session)
        # Backend received-at is host receipt, never exposure time.
        if r.headers.get('X-Roboclaw-Received-At'):
            meta.evidence.received_at = host_ms(r.headers['X-Roboclaw-Received-At'])
        info = self.calibrations.get(camera)
        if info and (info[0].width, info[0].height) == (width, height):
            calibrated = deepcopy(info[0]); calibrated.header = deepcopy(meta.header)
            meta.calibration_id = info[1]; self.pub_info[camera].publish(calibrated)
        else:
            meta.evidence.valid = False; meta.evidence.reason = 'CALIBRATION_MISMATCH'
            self.diagnostic('camera', 'CALIBRATION_MISMATCH')
        compressed = CompressedImage(header=meta.header, format='jpeg', data=r.content)
        raw = Image(header=meta.header, height=height, width=width, encoding='bgr8',
                    step=width*3, data=image.tobytes())
        self.pub_img[camera].publish(compressed); self.pub_raw[camera].publish(raw)
        self.remember('camera', meta, self.pub_metadata[camera])
        state = CameraState(header=meta.header, selected_camera=CAMERAS[camera], selection_confirmed=True,
                            evidence=deepcopy(meta.evidence))
        # Lens tagging valid even if the calibration is missing/mismatched.
        state.evidence.valid = True; state.evidence.reason = ''
        self.remember('selection', state, self.pub_camera_state)

    def service_outcome(self, operation):
        now = host_time(time.time())
        return CommandOutcome(command_id=uid(uuid.uuid4()), attempt_id=uid(uuid.uuid4()),
                              requested_at=now, error_code='NONE')

    def select(self, request, response):
        outcome = self.service_outcome('select_camera')
        if not self.dispatcher.session_enabled:
            outcome.error_code = 'SESSION_STOPPED'; response.outcome = outcome
            return response
        if request.camera not in (1,2):
            outcome.error_code = 'INVALID_INPUT'; response.outcome = outcome
            return response
        # Do not queue indefinitely behind a hung image response.
        if not self.camera_lock.acquire(timeout=.1):
            outcome.error_code = 'BUSY'; response.outcome = outcome
            return response
        try:
            camera = next(k for k,v in CAMERAS.items() if v == request.camera)
            result = self.request('POST', '/api/driver/camera/select', json={'camera':camera}, timeout=19).json()
            if result.get('camera') != camera: raise ValueError('unconfirmed lens')
            state = CameraState(selected_camera=request.camera, selection_confirmed=True)
            self.header(state); state.evidence = self.evidence('camera_state')
            self.remember('selection', state, self.pub_camera_state)
            rows=result.get('outcomes',[])
            wire=next((r['outcome'] for r in rows if r['method']=='app_change_camera'),None)
            if wire is not None: outcome=self.backend_outcome(wire)
            else:
                outcome=self.local_outcome() # no selector write was needed
            # A confirmed bit flip is selection evidence, not a fabricated RPC reply.
            # Expected firmware timeout remains UNKNOWN/TIMEOUT in the outcome.
            response.state = state
        except (requests.RequestException, ValueError):
            outcome.dispatch = 2; outcome.response = 3; outcome.error_code = 'OBSERVATION_GAP'
            self.gap('selection')
        finally: self.camera_lock.release()
        outcome.finished_at = host_time(time.time()); response.outcome = outcome
        return response

    def read_map(self, refresh=False):
        if not self.map_lock.acquire(blocking=False): raise BlockingIOError('map acquisition busy')
        try:
            if refresh: self.request('POST', '/api/map/cache/refresh', json={}, timeout=30)
            r = self.request('GET', '/api/map/cache/raw', timeout=3)
            m = MapSnapshot(raw=list(r.content), map_index=int(r.headers['X-Map-Index']),
                            map_sequence=int(r.headers['X-Map-Sequence']))
            self.header(m, 'map')
            session=self.acquisition_session(r.headers['X-Roboclaw-Observation-Session']) if r.headers.get('X-Roboclaw-Observation-Session') else self.session
            m.evidence = self.evidence('map', kind=2, session=session)
            # X-Map-Saved-At is CACHE receipt; NOT vendor capture/source time.
            m.evidence.received_at = host_ms(r.headers['X-Map-Saved-At'])
            m.evidence.reason = 'SOURCE_FRESHNESS_UNKNOWN'
            try:
                rooms = self.request('GET', '/api/home/rooms', timeout=2).json().get('rooms', [])
                _, mapping = self.driver_rpc('get_room_mapping', [], 2000)
                m.rooms = [RoomLabel(segment_id=seg, name=name[:128])
                           for seg, name in room_names(mapping or [], rooms).items()]
                if m.rooms and m.map_index >= 0 and m.map_sequence > 0:
                    decoded = decode_map(bytes(m.raw))
                    with self.sample_lock:
                        cur_state = self.samples.get('state')
                    status_vals = (self.dashboard.state or {}).get('telemetry', {}).get('status', {}).get('value') or {}
                    locating = bool(status_vals.get('is_locating', 0)) or bool(status_vals.get('is_exploring', 0))
                    r_state = cur_state[0].robot_state if cur_state and cur_state[0].robot_state_known else status_vals.get('state')
                    if (decoded.robot is not None and len(decoded.robot) == 3 and
                            all(v is not None and math.isfinite(v) for v in decoded.robot) and
                            bool(decoded.rooms) and len(decoded.pixels) > 64 and
                            not locating and r_state not in (22,)):
                        m.active_floor_verified = True
                        m.current_pose_verified = True
            except (requests.RequestException, ValueError, TypeError, KeyError):
                self.diagnostic('map', 'ROOM_LABELS_UNAVAILABLE')
            with self.sample_lock:
                old = self.samples.get('map')
                if (not refresh and old and old[0].evidence.valid and not old[0].evidence.stale and
                    old[0].evidence.session_id == m.evidence.session_id and
                    old[0].raw == m.raw and old[0].rooms == m.rooms):
                    return old[0]  # Preserve original evidence when cache did not change.
            self.remember('map', m, self.pub_map)
            return m
        finally: self.map_lock.release()

    def map_loop(self):
        first = True
        while not self.stop_event.is_set():
            try: self.read_map(refresh=not first)
            except BlockingIOError: pass
            except (requests.RequestException, ValueError, KeyError, TypeError):
                if first: self.gap('map')
            first = False
            self.stop_event.wait(self.map_interval)

    def refresh(self, request, response):
        outcome = self.service_outcome('refresh_map')
        try:
            response.snapshot = self.read_map(refresh=True).evidence
            outcome.dispatch = 1; outcome.response = 1
        except (BlockingIOError, requests.RequestException, ValueError, KeyError, TypeError):
            with self.sample_lock:
                cached = self.samples.get('map')
            if cached and cached[0].evidence.valid and not cached[0].evidence.stale:
                m = deepcopy(cached[0])
                self.header(m, 'map')
                m.evidence = self.evidence('map', kind=2, session=m.evidence.session_id)
                m.evidence.received_at = cached[0].evidence.received_at
                m.evidence.reason = cached[0].evidence.reason
                self.remember('map', m, self.pub_map)
                response.snapshot = m.evidence
                outcome.dispatch = 1; outcome.response = 1
            else:
                self.gap('map'); outcome.dispatch = 2; outcome.response = 3; outcome.error_code = 'OBSERVATION_GAP'
                if cached: response.snapshot = cached[0].evidence
        outcome.finished_at = host_time(time.time()); response.outcome = outcome
        return response

    def publish_event(self, row):
        session, sequence = row['sessionId'], int(row['sequence'])
        if session == self.event_session and self.event_sequence is not None:
            if sequence <= self.event_sequence: return
            if sequence != self.event_sequence+1:
                self.diagnostic('events', 'EVENT_GAP')
                with self.estimator_lock: self.estimator.invalidate('COMMAND_STREAM_GAP')
        elif self.event_session is not None:
            self.diagnostic('events', 'SESSION_CHANGED')
            with self.estimator_lock: self.estimator.invalidate('SESSION_CHANGED')
        self.event_session, self.event_sequence = session, sequence
        msg = CommandEvent(operation=row['operation'])
        msg.evidence = self.evidence('events', source_ms=row.get('attemptedAt'), session=uid(session), sequence=sequence)
        msg.phase = {'accepted':0,'dispatched':1,'replied':2,'finished':4}[row['phase']]
        msg.outcome = self.backend_outcome(row)
        p = row.get('params')
        if p and row['operation'] == 'app_rc_move':
            msg.requested_twist.linear.x = float(p['velocity']); msg.requested_twist.angular.z = float(p['omega'])
            ns = int(float(p['duration'])*1e6)
            msg.requested_duration.sec = ns//10**9; msg.requested_duration.nanosec = ns%10**9
        self.pub_events.publish(msg)

    def events_loop(self):
        while not self.stop_event.is_set():
            try:
                with self.request('GET', '/api/commands/events', timeout=(3, 2), stream=True) as r:
                    self.stream_response = r
                    for line in r.iter_lines(chunk_size=1):
                        if self.stop_event.is_set(): break
                        if len(line) > 8192: raise ValueError('oversized event')
                        if line: self.publish_event(json.loads(line))
            except (requests.RequestException, ValueError, TypeError, KeyError, AttributeError):
                if not self.stop_event.is_set():
                    self.diagnostic('events', 'OBSERVATION_GAP')
                    with self.estimator_lock:
                        self.estimator.invalidate('COMMAND_STREAM_GAP')
                        self.publish_estimate(self.estimator.snapshot())
            finally: self.stream_response = None
            self.stop_event.wait(1.)

    def close(self):
        self.stop_event.set()
        # Network reads have finite deadlines; let their owning thread close the response.
        deadline = time.monotonic()+130
        while time.monotonic() < deadline:
            with self.worker_lock:
                workers = list(self.workers)
            alive = [w for w in workers if w.is_alive()]
            if not alive: break
            for worker in alive:
                worker.join(timeout=max(0., min(.2, deadline-time.monotonic())))
        if any(w.is_alive() for w in self.workers):
            raise RuntimeError('driver worker failed to stop within network deadline')

    def destroy_node(self):
        for server in self.action_servers:
            server.destroy()
        self.action_servers.clear()
        return super().destroy_node()


def main():
    rclpy.init(); node = Driver(); executor = MultiThreadedExecutor(num_threads=8); executor.add_node(node)
    try: executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.close(); executor.shutdown(); node.destroy_node(); rclpy.try_shutdown()
