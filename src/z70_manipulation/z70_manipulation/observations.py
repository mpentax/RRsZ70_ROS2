"""ROS 2 observation adapter for manipulation and navigation actions."""
from copy import deepcopy
from types import SimpleNamespace
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from z70_interfaces.msg import RobotState, ArmEstimate, FrameMetadata, MapSnapshot, GarmentObservations
from z70_interfaces.srv import SelectCamera, RefreshMap

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec/1e9


def read_status(observations) -> dict:
    fn = getattr(observations, 'status', None)
    return fn() if callable(fn) else observations.get('/api/state')['telemetry']['status']['value']


def read_arm(observations) -> dict:
    fn = getattr(observations, 'arm_estimate', None)
    return fn() if callable(fn) else observations.get('/api/arm/estimate')


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
        self.frames = {}; self.metadata = {}
        self.group = ReentrantCallbackGroup()
        self.subscriptions = [
            self.node.create_subscription(RobotState, '/z70/state', self.on_state, LATCHED, callback_group=self.group),
            self.node.create_subscription(ArmEstimate, '/z70/arm_estimate', self.on_arm, LATCHED, callback_group=self.group),
            self.node.create_subscription(GarmentObservations, '/z70/garments/observations', self.on_garments, LATCHED, callback_group=self.group),
            self.node.create_subscription(MapSnapshot, '/z70/driver/map_snapshot', self.on_map, LATCHED, callback_group=self.group)]
        self.map_refresher = self.node.create_client(RefreshMap, '/z70/map/refresh', callback_group=self.group)
        for camera in ('front','gripper'):
            self.subscriptions.extend([
                self.node.create_subscription(CompressedImage, f'/camera/{camera}/image_raw/compressed',
                                              lambda m,c=camera:self.on_image(m,c), 2, callback_group=MutuallyExclusiveCallbackGroup()),
                self.node.create_subscription(FrameMetadata, f'/camera/{camera}/metadata',
                                              lambda m,c=camera:self.on_metadata(m,c), 2, callback_group=MutuallyExclusiveCallbackGroup())])
        self.selector = self.node.create_client(SelectCamera, '/z70/camera/select', callback_group=self.group)
        if self.owned:
            self.executor = SingleThreadedExecutor(); self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin, name='pickup_observations')
            self.thread.start()

    @staticmethod
    def evidence_id(evidence):
        return (bytes(evidence.session_id.uuid), evidence.sequence)

    def on_garments(self, msg):
        with self.condition: self.garments = msg; self.condition.notify_all()

    def garment_barrier(self):
        with self.condition:
            meta = self.metadata.get('front')
            return dict(garment=self.evidence_id(self.garments.evidence) if self.garments else None,
                        image=self.evidence_id(meta.evidence) if meta else None)

    def garment_snapshot(self, after=None, max_age_s=6.):
        prior = after.get('garment') if isinstance(after,dict) else after
        def ready():
            m = self.garments
            return (m is not None and self.fresh(m.evidence,max_age_s)
                    and (prior is None or self.evidence_id(m.evidence)!=prior))
        self.wait(ready)
        with self.condition: message = deepcopy(self.garments)
        if not message.model_available: raise LookupError('GARMENT_MODEL_UNAVAILABLE')
        return message

    def garment_frame(self, after=None):
        prior = after.get('image') if isinstance(after,dict) else after
        def ready():
            meta=self.metadata.get('front'); frame=self.frames.get('front')
            return (meta is not None and frame is not None and self.key(meta)==self.key(frame)
                    and self.fresh(meta.evidence,6.) and self.evidence_id(meta.evidence)!=prior
                    and self.evidence_id(meta.evidence)!=self.used)
        self.wait(ready)
        with self.condition:
            frame=deepcopy(self.frames['front']); meta=deepcopy(self.metadata['front'])
            self.used=self.evidence_id(meta.evidence)
        image=cv2.imdecode(np.frombuffer(frame.data,np.uint8),cv2.IMREAD_COLOR)
        if image is None: raise LookupError('invalid garment evidence frame')
        visible = (image.shape[:2]==(meta.height,meta.width) and float(image.mean())>8.
                   and float(np.percentile(image,95)-np.percentile(image,5))>12.)
        return dict(image=image,source_id=(bytes(meta.evidence.session_id.uuid).hex(),meta.evidence.sequence),
                    evidence=meta.evidence,coverage_valid=visible)

    def on_map(self, msg):
        with self.condition: self.map_snapshot = msg; self.condition.notify_all()

    def _call_service(self, client, request, timeout_s, timeout_exc):
        future = client.call_async(request)
        done = threading.Event(); future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + timeout_s
        while not done.wait(.02):
            if self.cancelled(): raise InterruptedError('CANCELLED')
            if time.monotonic() >= deadline: raise timeout_exc
        return future.result()

    def navigation_snapshot(self, refresh=False, max_age_s=5.):
        from z70_mapping.decoder import decode_map
        expected = None
        if refresh:
            if not self.map_refresher.wait_for_service(timeout_sec=1.): raise LookupError('MAP_REFRESH_UNAVAILABLE')
            response = self._call_service(self.map_refresher, RefreshMap.Request(), 35., TimeoutError('MAP_REFRESH_TIMEOUT'))
            if response.outcome.error_code != 'NONE' or not response.snapshot.valid or response.snapshot.stale:
                raise LookupError('MAP_REFRESH_FAILED')
            expected = response.snapshot
        def ready():
            m=self.map_snapshot
            return (m is not None and m.evidence.valid and not m.evidence.stale
                    and (refresh or max_age_s is None or self.fresh(m.evidence,max_age_s)
                         or (m.header.stamp.sec > 0 and 0 <= time.time()-stamp_seconds(m.header.stamp) <= max_age_s))
                    and (expected is None or (m.evidence.session_id == expected.session_id
                         and m.evidence.sequence >= expected.sequence)))
        self.wait(ready)
        with self.condition: m=deepcopy(self.map_snapshot)
        raw_age=time.time()-stamp_seconds(m.evidence.received_at)
        hdr_age=time.time()-stamp_seconds(m.header.stamp) if m.header.stamp.sec > 0 else raw_age
        cache_age_s=0. if refresh else min(raw_age, hdr_age) if hdr_age >= 0 else raw_age
        return SimpleNamespace(map=decode_map(bytes(m.raw)), labels={r.segment_id:r.name for r in m.rooms},
            sequence=m.evidence.sequence, map_index=m.map_index, map_sequence=m.map_sequence,
            session_id=bytes(m.evidence.session_id.uuid),
            active_floor_verified=m.active_floor_verified, current_pose_verified=m.current_pose_verified,
            cache_age_s=cache_age_s, evidence=m.evidence)

    def on_state(self, msg):
        with self.condition: self.state = msg; self.condition.notify_all()

    def on_arm(self, msg):
        with self.condition: self.arm = msg; self.condition.notify_all()

    @staticmethod
    def key(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id)

    def match_frame(self, camera):
        frame, meta = self.frames.get(camera), self.metadata.get(camera)
        if frame is not None and meta is not None and self.key(frame) == self.key(meta):
            self.image = (frame, meta, camera)
            self.condition.notify_all()

    def on_image(self, msg, camera):
        with self.condition: self.frames[camera] = msg; self.match_frame(camera)

    def on_metadata(self, msg, camera):
        with self.condition: self.metadata[camera] = msg; self.match_frame(camera)

    @staticmethod
    def fresh(evidence, max_age):
        return (evidence.valid and not evidence.stale and
                0 <= time.time()-stamp_seconds(evidence.received_at) <= max_age)

    def wait(self, predicate):
        deadline = time.monotonic()+self.timeout
        with self.condition:
            while not predicate():
                if self.cancelled(): raise InterruptedError('CANCELLED')
                remaining = deadline-time.monotonic()
                if remaining <= 0: raise LookupError('shared driver observation unavailable or stale')
                self.condition.wait(min(remaining,.05))

    def status(self) -> dict:
        self.wait(lambda: self.state is not None and self.fresh(self.state.evidence, 15.))
        with self.condition: m = deepcopy(self.state)
        if not m.connected: raise LookupError('driver disconnected')
        status = {}
        for field, key in [('robot_state','state'),('error_code','error_code'),('common_status','common_status'),('grab_status','grab_status')]:
            if getattr(m,field+'_known'): status[key] = getattr(m,field)
        if m.battery_known: status['battery'] = m.battery_fraction*100
        if m.cleaning_known: status['in_cleaning']=m.in_cleaning
        if m.returning_known: status['in_returning']=m.in_returning
        return status

    def arm_estimate(self) -> dict:
        self.wait(lambda: self.arm is not None and self.arm.evidence.valid and not self.arm.evidence.stale
                  and self.state is not None and self.fresh(self.state.evidence, 15.))
        with self.condition: m = deepcopy(self.arm)
        usable = m.validity.value in (1,2) and not m.restored
        q = dict(zip(m.joints.name, m.joints.position))
        j = {}
        if usable and all(k in q for k in ('m3_joint','m4_joint','m5_joint')):
            j = {'m3':math.degrees(q['m3_joint'])+270, 'm4':180-math.degrees(q['m4_joint']),
                 'm5':-math.degrees(q['m5_joint'])}
        else: usable = False
        state='IN_PROCESS_POS' if usable else 'IN_ROBOT' if m.validity.value==3 and m.validity.reason=='COMMAND_DERIVED_STOW' else 'UNKNOWN'
        return {'state':state, 'joints':j,
                'source':'ROS_COMMAND_ESTIMATE', 'updatedAt':stamp_seconds(m.evidence.source_at)*1000,
                'stepsSinceAnchor':m.steps_since_anchor}

    def get(self, path):
        if path == '/api/state': return {'telemetry':{'status':{'value':self.status()}}}
        if path == '/api/arm/estimate': return self.arm_estimate()
        raise ValueError('unsupported observation path')

    def frame(self):
        def available():
            if self.image is None: return False
            _, meta, _ = self.image
            return self.evidence_id(meta.evidence) != self.used and self.fresh(meta.evidence, 5.)
        self.wait(available)
        with self.condition:
            frame, meta, camera = self.image
            self.used = self.evidence_id(meta.evidence)
        image = cv2.imdecode(np.frombuffer(frame.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None: raise LookupError('invalid shared frame')
        return image, camera

    def select(self, camera):
        if camera not in ('front','gripper'): raise ValueError('unsupported camera')
        if not self.selector.wait_for_service(timeout_sec=2.): raise LookupError('driver selector unavailable')
        response = self._call_service(self.selector, SelectCamera.Request(camera=1 if camera=='front' else 2),
                                      20., LookupError('CAMERA_SELECT_TIMEOUT'))
        if not response.state.selection_confirmed or response.outcome.error_code not in ('NONE','TIMEOUT'):
            raise LookupError(response.outcome.error_code or 'selection unconfirmed')
        with self.condition:
            self.image = None; self.frames.clear(); self.metadata.clear()
        return {'camera':camera}

    def close(self):
        if self.owned:
            self.executor.shutdown(); self.thread.join(timeout=5)
            self.node.destroy_node()
            if self.initialized: rclpy.try_shutdown()
        else:
            for sub in self.subscriptions: self.node.destroy_subscription(sub)
            self.node.destroy_client(self.selector)
            self.node.destroy_client(self.map_refresher)
