"""ROS-only latest observation adapter for the transitional pickup executor.

No camera acquisition or HTTP. An unchanged latched sample is not made fresh by receipt.
The one image slot is shared by all reads; each caller waits for a newer acquisition.
"""
from copy import deepcopy
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from z70_interfaces.msg import RobotState, ArmEstimate, FrameMetadata
from z70_interfaces.srv import SelectCamera

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec/1e9


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
        self.frames = {}; self.metadata = {}
        self.group = ReentrantCallbackGroup()
        self.subscriptions = [
            self.node.create_subscription(RobotState, '/z70/state', self.on_state, LATCHED, callback_group=self.group),
            self.node.create_subscription(ArmEstimate, '/z70/arm_estimate', self.on_arm, LATCHED, callback_group=self.group)]
        for camera in ('front','gripper'):
            self.subscriptions.extend([
                self.node.create_subscription(CompressedImage, f'/camera/{camera}/image_raw/compressed',
                                              lambda m,c=camera:self.on_image(m,c), 2, callback_group=self.group),
                self.node.create_subscription(FrameMetadata, f'/camera/{camera}/metadata',
                                              lambda m,c=camera:self.on_metadata(m,c), 2, callback_group=self.group)])
        self.selector = self.node.create_client(SelectCamera, '/z70/camera/select', callback_group=self.group)
        if self.owned:
            self.executor = SingleThreadedExecutor(); self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin, name='pickup_observations')
            self.thread.start()

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

    def get(self, path):
        if path == '/api/state':
            self.wait(lambda:self.state is not None and self.fresh(self.state.evidence, 5.))
            with self.condition: m = deepcopy(self.state)
            if not m.connected: raise LookupError('driver disconnected')
            status = {}
            for field, key in [('robot_state','state'),('error_code','error_code'),('common_status','common_status'),('grab_status','grab_status')]:
                if getattr(m,field+'_known'): status[key] = getattr(m,field)
            if m.battery_known: status['battery'] = m.battery_fraction*100
            if m.cleaning_known: status['in_cleaning']=m.in_cleaning
            if m.returning_known: status['in_returning']=m.in_returning
            return {'telemetry':{'status':{'value':status}}}
        if path == '/api/arm/estimate':
            # Command-derived update time can predate the current poll. Fresh
            # transport/state plus explicit estimate validity establish availability;
            # never rewrite source_at to hide the age of the command-derived pose.
            self.wait(lambda:self.arm is not None and self.arm.evidence.valid and not self.arm.evidence.stale
                      and self.state is not None and self.fresh(self.state.evidence,5.))
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
        raise ValueError('unsupported observation path')

    def frame(self):
        def available():
            if self.image is None: return False
            frame, meta, _ = self.image
            identity = (bytes(meta.evidence.session_id.uuid), meta.evidence.sequence)
            return identity != self.used and self.fresh(meta.evidence, 5.)
        self.wait(available)
        with self.condition:
            frame, meta, camera = self.image
            self.used = (bytes(meta.evidence.session_id.uuid), meta.evidence.sequence)
        image = cv2.imdecode(np.frombuffer(frame.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None: raise LookupError('invalid shared frame')
        return image, camera

    def select(self, camera):
        if camera not in ('front','gripper'): raise ValueError('unsupported camera')
        if not self.selector.wait_for_service(timeout_sec=2.): raise LookupError('driver selector unavailable')
        future = self.selector.call_async(SelectCamera.Request(camera=1 if camera=='front' else 2))
        done = threading.Event(); future.add_done_callback(lambda _:done.set())
        deadline=time.monotonic()+20.
        while not done.wait(.05):
            if self.cancelled(): raise InterruptedError('CANCELLED')
            if time.monotonic()>=deadline: raise LookupError('driver selection deadline; do not replay blindly')
        response = future.result()
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
