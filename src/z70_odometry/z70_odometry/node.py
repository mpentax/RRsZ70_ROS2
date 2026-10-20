"""Floor VO and appearance-only floor mosaic nodes for /camera/front/image_raw -> /odom + TF."""
from collections import OrderedDict
import json
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from tf2_ros import TransformBroadcaster

from z70_odometry.floor_vo import FloorCamera, FloorOdometry, decode_bgr8
from z70_odometry.chassis_calibrator import _arc, calibrated_camera_x, front_calibration_path, initial_camera_x_m, quat_yaw, stamp_ns
from z70_interfaces.msg import CalibrationState, CommandEvent, OdometryStatus
from rclpy.qos import QoSProfile, DurabilityPolicy
from z70_driver.frame_subscription import MatchedFrames


class FloorOdomNode(Node):
    def __init__(self):
        super().__init__("z70_floor_odometry")
        cal = self.declare_parameter("calibration", str(front_calibration_path())).value
        cam_x = float(self.declare_parameter("camera_x_m", initial_camera_x_m()).value)
        options = {name: self.declare_parameter(name, default).value for name, default in
                   [("max_range", 1.5), ("n_features", 3000), ("ransac_thresh_m", .008),
                    ("min_inliers", 80), ("max_rms_m", .003)]}
        self.vo = FloorOdometry(FloorCamera.load(cal), camera_x_m=cam_x, **options)
        self.x = self.y = self.th = 0.0
        self.prev = None
        self.ok = self.rejected = 0
        self.last_stamp = self.get_clock().now().to_msg()
        self.last_rejected = False
        self._pose_uncertain = False
        self._odom_published = False
        self.last_image_ns = 0
        self.last_vo_accept_ns = 0
        self.prev_stamp_ns = 0
        self.yaw_pos_gain, self.yaw_neg_gain, self.forward_gain = 0.70, 0.65, 1.0
        self.pending = []
        self._command_session = self._command_sequence = None
        self._attempts = {}
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.pub_status = self.create_publisher(OdometryStatus, "/z70/odom_status", 10)
        self.tf = TransformBroadcaster(self)
        self.frames=MatchedFrames(self,'front',self.on_image,on_session=self.reset_reference)
        self.create_subscription(CalibrationState,'/z70/chassis_calibration',self.on_calibration,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(CommandEvent, '/z70/commands', self.on_command, 256)
        self.create_timer(0.2, self.broadcast_tf)

    def broadcast_tf(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        if self.prev is None or now.nanoseconds - self.last_image_ns > 1_000_000_000:
            self._apply_pending(now.nanoseconds)
        if not self._odom_published:
            self.publish(stamp)
        else:
            self._send_tf(stamp)

    def _send_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        self.tf.sendTransform(t)

    def on_calibration(self, msg: CalibrationState):
        value = calibrated_camera_x(msg)
        if value is not None:
            self.vo.cam_x = value
        for field, attribute, default in [('yaw_positive', 'yaw_pos_gain', .70),
                                          ('yaw_negative', 'yaw_neg_gain', .65),
                                          ('forward', 'forward_gain', 1.0)]:
            gain = getattr(msg, field + '_gain')
            known = getattr(msg, field + '_known')
            setattr(self, attribute, float(gain) if known and math.isfinite(gain) and gain > 0 else default)

    def on_command(self, msg: CommandEvent):
        session, sequence = bytes(msg.evidence.session_id.uuid), msg.evidence.sequence
        if self._command_session == session and self._command_sequence is not None and sequence <= self._command_sequence:
            return
        if self._command_sequence is not None and (session != self._command_session or sequence != self._command_sequence + 1):
            self._attempts.clear()
            self.pending.clear()
        self._command_session, self._command_sequence = session, sequence
        ident = bytes(msg.outcome.attempt_id.uuid)
        if msg.phase == 1 and msg.operation == 'app_rc_move':
            if msg.evidence.source_time_known and msg.evidence.source_clock == 1:
                self._attempts[ident] = stamp_ns(msg.evidence.source_at)
                while len(self._attempts) > 256:
                    self._attempts.pop(next(iter(self._attempts)))
            return
        if msg.phase != 2:
            return
        start_ns = self._attempts.pop(ident, None)
        if start_ns is None or msg.operation != 'app_rc_move' or msg.outcome.dispatch != 1 or msg.outcome.response != 1:
            return
        velocity, omega = msg.requested_twist.linear.x, msg.requested_twist.angular.z
        duration_s = msg.requested_duration.sec + msg.requested_duration.nanosec / 1e9
        if not all(math.isfinite(v) for v in (velocity, omega, duration_s)) or duration_s <= 0:
            return
        motion_end_ns = start_ns + int(round(duration_s * 1e9))
        end_ns = max(stamp_ns(msg.outcome.finished_at), motion_end_ns)
        if self.last_vo_accept_ns >= motion_end_ns:
            return
        if start_ns < self.last_vo_accept_ns < motion_end_ns:
            duration_s = max(0.0, (motion_end_ns - self.last_vo_accept_ns) / 1e9)
            start_ns = self.last_vo_accept_ns
            if duration_s <= 1e-6:
                return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < end_ns and self.prev is not None and start_ns < self.prev_stamp_ns and self.last_vo_accept_ns < self.prev_stamp_ns:
            pre_end_ns = min(self.prev_stamp_ns, motion_end_ns)
            pre_s = (pre_end_ns - start_ns) / 1e9
            if pre_s > 1e-6:
                self._dead_reckon(dict(velocity=velocity, omega=omega, duration_s=pre_s,
                                       stamp=msg.outcome.finished_at, evidence=msg.evidence),
                                  keep_reference=pre_end_ns == self.prev_stamp_ns)
            start_ns = pre_end_ns
            duration_s = max(0.0, (motion_end_ns - start_ns) / 1e9)
            if duration_s <= 1e-6:
                return
        pulse = dict(start_ns=start_ns, motion_end_ns=motion_end_ns, end_ns=end_ns,
                     velocity=velocity, omega=omega, duration_s=duration_s,
                     stamp=msg.outcome.finished_at, evidence=msg.evidence)
        if now_ns >= end_ns and (self.prev is None or now_ns - self.last_image_ns > 1_000_000_000 or self.last_rejected):
            self._apply_pending(now_ns)
            self._dead_reckon(pulse)
        else:
            self.pending.append(pulse)

    def _trim_pending(self, stamp_ns):
        uncovered = []
        for pulse in self.pending:
            motion_end_ns = pulse['motion_end_ns']
            if motion_end_ns <= stamp_ns:
                continue
            if pulse['start_ns'] < stamp_ns < motion_end_ns:
                pulse['duration_s'] = max(0.0, (motion_end_ns - stamp_ns) / 1e9)
                pulse['start_ns'] = stamp_ns
                if pulse['duration_s'] <= 1e-6:
                    continue
            uncovered.append(pulse)
        self.pending = uncovered

    def _apply_pending(self, now_ns, partial=False, stamp=None):
        self._trim_pending(self.last_vo_accept_ns)
        applied = False
        pending, self.pending = self.pending, []
        for pulse in pending:
            motion_end_ns = pulse['motion_end_ns']
            if partial and pulse['start_ns'] < now_ns < motion_end_ns:
                slice_s = (now_ns - pulse['start_ns']) / 1e9
                if slice_s > 1e-6:
                    self._dead_reckon({**pulse, 'duration_s': slice_s, 'stamp': stamp or pulse['stamp']})
                    applied = True
                rem_s = max(0.0, (motion_end_ns - now_ns) / 1e9)
                if rem_s > 1e-6:
                    pulse['start_ns'] = now_ns
                    pulse['duration_s'] = rem_s
                    self.pending.append(pulse)
                continue
            age_ns = now_ns - pulse['end_ns']
            if age_ns < 0:
                self.pending.append(pulse)
            elif age_ns <= 2_000_000_000:
                self._dead_reckon(pulse)
                applied = True
        return applied

    def _publish_status(self, stamp, evidence, validity_value, reason, pair_accepted=False, calibration_id=''):
        quality = OdometryStatus()
        quality.header.stamp = stamp
        quality.header.frame_id = 'odom'
        quality.evidence = evidence
        quality.calibration_id = calibration_id
        quality.pair_accepted = pair_accepted
        quality.validity.value = validity_value
        quality.rejection = quality.validity.reason = reason
        quality.covariance_calibrated = False
        quality.active_camera_x_known = math.isfinite(self.vo.cam_x)
        quality.active_camera_x_m = self.vo.cam_x if quality.active_camera_x_known else 0.
        self.pub_status.publish(quality)

    def _dead_reckon(self, pulse, keep_reference=False):
        omega, duration = pulse['omega'], pulse['duration_s']
        yaw_gain = self.yaw_pos_gain if omega >= 0 else self.yaw_neg_gain
        angle = omega * duration * yaw_gain
        distance = pulse['velocity'] * duration * self.forward_gain
        arc_x, arc_y = _arc(angle)
        c, s = math.cos(self.th), math.sin(self.th)
        self.x += distance * (c * arc_x - s * arc_y)
        self.y += distance * (s * arc_x + c * arc_y)
        self.th = math.atan2(math.sin(self.th + angle), math.cos(self.th + angle))
        if not keep_reference:
            self.reset_reference()
        self.last_rejected = True
        self._pose_uncertain = True
        self.publish(pulse['stamp'], rejected=True)
        self._publish_status(pulse['stamp'], pulse['evidence'], 2, 'COMMAND_DEAD_RECKONED')

    def reset_reference(self):
        self.prev = None
        self.prev_stamp_ns = 0

    def on_image(self, msg: Image):
        img = decode_bgr8(msg, self.vo.cam)
        if img is None:
            return
        now_stamp_ns = stamp_ns(msg)
        self.last_image_ns = now_stamp_ns
        state = "init"
        stationary = False
        def set_ref():
            self.prev = img.copy()
            self.prev_stamp_ns = now_stamp_ns
        if self.prev is None:
            if self._apply_pending(now_stamp_ns, partial=True, stamp=msg.header.stamp):
                set_ref()
                return
        else:
            m = self.vo.estimate(self.prev, img)
            if m is None:
                self.rejected += 1
                self.last_rejected = True
                if self._apply_pending(now_stamp_ns, partial=True, stamp=msg.header.stamp):
                    set_ref()
                    return
                if any(s < now_stamp_ns for s in self._attempts.values()):
                    set_ref()
                state = "rejected"
            else:
                self.last_vo_accept_ns = now_stamp_ns
                self.last_rejected = False
                self._pose_uncertain = False
                self._trim_pending(now_stamp_ns)
                stationary = math.hypot(m.dx, m.dy) < 5e-4 and abs(m.dtheta) < 1e-3
                if not stationary:
                    c, s = math.cos(self.th), math.sin(self.th)
                    self.x += c * m.dx - s * m.dy
                    self.y += s * m.dx + c * m.dy
                    self.th = math.atan2(math.sin(self.th + m.dtheta), math.cos(self.th + m.dtheta))
                self.ok += 1
                state = "ok"
        if state != "rejected":
            if self.prev is None or not stationary:
                self.prev = img.copy()
            self.prev_stamp_ns = now_stamp_ns
        self.publish(msg.header.stamp, rejected=(state == "rejected" or self._pose_uncertain))
        accepted = state == 'ok'
        self._publish_status(
            msg.header.stamp,
            self.frames.metadata.evidence,
            1 if accepted else (4 if state == 'rejected' else 0),
            '' if accepted else ('VO_REJECTED' if state == 'rejected' else 'INITIAL_REFERENCE'),
            pair_accepted=accepted,
            calibration_id=self.frames.metadata.calibration_id,
        )

    def publish(self, stamp, rejected=False):
        qz, qw = math.sin(self.th / 2), math.cos(self.th / 2)
        o = Odometry()
        o.header.stamp = stamp; o.header.frame_id = "odom"; o.child_frame_id = "base_footprint"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        if rejected:
            for index in [0, 7, 35]:
                o.pose.covariance[index] = 1e6
        self.pub.publish(o)
        self.last_stamp = stamp
        self._odom_published = True
        self._send_tf(stamp)


def main():
    rclpy.init()
    node = FloorOdomNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()


class FloorMosaic:
    def __init__(self, cam, resolution=.005, size_m=4., max_range=1.2, camera_x_m=None):
        if camera_x_m is None: camera_x_m = initial_camera_x_m()
        if resolution <= 0 or size_m <= 0 or max_range <= .06:
            raise ValueError('positive map size/resolution and range > 0.06 required')
        self.cam, self.resolution, self.max_range, self.cam_x = cam, resolution, max_range, camera_x_m
        self.n = int(math.ceil(size_m / resolution))
        self.origin = (-self.n * resolution / 2, -self.n * resolution / 2)
        self.mean = np.zeros((self.n, self.n, 3), np.float32)
        self.count = np.zeros((self.n, self.n), np.uint32)
        yy, xx = np.meshgrid(np.arange(self.n), np.arange(self.n), indexing='ij')
        self.world = np.stack([self.origin[0]+(xx.ravel()+.5)*resolution,
                               self.origin[1]+(yy.ravel()+.5)*resolution], 1)

    def update(self, image, pose):
        if image.shape != (self.cam.H, self.cam.W, 3):
            raise ValueError('image dimensions must match camera calibration')
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s], [s, c]])
        camera_origin = np.array([x, y]) + R @ np.array([self.cam_x, 0])
        local = (self.world - camera_origin) @ R
        r = np.linalg.norm(local, axis=1)
        candidate = np.flatnonzero((r > .06) & (r < self.max_range) & (local[:, 0] > 0))
        uv, ok = self.cam.floor_to_pixels(local[candidate])
        ok &= (uv[:, 0] >= 1) & (uv[:, 0] < self.cam.W - 2) & (uv[:, 1] >= 1) & (uv[:, 1] < self.cam.H - 2)
        ids = candidate[ok]; uv = uv[ok].astype(np.float32)
        sample = np.empty((len(ids), 3), np.float32)
        for start in range(0, len(ids), 30000):
            part = uv[start:start+30000]
            sample[start:start+len(part)] = cv2.remap(
                image, part[:, 0].reshape(-1, 1), part[:, 1].reshape(-1, 1), cv2.INTER_LINEAR).reshape(-1, 3)
        supported = np.any(sample != 0, axis=1)
        ids, sample = ids[supported], sample[supported]
        count = self.count.ravel(); mean = self.mean.reshape(-1, 3)
        overlap = count[ids] > 0
        ncc = None
        if overlap.sum() > 20:
            a = mean[ids[overlap]].mean(1); b = sample[overlap].mean(1)
            a = a - a.mean(); b = b - b.mean(); denom = np.linalg.norm(a) * np.linalg.norm(b)
            if denom > 1e-6: ncc = float(a @ b / denom)
        mean[ids] += (sample - mean[ids]) / (count[ids, None] + 1)
        count[ids] += 1
        return {'observed_cells': int(len(ids)), 'overlap_cells': int(overlap.sum()),
                'overlap_ncc': ncc, 'total_cells': int(np.count_nonzero(count))}

    def image(self):
        return np.clip(np.rint(self.mean), 0, 255).astype(np.uint8)


class FloorMosaicNode(Node):
    def __init__(self):
        super().__init__('z70_floor_mosaic')
        cam = FloorCamera.load(self.declare_parameter('calibration', str(front_calibration_path())).value)
        params = {name: float(self.declare_parameter(name, value).value) for name, value in
                  [('resolution', .005), ('size_m', 4.), ('max_range', 1.2), ('camera_x_m', initial_camera_x_m())]}
        self.mosaic = FloorMosaic(cam, **params)
        self.images = OrderedDict(); self.odoms = OrderedDict()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Image, '/z70/floor_mosaic', qos)
        self.mask_pub = self.create_publisher(Image, '/z70/floor_mosaic_mask', qos)
        self.info_pub = self.create_publisher(CameraInfo, '/z70/camera_info', qos)
        self.status_pub = self.create_publisher(String, '/z70/floor_mosaic_status', 10)
        self.frames = MatchedFrames(self, 'front', lambda m: self.add(self.images, m))
        self.create_subscription(Odometry, '/odom', lambda m: self.add(self.odoms, m), 10)
        self.create_subscription(CalibrationState, '/z70/chassis_calibration', self.on_calibration, qos)
        self._publish_outputs(self.get_clock().now().to_msg())

    def _camera_info(self, stamp):
        fx = fy = 0.01 / self.mosaic.resolution
        cx = cy = self.mosaic.n / 2.0
        info = CameraInfo()
        info.header.stamp = stamp; info.header.frame_id = 'odom'
        info.width = info.height = self.mosaic.n
        info.distortion_model = 'plumb_bob'
        info.d = [0., 0., 0., 0., 0.]
        info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
        return info

    def _publish_outputs(self, stamp):
        self.info_pub.publish(self._camera_info(stamp))
        for data, encoding, pub in [(self.mosaic.image(), 'bgr8', self.pub),
                                    ((self.mosaic.count > 0).astype(np.uint8) * 255, 'mono8', self.mask_pub)]:
            out = Image(); out.header.stamp = stamp; out.header.frame_id = 'odom'
            out.height, out.width = data.shape[:2]; out.encoding = encoding
            out.step = out.width * (3 if encoding == 'bgr8' else 1); out.data = data.tobytes(); pub.publish(out)

    def on_calibration(self, msg):
        value = calibrated_camera_x(msg)
        if value is not None: self.mosaic.cam_x = value

    def add(self, cache, msg):
        key = stamp_ns(msg); cache[key] = msg
        while len(cache) > 20: cache.popitem(last=False)
        if key not in self.images or key not in self.odoms: return
        image = self.images.pop(key); odom = self.odoms.pop(key)
        if odom.header.frame_id != 'odom' or odom.pose.covariance[0] > 1000: return
        view = decode_bgr8(image, self.mosaic.cam)
        if view is None: return
        pose = odom.pose.pose
        stats = self.mosaic.update(view, (pose.position.x, pose.position.y, quat_yaw(pose.orientation)))
        self._publish_outputs(image.header.stamp)
        stats.update(stamp_ns=key, resolution=self.mosaic.resolution, origin_xy=list(self.mosaic.origin),
                     frame_id='odom', width=self.mosaic.n, height=self.mosaic.n)
        self.status_pub.publish(String(data=json.dumps(stats)))


def mosaic_main():
    rclpy.init(); node = FloorMosaicNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.destroy_node(); rclpy.try_shutdown()
