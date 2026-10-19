"""Floor VO node: /camera/front/image_raw -> /odom (nav_msgs/Odometry) + TF odom -> base_footprint.

Reference-to-current floor odometry with a 5 Hz TF heartbeat and command-model dead-reckoning fallback.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from tf2_ros import TransformBroadcaster

from z70_odometry.floor_vo import FloorCamera, FloorOdometry
from z70_odometry.calibration_state import calibrated_camera_x
from z70_interfaces.msg import CalibrationState, CommandEvent, OdometryStatus
from rclpy.qos import QoSProfile, DurabilityPolicy
from z70_odometry.resources import front_calibration_path, initial_camera_x_m
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

    @staticmethod
    def _ns(stamp):
        return stamp.sec * 10**9 + stamp.nanosec

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
                self._attempts[ident] = self._ns(msg.evidence.source_at)
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
        end_ns = max(self._ns(msg.outcome.finished_at), motion_end_ns)
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

    def _dead_reckon(self, pulse, keep_reference=False):
        omega, duration = pulse['omega'], pulse['duration_s']
        yaw_gain = self.yaw_pos_gain if omega >= 0 else self.yaw_neg_gain
        angle = omega * duration * yaw_gain
        distance = pulse['velocity'] * duration * self.forward_gain
        arc_x = math.sin(angle) / angle if abs(angle) > 1e-8 else 1.0
        arc_y = (1.0 - math.cos(angle)) / angle if abs(angle) > 1e-8 else 0.0
        c, s = math.cos(self.th), math.sin(self.th)
        self.x += distance * (c * arc_x - s * arc_y)
        self.y += distance * (s * arc_x + c * arc_y)
        self.th = math.atan2(math.sin(self.th + angle), math.cos(self.th + angle))
        if not keep_reference:
            self.reset_reference()
        self.last_rejected = True
        self._pose_uncertain = True
        self.publish(pulse['stamp'], rejected=True)
        quality = OdometryStatus()
        quality.header.stamp = pulse['stamp']
        quality.header.frame_id = 'odom'
        quality.evidence = pulse['evidence']
        quality.pair_accepted = False
        quality.validity.value = 2
        quality.rejection = quality.validity.reason = 'COMMAND_DEAD_RECKONED'
        quality.covariance_calibrated = False
        quality.active_camera_x_known = math.isfinite(self.vo.cam_x)
        quality.active_camera_x_m = self.vo.cam_x if quality.active_camera_x_known else 0.
        self.pub_status.publish(quality)

    def reset_reference(self):
        self.prev = None
        self.prev_stamp_ns = 0

    def on_image(self, msg: Image):
        if msg.encoding != "bgr8":
            return
        if msg.width != self.vo.cam.W or msg.height != self.vo.cam.H:
            return
        if msg.step < msg.width * 3 or len(msg.data) != msg.height * msg.step:
            return
        stamp_ns = self._ns(msg.header.stamp)
        self.last_image_ns = stamp_ns
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        status = {"state": "init"}
        if self.prev is None:
            if self._apply_pending(stamp_ns, partial=True, stamp=msg.header.stamp):
                self.prev = img.copy()
                self.prev_stamp_ns = stamp_ns
                return
        elif self.prev is not None:
            m = self.vo.estimate(self.prev, img)
            if m is None:
                self.rejected += 1
                self.last_rejected = True
                if self._apply_pending(stamp_ns, partial=True, stamp=msg.header.stamp):
                    self.prev = img.copy()
                    self.prev_stamp_ns = stamp_ns
                    return
                if any(s < stamp_ns for s in self._attempts.values()):
                    self.prev = img.copy()
                    self.prev_stamp_ns = stamp_ns
                status = {"state": "rejected", "ok": self.ok, "rejected": self.rejected}
            else:
                self.last_vo_accept_ns = stamp_ns
                self.last_rejected = False
                self._pose_uncertain = False
                self._trim_pending(stamp_ns)
                stationary = math.hypot(m.dx, m.dy) < 5e-4 and abs(m.dtheta) < 1e-3
                if not stationary:
                    c, s = math.cos(self.th), math.sin(self.th)
                    self.x += c * m.dx - s * m.dy
                    self.y += s * m.dx + c * m.dy
                    self.th = math.atan2(math.sin(self.th + m.dtheta), math.cos(self.th + m.dtheta))
                self.ok += 1
                status = {"state": "ok", "stationary": stationary, "step": {"dx": m.dx, "dy": m.dy, "dtheta_deg": m.dtheta_deg},
                          "inliers": m.inliers, "rms_mm": m.rms_m * 1000, "ok": self.ok, "rejected": self.rejected}
        if status["state"] != "rejected":
            if self.prev is None or not status.get("stationary", False):
                self.prev = img.copy()
            self.prev_stamp_ns = stamp_ns
        status['camera_x_m']=self.vo.cam_x
        status["stamp_ns"] = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.publish(msg.header.stamp, rejected=(status["state"] == "rejected" or self._pose_uncertain))
        quality=OdometryStatus()
        quality.header=msg.header
        quality.evidence=self.frames.metadata.evidence
        quality.calibration_id=self.frames.metadata.calibration_id
        quality.pair_accepted=status['state']=='ok'
        quality.validity.value=1 if quality.pair_accepted else (4 if status['state']=='rejected' else 0)
        quality.rejection='' if quality.pair_accepted else ('VO_REJECTED' if status['state']=='rejected' else 'INITIAL_REFERENCE')
        quality.validity.reason=quality.rejection
        quality.covariance_calibrated=False
        quality.active_camera_x_known=math.isfinite(self.vo.cam_x)
        quality.active_camera_x_m=self.vo.cam_x if quality.active_camera_x_known else 0.
        self.pub_status.publish(quality)

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
