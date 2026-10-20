"""Vendor LiDAR map decoder and ROS 2 publisher node."""
import json
import signal
import math
import threading
import time
from z70_interfaces.msg import MapSnapshot
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
from .decoder import decode_map
from .geometry import alignment, spatial_context, transform_pose
from .boundary import BoundaryConfig, compute_safe_interior
from .messages import grid_message, safe_grid_message, markers_message, pose_message


class MappingNode(Node):
    def __init__(self):
        super().__init__('z70_mapping')
        self.realign = self.declare_parameter('realign_on_refresh', True).value
        self.last_sequence = None
        defaults = BoundaryConfig()
        values = {}
        for name in defaults.__dataclass_fields__:
            default = getattr(defaults, name)
            value = self.declare_parameter('boundary.' + name,
                                           list(default) if isinstance(default, tuple) else default).value
            values[name] = tuple(value) if name == 'allowed_segments' else value
        self.boundary_config = BoundaryConfig(**values)
        self.boundary = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', qos)
        self.pub_safe_grid = self.create_publisher(OccupancyGrid, '/z70/map/safe_grid', qos)
        self.pub_pose = self.create_publisher(PoseStamped, '/z70/map/robot_pose', qos)
        self.pub_dock = self.create_publisher(PoseStamped, '/z70/map/dock_pose', qos)
        self.pub_markers = self.create_publisher(MarkerArray, '/z70/map/markers', qos)
        self.pub_status = self.create_publisher(String, '/z70/map/status', qos)
        self.pub_spatial = self.create_publisher(String, '/z70/map/spatial_context', qos)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.odom = (0.0, 0.0, 0.0); self.anchor = None; self.current = None
        self.pending = None; self.lock = threading.Lock()
        self._last_viz_at = 0.0
        self.create_subscription(MapSnapshot, '/z70/driver/map_snapshot', self.snapshot, qos)
        self.create_timer(.2, self.publish)

    def snapshot(self, msg):
        if not msg.evidence.valid or msg.evidence.stale:
            if self.current:
                self.current[2]['refresh_error'] = msg.evidence.reason or 'OBSERVATION_GAP'
            return
        try:
            m = decode_map(bytes(msg.raw))
            labels = {r.segment_id: r.name for r in msg.rooms}
            stamp = msg.evidence.received_at
            status = dict(source='vendor_lidar_slam_snapshot', saved_at_ms=stamp.sec*1000+stamp.nanosec/1e6,
                          map_index=msg.map_index, map_sequence=msg.map_sequence,
                          width=m.width, height=m.height, resolution_m=m.resolution,
                          origin_m=m.origin, blocks=m.blocks, room_labels=labels,
                          obstacle_record_counts=m.obstacle_counts,
                          active_map_verified=msg.active_floor_verified,
                          current_pose_verified=msg.current_pose_verified,
                          refresh_error=None)
            with self.lock: self.pending = (m, labels, status)
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('Map decode failed: INVALID_MAP', throttle_duration_sec=20)

    def on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint':
            return
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        self.odom = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z)))

    def publish(self):
        with self.lock:
            new, self.pending = self.pending, None
        if new:
            self.current = new
            m, labels, status = new
            if self.realign and status['map_sequence'] != self.last_sequence:
                self.anchor = None
                self.last_sequence = status['map_sequence']
        if self.current is None:
            return
        m, labels, status = self.current
        if self.anchor is None and self.odom is not None and m.robot is not None:
            self.anchor = alignment(m.robot, self.odom)
        now_msg = self.get_clock().now().to_msg()
        if self.anchor:
            t = TransformStamped(); t.header.frame_id = 'map'; t.child_frame_id = 'odom'
            t.header.stamp = now_msg
            t.transform.translation.x, t.transform.translation.y = self.anchor[:2]
            t.transform.rotation.z = math.sin(self.anchor[2] / 2)
            t.transform.rotation.w = math.cos(self.anchor[2] / 2)
            self.tf.sendTransform(t)
        robot_map = transform_pose(self.anchor, self.odom) if self.anchor is not None else m.robot
        now_mono = time.monotonic()
        if new or now_mono - self._last_viz_at >= 2.0:
            self.boundary = compute_safe_interior(m, labels, self.boundary_config, robot_map)
            self._last_viz_at = now_mono
            saved_at = status['saved_at_ms']
            now_ms = int(now_msg.sec * 1000 + now_msg.nanosec // 1_000_000)
            self.pub_map.publish(grid_message(m, now_ms, load_at=saved_at))
            self.pub_safe_grid.publish(safe_grid_message(self.boundary, now_ms, load_at=saved_at))
            self.pub_markers.publish(markers_message(m, labels, now_ms, boundary_result=self.boundary))
            if m.robot:
                self.pub_pose.publish(pose_message(m.robot, now_ms))
            if m.dock:
                self.pub_dock.publish(pose_message(m.dock, now_ms))
        contract = self.boundary.summary()
        ctx = spatial_context(m, labels, robot_map)
        ctx['boundary_contract'] = contract
        self.pub_spatial.publish(String(data=json.dumps(ctx)))
        diagnostic = dict(status, boundary_contract=contract,
                          cache_age_s=time.time() - status['saved_at_ms']/1000.,
                          map_to_odom=self.anchor)
        self.pub_status.publish(String(data=json.dumps(diagnostic)))


def main():
    rclpy.init(); node = MappingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node(); rclpy.try_shutdown()
