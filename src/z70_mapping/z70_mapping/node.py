"""Vendor LiDAR map decoder and ROS 2 publisher node."""
import json
import signal
import math
import threading
import time
from z70_interfaces.msg import GarmentObservations, MapSnapshot
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
from .decoder import alignment, decode_map, quat_yaw, spatial_context, transform_pose
from .dynamic_layer import SessionWorldMap
from .boundary import BoundaryConfig, compute_safe_interior
from .messages import (
    dynamic_grid_message,
    grid_message,
    markers_message,
    pose_message,
    safe_grid_message,
    world_markers_message,
)


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
        self.world = SessionWorldMap()
        self._world_dirty = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', qos)
        self.pub_safe_grid = self.create_publisher(OccupancyGrid, '/z70/map/safe_grid', qos)
        self.pub_dynamic_grid = self.create_publisher(OccupancyGrid, '/z70/map/dynamic_obstacles', qos)
        self.pub_pose = self.create_publisher(PoseStamped, '/z70/map/robot_pose', qos)
        self.pub_dock = self.create_publisher(PoseStamped, '/z70/map/dock_pose', qos)
        self.pub_markers = self.create_publisher(MarkerArray, '/z70/map/markers', qos)
        self.pub_world_markers = self.create_publisher(MarkerArray, '/z70/world/markers', qos)
        self.pub_world_objects = self.create_publisher(String, '/z70/world/objects', qos)
        self.pub_status = self.create_publisher(String, '/z70/map/status', qos)
        self.pub_spatial = self.create_publisher(String, '/z70/map/spatial_context', qos)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.odom = (0.0, 0.0, 0.0); self.anchor = None; self.current = None
        self.pending = None; self.lock = threading.Lock()
        self._last_viz_at = 0.0
        self.create_subscription(MapSnapshot, '/z70/driver/map_snapshot', self.snapshot, qos)
        self.create_subscription(GarmentObservations, '/z70/garments/observations', self.on_garments, qos)
        self.create_subscription(String, '/z70/world/update', self.on_world_update, 10)
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
            session_id = bytes(getattr(getattr(msg.evidence, 'session_id', None), 'uuid', b'\x01' * 16))
            status = dict(source='vendor_lidar_slam_snapshot', saved_at_ms=stamp.sec*1000+stamp.nanosec/1e6,
                          session_id=session_id,
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
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, quat_yaw(msg.pose.pose.orientation))

    def _current_robot_pose(self):
        if self.current is None:
            return None
        m = self.current[0]
        return transform_pose(self.anchor, self.odom) if self.anchor is not None else m.robot

    def on_garments(self, msg):
        if not getattr(msg, 'model_available', False) or not getattr(msg, 'geofence_active', False):
            return
        pose = self._current_robot_pose()
        if pose is None or pose[2] is None:
            return
        with self.lock:
            if self.world.ingest_observations(msg, pose):
                self._world_dirty = True

    def on_world_update(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            if self.world.apply_event(payload):
                self._world_dirty = True

    def publish(self):
        with self.lock:
            new, self.pending = self.pending, None
            dirty, self._world_dirty = self._world_dirty, False
        if new:
            self.current = new
            m, labels, status = new
            self.world.bind_session(
                status.get('session_id', b'\x01' * 16),
                status['map_index'],
                m.origin,
                m.width,
                m.height,
                m.resolution,
            )
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
        if new or dirty or now_mono - self._last_viz_at >= 2.0:
            dyn_mask = self.world.obstacle_mask(
                m.width, m.height, m.origin, m.resolution,
                include_completed=False,
                protect_xy=robot_map[:2] if robot_map is not None else None,
            )
            self.boundary = compute_safe_interior(m, labels, self.boundary_config, robot_map, dynamic_obstacles=dyn_mask)
            self._last_viz_at = now_mono
            saved_at = status['saved_at_ms']
            now_ms = int(now_msg.sec * 1000 + now_msg.nanosec // 1_000_000)
            self.pub_map.publish(grid_message(m, now_ms, load_at=saved_at))
            self.pub_safe_grid.publish(safe_grid_message(self.boundary, now_ms, load_at=saved_at))
            self.pub_dynamic_grid.publish(dynamic_grid_message(m, self.world, now_ms, load_at=saved_at))
            self.pub_markers.publish(markers_message(m, labels, now_ms, boundary_result=self.boundary))
            self.pub_world_markers.publish(world_markers_message(self.world, now_ms))
            self.pub_world_objects.publish(String(data=json.dumps(self.world.summary(robot_map[:2] if robot_map else None))))
            if m.robot:
                self.pub_pose.publish(pose_message(m.robot, now_ms))
            if m.dock:
                self.pub_dock.publish(pose_message(m.dock, now_ms))
        contract = self.boundary.summary()
        ctx = spatial_context(m, labels, robot_map)
        ctx['boundary_contract'] = contract
        self.pub_spatial.publish(String(data=json.dumps(ctx)))
        diagnostic = {k: v for k, v in status.items() if k != 'session_id'}
        diagnostic.update(boundary_contract=contract,
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
