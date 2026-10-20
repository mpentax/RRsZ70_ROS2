"""Vendor LiDAR map decoder and ROS 2 publisher node."""
import json
import math
import signal
import threading
import time
import cv2
import numpy as np
from builtin_interfaces.msg import Time
from z70_interfaces.msg import GarmentObservations, MapSnapshot
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from .decoder import alignment, decode_map, quat_yaw, spatial_context, transform_pose
from .dynamic_layer import SessionWorldMap
from .boundary import BoundaryConfig, compute_safe_interior


def stamp(ms):
    return Time(sec=int(ms // 1000), nanosec=int(ms % 1000) * 1000000)


def pose_message(pose, saved_at):
    m = PoseStamped()
    m.header.frame_id = 'map'; m.header.stamp = stamp(saved_at)
    m.pose.position.x, m.pose.position.y = pose[:2]
    yaw = pose[2] if pose[2] is not None else 0.
    m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return m


def _occupancy_grid(meta, data, saved_at, load_at=None):
    g = OccupancyGrid()
    g.header.frame_id = 'map'; g.header.stamp = stamp(saved_at)
    g.info.map_load_time = stamp(saved_at if load_at is None else load_at)
    g.info.width, g.info.height, g.info.resolution = meta.width, meta.height, meta.resolution
    g.info.origin.position.x, g.info.origin.position.y = meta.origin
    g.info.origin.orientation.w = 1.
    g.data = data.reshape(-1).tolist()
    return g


def grid_message(m, saved_at, load_at=None):
    return _occupancy_grid(m, m.occupancy, saved_at, load_at)


def safe_grid_message(boundary_result, saved_at, load_at=None):
    return _occupancy_grid(boundary_result, boundary_result.safe_grid, saved_at, load_at)


def dynamic_grid_message(m, dynamic_layer, saved_at, load_at=None):
    data = (
        dynamic_layer.occupancy_grid_data(m.width, m.height, m.origin, m.resolution)
        if hasattr(dynamic_layer, 'occupancy_grid_data')
        else np.asarray(dynamic_layer, dtype=np.int8)
    )
    return _occupancy_grid(m, data, saved_at, load_at)


def world_markers_message(dynamic_layer, saved_at):
    result = MarkerArray()
    clear = Marker(); clear.action = Marker.DELETEALL
    clear.header.frame_id = 'map'; clear.header.stamp = stamp(saved_at)
    result.markers.append(clear)
    entries = list(dynamic_layer.entries.values()) if hasattr(dynamic_layer, 'entries') else list(dynamic_layer or [])
    for idx, entry in enumerate(entries):
        mobility = getattr(entry, 'mobility', entry.get('mobility', 'stationary') if isinstance(entry, dict) else 'stationary')
        relevance = getattr(entry, 'relevance', entry.get('relevance', 'ignored') if isinstance(entry, dict) else 'ignored')
        is_obs = getattr(entry, 'is_obstacle', mobility == 'stationary' or relevance in ('deferred', 'ignored'))
        category = getattr(entry, 'category', entry.get('category', 'object') if isinstance(entry, dict) else 'object')
        x = float(getattr(entry, 'x', entry['x'] if isinstance(entry, dict) else 0.))
        y = float(getattr(entry, 'y', entry['y'] if isinstance(entry, dict) else 0.))
        r = float(getattr(entry, 'radius_m', entry.get('radius_m', 0.08) if isinstance(entry, dict) else 0.08))
        if relevance == 'completed':
            color = (0.15, 0.90, 0.35, 0.90)
            ns = 'world_completed'
        elif is_obs:
            color = (0.95, 0.20, 0.20, 0.90)
            ns = 'world_obstacles'
        elif mobility == 'draggable':
            color = (0.20, 0.75, 1.0, 0.95)
            ns = 'world_objectives'
        else:
            color = (1.0, 0.65, 0.10, 0.95)
            ns = 'world_objectives'
        cyl = Marker(); cyl.header = clear.header; cyl.ns = ns; cyl.id = idx
        cyl.type = Marker.CYLINDER; cyl.action = Marker.ADD; cyl.pose.orientation.w = 1.
        cyl.frame_locked = True
        cyl.pose.position.x, cyl.pose.position.y, cyl.pose.position.z = x, y, 0.04
        cyl.scale.x = cyl.scale.y = max(0.08, 2.0 * r); cyl.scale.z = 0.08
        cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = color
        result.markers.append(cyl)
        lbl = Marker(); lbl.header = clear.header; lbl.ns = ns + '_labels'; lbl.id = idx
        lbl.type = Marker.TEXT_VIEW_FACING; lbl.action = Marker.ADD; lbl.pose.orientation.w = 1.
        lbl.frame_locked = True
        lbl.pose.position.x, lbl.pose.position.y, lbl.pose.position.z = x, y, 0.18
        lbl.scale.z = 0.12
        lbl.color.r, lbl.color.g, lbl.color.b, lbl.color.a = color
        lbl.text = f'{category} [{mobility}:{relevance}]'
        result.markers.append(lbl)
    return result


def markers_message(m, labels, saved_at, waypoints=None, boundary_result=None):
    result = MarkerArray()
    clear = Marker(); clear.action = Marker.DELETEALL
    clear.header.frame_id = 'map'; clear.header.stamp = stamp(saved_at)
    result.markers.append(clear)
    def marker(ns, ident, kind, color):
        x = Marker(); x.header = clear.header; x.ns = ns; x.id = ident
        x.type = kind; x.action = Marker.ADD; x.pose.orientation.w = 1.
        x.frame_locked = True
        x.color.r, x.color.g, x.color.b, x.color.a = color
        result.markers.append(x)
        return x
    def text_marker(ns, ident, color, xy, z, scale, content):
        t = marker(ns, ident, Marker.TEXT_VIEW_FACING, color)
        t.pose.position.x, t.pose.position.y, t.pose.position.z = xy[0], xy[1], z
        t.scale.z = scale; t.text = content
    def add_polygon(lines, poly, z):
        for a, b in zip(poly, poly[1:] + poly[:1]):
            lines.points.extend([Point(x=a[0], y=a[1], z=z), Point(x=b[0], y=b[1], z=z)])
    for segment, room in m.rooms.items():
        color = ((segment * .31) % .7 + .3, (segment * .51) % .7 + .3, .8, 1.)
        text_marker('rooms', segment, color, room['centroid'], .15, .18, labels.get(segment, f'Segment {segment}'))
        lines = marker('room_polygons', segment, Marker.LINE_LIST, color)
        lines.scale.x = .025
        for poly in room['polygons']:
            add_polygon(lines, poly, .025)
    for ns, p, color in [('dock', m.dock, (0., 1., .5, 1.)), ('vendor_robot', m.robot, (1., .35, .1, 1.))]:
        if p is None:
            continue
        x = marker(ns, 0, Marker.ARROW if p[2] is not None else Marker.SPHERE, color)
        x.pose = pose_message(p, saved_at).pose
        x.scale.x, x.scale.y, x.scale.z = .35, .12, .12
        if ns == 'dock':
            text_marker(ns + '_label', 0, color, p[:2], .3, .14, 'Dock (vendor map)')
    for ns, points, color, z in [
        ('path', m.path, (.22, .74, .97, .9), .03),
        ('goto_path', m.goto_path, (.98, .75, .14, .95), .035),
    ]:
        if points:
            line = marker(ns, 0, Marker.LINE_STRIP, color)
            line.scale.x = .03
            line.points = [Point(x=pt[0], y=pt[1], z=z) for pt in points]
    if m.virtual_walls:
        lines = marker('virtual_walls', 0, Marker.LINE_LIST, (.94, .27, .27, .95))
        lines.scale.x = .04
        for x0, y0, x1, y1 in m.virtual_walls:
            lines.points.extend([Point(x=x0, y=y0, z=.05), Point(x=x1, y=y1, z=.05)])
    if m.zones:
        lines = marker('zones', 0, Marker.LINE_LIST, (.98, .45, .09, .9))
        lines.scale.x = .035
        for poly in m.zones:
            add_polygon(lines, poly, .04)
    if m.carpet is not None and np.any(m.carpet):
        contours, _ = cv2.findContours(m.carpet.astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        lines = marker('carpet', 0, Marker.LINE_LIST, (.65, .55, .98, .85))
        lines.scale.x = .02
        for contour in contours:
            poly = [(m.origin[0] + (int(x) + .5) * m.resolution,
                     m.origin[1] + (int(y) + .5) * m.resolution)
                    for x, y in contour.reshape(-1, 2)]
            add_polygon(lines, poly, .015)
    if boundary_result is not None:
        waypoints = boundary_result.waypoints if waypoints is None else waypoints
        text_marker('boundary_contract', 0, (.2, 1., .6, 1.), m.origin, .4, .16,
                    f'Interior: {int(boundary_result.reachable_mask.sum())} reachable cells; '
                    f'clearance {boundary_result.config.clearance_m:.3f} m; '
                    f'lost rooms {boundary_result.lost_rooms}')
    for i, waypoint in enumerate(waypoints or []):
        point = marker('patrol_waypoints', i, Marker.SPHERE, (.1, 1., .8, 1.))
        point.pose.position.x, point.pose.position.y = waypoint['x'], waypoint['y']
        point.pose.position.z = .08
        point.scale.x = point.scale.y = point.scale.z = .12
        text_marker('patrol_waypoint_labels', i, (.1, 1., .8, 1.), (waypoint['x'], waypoint['y']), .22, .12,
                    f"{i + 1}: S{waypoint['segment_id']}")
    return result


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
