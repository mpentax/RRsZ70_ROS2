"""Vendor LiDAR map decoder, monocular depth worker/node, and ROS 2 publisher node."""
import argparse
import contextlib
from dataclasses import dataclass, field
import hashlib
import io
import json
import math
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time
import cv2
import numpy as np
from .boundary import BoundaryConfig, SessionWorldMap, compute_safe_interior

try:
    from builtin_interfaces.msg import Time
    from z70_interfaces.msg import GarmentObservations, MapSnapshot
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import Point, PoseStamped, TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import TransformBroadcaster
    LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
except ImportError:
    Node = object
    LATCHED = None


@dataclass
class VendorMap:
    width: int
    height: int
    origin: tuple
    pixels: np.ndarray
    occupancy: np.ndarray
    robot: tuple | None
    dock: tuple | None
    rooms: dict
    blocks: dict
    obstacle_counts: dict
    resolution: float = .05
    carpet: np.ndarray | None = None
    path: list[tuple[float, float]] = field(default_factory=list)
    goto_path: list[tuple[float, float]] = field(default_factory=list)
    virtual_walls: list[tuple[float, float, float, float]] = field(default_factory=list)
    zones: list[list[tuple[float, float]]] = field(default_factory=list)


def decode_map(raw):
    if (len(raw) < 40 or len(raw) > 32 * 1024 * 1024 or raw[:2] != b'rr'
            or struct.unpack_from('<H', raw, 2)[0] != 20
            or struct.unpack_from('<I', raw, 4)[0] != len(raw) - 20
            or hashlib.sha1(raw[:-20]).digest() != raw[-20:]):
        raise ValueError('Invalid V1 integrity')
    offset, blocks, image, robot, dock, digest = 20, {}, None, None, None, False
    obstacle_counts = {}
    path, goto_path, virtual_walls, zones = [], [], [], []
    carpet_raw = None
    while offset < len(raw):
        if sum(blocks.values()) >= 1024 or offset + 8 > len(raw):
            raise ValueError('Invalid V1 blocks')
        kind, header, length = struct.unpack_from('<HHI', raw, offset)
        data = offset + header
        if header < 8 or header > 256 or data + length > len(raw):
            raise ValueError('Invalid V1 block extent')
        blocks[kind] = blocks.get(kind, 0) + 1
        if kind == 1024:
            if length != 20 or data + length != len(raw):
                raise ValueError('Invalid V1 digest block')
            digest = True
            break
        if kind in (12, 15, 16) and header >= 10:
            obstacle_counts[kind] = struct.unpack_from('<H', raw, offset + 8)[0]
        if kind in (1, 8):
            if length not in (8, 12):
                raise ValueError('Invalid V1 position')
            x, y = struct.unpack_from('<ii', raw, data)
            angle = math.radians(struct.unpack_from('<i', raw, data + 8)[0]) if length == 12 else None
            pose = (x / 1000., y / 1000., angle)
            if kind == 1:
                dock = pose
            else:
                robot = pose
        if kind in (3, 9) and length >= 4 and length % 4 == 0:
            points = [(x / 1000., y / 1000.)
                      for x, y in struct.iter_unpack('<HH', raw[data:data + length])]
            (path if kind == 3 else goto_path).extend(points)
        if kind == 10 and length >= 8 and length % 8 == 0:
            virtual_walls.extend(tuple(v / 1000. for v in record)
                                 for record in struct.iter_unpack('<4H', raw[data:data + length]))
        if kind in (18, 19, 47, 53):
            size, fmt = (16, '<8H') if kind in (18, 19) else (20, '<2H8H')
            if length >= size and length % size == 0:
                for record in struct.iter_unpack(fmt, raw[data:data + length]):
                    corners = record[-8:]
                    zones.append([(corners[i] / 1000., corners[i + 1] / 1000.)
                                  for i in range(0, 8, 2)])
        if kind == 17:
            carpet_raw = raw[data:data + length]
        if kind == 2:
            if image is not None or header < 24:
                raise ValueError('Invalid V1 image header')
            top, left, height, width = struct.unpack_from('<iiII', raw, data - 16)
            if not width or not height or max(width, height) > 1024 or width * height != length:
                raise ValueError('Invalid V1 image size')
            image = (width, height, (left * .05, top * .05),
                     np.frombuffer(raw, np.uint8, length, data).reshape(height, width).copy())
        offset = data + length
    if not digest or image is None:
        raise ValueError('Incomplete V1 map')
    width, height, origin, pixels = image
    carpet = None
    if carpet_raw is not None and len(carpet_raw) == width * height:
        carpet = np.frombuffer(carpet_raw, np.uint8).reshape(height, width) > 0
    floor = (pixels == 255) | ((pixels & 7) == 7)
    wall = (pixels & 7) == 1
    occupancy = np.full(pixels.shape, -1, np.int8)
    occupancy[wall] = 100
    occupancy[floor] = 0
    rooms = {}
    segmented = floor & (pixels != 255) & (pixels != 7)
    for segment in np.unique(pixels[segmented] >> 3):
        mask = (segmented & ((pixels >> 3) == segment)).astype(np.uint8)
        ys, xs = np.nonzero(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        polygons = [[(origin[0] + (int(x) + .5) * .05, origin[1] + (int(y) + .5) * .05)
                     for x, y in c.reshape(-1, 2)] for c in contours if len(c) >= 3]
        rooms[int(segment)] = {'centroid': (origin[0] + (float(xs.mean()) + .5) * .05,
                                           origin[1] + (float(ys.mean()) + .5) * .05),
                               'polygons': polygons, 'area_m2': len(xs) * .0025}
    return VendorMap(width, height, origin, pixels, occupancy, robot, dock, rooms, blocks,
                     obstacle_counts, carpet=carpet, path=path, goto_path=goto_path,
                     virtual_walls=virtual_walls, zones=zones)


def quat_yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def transform_pose(t, p):
    x, y, yaw = t
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * p[0] - s * p[1], y + s * p[0] + c * p[1],
            math.atan2(math.sin(yaw + p[2]), math.cos(yaw + p[2])))


def alignment(vendor, odom):
    if vendor[2] is None:
        return None
    yaw = math.atan2(math.sin(vendor[2] - odom[2]), math.cos(vendor[2] - odom[2]))
    c, s = math.cos(yaw), math.sin(yaw)
    return (vendor[0] - c * odom[0] + s * odom[1],
            vendor[1] - s * odom[0] - c * odom[1], yaw)


def room_names(mapping, rooms):
    labels = {str(r['id']): str(r['name']) for r in rooms}
    return {int(r[0]): labels[str(r[1])] for r in mapping
            if isinstance(r, list) and len(r) >= 2 and str(r[1]) in labels}


def spatial_context(m, labels, robot_pose_map=None, boundary_result=None) -> dict:
    """Compute current room, relative room/dock bearings, and 8-sector clearance rays."""
    pose = robot_pose_map if robot_pose_map is not None else m.robot

    def cell(x, y):
        return (int(math.floor((x - m.origin[0]) / m.resolution)),
                int(math.floor((y - m.origin[1]) / m.resolution)))

    def in_bounds(gx, gy):
        return 0 <= gx < m.width and 0 <= gy < m.height

    def relative(x, y):
        if pose is None:
            return None, None
        dx, dy = x - pose[0], y - pose[1]
        bearing = None
        if pose[2] is not None:
            angle = math.atan2(dy, dx) - pose[2]
            bearing = round(math.atan2(math.sin(angle), math.cos(angle)), 3)
        return round(math.hypot(dx, dy), 3), bearing

    current_room = None
    if pose is not None:
        gx, gy = cell(pose[0], pose[1])
        if in_bounds(gx, gy):
            pixel = int(m.pixels[gy, gx])
            segment = None
            if (pixel & 7) == 7 and pixel not in (7, 255):
                segment = pixel >> 3
            elif m.rooms and m.occupancy[gy, gx] == 0:
                distance, nearest = min(
                    (math.hypot(room['centroid'][0] - pose[0], room['centroid'][1] - pose[1]), seg)
                    for seg, room in m.rooms.items())
                if distance <= 1.5:
                    segment = nearest
            if segment is not None:
                current_room = {'segment_id': segment, 'name': labels.get(segment, f'Segment {segment}')}

    rooms = []
    wp_by_seg = {}
    if boundary_result is not None:
        for w in getattr(boundary_result, 'waypoints', ()):
            s = int(w.get('segment_id', -1))
            if s not in wp_by_seg or float(w.get('clearance_m', 0.)) > float(wp_by_seg[s].get('clearance_m', 0.)):
                wp_by_seg[s] = w
    for seg, room in sorted(m.rooms.items()):
        x, y = room['centroid']
        distance, bearing = relative(x, y)
        entry = {'segment_id': seg, 'name': labels.get(seg, f'Segment {seg}'),
                 'centroid': [round(x, 3), round(y, 3)], 'area_m2': round(room['area_m2'], 3),
                 'distance_m': distance, 'bearing_rad': bearing}
        if boundary_result is not None:
            wp = wp_by_seg.get(int(seg))
            entry['safe_waypoint'] = [round(float(wp['x']), 3), round(float(wp['y']), 3)] if wp else None
            entry['reachable'] = wp is not None
        rooms.append(entry)
    if pose is not None:
        rooms.sort(key=lambda room: room['distance_m'])

    dock = None
    if m.dock is not None:
        distance, bearing = relative(m.dock[0], m.dock[1])
        dock = {'x': round(m.dock[0], 3), 'y': round(m.dock[1], 3), 'yaw': m.dock[2],
                'distance_m': distance, 'bearing_rad': bearing}

    sectors = {}
    if pose is not None:
        base_yaw = pose[2] if pose[2] is not None else 0.
        step, max_range = m.resolution * .5, 5.
        for name, angle in [('front', 0.), ('front_left', math.pi / 4), ('left', math.pi / 2),
                            ('rear_left', 3 * math.pi / 4), ('rear', math.pi),
                            ('rear_right', -3 * math.pi / 4), ('right', -math.pi / 2),
                            ('front_right', -math.pi / 4)]:
            dx, dy = math.cos(base_yaw + angle), math.sin(base_yaw + angle)
            distance = max_range
            for i in range(int(math.ceil(max_range / step)) + 1):
                distance = min(i * step, max_range)
                gx, gy = cell(pose[0] + distance * dx, pose[1] + distance * dy)
                if not in_bounds(gx, gy) or m.occupancy[gy, gx] == 100:
                    break
            sectors[name] = round(distance, 3)
    robot_pose = ({'x': round(pose[0], 3), 'y': round(pose[1], 3),
                   'yaw': round(pose[2], 3) if pose[2] is not None else None}
                  if pose is not None else None)
    res = {'robot_pose_map': robot_pose, 'current_room': current_room,
           'rooms': rooms, 'dock': dock, 'clearance_sectors_m': sectors}
    if boundary_result is not None and hasattr(boundary_result, 'summary'):
        res['boundary_contract'] = boundary_result.summary()
    return res


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
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.pub_safe_grid = self.create_publisher(OccupancyGrid, '/z70/map/safe_grid', LATCHED)
        self.pub_dynamic_grid = self.create_publisher(OccupancyGrid, '/z70/map/dynamic_obstacles', LATCHED)
        self.pub_pose = self.create_publisher(PoseStamped, '/z70/map/robot_pose', LATCHED)
        self.pub_dock = self.create_publisher(PoseStamped, '/z70/map/dock_pose', LATCHED)
        self.pub_markers = self.create_publisher(MarkerArray, '/z70/map/markers', LATCHED)
        self.pub_world_markers = self.create_publisher(MarkerArray, '/z70/world/markers', LATCHED)
        self.pub_world_objects = self.create_publisher(String, '/z70/world/objects', LATCHED)
        self.pub_status = self.create_publisher(String, '/z70/map/status', LATCHED)
        self.pub_spatial = self.create_publisher(String, '/z70/map/spatial_context', LATCHED)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.odom = (0.0, 0.0, 0.0); self.anchor = None; self.current = None
        self.pending = None; self.lock = threading.Lock()
        self._last_viz_at = 0.0
        self.create_subscription(MapSnapshot, '/z70/driver/map_snapshot', self.snapshot, LATCHED)
        self.create_subscription(GarmentObservations, '/z70/garments/observations', self.on_garments, LATCHED)
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
            with self.lock:
                self.pending = (m, labels, status)
                self.world.bind_session(session_id, msg.map_index, m.origin, m.width, m.height, m.resolution)
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning('Map decode failed: INVALID_MAP', throttle_duration_sec=20)

    def on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint':
            return
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, quat_yaw(msg.pose.pose.orientation))

    def _current_robot_pose(self):
        state = self.current or self.pending
        if state is None:
            return None
        m = state[0]
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
        ctx = spatial_context(m, labels, robot_map, boundary_result=self.boundary)
        contract = ctx['boundary_contract']
        self.pub_spatial.publish(String(data=json.dumps(ctx)))
        diagnostic = {k: v for k, v in status.items() if k != 'session_id'}
        diagnostic.update(boundary_contract=contract,
                          cache_age_s=time.time() - status['saved_at_ms']/1000.,
                          map_to_odom=self.anchor)
        self.pub_status.publish(String(data=json.dumps(diagnostic)))


def _spin(node_cls):
    rclpy.init(); node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if hasattr(node, 'close'):
            node.close()
        node.destroy_node(); rclpy.try_shutdown()


def main():
    _spin(MappingNode)


def camera_rays(cal, width, height):
    u, v = np.meshgrid((np.arange(width) + .5) * cal['width'] / width - .5,
                       (np.arange(height) + .5) * cal['height'] / height - .5)
    xd, yd = (u - cal['cx']) / cal['f_px'], (v - cal['cy']) / cal['f_px']
    x, y = xd.copy(), yd.copy()
    for _ in range(30):
        r2 = x * x + y * y
        factor = 1 + cal['k1'] * r2 + cal['k2'] * r2 * r2
        x, y = xd / factor, yd / factor
    rays = np.stack([x, y, np.ones_like(x)], -1)
    down = -(rays @ np.array(cal['R_floor_to_cam']))[..., 2]
    floor = np.full(down.shape, np.inf)
    valid = down > 1e-4
    floor[valid] = cal['camera_height_mm'] / 1000. / down[valid]
    return rays, floor


def metric_cloud(depth, rays, floor, min_floor_pixels=200, max_range=4.):
    valid = np.isfinite(depth) & (depth > .05)
    candidate = valid & np.isfinite(floor) & (floor >= .15) & (floor < 1.2)
    candidate[int(depth.shape[0] * .92):] = False
    ratios = np.log(floor[candidate] / depth[candidate])
    scale, aligned, count = 1., False, 0
    if len(ratios) >= min_floor_pixels:
        median = np.median(ratios)
        inlier = np.abs(ratios - median) < .15
        count = int(inlier.sum())
        if count >= min_floor_pixels and count >= .35 * len(ratios):
            scale = float(np.exp(np.median(ratios[inlier])))
            aligned = .05 < scale < 20.
            if not aligned:
                scale = 1.
    corrected = depth * scale
    keep = valid & (corrected > .05) & (corrected < max_range)
    points = (rays * corrected[..., None])[keep].astype('<f4')
    return points, {'floor_aligned': bool(aligned), 'scale': scale, 'floor_inliers': count,
                    'floor_candidates': int(candidate.sum()), 'point_count': len(points),
                    'source': 'monocular_network_plus_floor_scale' if aligned else 'uncorrected_monocular_network'}


def read_exact(stream, count):
    data = bytearray()
    while len(data) < count:
        chunk = stream.read(count - len(data))
        if not chunk:
            raise EOFError('Worker input ended')
        data.extend(chunk)
    return bytes(data)


def worker_main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--device', default='mps')
    parser.add_argument('--model', default='depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf')
    parser.add_argument('--revision', default='8078d68a9c75a972131914f6afd0c1723be0da7f')
    args = parser.parse_args()
    cal = json.loads(Path(args.calibration).read_text())
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    output = sys.stdout.buffer
    with contextlib.redirect_stdout(sys.stderr):
        import torch
        from PIL import Image
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        proc = AutoImageProcessor.from_pretrained(args.model, revision=args.revision, local_files_only=True)
        model = AutoModelForDepthEstimation.from_pretrained(args.model, revision=args.revision, local_files_only=True).to(args.device).eval()
    rays, floor = camera_rays(cal, 400, 300)
    while True:
        try:
            size = struct.unpack('<I', read_exact(sys.stdin.buffer, 4))[0]
            if size > 8 * 1024 * 1024:
                raise ValueError('Oversized worker frame')
            jpeg = read_exact(sys.stdin.buffer, size)
        except EOFError:
            break
        t0 = time.monotonic()
        try:
            image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if image is None or image.shape[:2] != (cal['height'], cal['width']):
                raise ValueError('Frame dimensions do not match calibration')
            rgb = cv2.cvtColor(cv2.resize(image, (400, 300)), cv2.COLOR_BGR2RGB)
            with contextlib.redirect_stdout(sys.stderr), torch.no_grad():
                inputs = proc(images=Image.fromarray(rgb), return_tensors='pt').to(args.device)
                prediction = model(**inputs)
                depth = proc.post_process_depth_estimation(prediction, target_sizes=[(300, 400)])[0]['predicted_depth'].float().cpu().numpy()
            points, quality = metric_cloud(depth, rays, floor)
            quality['inference_s'] = time.monotonic() - t0
        except Exception as error:
            points = np.empty((0, 3), dtype='<f4')
            quality = {'error': type(error).__name__, 'point_count': 0}
        packet = io.BytesIO()
        np.savez(packet, xyz=points, quality=np.array(json.dumps(quality)))
        data = packet.getvalue()
        output.write(struct.pack('<I', len(data)) + data); output.flush()


try:
    from z70_driver.driver import MatchedFrames, read_pipe_exact, stop_worker_process, worker_env
except ImportError:
    MatchedFrames = read_pipe_exact = stop_worker_process = worker_env = None


class DepthNode(Node):
    def __init__(self):
        global MatchedFrames, read_pipe_exact, stop_worker_process, worker_env
        if worker_env is None:
            from z70_driver.driver import (
                MatchedFrames as _MF, read_pipe_exact as _rpe,
                stop_worker_process as _swp, worker_env as _we,
            )
            worker_env = _we
            if MatchedFrames is None:
                MatchedFrames = _MF
            if read_pipe_exact is None:
                read_pipe_exact = _rpe
            if stop_worker_process is None:
                stop_worker_process = _swp
        super().__init__('z70_depth_cloud')
        python = self.declare_parameter('depth_python', str(Path.home() / '.venvs/roboclaw-depth/bin/python')).value
        cal = self.declare_parameter('calibration', '').value
        device = self.declare_parameter('device', 'mps').value
        hz = float(self.declare_parameter('depth_hz', .2).value)
        self.pub = self.create_publisher(PointCloud2, '/camera/front/depth/points', 1)
        self.status = self.create_publisher(String, '/z70/depth/status', LATCHED)
        self.latest = None; self.result = None; self.last_stamp = None
        self.lock = threading.Lock(); self.stop_event = threading.Event()
        env = worker_env()
        self.process = None
        try:
            self.process = subprocess.Popen([python, '-u', str(Path(__file__).resolve()),
                                             '--calibration', cal, '--device', device],
                                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env, bufsize=-1)
        except OSError as e:
            self.result = (None, np.empty((0, 3), dtype='<f4'),
                           {'available': False, 'error': type(e).__name__})
        self.frames = MatchedFrames(self, 'front', self.on_frame, compressed=True)
        self.worker = None
        if self.process is not None:
            self.worker = threading.Thread(target=self.run_worker, daemon=True, args=(max(1., 1 / max(.01, hz)),))
            self.worker.start()
        self.create_timer(.2, self.publish)

    def on_frame(self, msg):
        if msg.header.frame_id != 'camera_optical_frame' or len(msg.data) > 8 * 1024 * 1024:
            return
        with self.lock:
            self.latest = msg

    def read_exact(self, count):
        return read_pipe_exact(self.process, count, self.stop_event)

    def run_worker(self, period):
        proc = self.process
        if proc is None or proc.stdin is None:
            return
        while not self.stop_event.is_set():
            with self.lock:
                msg = self.latest
            if msg is not None and (msg.header.stamp.sec, msg.header.stamp.nanosec) != self.last_stamp:
                stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                try:
                    data = bytes(msg.data)
                    proc.stdin.write(struct.pack('<I', len(data)) + data)
                    proc.stdin.flush()
                    size = struct.unpack('<I', self.read_exact(4))[0]
                    if size > 4 * 1024 * 1024:
                        raise ValueError('Oversized worker output')
                    packet = np.load(io.BytesIO(self.read_exact(size)), allow_pickle=False)
                    xyz = np.asarray(packet['xyz'], dtype='<f4')
                    quality = json.loads(str(packet['quality']))
                    with self.lock:
                        self.result = (msg.header, xyz, quality)
                    self.last_stamp = stamp
                except (ValueError, OSError, EOFError, TimeoutError) as e:
                    with self.lock:
                        self.result = (msg.header, np.empty((0, 3), dtype='<f4'),
                                       {'error': type(e).__name__})
                    if isinstance(e, ValueError):
                        self.last_stamp = stamp
                    else:
                        break
            self.stop_event.wait(period)

    def publish(self):
        with self.lock:
            result, self.result = self.result, None
        if result is None:
            return
        header, points, quality = result
        self.status.publish(String(data=json.dumps(quality)))
        if header is None or not len(points):
            return
        msg = PointCloud2(); msg.header = header; msg.height = 1; msg.width = len(points)
        msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z'))]
        msg.point_step = 12; msg.row_step = len(points) * 12
        msg.is_bigendian = False; msg.is_dense = True; msg.data = points.tobytes()
        self.pub.publish(msg)

    def close(self):
        stop_worker_process(self.process, self.worker, self.stop_event)


def depth_main():
    _spin(DepthNode)


if __name__ == '__main__':
    worker_main()
