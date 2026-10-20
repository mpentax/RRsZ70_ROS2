"""Roborock V1 LiDAR map binary decoder."""
from dataclasses import dataclass, field
import hashlib
import math
import struct
import numpy as np
import cv2


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
        # All disconnected contours retained; boundary follows the actual segmented raster,
        # not a convex hull/bounding box that would invent walkable space.
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


def spatial_context(m, labels, robot_pose_map=None) -> dict:
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
    for seg, room in sorted(m.rooms.items()):
        x, y = room['centroid']
        distance, bearing = relative(x, y)
        rooms.append({'segment_id': seg, 'name': labels.get(seg, f'Segment {seg}'),
                      'centroid': [round(x, 3), round(y, 3)], 'area_m2': round(room['area_m2'], 3),
                      'distance_m': distance, 'bearing_rad': bearing})
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
    return {'robot_pose_map': robot_pose, 'current_room': current_room,
            'rooms': rooms, 'dock': dock, 'clearance_sectors_m': sectors}
