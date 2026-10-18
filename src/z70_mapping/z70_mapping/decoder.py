"""Bounded Roborock rr/V1 decoder. Vendor bottom-up raster maps directly to ROS xy.

Pixels 1/(low bits 1) are walls, 7/255/(low bits 7) floor; other values unknown.
Coordinates in position blocks are millimetres, image origins are 50-mm cells.
No claims of capture-time freshness, live localisation, or obstacle-free floor.
"""
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
