"""SE(2) single-pose alignment, not a SLAM/fusion/localisation filter."""
import math


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
    """Snapshot-grid context; ray distances are not live collision clearance."""
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
