"""ROS OccupancyGrid, PoseStamped, and MarkerArray serializers for vendor maps."""
import math
import cv2
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray


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
    for segment, room in m.rooms.items():
        color = ((segment * .31) % .7 + .3, (segment * .51) % .7 + .3, .8, 1.)
        text = marker('rooms', segment, Marker.TEXT_VIEW_FACING, color)
        text.pose.position.x, text.pose.position.y = room['centroid']
        text.pose.position.z = .15; text.scale.z = .18
        text.text = labels.get(segment, f'Segment {segment}')
        lines = marker('room_polygons', segment, Marker.LINE_LIST, color)
        lines.scale.x = .025
        for poly in room['polygons']:
            for a, b in zip(poly, poly[1:] + poly[:1]):
                lines.points.extend([Point(x=a[0], y=a[1], z=.025), Point(x=b[0], y=b[1], z=.025)])
    for ns, p, color in [('dock', m.dock, (0., 1., .5, 1.)), ('vendor_robot', m.robot, (1., .35, .1, 1.))]:
        if p is None:
            continue
        x = marker(ns, 0, Marker.ARROW if p[2] is not None else Marker.SPHERE, color)
        x.pose = pose_message(p, saved_at).pose
        x.scale.x, x.scale.y, x.scale.z = .35, .12, .12
        if ns == 'dock':
            text = marker(ns + '_label', 0, Marker.TEXT_VIEW_FACING, color)
            text.pose.position.x, text.pose.position.y = p[:2]
            text.pose.position.z = .3; text.scale.z = .14
            text.text = 'Dock (vendor map)'
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
            for a, b in zip(poly, poly[1:] + poly[:1]):
                lines.points.extend([Point(x=a[0], y=a[1], z=.04), Point(x=b[0], y=b[1], z=.04)])
    if m.carpet is not None and np.any(m.carpet):
        contours, _ = cv2.findContours(m.carpet.astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        lines = marker('carpet', 0, Marker.LINE_LIST, (.65, .55, .98, .85))
        lines.scale.x = .02
        for contour in contours:
            poly = [(m.origin[0] + (int(x) + .5) * m.resolution,
                     m.origin[1] + (int(y) + .5) * m.resolution)
                    for x, y in contour.reshape(-1, 2)]
            for a, b in zip(poly, poly[1:] + poly[:1]):
                lines.points.extend([Point(x=a[0], y=a[1], z=.015), Point(x=b[0], y=b[1], z=.015)])
    if boundary_result is not None:
        waypoints = boundary_result.waypoints if waypoints is None else waypoints
        text = marker('boundary_contract', 0, Marker.TEXT_VIEW_FACING, (.2, 1., .6, 1.))
        text.pose.position.x, text.pose.position.y = m.origin
        text.pose.position.z = .4; text.scale.z = .16
        text.text = (f'Interior: {int(boundary_result.reachable_mask.sum())} reachable cells; '
                     f'clearance {boundary_result.config.clearance_m:.3f} m; '
                     f'lost rooms {boundary_result.lost_rooms}')
    for i, waypoint in enumerate(waypoints or []):
        point = marker('patrol_waypoints', i, Marker.SPHERE, (.1, 1., .8, 1.))
        point.pose.position.x, point.pose.position.y = waypoint['x'], waypoint['y']
        point.pose.position.z = .08
        point.scale.x = point.scale.y = point.scale.z = .12
        label = marker('patrol_waypoint_labels', i, Marker.TEXT_VIEW_FACING, (.1, 1., .8, 1.))
        label.pose.position.x, label.pose.position.y = waypoint['x'], waypoint['y']
        label.pose.position.z = .22; label.scale.z = .12
        label.text = f"{i + 1}: S{waypoint['segment_id']}"
    return result
