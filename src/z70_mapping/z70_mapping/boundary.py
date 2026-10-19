"""Safe interior grid, patrol waypoint dispersion, and swept-corridor checks."""
from dataclasses import asdict, dataclass, field
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class BoundaryConfig:
    """Physical clearance and waypoint dispersion parameters (metres)."""
    allowed_segments: tuple[int, ...] = (1, 2, 3, 4, 5, 6)
    chassis_radius_m: float = .199
    deployed_arm_extension_m: float = 0.
    localization_uncertainty_m: float = 0.
    stopping_distance_m: float = 0.
    waypoint_clearance_m: float = .25
    waypoint_spacing_m: float = .90

    def __post_init__(self):
        if any(not isinstance(s, (int, np.integer)) or not 1 <= s <= 30
               for s in self.allowed_segments):
            raise ValueError('allowed_segments must contain explicit segment IDs 1..30')
        for name in ('chassis_radius_m', 'deployed_arm_extension_m',
                     'localization_uncertainty_m', 'stopping_distance_m',
                     'waypoint_clearance_m', 'waypoint_spacing_m'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if self.waypoint_spacing_m <= 0:
            raise ValueError('waypoint_spacing_m must be positive')

    @property
    def clearance_m(self):
        return (self.chassis_radius_m + self.deployed_arm_extension_m
                + self.localization_uncertainty_m + self.stopping_distance_m)


@dataclass
class BoundaryResult:
    width: int
    height: int
    origin: tuple
    resolution: float
    config: BoundaryConfig
    segment_ids: np.ndarray
    allowed_mask: np.ndarray
    forbidden_mask: np.ndarray
    interior_mask: np.ndarray
    clearance_m: np.ndarray
    safe_mask: np.ndarray
    component_labels: np.ndarray
    reachable_mask: np.ndarray
    active_component: int
    selection_method: str
    room_connectivity: dict
    lost_rooms: list
    robot_pose_map: tuple | None
    waypoints: list = field(default_factory=list)

    @property
    def safe_grid(self):
        """ROS-compatible values: only active reachable safe cells are free."""
        return np.where(self.reachable_mask, 0, 100).astype(np.int8)

    def summary(self):
        return dict(version=1, frame_id='map', config=asdict(self.config),
                    required_clearance_m=self.config.clearance_m,
                    allowed_cells=int(self.allowed_mask.sum()),
                    interior_cells=int(self.interior_mask.sum()),
                    safe_cells=int(self.safe_mask.sum()),
                    reachable_cells=int(self.reachable_mask.sum()),
                    component_count=int(self.component_labels.max()),
                    active_component=self.active_component,
                    component_selection=self.selection_method,
                    room_connectivity=self.room_connectivity, lost_rooms=self.lost_rooms,
                    patrol_waypoints=self.waypoints)


def extract_segment_ids(m):
    """Extract room segment IDs (1..30) from free segmented pixels."""
    valid = ((m.occupancy == 0) & ((m.pixels & 7) == 7)
             & (m.pixels != 7) & (m.pixels != 255))
    return np.where(valid, m.pixels >> 3, 0).astype(np.uint8)


def _segment_intersects_box(a, b, lo, hi):
    """Closed segment / closed AABB intersection (including corner touches)."""
    enter, leave = 0., 1.
    for axis in (0, 1):
        d = b[axis] - a[axis]
        if abs(d) < 1e-14:
            if a[axis] < lo[axis] or a[axis] > hi[axis]:
                return False
        else:
            t0, t1 = (lo[axis] - a[axis]) / d, (hi[axis] - a[axis]) / d
            enter, leave = max(enter, min(t0, t1)), min(leave, max(t0, t1))
            if enter > leave + 1e-12:
                return False
    return True


def _raster_line(mask, a, b):
    # Supercover: every touched square is excluded, including both sides of a
    # grid-line wall. OpenCV's thin Bresenham line alone misses diagonal corners.
    h, w = mask.shape
    x0, y0 = np.floor(np.minimum(a, b) - 1e-9).astype(int)
    x1, y1 = np.floor(np.maximum(a, b) + 1e-9).astype(int)
    for y in range(max(0, y0), min(h - 1, y1) + 1):
        for x in range(max(0, x0), min(w - 1, x1) + 1):
            if _segment_intersects_box(a, b, (x, y), (x + 1, y + 1)):
                mask[y, x] = 1


def rasterize_forbidden(m):
    """Metric polygons/segments -> conservative touched-cell masks in raw grid xy."""
    mask = np.zeros((m.height, m.width), np.uint8)
    def grid(p):
        return (np.asarray(p, dtype=float) - m.origin) / m.resolution
    for polygon in m.zones:
        points = grid(polygon)
        if len(points) >= 3:
            cv2.fillPoly(mask, [np.floor(points).astype(np.int32)], 1)
            for a, b in zip(points, np.roll(points, -1, axis=0)):
                _raster_line(mask, a, b)
    for x0, y0, x1, y1 in m.virtual_walls:
        _raster_line(mask, grid((x0, y0)), grid((x1, y1)))
    return mask.astype(bool)


def compute_safe_interior(m, labels, config, robot_pose_map=None):
    """Union allowlisted rooms before erosion and subtract zones, walls, and obstacles."""
    if not math.isfinite(m.resolution) or m.resolution <= 0:
        raise ValueError('map resolution must be positive and finite')
    ids = extract_segment_ids(m)
    allowed = (ids != 0) & np.isin(ids, config.allowed_segments)
    forbidden = rasterize_forbidden(m)
    interior = allowed & ~forbidden & (m.occupancy == 0)
    padded = np.pad(interior.astype(np.uint8), 1)
    distance = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    clearance = np.maximum(0., distance * m.resolution - m.resolution / math.sqrt(2))
    safe = interior & (clearance + 1e-9 >= config.clearance_m)
    count, components, stats, _ = cv2.connectedComponentsWithStats(safe.astype(np.uint8), connectivity=4)
    pose = robot_pose_map if robot_pose_map is not None else m.robot
    active, selection = 0, 'empty'
    if count > 1:
        if pose is not None and np.isfinite(pose[:2]).all():
            x, y = np.floor((np.asarray(pose[:2]) - m.origin) / m.resolution).astype(int)
            if 0 <= x < m.width and 0 <= y < m.height:
                active = int(components[y, x])
                if active:
                    selection = 'robot_safe_cell'
            if not active:
                ys, xs = np.nonzero(safe)
                d2 = ((m.origin[0] + (xs + .5) * m.resolution - pose[0]) ** 2
                      + (m.origin[1] + (ys + .5) * m.resolution - pose[1]) ** 2)
                nearest = int(np.argmin(d2))
                if d2[nearest] <= config.chassis_radius_m ** 2 + 1e-12:
                    active = int(components[ys[nearest], xs[nearest]])
                    selection = 'nearest_safe_cell'
        if not active:
            active = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            selection = 'largest_component_fallback'
    reachable = (components == active) & safe if active else np.zeros_like(safe)
    rooms, lost = {}, []
    for segment in sorted(set(m.rooms) | set(config.allowed_segments)):
        cells = ids == segment
        raw, n_safe, n_reachable = (int(np.count_nonzero(cells & mask))
                                   for mask in (np.ones_like(safe), safe, reachable))
        is_allowed = segment in config.allowed_segments
        reasons = []
        if not is_allowed:
            reasons.append('not_allowlisted')
        if is_allowed and not raw:
            reasons.append('no_segmented_floor')
        blocked = int(np.count_nonzero(cells & forbidden))
        if blocked:
            reasons.append('forbidden_zones_or_virtual_walls')
        if is_allowed and raw and not n_safe:
            reasons.append('lost_after_exclusion_or_clearance')
        if is_allowed and n_safe > n_reachable:
            reasons.append('disconnected_after_exclusion_or_clearance')
        disconnected = is_allowed and (not n_reachable or n_reachable < n_safe)
        if is_allowed and not n_reachable:
            lost.append(int(segment))
        rooms[int(segment)] = dict(name=labels.get(segment, f'Segment {segment}'),
                                  allowed=is_allowed, raw_cells=raw, safe_cells=n_safe,
                                  reachable_cells=n_reachable,
                                  reachable_fraction=n_reachable / raw if raw else 0.,
                                  forbidden_cells=blocked, disconnected=disconnected,
                                  lost=is_allowed and n_reachable == 0, reasons=reasons)
    result = BoundaryResult(m.width, m.height, m.origin, m.resolution, config, ids, allowed,
                            forbidden, interior, clearance, safe, components, reachable,
                            active, selection, rooms, lost, pose)
    result.waypoints = generate_patrol_waypoints(result)
    return result


def generate_patrol_waypoints(result, robot_pose_map=None):
    """Clearance-first greedy dispersion with one seed per reachable room, ordered as a nearest-neighbor tour."""
    ys, xs = np.nonzero(result.reachable_mask)
    if not len(xs):
        return []
    clearance = result.clearance_m[ys, xs]
    segments = result.segment_ids[ys, xs]
    # np.lexsort's last key is primary. Equal clearance resolves by row/column.
    ranked = np.lexsort((xs, ys, -clearance))
    chosen = []
    for segment in sorted(np.unique(segments)):
        chosen.append(int(next(i for i in ranked if segments[i] == segment)))
    preferred = clearance >= result.config.waypoint_clearance_m
    spacing2 = (result.config.waypoint_spacing_m / result.resolution) ** 2
    for i in ranked:
        if not preferred[i] or int(i) in chosen:
            continue
        if all((int(xs[i]) - int(xs[j])) ** 2 + (int(ys[i]) - int(ys[j])) ** 2
               >= spacing2 - 1e-9 for j in chosen):
            chosen.append(int(i))
    points = []
    for i in chosen:
        points.append(dict(segment_id=int(segments[i]), grid_xy=[int(xs[i]), int(ys[i])],
                           x=float(result.origin[0] + (xs[i] + .5) * result.resolution),
                           y=float(result.origin[1] + (ys[i] + .5) * result.resolution),
                           clearance_m=float(clearance[i]),
                           clearance_preference_met=bool(preferred[i])))
    pose = robot_pose_map if robot_pose_map is not None else result.robot_pose_map
    ordered = []
    if pose is None or not np.isfinite(pose[:2]).all():
        # Without a pose start with the safest cell in the first room.
        ordered.append(points.pop(0))
        current = (ordered[0]['x'], ordered[0]['y'])
    else:
        current = pose[:2]
    while points:
        i = min(range(len(points)), key=lambda i: (
            (points[i]['x'] - current[0]) ** 2 + (points[i]['y'] - current[1]) ** 2,
            points[i]['segment_id'], points[i]['grid_xy'][1], points[i]['grid_xy'][0]))
        point = points.pop(i)
        ordered.append(point)
        current = (point['x'], point['y'])
    return ordered


def _point_segment_distance2(p, a, b):
    direction = b - a
    length2 = float(direction @ direction)
    t = float(np.clip((p - a) @ direction / length2, 0., 1.)) if length2 else 0.
    delta = p - (a + t * direction)
    return float(delta @ delta)


def check_corridor_containment(safe_grid_or_result, start_xy, end_xy, radius_m):
    """Check whether a straight swept disk stays inside the reachable safe grid."""
    if isinstance(safe_grid_or_result, BoundaryResult):
        mask = safe_grid_or_result.reachable_mask
        origin, resolution = safe_grid_or_result.origin, safe_grid_or_result.resolution
    elif hasattr(safe_grid_or_result, 'info'):
        g = safe_grid_or_result
        q = g.info.origin.orientation
        if abs(q.x) + abs(q.y) + abs(q.z) > 1e-12 or abs(q.w - 1.) > 1e-12:
            return False
        mask = np.asarray(g.data).reshape(g.info.height, g.info.width) == 0
        origin = (g.info.origin.position.x, g.info.origin.position.y)
        resolution = g.info.resolution
    else:
        array = np.asarray(safe_grid_or_result)
        mask = array if array.dtype == bool else array == 0
        origin, resolution = (0., 0.), 1.
    if (mask.ndim != 2 or not mask.size or resolution <= 0
            or not math.isfinite(resolution) or not math.isfinite(radius_m) or radius_m < 0):
        return False
    a = (np.asarray(start_xy, dtype=float) - origin) / resolution
    b = (np.asarray(end_xy, dtype=float) - origin) / resolution
    if a.shape != (2,) or b.shape != (2,) or not np.isfinite([a, b]).all():
        return False
    radius = radius_m / resolution
    h, w = mask.shape
    low, high = np.minimum(a, b) - radius, np.maximum(a, b) + radius
    if (low <= 0).any() or high[0] >= w or high[1] >= h:
        return False
    samples = np.linspace(a, b, max(2, int(math.ceil(np.linalg.norm(b - a) * 4)) + 1))
    cells = np.floor(samples).astype(int)
    if not mask[cells[:, 1], cells[:, 0]].all():
        return False
    x0, y0 = np.maximum(0, np.floor(low - 1e-9).astype(int))
    x1, y1 = np.minimum((w - 1, h - 1), np.floor(high + 1e-9).astype(int))
    ys, xs = np.nonzero(~mask[y0:y1 + 1, x0:x1 + 1])
    for y, x in zip(ys + y0, xs + x0):
        lo, hi = np.array((x, y)), np.array((x + 1, y + 1))
        if _segment_intersects_box(a, b, lo, hi):
            return False
        # Disjoint segment/rectangle closest points are an endpoint/edge pair
        # or a rectangle corner projected onto the segment.
        d2 = min(float(np.sum((p - np.clip(p, lo, hi)) ** 2)) for p in (a, b))
        for corner in (lo, hi, np.array((x, y + 1)), np.array((x + 1, y))):
            d2 = min(d2, _point_segment_distance2(corner, a, b))
        if d2 <= radius ** 2 + 1e-12:
            return False
    return True
