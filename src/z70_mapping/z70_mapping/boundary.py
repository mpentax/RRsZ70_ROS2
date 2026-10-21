"""Safe interior grid, patrol waypoint dispersion, swept-corridor checks, and session world map."""
from dataclasses import asdict, dataclass, field
import math
import time
import uuid

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


def _normalize_dynamic_mask(m, dynamic_obstacles):
    if dynamic_obstacles is None:
        return np.zeros((m.height, m.width), dtype=bool)
    if hasattr(dynamic_obstacles, 'obstacle_mask'):
        return np.asarray(
            dynamic_obstacles.obstacle_mask(m.width, m.height, m.origin, m.resolution),
            dtype=bool,
        )
    if hasattr(dynamic_obstacles, 'info'):
        g = dynamic_obstacles
        if (g.info.width, g.info.height) != (m.width, m.height):
            return np.zeros((m.height, m.width), dtype=bool)
        return np.asarray(g.data).reshape(m.height, m.width) >= 100
    arr = np.asarray(dynamic_obstacles)
    if arr.shape != (m.height, m.width):
        return np.zeros((m.height, m.width), dtype=bool)
    return arr if arr.dtype == bool else arr >= 100


def compute_safe_interior(m, labels, config, robot_pose_map=None, dynamic_obstacles=None):
    """Union allowlisted rooms before erosion and subtract zones, walls, and obstacles."""
    if not math.isfinite(m.resolution) or m.resolution <= 0:
        raise ValueError('map resolution must be positive and finite')
    ids = extract_segment_ids(m)
    allowed = (ids != 0) & np.isin(ids, config.allowed_segments)
    dyn_mask = _normalize_dynamic_mask(m, dynamic_obstacles)
    forbidden = rasterize_forbidden(m) | dyn_mask
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


MOBILITY_CLASSES = ('pickupable', 'draggable', 'stationary')
RELEVANCE_STATES = ('objective', 'completed', 'deferred', 'ignored')


def mobility_from_suitability(suitability: int) -> str:
    return 'pickupable' if suitability == 1 else 'draggable' if suitability == 2 else 'stationary'


@dataclass
class WorldObject:
    id: str
    category: str
    mobility: str
    relevance: str
    x: float
    y: float
    radius_m: float = 0.08
    width_m: float = 0.08
    depth_m: float = 0.08
    track_id: str = ''
    hits: int = 1
    attempts: int = 0
    detail: str = ''
    updated_at: float = 0.0

    def __post_init__(self):
        if self.mobility not in MOBILITY_CLASSES:
            self.mobility = 'stationary'
        if self.relevance not in RELEVANCE_STATES:
            self.relevance = 'ignored'

    @property
    def is_obstacle(self) -> bool:
        return self.mobility == 'stationary' or self.relevance in ('deferred', 'ignored')

    def to_dict(self) -> dict:
        d = asdict(self)
        d['is_obstacle'] = self.is_obstacle
        return d


class SessionWorldMap:
    """In-memory, session-scoped dynamic costmap and world object registry."""

    def __init__(self, merge_radius_m: float = 0.18, max_attempts: int = 2):
        self.merge_radius_m = float(merge_radius_m)
        self.max_attempts = int(max_attempts)
        self.session_key: tuple | None = None
        self.origin: tuple[float, float] = (0.0, 0.0)
        self.resolution: float = 0.05
        self.width: int = 0
        self.height: int = 0
        self.entries: dict[str, WorldObject] = {}
        self.manual_mask: np.ndarray | None = None

    def bind_session(self, session_id, map_index: int, origin: tuple, width: int, height: int, resolution: float) -> bool:
        key = (bytes(session_id) if isinstance(session_id, (bytes, bytearray, list, tuple)) else str(session_id),
               int(map_index), round(float(origin[0]), 4), round(float(origin[1]), 4),
               int(width), int(height), round(float(resolution), 4))
        if self.session_key != key:
            self.session_key = key
            self.origin = (float(origin[0]), float(origin[1]))
            self.width, self.height, self.resolution = int(width), int(height), float(resolution)
            self.entries.clear()
            self.manual_mask = np.zeros((self.height, self.width), dtype=bool)
            return True
        return False

    def clear(self) -> None:
        self.entries.clear()
        if self.manual_mask is not None:
            self.manual_mask.fill(False)

    def _find_near(self, x: float, y: float, radius_m: float | None = None) -> WorldObject | None:
        best: WorldObject | None = None
        best_d = self.merge_radius_m if radius_m is None else float(radius_m)
        for entry in self.entries.values():
            d = math.hypot(entry.x - x, entry.y - y)
            if d <= max(best_d, entry.radius_m):
                best, best_d = entry, d
        return best

    def _resolve(self, x: float, y: float, track_id: str = '', radius_m: float | None = None) -> WorldObject | None:
        return self.entries.get(track_id) if track_id and track_id in self.entries else self._find_near(x, y, radius_m)

    def upsert_object(self, x: float, y: float, *, category: str = 'object', mobility: str = 'pickupable',
                      relevance: str = 'objective', width_m: float = 0.08, depth_m: float = 0.08,
                      radius_m: float | None = None, track_id: str = '', detail: str = '',
                      confirmed: bool = True) -> WorldObject | None:
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        existing = self._resolve(x, y, track_id, radius_m)
        rad = float(radius_m if radius_m is not None else max(0.06, min(0.35, 0.5 * max(width_m, depth_m, 0.12))))
        now = time.time()
        if existing is not None:
            if relevance == 'objective' and existing.relevance in ('completed', 'deferred'):
                return existing
            alpha = 0.35 if relevance == 'objective' else 1.0
            existing.x = float((1.0 - alpha) * existing.x + alpha * x)
            existing.y = float((1.0 - alpha) * existing.y + alpha * y)
            existing.width_m = float(max(existing.width_m, width_m))
            existing.depth_m = float(max(existing.depth_m, depth_m))
            existing.radius_m = float(max(existing.radius_m, rad))
            if mobility in MOBILITY_CLASSES:
                existing.mobility = mobility
            if relevance in RELEVANCE_STATES:
                existing.relevance = relevance
            if category and category not in ('object', 'garment'):
                existing.category = str(category)
            if track_id:
                existing.track_id = str(track_id)
            if detail:
                existing.detail = str(detail)
            existing.hits += 1
            existing.updated_at = now
            return existing
        if not confirmed:
            return None
        ident = str(track_id) if track_id else str(uuid.uuid4().hex[:12])
        entry = WorldObject(
            id=ident, category=str(category or 'object'),
            mobility=mobility if mobility in MOBILITY_CLASSES else 'stationary',
            relevance=relevance if relevance in RELEVANCE_STATES else 'ignored',
            x=float(x), y=float(y), radius_m=rad, width_m=float(width_m), depth_m=float(depth_m),
            track_id=str(track_id), hits=1, attempts=0, detail=str(detail), updated_at=now,
        )
        self.entries[ident] = entry
        return entry

    def observe_garment(self, x: float, y: float, *, category: str = 'garment', width_m: float = 0.08,
                        depth_m: float = 0.08, transport_suitability: int = 1, track_id: str = '',
                        confirmed: bool = True, drop_xy: tuple[float, float] | None = None) -> WorldObject | None:
        if drop_xy is not None and math.hypot(x - drop_xy[0], y - drop_xy[1]) <= 0.45:
            if any(e.relevance == 'completed' for e in self.entries.values()):
                return None
        mob = mobility_from_suitability(int(transport_suitability))
        rel = 'objective' if mob in ('pickupable', 'draggable') else 'deferred'
        return self.upsert_object(x, y, category=category, mobility=mob, relevance=rel,
                                  width_m=width_m, depth_m=depth_m, track_id=track_id, confirmed=confirmed)

    def ingest_observations(self, batch, pose: tuple[float, float, float] | None, *,
                            drop_xy: tuple[float, float] | None = None) -> list[WorldObject]:
        if batch is None or pose is None or len(pose) < 3 or not all(math.isfinite(v) for v in pose[:3]):
            return []
        c, s = math.cos(pose[2]), math.sin(pose[2])
        updated = []
        for g in getattr(batch, 'garments', ()):
            if not (getattr(g, 'contact_valid', False) and getattr(g, 'inside_geofence', False) and getattr(g, 'geofence_verified', False)):
                continue
            p = g.floor_contact_m
            mx, my = float(pose[0] + c * p.x - s * p.y), float(pose[1] + s * p.x + c * p.y)
            tid = bytes(g.track_id.uuid).hex() if hasattr(g, 'track_id') and hasattr(g.track_id, 'uuid') else ''
            entry = self.observe_garment(
                mx, my, category=getattr(g, 'category', 'garment'),
                width_m=float(getattr(g, 'footprint_width_m', 0.08)),
                depth_m=float(getattr(g, 'footprint_depth_m', 0.08)),
                transport_suitability=int(getattr(g, 'transport_suitability', 1)),
                track_id=tid, confirmed=True, drop_xy=drop_xy,
            )
            if entry is not None:
                updated.append(entry)
        return updated

    def mark_obstacle(self, x: float, y: float, *, radius_m: float = 0.12, category: str = 'obstacle',
                      mobility: str = 'stationary', relevance: str = 'deferred', detail: str = 'OBSTACLE',
                      track_id: str = '') -> WorldObject:
        entry = self.upsert_object(
            x, y, category=category, mobility=mobility if mobility in MOBILITY_CLASSES else 'stationary',
            relevance=relevance if relevance in ('deferred', 'ignored') else 'deferred',
            radius_m=max(0.05, float(radius_m)), width_m=2.0 * float(radius_m), depth_m=2.0 * float(radius_m),
            track_id=track_id, detail=detail, confirmed=True,
        )
        assert entry is not None
        return entry

    def mark_completed(self, x: float, y: float, *, track_id: str = '', category: str = 'object',
                       mobility: str = 'pickupable', radius_m: float = 0.12, detail: str = 'COMPLETED') -> WorldObject:
        existing = self._resolve(x, y, track_id)
        mob = existing.mobility if existing is not None else (mobility if mobility in MOBILITY_CLASSES else 'pickupable')
        entry = self.upsert_object(x, y, category=category, mobility=mob, relevance='completed',
                                   radius_m=max(0.08, float(radius_m)), track_id=track_id, detail=detail, confirmed=True)
        assert entry is not None
        return entry

    def mark_attempt_failed(self, x: float, y: float, *, track_id: str = '', detail: str = 'FAILED') -> WorldObject | None:
        entry = self._resolve(x, y, track_id)
        if entry is None:
            return None
        entry.attempts += 1
        entry.detail = str(detail)
        entry.updated_at = time.time()
        entry.relevance = 'deferred' if entry.attempts >= self.max_attempts else 'objective'
        return entry

    def apply_event(self, payload: dict) -> bool:
        if not isinstance(payload, dict):
            return False
        action = payload.get('action')
        if action == 'clear':
            self.clear(); return True
        if action in ('upsert', 'objective'):
            mob = str(payload.get('mobility') or (
                mobility_from_suitability(int(payload['transport_suitability']))
                if 'transport_suitability' in payload else 'pickupable'
            ))
            self.upsert_object(
                float(payload['x']), float(payload['y']), category=str(payload.get('category', 'object')),
                mobility=mob, relevance=str(payload.get('relevance', 'objective')),
                width_m=float(payload.get('width_m', 0.08)), depth_m=float(payload.get('depth_m', 0.08)),
                radius_m=float(payload['radius_m']) if 'radius_m' in payload and payload['radius_m'] is not None else None,
                track_id=str(payload.get('track_id', '')), detail=str(payload.get('detail', '')), confirmed=True,
            )
            return True
        if action == 'obstacle':
            self.mark_obstacle(
                float(payload['x']), float(payload['y']), radius_m=float(payload.get('radius_m', 0.12)),
                category=str(payload.get('category', 'obstacle')), mobility=str(payload.get('mobility', 'stationary')),
                relevance=str(payload.get('relevance', 'deferred')), detail=str(payload.get('detail', 'OBSTACLE')),
                track_id=str(payload.get('track_id', '')),
            )
            return True
        if action in ('completed', 'delivered'):
            self.mark_completed(
                float(payload['x']), float(payload['y']), track_id=str(payload.get('track_id', '')),
                category=str(payload.get('category', 'object')), mobility=str(payload.get('mobility', 'pickupable')),
                radius_m=float(payload.get('radius_m', 0.12)), detail=str(payload.get('detail', 'COMPLETED')),
            )
            return True
        if action == 'failed':
            self.mark_attempt_failed(float(payload['x']), float(payload['y']),
                                     track_id=str(payload.get('track_id', '')), detail=str(payload.get('detail', 'FAILED')))
            return True
        return False

    def load_entries(self, data: dict) -> None:
        if not isinstance(data, dict):
            return
        defaults = dict(category='object', mobility='pickupable', relevance='objective', x=0.0, y=0.0)
        for raw in data.get('entries', ()):
            if not isinstance(raw, dict) or 'id' not in raw:
                continue
            fields = {**defaults, **{k: raw[k] for k in WorldObject.__dataclass_fields__ if k in raw}}
            self.entries[str(raw['id'])] = WorldObject(**fields)

    def pending_objectives(self, pose_xy: tuple[float, float] | None = None, *,
                           allowed_mobility: tuple[str, ...] = ('pickupable', 'draggable')) -> list[WorldObject]:
        items = [e for e in self.entries.values()
                 if e.relevance == 'objective' and e.mobility in allowed_mobility and e.attempts < self.max_attempts]
        items.sort(key=(lambda e: (math.hypot(e.x - pose_xy[0], e.y - pose_xy[1]), e.id)) if pose_xy is not None else (lambda e: e.id))
        return items

    def obstacle_entries(self, include_completed: bool = False) -> list[WorldObject]:
        return [e for e in self.entries.values() if e.is_obstacle or (include_completed and e.relevance == 'completed')]

    def obstacle_mask(self, width: int | None = None, height: int | None = None,
                      origin: tuple[float, float] | None = None, resolution: float | None = None, *,
                      include_completed: bool = False, protect_xy: tuple[float, float] | None = None,
                      protect_radius_m: float = 0.28) -> np.ndarray:
        w = int(width if width is not None else self.width)
        h = int(height if height is not None else self.height)
        orig = origin if origin is not None else self.origin
        res = float(resolution if resolution is not None else self.resolution)
        if w <= 0 or h <= 0 or res <= 0:
            return np.zeros((max(0, h), max(0, w)), dtype=bool)
        mask = np.zeros((h, w), dtype=np.uint8)
        if self.manual_mask is not None and self.manual_mask.shape == (h, w):
            mask |= self.manual_mask.astype(np.uint8)
        for entry in self.obstacle_entries(include_completed=include_completed):
            gx = int(round((entry.x - orig[0]) / res - 0.5))
            gy = int(round((entry.y - orig[1]) / res - 0.5))
            if not (0 <= gx < w and 0 <= gy < h):
                continue
            r_cells = max(0, int(math.ceil(max(0.0, entry.radius_m - 0.5 * res) / res)))
            cv2.circle(mask, (gx, gy), r_cells, 1, -1)
        if protect_xy is not None and all(math.isfinite(v) for v in protect_xy[:2]):
            px = int(round((protect_xy[0] - orig[0]) / res - 0.5))
            py = int(round((protect_xy[1] - orig[1]) / res - 0.5))
            pr = max(1, int(math.ceil(protect_radius_m / res)))
            if 0 <= px < w and 0 <= py < h:
                cv2.circle(mask, (px, py), pr, 0, -1)
        return mask.astype(bool)

    def occupancy_grid_data(self, width: int | None = None, height: int | None = None,
                            origin: tuple[float, float] | None = None, resolution: float | None = None) -> np.ndarray:
        obs = self.obstacle_mask(width, height, origin, resolution, include_completed=False)
        completed = self.obstacle_mask(width, height, origin, resolution, include_completed=True) & ~obs
        grid = np.zeros(obs.shape, dtype=np.int8)
        grid[completed] = 50
        grid[obs] = 100
        return grid

    def summary(self, pose_xy: tuple[float, float] | None = None) -> dict:
        return dict(
            session_bound=self.session_key is not None,
            objectives=[e.to_dict() for e in self.pending_objectives(pose_xy)],
            obstacles=[e.to_dict() for e in self.obstacle_entries(include_completed=False)],
            completed=[e.to_dict() for e in self.entries.values() if e.relevance == 'completed'],
            by_mobility={mob: [e.to_dict() for e in self.entries.values() if e.mobility == mob] for mob in MOBILITY_CLASSES},
            entries=[e.to_dict() for e in self.entries.values()],
        )
