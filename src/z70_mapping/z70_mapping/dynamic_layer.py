"""Session-scoped dynamic obstacle layer and map-frame world object registry.

Each tracked world object has two orthogonal dimensions:
1. `mobility`: physical interaction class ('pickupable' | 'draggable' | 'stationary').
2. `relevance`: task/objective relevance ('objective' | 'completed' | 'deferred' | 'ignored').
"""
from dataclasses import asdict, dataclass
import math
import time
import uuid

import cv2
import numpy as np

MOBILITY_CLASSES = ('pickupable', 'draggable', 'stationary')
RELEVANCE_STATES = ('objective', 'completed', 'deferred', 'ignored')


def mobility_from_suitability(suitability: int) -> str:
    if suitability == 1:
        return 'pickupable'
    if suitability == 2:
        return 'draggable'
    return 'stationary'


def suitability_from_mobility(mobility: str) -> int:
    if mobility == 'pickupable':
        return 1
    if mobility == 'draggable':
        return 2
    return 3


@dataclass
class WorldObject:
    id: str
    category: str
    mobility: str  # 'pickupable' | 'draggable' | 'stationary'
    relevance: str  # 'objective' | 'completed' | 'deferred' | 'ignored'
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
        """Stationary objects and deferred/ignored unmovable items block routes."""
        return self.mobility == 'stationary' or self.relevance in ('deferred', 'ignored')

    @property
    def is_active_objective(self) -> bool:
        return self.relevance == 'objective' and self.mobility in ('pickupable', 'draggable')

    @property
    def transport_suitability(self) -> int:
        return suitability_from_mobility(self.mobility)

    def to_dict(self) -> dict:
        d = asdict(self)
        d['is_obstacle'] = self.is_obstacle
        return d


# Backwards-compatible alias for external type imports
DynamicEntry = WorldObject


class SessionWorldMap:
    """In-memory, session-scoped dynamic costmap and world object registry.

    Resets automatically when the map session or floor geometry changes.
    """

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
        key = (
            bytes(session_id) if isinstance(session_id, (bytes, bytearray, list, tuple)) else str(session_id),
            int(map_index),
            round(float(origin[0]), 4),
            round(float(origin[1]), 4),
            int(width),
            int(height),
            round(float(resolution), 4),
        )
        if self.session_key != key:
            self.session_key = key
            self.origin = (float(origin[0]), float(origin[1]))
            self.width = int(width)
            self.height = int(height)
            self.resolution = float(resolution)
            self.entries.clear()
            self.manual_mask = np.zeros((self.height, self.width), dtype=bool)
            return True
        return False

    def clear(self) -> None:
        self.entries.clear()
        if self.manual_mask is not None:
            self.manual_mask.fill(False)

    def _find_near(self, x: float, y: float, radius_m: float | None = None) -> WorldObject | None:
        gate = self.merge_radius_m if radius_m is None else float(radius_m)
        best: WorldObject | None = None
        best_d = gate
        for entry in self.entries.values():
            d = math.hypot(entry.x - x, entry.y - y)
            if d <= max(best_d, entry.radius_m):
                best = entry
                best_d = d
        return best

    def upsert_object(
        self,
        x: float,
        y: float,
        *,
        category: str = 'object',
        mobility: str = 'pickupable',
        relevance: str = 'objective',
        width_m: float = 0.08,
        depth_m: float = 0.08,
        radius_m: float | None = None,
        track_id: str = '',
        detail: str = '',
        confirmed: bool = True,
    ) -> WorldObject | None:
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        existing = (
            self.entries.get(track_id)
            if track_id and track_id in self.entries
            else self._find_near(x, y, radius_m)
        )
        rad = float(
            radius_m
            if radius_m is not None
            else max(0.06, min(0.35, 0.5 * max(width_m, depth_m, 0.12)))
        )
        now = time.time()
        if existing is not None:
            # Preserve terminal task relevance ('completed' / 'deferred') during passive re-observation
            # unless an explicit non-objective update is applied.
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
            id=ident,
            category=str(category or 'object'),
            mobility=mobility if mobility in MOBILITY_CLASSES else 'stationary',
            relevance=relevance if relevance in RELEVANCE_STATES else 'ignored',
            x=float(x),
            y=float(y),
            radius_m=rad,
            width_m=float(width_m),
            depth_m=float(depth_m),
            track_id=str(track_id),
            hits=1,
            attempts=0,
            detail=str(detail),
            updated_at=now,
        )
        self.entries[ident] = entry
        return entry

    def observe_garment(
        self,
        x: float,
        y: float,
        *,
        category: str = 'garment',
        width_m: float = 0.08,
        depth_m: float = 0.08,
        transport_suitability: int = 1,
        track_id: str = '',
        confirmed: bool = True,
        drop_xy: tuple[float, float] | None = None,
    ) -> WorldObject | None:
        if drop_xy is not None and math.hypot(x - drop_xy[0], y - drop_xy[1]) <= 0.45:
            if any(e.relevance == 'completed' for e in self.entries.values()):
                return None
        mob = mobility_from_suitability(int(transport_suitability))
        rel = 'objective' if mob in ('pickupable', 'draggable') else 'deferred'
        return self.upsert_object(
            x,
            y,
            category=category,
            mobility=mob,
            relevance=rel,
            width_m=width_m,
            depth_m=depth_m,
            track_id=track_id,
            confirmed=confirmed,
        )

    def ingest_observations(self, batch, pose: tuple[float, float, float] | None, *, drop_xy: tuple[float, float] | None = None) -> list[WorldObject]:
        if batch is None or pose is None or len(pose) < 3 or not all(math.isfinite(v) for v in pose[:3]):
            return []
        c, s = math.cos(pose[2]), math.sin(pose[2])
        updated = []
        for g in getattr(batch, 'garments', ()):
            if not (getattr(g, 'contact_valid', False) and getattr(g, 'inside_geofence', False) and getattr(g, 'geofence_verified', False)):
                continue
            p = g.floor_contact_m
            mx = float(pose[0] + c * p.x - s * p.y)
            my = float(pose[1] + s * p.x + c * p.y)
            tid = bytes(g.track_id.uuid).hex() if hasattr(g, 'track_id') and hasattr(g.track_id, 'uuid') else ''
            entry = self.observe_garment(
                mx,
                my,
                category=getattr(g, 'category', 'garment'),
                width_m=float(getattr(g, 'footprint_width_m', 0.08)),
                depth_m=float(getattr(g, 'footprint_depth_m', 0.08)),
                transport_suitability=int(getattr(g, 'transport_suitability', 1)),
                track_id=tid,
                confirmed=True,
                drop_xy=drop_xy,
            )
            if entry is not None:
                updated.append(entry)
        return updated

    def mark_obstacle(
        self,
        x: float,
        y: float,
        *,
        radius_m: float = 0.12,
        category: str = 'obstacle',
        mobility: str = 'stationary',
        relevance: str = 'deferred',
        detail: str = 'OBSTACLE',
        track_id: str = '',
    ) -> WorldObject:
        entry = self.upsert_object(
            x,
            y,
            category=category,
            mobility=mobility if mobility in MOBILITY_CLASSES else 'stationary',
            relevance=relevance if relevance in ('deferred', 'ignored') else 'deferred',
            radius_m=max(0.05, float(radius_m)),
            width_m=2.0 * float(radius_m),
            depth_m=2.0 * float(radius_m),
            track_id=track_id,
            detail=detail,
            confirmed=True,
        )
        assert entry is not None
        return entry

    def mark_completed(
        self,
        x: float,
        y: float,
        *,
        track_id: str = '',
        category: str = 'object',
        mobility: str = 'pickupable',
        radius_m: float = 0.12,
        detail: str = 'COMPLETED',
    ) -> WorldObject:
        existing = (
            self.entries.get(track_id)
            if track_id and track_id in self.entries
            else self._find_near(x, y)
        )
        mob = existing.mobility if existing is not None else (mobility if mobility in MOBILITY_CLASSES else 'pickupable')
        entry = self.upsert_object(
            x,
            y,
            category=category,
            mobility=mob,
            relevance='completed',
            radius_m=max(0.08, float(radius_m)),
            track_id=track_id,
            detail=detail,
            confirmed=True,
        )
        assert entry is not None
        return entry

    # Alias for existing callers
    mark_delivered = mark_completed

    def mark_attempt_failed(self, x: float, y: float, *, track_id: str = '', detail: str = 'FAILED') -> WorldObject | None:
        entry = (
            self.entries.get(track_id)
            if track_id and track_id in self.entries
            else self._find_near(x, y)
        )
        if entry is None:
            return None
        entry.attempts += 1
        entry.detail = str(detail)
        entry.updated_at = time.time()
        if entry.attempts >= self.max_attempts:
            entry.relevance = 'deferred'
        else:
            entry.relevance = 'objective'
        return entry

    def apply_event(self, payload: dict) -> bool:
        if not isinstance(payload, dict):
            return False
        action = payload.get('action')
        if action == 'clear':
            self.clear()
            return True
        if action == 'upsert':
            self.upsert_object(
                float(payload['x']),
                float(payload['y']),
                category=str(payload.get('category', 'object')),
                mobility=str(payload.get('mobility', 'pickupable')),
                relevance=str(payload.get('relevance', 'objective')),
                width_m=float(payload.get('width_m', 0.08)),
                depth_m=float(payload.get('depth_m', 0.08)),
                radius_m=float(payload['radius_m']) if 'radius_m' in payload and payload['radius_m'] is not None else None,
                track_id=str(payload.get('track_id', '')),
                detail=str(payload.get('detail', '')),
                confirmed=True,
            )
            return True
        if action == 'obstacle':
            self.mark_obstacle(
                float(payload['x']),
                float(payload['y']),
                radius_m=float(payload.get('radius_m', 0.12)),
                category=str(payload.get('category', 'obstacle')),
                mobility=str(payload.get('mobility', 'stationary')),
                relevance=str(payload.get('relevance', 'deferred')),
                detail=str(payload.get('detail', 'OBSTACLE')),
                track_id=str(payload.get('track_id', '')),
            )
            return True
        if action in ('completed', 'delivered'):
            self.mark_completed(
                float(payload['x']),
                float(payload['y']),
                track_id=str(payload.get('track_id', '')),
                category=str(payload.get('category', 'object')),
                mobility=str(payload.get('mobility', 'pickupable')),
                radius_m=float(payload.get('radius_m', 0.12)),
                detail=str(payload.get('detail', 'COMPLETED')),
            )
            return True
        if action == 'failed':
            self.mark_attempt_failed(
                float(payload['x']),
                float(payload['y']),
                track_id=str(payload.get('track_id', '')),
                detail=str(payload.get('detail', 'FAILED')),
            )
            return True
        if action == 'objective':
            mob = str(payload.get('mobility') or mobility_from_suitability(int(payload.get('transport_suitability', 1))))
            self.upsert_object(
                float(payload['x']),
                float(payload['y']),
                category=str(payload.get('category', 'object')),
                mobility=mob,
                relevance=str(payload.get('relevance', 'objective')),
                width_m=float(payload.get('width_m', 0.08)),
                depth_m=float(payload.get('depth_m', 0.08)),
                track_id=str(payload.get('track_id', '')),
                confirmed=True,
            )
            return True
        return False

    def load_entries(self, data: dict) -> None:
        if not isinstance(data, dict):
            return
        for raw in data.get('entries', ()):
            if not isinstance(raw, dict) or 'id' not in raw:
                continue
            fields = {k: raw[k] for k in WorldObject.__dataclass_fields__ if k in raw}
            self.entries[str(raw['id'])] = WorldObject(**fields)

    def pending_objectives(
        self,
        pose_xy: tuple[float, float] | None = None,
        *,
        allowed_mobility: tuple[str, ...] = ('pickupable', 'draggable'),
    ) -> list[WorldObject]:
        items = [
            e for e in self.entries.values()
            if e.relevance == 'objective' and e.mobility in allowed_mobility and e.attempts < self.max_attempts
        ]
        if pose_xy is not None:
            items.sort(key=lambda e: (math.hypot(e.x - pose_xy[0], e.y - pose_xy[1]), e.id))
        else:
            items.sort(key=lambda e: e.id)
        return items

    def obstacle_entries(self, include_completed: bool = False) -> list[WorldObject]:
        return [
            e for e in self.entries.values()
            if e.is_obstacle or (include_completed and e.relevance == 'completed')
        ]

    def obstacle_mask(
        self,
        width: int | None = None,
        height: int | None = None,
        origin: tuple[float, float] | None = None,
        resolution: float | None = None,
        *,
        include_completed: bool = False,
        protect_xy: tuple[float, float] | None = None,
        protect_radius_m: float = 0.28,
    ) -> np.ndarray:
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

    def occupancy_grid_data(
        self,
        width: int | None = None,
        height: int | None = None,
        origin: tuple[float, float] | None = None,
        resolution: float | None = None,
    ) -> np.ndarray:
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
            by_mobility={
                mob: [e.to_dict() for e in self.entries.values() if e.mobility == mob]
                for mob in MOBILITY_CLASSES
            },
            entries=[e.to_dict() for e in self.entries.values()],
        )

    @classmethod
    def from_summary(cls, data: dict) -> 'SessionWorldMap':
        world = cls()
        world.load_entries(data)
        return world
