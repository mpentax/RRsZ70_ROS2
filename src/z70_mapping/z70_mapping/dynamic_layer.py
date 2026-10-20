"""Session-scoped dynamic obstacle layer and map-frame semantic object registry."""
from dataclasses import asdict, dataclass
import math
import time
import uuid

import cv2
import numpy as np


@dataclass
class DynamicEntry:
    id: str
    kind: str  # 'objective' | 'obstacle' | 'delivered_pile'
    category: str
    status: str  # 'PENDING' | 'IN_FLIGHT' | 'DELIVERED' | 'SKIPPED_OBSTACLE' | 'FAILED_OBSTACLE' | 'OBSTACLE'
    x: float
    y: float
    radius_m: float = 0.08
    width_m: float = 0.08
    depth_m: float = 0.08
    transport_suitability: int = 1
    track_id: str = ''
    hits: int = 1
    attempts: int = 0
    detail: str = ''
    updated_at: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


class SessionWorldMap:
    """In-memory, session-scoped dynamic obstacle raster and semantic object registry.

    Resets automatically when the map session or floor geometry changes.
    """

    OBSTACLE_STATUSES = {'SKIPPED_OBSTACLE', 'FAILED_OBSTACLE', 'OBSTACLE'}
    PILE_STATUSES = {'DELIVERED'}
    ACTIVE_OBJECTIVE_STATUSES = {'PENDING', 'IN_FLIGHT'}

    def __init__(self, merge_radius_m: float = 0.18, max_attempts: int = 2):
        self.merge_radius_m = float(merge_radius_m)
        self.max_attempts = int(max_attempts)
        self.session_key: tuple | None = None
        self.origin: tuple[float, float] = (0.0, 0.0)
        self.resolution: float = 0.05
        self.width: int = 0
        self.height: int = 0
        self.entries: dict[str, DynamicEntry] = {}
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

    def _find_near(self, x: float, y: float, radius_m: float | None = None) -> DynamicEntry | None:
        gate = self.merge_radius_m if radius_m is None else float(radius_m)
        best: DynamicEntry | None = None
        best_d = gate
        for entry in self.entries.values():
            d = math.hypot(entry.x - x, entry.y - y)
            if d <= max(best_d, entry.radius_m):
                best = entry
                best_d = d
        return best

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
    ) -> DynamicEntry | None:
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        if drop_xy is not None and math.hypot(x - drop_xy[0], y - drop_xy[1]) <= 0.45:
            if any(e.status == 'DELIVERED' for e in self.entries.values()):
                return None
        existing = (
            self.entries.get(track_id)
            if track_id and track_id in self.entries
            else self._find_near(x, y)
        )
        radius = float(max(0.06, min(0.35, 0.5 * max(width_m, depth_m, 0.12))))
        now = time.time()
        if existing is not None:
            if existing.status in ('DELIVERED', 'SKIPPED_OBSTACLE', 'FAILED_OBSTACLE', 'OBSTACLE'):
                return existing
            alpha = 0.35
            existing.x = float((1.0 - alpha) * existing.x + alpha * x)
            existing.y = float((1.0 - alpha) * existing.y + alpha * y)
            existing.width_m = float(max(existing.width_m, width_m))
            existing.depth_m = float(max(existing.depth_m, depth_m))
            existing.radius_m = float(max(existing.radius_m, radius))
            if transport_suitability:
                existing.transport_suitability = int(transport_suitability)
            if category and category != 'garment':
                existing.category = str(category)
            if track_id:
                existing.track_id = str(track_id)
            existing.hits += 1
            existing.updated_at = now
            return existing
        if not confirmed:
            return None
        ident = str(track_id) if track_id else str(uuid.uuid4().hex[:12])
        entry = DynamicEntry(
            id=ident,
            kind='objective',
            category=str(category or 'garment'),
            status='PENDING',
            x=float(x),
            y=float(y),
            radius_m=radius,
            width_m=float(width_m),
            depth_m=float(depth_m),
            transport_suitability=int(transport_suitability),
            track_id=str(track_id),
            hits=1,
            attempts=0,
            updated_at=now,
        )
        self.entries[ident] = entry
        return entry

    def ingest_observations(self, batch, pose: tuple[float, float, float] | None, *, drop_xy: tuple[float, float] | None = None) -> list[DynamicEntry]:
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
        detail: str = 'OBSTACLE',
        track_id: str = '',
        status: str = 'OBSTACLE',
    ) -> DynamicEntry:
        existing = (
            self.entries.get(track_id)
            if track_id and track_id in self.entries
            else self._find_near(x, y, max(self.merge_radius_m, radius_m))
        )
        now = time.time()
        if existing is not None:
            existing.kind = 'obstacle'
            existing.status = status if status in self.OBSTACLE_STATUSES else 'OBSTACLE'
            existing.x = float(x)
            existing.y = float(y)
            existing.radius_m = float(max(existing.radius_m, radius_m))
            existing.detail = str(detail)
            existing.updated_at = now
            return existing
        ident = str(track_id) if track_id else f'obs_{uuid.uuid4().hex[:8]}'
        entry = DynamicEntry(
            id=ident,
            kind='obstacle',
            category=str(category),
            status=status if status in self.OBSTACLE_STATUSES else 'OBSTACLE',
            x=float(x),
            y=float(y),
            radius_m=float(max(0.05, radius_m)),
            width_m=float(2.0 * radius_m),
            depth_m=float(2.0 * radius_m),
            track_id=str(track_id),
            detail=str(detail),
            updated_at=now,
        )
        self.entries[ident] = entry
        return entry

    def mark_delivered(
        self,
        x: float,
        y: float,
        *,
        track_id: str = '',
        category: str = 'garment',
        radius_m: float = 0.12,
        detail: str = 'DELIVERED',
    ) -> DynamicEntry:
        existing = (
            self.entries.get(track_id)
            if track_id and track_id in self.entries
            else self._find_near(x, y)
        )
        now = time.time()
        if existing is not None:
            existing.kind = 'delivered_pile'
            existing.status = 'DELIVERED'
            existing.x = float(x)
            existing.y = float(y)
            existing.radius_m = float(max(existing.radius_m, radius_m))
            existing.detail = str(detail)
            existing.updated_at = now
            return existing
        ident = str(track_id) if track_id else f'pile_{uuid.uuid4().hex[:8]}'
        entry = DynamicEntry(
            id=ident,
            kind='delivered_pile',
            category=str(category),
            status='DELIVERED',
            x=float(x),
            y=float(y),
            radius_m=float(max(0.08, radius_m)),
            track_id=str(track_id),
            detail=str(detail),
            updated_at=now,
        )
        self.entries[ident] = entry
        return entry

    def mark_attempt_failed(self, x: float, y: float, *, track_id: str = '', detail: str = 'FAILED') -> DynamicEntry | None:
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
            entry.kind = 'obstacle'
            entry.status = 'FAILED_OBSTACLE'
        else:
            entry.status = 'PENDING'
        return entry

    def apply_event(self, payload: dict) -> bool:
        if not isinstance(payload, dict):
            return False
        action = payload.get('action')
        if action == 'clear':
            self.clear()
            return True
        if action == 'obstacle':
            self.mark_obstacle(
                float(payload['x']),
                float(payload['y']),
                radius_m=float(payload.get('radius_m', 0.12)),
                category=str(payload.get('category', 'obstacle')),
                detail=str(payload.get('detail', 'OBSTACLE')),
                track_id=str(payload.get('track_id', '')),
                status=str(payload.get('status', 'OBSTACLE')),
            )
            return True
        if action == 'delivered':
            self.mark_delivered(
                float(payload['x']),
                float(payload['y']),
                track_id=str(payload.get('track_id', '')),
                category=str(payload.get('category', 'garment')),
                radius_m=float(payload.get('radius_m', 0.12)),
                detail=str(payload.get('detail', 'DELIVERED')),
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
            self.observe_garment(
                float(payload['x']),
                float(payload['y']),
                category=str(payload.get('category', 'garment')),
                width_m=float(payload.get('width_m', 0.08)),
                depth_m=float(payload.get('depth_m', 0.08)),
                transport_suitability=int(payload.get('transport_suitability', 1)),
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
            fields = {k: raw[k] for k in DynamicEntry.__dataclass_fields__ if k in raw}
            self.entries[str(raw['id'])] = DynamicEntry(**fields)

    def pending_objectives(self, pose_xy: tuple[float, float] | None = None) -> list[DynamicEntry]:
        items = [
            e for e in self.entries.values()
            if e.kind == 'objective' and e.status == 'PENDING' and e.attempts < self.max_attempts
        ]
        if pose_xy is not None:
            items.sort(key=lambda e: (math.hypot(e.x - pose_xy[0], e.y - pose_xy[1]), e.id))
        else:
            items.sort(key=lambda e: e.id)
        return items

    def obstacle_entries(self, include_piles: bool = False) -> list[DynamicEntry]:
        statuses = self.OBSTACLE_STATUSES | (self.PILE_STATUSES if include_piles else set())
        return [e for e in self.entries.values() if e.status in statuses]

    def obstacle_mask(
        self,
        width: int | None = None,
        height: int | None = None,
        origin: tuple[float, float] | None = None,
        resolution: float | None = None,
        *,
        include_piles: bool = False,
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
        for entry in self.obstacle_entries(include_piles=include_piles):
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
        obs = self.obstacle_mask(width, height, origin, resolution, include_piles=False)
        piles = self.obstacle_mask(width, height, origin, resolution, include_piles=True) & ~obs
        grid = np.zeros(obs.shape, dtype=np.int8)
        grid[piles] = 50
        grid[obs] = 100
        return grid

    def summary(self, pose_xy: tuple[float, float] | None = None) -> dict:
        return dict(
            session_bound=self.session_key is not None,
            objectives=[e.to_dict() for e in self.pending_objectives(pose_xy)],
            obstacles=[e.to_dict() for e in self.obstacle_entries(include_piles=False)],
            delivered_piles=[e.to_dict() for e in self.entries.values() if e.status == 'DELIVERED'],
            entries=[e.to_dict() for e in self.entries.values()],
        )

    @classmethod
    def from_summary(cls, data: dict) -> 'SessionWorldMap':
        world = cls()
        world.load_entries(data)
        return world
