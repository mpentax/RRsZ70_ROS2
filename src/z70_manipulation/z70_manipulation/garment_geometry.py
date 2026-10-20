"""Garment floor-contact projection, geofence filtering, and multi-frame tracking."""
import math
import uuid
import cv2
import numpy as np

SUITABILITY_UNKNOWN = 0
SMALL_LIFT_CANDIDATE = 1
TOW_CANDIDATE = 2
UNSUPPORTED = 3


def suitability(width, depth, *, qualified_small_lift=False, max_small_extent_m=.15,
                allow_tow=False):
    if qualified_small_lift and max(width, depth) <= max_small_extent_m:
        return SMALL_LIFT_CANDIDATE, 'QUALIFIED_SMALL_LIFT_POLICY'
    if qualified_small_lift and allow_tow:
        return TOW_CANDIDATE, 'TOW_CANDIDATE'
    return SUITABILITY_UNKNOWN, 'UNQUALIFIED'


def project(camera, pixel, camera_x_m=.275):
    u, v = pixel
    if not (0 <= u < camera.width and 0 <= v < .94*camera.height):
        return None
    p = camera.pixel_to_floor(float(u), float(v))
    if p is None or not np.isfinite([p['forward_mm'], p['left_mm']]).all():
        return None
    if not 0 < p['forward_mm'] <= 3000:
        return None
    return np.array([camera_x_m+p['forward_mm']/1000, p['left_mm']/1000, 0.])


def position_uncertainty(camera, pixel, pixel_sigma=2.):
    """Largest floor-plane pixel Jacobian singular value plus 5mm model floor."""
    u, v = pixel
    points = [project(camera, p, 0.) for p in ((u-1, v), (u+1, v), (u, v-1), (u, v+1))]
    if any(p is None for p in points):
        return 10.
    jac = np.column_stack(((points[1]-points[0])[:2]/2, (points[3]-points[2])[:2]/2))
    return float(.005+pixel_sigma*np.linalg.svd(jac, compute_uv=False)[0])


def observe(mask, camera, *, bbox=None, category='garment', confidence=0.,
            camera_x_m=.275, calibration_id='', qualified_policy=None):
    result = dict(category=str(category)[:32], confidence=float(np.clip(confidence, 0, 1)),
                  bbox_px=[0]*4, contact_px=[0., 0.], grasp_px=[0., 0.],
                  floor_contact_m=[0., 0., 0.], grasp_point_m=[0., 0., 0.],
                  contact_valid=False, grasp_valid=False, footprint_width_m=0.,
                  footprint_depth_m=0., position_uncertainty_m=10., inside_geofence=False,
                  geofence_verified=False, reachable_for_grasp=False,
                  transport_suitability=0, suitability_reason='UNQUALIFIED',
                  calibration_id=str(calibration_id)[:128])
    box_only = mask is None
    if box_only:
        mask = np.zeros((camera.height, camera.width), np.uint8)
        if bbox is None or len(bbox) != 4:
            return result
        x, y, w, h = map(int, bbox)
        if w <= 0 or h <= 0 or x < 0 or y < 0 or x+w > camera.width or y+h > camera.height:
            return result
        mask[y:y+h, x:x+w] = 1
    mask = np.asarray(mask)
    if mask.shape != (camera.height, camera.width):
        raise ValueError('Mask dimensions do not match calibration')
    mask = (mask > 0).astype(np.uint8)
    ys, xs = np.nonzero(mask)
    if len(xs) < 20:
        return result
    x, y, w, h = cv2.boundingRect(mask)
    result['bbox_px'] = [x, y, w, h]
    # Never crop a bumper-occluded silhouette and treat the artificial cut as an edge.
    if ys.max() >= .94*camera.height:
        return result
    # Per-column bottom, retaining only a dense, lowest band. No upper contour.
    columns = np.flatnonzero(mask.any(axis=0))
    bottom = camera.height-1-np.argmax(mask[::-1, columns], axis=0)
    low = float(np.percentile(bottom, 95))
    edge = [(int(u), int(v)) for u, v in zip(columns, bottom) if low-4 <= v <= low+4
            and np.count_nonzero(mask[max(0, v-3):v+1, max(0, u-2):u+3]) >= 10]
    candidates = [(px, project(camera, px, camera_x_m)) for px in edge]
    candidates = [(px, p) for px, p in candidates if p is not None]
    if len(candidates) < 5:
        return result
    # Nearest forward contact, with centred tie-breaking to avoid arbitrary corners.
    px, p = min(candidates, key=lambda item: (item[1][0], abs(item[1][1])))
    points = np.array([p for _, p in candidates])
    result.update(contact_px=list(map(float, px)), floor_contact_m=p.tolist(), contact_valid=True,
                  position_uncertainty_m=position_uncertainty(camera, px),
                  footprint_width_m=float(np.ptp(points[:, 1])), footprint_depth_m=float(np.ptp(points[:, 0])))
    # Leading edge = dense local lower contour with enough lateral support to pinch.
    reachable = [(q, p) for q, p in candidates if .26 <= p[0] <= .45 and abs(p[1]) <= .15]
    if reachable and not box_only and result['footprint_width_m'] >= .008:
        q, gp = min(reachable, key=lambda item: (item[1][0], abs(item[1][1])))
        result.update(grasp_px=list(map(float, q)), grasp_point_m=gp.tolist(), grasp_valid=True,
                      reachable_for_grasp=True)
    result['transport_suitability'], result['suitability_reason'] = suitability(
        result['footprint_width_m'], result['footprint_depth_m'], **(qualified_policy or {}))
    return result


def transform_xy(position, pose):
    tx, ty, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = position[:2]
    return np.array([tx+c*x-s*y, ty+s*x+c*y])


def geofence(position, pose, grid, *, tf_age_s=0., grid_age_s=0., max_tf_age_s=.25, max_grid_age_s=5.):
    """Return (inside, verified); missing/stale/unknown cells are never free."""
    if (pose is None or grid is None or not np.isfinite([*position, *pose, tf_age_s, grid_age_s]).all()
            or not 0 <= tf_age_s <= max_tf_age_s or not 0 <= grid_age_s <= max_grid_age_s):
        return False, False
    try:
        if hasattr(grid, 'info'):
            if grid.header.frame_id != 'map':
                return False, False
            q = grid.info.origin.orientation
            if abs(q.x)+abs(q.y)+abs(q.z) > 1e-9 or abs(q.w-1) > 1e-9:
                return False, False
            cells = np.asarray(grid.data).reshape(grid.info.height, grid.info.width)
            origin = (grid.info.origin.position.x, grid.info.origin.position.y)
            resolution = grid.info.resolution
        else:
            cells, origin, resolution = np.asarray(grid.safe_grid), grid.origin, grid.resolution
        if cells.ndim != 2 or not np.isfinite([*origin, resolution]).all() or resolution <= 0:
            return False, False
        xy = np.floor((transform_xy(position, pose)-origin)/resolution).astype(int)
        x, y = xy
        return bool(0 <= y < cells.shape[0] and 0 <= x < cells.shape[1] and cells[y, x] == 0), True
    except (AttributeError, ValueError, TypeError):
        return False, False


def predict(position, odom_delta):
    """Stationary object's old-base position -> new base; delta in old base."""
    x, y = np.asarray(position[:2])-np.asarray(odom_delta[:2])
    c, s = math.cos(odom_delta[2]), math.sin(odom_delta[2])
    return [float(c*x+s*y), float(-s*x+c*y), float(position[2]) if len(position)>2 else 0.]


class GarmentTracker:
    def __init__(self, gate_m=.18, max_misses=4):
        self.gate_m, self.max_misses = gate_m, max_misses
        self.tracks = {}; self.id_switches = 0
        # ID-switch ground truth is available only in labelled offline replay.
        # Unlabelled live zero means no OBSERVED switches, not proven identity.
        self.instance_assignments = {}; self.instance_switches = {}

    def update(self, observations, odom_delta=(0., 0., 0.)):
        for t in self.tracks.values():
            t['position'] = predict(t['position'], odom_delta)
            t['misses'] += 1
        pairs = []
        for i, obs in enumerate(observations):
            if not obs.get('contact_valid'):
                continue
            for ident, t in self.tracks.items():
                d = float(np.linalg.norm(np.asarray(obs['floor_contact_m'][:2])-t['position'][:2]))
                if d <= self.gate_m and obs.get('category') == t.get('category'):
                    pairs.append((d, i, ident))
        used, assigned = set(), {}
        for _, i, ident in sorted(pairs):
            if i not in assigned and ident not in used:
                assigned[i] = ident; used.add(ident)
        output = []
        for i, obs in enumerate(observations):
            obs = dict(obs)
            if not obs.get('contact_valid'):
                obs.update(track_id=str(uuid.UUID(int=0)), confirmed=False, hits=0, misses=0, id_switches=0)
                output.append(obs); continue
            ident = assigned.get(i, str(uuid.uuid4()))
            t = self.tracks.get(ident, dict(hits=0, misses=0, category=obs.get('category'), id_switches=0))
            instance = obs.get('instance_id')
            if instance is not None:
                previous = self.instance_assignments.get(instance)
                if previous is not None and previous != ident:
                    self.id_switches += 1
                    self.instance_switches[instance] = self.instance_switches.get(instance, 0)+1
                self.instance_assignments[instance] = ident
                t['id_switches'] = self.instance_switches.get(instance, 0)
            # Confirmation needs consecutive consistent observations, not sightings separated by misses.
            t['hits'] = t['hits']+1 if t['misses'] <= 1 else 1
            t.update(position=list(obs['floor_contact_m']), misses=0)
            self.tracks[ident] = t
            obs.update(track_id=ident, hits=t['hits'], misses=0, id_switches=t['id_switches'], confirmed=t['hits'] >= 2)
            output.append(obs)
        self.tracks = {k: v for k, v in self.tracks.items() if v['misses'] <= self.max_misses}
        return output
