"""HSV color segmentation, floor projection, garment geometry, and ArUco ID68 wrist pose estimation."""
from functools import lru_cache
import json
import math
from pathlib import Path
import uuid
import cv2
import numpy as np
import yaml
from z70_description import pincher_geometry as hand
from z70_description.arm_model import config_path, package_root

_PACKAGE_ROOT = Path(__file__).resolve().parent.parent


class FrontCamera:
    """Pixel <-> floor plane projection for the Z70 front camera."""

    def __init__(self, f, cx, cy, k1, k2, R_floor_to_cam, height_mm, width, height):
        self.f, self.cx, self.cy, self.k1, self.k2 = f, cx, cy, k1, k2
        self.R = np.asarray(R_floor_to_cam, float)
        self.h = height_mm
        self.C = np.array([0.0, 0.0, height_mm])
        self.width, self.height = width, height

    @classmethod
    def load(cls, path):
        d = json.loads(Path(path).read_text())
        return cls(d['f_px'], d['cx'], d['cy'], d['k1'], d['k2'], d['R_floor_to_cam'],
                   d['camera_height_mm'], d['width'], d['height'])

    def _distort(self, x, y):
        r2 = x * x + y * y
        s = 1 + self.k1 * r2 + self.k2 * r2 * r2
        return x * s, y * s

    def _undistort(self, xd, yd):
        x, y = xd, yd
        for _ in range(30):
            r2 = x * x + y * y
            s = 1 + self.k1 * r2 + self.k2 * r2 * r2
            x, y = xd / s, yd / s
        return x, y

    def pixel_to_ray(self, u, v):
        x, y = self._undistort((u - self.cx) / self.f, (v - self.cy) / self.f)
        d = self.R.T @ np.array([x, y, 1.0])
        return d / np.linalg.norm(d)

    def pixel_to_floor(self, u, v):
        d = self.pixel_to_ray(u, v)
        if d[2] >= -1e-9:
            return None
        t = self.h / -d[2]
        p = self.C + t * d
        return dict(forward_mm=float(p[0]), left_mm=float(p[1]),
                    range_mm=float(math.hypot(p[0], p[1])),
                    bearing_deg=float(math.degrees(math.atan2(p[1], p[0]))))

    def floor_to_pixel(self, forward_mm, left_mm, z_mm=0.0):
        pc = self.R @ (np.array([forward_mm, left_mm, z_mm]) - self.C)
        if pc[2] <= 0:
            return None
        x, y = self._distort(pc[0] / pc[2], pc[1] / pc[2])
        return float(self.f * x + self.cx), float(self.f * y + self.cy)

    def horizon_v(self, u=None):
        u = self.cx if u is None else u
        lo, hi = 0.0, float(self.height)
        for _ in range(40):
            mid = (lo + hi) / 2
            if self.pixel_to_ray(u, mid)[2] < 0:
                hi = mid
            else:
                lo = mid
        return hi

@lru_cache(maxsize=1)
def calibration():
    return yaml.safe_load(config_path('mount_calibration.yaml').read_text())

@lru_cache(maxsize=1)
def front_camera():
    return FrontCamera.load(config_path('front_camera.json'))

@lru_cache(maxsize=1)
def gripper_camera_info():
    """Load gripper camera intrinsics and distortion parameters."""
    return yaml.safe_load(config_path('z70_gripper_camera_info.yaml').read_text())

@lru_cache(maxsize=1)
def colors():
    path = package_root('z70_manipulation', _PACKAGE_ROOT) / 'config/colors.yaml'
    if not path.is_file():
        raise FileNotFoundError(path)
    return yaml.safe_load(path.read_text())

def detect(image,thresholds=None,min_area=70,keep_masks=False):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV); found=[]
    kernel=np.ones((3,3),np.uint8)
    for color,c in (thresholds or colors()).items():
        mask=cv2.inRange(hsv,np.array([c['h'][0],c['s'][0],c['v'][0]]),np.array([c['h'][1],c['s'][1],c['v'][1]]))
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area=cv2.contourArea(contour)
            if area<min_area: continue
            x,y,w,h=cv2.boundingRect(contour)
            hull_area=cv2.contourArea(cv2.convexHull(contour))
            if w/h>3 or area/max(hull_area,1)<.5: continue
            moments=cv2.moments(contour)
            cx=moments['m10']/moments['m00']; cy=moments['m01']/moments['m00']
            dense=np.zeros((h,w),np.uint8)
            local=contour.copy();local[:,:,0]-=x;local[:,:,1]-=y
            cv2.drawContours(dense,[local],-1,255,-1)
            ys,xs=np.nonzero(dense[max(0,h-4):,:])
            contact_u=x+float(np.median(xs))
            found.append(dict(color=color,pixel=[cx,cy],contact_px=[contact_u,float(y+h-1)],
                              bbox=[int(x),int(y),int(w),int(h)],size_px=float(max(w,h)),area_px=float(area),image_size=[image.shape[1],image.shape[0]],
                              _mask=dense))
    found=sorted(found,key=lambda b:(b['color'],-b['area_px']))
    return found if keep_masks else [{k:v for k,v in b.items() if k!='_mask'} for b in found]

def front_objects(image,thresholds=None):
    cam=front_camera(); offset=calibration()['front_camera_x_m']; result=[]
    sx=cam.width/image.shape[1]; sy=cam.height/image.shape[0]
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    for b in detect(image,thresholds,keep_masks=True):
        mask=b.pop('_mask')
        u,v=b['contact_px']; point=cam.pixel_to_floor(u*sx,v*sy)
        if point is None or not 0<point['forward_mm']<3000: continue
        def apply_trim(sub,nx,ny,nw,nh,tag):
            ys,xs=np.nonzero(sub)
            if len(xs)==0: return None
            bottom=ys.max()
            b['bbox']=[nx,ny,nw,nh]; b['trimmed']=tag
            b['pixel']=[nx+float(xs.mean()),ny+float(ys.mean())]
            b['contact_px']=[nx+float(np.median(xs[ys>=bottom-3])),float(ny+bottom)]
            b['size_px']=float(max(nw,nh))
            cu,cv=b['contact_px']
            return cu,cv,cam.pixel_to_floor(cu*sx,cv*sy),len(xs)
        # Trim merged reflections or background regions using high-saturation windows.
        x,y,w,h=b['bbox']
        sat_patch=hsv[y:y+h,x:x+w,1]
        core=cv2.morphologyEx(((mask>0)&(sat_patch>=165)).astype(np.uint8)*255,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
        core_cnts,_=cv2.findContours(core,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        if core_cnts and b['color'] in ('yellow', 'orange'):
            best_c=max(core_cnts,key=cv2.contourArea)
            if cv2.contourArea(best_c)>=70:
                x0,y0,cw,ch=cv2.boundingRect(best_c)
                if cw<0.85*w or ch<0.85*h:
                    sub=np.zeros((ch,cw),np.uint8)
                    local=best_c.copy();local[:,:,0]-=x0;local[:,:,1]-=y0
                    cv2.drawContours(sub,[local],-1,255,-1)
                    trimmed=apply_trim(sub,x+x0,y+y0,cw,ch,'sat_core')
                    if trimmed is None: continue
                    u,v,point,area_px=trimmed
                    b['area_px']=float(area_px); mask=sub; x,y,w,h=b['bbox']
                    if point is None or not 0<point['forward_mm']<3000: continue
        if w>1.4*h:
            keepw=int(round(1.1*h)); satw=hsv[y:y+h,x:x+w,1].astype(float)
            def scorew(c0): m=mask[:,c0:c0+keepw]>0; return float(satw[:,c0:c0+keepw][m].mean()) if m.any() else -1
            starts=range(0,w-keepw+1,max(1,keepw//8)); c0=max(starts,key=scorew)
            sub=mask[:,c0:c0+keepw]; b['trimmed_from_w']=w
            trimmed=apply_trim(sub,x+c0,y,keepw,h,'sides')
            if trimmed is None: continue
            u,v,point,_=trimmed
            mask=np.zeros_like(mask); mask[:,c0:c0+keepw]=sub
            if point is None: continue
            x=x+c0; w=keepw
        if h>1.4*w:
            keep=int(round(1.15*w)); sat=hsv[y:y+h,x:x+w,1].astype(float)
            def score(r0): m=mask[r0:r0+keep]>0; return float(sat[r0:r0+keep][m].mean()) if m.any() else -1
            r0=0 if score(0)>=score(h-keep) else h-keep
            sub=mask[r0:r0+keep]; b['trimmed_from_h']=h
            trimmed=apply_trim(sub,x,y+r0,w,keep,'top' if r0==0 else 'bottom')
            if trimmed is None: continue
            u,v,point,_=trimmed
            if point is None: continue
        b['position']=[point['forward_mm']/1000+offset,point['left_mm']/1000,0.]
        b['camera']='front'
        left=cam.pixel_to_floor((u-b['bbox'][2]/2)*sx,v*sy)
        right=cam.pixel_to_floor((u+b['bbox'][2]/2)*sx,v*sy)
        if not left or not right: continue
        size_m=abs(left['left_mm']-right['left_mm'])/1000
        limits=(thresholds or colors())[b['color']].get('size_m',[.018,.085])
        aspect=max(b['bbox'][2]/b['bbox'][3],b['bbox'][3]/b['bbox'][2])
        if not limits[0]<=size_m<=limits[1] or aspect>(thresholds or colors())[b['color']].get('max_aspect',1.8): continue
        b['size_m']=float(size_m)
        result.append(b)
    return result

def front_jaw(image):
    """Estimate elevated ArUco ID68 wrist pose and grasp reference in base_footprint."""
    cam=front_camera();cfg=calibration()
    if image.shape[:2]!=(cam.height,cam.width): raise LookupError('front frame dimensions')
    K=np.array([[cam.f,0,cam.cx],[0,cam.f,cam.cy],[0,0,1.]])
    D=np.array([cam.k1,cam.k2,0.,0.,0.])
    p=cv2.aruco.DetectorParameters();p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX;p.adaptiveThreshWinSizeMax=53
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),p)
    corners,ids,_=detector.detectMarkers(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY))
    candidates=[];s=.0075
    obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]])
    for px,i in zip(corners,ids.flatten() if ids is not None else []):
        if int(i)!=68: continue
        px=px.reshape(4,2)
        ok,r,t=cv2.solvePnP(obj,px,K,D,flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok: continue
        r,t=cv2.solvePnPRefineLM(obj,px,K,D,r,t)
        R=cv2.Rodrigues(r)[0];C=np.array([0,0,cam.h/1000])
        marker=C+cam.R.T@t.ravel()
        rms=float(np.sqrt(np.mean(np.sum((cv2.projectPoints(obj,r,t,K,D)[0].reshape(4,2)-px)**2,axis=1))))
        if marker[2]<.025 or rms>2. or not .035<marker[0]<.45: continue
        marker[0]+=cfg['front_camera_x_m']
        marker_pose=np.eye(4);marker_pose[:3,:3]=cam.R.T@R;marker_pose[:3,3]=marker
        hand_pose=hand.marker_hand_pose(marker_pose)
        operational=hand_pose[:3,:3]@hand.grasp_reference()+hand_pose[:3,3]
        candidates.append(dict(position=operational.tolist(),
                               marker_pose_base=marker_pose.tolist(),hand_pose_base=hand_pose.tolist(),
                               marker_position=marker.tolist(),marker_id=68,marker_corners=px.tolist(),rms_px=rms,
                               uncertainty_bound_m=max(hand.CONFIG['uncertainty_m'],cfg.get('jaw_uncertainty_bound_m',.015))))
    if len(candidates)!=1: raise LookupError('elevated wrist marker 68 missing or ambiguous')
    return candidates[0]

def ray_floor(pixel,K,D,translation,rotation):
    uv=cv2.undistortPoints(np.array([[pixel]],float),np.asarray(K,float),np.asarray(D,float))[0,0]
    ray=np.asarray(rotation,float)@np.array([uv[0],uv[1],1.])
    C=np.asarray(translation,float)
    if ray[2]>=-1e-8 or C[2]<=0: return None
    p=C-C[2]/ray[2]*ray
    return p.tolist()


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
    if ys.max() >= .94*camera.height:
        return result
    columns = np.flatnonzero(mask.any(axis=0))
    bottom = camera.height-1-np.argmax(mask[::-1, columns], axis=0)
    low = float(np.percentile(bottom, 95))
    edge = [(int(u), int(v)) for u, v in zip(columns, bottom) if low-4 <= v <= low+4
            and np.count_nonzero(mask[max(0, v-3):v+1, max(0, u-2):u+3]) >= 10]
    candidates = [(px, project(camera, px, camera_x_m)) for px in edge]
    candidates = [(px, p) for px, p in candidates if p is not None]
    if len(candidates) < 5:
        return result
    px, p = min(candidates, key=lambda item: (item[1][0], abs(item[1][1])))
    points = np.array([p for _, p in candidates])
    dense_bottom = [((int(u), int(v)), project(camera, (int(u), int(v)), camera_x_m))
                    for u, v in zip(columns, bottom)
                    if np.count_nonzero(mask[max(0, int(v)-3):int(v)+1, max(0, int(u)-2):int(u)+3]) >= 10]
    dense_bottom = [(q, bp) for q, bp in dense_bottom if bp is not None]
    bottom_pts = np.array([bp for _, bp in dense_bottom])
    width_m = float(max(np.ptp(points[:, 1]), np.ptp(bottom_pts[:, 1]) if len(bottom_pts) else 0.))
    if len(dense_bottom) >= 5 and width_m > .15:
        px, p = min(dense_bottom, key=lambda item: (abs(item[1][1] - -.012), item[1][0]))
    result.update(contact_px=list(map(float, px)), floor_contact_m=p.tolist(), contact_valid=True,
                  position_uncertainty_m=position_uncertainty(camera, px),
                  footprint_width_m=width_m, footprint_depth_m=float(np.ptp(points[:, 0])))
    pool = dense_bottom if len(dense_bottom) >= 5 and width_m > .15 else candidates
    reachable = [(q, p) for q, p in pool if .26 <= p[0] <= .45 and abs(p[1]) <= .15]
    if reachable and not box_only and result['footprint_width_m'] >= .008:
        key_fn = (lambda item: (abs(item[1][1] - -.012), item[1][0])) if width_m > .15 else (lambda item: (item[1][0], abs(item[1][1])))
        q, gp = min(reachable, key=key_fn)
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
        self.tracks = {}

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
                obs.update(track_id=str(uuid.UUID(int=0)), confirmed=False, hits=0, misses=0)
                output.append(obs); continue
            ident = assigned.get(i, str(uuid.uuid4()))
            t = self.tracks.get(ident, dict(hits=0, misses=0, category=obs.get('category')))
            t['hits'] = t['hits']+1 if t['misses'] <= 1 else 1
            t.update(position=list(obs['floor_contact_m']), misses=0)
            self.tracks[ident] = t
            obs.update(track_id=ident, hits=t['hits'], misses=0, confirmed=t['hits'] >= 2)
            output.append(obs)
        self.tracks = {k: v for k, v in self.tracks.items() if v['misses'] <= self.max_misses}
        return output
