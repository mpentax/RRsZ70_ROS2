"""HSV color segmentation, floor projection, garment geometry/segmentation worker, and ArUco ID68 wrist pose estimation."""
import argparse
import contextlib
from copy import deepcopy
from functools import lru_cache
import io
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time
import uuid
import cv2
import numpy as np
import yaml

try:
    from z70_description import arm_model as hand
    from z70_description.arm_model import config_path, package_root
except ImportError:
    hand = config_path = package_root = None

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import Point
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener, TransformException
    from z70_interfaces.msg import GarmentObservation, GarmentObservations
    from z70_driver.driver import LATCHED, MatchedFrames, read_pipe_exact, stop_worker_process, worker_env
    from z70_mapping.node import quat_yaw, read_exact
except ImportError:
    Node = object
    LATCHED = None

    def read_exact(stream, count):
        data = bytearray()
        while len(data) < count:
            chunk = stream.read(count - len(data))
            if not chunk:
                raise EOFError('Worker input ended')
            data.extend(chunk)
        return bytes(data)

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


DINO_ID = 'IDEA-Research/grounding-dino-tiny'
DINO_REV = 'a2bb814dd30d776dcf7e30523b00659f4f141c71'
CLIP_ID = 'CIDAS/clipseg-rd64-refined'
CLIP_REV = '999e0328d9e10b484360c477313983f9afdd7050'
BACKENDS = ('grounding_dino_clipseg', 'grounding_dino', 'clipseg', 'sim_hsv', 'contour_fallback')
SIM_GARMENT_HSV = (
    ('sock', (5, 110, 65), (17, 255, 255)),
    ('towel', (39, 75, 45), (85, 255, 255)),
    ('shirt', (127, 65, 45), (168, 255, 255)),
)
PROMPTS = ['sock or shirt or towel or pants on floor', 'bare floor tile wood carpet',
           'furniture wall cabinet door']
MAX_PACKET = 8 * 1024 * 1024


def read_packet(stream):
    size = struct.unpack('<I', read_exact(stream, 4))[0]
    if not 0 < size <= MAX_PACKET:
        raise ValueError('Oversized or empty worker frame')
    return read_exact(stream, size)


def encode_result(detections, quality):
    packet = io.BytesIO()
    metadata = [{k: v for k, v in d.items() if k != 'mask'} for d in detections]
    masks = {f'mask_{i}': np.asarray(d['mask'], np.uint8) for i, d in enumerate(detections)}
    np.savez_compressed(packet, quality=np.array(json.dumps(quality)),
                        detections=np.array(json.dumps(metadata)), **masks)
    return packet.getvalue()


def decode_result(data):
    with np.load(io.BytesIO(data), allow_pickle=False) as packet:
        detections = json.loads(str(packet['detections']))
        if len(detections) > 32:
            raise ValueError('Too many detections')
        for i, d in enumerate(detections):
            mask = np.asarray(packet[f'mask_{i}'], np.uint8)
            if mask.ndim != 2 or mask.size > 1600 * 1200:
                raise ValueError('Invalid mask')
            d['mask'] = mask
        return detections, json.loads(str(packet['quality']))


def components(mask, confidence=.5, category='garment', min_area=150):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(np.asarray(mask, np.uint8), 8)
    result = []
    for i in sorted(range(1, n), key=lambda i: -stats[i, cv2.CC_STAT_AREA])[:32]:
        x, y, w, h, area = map(int, stats[i])
        if area < min_area or area > mask.size * .6:
            continue
        result.append(dict(category=category[:32], confidence=float(confidence), bbox=[x, y, w, h],
                           mask=(labels == i).astype(np.uint8)))
    return result


def refine_box(image, box):
    x0, y0, x1, y1 = box
    h, w = image.shape[:2]
    x0, y0 = max(0, int(x0)), max(0, int(y0))
    x1, y1 = min(w - 1, int(x1)), min(h - 1, int(y1))
    mask = np.zeros((h, w), np.uint8)
    if x1 - x0 < 3 or y1 - y0 < 3:
        return mask
    try:
        cv2.grabCut(image, mask, (x0, y0, x1 - x0, y1 - y0), np.zeros((1, 65)), np.zeros((1, 65)), 2, cv2.GC_INIT_WITH_RECT)
    except cv2.error:
        return np.zeros((h, w), np.uint8)
    mask = ((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD)).astype(np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


class GarmentModel:
    def __init__(self, backend='grounding_dino_clipseg', device='mps'):
        if backend not in BACKENDS:
            raise ValueError('Unknown garment backend')
        self.backend, self.device = backend, device
        self.dino = self.clip = None
        self.error = None
        os.environ['HF_HUB_OFFLINE'] = '1'; os.environ['TRANSFORMERS_OFFLINE'] = '1'
        os.environ['HF_HUB_DISABLE_TELEMETRY'] = '1'
        if backend in ('contour_fallback', 'sim_hsv'):
            return
        try:
            with contextlib.redirect_stdout(sys.stderr):
                import torch
                from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection, CLIPSegForImageSegmentation
                self.torch = torch
                if backend in ('grounding_dino', 'grounding_dino_clipseg'):
                    self.dino_proc = AutoProcessor.from_pretrained(DINO_ID, revision=DINO_REV, local_files_only=True)
                    self.dino = AutoModelForZeroShotObjectDetection.from_pretrained(DINO_ID, revision=DINO_REV, local_files_only=True).to(device).eval()
                if backend in ('clipseg', 'grounding_dino_clipseg'):
                    self.clip_proc = AutoProcessor.from_pretrained(CLIP_ID, revision=CLIP_REV, local_files_only=True)
                    self.clip = CLIPSegForImageSegmentation.from_pretrained(CLIP_ID, revision=CLIP_REV, local_files_only=True).to(device).eval()
        except Exception as error:
            self.error = type(error).__name__ + ': ' + str(error)[:240]
            self.dino = self.clip = None

    def sync(self):
        if self.device == 'mps' and hasattr(self, 'torch'):
            self.torch.mps.synchronize()

    def infer(self, image):
        self.sync(); start = time.monotonic()
        quality = dict(backend=self.backend, model_id=self.backend, model_available=self.error is None and self.backend != 'contour_fallback')
        if self.error:
            quality.update(error=self.error, inference_ms=0.)
            return [], quality
        detections = []
        if self.backend == 'contour_fallback':
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = ((hsv[:, :, 1] > 65) & (np.indices(image.shape[:2])[0] > image.shape[0] * .45)).astype(np.uint8)
            detections = components(mask, confidence=.1, category='unqualified_contour', min_area=50)
        elif self.backend == 'sim_hsv':
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            floor_band = (np.indices(image.shape[:2])[0] > image.shape[0] * .38).astype(np.uint8)
            kernel = np.ones((3, 3), np.uint8)
            min_area = max(50, image.shape[0] * image.shape[1] // 10000)
            for category, lo, hi in SIM_GARMENT_HSV:
                raw = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8)) > 0
                cleaned = cv2.morphologyEx((raw & (floor_band > 0)).astype(np.uint8), cv2.MORPH_OPEN, kernel)
                cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
                detections.extend(components(cleaned, confidence=.95, category=category, min_area=min_area))
        else:
            from PIL import Image
            rgb = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            with contextlib.redirect_stdout(sys.stderr), self.torch.inference_mode():
                clip_mask = None
                if self.clip is not None:
                    inputs = self.clip_proc(text=PROMPTS, images=[rgb] * 3, padding=True, return_tensors='pt').to(self.device)
                    logits = self.clip(**inputs).logits.float()
                    probabilities = self.torch.sigmoid(logits).cpu().numpy()
                    contrast = probabilities[0] - np.maximum(probabilities[1], probabilities[2])
                    clip_mask = ((cv2.resize(contrast, (image.shape[1], image.shape[0])) > .12) &
                                 (cv2.resize(probabilities[0], (image.shape[1], image.shape[0])) > .45)).astype(np.uint8)
                min_area = max(50, image.shape[0] * image.shape[1] // 10000)
                if self.dino is not None:
                    inputs = self.dino_proc(images=rgb, text='sock. shirt. towel. pants. jeans. hoodie.', return_tensors='pt').to(self.device)
                    prediction = self.dino(**inputs)
                    import inspect
                    post = self.dino_proc.post_process_grounded_object_detection
                    threshold_key = 'threshold' if 'threshold' in inspect.signature(post).parameters else 'box_threshold'
                    result = post(prediction, inputs.input_ids, **{threshold_key: .3}, text_threshold=.25,
                                  target_sizes=[image.shape[:2]])[0]
                    for box, score in zip(result['boxes'][:16], result['scores'][:16]):
                        mask = refine_box(image, box.cpu().numpy())
                        if clip_mask is not None:
                            mask &= clip_mask
                        detections.extend(components(mask, float(score.cpu()), min_area=min_area))
                elif clip_mask is not None:
                    detections = components(clip_mask, min_area=min_area)
        self.sync()
        quality['inference_ms'] = (time.monotonic() - start) * 1000
        if self.device == 'mps' and hasattr(self, 'torch'):
            quality['mps_allocated_bytes'] = self.torch.mps.current_allocated_memory()
            quality['mps_driver_bytes'] = self.torch.mps.driver_allocated_memory()
        return detections[:32], quality


def worker_main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=BACKENDS, default='grounding_dino_clipseg')
    parser.add_argument('--device', default='mps')
    args = parser.parse_args()
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    output = sys.stdout.buffer
    model = GarmentModel(args.backend, args.device)
    try:
        while True:
            try:
                jpeg = read_packet(sys.stdin.buffer)
            except EOFError:
                break
            try:
                image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if image is None or image.shape[0] * image.shape[1] > 1600 * 1200:
                    raise ValueError('Invalid JPEG or oversized image')
                detections, quality = model.infer(image)
            except Exception as error:
                detections, quality = [], dict(model_available=False, backend=args.backend, error=type(error).__name__)
            data = encode_result(detections, quality)
            if len(data) > MAX_PACKET:
                data = encode_result([], dict(model_available=False, error='OversizedResult'))
            output.write(struct.pack('<I', len(data)) + data); output.flush()
    finally:
        del model


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


stamp_seconds = seconds


def observations_message(items, meta, quality, geofence_active):
    msg = GarmentObservations()
    msg.header = deepcopy(meta.header); msg.header.frame_id = 'base_footprint'
    msg.evidence = deepcopy(meta.evidence)
    msg.model_id = str(quality.get('model_id', quality.get('backend', 'unavailable')))[:64]
    msg.model_available = bool(quality.get('model_available', False))
    msg.geofence_active = bool(geofence_active)
    scalar_fields = ('category', 'confidence', 'bbox_px', 'contact_px', 'grasp_px', 'contact_valid',
                     'grasp_valid', 'footprint_width_m', 'footprint_depth_m', 'position_uncertainty_m',
                     'inside_geofence', 'geofence_verified', 'reachable_for_grasp', 'transport_suitability',
                     'suitability_reason', 'calibration_id')
    for item in items:
        obs = GarmentObservation(); obs.header = deepcopy(msg.header); obs.evidence = deepcopy(msg.evidence)
        obs.track_id.uuid = list(uuid.UUID(item['track_id']).bytes)
        for field in scalar_fields:
            setattr(obs, field, item[field])
        for field in ('floor_contact_m', 'grasp_point_m'):
            x, y, z = map(float, item[field])
            setattr(obs, field, Point(x=x, y=y, z=z))
        msg.garments.append(obs)
    return msg


class GarmentNode(Node):
    def __init__(self):
        super().__init__('z70_garment_perception')
        python = self.declare_parameter('garment_python', str(Path.home() / '.venvs/roboclaw-depth/bin/python')).value
        backend = self.declare_parameter('backend', 'grounding_dino_clipseg').value
        device = self.declare_parameter('device', 'mps').value
        hz = float(self.declare_parameter('garment_hz', .5).value)
        self.max_age = float(self.declare_parameter('max_frame_age_s', 5.).value)
        qual_small = bool(self.declare_parameter('qualified_small_lift', backend == 'sim_hsv').value)
        allow_tow = bool(self.declare_parameter('allow_tow', backend == 'sim_hsv').value)
        self.qualified_policy = dict(qualified_small_lift=qual_small, allow_tow=allow_tow) if (qual_small or allow_tow) else None
        self.cam = front_camera(); self.camera_x = calibration()['front_camera_x_m']
        self.pub = self.create_publisher(GarmentObservations, '/z70/garments/observations', LATCHED)
        self.markers = self.create_publisher(MarkerArray, '/z70/garments', 1)
        self.json_pub = self.create_publisher(String, '/z70/garments/json', 1)
        self.status = self.create_publisher(String, '/z70/garments/status', LATCHED)
        self.grid = None; self.grid_received = None
        self.create_subscription(OccupancyGrid, '/z70/map/safe_grid', self.on_grid, LATCHED)
        self.tf = Buffer(cache_time=Duration(seconds=30.)); self.tf_listener = TransformListener(self.tf, self)
        self.tracker = GarmentTracker(); self.last_pose = None; self.last_session = None
        self.lock = threading.Lock(); self.stop_event = threading.Event()
        self.latest = None; self.result = None; self.process = None; self.worker = None
        self.frames = MatchedFrames(self, 'front', self.on_frame, compressed=True)
        self.set_status(dict(model_available=False, backend=backend, state='STARTING'))
        if backend not in BACKENDS:
            self.set_status(dict(model_available=False, error='UNKNOWN_BACKEND')); return
        env = worker_env(HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1', HF_HUB_DISABLE_TELEMETRY='1')
        try:
            self.process = subprocess.Popen([python, '-u', str(Path(__file__).resolve()),
                                             '--backend', backend, '--device', device], stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, env=env, bufsize=0)
            self.worker = threading.Thread(target=self.run_worker, args=(1 / max(.01, hz),), daemon=True)
            self.worker.start()
        except OSError as error:
            self.set_status(dict(model_available=False, error=type(error).__name__))
        self.create_timer(.1, self.publish)

    def set_status(self, quality):
        self.status.publish(String(data=json.dumps(quality, allow_nan=False)))

    def on_grid(self, msg):
        self.grid = msg; self.grid_received = time.monotonic()

    def on_frame(self, msg):
        meta = self.frames.metadata
        if (msg.header.frame_id != 'camera_optical_frame' or meta is None
                or (meta.width, meta.height) != (self.cam.width, self.cam.height)
                or not 0 < len(msg.data) <= MAX_PACKET):
            return
        with self.lock:
            self.latest = (msg, deepcopy(meta))

    def read_exact(self, count):
        return read_pipe_exact(self.process, count, self.stop_event)

    def run_worker(self, period):
        proc = self.process
        while not self.stop_event.is_set():
            with self.lock:
                frame, self.latest = self.latest, None
            if frame is None:
                self.stop_event.wait(.05); continue
            msg, meta = frame
            if not 0 <= time.time() - seconds(meta.evidence.received_at) <= self.max_age:
                continue
            try:
                data = bytes(msg.data)
                payload = memoryview(struct.pack('<I', len(data)) + data)
                while payload and not self.stop_event.is_set():
                    written = proc.stdin.write(payload)
                    if not written:
                        raise EOFError('Garment stdin closed')
                    payload = payload[written:]
                size = struct.unpack('<I', self.read_exact(4))[0]
                if not 0 < size <= MAX_PACKET:
                    raise ValueError('Oversized worker response')
                detections, quality = decode_result(self.read_exact(size))
                with self.lock:
                    self.result = (meta, detections, quality)
            except (OSError, EOFError, TimeoutError, ValueError, KeyError) as error:
                with self.lock:
                    self.result = (meta, [], dict(model_available=False, error=type(error).__name__))
                break
            self.stop_event.wait(period)

    def pose_at(self, stamp):
        try:
            tf = self.tf.lookup_transform('map', 'base_footprint', Time.from_msg(stamp), timeout=Duration(seconds=0.))
            t = tf.transform.translation
            age = abs(seconds(tf.header.stamp) - seconds(stamp))
            return (t.x, t.y, quat_yaw(tf.transform.rotation)), age
        except TransformException:
            return None, float('inf')

    def publish(self):
        with self.lock:
            result, self.result = self.result, None
        if result is None:
            if self.process is not None and self.process.poll() is not None:
                self.set_status(dict(model_available=False, error='WORKER_EXITED'))
            return
        meta, detections, quality = result
        age = time.time() - seconds(meta.evidence.received_at)
        if not 0 <= age <= self.max_age:
            quality = dict(quality, model_available=False, error='STALE_INFERENCE', frame_age_s=age)
            detections = []
            meta = deepcopy(meta); meta.evidence.stale = True; meta.evidence.valid = False
            meta.evidence.reason = 'STALE_INFERENCE'
        pose, tf_age = self.pose_at(meta.header.stamp)
        grid = self.grid
        grid_age = (max(time.monotonic() - self.grid_received, time.time() - seconds(grid.header.stamp))
                    if grid is not None else float('inf'))
        session = bytes(meta.evidence.session_id.uuid)
        if session != self.last_session or pose is None or self.last_pose is None:
            self.tracker = GarmentTracker(); delta = (0., 0., 0.)
        else:
            old_x, old_y, old_yaw = self.last_pose
            dx, dy = pose[0] - old_x, pose[1] - old_y
            delta = (math.cos(old_yaw) * dx + math.sin(old_yaw) * dy,
                     -math.sin(old_yaw) * dx + math.cos(old_yaw) * dy,
                     math.atan2(math.sin(pose[2] - old_yaw), math.cos(pose[2] - old_yaw)))
        self.last_pose, self.last_session = pose, session
        items = []
        for detection in detections:
            try:
                item = observe(detection['mask'], self.cam, category=detection['category'],
                               confidence=detection['confidence'], camera_x_m=self.camera_x,
                               calibration_id=meta.calibration_id,
                               qualified_policy=getattr(self, 'qualified_policy', None))
            except (ValueError, KeyError):
                continue
            if item['contact_valid']:
                item['inside_geofence'], item['geofence_verified'] = geofence(
                    item['floor_contact_m'], pose, grid, tf_age_s=tf_age, grid_age_s=grid_age)
            if not quality.get('model_available', False):
                item['grasp_valid'] = False; item['reachable_for_grasp'] = False
            items.append(item)
        items = self.tracker.update(items, delta)
        for item in items:
            item['reachable_for_grasp'] = bool(item['reachable_for_grasp'] and item['confirmed']
                                               and item['inside_geofence'] and item['geofence_verified'])
        _, active = geofence((0., 0., 0.), pose, grid, tf_age_s=tf_age, grid_age_s=grid_age)
        msg = observations_message(items, meta, quality, active)
        self.pub.publish(msg)
        self.json_pub.publish(String(data=json.dumps(dict(model=quality, geofence_active=active,
                                                          frame_age_s=age, garments=items), allow_nan=False)))
        markers = MarkerArray()
        clear = Marker(); clear.header = deepcopy(msg.header); clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, item in enumerate(items):
            if not item['contact_valid']:
                continue
            marker = Marker(); marker.header = deepcopy(msg.header); marker.ns = 'garments'; marker.id = i
            marker.type = Marker.SPHERE; marker.action = Marker.ADD
            marker.pose.position = Point(**dict(zip(('x', 'y', 'z'), item['floor_contact_m'])))
            marker.pose.orientation.w = 1.; marker.scale.x = marker.scale.y = marker.scale.z = .025
            marker.color.a = 1.; marker.color.g = .8 if item['inside_geofence'] else 0.; marker.color.r = .8
            marker.lifetime = Duration(seconds=5.).to_msg(); markers.markers.append(marker)
        self.markers.publish(markers)
        self.set_status(dict(quality, geofence_active=active, frame_age_s=age, tracks=len(self.tracker.tracks),
                             confirmed=sum(x['confirmed'] for x in items)))

    def close(self):
        stop_worker_process(self.process, self.worker, self.stop_event)


def main():
    rclpy.init(); node = GarmentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.close(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    worker_main()
