"""HSV color segmentation, floor projection, and ArUco ID68 wrist pose estimation."""
from functools import lru_cache
import cv2
import numpy as np
import yaml
from z70_description.floor_geometry import FrontCamera
from .resources import front_calibration_path, description_config, colors_path

@lru_cache(maxsize=1)
def calibration():
    return description_config('mount_calibration.yaml')

@lru_cache(maxsize=1)
def front_camera():
    return FrontCamera.load(front_calibration_path())

@lru_cache(maxsize=1)
def gripper_camera_info():
    """Load gripper camera intrinsics and distortion parameters."""
    return description_config('z70_gripper_camera_info.yaml')

@lru_cache(maxsize=1)
def colors():
    return yaml.safe_load(colors_path().read_text())

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
                    ys,xs=np.nonzero(sub)
                    b['bbox']=[x+x0,y+y0,cw,ch]; b['trimmed']='sat_core'
                    b['pixel']=[x+x0+float(xs.mean()),y+y0+float(ys.mean())]
                    bottom=ys.max(); b['contact_px']=[x+x0+float(np.median(xs[ys>=bottom-3])),float(y+y0+bottom)]
                    b['size_px']=float(max(cw,ch)); b['area_px']=float(len(xs))
                    mask=sub; x,y,w,h=b['bbox']
                    u,v=b['contact_px']; point=cam.pixel_to_floor(u*sx,v*sy)
                    if point is None or not 0<point['forward_mm']<3000: continue
        if w>1.4*h:
            keepw=int(round(1.1*h)); satw=hsv[y:y+h,x:x+w,1].astype(float)
            def scorew(c0): m=mask[:,c0:c0+keepw]>0; return float(satw[:,c0:c0+keepw][m].mean()) if m.any() else -1
            starts=range(0,w-keepw+1,max(1,keepw//8)); c0=max(starts,key=scorew)
            sub=mask[:,c0:c0+keepw]; ys,xs=np.nonzero(sub)
            if len(xs)==0: continue
            b['bbox']=[x+c0,y,keepw,h]; b['trimmed_from_w']=w; b['trimmed']='sides'
            b['pixel']=[x+c0+float(xs.mean()),y+float(ys.mean())]
            bottom=ys.max(); b['contact_px']=[x+c0+float(np.median(xs[ys>=bottom-3])),float(y+bottom)]; b['size_px']=float(max(keepw,h))
            mask=np.zeros_like(mask); mask[:,c0:c0+keepw]=sub
            u,v=b['contact_px']; point=cam.pixel_to_floor(u*sx,v*sy)
            if point is None: continue
            x=x+c0; w=keepw
        if h>1.4*w:
            keep=int(round(1.15*w)); sat=hsv[y:y+h,x:x+w,1].astype(float)
            def score(r0): m=mask[r0:r0+keep]>0; return float(sat[r0:r0+keep][m].mean()) if m.any() else -1
            r0=0 if score(0)>=score(h-keep) else h-keep
            sub=mask[r0:r0+keep]; ys,xs=np.nonzero(sub)
            if len(xs)==0: continue
            b['bbox']=[x,y+r0,w,keep]; b['trimmed_from_h']=h; b['trimmed']='top' if r0==0 else 'bottom'
            b['pixel']=[x+float(xs.mean()),y+r0+float(ys.mean())]
            bottom=ys.max(); b['contact_px']=[x+float(np.median(xs[ys>=bottom-3])),float(y+r0+bottom)]; b['size_px']=float(max(w,keep))
            u,v=b['contact_px']; point=cam.pixel_to_floor(u*sx,v*sy)
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
        from z70_description import pincher_geometry as hand
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

def gripper_objects(image,K,D,translation,rotation,thresholds=None):
    result=[]
    for b in detect(image,thresholds):
        p=ray_floor(b['contact_px'],K,D,translation,rotation)
        if p is None or np.linalg.norm(np.array(p)-translation)>3: continue
        b.update(position=p,camera='gripper',size_m=.05); result.append(b)
    return result
