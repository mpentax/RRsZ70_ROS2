#!/usr/bin/env python3
"""Offline settling and motion quality checks; no network."""
import sys,json,math
from pathlib import Path
import cv2
ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]
sys.path.insert(0,str(REPO/'ros2/src/z70_odometry'))
sys.path.insert(0,str(REPO/'calibration/chassis'))
from z70_odometry.floor_vo import FloorCamera,FloorOdometry
from analyze import analyze
cam=FloorCamera.load(REPO/'calibration/front-camera/front_camera.json')
mode=sys.argv[1]
if mode=='settle':
    images=[cv2.imread(str(ROOT/p)) for p in sys.argv[2:4]]
    if any(im is None or im.shape[:2]!=(1200,1600) for im in images): raise SystemExit('STOP: invalid image')
    m=FloorOdometry(cam,camera_x_m=0,n_features=6000).estimate(*images)
    result={'before':sys.argv[2],'after':sys.argv[3],'accepted':m is not None}
    if m:
        result.update(dx_m=m.dx,dy_m=m.dy,yaw_deg=m.dtheta_deg,inliers=m.inliers,rms_m=m.rms_m)
    result['settled']=m is not None and math.hypot(m.dx,m.dy)<.002 and abs(m.dtheta_deg)<.3
    with (ROOT/'settling.jsonl').open('a') as f:f.write(json.dumps(result)+'\n')
    print(json.dumps(result))
    if not result['settled']:raise SystemExit(2)
else:
    ident,before,after,v,w,d=sys.argv[2:]
    p=ROOT/'pairs.json'
    rows=json.loads(p.read_text()) if p.exists() else []
    rows.append(dict(id=ident,before=before,after=after,velocity=float(v),omega=float(w),duration_ms=int(d)))
    p.write_text(json.dumps(rows,indent=2)+'\n')
    try:result=analyze(p,REPO/'calibration/front-camera/front_camera.json')
    except ValueError:
        result=analyze(p,REPO/'calibration/front-camera/front_camera.json',camera_x_m=0)
        result['unidentifiable_fit']=True
    (ROOT/'fit.json').write_text(json.dumps(result,indent=2)+'\n')
    last=next((r for r in result['pulses'] if r['id']==ident),None)
    print(json.dumps({'pair':last,'rejected':result['rejected'],'fit':result['fit']}))
    if last is None or abs(last['dtheta_deg'])>15 or math.hypot(last['dx_m'],last['dy_m'])>.06: raise SystemExit(2)
