"""Deterministic offline board/ID68 observations; never contacts robot."""
import json,sys
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
DICT=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
BOARD=cv2.aruco.CharucoBoard((10,14),.020,.015,DICT);BOARD.setLegacyPattern(True)
OBJ=BOARD.getChessboardCorners().astype(float)

def intrinsics(camera,perturb=None):
    if camera=='front':
        f=json.loads((ROOT/'calibration/front-camera/front_camera.json').read_text())
        K=np.array([[f['f_px'],0,f['cx']],[0,f['f_px'],f['cy']],[0,0,1.]])
        d=np.array([f['k1'],f['k2'],0,0,0.])
    else:
        f=json.loads((ROOT/'calibration/arm-calibration/20261014T090217Z/arm_calibration.json').read_text())['gripper_intrinsics']
        K=np.array(f['K'],dtype=float);d=np.array(f['dist'],dtype=float)
    if perturb:
        K[0,0]*=perturb.get('fscale',1);K[1,1]*=perturb.get('fscale',1)
        K[0,2]+=perturb.get('cx',0);K[1,2]+=perturb.get('cy',0)
        d[0]+=perturb.get('k1',0)
    return K,d

def transform(r,t):
    T=np.eye(4);T[:3,:3]=cv2.Rodrigues(r)[0];T[:3,3]=np.asarray(t).ravel();return T

def detect(path,camera,floor_min_y=710,perturb=None):
    im=cv2.imread(str(path))
    if im is None:raise ValueError('Unreadable retained image')
    gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);K,d=intrinsics(camera,perturb)
    g=gray.copy()
    if camera=='front':g[:floor_min_y]=255
    candidates=[]
    cp=cv2.aruco.CharucoParameters();cp.minMarkers=1;cp.tryRefineMarkers=True
    for refine,scale,enhance in [(1,1,False),(0,1,False),(1,2,True),(0,2,True)]:
        p=cv2.aruco.DetectorParameters();p.adaptiveThreshWinSizeMax=53;p.cornerRefinementMethod=refine
        gg=cv2.createCLAHE(3.,(8,8)).apply(g) if enhance else g
        gg=cv2.resize(gg,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC) if scale!=1 else gg
        cc,ci,_,_=cv2.aruco.CharucoDetector(BOARD,cp,p).detectBoard(gg)
        if ci is None or len(ci)<8:continue
        px=cc.reshape(-1,2).astype(float)/scale;ids=ci.flatten();obj=OBJ[ids]
        ok,r,t=cv2.solvePnP(obj,px,K,d,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:continue
        full=cv2.projectPoints(obj,r,t,K,d)[0].reshape(-1,2)
        keep=np.linalg.norm(full-px,axis=1)<4
        if keep.sum()<8:continue # retain low-coverage observations as rejected quality, not absence
        r,t=cv2.solvePnPRefineLM(obj[keep],px[keep],K,d,r,t)
        full=cv2.projectPoints(obj,r,t,K,d)[0].reshape(-1,2)
        err=np.linalg.norm(full-px,axis=1);T=transform(r,t)
        if np.min((obj@T[:3,:3].T+T[:3,3])[:,2])<=0:continue
        candidates.append(dict(quality_accepted=bool(keep.mean()>=.65),inlier_fraction=float(keep.mean()),T_cam_board=T.tolist(),ids=ids.tolist(),px=px.tolist(),accepted_ids=ids[keep].tolist(),accepted_px=px[keep].tolist(),accepted_corners=int(keep.sum()),rms_px=float(np.sqrt(np.mean(err[keep]**2))),full_rms_px=float(np.sqrt(np.mean(err**2))),normal_cam=T[:3,2].tolist()))
    # Sparse clipped sheets still provide metric marker corners. Never pretend
    # their IDs are ChArUco corner IDs; keep explicit object-point provenance.
    p=cv2.aruco.DetectorParameters();p.adaptiveThreshWinSizeMax=53;p.cornerRefinementMethod=1
    fc,fi,_=cv2.aruco.ArucoDetector(DICT,p).detectMarkers(g)
    lut=dict(zip(BOARD.getIds().flatten(),BOARD.getObjPoints()))
    if fi is not None:
        pairs=[(lut[int(i)],c.reshape(4,2)) for c,i in zip(fc,fi.flatten()) if int(i) in lut]
        if len(pairs)>=3:
            obj=np.concatenate([x for x,p in pairs]).astype(float);px=np.concatenate([p for x,p in pairs]).astype(float)
            ok,r,t=cv2.solvePnP(obj,px,K,d,flags=cv2.SOLVEPNP_ITERATIVE)
            if ok and t.ravel()[2]>0:
                err=np.linalg.norm(cv2.projectPoints(obj,r,t,K,d)[0].reshape(-1,2)-px,axis=1)
                keep=err<4
                if keep.sum()>=8:
                    r,t=cv2.solvePnPRefineLM(obj[keep],px[keep],K,d,r,t)
                    err=np.linalg.norm(cv2.projectPoints(obj,r,t,K,d)[0].reshape(-1,2)-px,axis=1);T=transform(r,t)
                    candidates.append(dict(kind='board_marker_corners',quality_accepted=bool(keep.mean()>=.65),inlier_fraction=float(keep.mean()),T_cam_board=T.tolist(),ids=fi.flatten().tolist(),px=px.tolist(),obj=obj.tolist(),accepted_obj=obj[keep].tolist(),accepted_ids=[],accepted_px=px[keep].tolist(),accepted_corners=int(keep.sum()),rms_px=float(np.sqrt(np.mean(err[keep]**2))),full_rms_px=float(np.sqrt(np.mean(err**2))),normal_cam=T[:3,2].tolist()))
    for x in candidates:
        if 'kind' not in x:
            x['kind']='charuco';x['obj']=OBJ[x['ids']].tolist();x['accepted_obj']=OBJ[x['accepted_ids']].tolist()
    board=max(candidates,key=lambda x:(x['quality_accepted'],x['accepted_corners'])) if candidates else None
    if board:
        X=np.array(board['obj']);px=np.array(board['px']);branches=[]
        ok,rs,ts,_=cv2.solvePnPGeneric(X,px,K,d,flags=cv2.SOLVEPNP_IPPE)
        for r,t in zip(rs,ts):
            T=transform(r,t);depth=(X@T[:3,:3].T+T[:3,3])[:,2]
            err=np.linalg.norm(cv2.projectPoints(X,r,t,K,d)[0].reshape(-1,2)-px,axis=1)
            branches.append({'T_cam_board':T.tolist(),'positive_depth':bool(np.all(depth>0)),'full_rms_px':float(np.sqrt(np.mean(err**2))),'normal_cam':T[:3,2].tolist()})
        board['planar_pnp_candidates']=branches
        board['selection_policy']='iterative PnP, positive depth, full-corner residual retained; IPPE alternatives are not independently surveyed orientation truth'
    p=cv2.aruco.DetectorParameters();p.adaptiveThreshWinSizeMax=53;p.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    cs,mi,_=cv2.aruco.ArucoDetector(DICT,p).detectMarkers(gray)
    raw=[];elevated=[];h=.0075
    mo=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0.]])
    if mi is not None:
        for corners,ident in zip(cs,mi.flatten()):
            px=corners.reshape(4,2).astype(float);raw.append(dict(id=int(ident),px=px.tolist()))
            if ident!=68 or camera!='front' or board is None:continue
            ok,rs,ts,_=cv2.solvePnPGeneric(mo,px,K,d,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            branches=[]
            for r,t in zip(rs,ts):
                if t.ravel()[2]<=0:continue
                T=transform(r,t);B=np.linalg.inv(np.array(board['T_cam_board']))@T
                err=cv2.projectPoints(mo,r,t,K,d)[0].reshape(4,2)-px
                branches.append(dict(T_cam_marker=T.tolist(),rms_px=float(np.sqrt(np.mean(np.sum(err**2,axis=1)))),absolute_height_over_board_m=float(abs(B[2,3]))))
            branches.sort(key=lambda x:x['rms_px'])
            if branches and branches[0]['absolute_height_over_board_m']>.03:
                elevated.append(dict(id=68,px=px.tolist(),branches=branches,**branches[0]))
    marker=elevated[0] if len(elevated)==1 else None
    return dict(file=str(path),camera=camera,board=board,marker=marker,raw_markers=raw,marker_ambiguous=len(elevated)>1,floor_mask_min_y=floor_min_y if camera=='front' else None)

if __name__=='__main__':
    out=detect(sys.argv[1],sys.argv[2]);print(json.dumps(out))
