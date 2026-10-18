"""Offline pincher intrinsics. No arm FK, jog angles, or robot IO."""
import json,hashlib,sys
from pathlib import Path
import cv2
import numpy as np
DICT=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
BOARD=cv2.aruco.CharucoBoard((10,14),.020,.015,DICT);BOARD.setLegacyPattern(True)
OBJ=BOARD.getChessboardCorners()

def detect_board(path):
    image=cv2.imread(str(path))
    if image is None:raise ValueError('Unreadable image')
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    cp=cv2.aruco.CharucoParameters();cp.minMarkers=1;cp.tryRefineMarkers=True
    candidates=[]
    for refine,scale,enhance in [(1,1,False),(0,1,False),(1,2,True),(0,2,True)]:
        p=cv2.aruco.DetectorParameters();p.adaptiveThreshWinSizeMax=53;p.cornerRefinementMethod=refine
        g=cv2.createCLAHE(3.,(8,8)).apply(gray) if enhance else gray
        if scale!=1:g=cv2.resize(g,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
        cc,ci,_,_=cv2.aruco.CharucoDetector(BOARD,cp,p).detectBoard(g)
        if ci is None or len(ci)<8:continue
        ids=ci.flatten();px=cc.reshape(-1,2).astype(float)/scale;obj=OBJ[ids].astype(float)
        H,mask=cv2.findHomography(obj[:,:2],px,cv2.RANSAC,5)
        if H is None:continue
        pred=cv2.perspectiveTransform(obj[:,:2].reshape(-1,1,2),H).reshape(-1,2)
        err=np.linalg.norm(pred-px,axis=1)
        candidates.append(dict(ids=ids.tolist(),px=px.tolist(),obj=obj.tolist(),homography_rms_px=float(np.sqrt(np.mean(err**2))),homography_inlier_fraction=float(mask.mean()),detector={'refine':refine,'scale':scale,'clahe':enhance}))
    best=max(candidates,key=lambda o:(o['homography_inlier_fraction']>=.8,len(o['px']))) if candidates else {'ids':[],'px':[],'obj':[],'reject':'no_board'}
    return dict(file=str(path),size=[image.shape[1],image.shape[0]],sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),**best)

def fit_model(observations,model='k1k2'):
    if len(observations)<6:raise ValueError('At least six distinct views required')
    flags=cv2.CALIB_ZERO_TANGENT_DIST|cv2.CALIB_FIX_K3
    if model=='none':flags|=cv2.CALIB_FIX_K1|cv2.CALIB_FIX_K2
    elif model=='k1':flags|=cv2.CALIB_FIX_K2
    elif model!='k1k2':raise ValueError('Unknown radial model')
    X=[np.asarray(o['obj'],np.float32) for o in observations];uv=[np.asarray(o['px'],np.float32) for o in observations]
    rms,K,d,rs,ts,ki,ei,per=cv2.calibrateCameraExtended(X,uv,tuple(observations[0]['size']),None,None,flags=flags,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,150,1e-10))
    valid=bool(np.isfinite(K).all() and rms<100 and min(K[0,0],K[1,1])>0 and all(t[2]>0 for t in ts))
    return dict(model=model,K=K.tolist(),dist=d.ravel().tolist(),rms_px=float(rms),success=valid,intrinsic_std_conditional_opencv=ki.ravel().tolist(),per_view_rms_px=per.ravel().tolist(),flags=int(flags),camera_board_poses=[{'r':r.ravel().tolist(),'t':t.ravel().tolist()} for r,t in zip(rs,ts)])

def evaluate(observations,fit):
    K=np.array(fit['K']);d=np.array(fit['dist']);allerr=[];views=[]
    for o in observations:
        X=np.asarray(o['obj'],float);uv=np.asarray(o['px'],float)
        ok,r,t=cv2.solvePnP(X,uv,K,d,flags=cv2.SOLVEPNP_ITERATIVE)
        err=np.linalg.norm(cv2.projectPoints(X,r,t,K,d)[0].reshape(-1,2)-uv,axis=1)
        views.append({'file':o.get('file'),'rms_px':float(np.sqrt(np.mean(err**2))),'positive_depth':bool(ok and t[2]>0)})
        allerr.extend(err)
    return {'rms_px':float(np.sqrt(np.mean(np.square(allerr)))),'p95_px':float(np.percentile(allerr,95)),'max_px':float(np.max(allerr)),'views':views,'conditional':'held-out camera-board pose estimated from its corners; lens fixed'}

def conditioning(observations,fit):
    K=np.array(fit['K']);d=np.array(fit['dist']);cols=[6,7,8,9]+([10] if fit['model'] in ['k1','k1k2'] else [])+([11] if fit['model']=='k1k2' else [])
    projected=[];normals=[];depths=[]
    for o in observations:
        X=np.asarray(o['obj'],float);uv=np.asarray(o['px'],float);ok,r,t=cv2.solvePnP(X,uv,K,d)
        _,J=cv2.projectPoints(X,r,t,K,d);A=J[:,:6];B=J[:,cols].copy();B[:,:4]*=1000
        projected.append((B-np.einsum('ij,jk->ik',A,np.linalg.lstsq(A,B,rcond=1e-10)[0]))/np.sqrt(len(X)))
        normals.append(cv2.Rodrigues(r)[0][:,2]);depths.append(float(t.ravel()[2]))
    _,s,V=np.linalg.svd(np.vstack(projected),full_matrices=False);n=np.array(normals);angles=np.degrees(np.arccos(np.clip(n@n.T,-1,1)))
    return {'singular_values':s.tolist(),'rank':int(np.sum(s>s[0]*1e-6)),'dimension':len(cols),'condition_number':float(s[0]/s[-1]),'weakest_normalized_parameter_direction':V[-1].tolist(),'scaling':'fx,fy,cx,cy divided by1000px; radial coefficients unscaled; extrinsic nuisance projected out per pose; equal pose weight','normal_span_deg':float(angles.max()),'normal_cam':n.tolist(),'board_origin_depth_range_m':[min(depths),max(depths)]}

def metric_check(o,fit):
    K=np.array(fit['K']);d=np.array(fit['dist']);X=np.asarray(o['obj'],float);uv=np.asarray(o['px'],float)
    ix=np.arange(len(X));train=ix%2==0;test=~train
    ok,r,t=cv2.solvePnP(X[train],uv[train],K,d);R=cv2.Rodrigues(r)[0];C=-R.T@t.ravel()
    rays=np.c_[cv2.undistortPoints(uv[test].reshape(-1,1,2),K,d).reshape(-1,2),np.ones(test.sum())]@R
    P=C+rays*(-C[2]/rays[:,2])[:,None];truth=X[test]
    A=np.linalg.norm(P[:,None]-P[None,:],axis=2);B=np.linalg.norm(truth[:,None]-truth[None,:],axis=2);keep=np.triu(np.ones(B.shape,dtype=bool),1)&(B>.035)
    rel=(A[keep]-B[keep])/B[keep]
    return {'relative_pairwise_distance_rms':float(np.sqrt(np.mean(rel**2))),'median_relative_bias':float(np.median(rel)),'max_relative_error':float(np.max(np.abs(rel))),'test_corners':int(test.sum()),'pose_from_other_corners':True,'scope':'held-out image and held-out corners, conditional on shared planar print; not independent ruler/scale validation'}

if __name__=='__main__':
    p=Path(sys.argv[1]);o=detect_board(p);dest=Path(sys.argv[2]);dest.write_text(json.dumps(o));print(json.dumps({k:v for k,v in o.items() if k not in ['ids','px','obj']}));print('corners',len(o['px']))
