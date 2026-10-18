from analyze import *
# Offline detector recovery only, preserve primary observations. Same metric board.
for seq in [2,18,22,25,30,33,34,37,42,45]:
 file=next((HERE/'raw').glob(f'{seq:03}-*.jpg'));im=cv2.imread(str(file));gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);roi=gray[680:930,300:1400];cand=[]
 lut=dict(zip(ob.BOARD.getIds().flatten(),ob.BOARD.getObjPoints()))
 for scale in [2,3]:
  for enhance in [False,True]:
   gg=cv2.createCLAHE(3.,(8,8)).apply(roi) if enhance else roi
   gg=cv2.resize(gg,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
   for win in [53,113,173]:
    p=cv2.aruco.DetectorParameters();p.adaptiveThreshWinSizeMax=win;p.cornerRefinementMethod=1
    c,ids,_=cv2.aruco.ArucoDetector(ob.DICT,p).detectMarkers(gg)
    if ids is None:continue
    pairs=[(lut[int(i)],cc.reshape(4,2)/scale+[300,680],int(i)) for cc,i in zip(c,ids.flatten()) if int(i) in lut]
    if len(pairs)<3:continue
    X=np.concatenate([x for x,px,i in pairs]).astype(float);px=np.concatenate([px for x,px,i in pairs]).astype(float);K,d=ob.intrinsics('front');ok,r,t=cv2.solvePnP(X,px,K,d)
    s=least_squares(lambda v:(j.project(X,j.mat(v),K,d)-px).ravel(),j.vec(ob.transform(r,t)),loss='soft_l1',f_scale=.5,max_nfev=200)
    T=j.mat(s.x);errs=np.linalg.norm(j.project(X,T,K,d)-px,axis=1)
    if not s.success or np.any((X@T[:3,:3].T+T[:3,3])[:,2]<=0):continue
    cand.append(dict(scale=scale,clahe=enhance,threshold_max=win,ids=[i for x,p,i in pairs],obj=X.tolist(),px=px.tolist(),full_rms_px=float(np.sqrt(np.mean(errs**2))),accepted_corners=int(sum(errs<4)),T_cam_board=T.tolist(),kind='board_marker_corners',quality_accepted=bool(np.mean(errs<4)>=.65)))
 old=read(HERE/(file.stem+'-obs.json'));best=max(cand,key=lambda c:c['accepted_corners']) if cand else None
 out={'file':str(file),'camera':'front','board':best,'raw_markers':old['raw_markers'],'marker':old['marker'],'recovery':'post-acquisition crop/upscale detector sensitivity; no optical/model fitting; primary results preserved','candidates':cand}
 if out['marker'] is None and best:
  elevated=[m for m in old['raw_markers'] if m['id']==68 and np.mean(np.array(m['px'])[:,1])<650]
  if len(elevated)==1:out['marker']={'px':elevated[0]['px']}
 save(file.stem+'-recovery.json',out)
 print(file.stem,'candidates',len(cand),'best',None if best is None else [best['accepted_corners'],best['full_rms_px'],best['ids']])
