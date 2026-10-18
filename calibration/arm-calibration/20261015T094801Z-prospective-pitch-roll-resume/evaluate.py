from analyze import *
SENSITIVITY='--recovery' in sys.argv
PREFIX='detector-recovery-' if SENSITIVITY else ''
# Evaluation only: no marker pixels or new lens/registration fit.
def get(stem,camera):
 p=HERE/(stem+('-recovery.json' if SENSITIVITY and camera=='front' else '-obs.json'))
 return read(p) if p.exists() else detect(stem+'.jpg',camera)
def boardpose(o,K,d):
 b=o['board'];X=np.array(b['obj']);px=np.array(b['px']);ok,r,t=cv2.solvePnP(X,px,K,d)
 s=least_squares(lambda v:(j.project(X,j.mat(v),K,d)-px).ravel(),j.vec(ob.transform(r,t)),loss='soft_l1',f_scale=.5,max_nfev=300)
 T=j.mat(s.x)
 if not s.success or not np.all((X@T[:3,:3].T+T[:3,3])[:,2]>0):raise ValueError('invalid board fit')
 return T
def delta(a,b):return {'translation_m':float(np.linalg.norm(a[:3,3]-b[:3,3])),'orientation_deg':float(np.degrees(Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).magnitude()))}
# Deployment performs chassis obstruction check. Use ONLY postdeployment
# premeasurement floor pixels, never the prior T_F_W or ID68 pixels.
regframes=[get('002-deployed-front','front'),get('018-route-front','front')]
reg={}
for name,m in models.items():
 K=np.array(m['K']);d=np.array(m['dist']);initial=boardpose(regframes[0],K,d)
 def residual(v):return np.concatenate([(j.project(o['board']['obj'],j.mat(v),K,d)-o['board']['px']).ravel()/np.sqrt(len(o['board']['px'])) for o in regframes])
 sol=least_squares(residual,j.vec(initial),loss='soft_l1',f_scale=.5,max_nfev=300)
 TF=j.mat(sol.x)
 if not sol.success or not all(np.all((np.array(o['board']['obj'])@TF[:3,:3].T+TF[:3,3])[:,2]>0) for o in regframes):raise ValueError('invalid floor registration')
 reg[name]={'T_F_W':TF.tolist(),'success':bool(sol.success),'training_files':[o['file'] for o in regframes],'scope':'floor-only postdeployment premeasurement training, no marker input','floor_metrics':j.stats([j.project(o['board']['obj'],TF,K,d)-o['board']['px'] for o in regframes])}
save(PREFIX+'evaluation-floor-registration.json',reg)
spec=[('comparator-out',22,23,24,25),('pitch-roll-repeat1',30,31,32,33),('pitch-roll-repeat2',34,35,36,37),('comparator-return',42,43,44,45)]
allresults=[];vectors={name:[] for name in models};by={name:{} for name in models}
for label,*seq in spec:
 files=[next((HERE/'raw').glob(f'{i:03}-*.jpg')).stem for i in seq]
 obs=[get(f,'front' if z in [0,3] else 'gripper') for z,f in enumerate(files)]
 out={'label':label,'files':files,'nominal_route_label_only':[250,95,40 if 'pitch-roll' in label else 0],'requested_target_exact':False,'missing':[files[i] for i,o in enumerate(obs) if o['board'] is None or (i in [0,3] and o['marker'] is None)],'models':{}}
 if out['missing']:allresults.append(out);continue
 p={'id':seq[0],'nominal':out['nominal_route_label_only'],'front':[obs[0],obs[3]],'gripper':[o['board'] for o in obs[1:3]]}
 WG=j.gripper_pose({'Kg':Kg,'dg':dg},p)
 gd=delta(boardpose(obs[1],Kg,dg),boardpose(obs[2],Kg,dg));out['G_duplicate_stationarity']=gd
 for name,m in models.items():
  K=np.array(m['K']);d=np.array(m['dist']);TF=np.array(reg[name]['T_F_W']);GM=np.array(m['T_G_M']);pred=TF@WG@GM
  fronts=[];err=[];markerT=[]
  for o in p['front']:
   px=np.array(o['marker']['px']);e=j.project(j.MARKER,pred,K,d)-px;err.append(e)
   _,rs,ts,_=cv2.solvePnPGeneric(j.MARKER,px,K,d,flags=cv2.SOLVEPNP_IPPE_SQUARE)
   branches=[]
   for r,t in zip(rs,ts):
    T=ob.transform(r,t);branches.append({'T_F_M':T.tolist(),'positive_depth':bool(np.all((j.MARKER@T[:3,:3].T+T[:3,3])[:,2]>0)),**delta(pred,T),'reprojection':j.stats([j.project(j.MARKER,T,K,d)-px])})
   branches.sort(key=lambda x:x['reprojection']['rms_px']);markerT.append(np.array(branches[0]['T_F_M']))
   fronts.append({'file':o['file'],'observed_px':px.tolist(),'errors_xy_px':e.tolist(),'metrics':j.stats([e]),'pnp_branches_diagnostic_only':branches})
  fd=delta(boardpose(obs[0],K,d),boardpose(obs[3],K,d));md=delta(*markerT)
  accepted=gd['translation_m']<=.003 and gd['orientation_deg']<=3 and fd['translation_m']<=.003 and fd['orientation_deg']<=3 and md['translation_m']<=.003 and md['orientation_deg']<=12.779567131728307
  depths=(j.MARKER@pred[:3,:3].T+pred[:3,3])[:,2]
  archived=np.array(m['T_F_W'])@np.array(read(HERE/'frozen-models/achieved-pose-acquisition-design.json')['best_two'][0]['T_W_G'])
  out['models'][name]={'metrics':j.stats(err),'accepted_stationarity':accepted,'marker_stationarity_diagnostic':md,'front_floor_stationarity':fd,'T_W_G_Gonly':WG.tolist(),'T_F_G':(TF@WG).tolist(),'T_F_M_prediction':pred.tolist(),'marker_depth_m':[float(depths.min()),float(depths.max())],'positive_depth':bool(np.all(depths>0)),'observed_marker_pixel_bounds':np.array([o['marker']['px'] for o in p['front']]).reshape(-1,2).tolist(),'archived_target_FG_departure':delta(TF@WG,archived),'front':fronts}
  if accepted and np.all(depths>0):
   vectors[name].extend(err);by[name].setdefault('pitch-roll' if 'pitch-roll' in label else 'comparator',[]).extend(err)
 allresults.append(out)
save(PREFIX+'prospective-evaluation.json',{'scope':'G-only board pose; fixed K/D/GM; floor-only NEW-block registration; no marker fitting; actual achieved neighboring pitch/roll, NOT exact archived target','brackets':allresults,'accepted_aggregate':{n:j.stats(e) if e else None for n,e in vectors.items()},'accepted_by_pose':{n:{k:j.stats(e) for k,e in groups.items()} for n,groups in by.items()},'uncertainty':'PnP diagnostic only; print scale/flatness, floor glare, lens extrapolation, mount continuity, sequential exposure systematics remain unknown'})
print('SUMMARY',json.dumps({'aggregate':{n:j.stats(e) if e else None for n,e in vectors.items()},'by_pose':{n:{k:j.stats(e) for k,e in groups.items()} for n,groups in by.items()},'accepted':[(p['label'],{n:m['accepted_stationarity'] for n,m in p['models'].items()}) for p in allresults]}))
