"""Offline rotation-first analysis; writes only beside this new script."""
import csv,json,hashlib,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
P=Path(__file__).resolve().parent

def ang(R):return float(np.degrees(Rot.from_matrix(R).magnitude()))
def mean(Rs):return Rot.from_matrix(np.asarray(Rs)).mean().as_matrix()
def rel(A,B):return B@A.T

def desc(D):
 v=Rot.from_matrix(D).as_rotvec(); t=np.linalg.norm(v)
 return {'angle_deg':float(np.degrees(t)),'axis':None if t<1e-10 else (v/t).tolist(),'R':D.tolist()}

def prediction(train_axis,gain,D,sign):
 axis=np.asarray(train_axis);v=Rot.from_matrix(D).as_rotvec();meas=np.linalg.norm(v)
 return {'observed':desc(D),'command_only_error_deg':ang(Rot.from_rotvec(axis*np.radians(gain)*sign).as_matrix()@D.T),'conditional_axis_error_deg':ang(Rot.from_rotvec(axis*meas*sign).as_matrix()@D.T)}

def main():
 logs=[json.loads(x) for x in (P/'log.jsonl').read_text().splitlines()]
 frames=[x for x in logs if x.get('event')=='frame'];observations={};prev=None;changes=0
 for f in frames:
  o=json.loads((P/f['file']).with_suffix('.json').read_text());b=o.get('board');R=None;selected=None
  if o.get('marker'):
   bs=o['marker']['branches'];valid=[i for i,x in enumerate(bs) if x['positive_depth']];best=min(valid,key=lambda i:bs[i]['rms_px']);ties=[i for i in valid if bs[i]['rms_px']<=bs[best]['rms_px']+.25];selected=best
   if prev is not None and len(ties)>1:selected=min(ties,key=lambda i:ang(np.asarray(bs[i]['R'])@prev.T))
   changes+=int(selected!=best);R=np.asarray(bs[selected]['R']);prev=R
  observations[Path(f['file']).stem]={'R_F_M':None if R is None else R.tolist(),'selected_branch':selected,'R_W_M':None if R is None or not b or not b['accepted'] else (np.asarray(b['R_cam_board']).T@R).tolist(),'R_W_G':None if o['camera']!='gripper' or not b or not b['accepted'] else np.asarray(b['R_cam_board']).T.tolist(),'raw':o,'receipt_at_ms':f['at'],'exposure_at':None}
 def pose(prefix,key):
  rs=[np.asarray(v[key]) for k,v in observations.items() if k.startswith(prefix) and v[key] is not None and (not k.endswith('Fafter') or prefix.endswith('Fafter'))]
  if not rs:raise ValueError('No valid '+prefix+' '+key)
  return mean(rs)
 def scatter(prefix,key):
  rs=[np.asarray(v[key]) for k,v in observations.items() if k.startswith(prefix) and v[key] is not None and (not k.endswith('Fafter') or prefix.endswith('Fafter'))];m=mean(rs)
  errors=[ang(x@m.T) for x in rs];pairs=[ang(a@b.T) for i,a in enumerate(rs) for b in rs[:i]]
  return {'n':len(rs),'rms_to_mean_deg':float(np.sqrt(np.mean(np.square(errors)))),'max_pair_deg':max(pairs) if pairs else None}
 F=lambda n:pose(n,'R_F_M');G=lambda n:pose(n,'R_W_G')
 Aref=F('Aref-F');Aplus=F('Aplus-F');Areturn=F('Areturn-F');D=rel(Aref,Aplus);train=desc(D)
 forward_g=rel(G('A0-G'),G('Aplus-G'));tg=desc(forward_g)
 # Use only pre-existing front reference floor observations to map the training axis into W.
 floor=[np.asarray(v['raw']['board']['R_cam_board']) for k,v in observations.items() if k.startswith('Aref-F') and v['raw'].get('board') and v['raw']['board']['accepted']]
 R_F_W=mean(floor);axisW=R_F_W.T@np.asarray(train['axis'])
 freturns={'Aplus_to_Areturn':prediction(train['axis'],train['angle_deg'],rel(Aplus,Areturn),-1),'A250forward_to_Afinalreturn':prediction(train['axis'],train['angle_deg'],rel(F('A250forward-F'),F('Afinalreturn-F')),-1)}
 gseq=['Aplus-G','Areturn-G','A230return-G','A220return-G','A230forward-G','A240forward-G','A250forward-G']
 gsteps=[]
 for i,(a,b) in enumerate(zip(gseq,gseq[1:])):
  sign=-1 if i<3 else 1;dd=rel(G(a),G(b));row={'from':a,'to':b,'sign':sign,**desc(dd)}
  row['G_training_prediction']=prediction(tg['axis'],tg['angle_deg'],dd,sign)
  row['F_training_axis_W_prediction']=prediction(axisW,train['angle_deg'],dd,sign);gsteps.append(row)
 C=rel(G('C240-G'),G('C230-G'))
 noise={n:scatter(n,'R_F_M') for n in ['A0-F','Aplus-F','C240-F']}
 gnoise={n:scatter(n,'R_W_G') for n in ['A0-G','Aplus-G','C240-G']}
 floor_R=[np.asarray(v['raw']['board']['R_cam_board']) for v in observations.values() if v['raw']['camera']=='front' and v['raw'].get('board') and v['raw']['board']['accepted']]
 mfloor=mean(floor_R);fe=[ang(x@mfloor.T) for x in floor_R]
 bracket={n:ang(F(n+'-Fafter')@F(n+'-F').T) for n in ['A0','Aplus']}
 possteps=np.array([x['angle_deg'] for x in gsteps if x['sign']==1]);negsteps=np.array([x['angle_deg'] for x in gsteps if x['sign']==-1])
 axisg=Rot.from_matrix(rel(G('A220return-G'),G('A250forward-G'))).as_rotvec();axisg/=np.linalg.norm(axisg)
 alignment=float(np.degrees(np.arccos(np.clip(axisg@axisW,-1,1))))
 # Repeated same-command endpoints retain hysteresis/noise, not encoder truth.
 returns={label:ang(rel(G(a),G(b))) for label,a,b in [('250', 'Aplus-G','A250forward-G'),('240','Areturn-G','A240forward-G'),('230','A230return-G','A230forward-G')]}
 rows=[]
 for key,v in observations.items():
  if v['R_F_M'] is not None:
   R=np.asarray(v['R_F_M']);q=Rot.from_matrix(R).as_quat();rows.append({'frame':key,'quaternion_xyzw':q.tolist(),'normal_F':R[:,2].tolist(),'centre_F_m':v['raw']['marker']['branches'][v['selected_branch']]['t']})
 results={'status':'PARTIAL front coverage; complete 4-setting G30deg return/forward arc; no fault recurrence','counts':{'capture_requests':sum(x.get('name')=='get_camera_frame' for x in logs),'retained_images':len(frames),'jog_attempts':sum(x.get('args',{}).get('method')=='app_arm_move' for x in logs),'null_marker_views':sum(v['R_F_M'] is None for v in observations.values()),'marker_observations':len(rows),'valid_G_board_observations':sum(v['R_W_G'] is not None for v in observations.values()),'valid_F_floor_observations':len(floor_R),'branch_tie_changes':changes,'capture_failures':sum(x.get('name')=='get_camera_frame' and x.get('isError',False) for x in logs)},'F_training_A105':train,'F_axis_positive_W':axisW.tolist(),'F_holdout_returns':freturns,'F_endpoint_closure_deg':ang(rel(Aref,Areturn)),'F_final_endpoint_closure_deg':ang(rel(Aref,F('Afinalreturn-F'))),'F_visible_settings_nominal':[240,250],'F_span_deg':train['angle_deg'],'G_training_A105':tg,'G_steps':gsteps,'G_four_settings_nominal':[220,230,240,250],'G_long_arc_forward':desc(rel(G('A220return-G'),G('A250forward-G'))),'G_long_arc_reverse':desc(rel(G('Aplus-G'),G('A220return-G'))),'G_forward_step_mean_deg':float(possteps.mean()),'G_forward_step_SD_deg':float(possteps.std(ddof=1)),'G_reverse_step_mean_deg':float(negsteps.mean()),'G_reverse_step_SD_deg':float(negsteps.std(ddof=1)),'G_endpoint_closures_deg':returns,'C95_negative_step':desc(C),'C95_G_training_holdout':prediction(tg['axis'],tg['angle_deg'],C,-1),'C95_F_training_axis_W_holdout':prediction(axisW,train['angle_deg'],C,-1),'cross_camera_A_step_angle_difference_deg':train['angle_deg']-tg['angle_deg'],'cross_camera_long_G_axis_vs_F_axis_W_deg':alignment,'front_stationary_noise':noise,'G_stationary_noise':gnoise,'F_G_F_bracket_change_deg':bracket,'front_floor_orientation_scatter':{'n':len(floor_R),'RMS_deg':float(np.sqrt(np.mean(np.square(fe)))),'max_deg':max(fe)},'identifiability':'Rotation-only: angles/axis directions, not lengths, axis stations, unique zeros or mechanical wrist mount. W is current board frame, not surveyed chassis. Front lens upper-region extrapolation and square planarity remain systematic uncertainties.','limits':'No repeat second full group: marker cropped at230; 260105 ground bound;240115 firmware reach bound. Ended at35 jogs. No production adoption.','timing':'Receipt timestamps only; exposure timestamps unavailable. Sequential F/G/F, not simultaneous.','training_SD':None}
 def swing_twist(A,B):
  normal=float(np.degrees(np.arccos(np.clip(A[:,2]@B[:,2],-1,1))))
  q=Rot.from_matrix(A.T@B).as_quat();tw=np.array([0.,0.,q[2],q[3]]);tw/=np.linalg.norm(tw)
  return {'normal_change_deg':normal,'twist_about_before_marker_z_deg':float(np.degrees(2*np.arctan2(tw[2],tw[3]))),'relative_SO3':desc(rel(A,B))}
 results['small_step_return_G']={'forward':desc(rel(G('deployed-gripper'),G('small-step-gripper'))),'reverse':desc(rel(G('small-step-gripper'),G('small-return-gripper'))),'closure_deg':ang(rel(G('deployed-gripper'),G('small-return-gripper')))}
 results['F_3D_normal_and_inplane_roll']={'training':swing_twist(Aref,Aplus),'return':swing_twist(Aplus,Areturn)}
 rng=np.random.default_rng(68);sigma=np.radians(max(x['rms_to_mean_deg'] for x in noise.values()))/np.sqrt(3);angles=[];cones=[]
 for _ in range(1000):
  aa=Rot.from_rotvec(rng.normal(0,sigma,3)).as_matrix()@Aref;bb=Rot.from_rotvec(rng.normal(0,sigma,3)).as_matrix()@Aplus
  vv=Rot.from_matrix(rel(aa,bb)).as_rotvec();angles.append(np.degrees(np.linalg.norm(vv)));cones.append(np.degrees(np.arccos(np.clip(vv@np.asarray(train['axis'])/np.linalg.norm(vv),-1,1))))
 results['F_conditional_random_noise_MC']={'draws':1000,'input_stationary_RMS_deg':float(np.degrees(sigma)*np.sqrt(3)),'angle_95_interval_deg':np.percentile(angles,[2.5,97.5]).tolist(),'axis_95_cone_halfangle_deg':float(np.percentile(cones,95)),'caveat':'Isotropic pilot-noise scenario for single exposures, not systematic-inclusive accuracy or confidence from independent anatomical validation.'}
 results['G_prospective_horizon_note']='Original protocol withheld all A returns and configuration B. Visibility adaptation was frozen before final A forward step and C95 capture. Expanded G forward steps220->230/230->240 were not the originally named third-forward holdout; treat their evaluation exploratory. A return steps and all C95 samples remained unused in training.'
 (P/'observations-selected.json').write_text(json.dumps(observations,indent=2));(P/'orientation-diagnostics.json').write_text(json.dumps(rows,indent=2));(P/'results.json').write_text(json.dumps(results,indent=2))
 with (P/'step_angles.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=['from','to','sign','angle_deg','axis']);w.writeheader();w.writerows({k:x[k] for k in w.fieldnames} for x in gsteps)
 # Baseline and originals verified, then hash manifest excludes itself and final verification.
 prior=json.loads((P/'prior-source-sha256.json').read_text());root=P.parents[2];mismatches=[x for x,h in prior.items() if hashlib.sha256((root/x).read_bytes()).hexdigest()!=h]
 imagebad=[x['file'] for x in frames if hashlib.sha256((P/x['file']).read_bytes()).hexdigest()!=x['sha256']]
 manifest={str(x.relative_to(P)):hashlib.sha256(x.read_bytes()).hexdigest() for x in P.rglob('*') if x.is_file() and x.name not in ['artifact-sha256.json','verification.json']}
 (P/'artifact-sha256.json').write_text(json.dumps(manifest,indent=2));(P/'verification.json').write_text(json.dumps({'prior_files_checked':len(prior),'prior_mismatches':mismatches,'original_images_checked':len(frames),'image_mismatches':imagebad,'all_hashes_ok':not mismatches and not imagebad},indent=2))
 print(json.dumps({k:v for k,v in results.items() if k not in ['G_steps']},indent=2))
if __name__=='__main__':main()
