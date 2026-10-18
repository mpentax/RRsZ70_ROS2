"""Offline rotation-only analysis. Original observations are never rewritten."""
import json,csv,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import least_squares
HERE=Path(__file__).resolve().parent
read=lambda p:json.loads(p.read_text())
obs={p.stem:read(p) for p in (HERE/'raw').glob('*.json')}
logs=[json.loads(s) for s in (HERE/'log.jsonl').read_text().splitlines()]
frames=[x for x in logs if x.get('event')=='frame']
def angle(A,B):return float(np.degrees(Rot.from_matrix(B@A.T).magnitude()))
def R(name):
 o=obs[name];m=o['marker']
 if m is None:return None
 # Model-free: positive-depth lowest reprojection seed. Branch alternatives retained;
 # primary remains same branch if adjacent-pose geodesic continuity and rms agree.
 b=min((b for b in m['branches'] if b['positive_depth']),key=lambda x:x['rms_px'])
 return np.array(b['R'])
def mean(names):
 rr=[R(n) for n in names if n in obs and R(n) is not None]
 return Rot.from_matrix(rr).mean().as_matrix() if rr else None
A0=mean(['M3-250-105a','M3-250-105b','M3-250-105c'])
B0=mean(['M4-250-115a','M4-250-115b'])
pilots={}
for key,names in {'A':['route24085','route24085-b','pilotA-1','pilotA-2','pilotA-3'],'B':[f'pilotB-{i}' for i in range(1,6)]}.items():
 mu=mean(names);dev=[angle(mu,R(n)) for n in names];pairs=[angle(R(a),R(b)) for i,a in enumerate(names) for b in names[i+1:]]
 pilots[key]=dict(n=len(names),rms_to_mean_deg=float(np.sqrt(np.mean(np.square(dev)))),max_pair_deg=max(pairs),pair_mean_deg=float(np.mean(pairs)),pair_std_deg=float(np.std(pairs)),selected_branch='minimum positive-depth reprojection; alternatives retained')
# Frame pair comparisons never silently zero out a missing marker.
seqs={
 'M4_240':['route24085','pilotB-5','M4-240-105a','M4-240-115a','M4-240-105ra'],
 'M3_105':['M3-230-105a','M3-240-105a','M3-250-105a'],
 'M4_250':['A-roll0ra','M4-250-115a'],
 'M5_A':['M3-250-105c']+[f'A-roll{i}a' for i in [10,20,30,40]]+[f'A-roll{i}ra' for i in [30,20,10,0]],
 'M5_B':['M4-250-115b']+[f'B-roll{i}a' for i in [10,20,30,40]]+[f'B-roll{i}ra' for i in [30,20,10,0]]}
rows=[];axes={}
for group,seq in seqs.items():
 vv=[]
 for ix,(a,b) in enumerate(zip(seq,seq[1:])):
  ra,rb=R(a),R(b)
  row=dict(group=group,before=a,after=b,angle_deg=None,axis_F=None,normal_change_deg=None,inplane_twist_marker_z_deg=None,accepted=False)
  if ra is not None and rb is not None:
   vec=Rot.from_matrix(rb@ra.T).as_rotvec();ang=np.linalg.norm(vec);axis=vec/ang if ang>1e-8 else np.zeros(3)
   # swing-twist decomposition about OLD marker normal, not 2D edge angles.
   local=Rot.from_matrix(ra.T@rb).as_quat();twist=2*np.arctan2(local[2],local[3]);twist=(twist+np.pi)%(2*np.pi)-np.pi
   row.update(angle_deg=float(np.degrees(ang)),axis_F=axis.tolist(),normal_change_deg=float(np.degrees(np.arccos(np.clip(ra[:,2]@rb[:,2],-1,1)))),inplane_twist_marker_z_deg=float(np.degrees(twist)),accepted=True)
   if np.degrees(ang)>max(p['max_pair_deg'] for p in pilots.values()):vv.append(vec*(1 if ix< (4 if group.startswith('M5') else len(seq)-2) else -1))
  rows.append(row)
 # axis sign resolved consistently by first identifiable observed change, not desired model.
 valid=[np.array(x['axis_F']) for x in rows if x['group']==group and x['accepted'] and x['angle_deg']>max(p['max_pair_deg'] for p in pilots.values())]
 if valid:
  valid=[v if v@valid[0]>=0 else -v for v in valid];u=np.mean(valid,axis=0);u/=np.linalg.norm(u)
  axes[group]=dict(axis_F_unsigned=u.tolist(),angular_scatter_deg=[float(np.degrees(np.arccos(np.clip(v@u,-1,1)))) for v in valid],n_steps=len(valid))
# New block stationary floor transform from current measured F board, no historical TF.
floors=[np.array(o['board']['R_cam_board']) for n,o in obs.items() if o['camera']=='front' and n not in ['preflight','center-turn1','center-turn2','deployed'] and o['board'] and o['board']['accepted']]
RFw=Rot.from_matrix(floors).mean().as_matrix()
floor_scatter=[angle(RFw,r) for r in floors]
for a in axes.values():a['axis_W_unsigned']=(RFw.T@a['axis_F_unsigned']).tolist()
# Hold out ALL B rolled poses. Train only A roll sequence + pure M4 observed transition.
D4=B0@A0.T;hold=[]
for i in [10,20,30,40]:
 ar=mean([f'A-roll{i}a',f'A-roll{i}b']);br=mean([f'B-roll{i}a',f'B-roll{i}b',f'B-roll{i}c'])
 hold.append(dict(nominal_roll=i,error_deg=angle(D4@ar,br),predicted_R=(D4@ar).tolist(),observed_R=br.tolist()))
# Axis/single-step uncertainty: whole-frame pilot bootstrap propagated to each measurement.
rng=np.random.default_rng(68);sig=max(p['rms_to_mean_deg'] for p in pilots.values())
for row in rows:
 if not row['accepted']:continue
 a,b=R(row['before']),R(row['after']);draw=[]
 for _ in range(500):
  aa=Rot.from_rotvec(rng.normal(0,np.radians(sig)/np.sqrt(3),3)).as_matrix()@a;bb=Rot.from_rotvec(rng.normal(0,np.radians(sig)/np.sqrt(3),3)).as_matrix()@b;draw.append(angle(aa,bb))
 row['conditional_angle_95_deg']=np.percentile(draw,[2.5,97.5]).tolist()
G={n:dict(angle_to_floor_deg=o['board']['optical_angle_to_floor_deg'],R_W_G=np.array(o['board']['R_cam_board']).T.tolist(),rms_px=o['board']['full_rms_px']) for n,o in obs.items() if o['camera']=='gripper' and o['board'] and o['board']['accepted']}
gnoise={}
for label,names in {'G180':['G180-a','G180-b','G180-c'],'GB':['GB-1','GB-2','GB-3']}.items():
 rr=[np.array(G[n]['R_W_G']) for n in names];mu=Rot.from_matrix(rr).mean().as_matrix();gnoise[label]=dict(n=len(rr),rms_to_mean_deg=float(np.sqrt(np.mean([angle(mu,r)**2 for r in rr]))),floor_angles_deg=[G[n]['angle_to_floor_deg'] for n in names])
gdeltas={}
for label,aa,bb in [('M3_20_nominal','G230105-a','G250105-a'),('M5_A40','G250105-a','GA-roll40a')]:
 gdeltas[label]=dict(observed_rotation_deg=angle(np.array(G[aa]['R_W_G']),np.array(G[bb]['R_W_G'])),before=aa,after=bb)
# Per-group aggregate and signed cumulative forward/reverse (full 3D SO3).
summary={}
for group,seq in seqs.items():
 rr=[r for r in rows if r['group']==group and r['accepted']];summary[group]=dict(measured_steps=len(rr),missing_steps=sum(r['group']==group and not r['accepted'] for r in rows),angle_mean_deg=float(np.mean([r['angle_deg'] for r in rr])) if rr else None,angle_std_deg=float(np.std([r['angle_deg'] for r in rr])) if rr else None)
 if group.startswith('M5'):
  mu0=R(seq[0]);mu40=R(seq[4]);muret=R(seq[-1]);summary[group].update(outward_total_deg=angle(mu0,mu40),return_to_start_deg=angle(mu0,muret),return_total_deg=angle(mu40,muret))
counts=dict(jog_attempts=sum(x.get('name')=='robot_rpc' and x.get('args',{}).get('method')=='app_arm_move' for x in logs),capture_requests=sum(x.get('name')=='get_camera_frame' for x in logs),capture_failures=sum(x.get('name')=='get_camera_frame' and bool(x.get('isError')) for x in logs),retained_logged_images=len(frames),retained_original_images=len(list((HERE/'raw').glob('*.jpg'))),front_marker_observations=sum(o['marker'] is not None for o in obs.values()),accepted_board_observations=sum(o['board'] is not None and o['board']['accepted'] for o in obs.values()),null_marker_observations=sum(o['marker'] is None for o in obs.values()),orientation_step_attempts=len(rows),orientation_step_accepted=sum(r['accepted'] for r in rows))
result=dict(counts=counts,pilot_front=pilots,pilot_gripper=gnoise,step_summary=summary,axes=axes,stationary_floor_orientation=dict(n=len(floors),R_F_W=RFw.tolist(),rms_scatter_deg=float(np.sqrt(np.mean(np.square(floor_scatter)))),max_scatter_deg=max(floor_scatter),warning='floor fit/board noise and actual rocking not separable; W axes conditional on this current block'),heldout=dict(protocol='all B-roll10..40 withheld; predict D4_observed_pure * R_Aroll; not command-only or independent anatomical validation',poses=hold,rms_deg=float(np.sqrt(np.mean([h['error_deg']**2 for h in hold])))),gripper_crosschecks=gdeltas,gripper_floor_angles=G,identifiability=dict(physical_link_lengths=None,physical_L2=None,physical_joint_zeros=None,M5_axis_station=None,reason='orientation has no link translations; fixed mount/joint-zero rotational gauges remain. M3/M4 only short visible arcs, no unique full mechanical zeros or lengths.'))
(HERE/'results.json').write_text(json.dumps(result,indent=2));(HERE/'step_rotations.json').write_text(json.dumps(rows,indent=2))
with (HERE/'step_angles.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=rows[0].keys()|{'conditional_angle_95_deg'});w.writeheader();w.writerows(rows)
(HERE/'image-sha256.json').write_text(json.dumps({str(p.relative_to(HERE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (HERE/'raw').glob('*.jpg')},indent=2))
print(json.dumps({k:result[k] for k in ['counts','pilot_front','pilot_gripper','step_summary','axes','heldout','gripper_crosschecks']},indent=2))
