# Supplemental analysis; consumes only this trial's immutable raw JSON.
import analyze_rotations as a
from scipy.spatial.transform import Rotation as Rot
import numpy as np,json
p=a.HERE
# Temporal branch continuity is audited independently of any arm model.
track=[];previous=None
for f in a.frames:
 name=__import__('pathlib').Path(f['file']).stem;o=a.obs.get(name)
 if not o or not o['marker']:continue
 bs=o['marker']['branches'];good=[(i,b) for i,b in enumerate(bs) if b['positive_depth']]
 # joint increments unconstrained: reprojection is primary, continuity only a tie-break
 # within 0.25px of the minimum; no FK or requested angle enters this selection.
 minerr=min(b['rms_px'] for i,b in good);eligible=[(i,b) for i,b in good if b['rms_px']<=minerr+.25]
 i,b=min(eligible,key=lambda ib:0 if previous is None else a.angle(previous,np.array(ib[1]['R'])))
 rr=np.array(b['R']);track.append(dict(file=name,chosen_branch=i,minimum_reprojection_branch=min(good,key=lambda ib:ib[1]['rms_px'])[0],separation_deg=a.angle(np.array(bs[0]['R']),np.array(bs[1]['R'])),rms_gap_px=abs(bs[0]['rms_px']-bs[1]['rms_px']),interexposure_deg=None if previous is None else a.angle(previous,rr)));previous=rr
# Long-arc axes reduce noise amplification of individual ten-degree increments.
long={}
for name,x,y in [('M3','M3-240-105a','M3-250-105a'),('M4','route24085','M4-240-115a'),('M5_A','M3-250-105c','A-roll40a'),('M5_B','M4-250-115b','B-roll40c')]:
 v=Rot.from_matrix(a.R(y)@a.R(x).T).as_rotvec();u=v/np.linalg.norm(v);long[name]=dict(angle_deg=float(np.degrees(np.linalg.norm(v))),positive_command_axis_F=u.tolist(),positive_command_axis_W=(a.RFw.T@u).tolist())
parallel=float(np.degrees(np.arccos(np.clip(abs(np.array(long['M3']['positive_command_axis_F'])@long['M4']['positive_command_axis_F']),0,1))))
u=np.array(long['M5_A']['positive_command_axis_F']);v=np.array(long['M5_B']['positive_command_axis_F']);transport=a.D4@u
consistency=float(np.degrees(np.arccos(np.clip(transport@v,-1,1))))
# All G and F are sequential, never simultaneous. Brackets used at three key poses.
brackets={}
for name,before,after in [('B-reference','pilotB-5','B-Fafter'),('A-zero','M3-250-105b','M3-250-105c'),('A-roll40','A-roll40b','A-roll40c'),('B-roll40','B-roll40a','B-roll40c')]:brackets[name]=dict(orientation_change_deg=a.angle(a.R(before),a.R(after)),before=before,after=after)
# Pairwise repeated nominal roll return errors; do not claim backlash below noise.
hysteresis={}
for g,zero in [('A',a.A0),('B',a.B0)]:
 vals={}
 for i in [10,20,30]:vals[str(i)]=a.angle(a.mean([f'{g}-roll{i}a',f'{g}-roll{i}b']),a.mean([f'{g}-roll{i}ra',f'{g}-roll{i}rb']))
 hysteresis[g]=vals
pose_stats=[]
for f in a.frames:
 name=__import__('pathlib').Path(f['file']).stem;o=a.obs.get(name)
 if not o or not o['marker']:continue
 r=a.R(name);board=o['board'];pose_stats.append(dict(file=name,command_count_nominal=f['nominal'],R_F_M=r.tolist(),quaternion_xyzw=Rot.from_matrix(r).as_quat().tolist(),R_W_M=None if not board or not board['accepted'] else (np.array(board['R_cam_board']).T@r).tolist(),normal_F=r[:,2].tolist(),marker_positive_z_angle_to_optical_deg=float(np.degrees(np.arccos(np.clip(r[2,2],-1,1)))),euler_intrinsic_xyz_deg=Rot.from_matrix(r).as_euler('XYZ',degrees=True).tolist()))
result=dict(temporal_branch_audit=track,long_arc_axes=long,M3_M4_unsigned_parallelism_deg=parallel,M5_axis_transport_discrepancy_deg=consistency,FGF_brackets=brackets,roll_hysteresis_deg=hysteresis,pose_orientations=pose_stats,branch_switches_vs_minimum=sum(t['chosen_branch']!=t['minimum_reprojection_branch'] for t in track),heldout_is_exploratory=True,rotation_zero_gauge='Rmount and M4 zero confounded; M3 zero gauges common pitch/mount; no physical zeros fitted',uncertainty_note='pilot-noise intervals conditional only; lens extrapolation, printed square flatness and corner bias not measured by bootstrap')
(p/'orientation-diagnostics.json').write_text(json.dumps(result,indent=2))
print(json.dumps({k:result[k] for k in ['long_arc_axes','M3_M4_unsigned_parallelism_deg','M5_axis_transport_discrepancy_deg','FGF_brackets','roll_hysteresis_deg','branch_switches_vs_minimum']},indent=2))
