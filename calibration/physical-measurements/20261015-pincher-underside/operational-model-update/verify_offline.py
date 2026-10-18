"""Deterministic offline verification/metrics; no external services or hardware."""
from pathlib import Path
import hashlib
import json
import math
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
from z70_description import pincher_geometry as g
from z70_manipulation import planner as p

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3];PKG=ROOT/'ros2/src/z70_description'
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
source=ET.fromstring(xacro.process_file(str(PKG/'urdf/z70.urdf.xacro')).toxml())
before=ET.fromstring(xacro.process_file(str(HERE/'before/z70.urdf.xacro')).toxml())
signature=lambda e:(e.tag,sorted(e.attrib.items()),[signature(c) for c in e])
preserved={}
for joint in before.findall('joint'):
    name=joint.get('name')
    if name.startswith('m6_'):continue
    preserved[name]=signature(joint)==signature(source.find(f"joint[@name='{name}']"))
assert all(preserved.values())
cal='ros2/src/z70_description/config/mount_calibration.yaml'
old=subprocess.check_output(['git','show','HEAD:'+cal],cwd=ROOT)
assert (ROOT/cal).read_bytes()==old
share=Path(get_package_share_directory('z70_description'))
assets={str(f.relative_to(PKG)):sha(f) for group in ('meshes/*.stl','config/pincher_geometry.yaml','urdf/pincher_collisions.xacro') for f in PKG.glob(group)}
assert all(sha(share/k)==v for k,v in assets.items())
poses=[]
for state in ((150,45,0,90),(240,95,0,0),(240,95,0,45),(240,95,0,90),(240,95,40,90),(250,95,0,90),(250,105,0,90)):
    detail=g.articulated_geometry(state)
    poses.append(dict(state=state,safe=p.safe(state),body_floor_bound_m=p.body_floor_bound(state),
        legacy_surrogate_floor_bound_m=p.tip_floor_bound(state),operational_aim_m=p.operational_point(state),
        terminal_gap_m=detail['terminal_gap_m'],finger_frames={k:v.tolist() for k,v in detail['finger_frames'].items()},
        bodies=[dict(name=b.name,pad=b.pad,center_m=b.center.tolist(),half_m=b.half.tolist(),rotation=b.rotation.tolist()) for b in detail['bodies']]))

states=[(90+(i%19)*10,35+((i//19)%14)*10,(i%4)*40,90) for i in range(1000)]
p._body_floor.cache_clear();start=time.perf_counter();answers=[p.safe(s) for s in states];query_s=time.perf_counter()-start
p._body_floor.cache_clear();start=time.perf_counter();bounds=[p.body_floor_bound(s) for s in states];full_geometry_s=time.perf_counter()-start
assert all(np.isfinite(bounds))
start=time.perf_counter();goal=p.grasp_goal(p.grasp_point(),(190,65));path=p.joint_plan((150,45),tuple(goal['state']));bfs_s=time.perf_counter()-start
T=g.hand_pose(tuple(goal['state']));target=p.operational_point(tuple(goal['state']));target[2]=.033
start=time.perf_counter();closure=p.closure_check(T,target,.066);closure_s=time.perf_counter()-start
assert not closure['ok']  # No anatomy/uncertainty fitting to the old66mm fixture.

# Convex distance maximum is bounded by the corners of each convex pad box.
# At EVERY opening used by closure_check, even the farthest pad corner is less
# than2*uncertainty+sweep_margin from housing. By triangle inequality a sphere
# cannot both reach that pad robustly (R-u) and clear housing (R+u+margin),
# regardless of sphere size or rigid hand pose. This is a conservative-model
# limitation, NOT a statement that the hardware cannot grasp.
housing=g.local_bodies(90)[0]
maximum=max((housing.sphere_distance(c),m6,c.tolist()) for m6 in range(91)
            for b in g.local_bodies(m6) if b.pad for c in b.corners())
required=2*g.CONFIG['uncertainty_m']+.0015
assert maximum[0]<required
known=np.eye(4);known[2,3]=.08
known_target=g.grasp_reference()+[0,0,.08]
nominal=p.closure_check(known,known_target,.032,uncertainty_m=0)
assert nominal['ok'] and not p.closure_check(known,known_target,.032)['ok']

# Retain the ACTUAL unpatched localhost HTTP/JPEG/marker/servo rejection trace.
trace_base=ROOT/'ros2/build/pytest-compact-final'
if not trace_base.exists():trace_base=ROOT/'ros2/build/pytest'
traces=sorted({path.resolve() for path in trace_base.glob('test_pick_http_jpeg_servo_r*/run/timeline.jsonl')})
assert len(traces)==1, traces
trace=traces[0];rows=[json.loads(line) for line in trace.read_text().splitlines()]
checks=[r for r in rows if r['event']=='preclose_geometry']
assert checks and not any(r['ok'] for r in checks)
assert not any(r['event']=='complete' for r in rows)
shutil.copy(trace,HERE/'offline-http-pick-timeline.jsonl')
e2e=dict(source='synthetic localhost HTTP+JPEG marker scene; NOT hardware',preclose=checks,rows=len(rows),
    source_trace=str(trace.relative_to(ROOT)),
    outcome='genuine geometry rejection before grip; cleanup stows',verified_pick=False,
    servo_checks=[r for r in rows if r['event']=='front_jaw_check'])

centers=np.array([r.mean(0) for r in g.finger_rings(1)])
arc=float(np.linalg.norm(np.diff(centers,axis=0),axis=1).sum())
# Reproduce discarded draft metrics without using the rejected solver at runtime.
old_rings=[r.copy() for r in g.finger_rings(1)]
for ring in old_rings:ring[:,0]/=g.CONFIG['finger_forward_reach_m']
a=math.radians(90)*.85;target_x=g.CONFIG['open_assembly_envelope_m']-g.CONFIG['root_elbow_x_m']
old_length=min((target_x+v[1]*math.sin(a))/(v[0]*math.cos(a)) for v in np.concatenate(old_rings) if v[0]>0)
for ring in old_rings:ring[:,0]*=old_length
old_centers=np.array([r.mean(0) for r in old_rings])
opened=np.concatenate([r@g.finger_frame(s,90)[:3,:3].T+g.root(s) for s in (-1,1) for r in g.finger_rings(s)])
results=dict(units='metres, degrees for app states',status=g.CONFIG['status'],
    owner_photos={name:sha(HERE.parent/name) for name in ('IMG_0647_4ec758.png','IMG_0648_87aeb8.png')},
    preserved_joint_definitions=preserved,mount_calibration_unchanged_sha256=sha(ROOT/cal),installed_asset_sha256=assets,
    collision_count_by_link={name:len(source.find(f"link[@name='{name}']").findall('collision')) for name in ('arm_wrist_link','finger_left','finger_right')},
    anatomy=dict(top_z_m=max(np.concatenate(g.shell_rings())[:,2]),width_m=float(np.ptp(np.concatenate(g.shell_rings())[:,1])),
        root_elbow_xyz_m=[g.CONFIG['root_elbow_x_m'],g.CONFIG['root_half_gap_m'],g.CONFIG['root_z_m']],
        rough_owner_open_envelope_m=g.CONFIG['open_assembly_envelope_m'],model_open_envelope_m=float(opened[:,0].max()+.205),
        finger_forward_reach_m=g.CONFIG['finger_forward_reach_m'],finger_centerline_arc_m=arc,
        finger_pivot_to_tip_centerline_m=float(np.linalg.norm(np.diff(centers[1:],axis=0),axis=1).sum()),
        closed_terminal_station_elbow_m=float(centers[-1,0]+g.CONFIG['root_elbow_x_m']),
        nominal_open_angle_deg=90*g.CONFIG['opening_ratio'],nominal_open_terminal_gap_m=g.articulated_geometry((240,95,0,90))['terminal_gap_m'],
        opening_ratio=g.CONFIG['opening_ratio'],uncertainty_m=g.CONFIG['uncertainty_m'],
        discarded_draft=dict(forward_reach_m=old_length,centerline_arc_m=float(np.linalg.norm(np.diff(old_centers,axis=0),axis=1).sum()),
            nominal_open_angle_deg=76.5,closed_terminal_station_elbow_m=old_length+g.CONFIG['root_elbow_x_m'],
            open_terminal_gap_m=float(2*(g.root(1)[1]+(old_rings[-1]@g.rpy([0,0,a]).T)[:,1].min())))),
    robust_contact_limitation=dict(maximum_pad_corner_distance_from_housing_m=maximum[0],m6_at_maximum=maximum[1],
        required_separation_m=required,conclusion='No sphere size/rigid hand pose can pass current default robust sampled model; not a physical impossibility.'),
    known_zero_error_32mm_unit_example=dict(hand_pose=known.tolist(),target_center=known_target.tolist(),size_m=.032,
        result=nominal,default_uncertainty_result=p.closure_check(known,known_target,.032),
        scope='Analytic classifier only; zero-error input is NOT executor configuration or hardware evidence.'),
    poses=poses,performance=dict(queries=1000,safe_count=sum(answers),query_seconds=query_s,full_geometry_seconds=full_geometry_s,bfs_seconds=bfs_s,path_steps=len(path),
        closure_seconds=closure_s,closure=closure),selected_model_goal=goal,offline_end_to_end=e2e,
    limitations=['No hardware validation; widths/depths/pivots/open angle/contact regions are provisional, NOT calibrated from photographs.',
        'Default15mm uncertainty and conservative housing make robust contact unresolved for ALL spherical targets; no successful default full-pick claim.',
        'Optional per-piece world-AABB refusals are not acquired scene mapping or complete chassis/self-collision.',
        'Opening/closure assume nominal open90 then commanded closure, not actual M6 feedback.',
        'Original nominal2.8mm lens is retained behind a smaller aperture, with partly masked rim; optical-forward ray only, NOT full FOV validation.'])
# Existing22-file ownership manifest excludes unrelated dirty source changes.
changed=json.loads((HERE/'changed-files.json').read_text())
assert len(changed)==22 and len(set(changed))==22 and all((ROOT/f).exists() for f in changed)
results['task_source_sha256']={f:sha(ROOT/f) for f in changed}
(HERE/'verification.json').write_text(json.dumps(results,indent=2)+'\n')
(HERE/'z70-operational.urdf').write_text(ET.tostring(source,encoding='unicode'))
print(json.dumps(dict(anatomy=results['anatomy'],robust_contact_limitation=results['robust_contact_limitation'],
    performance=results['performance'],model_goal=goal,offline_end_to_end=e2e,
    preserved_joint_count=len(preserved),installed_asset_count=len(assets),task_source_file_count=len(changed)),indent=2))
