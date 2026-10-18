from analyze import *
from collections import Counter
rows=[json.loads(l) for l in (HERE/'log.jsonl').read_text().splitlines()]
caps=[r for r in rows if r.get('event')=='capture'];jogs=[r for r in rows if r.get('method')=='app_arm_move'];c=Counter(r['camera'] for r in caps);c['front']+=1
frozen={path:hashlib.sha256((BASE/path).read_bytes()).hexdigest()==h for path,h in read(HERE/'frozen-hashes.json').items()}
raw={p.name:{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'dimensions':list(cv2.imread(str(p)).shape[1::-1])} for p in (HERE/'raw').glob('*.jpg')}
save('manifest.json',{'raw':raw,'frozen_verified':frozen,'code_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')}})
assert len(raw)==len(caps)+1==47
assert all(frozen.values())
counts={'raw_images':len(raw),'front':c['front'],'gripper':c['gripper'],'route_jogs':sum(r['params']['m'][2]==0 for r in jogs),'roll_jogs':sum(r['params']['m'][2]!=0 for r in jogs),'jog_RPC_ACKs':sum(r['result']['result']==['ok'] for r in jogs),'actual_successful_full_jogs':None,'actual_refused_or_partial_jogs':None,'reason':'ACKs do not measure completion; G observations retained, end nominal angles invalidated by partial/refused route movement','chassis_pulses':0,'brackets_captured':4,'target_neighbor_brackets':2,'exact_archived_target_brackets':0,'comparator_neighbor_brackets':2,'marker_or_optical_refits':0,'primary_accepted_brackets':2,'recovery_sensitivity_accepted_brackets':3}
save('counts.json',counts)
primary=read(HERE/'prospective-evaluation.json');rec=read(HERE/'detector-recovery-prospective-evaluation.json')
# Board-observed span of achieved camera, not nominal M5.
a=np.array(rec['brackets'][0]['models']['fcyk1']['T_F_G']);b=np.array(rec['brackets'][1]['models']['fcyk1']['T_F_G'])
span={'rotation_deg':float(np.degrees(Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).magnitude())),'camera_center_change_m':float(np.linalg.norm(a[:3,3]-b[:3,3])),'marker_pixel_bounds_xy':{'min':np.min(np.array([f['observed_px'] for p in rec['brackets'] for f in p['models']['fcyk1']['front']]).reshape(-1,2),axis=0).tolist(),'max':np.max(np.array([f['observed_px'] for p in rec['brackets'] for f in p['models']['fcyk1']['front']]).reshape(-1,2),axis=0).tolist()}}
save('observed-span.json',span)
statuses=[r for r in rows if r.get('method')=='get_status'];final=statuses[-1]['result']['result'][0]
save('cleanup-verification.json',{'raw_status':final,'grab_mode_bits10_11':(final['common_status']>>10)&3,'raw_zeros_not_pose_proof':True,'final_image':'raw/046-final-front.jpg','image_observation':'no deployed arm visible; corroboration only','physical_stow_corroboration':['explicit stow and settling','last front image absence','grab-mode0','IN_ROBOT estimator after final explicit stow (dead reckoning, not encoder truth)'],'program_runtime':[r['result']['result'] for r in rows if r.get('method')=='app_get_program_runtime'][-1],'off_dock':True,'camera_consumers_empty':all(not x['consumers'] for x in read(HERE/'final-state.json')['camera']['streamDiagnostics'].values())})
print(json.dumps({'counts':counts,'span':span,'cleanup':{'state':final['state'],'battery':final['battery'],'common_status':final['common_status'],'grab_mode':(final['common_status']>>10)&3},'frozen_verified':len(frozen)}))
