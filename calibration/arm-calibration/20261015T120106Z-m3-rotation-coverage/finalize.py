import json,hashlib
from pathlib import Path
P=Path(__file__).resolve().parent
rows=[json.loads(x) for x in (P/'log.jsonl').read_text().splitlines()]
frames=[r for r in rows if r.get('event')=='frame']
captures=[r for r in rows if r.get('name')=='get_camera_frame']
jogs=[r for r in rows if r.get('args',{}).get('method')=='app_arm_move']
prior=P.parent/'20261015T104441Z-marker68-rotation'
before=json.loads((P/'prior-before-sha256.json').read_text())
after={str(x.relative_to(prior)):hashlib.sha256(x.read_bytes()).hexdigest() for x in prior.rglob('*') if x.is_file()}
assert before==after,'Prior archive changed'
assert all(hashlib.sha256((P/r['file']).read_bytes()).hexdigest()==r['sha256'] for r in frames)
r={'outcome':'BLOCKED before M3 motion; camera unavailable then local TCP transport unavailable','capture_requests':len(captures),'retained_original_images':len(frames),'failed_capture_requests':sum(bool(r.get('isError')) for r in captures),'jog_attempts':len(jogs),'observed_m3_settings':0,'fixed_m4_sweep_configurations':0,'m3_span_deg':None,'m3_axis_F':None,'forward_return_closure_deg':None,'backlash_deg':None,'stationary_sigma_deg':None,'pincher_crosscheck_deg':None,'heldout_errors_deg':None,'cleanup':{'stow_ack':True,'program_exit_ack':True,'pickup_exit_ack':True,'physical_stow_verified':False,'modes_exited_verified':False,'last_raw_status_state':39,'last_raw_common_status':51458,'last_valid_battery_pct':55,'last_valid_error_code':0,'front_selected_last_known':True,'final_selected_camera_verified':False,'final_stream_consumers_empty':True,'stop_attempts':2,'stop_failures':'not_sent transport_error unavailable','final_link':'disconnected tcp-connect','final_estimate':'IN_ROBOT dead reckoning; NOT physical stow proof'},'camera_first_failure':json.loads((P/'first-failure-state.json').read_text())['camera']['acquisition'],'camera_quiet_retry_failure':json.loads((P/'quiet-recovery-failure-state.json').read_text())['camera']['acquisition'],'prior_archive_unchanged':True,'prior_images_verified':sum(k.endswith('.jpg') for k in before),'no_new_rotation_measurement':True}
(P/'results.json').write_text(json.dumps(r,indent=2))
(P/'verification.json').write_text(json.dumps({'prior_file_count':len(before),'prior_hashes_unchanged':True,'new_original_hashes_verified':True,'new_images':len(frames)},indent=2))
manifest={str(x.relative_to(P)):hashlib.sha256(x.read_bytes()).hexdigest() for x in P.rglob('*') if x.is_file() and x.name!='artifact-sha256.json'}
(P/'artifact-sha256.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(r,indent=2))
