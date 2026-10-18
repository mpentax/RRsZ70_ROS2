"""Offline report data and rejection diagnostics; no robot/service access."""
import sys,json,math
from pathlib import Path
P=Path(__file__).resolve().parent
REPO=P.parents[2]
sys.path.insert(0,str(REPO/'calibration/chassis'))
from analyze import analyze
rejection_details=[]
def trace(frame,event,arg):
    if frame.f_code.co_name=='estimate' and frame.f_code.co_filename.endswith('floor_vo.py') and event=='return':
        l=frame.f_locals
        rejection_details.append(dict(accepted=arg is not None,return_line=frame.f_lineno,matches=l.get('n'),inliers=int(l['best'].sum()) if l.get('best') is not None else None,rms_m=l.get('rms'),min_inliers=l.get('self').min_inliers,max_rms_m=l.get('self').max_rms))
    return trace
sys.settrace(trace)
result=analyze(P/'pairs.json',REPO/'calibration/front-camera/front_camera.json')
sys.settrace(None)
(P/'rejection-diagnostics.json').write_text(json.dumps(rejection_details,indent=2)+'\n')
rows=[json.loads(l) for l in (P/'ros-odom.jsonl').read_text().splitlines()]
status=[r for r in rows if r['topic']=='/z70/odom_status']
from collections import Counter
states=Counter(r['data']['state'] for r in status)
last=status[-1]['data']; ok=last['ok']; rejected=last['rejected']
stamps=sorted(set(r['data']['stamp_ns'] for r in status))
rate=(len(stamps)-1)/((stamps[-1]-stamps[0])/1e9)
frames=[json.loads(l) for l in (P/'frames.jsonl').read_text().splitlines()]
# Add pre-helper setup/baseline receipts from the actual MCP replies.
frames=list({r['file']:r for r in ([dict(file=n,receivedAt=t,exposure_time=None) for n,t in [('setup.jpg',1791956415632),('baseline-1.jpg',1791956476457),('baseline-2.jpg',1791956488278)]]+frames)}.values())
(P/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in frames))
rpc=[json.loads(l) for l in (P/'rpc.jsonl').read_text().splitlines()]
moves=[r for r in rpc if r['method']=='app_rc_move']
comparisons=[json.loads(l) for l in (P/'ros-comparison.jsonl').read_text().splitlines()]
settle=[json.loads(l) for l in (P/'settling.jsonl').read_text().splitlines()]
intervals={}
for ident in [r['id'] for r in json.loads((P/'pairs.json').read_text())]:
    f={r['file']:r['receivedAt'] for r in frames}
    intervals[ident]=(f[ident+'-after-2.jpg']-f[ident+'-after-1.jpg'])/1000
summary=dict(attempted=len(moves),complete_pairs=result['input_pairs'],accepted=result['accepted_pairs'],rejected=result['rejected_pairs'],incomplete=0,fit=result['fit'],ros_status_messages=len(status),ros_state_counts=dict(states),ros_accepted=ok,ros_rejected=rejected,ros_rejection_rate=rejected/(ok+rejected),ros_delivered_hz=rate,ros_first_stamp_ns=stamps[0],ros_last_stamp_ns=stamps[-1],after_pair_receipt_intervals_s=intervals,baseline_receipt_interval_s=(1791956488278-1791956476457)/1000,pulse_start_intervals_s=[(b['reply']['startedAt']-a['reply']['startedAt'])/1000 for a,b in zip(moves,moves[1:])],rejection_diagnostics=rejection_details,ros_comparison=comparisons,settling=settle)
(P/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
