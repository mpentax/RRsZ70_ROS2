"""Trial file handling only; no robot access."""
import sys,json,time,shutil,math
from pathlib import Path
from datetime import datetime,timezone
P=Path(__file__).resolve().parent
cmd=sys.argv[1]
def append(name,row):
    with (P/name).open('a') as f: f.write(json.dumps(row)+'\n')
if cmd=='frame':
    src,name,received=sys.argv[2:5]
    shutil.copy2(src,P/name)
    append('frames.jsonl',dict(file=name,receivedAt=int(received),saved_at_ns=time.time_ns(),exposure_time=None))
    if len(sys.argv)>5: time.sleep(float(sys.argv[5]))
elif cmd=='prepare':
    method,params=sys.argv[2:4]
    data=dict(timestamp=datetime.now(timezone.utc).isoformat(),request_at_ns=time.time_ns(),method=method,params=json.loads(params))
    if method=='app_rc_move': data['seqnum']=data['params'][0]['seqnum']
    (P/'pending-rpc.json').write_text(json.dumps(data))
elif cmd=='reply':
    d=json.loads((P/'pending-rpc.json').read_text()); d['reply']=json.loads(sys.argv[2]); d['reply_at_ns']=time.time_ns(); append('rpc.jsonl',d)
    if len(sys.argv)>3: time.sleep(float(sys.argv[3]))
elif cmd=='compare':
    ident=sys.argv[2]
    rpc=[json.loads(l) for l in (P/'rpc.jsonl').read_text().splitlines()]
    pairs=json.loads((P/'pairs.json').read_text())
    move=[r for r in rpc if r['method']=='app_rc_move'][next(i for i,p in enumerate(pairs) if p['id']==ident)]
    frames=[json.loads(l) for l in (P/'frames.jsonl').read_text().splitlines()]
    end=next(r['receivedAt']*1000000 for r in frames if r['file']==ident+'-after-2.jpg')
    ros=[json.loads(l) for l in (P/'ros-odom.jsonl').read_text().splitlines()]
    od=[r for r in ros if r['topic']=='/odom']
    a=[r for r in od if r['received_at_ns']<move['reply']['startedAt']*1000000][-1]
    b=[r for r in od if r['received_at_ns']<=end][-1]
    def pose(r):
        p=r['data']['pose']['pose']; q=p['orientation']; return p['position']['x'],p['position']['y'],math.atan2(2*q['w']*q['z'],1-2*q['z']**2)
    x,y,t=pose(a); X,Y,T=pose(b)
    status=[r for r in ros if r['topic']=='/z70/odom_status' and r['received_at_ns']<=end][-1]
    row=dict(id=ident,pre_receipt_ns=a['received_at_ns'],post_receipt_ns=b['received_at_ns'],dx_m=math.cos(t)*(X-x)+math.sin(t)*(Y-y),dy_m=-math.sin(t)*(X-x)+math.cos(t)*(Y-y),yaw_deg=math.degrees(math.atan2(math.sin(T-t),math.cos(T-t))),live_status=status['data'],camera_x_m=.16,window='last ROS receipt before move request to last receipt before second after-frame receipt')
    path=P/'ros-comparison.jsonl'
    previous=[json.loads(l) for l in path.read_text().splitlines()] if path.exists() else []
    updated=[r for r in previous if r['id']!=ident]+[row]
    order={r['id']:i for i,r in enumerate(pairs)}
    updated.sort(key=lambda r:order[r['id']])
    path.write_text(''.join(json.dumps(r)+'\n' for r in updated)); print(json.dumps(row))
