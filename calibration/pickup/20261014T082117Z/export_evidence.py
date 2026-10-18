"""Export only this run's robot/vision tool evidence from local transcript.
No robot access. No credentials or unrelated session data are read/exported.
"""
import collections
import datetime
import json
import math
import pathlib
import sqlite3

ROOT=pathlib.Path(__file__).resolve().parent
SID='20261014_112106_adb37d'
db=sqlite3.connect('file:/Users/mpentax/.hermes/state.db?mode=ro',uri=True)
db.row_factory=sqlite3.Row
rows=db.execute('SELECT id,role,content,tool_calls,tool_name,timestamp FROM messages WHERE session_id=? ORDER BY id',(SID,)).fetchall()
existing=[]
if (ROOT/'log.jsonl').exists():
    existing=[json.loads(x) for x in (ROOT/'log.jsonl').read_text().splitlines() if x.strip()]
seen={x.get('transcript_key') for x in existing}
records=[]
methods=collections.Counter()
def flatten(calls):
    for call in calls:
        fn=call.get('function',{})
        name=fn.get('name','')
        args=fn.get('arguments',{})
        if isinstance(args,str):
            args=json.loads(args)
        if name=='tool_call':
            for idx, nested in enumerate(args['calls']):
                yield {'id':f"{call.get('id')}:{idx}", 'function':nested}
        elif name=='parallel':
            nested_calls=[{'id':f"{call.get('id')}:{idx}", 'function':{'name':nested['recipient_name'].split('.')[-1], 'arguments':nested['parameters']}} for idx,nested in enumerate(args['tool_uses'])]
            yield from flatten(nested_calls)
        else:
            yield call

for row in rows:
    ts=datetime.datetime.fromtimestamp(row['timestamp'],datetime.timezone.utc).isoformat()
    if row['tool_calls']:
        for call in flatten(json.loads(row['tool_calls'])):
            fn=call.get('function',{})
            name=fn.get('name','')
            if not (name.startswith('mcp__roboclaw__') or name=='vision_analyze'):
                continue
            args=fn.get('arguments',{})
            if isinstance(args,str):
                args=json.loads(args)
            methods[args.get('method',name)]+=1
            key=f"{row['id']}:{call.get('id')}"
            if key not in seen:
                records.append({'kind':'exact_tool_request','timestamp':ts,'transcript_key':key,'tool':name,'arguments':args,'call_id':call.get('id')})
    name=row['tool_name'] or ''
    if row['role']=='tool' and (name.startswith('mcp__roboclaw__') or name=='vision_analyze'):
        key=f"{row['id']}:response"
        if key not in seen:
            records.append({'kind':'exact_tool_response','timestamp':ts,'transcript_key':key,'tool':name,'content':row['content']})
    if row['role']=='assistant' and row['content'] and not row['tool_calls']:
        key=f"{row['id']}:commentary"
        if key not in seen:
            records.append({'kind':'operator_commentary','timestamp':ts,'transcript_key':key,'content':row['content']})
with (ROOT/'log.jsonl').open('a') as f:
    for rec in records:
        f.write(json.dumps(rec,separators=(',',':'))+'\n')
summary={
    'session':SID,'method_counts':dict(methods),'grasp_attempts':methods['app_grip'],
    'vendor_autonomous_pickup_calls':methods['app_pick_up'],
    'forward_image_pulse2_mm':262.47081670377264-227.2536297254597,
    'forward_image_pulse3_mm':227.2536297254597-151.42500053177423,
    'forward_gain_pulse2':(262.47081670377264-227.2536297254597)/50,
    'forward_gain_pulse3':(227.2536297254597-151.42500053177423)/100,
    'forward_gain_combined':(262.47081670377264-151.42500053177423)/150,
    'yaw_observed_deg':math.degrees(2*math.atan2(.09775447933098308,.9952105615249108)-2*math.atan2(.00038603262829655966,.9999999254894022)),
    'hold_ms':1791966928018-1791966769983,
    'yellow_width_estimate_mm':375*151.42500053177423/823,
    'purple_width_estimate_mm':267*208.44174974806043/823,
    'frames_retained':len(list(ROOT.glob('*.jpg'))),
    'appended_exact_records':len(records),'log_records':len(existing)+len(records),
    'note':'size rough pinhole estimate, not object measurement; image-derived gain not full-pose calibrated; exact records appended by transcript order, upstream receipt times not exposures'
}
(ROOT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
assert summary['grasp_attempts']==2
assert summary['vendor_autonomous_pickup_calls']==0
assert summary['frames_retained']==33
