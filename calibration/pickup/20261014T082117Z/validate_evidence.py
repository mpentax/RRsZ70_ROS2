"""Validate retained evidence and attempt accounting, no robot access."""
import collections
import json
import pathlib
import subprocess

root=pathlib.Path(__file__).resolve().parent
records=[json.loads(x) for x in (root/'log.jsonl').read_text().splitlines() if x.strip()]
requests=[r for r in records if r['kind']=='exact_tool_request']
responses=[r for r in records if r['kind']=='exact_tool_response']
req=collections.Counter(r['tool'] for r in requests)
res=collections.Counter(r['tool'] for r in responses)
assert req==res, {'requests':dict(req),'responses':dict(res)}
keys=[r['transcript_key'] for r in records if 'transcript_key' in r]
assert len(keys)==len(set(keys))
rpcs=[r['arguments'] for r in requests if r['tool']=='mcp__roboclaw__robot_rpc']
assert sum(r['method']=='app_grip' for r in rpcs)==2
assert not any(r['method']=='app_pick_up' for r in rpcs)
assert len(list(root.glob('*.jpg')))==33
for script in root.glob('*.py'):
    compile(script.read_text(),str(script),'exec')
repo=root.parents[2]
staged=subprocess.check_output(['git','diff','--cached','--name-only'],cwd=repo,text=True).splitlines()
assert not any(p.lower().endswith(('.jpg','.jpeg','.png')) for p in staged)
print(json.dumps({'valid':True,'records':len(records),'matched_tool_requests_and_responses':sum(req.values()),'grasp_attempts':2,'local_ignored_frames':33,'scripts_compiled':len(list(root.glob('*.py'))),'staged_files':staged},indent=2))
