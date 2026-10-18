"""Rehash completed retry evidence without touching either previous archive."""
from pathlib import Path
import json,hashlib
P=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
prior=json.loads((P/'prior-source-sha256.json').read_text())
assert all(sha(P.parents[2]/n)==h for n,h in prior.items())
logs=[json.loads(x) for x in (P/'log.jsonl').read_text().splitlines()]
frames=[x for x in logs if x.get('event')=='frame']
assert all(sha(P/x['file'])==x['sha256'] for x in frames)
s=json.loads((P/'final-state.json').read_text());e=json.loads((P/'final-estimate.json').read_text())
assert s['connection']['state']=='connected' and s['telemetry']['status']['value']=={'battery':50,'state':3,'error_code':0,'in_cleaning':0,'in_returning':0}
assert e['state']=='IN_ROBOT' and s['telemetry']['camera']['value']==[11685]
assert all(not x['consumers'] for x in s['camera']['streamDiagnostics'].values())
assert any(x.get('args',{}).get('method')=='app_get_program_runtime' and x['result']['result']['program_state']==0 for x in logs)
assert len(frames)==62 and len({x['sha256'] for x in frames})==62
assert sum(x.get('args',{}).get('method')=='app_arm_move' for x in logs)==35
assert 'Ran 4 tests' in (P/'tests.txt').read_text() and '\nOK\n' in (P/'tests.txt').read_text()
verification={'prior_files_checked':len(prior),'prior_mismatches':[],'original_images_checked':len(frames),'unique_JPEG_hashes':62,'image_mismatches':[],'all_hashes_ok':True,'cleanup_readbacks_verified':True,'physical_stow_evidence':'final front image absence corroborates stow estimate and mode0/error0 idle status; not inferred from [0,0,0] feedback','tests_passed':4}
(P/'verification.json').write_text(json.dumps(verification,indent=2))
manifest={str(x.relative_to(P)):sha(x) for x in P.rglob('*') if x.is_file() and x.name!='artifact-sha256.json'}
(P/'artifact-sha256.json').write_text(json.dumps(manifest,indent=2))
assert all(sha(P/n)==h for n,h in manifest.items())
print(json.dumps({**verification,'current_artifacts_hashed':len(manifest)},indent=2))
