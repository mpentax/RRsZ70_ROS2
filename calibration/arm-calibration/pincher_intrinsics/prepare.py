"""Regenerate independent raw board corners from retained MCP log/JPEGs."""
import argparse,json,hashlib
from pathlib import Path
from calibrate import detect_board

def main():
    ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);args=ap.parse_args();D=args.session.resolve()
    if not D.name.endswith('-pincher-intrinsics-arm'):raise ValueError('Use a newly owned pincher-intrinsics-arm session, never an original baseline archive')
    rows=[json.loads(line) for line in (D/'log.jsonl').read_text().splitlines()];accepted=[];allobs=[]
    for row in rows:
        if row.get('event')!='capture' or row.get('camera')!='gripper':continue
        file=D/row['file'];o=detect_board(file);o['file']=str(file.relative_to(D.parents[2]));o['nominal']=row.get('tracked');allobs.append(o)
        if len(o['px'])>=12:accepted.append(o)
    provenance={'detector_sha256':hashlib.sha256(Path(__file__).with_name('calibrate.py').read_bytes()).hexdigest(),'capture_log_sha256':hashlib.sha256((D/'log.jsonl').read_bytes()).hexdigest(),'board':{'size':[10,14],'square_m':.02,'black_marker_m':.015,'legacy':True,'dictionary':'DICT_4X4_1000'},'all_frame_sha256':{o['file']:o['sha256'] for o in allobs},'selection':'all raw Charuco corners, min12 per view; no arm FK, nominal joint angles, or lens-dependent corner selection','exposure_timestamps_unknown':True}
    provenance['cache_key']=hashlib.sha256(json.dumps(provenance,sort_keys=True).encode()).hexdigest()
    (D/'intrinsics-observations.json').write_text(json.dumps(accepted));(D/'all-pincher-observations.json').write_text(json.dumps(allobs));(D/'observation-provenance.json').write_text(json.dumps(provenance,indent=2))
    print(json.dumps({'MCP_gripper_frames':len(allobs),'board_views_min12':len(accepted),'rejected_no_usable_board':len(allobs)-len(accepted),'cache_key':provenance['cache_key']}))

if __name__=='__main__':main()
