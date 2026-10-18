"""Offline session logging, model cross-check and frame analysis. No robot access."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
spec = importlib.util.spec_from_file_location('marker_analysis', BASE / 'analyze.py')
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
model_path = BASE.parents[1] / 'ros2/src/z70_description/z70_description/arm_model.py'
spec = importlib.util.spec_from_file_location('arm_crosscheck', model_path)
m = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = m
spec.loader.exec_module(m)
CAL = json.loads(a.CALIBRATION.read_text())

def log(value):
    with (HERE / 'log.jsonl').open('a') as f:
        f.write(json.dumps(value) + '\n')

p = argparse.ArgumentParser()
p.add_argument('action', choices=['log', 'estimate', 'frame', 'pair', 'check'])
p.add_argument('value')
p.add_argument('--label')
p.add_argument('--second')
p.add_argument('--roi', nargs=4, type=int, default=[0, 0, 1600, 520])
args = p.parse_args()
if args.action == 'log':
    log(json.loads(args.value))
elif args.action == 'estimate':
    e = json.loads(args.value)
    (HERE / 'latest-estimate.json').write_text(json.dumps(e, indent=2) + '\n')
    log({'event': 'estimate', 'estimate': e, 'label': args.label})
    print(json.dumps(e))
elif args.action == 'frame':
    v = json.loads(args.value)
    dest = HERE / (args.label + '.jpg')
    shutil.copy2(v['imagePath'], dest)
    v['retainedFile'] = str(dest)
    log({'event': 'frame', 'frame': v, 'label': args.label})
    im, detections = a.detect(dest, CAL, args.roi, 0.015, False)
    for d in detections:
        pts = np.round(d['corners_px']).astype(np.int32)
        cv2.polylines(im, [pts], True, (0, 255, 0), 2)
        cv2.putText(im, str(d['id']), tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
    cv2.imwrite(str(HERE / (args.label + '-overlay.jpg')), im)
    out = {'file': str(dest), 'detections': detections, 'roi': args.roi, 'metric_pose_computed': False}
    (HERE / (args.label + '-detections.json')).write_text(json.dumps(out, indent=2) + '\n')
    print(json.dumps(out))
elif args.action == 'pair':
    f1, f2 = HERE / args.value, HERE / args.second
    im1, d1 = a.detect(f1, CAL, args.roi, 0.015)
    im2, d2 = a.detect(f2, CAL, args.roi, 0.015)
    out = a.analyse_pair(d1, d2, 0)
    out.update({'frame1': str(f1), 'frame2': str(f2), 'markers_first': d1, 'markers_second': d2, 'roi': args.roi, 'marker_side_m': .015, 'marker_to_wrist_offset_assumed': False})
    d1id = [d for d in d1 if d['id'] == 0]
    d2id = [d for d in d2 if d['id'] == 0]
    if len(d1id) == len(d2id) == 1 and d1id[0]['pose'] and d2id[0]['pose']:
        out['stationary_angle_difference_deg'] = a.relative_angle(d1id[0]['pose']['rvec'], d2id[0]['pose']['rvec'])
        out['max_reprojection_rms_px'] = max(d['pose']['reprojection_rms_px'] for d in (d1id[0], d2id[0]))
    for d in d2:
        pts = np.round(d['corners_px']).astype(np.int32)
        cv2.polylines(im2, [pts], True, (0, 255, 0), 2)
        cv2.putText(im2, str(d['id']), tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
    cv2.imwrite(str(HERE / (args.label + '-overlay.jpg')), im2)
    out['estimate'] = json.loads((HERE / 'latest-estimate.json').read_text())
    (HERE / (args.label + '-analysis.json')).write_text(json.dumps(out, indent=2) + '\n')
    log({'event': 'pair_analysis', 'label': args.label, 'analysis': out})
    compact = {k: out.get(k) for k in ['status', 'accepted', 'centre_shift_px', 'stationary_angle_difference_deg', 'max_reprojection_rms_px']}
    compact['label'] = args.label
    compact['marker'] = out.get('marker')
    print(json.dumps(compact))
elif args.action == 'check':
    e = json.loads((HERE / 'latest-estimate.json').read_text())
    if e['state'] in ['UNKNOWN', 'IN_ROBOT']:
        raise SystemExit('No jog check from unknown or stowed pose')
    s = m.ArmState(state=e['state'], **e['joints'])
    v = json.loads(args.value)
    q = copy.deepcopy(s)
    if v['method'] == 'app_arm_direction_move':
        axis, direction = {1: ('y', 1), 2: ('y', -1), 3: ('x', 1), 4: ('x', -1)}[v['params']['direction']]
        fault = q.apply('ARM_MOVE', axis=axis, direction=direction)
        back = copy.deepcopy(q)
        return_fault = back.apply('ARM_MOVE', axis=axis, direction=-direction)
    elif v['method'] == 'app_arm_move':
        active = [(i, x) for i, x in enumerate(v['params']['m']) if x]
        if len(active) != 1:
            raise SystemExit('Require exactly one joint')
        i, x = active[0]
        joint, direction = 'm' + str(i + 3), {1: 1, 2: -1}[x]
        fault = q.apply('ARM_JOINTS', joint=joint, direction=direction)
        back = copy.deepcopy(q)
        return_fault = back.apply('ARM_JOINTS', joint=joint, direction=-direction)
    else:
        raise SystemExit('Unsupported check')
    out = {'event': 'offline_precheck', 'command': v, 'start_estimate': e, 'check': fault, 'return_check': return_fault, 'endpoint_joints': {k: getattr(q, k) for k in ['m3', 'm4', 'm5', 'm6']}, 'endpoint_wrist_m': m.forward(q.m3, q.m4)[1], 'crosscheck_only': True}
    log(out)
    print(json.dumps(out))
    if fault != 'SUCCESS' or return_fault != 'SUCCESS':
        raise SystemExit(2)
