"""Residual diagnostics from frozen real observations and refitted pure axes."""
import json
from pathlib import Path
import sys
import cv2
import numpy as np
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT/'calibration/arm-calibration/physical_base'))
from geometry import predict_pose
from observations import intrinsics
from analyze import record, summarize
SESSION = ROOT/'calibration/arm-calibration/20261015T061415Z-physical-base'
obs = json.loads((SESSION/'observations.json').read_text())['frames']
by = {o['file']: o for o in obs}
groups = json.loads((SESSION/'groups.json').read_text())
fits = json.loads((HERE/'pincher-fits.json').read_text())['fits']
K, d = intrinsics('gripper')
output = {'pincher': {}}
for key, fit in fits.items():
    errs = []; pixels = []; deltas = []
    available = [f for f in groups[key] if by[f]['board']]
    frames = []
    for filename, angle in zip(available, fit['latent_angles_rad']):
        r = record(by[filename]); pred = predict_pose(fit, angle)
        px = r['px']
        uv = cv2.projectPoints((r['obj']-pred[:3, 3])@pred[:3, :3], np.zeros(3), np.zeros(3), K, d)[0].reshape(-1, 2)
        delta = uv-px; e = np.linalg.norm(delta, axis=1)
        errs.extend(e); pixels.extend(px); deltas.extend(delta)
        frames.append({'file': filename, 'error_px': summarize(e), 'mean_vector_px': delta.mean(axis=0).tolist()})
    px = np.array(pixels); e = np.array(errs)
    radius = np.linalg.norm(px-K[:2, 2], axis=1)
    output['pincher'][key] = {'pixel_bounds': [px.min(axis=0).tolist(), px.max(axis=0).tolist()],
        'per_frame': frames, 'radius_bins_from_principal_point_px': {
        name: summarize(e[mask]) for name, mask in [('below350', radius<350), ('350to600', (radius>=350)&(radius<600)), ('above600', radius>=600)]}}
markers = [o for o in obs if o['camera']=='front' and o['marker']]
output['elevated_front_markers'] = [{'file': o['file'], 'pixel_bounds': [np.array(o['marker']['px']).min(axis=0).tolist(), np.array(o['marker']['px']).max(axis=0).tolist()],
    'branch_rms_px': [b['rms_px'] for b in o['marker']['branches']]} for o in markers]
(HERE/'residual-diagnostics.json').write_text(json.dumps(output, indent=2))
print(json.dumps(output, indent=2))
