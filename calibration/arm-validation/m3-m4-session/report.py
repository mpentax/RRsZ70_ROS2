"""Reproduce M3/M4 results from retained observations; never contacts robot."""
import csv
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys
import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir', type=Path, default=HERE)
args = parser.parse_args()
OUT = args.output_dir.resolve()
OUT.mkdir(parents=True, exist_ok=True)
log = [json.loads(line) for line in (HERE / 'log.jsonl').read_text().splitlines() if line.strip()]
spec = importlib.util.spec_from_file_location('arm_model_crosscheck', BASE.parents[1] / 'src/z70_description/z70_description/arm_model.py')
assert spec and spec.loader
model = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = model
spec.loader.exec_module(model)

def load(label):
    v = json.loads((HERE / (label + '-analysis.json')).read_text())
    assert v['accepted'], label
    first = [x for x in v['markers_first'] if x['id'] == 0]
    second = [x for x in v['markers_second'] if x['id'] == 0]
    assert len(first) == len(second) == 1
    x, y = first[0], second[0]
    r0 = cv2.Rodrigues(np.array(x['rvec']))[0]
    r1 = cv2.Rodrigues(np.array(y['rvec']))[0]
    dr = cv2.Rodrigues(r1 @ r0.T)[0]
    v['R'] = cv2.Rodrigues(.5 * dr)[0] @ r0
    v['position'] = .5 * (np.array(x['tvec_m']) + np.array(y['tvec_m']))
    v['centre'] = .5 * (np.array(x['centre_px']) + np.array(y['centre_px']))
    v['label'] = label
    return v

m4labels = ['noise3', 'm4minus1', 'm4return1', 'm4plus1', 'm4return2', 'm4minus2', 'm4return3', 'm4plus2', 'm4return4']
m3labels = ['m3noise3', 'm3plus1', 'm3return1', 'm3plus2', 'm3return2']
poses = {label: load(label) for label in set(m4labels + m3labels)}
# Fix the orientation comparison axis from the FIRST positive M4 excursion only.
axis_vec = cv2.Rodrigues(poses['m4plus1']['R'] @ poses['noise3']['R'].T)[0].ravel()
axis = axis_vec / np.linalg.norm(axis_vec)
rows = []
for joint, labels in [('m4', m4labels), ('m3', m3labels)]:
    for before, after in zip(labels, labels[1:]):
        p, q = poses[before], poses[after]
        rv = cv2.Rodrigues(q['R'] @ p['R'].T)[0].ravel()
        signed_forearm = float(np.degrees(rv @ axis))
        delta_q = q['estimate']['joints'][joint] - p['estimate']['joints'][joint]
        measured_joint_proxy = signed_forearm if joint == 'm4' else -signed_forearm
        rows.append({'joint': joint, 'before': before, 'after': after, 'command_sign': '+' if delta_q > 0 else '-',
                     'estimator_joint_delta_deg': delta_q, 'estimator_forearm_delta_deg': delta_q if joint == 'm4' else -delta_q,
                     'measured_forearm_delta_deg': signed_forearm, 'measured_joint_proxy_delta_deg': measured_joint_proxy,
                     'rotation_magnitude_deg': float(np.degrees(np.linalg.norm(rv))),
                     'cross_axis_rotation_deg': float(np.degrees(np.linalg.norm(rv - axis * (rv @ axis)))),
                     'marker_displacement_camera_mm': ((q['position'] - p['position']) * 1000).tolist(),
                     'marker_displacement_image_px': (q['centre'] - p['centre']).tolist(),
                     'stationary_pair_angle_deg': q['stationary_angle_difference_deg'],
                     'centre_shift_px': q['centre_shift_px'], 'pose_rms_px': q['max_reprojection_rms_px'],
                     'sigmaDeg': q['estimate']['sigmaDeg'], 'estimator_state': q['estimate']['state']})
assert len(rows) == 12
summary = []
for joint in ['m4', 'm3']:
    for sign in ['+', '-']:
        group = [r for r in rows if r['joint'] == joint and r['command_sign'] == sign]
        values = [r['measured_joint_proxy_delta_deg'] for r in group]
        summary.append({'joint': joint.upper(), 'sign': sign, 'n': len(group),
                        'predicted_joint_increment_deg': 10 if sign == '+' else -10,
                        'measured_joint_proxy_mean_deg': float(np.mean(values)),
                        'sample_sd_deg': float(np.std(values, ddof=1)),
                        'minimum_deg': float(min(values)), 'maximum_deg': float(max(values))})

noise = {}
for joint, labels in [('M4', ['noise1', 'noise2', 'noise3']), ('M3', ['m3noise1', 'm3noise2', 'm3noise3'])]:
    n = [json.loads((HERE / (x + '-analysis.json')).read_text()) for x in labels]
    assert len(n) == 3 and all(x['accepted'] for x in n)
    noise[joint] = {'angular_differences_deg': [x['stationary_angle_difference_deg'] for x in n],
                    'centre_shifts_px': [x['centre_shift_px'] for x in n],
                    'max_fit_rms_px': max(x['max_reprojection_rms_px'] for x in n),
                    'overlapping_pairs': True, 'unique_frames': 4, 'independent_pairs': False}
returns = []
for joint, ref, labels in [('M4', 'noise3', ['m4return1', 'm4return2', 'm4return3', 'm4return4']),
                            ('M3', 'm3noise3', ['m3return1', 'm3return2'])]:
    for label in labels:
        p, q = poses[ref], poses[label]
        rv = cv2.Rodrigues(q['R'] @ p['R'].T)[0].ravel()
        returns.append({'joint': joint, 'pose': label, 'marker_centre_difference_px': float(np.linalg.norm(q['centre'] - p['centre'])),
                        'marker_position_difference_mm': float(np.linalg.norm(q['position'] - p['position']) * 1000),
                        'orientation_difference_deg': float(np.degrees(np.linalg.norm(rv))),
                        'signed_forearm_difference_deg': float(np.degrees(rv @ axis))})

excursions = []
for joint, labels, nominal in [
    ('M4', m4labels[:5], 20), ('M4', m4labels[4:], 20),
    ('M3', m3labels[:3], 10), ('M3', m3labels[2:], 10),
]:
    reference = poses[labels[0]]['R']
    values = []
    for label in labels:
        rv = cv2.Rodrigues(poses[label]['R'] @ reference.T)[0].ravel()
        values.append(float(np.degrees(rv @ axis)) * (1 if joint == 'M4' else -1))
    excursions.append({'joint': joint, 'poses': labels, 'nominal_peak_to_peak_deg': nominal,
                       'measured_joint_proxy_from_cycle_reference_deg': values,
                       'measured_peak_to_peak_deg': max(values) - min(values)})

# Fit a SINGLE local, 2-D camera-YZ registration and a nonzero unknown marker-to-wrist offset.
# The lateral offset is NOT assumed zero: it is inseparable from camera translation and is not fitted.
training = ['noise3', 'm4minus1', 'm4plus1', 'm3noise3', 'm3plus1']
heldout = ['m4minus2', 'm4return3', 'm4plus2', 'm4return4', 'm3plus2', 'm3return2']

def body_marker(p, u, v):
    j = p['estimate']['joints']
    wrist = np.array(model.forward(j['m3'], j['m4'])[1])
    theta = math.radians(j['m4'] - j['m3'] + 90)
    rotation = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    return wrist + rotation @ np.array([u, v])

def predict(p, params):
    phi, ty, tz, u, v = params
    rotation = np.array([[math.cos(phi), -math.sin(phi)], [math.sin(phi), math.cos(phi)]])
    return rotation @ body_marker(p, u, v) + np.array([ty, tz])

def residual(params):
    return np.concatenate([predict(poses[x], params) - poses[x]['position'][1:] for x in training])
fits = [least_squares(residual, [angle, 0, 0, 0, 0], max_nfev=2000) for angle in [-math.pi/2, 0, math.pi/2, math.pi]]
fit = min(fits, key=lambda f: np.sum(f.fun ** 2))
sv = np.linalg.svd(fit.jac, compute_uv=False)
rank = int(np.linalg.matrix_rank(fit.jac))
condition = float(sv[0] / sv[-1])
fit_valid = bool(fit.success and rank == 5 and np.isfinite(fit.x).all())
fit_info = {'label': 'Fitted local model-consistency check, not independent extrinsics or absolute wrist calibration',
            'dimensions': 'camera YZ only; lateral marker-to-wrist offset and full 3-D camera registration remain unknown',
            'training_poses': training, 'heldout_poses': heldout, 'parameters_frozen_for_heldout': True,
            'valid_local_fit': fit_valid, 'jacobian_rank': rank, 'jacobian_singular_values': sv.tolist(), 'condition_number': condition,
            'parameters': dict(zip(['plane_rotation_rad', 'camera_y_translation_m', 'camera_z_translation_m', 'fitted_marker_local_u_m', 'fitted_marker_local_v_m'], map(float, fit.x))),
            'training_rms_marker_yz_mm': float(np.sqrt(np.mean(np.sum(fit.fun.reshape(-1, 2) ** 2, axis=1))) * 1000),
            'heldout': []}
if fit_valid:
    for label in heldout:
        observed = poses[label]['position'][1:]
        pred = predict(poses[label], fit.x)
        fit_info['heldout'].append({'pose': label, 'observed_marker_yz_m': observed.tolist(), 'predicted_marker_yz_m': pred.tolist(),
                                   'error_mm': float(np.linalg.norm(pred - observed) * 1000)})
    fit_info['heldout_rms_marker_yz_mm'] = float(np.sqrt(np.mean([x['error_mm'] ** 2 for x in fit_info['heldout']])))
    fit_info['heldout_max_marker_yz_mm'] = max(x['error_mm'] for x in fit_info['heldout'])

# Local-only photographic contact sheets; images are not staged for commit.
for joint, labels in [('M4', m4labels), ('M3', m3labels)]:
    width, height, cols = 480, 380, 3
    canvas = np.full((math.ceil(len(labels) / cols) * height, cols * width, 3), 240, np.uint8)
    baseline = poses[labels[0]]['centre']
    previous = baseline
    for i, label in enumerate(labels):
        p = poses[label]
        image = cv2.imread(str(HERE / Path(p['frame2']).name))
        assert image is not None
        corners = np.round(p['markers_second'][0]['corners_px']).astype(np.int32)
        cv2.polylines(image, [corners], True, (0, 255, 0), 3)
        centre = p['centre']
        cv2.arrowedLine(image, tuple(np.round(previous).astype(int)), tuple(np.round(centre).astype(int)), (0, 0, 255), 3, tipLength=.18)
        cv2.circle(image, tuple(np.round(baseline).astype(int)), 6, (255, 0, 0), -1)
        image = cv2.resize(image, (width, 360))
        x, y = i % cols * width, i // cols * height
        canvas[y:y+360, x:x+width] = image
        cv2.putText(canvas, label + ' | settled', (x+8, y+375), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 1)
        previous = centre
    assert cv2.imwrite(str(OUT / (joint + '-contact-sheet.jpg')), canvas)

# Plain SVG plot: no additional plotting dependency.
def svg_plot(joint, labels):
    observed = np.array([poses[x]['position'][1:] * 1000 for x in labels])
    predicted = np.array([predict(poses[x], fit.x) * 1000 for x in labels]) if fit_valid else None
    points = np.vstack([observed, predicted]) if predicted is not None else observed
    lo, hi = points.min(axis=0) - 3, points.max(axis=0) + 3
    def xy(v):
        return 65 + (v[1]-lo[1])/(hi[1]-lo[1])*630, 370 - (v[0]-lo[0])/(hi[0]-lo[0])*280
    pieces = ['<svg xmlns="http://www.w3.org/2000/svg" width="780" height="450" viewBox="0 0 780 450">', '<rect width="780" height="450" fill="white"/>',
              f'<text x="30" y="28" font-family="sans-serif" font-size="18">{joint}: marker trajectory, camera YZ (mm)</text>',
              '<text x="30" y="52" font-family="sans-serif" font-size="12">Blue observed; orange frozen local model fit. Not wrist error.</text>',
              '<path d="M65 85 V370 H705" fill="none" stroke="black"/>',
              '<text x="320" y="422" font-family="sans-serif" font-size="14">Camera Z / depth (mm)</text>',
              '<text x="8" y="78" font-family="sans-serif" font-size="12">Camera Y</text>']
    for series, color in [(observed, '#1768ac'), (predicted, '#cf6300')]:
        if series is None:
            continue
        mapped = [xy(v) for v in series]
        pieces.append('<polyline points="' + ' '.join(f'{x:.2f},{y:.2f}' for x,y in mapped) + f'" fill="none" stroke="{color}" stroke-width="2"/>')
        for x,y in mapped:
            pieces.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="4" fill="{color}"/>')
    for i, v in enumerate(observed):
        x,y = xy(v)
        pieces.append(f'<text x="{x+6:.2f}" y="{y-6:.2f}" font-family="sans-serif" font-size="11">{i}</text>')
    for fraction in [0, .25, .5, .75, 1]:
        z = lo[1] + fraction * (hi[1]-lo[1]); yy = lo[0] + fraction * (hi[0]-lo[0])
        pieces.append(f'<text x="{65+fraction*630:.1f}" y="391" font-family="sans-serif" font-size="11">{z:.1f}</text>')
        pieces.append(f'<text x="25" y="{374-fraction*280:.1f}" font-family="sans-serif" font-size="11">{yy:.1f}</text>')
    pieces.append('</svg>')
    (OUT / (joint + '-trajectory.svg')).write_text('\n'.join(pieces))
svg_plot('M4', m4labels)
svg_plot('M3', m3labels)

release = [x for x in log if x.get('event') == 'robot released'][-1]
measurement_commands = [x for x in log if x.get('measurement_sample') is True]
assert len(measurement_commands) == 12
assert all(x['result'] == ['ok'] for x in measurement_commands)
for joint in ['m3', 'm4']:
    estimates = [poses[r['after']]['estimate'] for r in rows if r['joint'] == joint]
    assert all(e['state'] != 'UNKNOWN' for e in estimates)
result = {'marker': {'dictionary': 'DICT_4X4_1000', 'id': 0, 'paper_side_mm': 20, 'black_side_mm': 15,
                     'marker_to_wrist_transform': None, 'marker_to_fingertips_offset': None},
          'orientation_axis_camera_empirical': axis.tolist(), 'orientation_axis_training': 'noise3 -> m4plus1; frozen for later signed comparisons',
          'orientation_method': 'SO(3) midpoint per accepted pair; signed rotation-vector projection on frozen empirical M4+ axis. M3 joint proxy reverses sign.',
          'measurement_command_count': len(measurement_commands), 'measurement_pose_pair_count': len(rows),
          'noise': noise, 'per_joint_sign': summary, 'increments': rows, 'returns': returns, 'excursions': excursions, 'local_fit': fit_info,
          'robot_release': release, 'decision': 'Do not change vendor constants; limited local observations support direction and coarse nominal increments, not full calibration'}
(OUT / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
with (OUT / 'increments.csv').open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator='\n')
    writer.writeheader(); writer.writerows(rows)

text = ['# M4 then M3: supervised estimator check', '',
        'Outcome: direction and coarse nominal 10-degree increments supported in the tested poses. No vendor constants changed; full arm calibration remains unvalidated.', '',
        '## Scope and evidence', '',
        '- Twelve measurement jogs: eight M4 commands (two baseline → −10 → baseline → +10 → baseline cycles), then four M3 commands (two baseline → +10 → baseline cycles). Every measurement endpoint has an accepted two-frame pair.',
        '- Robot access was MCP only. One operator-authorized programming-mode entry undocked the robot; Michael confirmed separation and later manually moved it into the testing area. No chassis pulse, docking or cleaning command was sent.',
        '- Initial conversational pauses caused operator-reported inactivity stowing. Explicit reset/stow/redeploy preceded the continuous acquisition. An apparent animal identification was corrected by Michael; the false positive and restart are retained in the log.',
        '- The flat, secure arm-mounted marker is DICT_4X4_1000 ID 0: 20 mm paper square, 15 mm printed black side. Mounting unchanged per Michael. Wrist/fingertip offsets remain independently unmeasured, not zero.',
        '- Existing front-camera intrinsics were reused outside their floor-board coverage. Lens identity and exposure times remain unverified; logged timestamps are receipt times.',
        '- Evidence: `m3-m4-session/log.jsonl`, per-pair `*-analysis.json`, `increments.csv`, `summary.json`. Reproduce from the repository root with `pixi run python calibration/arm-validation/m3-m4-session/report.py --output-dir calibration/arm-validation/phase5-replay`.', '',
        '## Setup and stationary noise', '',
        '- Four forward/four down setup commands left the marker cropped. One additional model-checked down step established the M4 reference: predicted M3=245.097896°, M4=96.977279°, M5=0°, M6=90°; wrist (0.316765, 0.040662) m. This was adaptive setup, not calibrated hand-step evidence.',
        '- M3 used a separate reference after two individually checked M4 positive steps: M3=245.097896°, M4=116.977279°, M5=0°, M6=90°; predicted wrist (0.369774, 0.088189) m. Marker and fingers remained observable. This reference is very near the nominal projection bound; no further expansion was made.',
        '- At each reference, three overlapping consecutive stationary pairs used four unique frames. They are correlated comparisons, not three independent replicates. All pairs passed ≤1 px centre-shift and the separately labelled ≤2 px pose-fit RMS screen before joint probes.', '',
        '| Reference | Stationary angular differences (deg) | Centre shifts (px) | Max fit RMS (px) |',
        '|---|---|---|---|']
for j in ['M4', 'M3']:
    n = noise[j]
    text.append(f"| {j} | {', '.join(f'{x:.3f}' for x in n['angular_differences_deg'])} | {', '.join(f'{x:.3f}' for x in n['centre_shifts_px'])} | {n['max_fit_rms_px']:.3f} |")
text += ['', '## Measured versus estimator: per joint/sign', '',
         'Pair orientations are averaged on SO(3). A common camera-frame rotation axis was fixed from the first M4 positive excursion; later signed rotations are projected onto it. These are marker-based relative joint proxies, not encoder or absolute joint measurements. M3 positive produces the opposite forearm rotation to M4 positive, as predicted.', '',
         '| Joint/sign | n | Estimator joint increment (deg) | Measured joint proxy mean ± sample SD (deg) | Range (deg) |',
         '|---|---:|---:|---:|---:|']
for r in summary:
    text.append(f"| {r['joint']}{r['sign']} | {r['n']} | {r['predicted_joint_increment_deg']:+.1f} | {r['measured_joint_proxy_mean_deg']:+.3f} ± {r['sample_sd_deg']:.3f} | {r['minimum_deg']:+.3f} to {r['maximum_deg']:+.3f} |")
text += ['', '| Pose | Estimator forearm delta (deg) | Measured signed forearm delta (deg) | Stationary pair angle (deg) | Centre shift (px) | Max fit RMS (px) |',
         '|---|---:|---:|---:|---:|---:|']
for r in rows:
    text.append(f"| {r['after']} | {r['estimator_forearm_delta_deg']:+.1f} | {r['measured_forearm_delta_deg']:+.3f} | {r['stationary_pair_angle_deg']:.3f} | {r['centre_shift_px']:.3f} | {r['pose_rms_px']:.3f} |")
text += ['', '## Measured excursions by cycle', '',
         '| Joint/cycle | Nominal peak-to-peak (deg) | Measured signed-axis peak-to-peak (deg) |',
         '|---|---:|---:|']
for i, r in enumerate(excursions):
    cycle = i % 2 + 1
    text.append(f"| {r['joint']} / {cycle} | {r['nominal_peak_to_peak_deg']:.1f} | {r['measured_peak_to_peak_deg']:.3f} |")
text += ['', '## Excursion and returns', '',
         '- M4 sampled nominal −10°, 0°, +10° twice; maximum nominal end-to-end excursion was 20°. M3 sampled 0° and +10° twice; no claim of a two-sided M3 sweep.', '',
         '| Joint | Return pose | Marker centre difference (px) | Camera-frame marker position difference (mm) | Orientation difference (deg) |',
         '|---|---|---:|---:|---:|']
for r in returns:
    text.append(f"| {r['joint']} | {r['pose']} | {r['marker_centre_difference_px']:.3f} | {r['marker_position_difference_mm']:.3f} | {r['orientation_difference_deg']:.3f} |")
text += ['', 'These are marker returns, not wrist errors. Do not attribute apparent return rotation entirely to backlash: stationary angular noise is material, especially at the M4 reference.', '',
         '## Local trajectory comparison', '',
         '- A single five-parameter planar camera-YZ registration plus rotating marker-to-wrist planar offset was fitted to the designated first subset: `noise3`, `m4minus1`, `m4plus1`, `m3noise3`, `m3plus1`. It was frozen before scoring the later six listed poses. No per-pose refitting.',
         '- The local offset is fitted, not independently measured; the lateral marker offset cannot be separated from camera translation here. Full 3-D extrinsics and absolute wrist position remain unknown. Held-out samples repeat trained nominal settings, so this tests local consistency/repeatability, not unseen workspace generalization. Training includes both joint blocks and does not test transfer to M3 from M4 alone.',
         f"- Local fit Jacobian rank: {rank}/5; condition number: {condition:.3f}. Training marker-YZ RMS: {fit_info['training_rms_marker_yz_mm']:.3f} mm."]
if fit_valid:
    text += [f"- Frozen-fit held-out marker-YZ RMS: {fit_info['heldout_rms_marker_yz_mm']:.3f} mm; maximum: {fit_info['heldout_max_marker_yz_mm']:.3f} mm. These are fitted local marker residuals, NOT measured wrist accuracy.", '',
             '| Held-out pose | Marker-YZ residual (mm) |', '|---|---:|']
    for r in fit_info['heldout']:
        text.append(f"| {r['pose']} | {r['error_mm']:.3f} |")
text += ['', 'Local-only figures (not committed): `m3-m4-session/M4-contact-sheet.jpg`, `M3-contact-sheet.jpg`, `M4-trajectory.svg`, `M3-trajectory.svg`. Contact sheets draw image-space arrows from preceding poses and blue reference centres. Trajectory plots compare observed markers with the one frozen local model fit.', '',
         '## Estimator end-to-end observations and limitations', '',
         '- The estimator applied each observed command sign and held the other joints fixed during each block. States stayed IN_PROCESS_POS throughout measurement; no UNKNOWN state or reported model fault occurred. Estimates agreed with the local Python model cross-check numerically.',
         '- Before final stow, sigmaDeg reached M3=9.013878°, M4=10.897247°, M5=0°. These uncertainty values are heuristic and not validated against a joint sensor; the estimator correctly accumulates uncertainty even after predicted return to the same joints.',
         '- Autonomous inactivity stowing during the early pauses was not reflected in the dead-reckoned state until operator confirmation/reset. This is an estimator coverage gap: outside-command physical changes remain unobserved. Do not carry the old estimate across inactivity without physical checking.',
         '- M3 negative outward motion from the higher visible reference was rejected offline by FAULT_PROJECTION. The negative M3 measurements are returns from +10°, not an independently sampled −10° endpoint.',
         '- No ±20° M3 sweep, broad workspace, link lengths, absolute wrist pose, gripper-length envelope, hand-step size, independent camera extrinsics or statistical uncertainty calibration was validated.',
         '- Decision: retain vendor constants unchanged. Relative directions and coarse 10° increments are supported locally; precision correction is not justified by this coverage and camera uncertainty.', '',
         '## Cleanup and handoff', '',
         '- Stow and manual-exit RPCs returned `ok`. Fresh status: idle (3), battery 81%, error_code 0. Michael then confirmed physical full stow in the confirmation form.',
         '- `set_arm_anchor {anchor:"stowed"}` was sent only after that confirmation; exact readback was IN_ROBOT, joints all zero, wrist null, sigmaDeg all zero, stepsSinceAnchor 0.',
         f"- **robot released — {release['timestamp']}**. Physically stowed and operator-confirmed; manual mode exited; confirmed-stow estimator anchor read back. No further robot commands planned."]
(OUT / 'M3_M4_RESULTS.md').write_text('\n'.join(text) + '\n')
print(json.dumps({'per_joint_sign': summary, 'noise': noise, 'returns': returns, 'local_fit': fit_info, 'robot_release': release}, indent=2))
