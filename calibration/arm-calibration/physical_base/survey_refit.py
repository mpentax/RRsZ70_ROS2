"""Offline fixed-intrinsics front-board height refit; never writes source archive.

A floor-plane constraint is imposed at the board-pixel objective, not by adding
height to an axis report. Rough owner axis/body measurements are checks only.
"""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from analyze import in_front, record, summarize
from observations import intrinsics, ROOT
from geometry import fit_axis, fit_circle_axis, fit_pixel_axis, predict_pose


def file_hashes(paths):
    return {str(Path(p).resolve()): hashlib.sha256(Path(p).read_bytes()).hexdigest()
            for p in sorted(paths, key=str)}


def provenance_key(paths, K, d, height):
    """No count-only cache. Scenario and all supplied content affect identity."""
    payload = {'files': file_hashes(paths), 'K': np.asarray(K).tolist(),
               'd': np.asarray(d).tolist(), 'height': height,
               'implementation': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def evaluate_front(T, records, K, d):
    r = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    rows = []
    for o in records:
        b = o['board']; X = np.array(b['obj']); px = np.array(b['px'])
        uv = cv2.projectPoints(X, r, T[:3, 3], K, d)[0].reshape(-1, 2)
        delta = uv - px
        rows.append({'file': o['file'], 'point_errors_px': summarize(np.linalg.norm(delta, axis=1)),
                     'mean_vector_px': delta.mean(axis=0).tolist(),
                     'observed_pixel_bounds': [px.min(axis=0).tolist(), px.max(axis=0).tolist()]})
    return rows


def fit_front(records, K, d, height=None, initial_T=None):
    """Equal-frame robust registration, all frozen full corners, optional exact h.

    R maps board to optical; C is optical centre in board coordinates, t=-R C.
    Fix C_z=sign(initial C_z)*h; R and in-plane C remain genuinely optimized.
    Height is a scenario, not a measured optical-origin accuracy guarantee.
    """
    Ts = [np.array(initial_T)] if initial_T is not None else [np.array(o['board']['T_cam_board']) for o in records]
    R0 = Rotation.from_matrix([T[:3, :3] for T in Ts]).mean().as_rotvec()
    C0 = np.median([np.linalg.inv(T)[:3, 3] for T in Ts], axis=0)
    sign = 1 if C0[2] > 0 else -1
    if height is not None and height <= 0:
        raise ValueError('height must be positive')
    def unpack(x):
        R = Rotation.from_rotvec(x[:3]).as_matrix()
        C = .1*x[3:] if height is None else np.r_[.1*x[3:5], sign*height]
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = -R@C
        return T
    def residual(x):
        T = unpack(x); out = []
        for o in records:
            b = o['board']; X = np.array(b['obj']); px = np.array(b['px'])
            uv = cv2.projectPoints(X, x[:3], T[:3, 3], K, d)[0].reshape(-1, 2)
            out.extend(((uv-px)/np.sqrt(len(X))).ravel())
        return np.array(out)
    x0 = np.r_[R0, C0/.1] if height is None else np.r_[R0, C0[:2]/.1]
    sol = least_squares(residual, x0, loss='soft_l1', f_scale=.5, max_nfev=500,
                        xtol=1e-11, ftol=1e-11, gtol=1e-11)
    T = unpack(sol.x); errs = []
    for o in records:
        b = o['board']; X = np.array(b['obj']); px = np.array(b['px'])
        uv = cv2.projectPoints(X, sol.x[:3], T[:3, 3], K, d)[0].reshape(-1, 2)
        if np.min((X@T[:3, :3].T+T[:3, 3])[:, 2]) <= 0:
            raise ValueError('nonpositive-depth front solution')
        errs.extend(np.linalg.norm(uv-px, axis=1))
    sv = np.linalg.svd(sol.jac, compute_uv=False)
    # Local linear covariance is conditional on independent residual coordinates,
    # and therefore not an absolute metrological uncertainty interval.
    cov = np.linalg.pinv(sol.jac.T@sol.jac) * (2*sol.cost/max(1, len(sol.fun)-len(sol.x)))
    result = {'T_F_W': T.tolist(), 'height_m': float(abs(np.linalg.inv(T)[2, 3])),
            'fixed_height_m': height, 'point_errors_px': summarize(errs),
            'frame_residuals': evaluate_front(T, records, K, d),
            'robust_cost': float(sol.cost), 'success': bool(sol.success),
            'jacobian_singular_values': sv.tolist(),
            'rank': int(np.sum(sv > sv[0]*1e-6)), 'parameter_count': len(sol.x),
            'conditional_local_covariance': cov.tolist(),
            'parameter_scaling': 'rotation radians, board-frame camera centre /0.1m',
            'uncertainty_excludes': ['lens/optical-origin offset', 'print scale/flatness', 'correlated corner errors']}
    if initial_T is not None:
        default = fit_front(records, K, d, height)
        costs = {'default_seed_cost': default['robust_cost'], 'alternative_seed_cost': result['robust_cost'],
                 'selection': 'lower TRAINING objective; no heldout pixels consulted'}
        result = min([default, result], key=lambda item: item['robust_cost'])
        result['branch_start_check'] = costs
    return result


def run(session, output, heights=(None, .055, .058, .060, .062, .065), bootstrap=100, lens=True):
    session = Path(session).resolve(); output = Path(output).resolve()
    if output == session or session in output.parents or output in session.parents:
        raise ValueError('output must be separate from the retained source archive')
    source_files = [p for p in session.rglob('*') if p.is_file()]
    before = file_hashes(source_files)
    output.mkdir(parents=True, exist_ok=True)
    (output/'archive-before-sha256.json').write_text(json.dumps(before, indent=2))
    old = json.loads((session/'fit.json').read_text())
    validation = json.loads((session/'validation.json').read_text())
    observations = json.loads((session/'observations.json').read_text())['frames']
    by = {o['file']: o for o in observations}
    groups = json.loads((session/'groups.json').read_text())
    manifest = json.loads((session/'manifest.json').read_text())
    for item in manifest:
        if before[str(session/item['file'])] != item['sha256']:
            raise ValueError('raw original disagrees with manifest: '+item['file'])
    survey_path = ROOT/'calibration/physical-measurements/20261014-owner-tape/owner-survey-20261015.json'
    survey = json.loads(survey_path.read_text())
    code = [Path(__file__), Path(__file__).with_name('geometry.py'),
            Path(__file__).with_name('observations.py'), Path(__file__).with_name('analyze.py')]
    external = [survey_path, ROOT/'calibration/front-camera/front_camera.json',
                ROOT/'calibration/arm-calibration/20261014T090217Z/arm_calibration.json'] + code
    inputs = source_files + external
    Kg, dg = intrinsics('gripper')
    front_records = {b: [by[f] for f in v['frames']] for b, v in old['front_registration'].items()}
    axes = {}; holds = {}; recs = {}
    for name, files in groups.items():
        oo = [by[f] for f in files if by[f]['board']]
        if len(oo) < 3:
            continue
        rs = [record(o) for o in oo]; recs[name] = rs
        axes[name] = fit_pixel_axis(rs, Kg, dg)
        if len(rs) >= 4:
            f = fit_pixel_axis(rs[:-1], Kg, dg)
            target = oo[-1]; alt = by.get(target['file'].replace('.jpg', '-settled.jpg'), target)
            alt = alt if alt.get('board') else target
            Rangle = record(alt)['T'][:3, :3]
            angle = float(np.array(f['direction'])@Rotation.from_matrix(Rangle@np.array(f['R0']).T).as_rotvec())
            pred = predict_pose(f, angle)
            uv = cv2.projectPoints((rs[-1]['obj']-pred[:3, 3])@pred[:3, :3], np.zeros(3), np.zeros(3), Kg, dg)[0].reshape(-1, 2)
            holds[name] = {'training_files': [o['file'] for o in oo[:-1]], 'whole_pose_holdout': target['file'],
                           'orientation_source': alt['file'], 'conditional_not_independent': True,
                           'point_errors_px': summarize(np.linalg.norm(uv-rs[-1]['px'], axis=1)),
                           'camera_center_error_m': float(np.linalg.norm(pred[:3, 3]-rs[-1]['T'][:3, 3])),
                           'train_rms_px': f['point_rms_px']}
    (output/'pincher-fits.json').write_text(json.dumps({'fits': axes, 'holdouts': holds}, indent=2))
    configs = [('free' if h is None else 'h'+format(h*1000, 'g'), h, {}) for h in heights]
    if lens:
        configs += [('h60_F_'+label, .06, p) for label, p in [
            ('fplus1pct', {'fscale': 1.01}), ('fminus1pct', {'fscale': .99}),
            ('cyplus20px', {'cy': 20}), ('cyminus20px', {'cy': -20}),
            ('k1plus0012', {'k1': .012})]]
    rng = np.random.default_rng(20261015)
    scenarios = {}
    for label, h, perturb in configs:
        K, d = intrinsics('front', perturb)
        fronts = {}; fitted_axes = {}
        for block, oo in front_records.items():
            # Same two-start policy in every scenario. The alternative is from
            # the first TRAINING exposure, never the heldout front image.
            alt = np.array(oo[0]['board']['planar_pnp_candidates'][1]['T_cam_board'])
            full = fit_front(oo, K, d, h, initial_T=alt)
            train = fit_front(oo[:-1], K, d, h, initial_T=alt)
            T = np.array(full['T_F_W'])
            T0 = np.array(old['front_registration'][block]['T_F_W'])
            C = np.linalg.inv(T)[:3, 3]; C0 = np.linalg.inv(T0)[:3, 3]
            fronts[block] = {'full_fit': full, 'training_fit': train,
                'holdout_file': oo[-1]['file'], 'training_files': [o['file'] for o in oo[:-1]],
                'heldout': evaluate_front(np.array(train['T_F_W']), oo[-1:], K, d)[0],
                'rotation_change_from_archived_free_deg': float(np.degrees(Rotation.from_matrix(T[:3, :3]@T0[:3, :3].T).magnitude())),
                'board_frame_camera_shift_from_archived_free_m': (C-C0).tolist()}
            if label in ('free', 'h60') and bootstrap:
                samples = []
                for _ in range(bootstrap):
                    chosen = [oo[i] for i in rng.integers(0, len(oo), len(oo))]
                    q = fit_front(chosen, K, d, h)
                    a = in_front(axes['D_M3_85'] if block == 'D' else axes[block+'_M3_45'], np.array(q['T_F_W']))
                    samples.append([q['height_m'], a['setback_from_optical_origin_m'], a['axis_height_above_camera_m']])
                arr = np.array(samples)
                fronts[block]['registration_bootstrap'] = {'attempted': bootstrap, 'successful': len(samples),
                    'components': ['camera_height', 'axis_setback', 'axis_above_camera'],
                    'conditional_std_m': np.std(arr, axis=0, ddof=1).tolist(),
                    'conditional_percentile95_m': np.percentile(arr, [2.5, 97.5], axis=0).tolist(),
                    'scope': 'whole-front-frame resampling; frozen pincher axis, no intrinsics/scale/origin systematic bounds'}
            for name, axis in axes.items():
                if name[0] == block:
                    fitted_axes[name] = in_front(axis, T)
        ref = fitted_axes['D_M3_85']; n = np.array(ref['direction_F'])
        replication = {}
        for name in ['A_M3_45', 'B_M3_45', 'B_M3_75']:
            a = fitted_axes[name]; v = np.array(a['closest_point_to_front_optical_origin_m'])-ref['closest_point_to_front_optical_origin_m']
            replication[name] = {'separation_m': float(np.linalg.norm(v-n*(n@v))),
                'direction_difference_deg': float(np.degrees(np.arccos(np.clip(abs(np.array(a['direction_F'])@n), -1, 1))))}
        scenarios[label] = {'height_scenario_m': h, 'intrinsics_scenario': perturb,
            'not_measured_tolerance': True, 'provenance_key': provenance_key(inputs, K, d, h),
            'front': fronts, 'axes': fitted_axes, 'replication': replication,
            'pincher_holdout_id': hashlib.sha256(json.dumps(holds, sort_keys=True).encode()).hexdigest()}
        diag = fronts['D']['full_fit']
        print(label, 'D height/rms/setback/axisheight', diag['height_m'], diag['point_errors_px']['rms'],
              ref['setback_from_optical_origin_m'], ref['axis_height_above_board_m'], flush=True)
        (output/('scenario-'+label+'.json')).write_text(json.dumps(scenarios[label], indent=2))
    circles = {}
    for key in ['D_M3_45', 'D_M3_85', 'D_M3_95']:
        c = fit_circle_axis([r['T'][:3, 3] for r in recs[key]])
        rows = {}
        for label, scenario in scenarios.items():
            T = np.array(scenario['front']['D']['full_fit']['T_F_W'])
            cf = in_front(c, T); a = scenario['axes'][key]; n = np.array(a['direction_F'])
            v = np.array(cf['closest_point_to_front_optical_origin_m'])-a['closest_point_to_front_optical_origin_m']
            rows[label] = {'perpendicular_line_discrepancy_m': float(np.linalg.norm(v-n*(v@n))),
                'direction_difference_deg': float(np.degrees(np.arccos(np.clip(abs(n@np.array(cf['direction_F'])), -1, 1))))}
        circles[key] = {'by_scenario': rows, 'station': 'closest points to front optical origin; not invariant to station for nonparallel estimates'}
    links = {}
    for key in ['D_M4_190', 'D_M4_210']:
        rows = {}
        for label, scenario in scenarios.items():
            a = scenario['axes']['D_M3_85']; e = scenario['axes'][key]; n = np.array(a['direction_F'])
            v = np.array(e['closest_point_to_front_optical_origin_m'])-a['closest_point_to_front_optical_origin_m']
            cross = np.cross(n, np.array(e['direction_F']))
            rows[label] = {'effective_parallel_axis_distance_m': float(np.linalg.norm(v-n*(v@n))),
                'skew_line_shortest_distance_m': float(abs(v@cross)/np.linalg.norm(cross)),
                'direction_difference_deg': float(np.degrees(np.arccos(np.clip(abs(n@np.array(e['direction_F'])), -1, 1))))}
        links[key] = {'by_scenario': rows, 'independently_supported_physical_L1': False,
                     'station': 'approximately parallel lines at front optical closest points, not anatomical joint stations; skew shortest distance also reported'}
    samples = []; fails = 0
    rs = recs['D_M3_85']
    for _ in range(bootstrap*2):
        try:
            a = fit_axis([rs[i]['T'] for i in rng.integers(0, len(rs), len(rs))])
            transformed = {}
            for label in ['free', 'h60']:
                if label in scenarios:
                    q = in_front(a, np.array(scenarios[label]['front']['D']['full_fit']['T_F_W']))
                    transformed[label] = [q['setback_from_optical_origin_m'], q['axis_height_above_camera_m']]
            samples.append(transformed)
        except (ValueError, np.linalg.LinAlgError):
            fails += 1
    axis_unc = {'attempted': bootstrap*2, 'successful': len(samples), 'failed': fails,
               'scope': 'whole-pincher-pose model-free bootstrap, NOT pixel-bundle covariance; front transform frozen', 'by_scenario': {}}
    for label in ['free', 'h60']:
        if samples and label in samples[0]:
            arr = np.array([s[label] for s in samples])
            axis_unc['by_scenario'][label] = {'conditional_std_m': np.std(arr, axis=0, ddof=1).tolist(),
                'conditional_percentile95_m': np.percentile(arr, [2.5, 97.5], axis=0).tolist()}
    branches = {o['file']: o['board']['planar_pnp_candidates'] for oo in front_records.values() for o in oo}
    result = {'adoption': 'none', 'survey': survey, 'raw_manifest_verified_count': len(manifest),
        'frozen_observations_policy': 'same archived full corners and grouping; detection not rerun or selected using holdout; old cache never written or assumed valid by count',
        'front_objective': 'soft_l1 scale0.5, residual (projected-observed)/sqrt(Ncorners) per frame; K,d fixed within scenario',
        'front_holdout_policy': 'last archived registered frame in each block, all its corners excluded; full_fit is separate reconstruction, never heldout prediction',
        'pincher_holdout_policy': 'last pose per archived group; orientation conditional on separate settled exposure where available, otherwise same target orientation; NOT independent metrology',
        'scenarios': scenarios, 'pincher_holdouts': holds, 'position_circle_checks': circles,
        'effective_L1_checks': links, 'axis_bootstrap': axis_unc, 'front_planar_pnp_candidates': branches,
        'cross_view_pair_eligible': bool(validation['stationarity_brackets']['D']['joint_pair_accepted']),
        'archived_stationarity_brackets': validation['stationarity_brackets'],
        'cross_view_policy': 'D pair remains rejected; no crosscamera mount refit or anatomy validation',
        'archived_baseline': old['baseline'], 'archived_primary': old['sweeps']['D_M3_85']['camera_relative_axis'],
        'external_input_sha256': file_hashes(external),
        'versions': {'numpy': np.__version__, 'opencv': cv2.__version__}}
    after = file_hashes(p for p in session.rglob('*') if p.is_file())
    if before != after:
        raise RuntimeError('retained archive changed during run')
    result['archive_unchanged'] = True
    (output/'archive-after-sha256.json').write_text(json.dumps(after, indent=2))
    (output/'comparison.json').write_text(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('session', type=Path); ap.add_argument('output', type=Path)
    ap.add_argument('--bootstrap', type=int, default=100)
    args = ap.parse_args()
    run(args.session, args.output, bootstrap=args.bootstrap)
