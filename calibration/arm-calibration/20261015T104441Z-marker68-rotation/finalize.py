import json,hashlib
from pathlib import Path
import numpy as np
P=Path(__file__).resolve().parent
r=json.loads((P/'results.json').read_text());d=json.loads((P/'orientation-diagnostics.json').read_text());s=json.loads((P/'final-state.json').read_text());e=json.loads((P/'final-estimate.json').read_text());logs=[json.loads(x) for x in (P/'log.jsonl').read_text().splitlines()]
frames=[x for x in logs if x.get('event')=='frame'];assert all(hashlib.sha256((P/x['file']).read_bytes()).hexdigest()==x['sha256'] for x in frames)
c=r['counts'];totalrequests=c['capture_requests']+2;failures=c['capture_failures']+1
assert totalrequests==c['retained_original_images']+failures+1
rawstatus=[x['result']['result'][0] for x in logs if x.get('name')=='robot_rpc' and x.get('args',{}).get('method')=='get_status'][-1]
assert rawstatus['state']==3 and rawstatus['error_code']==0 and not rawstatus['in_cleaning'] and not rawstatus['in_returning']
assert ((rawstatus['common_status']>>10)&3)==0
assert e['state']=='IN_ROBOT'
assert all(not v['consumers'] for v in s['camera']['streamDiagnostics'].values())
assert (P/'raw/final-front.jpg').exists()
summary=[]
for k,v in r['step_summary'].items():summary.append(f"| {k} | {v['measured_steps']} | {v['angle_mean_deg']:.2f} ± {v['angle_std_deg']:.2f} |")
axes=[]
for k,v in d['long_arc_axes'].items():axes.append(f"| {k} | {', '.join(f'{x:+.4f}' for x in v['positive_command_axis_F'])} | {', '.join(f'{x:+.4f}' for x in v['positive_command_axis_W'])} |")
text=f'''# Marker68 orientation-first trial — RESULTS

## Outcome
Angular calibration evidence acquired; **no production calibration adopted**. Board centred by observed bounded chassis yaw while stowed, then chassis frozen for all arm measurements. Captured independent M3/M4/M5 changes, two M5 0→40→0 sweeps at different arm pitch configurations, fixed-floor pincher-camera poses, and rotation-chain mixed-pose holdouts. No grip, pick, cleaning, docking or map navigation.

**Coverage is partial:** M3 has only one fully front-visible 10° step, not the requested ≥3 settings/20–40° forward-and-return range. M4 has a useful multi-setting first group but only one measured step at the second M3 level. Last attempted M3 reverse was ACKed but its frame failed; it is NOT counted as a measured return. Camera failures curtailed the remaining revisit, not setup-only abandonment. No unobserved jog was replayed. Total {c['jog_attempts']} jog attempts, within the 45 bound.

## Measured rotations, not encoder angles
| group | measurable consecutive changes | mean ± SD, deg |
|---|---:|---:|
{chr(10).join(summary)}

Nominal numbers in filenames/CSV identify **command counts**, not measured encoders or reproducible absolute joints. The estimator flagged a reach-limit refusal during the route; physical images still changed. Thus no estimator refusal is promoted into a physical zero-angle claim. The first M5 steps were only about 6.8–7.6°, later steps around 10–12°; gains are not uniformly exact 10°.

M5 A outward/return totals: {r['step_summary']['M5_A']['outward_total_deg']:.2f}° / {r['step_summary']['M5_A']['return_total_deg']:.2f}°, endpoint closure {r['step_summary']['M5_A']['return_to_start_deg']:.2f}°. M5 B: {r['step_summary']['M5_B']['outward_total_deg']:.2f}° / {r['step_summary']['M5_B']['return_total_deg']:.2f}°, closure {r['step_summary']['M5_B']['return_to_start_deg']:.2f}°. Mid-sweep approach/return discrepancies are roughly 1–3° and are not separable from pose noise at this scale.

### Axes (positive-command direction, unit vectors)
F = OpenCV front optical x-right, y-down, z-forward. W = this trial's fixed ChArUco board coordinates, NOT a surveyed chassis frame. W transformations use the current block only.

| joint/configuration | axis F | axis W |
|---|---|---|
{chr(10).join(axes)}

Long-arc M3/M4 unsigned parallelism discrepancy {d['M3_M4_unsigned_parallelism_deg']:.2f}°; insufficient evidence to resolve a real mechanical skew from marker/lens noise. M5 axis transported between the two pitch configurations agrees within {d['M5_axis_transport_discrepancy_deg']:.2f}°. Axis stationarity scatter from individual short steps is retained in results.json; it is several degrees and grows for weak increments.

## Noise / ambiguity / ground crosschecks
Front stationary pilots: 5 frames at each of two visibly different tilts. RMS to each SO(3) mean {r['pilot_front']['A']['rms_to_mean_deg']:.2f}° and {r['pilot_front']['B']['rms_to_mean_deg']:.2f}°; maximum stationary pair differences {r['pilot_front']['A']['max_pair_deg']:.2f}° and {r['pilot_front']['B']['max_pair_deg']:.2f}°. This is not 1° precision. Per-step Monte Carlo intervals propagate the larger pilot noise through 500 draws; these are conditional random-noise intervals, NOT systematic-inclusive calibration accuracy.

Pincher stationary pilots (3 each): full-orientation RMS {r['pilot_gripper']['G180']['rms_to_mean_deg']:.3f}° / {r['pilot_gripper']['GB']['rms_to_mean_deg']:.3f}°. Optical-axis angles to the floor span the recorded poses; initial pilots approximately 43.36° and 53.29°. Every accepted G board pose and angle is in results.json. A 20° nominal M3 excursion measured {r['gripper_crosschecks']['M3_20_nominal']['observed_rotation_deg']:.2f}° in G; M5 A40 measured {r['gripper_crosschecks']['M5_A40']['observed_rotation_deg']:.2f}° in G versus {r['step_summary']['M5_A']['outward_total_deg']:.2f}° in F. The latter ~1.5° disagreement is compatible with the front pilot noise but is not an independent mount calibration.

F/G/F bracket orientation changes 0.50–1.37°; streams are sequential, never simultaneous. Current F floor orientation RMS scatter {r['stationary_floor_orientation']['rms_scatter_deg']:.2f}°, max {r['stationary_floor_orientation']['max_scatter_deg']:.2f}°. Body rocking cannot be separated from floor-board fitting noise; W-frame axis accuracy carries that uncertainty. Per-exposure R_W_M is retained when its own floor pose passes, otherwise null. No old floor TF or fixed floor height was imposed.

Both IPPE_SQUARE branches, positive-depth checks, ordered decoded corners, residuals, matrices and quaternions are retained. Primary refined SUBPIX marker detector and fallback board sequence were implemented before deployment. Branch selection is positive-depth minimum reprojection, with a separately audited 0.25px-tie continuity rule; {d['branch_switches_vs_minimum']} changes versus minimum reprojection. No FK/desired angle used for branch choice. Near-planar ambiguity/lens extrapolation and print flatness remain systematic caveats. Marker ID68 is selected above the floor-board mask, not confused with the floor ID68. The printed black side is 15mm, not the 20mm paper/checker square.

## Held-out mixed rotation chain
Withhold all B rolled poses. Learn only A roll observations and the pure M4 change D4 = R_B0 R_A0ᵀ; predict R_Broll = D4 R_Aroll. Four held-out mixed poses at nominal 10/20/30/40 have errors {', '.join(f"{h['error_deg']:.2f}°" for h in r['heldout']['poses'])}, RMS **{r['heldout']['rms_deg']:.2f}°**. This checks rotational composition and axis transport, not command-only FK, full-chain zeros or physical link lengths. Holdout intent existed during acquisition, but exact analysis membership was saved after capture: treat this as exploratory validation, not strictly preregistered independent confirmation. No B rolled observations enter D4 or A predictions.

## Identifiability / adoption
Supported provisionally: M3/M4 are approximately oppositely signed parallel pitch rotations; M5 is a terminal roll transported by preceding pitch, about 41° for four outward steps in these configurations. Retain measured axes/step uncertainty for further calibration; **do not overwrite shared model constants** from these short arcs.

**Rotation matrices contain no link lengths.** Physical L1/L2, M5 axial station, marker mounting location and shoulder axis location are not estimated by this rotation-only trial. Stored marker translations are secondary 15mm-scale PnP trajectories only, not a length fit. Physical M3/M4 zeros and a free fixed mount orientation have rotational gauge freedoms; no unique mechanical zeros or physical wrist reference were fitted. Existing ~180mm L1 is neither validated nor refuted here.

## Evidence, counts and reproduction
- {c['retained_original_images']} retained original JPEGs, including preflight and successful final cleanup.
- {totalrequests} capture requests total = {c['capture_requests']} SDK + 2 direct MCP. {failures} failed requests; one early successful SDK image was not retained due an inline-image logger defect, documented separately. {c['retained_logged_images']} SDK retained images + one direct preflight = retained originals.
- {c['front_marker_observations']} full marker orientation observations; {c['accepted_board_observations']} accepted floor poses across F/G; {c['null_marker_observations']} null marker views (G intentionally included).
- {c['orientation_step_accepted']}/{c['orientation_step_attempts']} consecutive step comparisons measurable; rejected/missing step is null, never zero.
- raw/*.jpg + raw/*.json: originals, hashes, detections, rejected counts, both marker PnP branches, board corners/residuals.
- log.jsonl: exact MCP calls, parameters, results, receipt time/exposure-null, camera identity/dimensions and dead-reckoned estimates.
- step_angles.csv / step_rotations.json: relative SO(3), axes, normal tilt, old-marker-z swing/twist and conditional intervals. Euler intrinsic XYZ is secondary presentation only in orientation-diagnostics.json.
- Reproduce: `calibration/arm-validation/.venv/bin/python {P.relative_to(P.parents[2])}/test_rotations.py` (4 tests pass), then `.../diagnostics.py`. Analyzer has fixed output in its own fresh session directory; tests do not write archives. No old calibration archive or shared model was edited; no commit.

## Final cleanup
Final image raw/final-front.jpg visually shows pincher/attached marker absent and board still present. Final raw status state=3, error=0, battery={rawstatus['battery']}%, no clean/return; pickup bits cleared, program_state=0. Front selected; both camera streams have empty consumer lists. Estimate IN_ROBOT is dead reckoning initialized by stow, not encoders. Stow and mode-exit ACKs alone were not used as proof: delayed raw status plus final image were checked.

Camera failure diagnosis: robot RPC link remained healthy, MQTT/PIN/ICE/DTLS connected, but acquisition reported zero video tracks/packets and timed out. Camera selection recovery did not restore frames. No direct robot transport, Docker/service restart or continuous polling was used. An initial pickup-mode exit was rejected (-10007); app_stop followed by settled stow restored idle and final capture. Cleanup now verified, camera root cause unresolved.
'''
(P/'README.md').write_text(text)
verification=dict(original_hashes_verified=len(frames),total_capture_requests=totalrequests,failed_capture_requests=failures,successful_unretained=1,counts_reconciled=True,final_state=rawstatus['state'],battery=rawstatus['battery'],error_code=0,pickup_bits_cleared=True,program_state=0,streams_empty=True,final_image='raw/final-front.jpg',tests='4 passed',production_edits=False,commits=False)
(P/'verification.json').write_text(json.dumps(verification,indent=2))
(P/'artifact-sha256.json').write_text(json.dumps({str(f.relative_to(P)):hashlib.sha256(f.read_bytes()).hexdigest() for f in P.rglob('*') if f.is_file() and f.name!='artifact-sha256.json' and '__pycache__' not in str(f)},indent=2))
print(json.dumps(verification,indent=2))
