import json,math,subprocess
from pathlib import Path
import numpy as np
D=Path(__file__).resolve().parent;ROOT=D.parents[2]
f=json.loads((D/'fit.json').read_text());r=json.loads((D/'recommendations.json').read_text());logs=list(map(json.loads,(D/'log.jsonl').read_text().splitlines()));caps=[q for q in logs if q.get('event')=='capture'];unique={q['file']:q for q in caps};assert len(unique)==f['counts']['retained_raw_frames'];assert all((D/x).is_file() for x in unique)
status=[q['result']['result'][0] for q in logs if q.get('event')=='rpc' and q.get('method')=='get_status' and q.get('result',{}).get('result')][-1]
correction=dict(event='analysis_corrections',filenames_at_actual_230_105=['g250-115-0-closed.jpg','f250-115-0-closed.jpg'],partial_joint_files_excluded=['g200-45-0.jpg','g200-45-settled.jpg','f-partial-200-45.jpg'],expected_wrist_id=69,observed_wrist_id=68,reason='earlier conservative acquisition guard stopped at230105 before subsequent capture block; final analyzer corrects tracked identities')
if not any(q.get('event')=='analysis_corrections' for q in logs):
 with (D/'log.jsonl').open('a') as out:out.write(json.dumps(correction)+'\n')
verification=dict(capture_calls=len(caps),retained_unique_raw_frames=len(unique),front=sum(q.get('camera')=='front' for q in unique.values()),gripper=sum(q.get('camera')=='gripper' for q in unique.values()),failed_camera_reads=sum(q.get('event')=='capture_failed' for q in logs),tests=dict(command='cd ros2 && ~/.pixi/bin/pixi run test src/z70_manipulation',manipulation_passed=33,full_passed=84,full_seconds=17.23),final_status=status,final_arm_estimate=dict(state='IN_ROBOT',joints=dict(m3=0,m4=0,m5=0,m6=0),stepsSinceAnchor=0),last_arm_motion='app_arm_in_compartment',no_chassis_motion=True,no_pick_goals=True,no_ROS_bringup=True,no_commits=True)
(D/'verification.json').write_text(json.dumps(verification,indent=1))
params=f['params'];sig=f['frame_bootstrap_sigma'];j4=f['rotation_statistics']['4'];axis=f['M5_camera_axis'];mark=f['marker_observed68_mount'];grasp=r['GRASP_base_m'];hover=r['near_hover'];obj=f['object_final']['rotation_center_m'];approach=r['nominal_approach_from_final_visible_contact']
report=f'''# Second live arm calibration — rotations and closed fingers

## Outcome

**Arm STOWED; M5 camera model corrected; physical L2 and exact jaw closing point NOT resolved.** This is a partial anatomy calibration, not a verified grasp. No chassis motion, pick/lift goals, ROS bringup, service restart or commit.

{len(caps)} capture calls produced **{len(unique)} retained raw frames ({verification['front']} front, {verification['gripper']} pincher), zero failed camera reads**. Analysis uses {f['counts']['gripper_board']} M5=0 board frames for the effective chain, {f['counts']['all_gripper_board']} total pincher-board frames, {f['counts']['front_board']} front-board frames, and {f['counts']['wrist_marker']} elevated-marker views. Counts are deduplicated from durable log/file paths.

The wrist target consistently decodes **ID68**, including with error correction disabled. The owner-reported **ID69 is preserved as the expected ID**, not silently normalized. Its 15 mm black side was used; its mount was re-registered from current views, never copied from ID0. The front upper-region marker mask separates this elevated target from floor-board identities.

## Measured rotations

- **M5 DOES move the pincher camera.** Nominal outward M5=10/20/30/40 gave 6.54/16.99/27.97/38.93 deg camera rotation. Return30/20/10 gave30.41/19.14/8.18 deg. These are board PnP, corroborated by the independently moving front marker. Nominal positive physical roll is about link2 **+x**. About3 deg direction-dependent slack/zero discrepancy remains; do not claim encoder feedback or a precise gain outside0..40.
- At nominal M5=40, lens translation was **4.883 mm**. Multi-view screw fit: camera-to-M5-axis perpendicular offset **{1000*axis['perpendicular_lens_offset_m']:.3f}±{1000*axis['frame_bootstrap_lens_offset_sigma_m']:.3f} mm** (800 frame resamples, zero reference held fixed; systematics excluded), translation point RMS **{1000*axis['axis_line_fit_point_rms_m']:.3f} mm**. Axis CF direction is {axis['direction_CF']}; its component frame scatter is {axis['direction_component_std']} (conditional, not absolute sigma).
- M3:13 single-joint pairs, step mean9.717 deg, step SD0.218 deg (mean sigma0.060); gain0.9717.
- M4:8 single-joint pairs, step mean{j4['step_mean_deg']:.3f} deg, step SD{j4['step_std_deg']:.3f} deg (mean sigma{j4['step_mean_sigma_deg']:.3f}); gain{j4['gain']:.4f}. Camera-to-elbow axis perpendicular distance **{1000*j4['axis_perpendicular_distance_mean_m']:.2f} mm**, pair SD **{1000*j4['axis_perpendicular_distance_std_m']:.2f} mm**. This is camera-to-axis distance, NOT physical L2.

## Conditional effective chain fit (not adopted)

Frames: CF is the floor below the front lens, x forward/y left/z up. Base/rotation-centre adds0.275m to CF x. Gains are anchored at deployed M3=150,M4=45. Link2 angle gauge is `M4cal-M3cal+90`; upper link angle is `270-M3cal+o3`.

| Parameter | Fit | Conditional frame-bootstrap 1sigma |
|---|---:|---:|
| Shoulder CF x | {1000*params['shoulder_CF_x']:.2f} mm | {1000*sig['shoulder_CF_x']:.2f} mm |
| Shoulder CF z | {1000*params['shoulder_CF_z']:.2f} mm | {1000*sig['shoulder_CF_z']:.2f} mm |
| L1 | {1000*params['L1_m']:.2f} mm | {1000*sig['L1_m']:.2f} mm |
| M3 upper-link offset | {math.degrees(params['M3_offset_rad']):.3f} deg | {math.degrees(sig['M3_offset_rad']):.3f} deg |
| M3 gain, global fit | {params['M3_gain']:.5f} | {sig['M3_gain']:.5f} |
| M4 gain, global fit | {params['M4_gain']:.5f} | {sig['M4_gain']:.5f} |
| Camera from elbow, effective x | {1000*params['camera_effective_x']:.2f} mm | {1000*sig['camera_effective_x']:.2f} mm |
| Camera from elbow, effective z | {1000*params['camera_effective_z']:.2f} mm | {1000*sig['camera_effective_z']:.2f} mm |
| Physical L2 | unknown/null | undefined/null |
| Unconditional physical M4 zero | unknown/null | undefined/null |
| Exact rubber-pad centre | unknown/null | undefined/null |

Gripper point RMS **{f['gripper_point_rms_px']:.3f}px**, versus **{f['previous_chain_same_data_point_rms_px']:.3f}px** for the retained old chain on the same new observations. Floor-constrained front-board point RMS **{f['front_board_point_rms_px']:.3f}px**.60 frame-block bootstraps. Bootstrap sigmas exclude front intrinsics' elevated extrapolation, printed-size/flatness and board/floor systematics. This worse absolute floor registration and unresolved pad point are why these new effective lengths/gains did **not** replace the prior M5=0 chain.

If physical link2 is DEFINED to align with the measured M5 roll axis, the conditional M4 bend zero is **{f['conditional_M4_zero']['offset_deg']:.3f}±{f['conditional_M4_zero']['frame_bootstrap_sigma_deg']:.3f}deg** (bootstrap only; several-degree inter-camera/systematic allowance excluded). This is a conditional alignment result, not independently verified anatomy, and is not adopted.

The observed ID68 marker effective vector from elbow is **({1000*mark['effective_vector_from_elbow_m'][0]:.2f}, {1000*mark['effective_vector_from_elbow_m'][1]:.2f}, {1000*mark['effective_vector_from_elbow_m'][2]:.2f})mm**, conditional bootstrap sigmas **{[round(1000*x,2) for x in mark['frame_bootstrap_sigma_m']]}mm**, position point RMS **{1000*mark['position_point_rms_m']:.3f}mm**. Full current rotation matrix is in fit.json. This is an effective mount in the new fit's gauge, not a transform to a physically located wrist-axis origin.

### Why L2 is still unobservable

M4 rotations locate the elbow axis and camera-to-elbow distance. M5 rotations locate a wrist-roll axis LINE. Translating a putative wrist origin along that roll axis leaves every camera/marker rotation unchanged; an unknown mount offset absorbs it. No marked/measured axial station identifies the physical wrist origin. More ±10deg sweeps cannot remove that null space. One independently measured elbow-to-wrist station or a rigid registered wrist reference is needed for physical L2.

## Jaw observation and retained target

Closed empty gripper frames were captured at nominal230/105,240/105 and250/115, plus the M5 sweep. The owner's TOP-MIDDLE location is confirmed as a dark cropped assembly (roughlyx700..900,y55..110). Even contrast-enhanced crops do **not** resolve two specific fingertip contact surfaces or a repeatably identifiable material point. A region is not a measured ray; known camera pose alone gives no metric depth along an unidentified ray.

Front seam hypotheses are explicitly recorded with8..13px annotation uncertainty, not treated as exact material correspondences. They produce a conditional effective vector around(213.53,1.73,25.98)mm under the new fit, with several-mm annotation uncertainty; **not adopted**. The front distal endpoint and opening apex are different landmarks from the contact seam midpoint.

The retained provisional seam vector remains **(202.004,-0.300,26.646)mm in nominal link2 coordinates from the elbow**, with the prior ±10mm provisional bound (not a fresh measured sigma). Physical L2=205mm remains only a coordinate gauge; the corresponding nominal-wrist offset is(-2.996,-0.300,26.646)mm. Exact closed-pad centre and lowest open-finger point remain unresolved.

## Provisional 10deg-grid recommendation

The old search restricted range to0.45..0.50m and missed a better nominal equator pose at shorter range. Searching0.35..0.50m gives:

| Pose M3,M4,M5 | Jaw base/rotation-centre xyz (mm) | Purpose |
|---|---|---|
| **240,85,0** | **({1000*grasp[0]:.2f},{1000*grasp[1]:.2f},{1000*grasp[2]:.2f})** | New provisional GRASP;33mm equator error -0.29mm |
| **230,85,0** | (428.56,23.37,80.35) | Near hover; single M3+ jog toGRASP |
| 240,95,0 | (445.28,23.34,54.34) | Higher observation pose, not equator |
| 250,115,0 | (481.21,23.25,52.02) | Live observed closed measurement pose, not equator |
| old250,105,0 | (457.22,23.31,25.82) |7.18mm below equator; no longer nominal |

At near hover, the prospective object equator projects to **({hover['target_equator_pixel'][0]:.1f},{hover['target_equator_pixel'][1]:.1f})px**, optical depth **{1000*hover['target_equator_pixel'][2]:.1f}mm**. Full path/grid data are in recommendations.json.

**These are model predictions, not exercised grasps.** GRASP's ±10mm provisional lower bound is22.71mm, so this does NOT certify ≥3cm open-finger clearance. No retained grid pose both hits the33mm equator and has a ≥3cm conservative lower bound. Do not blindly run a pick on the strength of sub-mm nominal height error.

## Final object and robot state

Final front image99-stowed-front.jpg: yellow target-only bbox approximately(708,604)..(864,747), contact reference(784,744)±(8,5)px. HSV merged the yellow object with the wood and shadow; that merged centroid/contact was rejected. Reviewed visible-bottom floor projection gives **({obj[0]:.4f},{obj[1]:.4f},0)m from rotation centre**, i.e. **{1000*obj[0]:.1f}mm ahead and {abs(1000*obj[1]):.1f}mm right**. Pixel-only sigma approximately(8.5,2.9)mm; use at least±20..30mm for physical contact/centre ambiguity and floor-camera systematics. It is a visible-contact estimate, not a surveyed geometric centre. Dodecahedron/board were not touched or driven over.

Under that estimate and the provisional GRASP, nominal approach is **{math.degrees(approach['yaw']):.2f}deg yaw, then {1000*approach['forward']:.1f}mm forward**; this is not a drive instruction or validated endpoint. Re-observe/align before closure.

Last arm motion was stow. Immediate settled status: state39,error0,common_status51330,bit256clear; front image has no gripper. Final fresh status after offline analysis: **state{status['state']},battery{status['battery']}%,error{status['error_code']},common_status{status['common_status']},readiness bitclear**. Exact estimate readback: **IN_ROBOT**, jointsall0,wristnull,stepsSinceAnchor0. Status/model/image corroborate stow; no joint encoders are claimed.

## Adoption, tests and acquisition issues

Changed planner.camera_pose and URDF to include positive physical M5 roll; moved optical mount under the roll chain and used axis-x with the bridge's existing negative M5 joint values. M5=0 geometry remains identical. Shared YAML records new evidence, ID discrepancy and unresolved anatomy. Planner GRASP changes to240/85, with regression for33mm height and measured camera roll. Real robot_state_publisher tests compare the full roll-dependent optical transform. README and synthetic range test are updated. Vendor firmware FK/model constants are unchanged.

**Tests: requested manipulation command33passed; fullROS84passed in17.23s; git diff --check passed.** No commit. Pre-existing chassis online.json and unrelated untracked artifacts were not modified by this task.

- A200/45 M3 command only moved about1.12deg; it was not counted. The arm was stowed/redeployed to restore a known sequence. No reported robot error_code occurred. Later acquisition uses observed image pose changes, not the invalidated grip estimate.
- A helper's conservative height guard stopped the route at230/105; two subsequent captures were mistakenly named250-115. Their correction is explicit in log.jsonl and analyze.py. True250/115 captures use the-actual suffix. One zero-M5 pincher filename was overwritten by its return frame; retained counts are unique-file counts, not inflated capture counts.
- OpenCV5 RANSAC found low-residual mirror/partial planar solutions. Final extraction uses consistent direct iterative PnP, residual trimming and no-SUBPIX fallback when ChArUco interpolation returns0. Original images and rejected partial-jog frames are retained.
- app_grip returnedACK and only closed; app_open_gripper timed out normally. No lift was sent. Grip invalidated the old service estimate asUNKNOWN; independently tracked/observed joints were used, then stow restoredIN_ROBOT.

Evidence: log.jsonl,59 raw frames, selected overlays/crops, observations.json, fit.json, analysis-output.txt, recommendations.json, verification.json and reproducible acquire/analyze/recommend helpers.
'''
(D/'RESULTS.md').write_text(report)
print(json.dumps(verification,indent=1));print('Created RESULTS.md')
