import json,math,datetime,collections
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
D=Path(__file__).resolve().parent;f=json.loads((D/'fit.json').read_text());obs=json.loads((D/'observations.json').read_text());p=f['params'];s=f['frame_bootstrap_sigma'];rows=[json.loads(x) for x in (D/'log.jsonl').read_text().splitlines()];caps=[x for x in rows if x.get('event')=='capture'];unique={x['file']:x for x in caps};status=next(x for x in reversed(rows) if x.get('method')=='get_status')['result']['result'][0]
# Gauge check: changing nominal L2 only shifts endpoint offsets, not the observed chain.
v=np.array([p['camera_effective_'+a] for a in 'xyz']);gauge=[]
for L2 in [.155,.205,.255]:
 offset=v-[L2,0,0];gauge.append(dict(nominal_L2=L2,camera_offset=offset.tolist(),effective_error_m=float(np.linalg.norm(np.array([L2,0,0])+offset-v))))
mk={x['file']:x for x in obs['wrist_marker']};base=np.array(mk['f240-105-0.jpg']['Tcf']);axes=[]
for name in ['f240-105-20.jpg','f240-105-30.jpg','f240-105-40.jpg','f240-105-50.jpg']:
 T=np.array(mk[name]['Tcf']);rv=Rotation.from_matrix(T[:3,:3]@base[:3,:3].T).as_rotvec();axis=rv/np.linalg.norm(rv);link=np.array([math.cos(-math.pi/4),0,math.sin(-math.pi/4)]);axes.append(dict(file=name,rotation_deg=float(np.degrees(np.linalg.norm(rv))),axis_CF=axis.tolist(),axis_link2_angle_deg=float(np.degrees(np.arccos(np.clip(abs(axis@link),-1,1))))))
checks=dict(L2_gauge_invariance=gauge,M5_relative_axes=axes,all_params_finite=bool(np.isfinite(list(p.values())).all()),image_count=len(unique),camera_counts=dict(collections.Counter(x['camera'] for x in unique.values())),capture_calls=len(caps),capture_failures=sum(x.get('event')=='capture_failed' for x in rows))
(D/'verification.json').write_text(json.dumps(checks,indent=1))
def mmvec(x):return '('+', '.join(f'{v*1000:.2f}' for v in x)+') mm'
text=f'''# Live arm calibration — effective geometry and provisional jaw endpoint

## Outcome and limits

The live measurement run succeeded: **{len(unique)} retained distinct raw frames ({checks['camera_counts']['front']} front, {checks['camera_counts']['gripper']} gripper), 0 failed camera reads**. Usable observations: **{f['counts']['gripper_board']} gripper-board frames at {f['counts']['distinct_gripper_M3M4']} distinct M3/M4 poses, {f['counts']['front_board']} front-board frames, {f['counts']['front_wrist']} independent elevated wrist-ID0 PnPs, and {f['counts']['closed_jaw_views']} closed contact-region views**. M3 and M4 were varied independently, not only along one diagonal. No chassis moves, pick goals, lift commands, service restarts, ROS bringup or commits were performed.

**This is not a complete true-wrist anatomy calibration.** The ID0 black side is known (15 mm), but its physical transform to the wrist axis is not independently measured. Physical L2 and M4 zero remain exactly confounded with marker/camera mounts; their estimates and sigmas are null, not fitted fiction. The M5 trajectory's rotation axis is close to link2, so it does not establish an axial wrist station. The true rubber-pad centre is not resolved in the dark front frames. The measured closed terminal contact-region seam midpoint is explicitly **provisional**.

Planner targeting and camera geometry now use the identifiable effective chain, with a 10 mm provisional jaw uncertainty bound; firmware checks continue to use the vendor model. URDF camera TF and the M5=0 contact TCP were updated consistently and exercised through real robot_state_publisher in isolated offline tests.

## Chain and fitted parameters

Frames: CF origin is the floor below the front camera, x forward, y left, z up. Base/rotation-centre frame adds +0.275 m to CF x. All quantities below use M5=0 and the nominal link2 orientation gauge.

`E = S + L1 * e(270 - M3 + o3)`

`camera = E + R_yaw,link2(M4 - M3 + 90) * v_camera`

`jaw = E + R_yaw,link2(M4 - M3 + 90) * v_jaw`

| Quantity | Fit | Conditional 1 sigma (100 frame-block bootstraps) | Vendor / previous convention |
|---|---:|---:|---|
| Shoulder CF x | {p['shoulder_CF_x']*1000:.2f} mm | {s['shoulder_CF_x']*1000:.2f} mm | vendor shoulder x=47.5 mm in its own frame; previous frozen transform gave approx -153.5 mm CF |
| Shoulder CF z | {p['shoulder_CF_z']*1000:.2f} mm | {s['shoulder_CF_z']*1000:.2f} mm | vendor z=140 mm; previous mount gave 148.5 mm floor height |
| L1 | {p['L1_m']*1000:.3f} mm | {s['L1_m']*1000:.3f} mm | 177.500 mm, consistent within sigma |
| M3 offset o3 | {f['M3_offset_deg']:.3f} degrees | {f['M3_offset_sigma_deg']:.3f} degrees | 0 degrees |
| Physical L2 | unidentifiable | undefined/null | vendor 205 mm retained ONLY as wrist-coordinate gauge |
| Physical M4 zero | unidentifiable | undefined/null | nominal zero absorbed in mount vector and rotation |
| Effective camera link2 x from elbow | {v[0]*1000:.2f} mm | {s['camera_effective_x']*1000:.2f} mm | previous camera-at-wrist convention 205 mm |
| Effective camera link2 z from elbow | {v[2]*1000:.2f} mm | {s['camera_effective_z']*1000:.2f} mm | previous convention 0 mm |

Shoulder in rotation-centre/base frame: {mmvec(f['shoulder_base_m'])}. Shoulder y is convention-dependent: the camera's lateral vector is fixed to zero to define the arm plane. Yaw is {math.degrees(p['arm_yaw_rad']):.3f} degrees with conditional sigma {math.degrees(s['arm_yaw_rad']):.3f} degrees. Nominal-wrist camera offset is {mmvec(f['camera_nominal_vendor_wrist_offset'])}; mount rotation is the full matrix frozen in YAML (nominal mechanical roll/pitch/yaw approximately +0.125/-2.212/+0.106 degrees). These are mount-gauge values, not independently measured camera-to-physical-wrist dimensions.

Gripper corner **2-D point RMS = {f['gripper_point_rms_px']:.2f} px** (per-coordinate RMS {f['gripper_scalar_rms_px']:.2f} px). Same-data vendor-L1/zero-M3 model with optimised origin/free camera mount: **{f['vendor_fixed_L1_zero_M3_free_mount_point_rms_px']:.2f} px** point RMS. Whole-pose-held-out fold RMSs: {', '.join(f"{x['rms_px']:.2f}" for x in f['held_out_pose_folds'])} px. Front board floor-constrained point RMS: {math.sqrt(2)*f['board_in_CF']['front_scalar_rms_px']:.2f} px. Independent marker positional point RMS: {math.sqrt(3)*f['marker_position_scalar_rms_m']*1000:.2f} mm. Old 13.1/7.1 px fits used different frames/models and are not a direct held-out comparison.

Sigmas are conditional: they exclude front intrinsics' elevated-image extrapolation, floor-board nonplanarity/height uncertainty, printed side tolerance and exact pad landmark choice. The floor-constrained front-board residual is noticeably larger than a free planar PnP; do not treat the sub-mm bootstrap values as absolute physical accuracy. Numpy covariance calculations emitted runtime warnings; retained parameters and reported sigmas are finite, and frame-bootstrap values are used here rather than relying on the gauge-singular covariance.

## Jaw measurement

Known marker poses and five front views at M5=0,10,20,30,40 degrees triangulate the same provisional closed contact-region midpoint; this is **not silhouette scaling**. Contact seam annotation uncertainty is 6-9 px. Close crops and fit overlays confirm the point lies on the terminal jaw region, but do not expose an exact rubber-pad centre.

- In ArUco marker coordinates (OpenCV square axes): {mmvec(f['jaw_marker_frame'])}; conditional annotation sigmas {mmvec(f['jaw_marker_sigma_m'])}.
- Effective link2 vector from elbow: **{mmvec(f['jaw_effective_from_elbow'])}**.
- Relative to the **nominal 205 mm vendor wrist endpoint**, with link2 x along link, y left, z perpendicular/up: **{mmvec(f['jaw_nominal_vendor_wrist_offset'])}**. This is NOT an independently registered offset to the real wrist motor.
- Jaw annotation fit coordinate RMS {f['jaw_scalar_rms_px']:.2f} px; point RMS {math.sqrt(2)*f['jaw_scalar_rms_px']:.2f} px.
- Conservative provisional contact-region bound: +/-10 mm. The actual pad centre and the lowest open-finger tip remain unresolved; this bound is not a certified collision guarantee.

| M3,M4 | Jaw x ahead of rotation centre | Jaw y left | Jaw height |
|---|---:|---:|---:|
'''
for key,pose in f['jaw_poses'].items():
 xyz=pose['rotation_center_frame'];text+=f"| {key} | {xyz[0]*1000:.1f} mm | {xyz[1]*1000:.1f} mm | {xyz[2]*1000:.1f} mm |\n"
text+='''
The former (240,105) contact estimate was roughly 3.6 cm; this run predicts about 8.1 cm. That is consistent with jaws closing several centimetres too high. Independent direct marker-based reconstructions at that pose are about 8.3-8.6 cm, within the provisional bound. At the actual closed measurement pose (250,115), the directly triangulated closing region was around 5.0-5.2 cm high; no floor contact was visible.

## Equator pose recommendations (offline only)

For a 33 mm-high equator 0.45-0.50 m ahead of the rotation centre:

'''
for x in f['continuous_equator_solutions']:
 text+=f"- Continuous fit at x={x['rotation_center_x_m']:.3f} m: M3={x['M3_M4_deg'][0]:.2f}, M4={x['M3_M4_deg'][1]:.2f} degrees, z={x['jaw_CF'][2]*1000:.1f} mm. **Not executable as an absolute target with the known 10-degree jog RPC.** Vendor wrist x={x['vendor_wrist_x_m']:.3f} m; {'within' if x['within_conservative_firmware_reach'] else 'OUTSIDE'} conservative firmware reach.\n"
text+='''
Closest conservative-reach 10-degree grid candidate: **(250,105)** predicts x=0.457 m, z=25.8 mm (7.2 mm below equator); it is now the planner's provisional nominal candidate, not a verified grasp or a guaranteed >=3 cm-clearance pose. **(250,115)** predicts x=0.481 m, z=52.0 mm, suitable as a higher observation/approach pose, not an equator closure. No 10-degree grid pose in the retained conservative reach envelope exactly satisfies z=33 mm in this range. Do not blindly round the continuous solution or call these live-validated picks.

## Acquisition and data-quality issues

- Preflight front frame delivered 52 legacy ChArUco corners. First immediate pickup/deploy sequence was acknowledged but never set readiness; gripper selection returned 409. After an observed stow/reset and a 3-second pickup-mode settling interval, deployment set common_status=51458 and gripper frames delivered normally.
- Every M3/M4 measurement jog was followed by settling >=3 seconds and a gripper-board PnP/image-change check before another jog. M5 used independent front-marker image-change checks. No refused M3/M4 jog was counted; the route stayed below the vendor reach envelope. Edge-on M5 marker loss was a measurement gap, not a camera failure; it stopped the local batch, was inspected, and was resolved by observed return steps or stow.
- An elevated wrist ID0 contaminated ChArUco interpolation, sometimes yielding 0 corners or >80 px false fits. Final analysis masks the upper front region for floor-board detection and retains wrist detections separately. The board's down-facing normal/handedness was explicitly corrected before chain fitting. Robust PnP kept corners within 4 px of each frame's planar model.
- app_grip returned an ACK in this run and invalidated RoboClaw's estimate as UNKNOWN because its old estimator assumes autonomous lift. Marker/frame observations showed only jaw closing; per the owner's grip-only fact, the closed-jaw M5 measurements used separately tracked joints and marker checks, not the stale estimate. No lift was sent.
- Two repeated-pose filenames (one gripper, one front at 230/95) were overwritten before the helper gained unique suffixes. The fit uses only the retained latest image per filename, never counts those log entries as independent retained views. 65 capture calls produced 63 retained raw images. All other repeat captures have unique names.
- Open fingertips were only qualitatively observed above the board. Do not claim a measured >=3 cm minimum throughout the open sweep; exact open-pad points are obscured. No contact/hazard was observed, and the lowest closed measurement remained around 5 cm.

## Code adoption and propagation

Changed:
- `ros2/src/z70_manipulation/z70_manipulation/planner.py`: explicit effective shoulder/L1/M3-offset chain and jaw point; full measured camera pose with distortion; provisional jaw bound, updated nominal grid candidate, and descent visibility for both equator and contact. Vendor limits/reach checks remain separate.
- `ros2/src/z70_description/config/mount_calibration.yaml`: shared effective geometry and provenance, with unidentifiable anatomy labelled explicitly.
- `ros2/src/z70_description/urdf/z70.urdf.xacro`: measured link1/origins, effective camera mount and provisional M5=0 contact TCP. Nominal L2 wrist link and open finger visuals are not measured anatomy; TCP at other M5 angles remains unvalidated.
- `ros2/src/z70_manipulation/test/test_planner.py`: live metric jaw-height/corner-projection regressions and updated floor checks; old 240/105 equator assumption removed.
- `ros2/src/z70_manipulation/test/test_executor.py`: synthetic camera no longer clamps near-field projected object radius, which had hidden the changed camera geometry.
- `ros2/src/z70_description/test/test_gripper_tf.py`, `test_wrist_tf.py`: actual TF agreement with effective chain, camera rotation and M5=0 TCP; wrist explicitly nominal-gauge.
- `ros2/src/z70_manipulation/README.md`: current model, uncertainty and limitations.

`src/arm-model.ts` and `ros2/src/z70_description/z70_description/arm_model.py` were **not changed**: they encode firmware/app predictive FK, IK and command checks, not identified physical anatomy. Do not replace their L2 with a marker-space length. The geometric URDF and planner needed the effective changes and now share them. If physical marker-to-wrist registration becomes available, refit L2/M4 zero before changing any physical wrist model; keep a separate vendor command model.

No commit. Unrelated pre-existing working-tree changes (including chassis online.json) were untouched. Created this entire dated evidence directory: log.jsonl, raw images, detection/fit overlays, observations.json, fit.json, marker-preliminary.json, verification.json, and acquisition/analysis/render/report helpers. The procedural skill reference was updated with ID0 masking and provisional multi-view contact lessons.

## Tests and final robot state

- Requested command `cd ros2 && ~/.pixi/bin/pixi run test src/z70_manipulation`: **31 passed**, up from 27, after red-green metric regressions.
- Combined description/manipulation: **42 passed**; full `~/.pixi/bin/pixi run test`: **82 passed in 17.20 s**. Test-only robot_state_publisher/ROS processes were reaped; no bringup or robot-contact observation process was started.
- `git diff --check`: passed.
- Sent stow and read exact status after 15 s: state=39 (pickup mode), battery=38%, error_code=0, common_status=51330, readiness bit 256 clear. Estimate read back IN_ROBOT, all joints zero, wrist null, stepsSinceAnchor=0. Front camera confirmed; final frame shows the previously visible wrist/gripper absent. This corroborates completed stow via status/model/image, not joint encoders. Last arm command was stow.
'''
(D/'RESULTS.md').write_text(text)
print(json.dumps(checks,indent=1));print('Wrote RESULTS.md')
