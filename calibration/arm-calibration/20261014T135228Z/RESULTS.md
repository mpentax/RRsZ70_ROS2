# Live arm calibration — effective geometry and provisional jaw endpoint

## Outcome and limits

The live measurement run succeeded: **63 retained distinct raw frames (39 front, 24 gripper), 0 failed camera reads**. Usable observations: **24 gripper-board frames at 21 distinct M3/M4 poses, 12 front-board frames, 25 independent elevated wrist-ID0 PnPs, and 5 closed contact-region views**. M3 and M4 were varied independently, not only along one diagonal. No chassis moves, pick goals, lift commands, service restarts, ROS bringup or commits were performed.

**This is not a complete true-wrist anatomy calibration.** The ID0 black side is known (15 mm), but its physical transform to the wrist axis is not independently measured. Physical L2 and M4 zero remain exactly confounded with marker/camera mounts; their estimates and sigmas are null, not fitted fiction. The M5 trajectory's rotation axis is close to link2, so it does not establish an axial wrist station. The true rubber-pad centre is not resolved in the dark front frames. The measured closed terminal contact-region seam midpoint is explicitly **provisional**.

Planner targeting and camera geometry now use the identifiable effective chain, with a 10 mm provisional jaw uncertainty bound; firmware checks continue to use the vendor model. URDF camera TF and the M5=0 contact TCP were updated consistently and exercised through real robot_state_publisher in isolated offline tests.

## Chain and fitted parameters

Frames: CF origin is the floor below the front camera, x forward, y left, z up. Base/rotation-centre frame adds +0.275 m to CF x. All quantities below use M5=0 and the nominal link2 orientation gauge.

`E = S + L1 * e(270 - M3 + o3)`

`camera = E + R_yaw,link2(M4 - M3 + 90) * v_camera`

`jaw = E + R_yaw,link2(M4 - M3 + 90) * v_jaw`

| Quantity | Fit | Conditional 1 sigma (100 frame-block bootstraps) | Vendor / previous convention |
|---|---:|---:|---|
| Shoulder CF x | -124.40 mm | 0.98 mm | vendor shoulder x=47.5 mm in its own frame; previous frozen transform gave approx -153.5 mm CF |
| Shoulder CF z | 122.00 mm | 1.49 mm | vendor z=140 mm; previous mount gave 148.5 mm floor height |
| L1 | 177.357 mm | 0.406 mm | 177.500 mm, consistent within sigma |
| M3 offset o3 | -2.271 degrees | 0.290 degrees | 0 degrees |
| Physical L2 | unidentifiable | undefined/null | vendor 205 mm retained ONLY as wrist-coordinate gauge |
| Physical M4 zero | unidentifiable | undefined/null | nominal zero absorbed in mount vector and rotation |
| Effective camera link2 x from elbow | 151.21 mm | 1.81 mm | previous camera-at-wrist convention 205 mm |
| Effective camera link2 z from elbow | 6.85 mm | 0.92 mm | previous convention 0 mm |

Shoulder in rotation-centre/base frame: (150.60, 24.31, 122.00) mm. Shoulder y is convention-dependent: the camera's lateral vector is fixed to zero to define the arm plane. Yaw is -0.131 degrees with conditional sigma 0.073 degrees. Nominal-wrist camera offset is (-53.79, -0.00, 6.85) mm; mount rotation is the full matrix frozen in YAML (nominal mechanical roll/pitch/yaw approximately +0.125/-2.212/+0.106 degrees). These are mount-gauge values, not independently measured camera-to-physical-wrist dimensions.

Gripper corner **2-D point RMS = 5.58 px** (per-coordinate RMS 3.94 px). Same-data vendor-L1/zero-M3 model with optimised origin/free camera mount: **8.21 px** point RMS. Whole-pose-held-out fold RMSs: 5.96, 4.32, 4.88, 7.15, 5.30 px. Front board floor-constrained point RMS: 8.35 px. Independent marker positional point RMS: 2.48 mm. Old 13.1/7.1 px fits used different frames/models and are not a direct held-out comparison.

Sigmas are conditional: they exclude front intrinsics' elevated-image extrapolation, floor-board nonplanarity/height uncertainty, printed side tolerance and exact pad landmark choice. The floor-constrained front-board residual is noticeably larger than a free planar PnP; do not treat the sub-mm bootstrap values as absolute physical accuracy. Numpy covariance calculations emitted runtime warnings; retained parameters and reported sigmas are finite, and frame-bootstrap values are used here rather than relying on the gauge-singular covariance.

## Jaw measurement

Known marker poses and five front views at M5=0,10,20,30,40 degrees triangulate the same provisional closed contact-region midpoint; this is **not silhouette scaling**. Contact seam annotation uncertainty is 6-9 px. Close crops and fit overlays confirm the point lies on the terminal jaw region, but do not expose an exact rubber-pad centre.

- In ArUco marker coordinates (OpenCV square axes): (1.87, 84.75, -23.53) mm; conditional annotation sigmas (1.78, 4.36, 3.91) mm.
- Effective link2 vector from elbow: **(202.00, -0.30, 26.65) mm**.
- Relative to the **nominal 205 mm vendor wrist endpoint**, with link2 x along link, y left, z perpendicular/up: **(-3.00, -0.30, 26.65) mm**. This is NOT an independently registered offset to the real wrist motor.
- Jaw annotation fit coordinate RMS 4.41 px; point RMS 6.24 px.
- Conservative provisional contact-region bound: +/-10 mm. The actual pad centre and the lowest open-finger tip remain unresolved; this bound is not a certified collision guarantee.

| M3,M4 | Jaw x ahead of rotation centre | Jaw y left | Jaw height |
|---|---:|---:|---:|
| [240, 105] | 469.3 mm | 23.3 mm | 80.5 mm |
| [230, 95] | 452.6 mm | 23.3 mm | 106.5 mm |
| [250, 115] | 481.2 mm | 23.3 mm | 52.0 mm |

The former (240,105) contact estimate was roughly 3.6 cm; this run predicts about 8.1 cm. That is consistent with jaws closing several centimetres too high. Independent direct marker-based reconstructions at that pose are about 8.3-8.6 cm, within the provisional bound. At the actual closed measurement pose (250,115), the directly triangulated closing region was around 5.0-5.2 cm high; no floor contact was visible.

## Equator pose recommendations (offline only)

For a 33 mm-high equator 0.45-0.50 m ahead of the rotation centre:

- Continuous fit at x=0.450 m: M3=246.45, M4=100.11 degrees, z=33.0 mm. **Not executable as an absolute target with the known 10-degree jog RPC.** Vendor wrist x=0.324 m; within conservative firmware reach.
- Continuous fit at x=0.475 m: M3=252.84, M4=114.00 degrees, z=33.0 mm. **Not executable as an absolute target with the known 10-degree jog RPC.** Vendor wrist x=0.352 m; within conservative firmware reach.
- Continuous fit at x=0.500 m: M3=261.71, M4=132.32 degrees, z=33.0 mm. **Not executable as an absolute target with the known 10-degree jog RPC.** Vendor wrist x=0.382 m; OUTSIDE conservative firmware reach.

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
