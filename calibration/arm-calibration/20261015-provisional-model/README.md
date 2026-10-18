> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [current arm model](../../../docs/z70-arm-model.md), [camera/calibration guide](../../../research/cameras/FINDINGS.md). Optical identifiability limits below apply to this experiment; copied CAD now supplies structural anatomy, not independent physical registration or contact validation.

# Shared provisional Z70 physical/optical model — 2026-10-15

**Delivered model, not a calibration proposal.** `z70_description/config/mount_calibration.yaml` now drives expanded URDF, real TF and planner geometry. `z70_gripper_camera_info.yaml` is the pincher lens source used by the planner. This is a visual-comparison/physical-preview model, **not a newly validated autonomous-pickup model**. No firmware FK/IK, nominal joint conversion, transport, live robot state, services or Foxglove instance was changed here. The two-line perception-node fallback synchronization in `synchronize-node-fallback.patch` is now applied and tested on the actual constructor, before any CameraInfo. The patch is retained as a record, not a pending step.

## Chosen geometry and its actual meaning

| Quantity | Display/shared value | Meaning / remaining limitation |
|---|---:|---|
| Body diameter | 355.6 mm | Rough owner exterior envelope, not a surveyed circle |
| Body visual centre x | 97.2 mm | Nose aligned to retained front lens x=275 mm; bumper-to-lens depth unknown. **Visual offset only**; odometry/base origin unchanged |
| Body height | 80 mm | Existing unmeasured illustration, not a new measurement |
| Shoulder setback from front optical origin | 126.483 mm | New pincher-lens axis fit, `N_M3_55/front_axis`, named camera optical x=0 station; lateral anatomical centre not determined |
| Shoulder rise above front optical origin | 68.197 mm | Same new axis fit; use this relative quantity rather than mixing floor-height registrations |
| Front camera height | 58.233433936 mm | New-run best `arm/front-registration.json`, not hard-constrained to rough60mm tape; same in active projector and ROS TF |
| Front camera pitch / roll | −1.358765985° / −0.415796432° | Observed floor normal gives slight pitch **up** (negative down); preserve measured roll, do not reuse old2.640° down |
| Displayed shoulder height | 126.430291524 mm | Same floor height58.233433936mm + observed rise68.196857589mm; rough owner shoulder127mm is a crosscheck |
| Shoulder station y | 0.516541246 mm | Named optical-x=0 shoulder-axis intersection projected into the same measured floor basis; **not** an identified anatomical lateral centre |
| L1 | 179.925 mm | Effective same-station pitch-axis separation; other primary blocks ~178.600/180.424 mm, not surveyed hidden joint centres |
| Open second exterior assembly | 177.8 mm | Existing rough tape envelope; not physical L2 |
| Nominal wrist/M5 station | 205 mm | **Firmware coordinate gauge only**, not measured anatomy |
| Elbow-to-lens perpendicular lever | 156.754 mm | Reconstructed as a full vector, not entered as an x-only radius |
| Lens vector in retained link2 zero gauge | (156.611, −1.126, 6.592) mm | Measured vector transported through an actually observed optical rotation; zero/mount orientation and axial station remain gauge choices |
| Marker ID68 centre in same gauge | (128.757, −1.639, −0.002) mm | Full new G→ID68 rigid registration under **retained front K/D**, not the rejected front-lens candidate |
| Terminal-region contact surrogate | (162.498, −1.129, 19.629) mm | Original marker-relative measured region preserved, re-expressed with marker/lens together; previous 165 mm was provisional, not measured exact pad anatomy |

The raised pedestal visibly connects the illustrative body top to the shoulder. Housing height 32 mm, housing centre z=16 mm, finger plane z=28 mm, pedestal width 55 mm, finger widths/hinges and lower lens enclosure are explicitly **unmeasured illustrative anatomy**. The lower lens enclosure is behind the blue lens and below the fingers; none of those visual offsets moves the calibrated optical frame. Preserve physical L2, M4 zero, M5 axial station and exact pad centre as null.

### One consistent new-run floor, not two gauges

The original active front JSON is archived byte-for-byte in `prior-front-camera.json` (SHA-256 `b1d00207c78d5f56da9781ce24cebb0719c6146a94b1812576365e62061d19ae`); its origin/hash note was written **before** replacing the active projector. Original raw calibration, lens files, fit artifacts and prior archives are untouched. Front K/D is unchanged; **only front floor extrinsics** are replaced in `calibration/front-camera/front_camera.json`, together with the ROS camera mount.

Source: `20261015T080103Z-pincher-intrinsics-arm/arm/front-registration.json`: best block `T_F_W`, camera height0.058233433935542395m,1332 residuals/21frames,3.242536374px RMS. Board +z points down in the chosen observation gauge, so `up_F=-T_F_W[:3,2]`. Project optical forward onto that plane: `forward_F=normalize(optical_z-up_F*up_F.z)`; `left_F=cross(up_F,forward_F)`. `R_floor_to_cam=[forward_F,left_F,up_F]` has x forward, y left, z up, **not the board's yaw**. Decompose its optical-to-robot inverse for the URDF; yaw0, pitch/roll above. Both projector and real optical TF now agree numerically.

The shoulder is `camera_origin + R_floor_to_cam.T @ measured_point_at_front_optical_x0`; this retains measured setback126.482724165mm and rise68.196857589mm while making displayed shoulder126.430291524mm. It also replaces the old arbitrary lateral gauge24.3115mm with the named station0.516541246mm. Arm origin is(101.017400632,0.625425351,−13.569708476)mm; nominal shoulder reference and retained arm yaw give the same shoulder station. Body radius177.8mm, body centre97.2mm and provisional camera x275mm remain unchanged.

Previously the preview retained camera68.355862410mm plus new rise, yielding shoulder136.552719998mm, while the new-run board implied126.430291524mm. That conflict is removed rather than hidden. This is a **PROVISIONAL physical-preview/shared-projector choice**, not precision certification: print scale/board flatness, pose/glare, retained front-lens systematics and prospective arm-chain validation remain unresolved. Rough owner camera60mm/shoulder127mm are independent plausibility checks, not fitted constraints.

## ID68 anatomy and the flat-pincher concern

`pincher_marker_68` is now an actual URDF/TF frame, not a wrist proxy. It carries a 15 mm black-square visual on a 20 mm paper backing. Its full rotation places the printed surface approximately toward **negative link2 z**, below the illustrative fingers, consistent with the owner's underside placement. Do not infer top/bottom from a single rolled camera view. Marker normal/orientation comes from the retained full rigid transform, not from flattening the housing.

The optical fit already allowed a free full `T_G_M` and used all four raw marker corners. Only the black printed square must be planar for square-marker PnP; the entire pincher need not be flat, and its unknown housing height did not constrain that free registration. The old model's missing marker visual/flat housing could mislead visual comparison, but fixing the drawing does **not** explain away the failed front-camera validation.

## Lens selection and honest validation status

Pincher delivered image: 1600×896, plumb_bob:

- fx=830.8441157256567, fy=830.2235617780392
- cx=800.4384886796951, cy=305.9484763166739
- D=[−0.08997298474623529, 0, 0, 0, 0]

Independent new G whole-pose holdout and historical board views supported this lens (reported ~1.668 vs2.253 px and ~2.293 vs4.310 px). Keep asymmetric fx/fy and distortion in all projections. ROS camera matrix and projection matrix are synchronized. Historical fit artifacts remain immutable.

**Do not adopt the rejected front fcyk1 lens.** Front lens K/D and CameraInfo YAML are unchanged; the front JSON floor height/normal are explicitly replaced as above. New G→marker registration is a useful provisional rigid mount, not independently generalized precision calibration. Original pitch holdout and the later prospective pitch/roll-neighbor comparison failed to validate that chain. Nominal route labels drifted after refused/partial jogs; this update does not fit new M3/M4 zeros to them. Existing M3 offset, yaw, nominal-angle optical orientation and M5 convention are retained as alignment gauges.

The historical nominal-FK optical checks are deliberately retained with their original **12 px** criterion and marked **strict expected failures** for this provisional geometry; do not hide them by widening the tolerance or call the suite a precision-validation pass. The three historical comparisons now differ by the residuals recorded in `verification.json`, an unresolved accuracy issue. The historical successful nominal grasp preset (250,95) is retained, not retuned or represented as newly hardware validated. Its new predicted contact height is42.243969 mm; real front-marker observation, not this prediction, remains the servo reference.

## Reproduction and verification

From `ros2/`:

```sh
~/.pixi/bin/pixi run test
```

Offline tests expand the actual xacro and run real robot_state_publisher/tf2 in isolated ROS_DOMAIN_ID91; no hardware bridge is started. Coverage includes real front optical TF/projector matrix and floor-ray agreement, correct z-up/down-ray signs and principal-ray heading, byte-preserved baseline/front K/D, no-CameraInfo node fallback, body/camera nose alignment, pedestal, full shoulder station, lens K/D/P synchronization, marker full G→M transform and underside/finger ordering, open assembly envelope, and camera/marker/TCP agreement through multiple M3/M4/M5 poses. The `model.json` records previous mount, chosen mount and exact source transform/vector, so the provisional change is reversible without touching raw history.

Observed suite result after these corrections: **105 passed, 3 strict expected failures**, with the expected failures explicitly limited to historical nominal-FK optical accuracy. These are not passing calibration results. Foxglove deployment and latched robot_description verification belong to the parent; none was claimed here.

Metric fixtures were updated, not runtime thresholds: predicted jaw heights now use the coherent floor gauge; the old24px “too small” blue fixture is actually29mm under this floor and is replaced by a genuinely sub18mm region. The near-grasp20° spin fixture projected its contact to u≈1660, outside the1600px frame; use a fully visible11.5° spin while retaining its >10° realignment, no-track-gap and before-descent assertions. Historical optical checks keep the original12px limit and remain3 strict expected failures. No autonomous controller, runtime tolerances or vendor joint angles were changed.

## Parent visualization deployment

The parent independently rebuilt and ran the ROS suite: **105 passed, 3 strict expected failures in24.28s**. Started only local `robot_state_publisher`, Foxglove on `ws://127.0.0.1:8765`, and `/z70/model_preview/joint_states` in an explicitly simulated deployed/open pose (M3=150, M4=45, M5=0, M6=90). No robot or camera connection was opened.

Read back the exact live latched `/robot_description` and TF: body355.6mm, shoulder setback126.482724mm/height126.430292mm, camera height58.233434mm, L1179.924889mm. Verified body nose meets front-camera x, live front optical rotation equals the active floor projector, and full `gripper_camera_optical_frame → pincher_marker_68` transform equals the shared calibration. Details: `deployment-readback.json`. This establishes deployment consistency, not hardware accuracy. The preview is simulated, not live robot telemetry.
