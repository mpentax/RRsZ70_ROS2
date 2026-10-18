# Second live arm calibration — rotations and closed fingers

## Outcome

**Arm STOWED; M5 camera model corrected; physical L2 and exact jaw closing point NOT resolved.** This is a partial anatomy calibration, not a verified grasp. No chassis motion, pick/lift goals, ROS bringup, service restart or commit.

60 capture calls produced **59 retained raw frames (23 front, 36 pincher), zero failed camera reads**. Analysis uses 27 M5=0 board frames for the effective chain, 34 total pincher-board frames, 18 front-board frames, and 11 elevated-marker views. Counts are deduplicated from durable log/file paths.

The wrist target consistently decodes **ID68**, including with error correction disabled. The owner-reported **ID69 is preserved as the expected ID**, not silently normalized. Its 15 mm black side was used; its mount was re-registered from current views, never copied from ID0. The front upper-region marker mask separates this elevated target from floor-board identities.

## Measured rotations

- **M5 DOES move the pincher camera.** Nominal outward M5=10/20/30/40 gave 6.54/16.99/27.97/38.93 deg camera rotation. Return30/20/10 gave30.41/19.14/8.18 deg. These are board PnP, corroborated by the independently moving front marker. Nominal positive physical roll is about link2 **+x**. About3 deg direction-dependent slack/zero discrepancy remains; do not claim encoder feedback or a precise gain outside0..40.
- At nominal M5=40, lens translation was **4.883 mm**. Multi-view screw fit: camera-to-M5-axis perpendicular offset **7.487±0.117 mm** (800 frame resamples, zero reference held fixed; systematics excluded), translation point RMS **0.303 mm**. Axis CF direction is [0.738904044306399, 0.00020508402980830436, -0.6738106345615124]; its component frame scatter is [0.0017438701586170503, 0.002619260279644639, 0.0019169610579610709] (conditional, not absolute sigma).
- M3:13 single-joint pairs, step mean9.717 deg, step SD0.218 deg (mean sigma0.060); gain0.9717.
- M4:8 single-joint pairs, step mean9.652 deg, step SD0.265 deg (mean sigma0.094); gain0.9652. Camera-to-elbow axis perpendicular distance **161.84 mm**, pair SD **6.04 mm**. This is camera-to-axis distance, NOT physical L2.

## Conditional effective chain fit (not adopted)

Frames: CF is the floor below the front lens, x forward/y left/z up. Base/rotation-centre adds0.275m to CF x. Gains are anchored at deployed M3=150,M4=45. Link2 angle gauge is `M4cal-M3cal+90`; upper link angle is `270-M3cal+o3`.

| Parameter | Fit | Conditional frame-bootstrap 1sigma |
|---|---:|---:|
| Shoulder CF x | -128.61 mm | 4.97 mm |
| Shoulder CF z | 126.55 mm | 1.59 mm |
| L1 | 180.29 mm | 2.12 mm |
| M3 upper-link offset | -1.163 deg | 0.862 deg |
| M3 gain, global fit | 0.98282 | 0.01194 |
| M4 gain, global fit | 0.95818 | 0.01136 |
| Camera from elbow, effective x | 164.45 mm | 5.07 mm |
| Camera from elbow, effective z | 7.96 mm | 2.05 mm |
| Physical L2 | unknown/null | undefined/null |
| Unconditional physical M4 zero | unknown/null | undefined/null |
| Exact rubber-pad centre | unknown/null | undefined/null |

Gripper point RMS **4.285px**, versus **6.946px** for the retained old chain on the same new observations. Floor-constrained front-board point RMS **15.723px**.60 frame-block bootstraps. Bootstrap sigmas exclude front intrinsics' elevated extrapolation, printed-size/flatness and board/floor systematics. This worse absolute floor registration and unresolved pad point are why these new effective lengths/gains did **not** replace the prior M5=0 chain.

If physical link2 is DEFINED to align with the measured M5 roll axis, the conditional M4 bend zero is **4.765±0.389deg** (bootstrap only; several-degree inter-camera/systematic allowance excluded). This is a conditional alignment result, not independently verified anatomy, and is not adopted.

The observed ID68 marker effective vector from elbow is **(142.74, 0.26, 4.83)mm**, conditional bootstrap sigmas **[4.97, 0.27, 2.13]mm**, position point RMS **2.799mm**. Full current rotation matrix is in fit.json. This is an effective mount in the new fit's gauge, not a transform to a physically located wrist-axis origin.

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
| **240,85,0** | **(417.11,23.40,32.71)** | New provisional GRASP;33mm equator error -0.29mm |
| **230,85,0** | (428.56,23.37,80.35) | Near hover; single M3+ jog toGRASP |
| 240,95,0 | (445.28,23.34,54.34) | Higher observation pose, not equator |
| 250,115,0 | (481.21,23.25,52.02) | Live observed closed measurement pose, not equator |
| old250,105,0 | (457.22,23.31,25.82) |7.18mm below equator; no longer nominal |

At near hover, the prospective object equator projects to **(803.3,513.2)px**, optical depth **82.3mm**. Full path/grid data are in recommendations.json.

**These are model predictions, not exercised grasps.** GRASP's ±10mm provisional lower bound is22.71mm, so this does NOT certify ≥3cm open-finger clearance. No retained grid pose both hits the33mm equator and has a ≥3cm conservative lower bound. Do not blindly run a pick on the strength of sub-mm nominal height error.

## Final object and robot state

Final front image99-stowed-front.jpg: yellow target-only bbox approximately(708,604)..(864,747), contact reference(784,744)±(8,5)px. HSV merged the yellow object with the wood and shadow; that merged centroid/contact was rejected. Reviewed visible-bottom floor projection gives **(0.5784,-0.0028,0)m from rotation centre**, i.e. **578.4mm ahead and 2.8mm right**. Pixel-only sigma approximately(8.5,2.9)mm; use at least±20..30mm for physical contact/centre ambiguity and floor-camera systematics. It is a visible-contact estimate, not a surveyed geometric centre. Dodecahedron/board were not touched or driven over.

Under that estimate and the provisional GRASP, nominal approach is **-2.59deg yaw, then 160.9mm forward**; this is not a drive instruction or validated endpoint. Re-observe/align before closure.

Last arm motion was stow. Immediate settled status: state39,error0,common_status51330,bit256clear; front image has no gripper. Final fresh status after offline analysis: **state3,battery23%,error0,common_status49154,readiness bitclear**. Exact estimate readback: **IN_ROBOT**, jointsall0,wristnull,stepsSinceAnchor0. Status/model/image corroborate stow; no joint encoders are claimed.

## Adoption, tests and acquisition issues

Changed planner.camera_pose and URDF to include positive physical M5 roll; moved optical mount under the roll chain and used axis-x with the bridge's existing negative M5 joint values. M5=0 geometry remains identical. Shared YAML records new evidence, ID discrepancy and unresolved anatomy. Planner GRASP changes to240/85, with regression for33mm height and measured camera roll. Real robot_state_publisher tests compare the full roll-dependent optical transform. README and synthetic range test are updated. Vendor firmware FK/model constants are unchanged.

**Tests: requested manipulation command33passed; fullROS84passed in17.23s; git diff --check passed.** No commit. Pre-existing chassis online.json and unrelated untracked artifacts were not modified by this task.

- A200/45 M3 command only moved about1.12deg; it was not counted. The arm was stowed/redeployed to restore a known sequence. No reported robot error_code occurred. Later acquisition uses observed image pose changes, not the invalidated grip estimate.
- A helper's conservative height guard stopped the route at230/105; two subsequent captures were mistakenly named250-115. Their correction is explicit in log.jsonl and analyze.py. True250/115 captures use the-actual suffix. One zero-M5 pincher filename was overwritten by its return frame; retained counts are unique-file counts, not inflated capture counts.
- OpenCV5 RANSAC found low-residual mirror/partial planar solutions. Final extraction uses consistent direct iterative PnP, residual trimming and no-SUBPIX fallback when ChArUco interpolation returns0. Original images and rejected partial-jog frames are retained.
- app_grip returnedACK and only closed; app_open_gripper timed out normally. No lift was sent. Grip invalidated the old service estimate asUNKNOWN; independently tracked/observed joints were used, then stow restoredIN_ROBOT.

Evidence: log.jsonl,59 raw frames, selected overlays/crops, observations.json, fit.json, analysis-output.txt, recommendations.json, verification.json and reproducible acquire/analyze/recommend helpers.
