# Physical arm-base placement calibration

**Status: planning only; no hardware acquisition or model change authorized by this document.**

## Decision to make

Determine the physical shoulder pitch-axis relative to the front camera and the physical arm pedestal relative to the bumper/body. Derive a defensible model correction, rather than moving the arm backwards until Foxglove looks plausible. Separate three questions:

1. Where is the **M3 shoulder pitch-axis line** relative to the front optical frame?
2. Where is the **pedestal/arm deployment base**, including any translating deployment mechanism, relative to the real bumper?
3. Where should those features be drawn relative to `base_footprint`? Its assumed chassis rotation origin requires separate evidence.

A shoulder rotation trajectory answers (1), not automatically (2) or (3). “About four inches behind the nose” is an owner observation to check against named landmarks, not an exact parameter to impose.

## Baseline audit: source, not fresh hardware observations

| Item | Current source/evidence | Interpretation |
|---|---|---|
| Effective shoulder in `base_footprint` | `mount_calibration.yaml`: `(0.1506013824, 0.0243115007, 0.1220034233)` m | Existing constrained effective-chain fit; not surveyed pedestal anatomy. Lateral coordinate is gauge-dependent. |
| Front camera | Same file: x = `0.275` m; z = `0.0683558624` m; pitch down = `2.6403593°` | Height/pitch from floor calibration; x from chassis VO/rotation fit, not independent physical survey. |
| Camera → effective shoulder longitudinal setback | `0.275 − 0.1506013824 = 0.1243986176` m = **4.90 in** | Existing fit already puts shoulder well behind camera; absolute base-origin x cancels in this difference. |
| Drawn body/nose | `z70.urdf.xacro`: cylinder radius `0.175` m, height `0.08` m, explicitly unverified | Drawn front x = `0.175` m: shoulder only **24.40 mm / 0.96 in** behind drawn nose. Camera is **100 mm outside** drawn cylinder. This inconsistency can explain the appearance without proving a shoulder error. |
| Upper link | `measured_L1_m = 0.17735678465` | Effective fit, consistent with rough first-assembly tape; still requires axis-based validation. |
| Second assembly display | Open longitudinal envelope `0.1778` m; provisional housing/finger split | Owner's rough exterior measurement, not axis-to-axis length. |
| Nominal wrist/M5 TF | `L2 = 0.205` m; physical L2, M4 zero and M5 axial station null | Vendor-coordinate gauge retained; not measured wrist anatomy. |
| Jaw endpoint | Effective elbow-relative `[0.165, 0.00118885, 0.02]` m | Contact-region surrogate, **not** a direct 165-mm tape measurement or exact pad centre. |
| Pincher camera/M5 | Effective mount; positive physical roll observed over nominal 0–40° | Camera moves with M5; historical sweep showed direction-dependent slack of about 3°. Do not freeze camera through roll. |
| Elevated target | Historical observed ID68, expected ID69; black side 15 mm on 20-mm paper | Mounted on pincher, not registered physical wrist. Verify current black square, rigidity and attachment before metric use. |

Read these references before implementing the offline analyzer:

- [Current mount configuration](../../src/z70_description/config/mount_calibration.yaml) and [xacro](../../src/z70_description/urdf/z70.urdf.xacro).
- [Retained owner tape originals and visual audit](../../calibration/physical-measurements/20261014-owner-tape/README.md): tape wraps housings; rough ~7-inch assemblies and ~2.5-inch hook-to-housing landmark do not locate joint centres.
- [Front calibration](../../calibration/front-camera/README.md), [parameters](../../calibration/front-camera/front_camera.json), and its `detect.py`, `fit.py`, `export.py`, `uncertainty.py`. Original fit used one coplanar image, primarily rows 758–1188; elevated-marker intrinsics are extrapolated. Historical floor validation is not full-image validation.
- [Effective-chain results](../../calibration/arm-calibration/20261014T135228Z/RESULTS.md), `analyze.py`, `fit.json`: L1/shoulder fit; camera-y fixed to define an arm plane; whole-pose holdouts; conditional uncertainties exclude systematics. Pincher point RMS 5.58 px and floor-constrained front RMS 8.35 px are historical diagnostics, not adoption limits.
- [Rotation results](../../calibration/arm-calibration/20261014T144906Z/RESULTS.md), `analyze.py`, `fit.json`: single-joint screw-axis analysis, ID68 registration, partial/refused jogs, M5 movement; new chain not adopted because floor registration worsened (15.72 px front RMS).
- [Earlier fitter](../../calibration/arm-calibration/fit.py): useful detection/projection scaffolding, **not** an authoritative model. It includes an obsolete M5 pitch interpretation and treats commanded/estimated joints as alternatives. Do not rerun it unchanged and adopt its output.
- [Shared camera mode](shared-camera-mode.md) and current `planner.py`, description TF tests and manipulation tests for later integration.

Historical reports describe their dated states; current configuration/source governs the baseline. No fresh `/robot_description`, TF, camera or robot state was queried for this plan.

## Frames, parameters and identifiability

Use metres/radians internally. `T_A_B` maps B coordinates into A. Define:

- **W**: fixed metric floor-board frame, origin and axes at named surveyed board corners, z up. Multiple boards have separately surveyed fixed `T_W_Bk`, including substrate height/flatness.
- **F/G**: delivered front/pincher optical frames (x right, y down, z forward).
- **M**: elevated ID68 square frame with documented corner order and measured black side.
- **C**: physical chassis landmark frame, defined by surveyed body/bumper landmarks, x forward/y left/z up; do not call it the rotation centre without evidence.
- **S/E/R**: shoulder pitch, elbow pitch and pincher roll **axis lines**. A selected axis-frame origin is a convention until a physical station/centre is independently registered.
- **P**: pedestal landmarks, independently surveyed; deployment state belongs in their record.

Primary outputs are `T_F_S` (line representation plus explicitly defined reference point), camera-to-pedestal and bumper-to-pedestal/shoulder distances, and calibrated effective optical/marker chains with uncertainty. Fit front/body pose per stationary acquisition block so body shift/tilt is not absorbed as changed arm anatomy.

| Unknown | What can identify it | Remaining ambiguity / required fallback |
|---|---|---|
| Shoulder and elbow axis directions/lines | Pure M3/M4 rotations of rigid ID68; independently, moving pincher-camera poses from fixed board | Axis points can translate along their own axis. Report line and closest-point convention, not a fake physical joint-centre y. |
| Upper-link geometry | Elbow axes carried through several independent M3 rotations; shortest distance/common normal between axes | Axis-centre station or nonparallel-axis anatomy needs survey. Check parallelism before imposing planar geometry. |
| Fixed marker and pincher-camera mounts | Full SE(3) trajectories, alternating stationary views and joint factorization | Unknown rigid mounts can be jointly fitted; this does not make every mechanical parameter identifiable. Attachment to an articulating finger can invalidate rigidity. |
| Physical L2 / wrist station | Independent surveyed axial landmark or registered wrist-axis fixture | Free camera/marker translation along link2 absorbs L2. M5 sweep determines a line, not its axial station. Keep null without the reference. |
| Physical M4 zero | Independently defined link orientation/zero landmark | Trades with free mount rotation; M5-line alignment supplies a conditional convention, not automatically physical zero. |
| Shoulder lateral centre / pedestal | Surveyed shoulder ends, pedestal landmarks or direct orthogonal measurements | A marker-only fit with camera-y fixed cannot prove physical shoulder y. |
| Camera → shoulder setback | Joint-axis observations in F, metric marker/boards, independently validated front intrinsics | Does **not** require the base rotation origin. Optical centre differs from an exterior lens/housing landmark; survey must bound that offset. |
| Bumper/nose relationship | Independent body landmarks and perpendicular bumper/lens/pedestal survey | A board plus moving marker alone does not locate the bumper. A cylinder radius prior is not a measurement. |
| Rotation centre → front camera | Separately authorized observed chassis rotations in W, or independent mechanical survey | Stationary arm pictures cannot establish the chassis rotation centre. VO x=0.275 remains provisional; do not force a new radius to agree with it. |

Do not invent axes from the rough photographs. If an M3 sweep locates a raised moving shoulder but excludes a translating deployed base joint, explicitly model separate deployed configurations or obtain independent pedestal/deployment landmark measurements.

## Executable phases

Future live work requires one explicit authorization covering the supervised acquisition. Offline preparation precedes deployment; this plan itself grants none. No pick/grip/lift/chassis program is needed for the principal sweep.

| Phase | Procedure | Durable exit artifact / decision |
|---|---|---|
| 0 — Offline preparation | Inventory archived calibrations and images; verify available OpenCV/NumPy/SciPy environment without assuming the obsolete README interpreter exists. Specify detector, logger, safe incremental route, held-out blocks and numerical acceptance policy before live work. | `session.json`, baseline hashes, declared tolerances/splits, acquisition checklist. No hardware call. |
| 1 — Target and body survey | Measure black-square side in both directions with tolerance, verify dictionary/observed ID and rigid planar attachment clear of fingers/joints. Survey board spacing/scale/height and body/pedestal/bumper landmarks. Record deployable versus fixed pedestal parts. | Target registry, setup photos and `survey.json` with endpoints, method and uncertainty; unresolved values null. |
| 2 — Optical validation | Collect independent multi-tilt/depth board views covering elevated-marker image region for F and usable G field. Keep principal physical sweep boards fixed; use a separate movable calibration board or finish this phase before fixing W. Validate held-out views/distances. | Frozen per-stream intrinsics, covariance/systematic ranges, overlays and held-out lens report. Existing lenses are initializations, not proof. |
| 3 — Visibility pilot | Select high, clear arm poses with front floor+ID68 visibility and pincher floor coverage. Verify actual stationary images before assigning pose blocks. Observe deployment/body change; never infer deployment from an empty front view. | Visibility/coverage map, settled-noise estimate, executable incremental route using observed states. |
| 4 — Independent sweeps | Pure M4 sweeps at several M3 settings; pure M3 sweeps at several locked M4 settings; M5 sweeps at several M3/M4 settings. Log forward/reverse approaches and actual trajectories. Alternate F/G views only while stationary. | Complete raw log, accepted/rejected observations, axis fits and observed-angle traces. |
| 5 — Mixed-pose/replication | Independent M3/M4 pose lattice, asymmetric/interleaved approaches, repeat references. Repeat on three independently established sessions, including redeployment and a changed chassis placement relative to the still-surveyed boards. Remount ID68 in one recorded replicate to test mount independence. | Whole-pose and whole-session holdouts, cross-session shoulder/pedestal estimates and mount changes. |
| 6 — Offline estimation | Model-free trajectory/axis initialization, joint pixel-space fit, gauge/rank assessment, multistart, held-out prediction and systematic sensitivity. Reconcile survey and trajectories without imposing owner estimate as truth. | Reproducible fit, equivalence classes, uncertainty budget, pass/fail/inconclusive report. |
| 7 — Proposed adoption | If accepted, specify coordinated physical visuals/calibration/planner changes and separate nominal vendor model. Exercise expanded URDF and actual TF against fitted poses in isolated offline validation before any deployment. | Proposed patch inventory and integration report. Implementation/deployment is a separate task. |

### Acquisition coverage and order

Target **at least 20 distinct M3/M4 combinations**, not one diagonal: at least four shoulder settings and three elbow settings, with safe additional off-diagonal poses. For axis conditioning aim for five observed positions spanning at least ~40° per independent sweep when reachable/visible; use several M4 sweeps at different shoulder settings and several M3 sweeps at different elbow settings. These are coverage goals, not permission to exceed clearance/reach. Report reduced angular span and its uncertainty consequence.

At each of at least three well-separated arm configurations, sample nominal M5 0/10/20/30/40 and the return sequence within the previously measured range, if clear. Keep jaw state fixed; establish whether the marker is on a rigid roll member or moving jaw. Do not close/open merely to preserve a pose estimate. Repeat at least three reference configurations from both approach directions; retain two stationary exposures per view and repeat the relevant blocks across the three sessions. Extra captures at one pose do not replace missing joint excitation.

Before fitting, assign roughly one quarter of distinct pose combinations, one sweep/approach-direction block and the third session to holdout. All frames of a pose/trial belong to the same split. Include extreme and interior poses; leave some combinations absent from training, not merely random corners from every image.

For each stationary block use **F-before → G → F-after**:

1. Front frames see **floor boards and elevated ID68 in the same exposure**. Board pose gives `T_W_F`; marker pose gives `T_F_M`; their product gives marker pose in W. Re-estimate front 6-DoF pose from current floor geometry, not a historical fixed pitch/height assumption.
2. Pincher views provide `T_W_G` from the same fixed board network. F and G share one selectable stream; they are **not simultaneous exposures**. Record selection and actual capture provenance. Historical streams are 1600×1200 F / 1600×896 G, but dimensions alone are not camera identity.
3. Verify F-before/F-after body/marker stationarity in translation **and orientation**, relative to the pilot's observed noise. If drift, stow or autonomous change occurs, do not combine views as one pose; create separate blocks or retain unpaired data. No fabricated synchronized extrinsics.

Use front-only observations where pincher boards are occluded, and pincher-only observations where marker is lost; name the reduced constraint. Missing floor visibility means no direct same-exposure W registration, not a license to call an old camera pose measured. Recover visibility or use independently surveyed references; uncertainty and coverage determine whether results remain conclusive.

Commands are inputs, not measured angles: historical `app_arm_move {m:[a,b,c]}` uses 0/no move, 1/nominal +10°, 2/nominal −10°; −1 was ignored. Future executor must verify current tool semantics. Dispatch one bounded step, inspect outcome and settled image trajectory before the next uncertain step. Log ACK/error/timeout separately from observed angular displacement, rejected/partial jogs and direction-dependent slack. No absolute joint target capability is assumed; no unobserved multi-command measurement batch or blind compensating move.

**Stops are hazards only:** robot error, uncommanded/oversized motion, obstacle/person/pet in path, unexpected clean/dock, or operator stop. A missing frame, rejected fit/jog, marker ambiguity, state 39 or estimate UNKNOWN is a data gap, not automatic trial abort. Observe and recover/skip within the authorized clear workspace; never replay uncertain motion blindly. If sufficient evidence remains unavailable, finish with an inconclusive calibration. Observe final stow and selected-camera/preview cleanup; ACK alone proves neither motion nor cleanup.

### Evidence contract

Retain everything under `calibration/arm-calibration/<session>-physical-base/`, never scratch or private service state. Unique sequence filenames prevent overwrites; images remain git-ignored. Preserve originals and hashes, including rejected samples. Minimum artifacts:

- `session.json`: frame definitions, intrinsics/board/target provenance, surveyed tolerance, route/splits/thresholds and software versions.
- `survey.json`: raw readings, named landmarks, independent versus fitting use, deployment state, optical-centre/housing offset bounds.
- `log.jsonl`: command and result, nominal counts, estimator explicitly labelled prediction, approach direction, capture request/receipt and exposure timestamp if actually available, camera identity/size, pose/session/block ID, observed stationarity and rejection reasons.
- `observations.json`: raw IDs/corners, board-specific masks, target identity/size, PnP candidate solutions, quality, actual camera/marker transforms and covariance. Keep expected69/observed68 separately; no borrowed ID0 mount.
- `fit.json`, `holdout.json`, `uncertainty.json`, overlays, `RESULTS.md` and an offline reproduction command. Count unique files/poses/steps programmatically and report attempted/retained/accepted/rejected separately.

## Estimation specification

### 1. Detection and metric reconstruction

Use exact board layouts: historical DICT_4X4_1000, 10×14 squares at 20/15 mm with legacy pattern; 5×7 at 40/30 mm modern layout. Verify current print dimensions rather than assuming sheets are true-scale. IDs can overlap between boards **and ID68**. Use board-region masks and known layout, followed by elevated-marker association from non-floor height, trajectory and attachment region. Do not identify the arm by ID alone or reuse a hard-coded historical row cutoff as universal geometry.

Inspect overlays for actual corners/planarity. If needed try minMarkers=1/refinement, brief upscale/CLAHE, and no-SUBPIX fallback. Compare planar PnP alternatives, positive depth, normals, full-board residuals and temporal consistency; narrow low-RANSAC-residual subsets can give mirror poses. Estimate stationary angular noise separately from translational settling; small reprojection residual does not prove accurate marker orientation.

Estimate F and G board poses freely in W first; allow actual board substrate z and body roll/pitch. Diagnose discrepancy with historical floor constraints rather than forcing an erroneous fixed floor height into shoulder geometry.

### 2. Model-free axes before anatomy

From two observed rigid-marker poses in a common F/body frame during a pure joint step, form `Delta = T_2 * inverse(T_1) = [Q,t]`. A zero-pitch rotation satisfies:

`(I − Q) c = t`, with axis direction `n` the unit eigenvector of Q at eigenvalue 1.

Fit a common line `c + lambda*n` across the whole sweep, not one noisy pair. Report smallest-angle sensitivity, screw translation along n and rotation magnitude from images; circle fits to marker centres provide a complementary check. A single circle centre is the axis point at the marker's axial station, **not** the physical joint centre.

Pure M4 sweeps locate E; pure M3 sweeps with other joints fixed locate S even with unknown fixed marker offset. Track E under M3 to constrain L1 and axis relationship. Repeat using G board poses as a complementary sensor. Correct body shift through W observations before attributing world motion to the arm. Nonzero screw pitch, inconsistent lines or deployment translation triggers a richer mechanism hypothesis/survey, not a forced planar fit. M5 provides a roll-axis line and mount lever arm, never its unmeasured station.

### 3. Joint fit and gauges

Fit observed joint rotations as latent per-pose states initialized from marker/camera rotation, not exact nominal counts. Commands supply weak gain/backlash/direction constraints; allow partial/rejected steps. Unknowns include axes/effective L1, fixed SE(3) mounts of M and G, per-block chassis pose, deployment-state offsets if supported, and bounded optical/board uncertainties. Start with measured nonparallel axes; retain a planar reduced chain only if residuals and held-out tests support it.

Minimize a robust, uncertainty-whitened objective:

`sum rho(||pixel_observed − project(K,D,T,X_board_or_marker)||² / sigma_pixel²)`

`+ surveyed-landmark residuals weighted by survey covariance`

`+ SE(3) stationary/trajectory residuals and weak observed-step/gain constraints`.

Use all marker corners and board corners in both views; PnP transforms initialize but are not independent duplicate observations of those same pixels. Balance per-pose/camera contributions so dense boards do not overwhelm four marker corners. Publish loss, scale, constraints and all rejected observations; inspect residual patterns by image row, range, joint and direction.

Fix W rigid gauge explicitly. Represent axis lines without inventing along-axis points; choose and label reference conventions only where needed by TF. Report observable camera/elbow/marker vectors separately from physical L2, physical M4 zero and physical M5 station. Do not constrain marker-to-wrist from a guessed “one centimetre” mount or fit shoulder y by fixing an unmeasured camera-y and then call it physical.

Compute the dimensionless, whitened Jacobian SVD with disclosed parameter scaling, singular-value spectrum/rank criterion and null vectors. Profile weak parameters and run multiple initializations spanning old fits and surveyed bounds. Exhibit representative equivalent solutions for L2/mount and zero/mount gauges; a pseudoinverse covariance or narrow bound is not evidence those dimensions were identified. Report null/interval/equivalence class, not fabricated anatomy sigmas.

### 4. Validation and uncertainty

- Hold out **whole poses, sweep blocks/directions and sessions**. Evaluate 2-D point RMS/p95/max, observed marker/camera pose errors, axis consistency and survey-distance errors. Do not select the best fit using final holdout outcomes.
- For geometry tests use independent observed angles or reference reconstruction, then test the other view; distinguish this conditional geometry accuracy from command-only angle/dead-reckoning prediction accuracy. Do not optimize a held-out marker mount/angles using the same marker pixels and report those pixels as prediction.
- Bootstrap whole pose/trial blocks (target 200 successful refits, report actual count/failures), with session-level variation. Random individual-corner bootstrap underestimates shared uncertainty.
- Perturb intrinsics jointly over calibration uncertainty/model alternatives, marker side measurement, mount flex/remount, board scale/flatness/substrate height, body tilt/shift, timestamp/stationarity errors and surveyed optical-centre offsets. Report conditional random uncertainty separately from systematic sensitivity and cross-session scatter.
- Validate held-out ruler/checker distances or surveyed pedestal/bumper landmarks **not used as fit constraints**. Include independent direct camera-to-shoulder/pedestal longitudinal/height readings using straightedge, square/plumb reference and repeated readings; avoid bent-tape housing routes. Exterior lens centre is not automatically optical centre. If an axis is concealed, measure clearly named housing landmarks with bounded axis offset or use a registered fixture; mark missing anatomical proof inconclusive.

## Predeclared adoption tests

Before the main sweep, freeze the numerical policy using the independent optical pilot/survey capability. Desired placement accuracy is **5 mm translation / 1° axis orientation**, but these are engineering goals, not magic proof thresholds. Claim that precision only if independent metrology and the systematic-inclusive uncertainty support it; the historical 15-mm marker/floor-only lenses may not. If pilot accuracy cannot support the goal, improve targets/optics/survey or report inconclusive at that goal. Any revised requirement is an explicit new scope, never a silently weakened pass.

| Test | Acceptance rule |
|---|---|
| Identity/scale | Current marker size/rigidity and board registry verified; repeated IDs cannot contaminate board fit. No unresolved PnP branch used as fact. |
| Excitation/identifiability | Independent rotations and direction coverage present; SVD/profile/multistart support each claimed parameter. Null dimensions stay labelled/null. |
| Independent physical placement | Held-out surveyed setbacks/heights agree within combined uncertainty; systematic-inclusive 95% error/uncertainty bounds meet 5 mm/1° if claiming that accuracy. Define shoulder axis reference versus pedestal point explicitly. |
| Pose/session generalization | Frozen-model whole-pose and third-session predictions meet predeclared bounds. Report median/p95/max and coverage, not only aggregate training RMS. Hysteresis and redeployment scatter are included, not averaged away. |
| Pixel fit/overlays | Set camera/region-specific pixel bounds from independent pilot noise and projected metric tolerance before sweep. No structured residuals or unexplained branch flips. Historical 5–16 px residuals are warnings, not automatic allowances. |
| Bumper/body drawing | Independent bumper/pedestal geometry, camera housing and body outline measurements support the drawn landmarks/envelope. Passing camera-marker agreement alone is insufficient. Do not tune body radius to 0.275. |
| Chassis-origin claim | Separate survey/rotation evidence supports rotation-centre transform; otherwise retain clearly labelled provisional origin and publish camera-relative physical correction only. |
| Integration | Fitted chain, expanded URDF, real TF and planner optical/marker/TCP outputs agree at multiple poses and M5 angles. Distinguish algebraic checks from measured roll coverage. Vendor predictive FK remains unchanged. |

Report **PASS** only for named quantities satisfying their full tests; **FAIL** for an observed contradiction; **INCONCLUSIVE** for insufficient visibility, rank, physical landmark evidence or uncertainty. Partial success (e.g. camera-to-axis setback) does not license claims about pedestal, bumper, rotation centre, L2 or pads.

## Adoption package, if justified later

1. Report old/new camera-relative axis and surveyed pedestal/bumper placement with intervals, raw landmark definitions, held-out results and unresolved gauges. Explain separately whether discrepancy came from body placeholder, base-origin convention, shoulder chain, deployment translation or several causes.
2. Propose a physical pedestal/shoulder visual and measured body outline; preserve unmeasured dimensions as provisional. Do not move the physical pedestal to the shoulder pitch-axis merely because one TF frame exists.
3. Update shared `mount_calibration.yaml` and xacro consistently with the effective camera/marker/TCP chain and planner. Derive `arm_origin_*` from the same shoulder transform/conventions; avoid retaining competing old/new shoulder constants. Changing a nominal gauge requires compensating all dependent mounts. Contact pad registration remains a separately labelled surrogate unless independently measured.
4. Keep `src/arm-model.ts` and `arm_model.py` vendor FK/IK/reach checks separate and unchanged by this calibration; physical observations do not establish revised firmware command semantics. Re-evaluate planner clearance against measured body/pedestal geometry and uncertainty without presenting old nominal reach as anatomical proof.
5. In a separately authorized implementation, add fit-versus-expanded-URDF/real-TF regressions across held-out poses/M5 settings and run the full ROS suite. Offline algebraic TF checks do not prove physical accuracy. Only a later scoped visualization deployment/readback of exact latched `/robot_description` proves Foxglove has loaded the correction; no Docker/camera/control-stack restart is implied.

**Deliverable of this task is this plan only.** Acquisition, analyzer implementation, code changes, process launches and deployment remain future work.

## Separately authorized execution evidence

The owner subsequently authorized supervised acquisition and offline reconstruction. Results: [20261015T061415Z physical-base acquisition and reconstruction](../../calibration/arm-calibration/20261015T061415Z-physical-base/RESULTS.md); reusable offline analyzer/tests: [physical_base](../../calibration/arm-calibration/physical_base/README.md).

The run retained 195 originals, centred the board, measured independent pitch/roll sweeps and fit conditional camera-relative axis lines. Absolute 5 mm / 1° placement and pedestal/bumper anatomy remain inconclusive; cross-method/redeployment discrepancies prevent adoption. No production geometry or firmware changed. This execution link does not change the original authoring boundary or weaken the adoption tests above.
