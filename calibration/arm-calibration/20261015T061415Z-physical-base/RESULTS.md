# Physical arm-base acquisition and reconstruction — results

## Outcome

**Useful camera-relative shoulder-axis reconstruction obtained; no physical/model adoption.** The best centred block places the shoulder pitch-axis about **128.55 mm behind the front optical origin**, with an axis point **65.45 mm above that origin along the inferred floor-up direction**. Its height over the printed board plane is **124.00 mm**.

These are **conditional reconstructions, not certified physical truth**. Centre-only circles disagree with corner-space/screw fits by 12–27 mm; poorly framed A/B versus centred D lines differ 28–38 mm and more than 2°. Missing independent survey and lens/print validation prevent a 5 mm / 1° accuracy claim. **Production configuration, URDF, planner and firmware remain unchanged. No commit.**

## Acquired evidence

- **195 unique, retained original JPEGs**: **37 front / 158 pincher**, with **195 distinct SHA-256 hashes**, all verified against `manifest.json`. No overwrites. The directory is a local private archive, not a remote backup.
- **195 successful retained captures**: 192 logged through the existing MCP helper plus 3 direct-tool captures. No recorded `capture_failed` events; helper timestamps are completion/receipt bookkeeping, never invented exposure times.
- **31 distinct nominal M3/M4 labels**, **22 labels with a quality-eligible board pose**; the well-centred D block covers **21 quality-eligible M3/M4 labels**. They are not encoder angles. Actual rotations initialize/fill latent-angle fits; partial/refused jogs are retained.
- **186 board-pose frames**, **131 quality-eligible**, **64 rejected-quality or absent**. Eligibility requires at least 8 retained points, positive depth, 4 px initialization trimming and at least 65% inlier fraction. All raw/full-corner residuals remain; the final pixel fit uses all selected board points, not only trimmed inliers.
- **12 elevated ID68 front observations**. Expected69 and detected68 stay distinct. DICT_4X4_1000, legacy dense 10×14 squares at nominal 20/15 mm; **15 mm is the black marker side**, not 20 mm. Marker association uses non-floor height and inspected arm region, not ID alone. Front floor masking excludes the elevated repeated ID. Sparse board-marker corner fallback explicitly retains metric object coordinates and point type.
- **89 arm jog commands**, **9 bounded chassis pulses**, **4 deployments**. Two 400 ms reverse pulses improved range. Later centering used separate 600/800 ms turns with stop/readback between pulses; one 600 ms turn showed no appreciable execution and was not assumed completed. Chassis relocation/setup images are excluded from pure-joint groups.
- Three usable deployment blocks A/B/D; C was a framing-only redeployment. Chassis placement changed while the board stayed fixed. No human marker remount or independently established board survey session was supplied. Do not relabel these as three fully independent metrology sessions.
- Independent M3 and M4 sweeps, forward/reverse routes and repeats. The centred D sweeps span **39.05° M3** and **38.66° M4**, plus other locked-joint blocks. M5 nominal 0/10/20/30/40 and returns were acquired at **240/105**, **190/55**, and **200/65**; the first configuration has front-target coverage but incomplete pincher-board coverage.

No grip/pick/lift program, cleaning, docking, map navigation, ROS/camera poller, service rebuild or Docker restart was used. Every live observation/action was a RoboClaw MCP tool call; the existing Node helper used the MCP SDK, not direct robot/go2rtc access.

## Fitted observable geometry

`T_A_B` maps B into A. F is the delivered front optical frame: x right, y down, z forward. W is the fixed OpenCV printed dense-board frame; its normal sign is chosen toward the observed camera. Free front pose is fitted separately in each stationary chassis block.

The primary line is:

- direction F = `[0.9999993614132415, -0.0009272767862094581, 0.000646011509948771]`;
- point intersecting **front optical x=0 plane** = `[0.0, -0.06815030673882776, -0.12713680058823632]` m;
- closest-point convention and the full line are retained in `fit.json`. This reference is **not a physical shoulder lateral centre**; position along the pitch-axis stays unobserved.

| Independent centred block | Observed span ° | All-point training RMS px | Camera→axis setback mm | Axis above camera mm | Axis above board mm |
|---|---:|---:|---:|---:|---:|
| D_M3_45 | 29.52 | 1.82 | 130.93 | 68.84 | 127.39 |
| D_M3_85 | 39.05 | 2.71 | 128.55 | 65.45 | 124.00 |
| D_M3_95 | 29.58 | 2.71 | 127.68 | 64.12 | 122.67 |

The old source has setback **124.40 mm**, height above camera **53.65 mm**, and height above historical floor **122.00 mm**. The centred primary reconstruction differs by **+4.15 mm setback** and **+11.80 mm above-camera height**. New free board registration places the camera **58.55 mm** above the print plane, versus historical **68.36 mm**; that roughly 9.8 mm discrepancy is unresolved body-pose/plane/optical systematics, not a measured camera relocation. Do not force the historical height or call board-plane agreement physical proof.

Pure elbow-axis reconstructions are within **0.32–0.33°** of the shoulder direction. Their conditional nearly-parallel-axis distances are **182.45–183.30 mm**, versus source effective L1 **177.36 mm**. This is a diagnostic discrepancy, **not an adopted anatomical link length**.

The source already places the shoulder 124.4 mm behind the camera, while its drawn body nose is only 24.4 mm ahead of the shoulder and the camera is 100 mm outside the placeholder body cylinder. This remains a body/origin/landmark discrepancy, not proof the shoulder itself needs moving.

## Estimator and uncertainty

- Model-free full-SE(3) screw line first, then a robust equal-pose-weight pixel-space pure-joint fit with **latent observed angles**. No exact nominal angles, guessed wrist registration, physical L2, M4 zero or shoulder lateral centre is imposed.
- Primary pixel-fit dimensionless Jacobian is **rank 14 / 14** after explicitly removing the along-axis gauge; singular values and translation/rotation scaling are published. This local rank does not identify omitted anatomy. Three starts shifted 30–40 mm converge to the same local axis/result.
- Primary whole-pose **model-free** bootstrap: **194 successful / 200 attempted**, 6 insufficient-excitation failures. Conditional 1σ: **1.98 mm setback / 1.18 mm above-camera height**. Conditional bootstrap percentiles: setback **123.67–131.66 mm**; height **62.26–66.46 mm**. These are not 200 pixel-bundle refits or systematic-inclusive absolute confidence intervals.
- An assumed pincher focal change of ±1% moves setback about **±5.5 mm**. Principal-point/radial and front-lens scenarios, plus hypothetical ±1% print scale, are published in `validation.json`. **These are sensitivity scenarios, not measured tolerance bounds.** Unknown board flatness/scale, front elevated-region intrinsics, body tilt and remount variance remain outside a certified interval.
- Independent **position-only** camera-centre circles disagree with screw/pixel axes by **11.92 / 15.34 / 27.18 mm**, with direction differences **1.59 / 0.95 / 3.91°** across D blocks. This exposes small-arc/pose-position/systematic sensitivity rather than proving a precise axis from one favourable estimator.
- A/B versus D camera-relative reconstructions differ **28–38 mm / >2°**. The data cannot separate real redeployment change from edge-of-image lens/board-registration bias. Do not average away these discrepancies or attribute them to physical arm movement without survey.

Physical L2, physical M4 zero, M5 axial station, shoulder lateral centre, pedestal/bumper landmarks, exact pads and physical print tolerance remain **null**. Equivalent L2/mount examples are algebraic gauge demonstrations, not observations.

## Held-out and independent validation

Whole final poses are excluded from each local training fit. A separate stationary exposure supplies orientation only where available; its held-out translation and the first image's corners never enter training. Thus this is **conditional geometry prediction from observed rotation**, not command-only prediction or fully independent lens metrology.

| Held-out pure-joint block | Pixel RMS | p95 | max | Camera-centre disagreement |
|---|---:|---:|---:|---:|
| D_M3_45 | 2.88 px | 4.92 px | 6.02 px | 0.96 mm |
| D_M3_85 | 4.65 px | 5.77 px | 27.18 px | 0.52 mm |
| D_M3_95 | 7.47 px | 10.12 px | 11.34 px | 2.35 mm |
| D_M4_190 | 8.14 px | 10.07 px | 11.63 px | 3.66 mm |
| D_M4_210 | 4.30 px | 6.62 px | 9.40 px | 0.76 mm |
| D_M5_20065 | 3.82 px | 5.87 px | 12.20 px | 0.48 mm |

A fixed unknown pincher-camera→ID68 transform registered at D 240/95 predicts **different front-camera pixels** at D 240/105 from the pincher board pose: **4.67 px RMS**, **1.75 mm / 2.22°** marker-pose disagreement. **This pairing is rejected for calibration:** D front-before/after marker orientation varies **3.12°**, exceeding the frozen **3°** rule even though marker centre and board registration are stable. It is retained as a diagnostic, not a passed independent validation. A free rigid mount would cancel shoulder/L2 anyway, so camera-to-marker agreement cannot prove physical shoulder anatomy.

Front ID68 independently observes **38.04°** M5 outward rotation at nominal40; centred pincher-board fit observes **38.60°**. Direction differences at nominal10/20/30 are about **1.3–2.3°** in the board traces. This supports rolling-camera/marker behaviour, not exact motor angles or the physical wrist axial station.

The initial modulo-four split wording selected no poses on the half-offset elbow grid. `analysis-policy.json` preserves the issue and documents the before-first-fit whole-pose group split. D values are independently reconstructed later-block estimates, **not frozen A/B-model third-session predictions**. No final holdout outcome was used to choose an adopted model; no model is adopted.

## Verdicts

| Named test | Verdict | Basis |
|---|---|---|
| Durable evidence, ID decode/separation, bounded observed acquisition | **PASS** | 195 hash-verified originals; observed68 preserved; board and arm targets separated in inspected overlays |
| Conditional pure M3/M4 excitation and local axis fit | **PASS** | Independent locked-joint sweeps; roughly39° span; rank/multistart/actual-angle fit |
| M5 moves optical camera/target in tested range | **PASS** | Independent front and pincher trajectories, with measured approach hysteresis |
| Whole-redeployment 5 mm / 1° reproducibility goal | **FAIL** | A/B→D axes differ 28–38 mm and >2° |
| Cross-method 5 mm agreement | **FAIL** | Position-only circles versus screw/pixel axes differ12–27 mm |
| D paired front/pincher reference stationarity | **FAIL** | Marker angular variation3.12° exceeds frozen3° threshold; diagnostic retained |
| Absolute shoulder placement at5 mm /1° | **INCONCLUSIVE** | No independent physical survey/optical/print/remount bound; random bootstrap is insufficient |
| Pedestal/bumper/body outline/chassis rotation origin | **INCONCLUSIVE** | No independent named-landmark survey; placeholder drawing and VO origin not physical truth |
| Physical L2/M4 zero/M5 station/exact pads | **INCONCLUSIVE** | Unobservable gauges/landmarks remain null |
| Full global chain/generalization/integration adoption | **INCONCLUSIVE** | Pure-joint fits delivered; no full global bundle, remount, surveyed third-session chain prediction or production/TF adoption |

## Final state and verification

Fresh post-exit status: **state3, battery31%, error0, cleaning0, returning0**, common_status49154 → **grab_status0 / grab_mode0**, camera_status11685 → front selected. Dead-reckoned estimate reads **IN_ROBOT**, wrist null, zero joints. `raw/final-post-exit-front.jpg` shows the previously visible gripper/target absent and clear board region. These corroborate stow; they are not encoders or a direct view inside the compartment. `app_program_enter_exit {flag:0}` established idle after stop/end ACKs alone had left state39.

All owned MCP helper clients closed. No ROS/live preview process was started; no parent's process or service was killed. Fresh RoboClaw MCP `get_state` independently verified **empty consumer lists for both z70 and z70_jpeg**, with idle3/battery31/error0 and front camera_status11685. See `safe-service-readback.json`. The camera service remains configured; no shutdown or restart was used.

**8 tests passed** (red/green numerical and real-frame regressions), Python compile passed, `git diff --check` passed, original calibration baseline hashes unchanged. See `verification.json`. One unsupported read-only `get_grab_status` query returned unknown_method; final grab state was instead decoded from documented common_status bits.

## Files and reproduction

- Evidence: `raw/`, `manifest.json`, `log.jsonl`, `session.json`, `survey.json`, `groups.json`, `analysis-policy.json`, `observations.json`, `fit.json`, `holdout.json`, `uncertainty.json`, `validation.json`, `overlays/`, `final-state.json`, `verification.json`, raw offline command outputs and this report.
- Reusable offline implementation/tests: sibling `../physical_base/`. Run its README commands. No production files were changed and no patch is proposed for adoption.
- Remaining genuine blockers: independent straightedge/plumb/camera optical-centre and pedestal/bumper survey, measured print scale/rigid board, elevated-region/multi-tilt lens validation, marker remount and fully frozen independent-session chain validation. The absence of these did **not** prevent the measurable sweeps/fits above; it prevents anatomical/absolute-precision adoption.
