# Pincher intrinsics + identifiable arm geometry — RESULTS

## Outcomes

- **PASS, conditional optical calibration:** an independently fitted pinhole + k1 pincher model improves final whole-pose holdout RMS **2.253 → 1.668 px**, and **150 historical views not used in the new lens fit** improve **4.310 → 2.293 px** on identical retained corners. This is an **offline proposed calibration**, not deployed production configuration.
- **Upper-link result: approximately 180 mm, conditional; absolute adoption INCONCLUSIVE.** New long shoulder sweep plus independent elbow sweep gives **179.925 mm** at the explicitly defined common transverse station. Other new elbow sweeps give **178.600 and180.424 mm**. Longer archived sweeps reconstructed with the new lens give **178.751–178.917 mm** for the principal centred shoulder block. No production length changed.
- **FAIL, uniform marker-registration/1° generalization:** paired front ID68 observations were acquired and a fixed optical-camera→marker transform estimated. One held-out pitch pose has **7.156 px** error and **3.68°** marker-orientation disagreement. Two held-out roll poses have **2.387/2.506 px**, **0.90/0.98°**. The improved pincher lens does not make the frozen front lens or full mechanical chain validated.
- **INCONCLUSIVE / unidentifiable:** physical L2, physical M4 zero, M5 axial wrist station, shoulder lateral centre, pedestal/bumper anatomy and exact jaw contact geometry remain null. Effective camera/marker lever vectors are reported separately, not renamed as physical segment lengths.
- **Cleanup PASS at18%:** arm absence in the final front image, grab-status0/mode0, stow estimator and idle3 corroborated stow; pickup/program modes exited, front selected, both camera consumer lists empty. A later offline readback reports **17%, state8, charge_status1**, no cleaning/returning/error, grab-status0/mode0, front selected and consumers empty. The later state is **not idle3**. No dock or navigation command was issued; that transition's route/cause was not observed.

## Evidence, scope and actual coverage

Retained **92 original JPEGs**: **33 front /59 gripper**. Pincher fitting uses **55 board views**, with **4 unusable/no-board setup views retained**. There are **32 pose groups including roll**, **18 M3/M4 combinations** after documented observed-equivalence grouping, **43 bounded arm jogs**, **17 front frames with elevated ID68**, and **10 complete F/G/G/F brackets:8 eligible,2 rejected for missing elevated marker**. Early unbracketed visibility attempts are also retained. This meets20+ distinct optical-view groups, **not** the plan's20 M3/M4-combination goal.

The chassis was not commanded during acquisition; no RC/chassis pulse, grip, pick, clean, dock, navigation, autonomous arm program, ROS poller or service rebuild was started. All robot actions/observations/camera selection used RoboClaw MCP. The existing MCP SDK helper copied each private frame before another frame request. No credentials were inspected or printed. No commits.

Nominal labels span M3 **150–250**, M4 **45–85**, M5 **0–40°**. These are **inputs, not encoders**: two reach-limited shoulder jogs near nominal230/240-85 produced almost no observed rotation. Label drift is retained, not silently repaired into measured angles. The new long pure-M3 sweep spans **39.33° observed**; new pure-M4 sweeps span only **19.20–20.19°**; new M5 sweeps span **37.46–38.25°**. The independent historical centred M4 sweep spans **39.26°** under the new lens. Intrinsics and axis fits impose neither exact jog angles nor nominal arm FK.

### Target/provenance

Legacy dense10×14 ChArUco, DICT_4X4_1000,20mm checker squares and **15mm black markers**. The elevated pincher marker decodes **68**; expected69 is historical provenance only. Its black side is **15mm, not20mm**. It is mounted afterM5, not at a registered wrist joint. Repeated floor-board ID68 is excluded from elevated association using the floor mask, board plane and height. No historical ID0 mounting transform was borrowed.

Vision inspection found the printed floor/marker regions approximately planar and no obvious black-square folding; white-margin curl/overhang was visible. Rigidity, planarity and printer tolerance are **not independently quantified**. Owner full-scale print supports a scale assumption, not a tolerance certificate.

`log.jsonl` contains RPC receipts, actual camera tags/sizes and estimator predictions. Helper `t` is completion time, **not exposure time or exact frame receipt**. Exact exposure/sequence metadata are unavailable; direct preflight receipt is in `preflight.json`. F and G are shared selectable-stream exposures, **not simultaneous cameras**. `raw/002-deployed-gripper.jpg` is actually FRONT in its MCP log and dimensions, despite the requested filename; it is excluded from pincher fitting.

## Pincher optical calibration

Model selection used **training-only grouped5-fold validation**, preferring the simplest within10% of the best CV result. Final held-out poses were not used for selection. **46 training frames/25 groups**, **9 held-out frames/7 groups**; repeated/refused-equivalent poses stay together.

| Model | Training grouped-CV mean RMS |
|---|---:|
| Pinhole, no distortion |4.960 px|
| Pinhole+k1 — selected |1.825 px|
| Pinhole+k1+k2 |1.826 px|

Proposed1600×896 plumb_bob parameters:

```
fx =830.844116     fy =830.223562
cx =800.438489     cy =305.948476
D =[-0.089972985,0,0,0,0]
```

Historical frozen parameters were fx=fy843.031112, centre(798.210915,310.057140), k1−0.097765272/k2+0.024093059. Final held-out new model: **RMS1.668, p95 3.036, max9.516 px**, positive depth in every held-out view. The maximum is retained, not hidden by trimming. Initializer trimming in arm PnP does not remove full corners from reported pixel objectives/residuals.

**200/200 successful whole-pose lens refits**, no failures. Conditional1σ: fx1.261px, fy1.240px, cx0.636px, cy2.365px, k1 0.000773. These exclude print/board/detector/model systematic error. The normalized intrinsic Jacobian after per-pose extrinsic-nuisance elimination has **rank5/5**, condition number **11.63**; k1k2 identifiability spectrum is retained separately.

Actual plane-normal span **56.43°**; board-origin optical depth **0.216–0.394m**. **53/60**10×6 image cells sampled; corner bounds x207.14–1595.44, y0.65–894.69. Centre/top/bottom and right edge are substantially better covered, but **far-left x0–207 and several lateral/corner regions are not calibrated by observations**. Coverage overlay and actual corner overlays were visually checked; circles align with checker intersections, not marker-internal corners.

On identical new corners, outer-radius≥600px RMS improves **7.606 →2.807px**. Conditional held-out checker pair-distance relative RMS averaged across poses improves **0.951% →0.757%**; worst relative pair error is still6.56%. Those are out-of-sample image/corner checks on the same print, **not an independent ruler or printer-scale validation**. Full-view/region residuals and old/new historical comparisons are retained.

## Primary arm result, validation and uncertainty

`T_A_B` maps B coordinates into A. W is the OpenCV printed-board frame; directions and lines retain their actual handedness/sign conventions. L1 is measured between pitch-axis intersections with a **common plane normal to their mean direction through W(0.1,0.14,0)m**. This avoids a remote infinite-skew-line minimum and does not invent physical along-axis joint centres.

| New principal shoulder sweep vs elbow sweep | Pixel L1 | SE3 L1 | Position-circle L1 |
|---|---:|---:|---:|
| M3-55 vs M4-200 forward |179.925mm|179.957mm|189.452mm|
| M3-55 vs M4-220 reverse |178.600mm|179.864mm|180.853mm|
| M3-55 vs M4-230 reverse |180.424mm|181.491mm|195.217mm|

The pixel axes are nearly parallel, **0.16–0.38°** apart. The principal conditional shoulder line gives **126.48mm setback from front optical origin**, **126.43mm height over the board**. Its long-sweep position-circle shoulder line disagrees by **16.23mm at the stated station**. Very short arcs are worse: a three-point circle can interpolate positions perfectly yet put its axis far away. These failures are retained rather than treated as metrology.

Longer independently retained centred D sweeps improve materially with the new lens: principal L1 **178.751–178.917mm**, versus frozen-lens **182.403–183.253mm**; position-circle L1 **176.806–177.389mm**. D principal shoulder pixel/circle line disagreement improves to **2.51mm**, from15.52mm. Historical A/B camera-relative shoulder setbacks move from roughly161–166mm with the frozen lens to122–124mm, near new126.48mm, but this retrospective correction is not an independent new deployment experiment.

The principal new whole-pose axis holdout uses a withheld observed **orientation only**, not its translation or exact nominal angle: centre error **1.002mm**, pixel RMS **3.697px**. Longer historical D held-out camera-centre errors are **0.111–0.428mm**, with pixel RMS1.62–2.20px. These are **conditional geometry** tests, not command-only accuracy or independent front-marker shoulder validation.

The principal normalized pixel-axis Jacobian has **rank14/14**. Three displaced multistarts converge to the same conditional solution. The SE3 translation profile has its grid minimum at **180mm**; this profile freezes measured axis directions/rotations and is **not a global joint-pixel profile**.

**200 successful whole-pose PnP/screw bootstrap fits /1061 attempts /861 rejected insufficient-excitation draws:** conditional L1σ **0.423mm**, percentile95% **179.468–181.089mm**. This is **not** joint pixel-bundle covariance. The three-pose elbow bootstrap has only one fully excited unique support; it cannot estimate full elbow uncertainty. It excludes lens/print/flatness/hysteresis/deployment systematics.

Twelve selected lens-bootstrap focal quantiles give L1 **178.933–180.347mm**; this is a propagated range, not a fittedσ. Assumed—not surveyed—sensitivity scenarios: +1% focal gives183.122mm; +10px cy gives181.011mm; k1k2 alternative179.847mm; ±1% print scale178.126–181.724mm. New M5 forward/reverse differences at10/20/30 nominal are **1.18/1.81/2.19°**. Pixel estimates across short shoulder sweeps range175.75–184.02mm; cross-method scatter cannot be replaced by the0.423mm bootstrapσ.

**Conclusion:** around179–180mm is the supported conditional upper-pitch-axis separation. It differs from the existing effective177.3568mm by **+2.568mm**, insufficient reason to replace production geometry while independent/systematic-inclusive5mm/1° certification is absent. **Absolute L1 adoption is INCONCLUSIVE**, not a new precise anatomical constant.

## Front ID68, mount and second assembly

Pilot stationary ID68 orientation variation was **4.26°**, despite0.645mm translation. The frozen pilot-derived3×noise orientation eligibility threshold is therefore **12.78°**; translation remains3mm. Floor-board and G duplicate stationarity also require≤3mm/3°. This permissive marker-orientation threshold honestly reflects poor planar orientation precision and **does not demonstrate1° mechanical accuracy**.

Eight stationary brackets survive; two complete brackets lose elevated ID68. A fixed optical `T_G_M` is fit over training paired poses, with whole held-out marker poses and both observed orientations/branches retained. Conditional mount Jacobian rank6/6, repeated multistarts agree. New-model training marker RMS2.260px. Held-out marker poses:

| Pose label, nominal only | Pixel RMS | Marker translation disagreement | Orientation disagreement |
|---|---:|---:|---:|
|250/45/0|7.156px|0.96mm|3.68°|
|250/55/40|2.387px|1.32mm|0.90°|
|250/55/20|2.506px|1.95mm|0.98°|

Aggregate held-out marker RMS is **4.589px new vs3.787px frozen lens**: **no blanket cross-view improvement claim**. The frozen front camera's elevated-region model, marker-planarity/noise, and rigid-mount/generalization remain unresolved. Front floor registration RMS is3.243px, recovered camera height58.23mm; the owner's60mm reading is rough, **not an exact prior**. The rough127mm shoulder height and139.7mm setback remain independent checks with undefined tolerances, not exact constraints.

An independent three-pose front-ID68 M4 trajectory puts the elbow line **1.86mm** from the pincher reconstruction at the defined front station, but axis direction differs **7.63°**; position-circle crosscheck differs3.75mm/1.97°. Front ID68 observes37.20° roll versus G≈38.25°. These corroborate a moving rigid pincher target but **do not establish a front-camera shoulder axis or independent L1 holdout**: ID68 was outside the front FoV across the useful shoulder sweep.

At the recorded elbow/camera axial convention, effective elbow-line→optical-centre perpendicular radius is **156.754mm**; elbow-line→ID68-centre radius is **128.766mm**. Full W-frame vectors, roll-axis lines and `T_G_M` are retained. Neither radius is physical L2 or a pad endpoint. Exact L2/free-mount and M4-zero/free-mount gauge profiles, Jacobian null direction and equivalent factorizations are in `validation/effective-second-assembly-and-gauges.json`. M5 locates a line, not its physical axial station. Anatomical quantities remain null.

## Adoption, limitations, remaining blocker

**Adopted into production: nothing. Proposed:** pincher k1 intrinsics and conditional axis/effective-mount artifacts for parent review. No URDF/planner/firmware/shared mount/front calibration edit; no commit. This run does not claim the full global latent-joint, two-camera pixel-bundle or independent third-session/remount certification.

To certify physical segment geometry, the remaining load-bearing constraints are: independently quantified print/planarity/scale; a rigid multi-tilt front target covering the upper marker region; longer independent elbow arcs and shoulder poses with visible ID68; mechanical wrist/axis-station/contact references for physical L2/zero anatomy. More equivalent coplanar front floor frames or nominal-angle labels cannot remove those gauges. Coverage goals were curtailed before the battery cleanup floor, not silently declared complete.

## Files and verification

- `raw/`, `manifest.json`, `log.jsonl`, `preflight.json`, `session.json`, `counts.json`, `observation-provenance.json`, `all-pincher-observations.json`, `intrinsics-observations.json`.
- `lens/`: model selection, splits, final holdout,200-refit uncertainty, coverage, proposed JSON/ROS YAML and overlays.
- `arm/`: full raw observations/PnP branches, axis fits/holdouts, link geometry, pairing, marker registration, uncertainty, conditional effective second-assembly vectors.
- `validation/`: historical lens holdouts, region residuals, lens uncertainty propagation, conditional L1 profile, front-ID68 elbow/roll, M5 hysteresis, gauge-equivalence profiles, held-out marker overlays and hash verification.
- `final-state.json`: cleanup at18%/idle3. `post-analysis-state.json`: later17%/state8 charge-status1, front/consumer readback.
- **9 tests passed**, plus all three real analysis CLIs executed. `prepare.py` independently regenerated an identical observation SHA-256. **92/92 originals and410/410 prior baseline/archive files hash-verified unchanged**. No legacy archive-writing tests were run on originals. Evidence remains private/local, not remotely backed up.

Reproduction commands and code/test descriptions: [`../pincher_intrinsics/README.md`](../pincher_intrinsics/README.md).
