> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [current arm model](../../../docs/z70-arm-model.md), [camera/calibration guide](../../../research/cameras/FINDINGS.md). Optical identifiability limits below apply to this experiment; copied CAD now supplies structural anatomy, not independent physical registration or contact validation.

# Owner-survey constrained-height refit — offline results

## Outcome: NO ADOPTION

**A genuine 60 mm constrained board-pixel refit does not resolve the contradictions.** The centred D primary axis changes from **128.548 mm setback / 65.448 mm above camera / 123.998 mm above board** to **128.910 / 64.000 / 124.000 mm**. D front registration rotates **0.506°**, so this is not a post-hoc height addition. Its all-corner RMS worsens **3.415 → 3.641 px**, and held-out front-frame RMS worsens **3.059 → 3.355 px**. A/B heldout front residuals improve modestly, but redeployment and cross-method agreement still fail.

Owner survey is archived independently in `calibration/physical-measurements/20261014-owner-tape/owner-survey-20261015.json`. The stated exterior front-lens height is 60 mm; the internal optical-origin offset is unknown. The 55/58/62/65 mm and lens variations below are **disclosed sensitivity scenarios, not tape error bars**. Rough shoulder setback 139.7 mm, axis height 127 mm, and body dimensions 355.6 mm were **never fitted or imposed**. Full-scale printing supports nominal 20 mm squares / 15 mm black markers, not a measured printer tolerance.

**No production configuration, URDF, planner, firmware, robot, camera session, Docker, service or commit was touched.** All 408 original archive files, including 195 original images, remained byte-identical; all 195 images match the retained manifest. Previous analyses were not overwritten. This is a local/private archive, not remote backup.

## What was actually optimized

- Frozen original `observations.json` full corners and original `groups.json`; no heldout-driven corner or method selection. Observation extraction is an explicit retained input, not a count-only cache hit. Raw files, log, observations, intrinsics, owner survey, old analyses and implementation hashes enter content/scenario provenance. The new code does not use a mutable result cache.
- Front registration is separate for A/B/D. For board points X, optical projection is `project(K,d,R X - R C)`, where C is optical centre in board coordinates. Free fit has six parameters; constrained fit fixes signed `C_z=±0.060 m` and optimizes the remaining five, including orientation and in-plane registration. Equal-frame residuals are `(projection-observation)/sqrt(corners_in_frame)` with soft-L1 scale 0.5. All original selected full corners, including outliers, are retained.
- Intrinsics stay fixed within every scenario. Nominal front K has f=822.829 px, c=(777.066,598.417), radial k=(-0.0736195,-0.00875963). Pincher intrinsics and objective are unchanged. All eleven pure-joint pincher fits and eleven whole-pose holdouts were recomputed, not substituted by commands or vendor FK.
- The final archived registered front frame in each block is a **whole-frame holdout**: `A-stowed-front.jpg`, `B-reference-Fafter.jpg`, `final-idle-front.jpg`. No corner from that frame enters its training registration. Separate all-frame fits are used only for reconstruction tables, not claimed as holdout predictions. There are only 4/4/9 registered front frames, so these are modest correlated stationary-view checks, not new independent metrology sessions.
- Pincher holdouts retain the previous predetermined final-pose split. Geometry is conditional on measured orientation from a separate settled exposure where available, otherwise from the target itself; no nominal-angle prediction or independent-sensor claim. Their identical error across height scenarios is expected: the front registration cannot alter pincher-only image trajectories.
- Default iterative and first-training-frame alternative IPPE starts are checked by **training objective only**, with positive-depth output. No branch is chosen by holdout. D training costs for the default/alternative starts are **36.518/1981.026 free**, **40.000/1982.815 at60**. Alternative planar minima are retained as diagnostics; they do not rescue the solution. All per-front-frame original IPPE candidates remain in `comparison.json`. Elevated single-marker orientation ambiguity remains separate from this floor-board branch test.

## Free versus 60 mm registration and independent blocks

| Block | Free camera height mm | Full RMS free /60 px | Front holdout RMS free /60 px | Registration rotation change at60 |
|---|---:|---:|---:|---:|
| A |61.008|2.685 /2.744|3.376 /3.035|0.295°|
| B |60.409|2.577 /2.576|3.041 /2.900|0.121°|
| D |58.550|3.415 /3.641|3.059 /3.355|0.506°|

B's unweighted RMS improves slightly while its actual robust objective worsens; the objective, not unweighted RMS alone, defines the optimizer. All constrained robust costs are at least the corresponding free cost.

At D, board-frame camera-centre displacement is **(+0.166,-0.362,-1.450) mm**. This changes setback and above-camera coordinates, but the board-frame pincher axis remains independently reconstructed. Its board height therefore stays approximately124 mm; asserting that an external height adds1.45 mm to the axis board height would be wrong.

| Pure shoulder sweep | Setback free /60 mm | Above camera free /60 mm | Above board free /60 mm |
|---|---:|---:|---:|
| A_M3_45 |161.295 /161.517|65.320 /66.328|126.328 /126.328|
| B_M3_45 |156.565 /156.674|66.064 /66.473|126.473 /126.473|
| B_M3_75 |166.374 /166.484|67.695 /68.103|128.103 /128.103|
| D_M3_45 |130.934 /131.295|68.837 /67.388|127.387 /127.388|
| D_M3_85 |128.548 /128.910|65.448 /64.000|123.998 /124.000|
| D_M3_95 |127.683 /128.044|64.117 /62.669|122.667 /122.669|

All axis stations use the intersection with **front optical x=0**, not an anatomical shoulder centre along the pitch axis. Source geometry remains setback124.399 mm, above-camera53.648 mm, above historical floor122.003 mm, effectiveL1=177.357 mm. No source value was changed.

Independent rough survey check: the D60 primary is **10.790 mm less setback** than139.7 mm and **3.000 mm lower** than127 mm over the board. With undefined tape landmarks/tolerances and optical-origin offset, neither agreement nor rejection at a numerical confidence level is justified. A/B independently suggest larger setbacks than D; fitting the tape would merely conceal that conflict.

## Height profile and lens sensitivity

All rows use the same data, objective and holdout. No scenario is adopted or chosen based on these holdouts. Profile costs are robust weighted objectives, **not calibrated chi-square values or confidence bounds**.

| D scenario | Camera height mm | Full objective | Full RMS px | Front holdout RMS px | Primary setback mm | Primary above camera mm |
|---|---:|---:|---:|---:|---:|---:|
| Free |58.550|40.329|3.415|3.059|128.548|65.448|
| h55 |55|63.140|4.449|4.234|127.674|68.994|
| h58 |58|40.930|3.436|3.072|128.419|65.997|
| h60 |60|44.326|3.641|3.355|128.910|64.000|
| h62 |62|61.210|4.481|4.330|129.477|62.004|
| h65 |65|102.975|6.348|6.383|130.486|59.008|
| h60, front f+1% |60|41.807|3.503|3.186|127.355|64.002|
| h60, front f−1% |60|48.736|3.863|3.647|130.466|63.999|
| h60, front cy+20px |60|45.922|3.721|3.441|130.092|64.001|
| h60, front cy−20px |60|43.333|3.587|3.300|127.670|64.000|
| h60, front k1+0.012 |60|49.987|3.902|3.625|127.820|64.001|

The free D minimum near58.55 mm is numerically identifiable **conditional on this lens/board model**, not evidence that the owner's exterior-height measurement is wrong. Lens scenarios show registration tradeoffs. The earlier retained pincher focal±1% sensitivity of approximately±5.5 mm setback remains unresolved: constraining a different camera's height cannot calibrate pincher intrinsics.

## Uncertainty, rank and unresolved gauges

- Front Jacobian ranks are **6/6 free and5/5 constrained** in each block, with rotations in radians and camera-centre translations scaled by0.1 m. D singular values are approximately `[2591,2487,697,470,168,157]` free and `[2579,2168,701,441,158]` constrained. Pixel-axis D primary remains **14/14** after removing its along-axis station gauge. Full spectra and conditional local covariance are in JSON. Local rank cannot identify anatomy omitted from the model.
- Whole-front-frame bootstrap: **100/100 successful for each block and each of free/60**. D free conditional standard deviations: camera height0.147 mm, setback0.057 mm, above-camera0.147 mm. At60, setback0.038 mm and above-camera0.0014 mm with the pincher axis frozen. The near-zero constrained height variance is **imposed by construction**, not extraordinary physical accuracy.
- Separate whole-pincher-pose **model-free** bootstrap: **192 successful /200 attempted**,8 insufficient-excitation failures. At60, conditional σ is **2.257 mm setback /1.304 mm above-camera**; marginal bootstrap percentile95 ranges **121.964–131.809 mm /59.897–65.010 mm**. These are not pixel-bundle or joint front/pincher refits. Front and pincher conditional uncertainties are reported separately, not silently combined into certified bounds.
- Correlated corner errors, distortion extrapolation, print scale/flatness, exterior-lens versus optical-origin offset and remount/redeployment systematics are not bounded by these bootstraps. No absolute95% interval or5mm/1° certification is available.
- Along-axis shoulder centre, physicalL2/M4 zero/M5 station, exact pads, pedestal and bumper landmarks remain unknown. A marker-only front trajectory is not a pincher-optical trajectory or an independently registered wrist reference.

## Contradictions that survive height conditioning

1. **Redeployment goal still FAILS.** A/B→D line separations change **33.221/28.403/38.118 →32.295/27.588/37.400 mm**; direction differences at60 remain **2.453/2.094/2.357°**. Do not average those blocks into a falsely precise shoulder estimate.
2. **Cross-method goal still FAILS.** Front-optical closest-point circle-versus-pixel line discrepancies are **11.924/15.338/27.177 mm free**, **11.919/15.341/27.187 mm at60** for D45/85/95. Directions differ1.586/0.948/3.912°. Same-block rigid registration cannot repair the underlying trajectory-method disagreement. These finite-station discrepancy metrics are slightly origin-dependent for nonparallel estimates, so both scenarios use exactly the archived front-optical station convention.
3. **EffectiveL1 is not independently supported.** At60, shoulder-to-elbow approximately-parallel-axis distances remain **183.302/182.447 mm**, compared with source177.357 mm; free values are183.303/182.448 mm. The inferred axes are not exactly parallel (0.335/0.321°). Infinite skew-line shortest distances,81.296/97.613 mm, are also published to expose why an unconstrained closest-line distance is not an anatomical link measurement. Those remote mathematical line minima are not alternate physicalL1 estimates. No link-length update is justified.
4. **The D F/G/F pair remains REJECTED.** Archived marker orientation change3.12° exceeds the frozen3° rule. It was not reclassified, used to refine a camera chain, or called an independent anatomy validation. No new synchronized/stationary pairing evidence exists.

### Same whole-pose pincher holdouts under free and constrained registration

| Block | Training RMS px | Holdout RMS px | p95 px | max px | Camera-centre disagreement mm |
|---|---:|---:|---:|---:|---:|
| D_M3_45 |1.797|2.876|4.920|6.023|0.955|
| D_M3_85 |2.247|4.650|5.767|27.183|0.525|
| D_M3_95 |2.959|7.471|10.123|11.341|2.349|
| D_M4_190 |1.908|8.143|10.069|11.626|3.663|
| D_M4_210 |2.363|4.299|6.616|9.404|0.764|
| D_M5_20065 |2.798|3.822|5.871|12.196|0.479|
| A_M3_45 |7.700|14.853|19.746|27.567|4.793|
| B_M3_45 |7.741|15.860|19.960|27.703|4.762|
| B_M3_75 |7.338|9.492|20.526|31.210|4.200|
| A_M4_200 |6.073|29.811|56.046|77.485|20.129|
| B_M5_19055 |4.126|3.473|5.828|11.775|2.718|

The source holdout identity is the same in every scenario. These are conditional geometry checks, not nominal-command accuracy. D85's27px extreme and D95/M4's coherent several-pixel errors warrant different diagnostics; retaining all points avoids hiding either pattern.

## Residual-grounded smallest discriminating next acquisition

`residual-diagnostics.json` makes image-region limitations explicit. In the retained all-pose pincher fits, A/B45 pixels within radius350px of principal point have approximately**2.96/3.25px RMS**, versus**11.47/11.55px** beyond600px. D85 has**2.20px within350**, **2.08px at350–600**, **6.56px beyond600**. This is evidence of image-region-dependent model/observation error, not proof of distortion as the sole cause. Whole-pose holdouts also show coherent pose errors, so a single bad corner is not the entire explanation.

The elevated front marker occupies approximately**y189–348**, whereas the D floor-board corners used here lie approximately**y801–1003**. Floor-height conditioning does not validate upper-image front intrinsics or the tiny elevated marker's planar pose. Existing front elevated ID68 frames mainly cover M5/references, not a new independently surveyed shoulder-axis trajectory.

The smallest useful follow-up is **a small rigid, physically scale-checked target imaged at several tilts/depths through the elevated front region y190–350 and through the pincher centre/edge regions**, with one or more whole target poses reserved before fitting. Keep chassis/target placement fixed where comparing methods. This directly tests the observed coverage/residual contradiction rather than collecting another equivalent floor-only sweep. Then one centred, larger-angular-span pureM3 sweep and pureM4 sweep with rigid attachment and observed execution can test whether the circle/screw disagreement shrinks; do not command longer spans without clearance/execution checks. A new truly stationary F/G/F bracket below the existing3mm/3° thresholds is needed before camera-chain refinement. None of this live acquisition was attempted in this task.

## What can safely update, and what cannot

Safe now: retain owner survey as exterior/rough independent evidence, nominal board scale confirmation, offline constrained-registration scenarios and the **no-adoption** result. A visual-body-only convention could use the rough355.6mm envelope with an explicitly provisional visual offset.

Not safe: moving the production shoulder, changing effectiveL1, claiming a surveyed rotation origin, nose radius or exact bumper setback. Body355.6mm does **not** fix historical camera x≈0.275m: geometric body centre versus chassis rotation origin is unresolved. Camera-relative axis placement alone cannot separately determine nose radius and base translation. “Bumper over camera / very similar setback” is not a separate exact nose measurement.

## Files and verification

- `comparison.json`, eleven `scenario-*.json`: actual numerical registrations, axes, residuals, profile/scenario provenance, rank/covariance and separate bootstraps.
- `pincher-fits.json`: recomputed pure-joint fits and identical whole-pose holdouts.
- `archive-before-sha256.json`, `archive-after-sha256.json`: matching408-file snapshots;195 raw original manifest entries verified.
- `residual-diagnostics.json`, `diagnostics.py`: actual image-region residual/marker-coverage diagnostics.
- `offline-refit-run.txt`, `tests.txt`, `all-tests.txt`, `verification.json`, `verify.py`, `README.md` and this report.
- New reusable code/tests: `../physical_base/survey_refit.py`, `../physical_base/test_survey_refit.py`. No existing analyzer or production module was modified.

**12 tests passed, zero failures/errors**, including4 new tests, numerical geometry, real observations and both previous integrations. Legacy integrations that normally overwrite source outputs ran **only on a disposable full archive copy**, redirected by `verify.py`; never run the old unrestricted unittest command against the original session. New tests use temporary output directories and assert source hashes. Python compilation and `git diff --check` passed. See README for exact commands.
