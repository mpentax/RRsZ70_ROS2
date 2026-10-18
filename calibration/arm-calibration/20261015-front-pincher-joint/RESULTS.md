# Front optics + pincher→ID68 joint fit — offline results

## Outcome

**Working two-camera pixel-bundle artifact produced; production adoption rejected.** The retained data support a locally identifiable conditional optical fit, but freeing front optics does not fix the held-out pitch failure or reconcile the front/G elbow directions. This is not merely a repeat of the previous no-adoption note: front K/D, floor registration, rigid camera→ID68 registration and latent G poses were jointly optimized, selected with training-only grouped CV, profiled, bootstrapped, and carried through to the identifiable arm geometry.

No live hardware/camera/service/Docker/network access, production configuration edit or commit. All changes are new code/tests/documentation in `../front_pincher_joint/` and outputs here. Earlier archives are unchanged.

## Protocol and evidential limits

- Input archive: 92 originals, 33 F / 59 G; 55 usable G board views. Existing ten F/G/G/F brackets remain eight eligible and two rejected for absent elevated ID68. No rejection or original stationarity threshold was relaxed.
- Fit uses five training brackets / four pose groups: 250/65/0, 250/55/0 (both approaches grouped), 250/55/30, 250/55/10. Three whole final pitch/roll brackets remain withheld: 250/45/0, 250/55/40, 250/55/20. Labels are nominal identifiers, never exact angle constraints.
- Frozen validated G lens: fx830.844116, fy830.223562, c(800.438489,305.948476), k1−0.089972985, delivered1600×896. ID68 black side15mm, not20mm; floor repeats are not treated as elevated marker observations.
- Model fits actual F floor pixels, F marker pixels and G floor pixels. It does not duplicate PnP transformations as observations. One fixed `T_F_W` applies to this stationary block, one `T_G_M` applies after M5, and G poses are latent nuisance variables constrained by their G pixels.
- Full final F images/poses are withheld. G board images supply pose inputs without using their F counterpart. Marker orientation/translation disagreements against square PnP are diagnostics, **not independent ground truth**; both IPPE branches are retained.
- **Prior analysis already inspected these final poses. These are exploratory withheld pixels under a newly declared protocol, not independent prospective validation.** Whole-image holdouts cannot erase that history. Historical floor images were excluded from current model selection but were previously seen in older analyses.
- Brackets are sequential shared-stream exposures with unknown exact exposure timestamps. The original marker orientation eligibility threshold12.78° reflects pilot planar-pose noise4.26°, not1° mechanical accuracy. Printer scale, floor/marker nonplanarity, detector bias and optical-origin offsets remain unquantified systematics.

## Training-only selection

Four leave-whole-training-pose-group-out folds; equal image weight after √N-corner normalization; soft-L1 scale0.5. Duplicates remain correlated, not independent poses. Selection uses combined marker/floor CV and prefers the simplest within10% of best, subject to floor RMS≤110% of frozen-lens baseline. All selected fits converged without bounds hit.

| Front optical model | CV marker RMS px | CV floor RMS px |
|---|---:|---:|
| Frozen K/D, joint TF/GM/G poses |4.298|3.205|
| shared f |4.348|3.205|
| f + cy |3.370|3.217|
| **f + cy + k1 — selected** |**2.997**|**3.212**|
| fx/fy/cx/cy/k1/k2 |2.628|3.132|

Selected candidate,1600×1200 front stream:

```
fx = fy = 824.966702
cx = 777.066386    cy = 583.778554
D = [-0.070084527, -0.008759625, 0, 0, 0]
```

cx and k2 are frozen at prior values in this nested model. The full six-parameter candidate instead gives fx831.759607, fy832.219423, c(783.311404,585.140434), k1−0.104800539/k2+0.027805871. It is a diagnostic alternative, not a model chosen after inspecting final holdouts.

Training point RMS, identical actual pixels:

| Target | Frozen front optics joint fit | Selected joint fit |
|---|---:|---:|
| F marker,40 corners |0.725|0.658|
| F floor,632 corners |3.239|3.244|
| G board,776 corners |2.277|2.249|

The low training marker residual is not generalization. G poses move up to0.517mm/0.176° from their G-only estimates during joint fitting, enough to matter for a close marker. `conditional-versus-joint.json` explicitly freezes those measured G-only poses: training marker RMS2.001→1.900px, but final marker RMS **4.952→5.258px**, worsening when optics are freed. The candidate is therefore not robust to how G-pose uncertainty is handled.

## Final withheld F pixels — failure retained

All six final F images are excluded from fitting and selection; K/TF/GM frozen, G pose inferred from G pixels only. Results use both F exposures per bracket, not just F-before.

| Nominal marker pose | Frozen-front joint marker RMS | Selected marker RMS | Selected max PnP orientation disagreement | Selected max PnP translation disagreement |
|---|---:|---:|---:|---:|
|250/45/0, pitch extrapolation|7.729px|**9.130px**|**2.933°**|1.644mm|
|250/55/40, roll|5.252px|1.413px|0.710°|0.709mm|
|250/55/20, roll|3.892px|2.304px|0.662°|0.735mm|

Aggregate marker RMS **5.844→5.498px**; p95 worsens **8.982→10.184px**, max **9.436→10.871px**. Final F floor RMS is essentially unchanged **3.383095→3.383508px** over372 corners. G-only input reconstruction RMS2.531px over332 corners is identical between front scenarios.

This is modest aggregate pixel improvement driven by roll, **not uniform pitch/roll validation or1° certification**. The prior report's4.589px registration result used a different objective, fixed G poses, floor-only TF and one F-before/G2 selection. It is not an apples-to-apples baseline for this two-exposure joint fit; both protocols remain preserved.

Historical floor checks keep K fixed and refit a separate floor TF per documented A/B/D block, training on earlier frames and withholding the last two complete frames:

| Previous stationary block | Old K held-out floor RMS | Candidate K held-out floor RMS |
|---|---:|---:|
|A|3.085px|3.089px|
|B|2.966px|2.972px|
|D|3.227px|3.255px|

Thus floor geometry is largely preserved, not improved; these floor-only checks cannot certify the upper image lens.

## Candidate transforms, identifiability and uncertainty

`T_A_B` maps B into A, metres. W is the printed board frame, F/G optical frames, M the ID68 square frame. Exact transforms are in `candidate-proposal.json`; rounded selected values:

```
T_F_W =
[ 0.99806577  0.06175104 -0.00717842 -0.12393089 ]
[ 0.00963621 -0.03959624  0.99916930  0.07358708 ]
[ 0.06141551 -0.99730584 -0.04011469  0.39688619 ]
[ 0           0           0           1          ]

T_G_M =
[-0.99995311  0.00943699 -0.00217461  0.00050051 ]
[-0.00215197  0.00240805  0.99999479  0.00546042 ]
[ 0.00944217  0.99995257 -0.00238763 -0.02922052 ]
[ 0           0           0           1          ]
```

These are board registration and optical-camera→marker registration, **not a wrist/URDF mount**. T_F_W cannot be copied to a different stationary chassis/board block. No prior-session marker remount is assumed identical.

After eliminating latent G-pose nuisance columns, selected global Jacobian rank **15/15**, condition **627.49**, singular values **4594.04…7.321**. Full model rank **18/18**, condition **1329.18**, singular values **4584.56…3.449**. Scaling: angles radians, translations/0.1m, focal/principal deltas/100px, radial deltas/0.1; image residuals/√N. These are practical weak directions, **not an exact front optical null**.

- Selected weakest normalized vector is dominated by cy(+0.872), TF_ty/0.1m(−0.410), k1/0.1(−0.223), TF_rx(+0.099). Principal row trades against floor pose and radial response.
- Full-model weakest vector is dominated by k2/0.1(+0.714), k1/0.1(−0.667), fx/100(+0.133), fy/100(+0.132), cy/100(+0.089). Radial/focal tradeoff remains weak over observed support.
- Reoptimized full-model profiles: optimum robust cost66.414; fx±50px gives86.862/83.271; cy±40px gives70.675/70.088, cy±80px81.976/76.742. fx+100 hits a k2 bound and is explicitly marked. These costs are not calibrated confidence limits under correlated errors.
- Two displaced registration multistarts converge to selected cost68.325427 and the same conditional fit. Optimizer convergence does not cure predictive failure.
- **50 successful whole-bracket/G-lens-propagated bootstrap fits /59 attempts /9 insufficient-excitation rejects**. Conditional2.5–97.5% ranges: focal823.686–826.442px, cy578.254–590.175px, k1−0.071655–−0.068835. GM translation marginal ranges: x0.258–0.805mm, y4.576–5.960mm, z−29.636…−28.751mm. Final marker RMS across refits4.416–7.330px. These are not systematic-inclusive physical bounds.

## Arm geometry carried through

Actual G floor corners were refit in pure-joint pixel-space axes under the validated G lens, without nominal FK/jog angles. Front optics change camera-relative registration, not W-frame axis separation.

| Principal long M3 shoulder vs elbow sweep | Common-station L1 |
|---|---:|
|M4-200 forward|**179.924890mm**|
|M4-220 reverse|178.599503mm|
|M4-230 reverse|180.424028mm|

Station: both axis lines intersect the plane normal to their mean direction through W(0.1,0.14,0)m; not a remote skew-line minimum or surveyed anatomical centre. These reproduce the prior validated-G reconstruction, rather than inventing a front-derived L1 improvement.

Candidate front registration gives shoulder setback **124.933mm from front optical origin**, board height **126.432mm**, and camera height **58.495mm**. Rough owner60mm lens height /127mm pedestal height /139.7mm setback are external checks with unspecified landmark/optical-offset tolerances, not exact fit priors.

The front-only three-ID68-pose elbow reconstruction still differs from the G axis direction by **7.866°**; it includes training and previously seen withheld poses, so is an exploratory cross-method reconstruction, not independent certification. The useful shoulder sweep still lacks visible ID68; no front shoulder or independent front L1 has been manufactured.

At each optical/marker centre's perpendicular projection onto the recovered elbow line:

- Elbow→G effective vector W **(8.239,−96.950,122.999)mm**, radius **156.832mm**.
- Elbow→ID68 effective vector W **(6.452,−74.687,103.389)mm**, radius **127.706mm**.

Neither is anatomical L2. Physical L2 + free along-link mount has Jacobian `[1,1]` with null direction `(1,−1)/√2`; rotating a free mount absorbs M4-zero changes; translating along the recovered roll axis leaves its physical M5 axial station unknown. Physical L2, M4 zero, M5 station and exact pad vector remain JSON null. Resolving them requires an independent mechanical wrist/axis/contact reference, not more observations of the same unregistered marker.

## Smallest useful additional acquisition, derived from retained support

**No new acquisition occurred here.** The selected training marker occupies only x505.41–676.54/y42.54–188.65px, depths103.10–121.47mm. Floor corners occupy x192.76–1309.03/y802.94–994.09px. Marker normals span31.24° mostly through roll; only two distinct pitch support levels train the model. Much of the middle image and lateral upper field is unobserved; pitch extrapolation is the actual failure, not lack of an optimizer rank number.

1. **Smallest informative already-achieved arm-pose follow-up:** acquire a fresh stationary F/G/G/F bracket near the actual G pose retained as `raw/065-g250-45-40-nominal.jpg` (nominal label is not a motion command). It combines the failing pitch configuration with roll40 rather than repeating55-pitch roll. Model-predicted marker centre≈(481.8,113.3)px, depth≈87.1mm;34 G floor corners are geometrically in view. The local full-model weakest information eigenvalue increases11.897→13.566, condition1329→1256 under idealized noise. This is a targeted discriminator, **not enough for global lens certification**. Actual visibility/stationarity must be observed; failed/missing-marker brackets remain failed.
2. Screening all retained achieved G poses, after excluding equivalent already paired poses and explicit missing-ID68 rejects, yields only that one non-equivalent fully visible candidate. Most useful shoulder poses project above the F image. Repeating the old long shoulder sweep cannot create an independent front shoulder axis with the present marker placement/framing.
3. An unconstrained optical-space design on an explicit81-view grid selects marker centre(1200,100),depth120mm with unchanged reference normal, then(400,100),depth120mm with an extra−20° optical-x tilt. Idealized weakest eigenvalue11.897→24.597→29.897, condition1329→927→846. **These are design predictions, not achieved reachable arm poses.** Chassis motion alone moves the floor relative to the robot but does not independently move the robot-fixed arm across its front image. Do not promise that a wrist jog reaches x1200. Lateral optical coverage needs a separate rigid known-scale target or independently registered mounting/view geometry; changed marker mount or stationary block adds new nuisance transforms and invalidates these fixed-block gain numbers.
4. A practical next validation set must include an **unseen pitch/depth case** after model freeze, plus the above combined pitch/roll discriminator. If a separate rigid front target is used to cover the missing lateral/middle region, aim explicitly at those missing pixels and include independent plane-normal/depth variation rather than more equivalent floor frames. No finite number here guarantees1° accuracy under unknown print/planarity/timing systematics.

Thus a fullK optical rank deficiency is **not** demonstrated: it is locally full-rank. What remains is weak radial/principal/registration conditioning, severe spatial/pose support limitations, and an observed pitch prediction failure. Anatomical L2/zero/station gauges are exact and cannot be fixed by any number of identical marker views.

## Adoption proposal and verification

- Keep the validated G optical model as an offline optical proposal. Do not change camera_info K alone in the arm stack while retaining mounts fitted under older K.
- Keep selected F K/D, T_F_W, T_G_M, station-defined L1 and effective vectors together as one explicitly **non-adopted candidate package**. `candidate-proposal.json` says DO_NOT_ADOPT_PRODUCTION and `production_patch` is null.
- No replacement production shoulder/L1/L2, URDF, planner TCP, camera-at-wrist or nominal-joint-zero patch is justified by this dataset. The board-frame effective geometry does not identify a physically surveyed wrist or establish correspondence to drifted vendor joint counters.
- **Eight tests pass**, including a fresh real-data full CLI run, transform math, no-F-pixel leakage into G pose, frozen-G conditional fitting, output/symlink isolation and design-information PSD checks. Exact commands and method are in `../front_pincher_joint/README.md`.
- **1,267 preserved source files unchanged** by SHA-256 before/after; **410/410** earlier baseline hashes and **92/92** original-image manifest hashes verified. Code/software/input hashes, convergence counts, full residual metrics, per-image PnP branches, CV folds, bootstrap samples, profiles and design-only labels are retained as JSON. No legacy archive-writing tests ran against originals.
