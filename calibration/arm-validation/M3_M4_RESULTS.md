> **Dated evidence: 2026-10-13; curation 2026-10-18.** The measurements and original report below are unchanged; this is local jog evidence, not current geometry certification. Use [current arm model](../../docs/z70-arm-model.md) for CAD anatomy and current estimate semantics. The retained reproduction/venv command below is historical, not a current executable recipe.

# M4 then M3: supervised estimator check

Outcome: direction and coarse nominal 10-degree increments supported in the tested poses. No vendor constants changed; full arm calibration remains unvalidated.

## Scope and evidence

- Twelve measurement jogs: eight M4 commands (two baseline → −10 → baseline → +10 → baseline cycles), then four M3 commands (two baseline → +10 → baseline cycles). Every measurement endpoint has an accepted two-frame pair.
- Robot access was MCP only. One operator-authorized programming-mode entry undocked the robot; Michael confirmed separation and later manually moved it into the testing area. No chassis pulse, docking or cleaning command was sent.
- Initial conversational pauses caused operator-reported inactivity stowing. Explicit reset/stow/redeploy preceded the continuous acquisition. An apparent animal identification was corrected by Michael; the false positive and restart are retained in the log.
- The flat, secure arm-mounted marker is DICT_4X4_1000 ID 0: 20 mm paper square, 15 mm printed black side. Mounting unchanged per Michael. Wrist/fingertip offsets remain independently unmeasured, not zero.
- Existing front-camera intrinsics were reused outside their floor-board coverage. Lens identity and exposure times remain unverified; logged timestamps are receipt times.
- Evidence: `m3-m4-session/log.jsonl`, per-pair `*-analysis.json`, `increments.csv`, `summary.json`. Reproduce with `.venv/bin/python m3-m4-session/report.py` from this directory.

## Setup and stationary noise

- Four forward/four down setup commands left the marker cropped. One additional model-checked down step established the M4 reference: predicted M3=245.097896°, M4=96.977279°, M5=0°, M6=90°; wrist (0.316765, 0.040662) m. This was adaptive setup, not calibrated hand-step evidence.
- M3 used a separate reference after two individually checked M4 positive steps: M3=245.097896°, M4=116.977279°, M5=0°, M6=90°; predicted wrist (0.369774, 0.088189) m. Marker and fingers remained observable. This reference is very near the nominal projection bound; no further expansion was made.
- At each reference, three overlapping consecutive stationary pairs used four unique frames. They are correlated comparisons, not three independent replicates. All pairs passed ≤1 px centre-shift and the separately labelled ≤2 px pose-fit RMS screen before joint probes.

| Reference | Stationary angular differences (deg) | Centre shifts (px) | Max fit RMS (px) |
|---|---|---|---|
| M4 | 4.742, 2.977, 1.967 | 0.615, 0.492, 0.691 | 0.528 |
| M3 | 0.673, 0.660, 0.488 | 0.954, 0.296, 0.194 | 0.322 |

## Measured versus estimator: per joint/sign

Pair orientations are averaged on SO(3). A common camera-frame rotation axis was fixed from the first M4 positive excursion; later signed rotations are projected onto it. These are marker-based relative joint proxies, not encoder or absolute joint measurements. M3 positive produces the opposite forearm rotation to M4 positive, as predicted.

| Joint/sign | n | Estimator joint increment (deg) | Measured joint proxy mean ± sample SD (deg) | Range (deg) |
|---|---:|---:|---:|---:|
| M4+ | 4 | +10.0 | +11.492 ± 2.073 | +8.616 to +13.567 |
| M4- | 4 | -10.0 | -11.748 ± 0.485 | -12.187 to -11.138 |
| M3+ | 2 | +10.0 | +10.328 ± 0.102 | +10.256 to +10.400 |
| M3- | 2 | -10.0 | -10.073 ± 1.414 | -11.073 to -9.073 |

| Pose | Estimator forearm delta (deg) | Measured signed forearm delta (deg) | Stationary pair angle (deg) | Centre shift (px) | Max fit RMS (px) |
|---|---:|---:|---:|---:|---:|
| m4minus1 | -10.0 | -11.138 | 1.864 | 0.839 | 0.439 |
| m4return1 | +10.0 | +13.567 | 2.538 | 0.792 | 0.529 |
| m4plus1 | +10.0 | +8.616 | 1.484 | 0.487 | 0.455 |
| m4return2 | -10.0 | -11.582 | 2.611 | 0.396 | 0.314 |
| m4minus2 | -10.0 | -12.187 | 3.526 | 0.476 | 0.602 |
| m4return3 | +10.0 | +11.888 | 4.198 | 0.468 | 0.445 |
| m4plus2 | +10.0 | +11.896 | 1.816 | 0.315 | 0.417 |
| m4return4 | -10.0 | -12.085 | 5.425 | 0.219 | 0.612 |
| m3plus1 | -10.0 | -10.400 | 0.935 | 0.266 | 0.457 |
| m3return1 | +10.0 | +11.073 | 1.580 | 0.495 | 0.339 |
| m3plus2 | -10.0 | -10.256 | 0.449 | 0.205 | 0.413 |
| m3return2 | +10.0 | +9.073 | 1.094 | 0.311 | 0.371 |

## Measured excursions by cycle

| Joint/cycle | Nominal peak-to-peak (deg) | Measured signed-axis peak-to-peak (deg) |
|---|---:|---:|
| M4 / 1 | 20.0 | 22.199 |
| M4 / 2 | 20.0 | 23.781 |
| M3 / 1 | 10.0 | 11.081 |
| M3 / 2 | 10.0 | 10.256 |

## Excursion and returns

- M4 sampled nominal −10°, 0°, +10° twice; maximum nominal end-to-end excursion was 20°. M3 sampled 0° and +10° twice; no claim of a two-sided M3 sweep.

| Joint | Return pose | Marker centre difference (px) | Camera-frame marker position difference (mm) | Orientation difference (deg) |
|---|---|---:|---:|---:|
| M4 | m4return1 | 0.710 | 0.183 | 2.988 |
| M4 | m4return2 | 1.254 | 0.780 | 1.972 |
| M4 | m4return3 | 2.856 | 1.238 | 1.730 |
| M4 | m4return4 | 3.060 | 1.540 | 2.610 |
| M3 | m3return1 | 0.493 | 0.259 | 0.757 |
| M3 | m3return2 | 0.386 | 0.551 | 0.603 |

These are marker returns, not wrist errors. Do not attribute apparent return rotation entirely to backlash: stationary angular noise is material, especially at the M4 reference.

## Local trajectory comparison

- A single five-parameter planar camera-YZ registration plus rotating marker-to-wrist planar offset was fitted to the designated first subset: `noise3`, `m4minus1`, `m4plus1`, `m3noise3`, `m3plus1`. It was frozen before scoring the later six listed poses. No per-pose refitting.
- The local offset is fitted, not independently measured; the lateral marker offset cannot be separated from camera translation here. Full 3-D extrinsics and absolute wrist position remain unknown. Held-out samples repeat trained nominal settings, so this tests local consistency/repeatability, not unseen workspace generalization. Training includes both joint blocks and does not test transfer to M3 from M4 alone.
- Local fit Jacobian rank: 5/5; condition number: 125.121. Training marker-YZ RMS: 0.620 mm.
- Frozen-fit held-out marker-YZ RMS: 1.111 mm; maximum: 2.304 mm. These are fitted local marker residuals, NOT measured wrist accuracy.

| Held-out pose | Marker-YZ residual (mm) |
|---|---:|
| m4minus2 | 2.304 |
| m4return3 | 0.800 |
| m4plus2 | 0.363 |
| m4return4 | 1.104 |
| m3plus2 | 0.174 |
| m3return2 | 0.281 |

Local-only figures (not committed): `m3-m4-session/M4-contact-sheet.jpg`, `M3-contact-sheet.jpg`, `M4-trajectory.svg`, `M3-trajectory.svg`. Contact sheets draw image-space arrows from preceding poses and blue reference centres. Trajectory plots compare observed markers with the one frozen local model fit.

## Estimator end-to-end observations and limitations

- The estimator applied each observed command sign and held the other joints fixed during each block. States stayed IN_PROCESS_POS throughout measurement; no UNKNOWN state or reported model fault occurred. Estimates agreed with the local Python model cross-check numerically.
- Before final stow, sigmaDeg reached M3=9.013878°, M4=10.897247°, M5=0°. These uncertainty values are heuristic and not validated against a joint sensor; the estimator correctly accumulates uncertainty even after predicted return to the same joints.
- Autonomous inactivity stowing during the early pauses was not reflected in the dead-reckoned state until operator confirmation/reset. This is an estimator coverage gap: outside-command physical changes remain unobserved. Do not carry the old estimate across inactivity without physical checking.
- M3 negative outward motion from the higher visible reference was rejected offline by FAULT_PROJECTION. The negative M3 measurements are returns from +10°, not an independently sampled −10° endpoint.
- No ±20° M3 sweep, broad workspace, link lengths, absolute wrist pose, gripper-length envelope, hand-step size, independent camera extrinsics or statistical uncertainty calibration was validated.
- Decision: retain vendor constants unchanged. Relative directions and coarse 10° increments are supported locally; precision correction is not justified by this coverage and camera uncertainty.

## Cleanup and handoff

- Stow and manual-exit RPCs returned `ok`. Fresh status: idle (3), battery 81%, error_code 0. Michael then confirmed physical full stow in the confirmation form.
- `set_arm_anchor {anchor:"stowed"}` was sent only after that confirmation; exact readback was IN_ROBOT, joints all zero, wrist null, sigmaDeg all zero, stepsSinceAnchor 0.
- **robot released — 2026-10-13T13:34:07.023050+00:00**. Physically stowed and operator-confirmed; manual mode exited; confirmed-stow estimator anchor read back. No further robot commands planned.
