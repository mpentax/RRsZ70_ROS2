# Marker68 orientation-first trial — RESULTS

## Outcome
Angular calibration evidence acquired; **no production calibration adopted**. Board centred by observed bounded chassis yaw while stowed, then chassis frozen for all arm measurements. Captured independent M3/M4/M5 changes, two M5 0→40→0 sweeps at different arm pitch configurations, fixed-floor pincher-camera poses, and rotation-chain mixed-pose holdouts. No grip, pick, cleaning, docking or map navigation.

**Coverage is partial:** M3 has only one fully front-visible 10° step, not the requested ≥3 settings/20–40° forward-and-return range. M4 has a useful multi-setting first group but only one measured step at the second M3 level. Last attempted M3 reverse was ACKed but its frame failed; it is NOT counted as a measured return. Camera failures curtailed the remaining revisit, not setup-only abandonment. No unobserved jog was replayed. Total 38 jog attempts, within the 45 bound.

## Measured rotations, not encoder angles
| group | measurable consecutive changes | mean ± SD, deg |
|---|---:|---:|
| M4_240 | 4 | 9.24 ± 1.20 |
| M3_105 | 1 | 10.30; single step, SD unavailable |
| M4_250 | 1 | 9.52; single step, SD unavailable |
| M5_A | 8 | 10.25 ± 1.66 |
| M5_B | 8 | 10.36 ± 1.79 |

Nominal numbers in filenames/CSV identify **command counts**, not measured encoders or reproducible absolute joints. The estimator flagged a reach-limit refusal during the route; physical images still changed. Thus no estimator refusal is promoted into a physical zero-angle claim. The first M5 steps were only about 6.8–7.6°, later steps around 10–12°; gains are not uniformly exact 10°.

M5 A outward/return totals: 40.87° / 40.85°, endpoint closure 0.77°. M5 B: 41.51° / 40.84°, closure 0.83°. Mid-sweep approach/return discrepancies are roughly 1–3° and are not separable from pose noise at this scale.

### Axes (positive-command direction, unit vectors)
F = OpenCV front optical x-right, y-down, z-forward. W = this trial's fixed ChArUco board coordinates, NOT a surveyed chassis frame. W transformations use the current block only.

| joint/configuration | axis F | axis W |
|---|---|---|
| M3 | -0.9995, -0.0313, -0.0095 | -0.9995, +0.0192, -0.0254 |
| M4 | +0.9945, +0.0966, +0.0401 | +0.9946, -0.0513, +0.0900 |
| M5_A | -0.0425, +0.8422, +0.5375 | -0.0428, -0.5575, +0.8291 |
| M5_B | -0.0249, +0.7342, +0.6785 | -0.0271, -0.6959, +0.7176 |

Long-arc M3/M4 unsigned parallelism discrepancy 4.14°; insufficient evidence to resolve a real mechanical skew from marker/lens noise. M5 axis transported between the two pitch configurations agrees within 1.33°. Axis stationarity scatter from individual short steps is retained in results.json; it is several degrees and grows for weak increments.

## Noise / ambiguity / ground crosschecks
Front stationary pilots: 5 frames at each of two visibly different tilts. RMS to each SO(3) mean 1.71° and 0.96°; maximum stationary pair differences 4.10° and 2.09°. This is not 1° precision. Per-step Monte Carlo intervals propagate the larger pilot noise through 500 draws; these are conditional random-noise intervals, NOT systematic-inclusive calibration accuracy.

Pincher stationary pilots (3 each): full-orientation RMS 0.047° / 0.013°. Optical-axis angles to the floor span the recorded poses; initial pilots approximately 43.36° and 53.29°. Every accepted G board pose and angle is in results.json. A 20° nominal M3 excursion measured 18.68° in G; M5 A40 measured 39.39° in G versus 40.87° in F. The latter ~1.5° disagreement is compatible with the front pilot noise but is not an independent mount calibration.

F/G/F bracket orientation changes 0.50–1.37°; streams are sequential, never simultaneous. Current F floor orientation RMS scatter 0.74°, max 2.23°. Body rocking cannot be separated from floor-board fitting noise; W-frame axis accuracy carries that uncertainty. Per-exposure R_W_M is retained when its own floor pose passes, otherwise null. No old floor TF or fixed floor height was imposed.

Both IPPE_SQUARE branches, positive-depth checks, ordered decoded corners, residuals, matrices and quaternions are retained. Primary refined SUBPIX marker detector and fallback board sequence were implemented before deployment. Branch selection is positive-depth minimum reprojection, with a separately audited 0.25px-tie continuity rule; 0 changes versus minimum reprojection. No FK/desired angle used for branch choice. Near-planar ambiguity/lens extrapolation and print flatness remain systematic caveats. Marker ID68 is selected above the floor-board mask, not confused with the floor ID68. The printed black side is 15mm, not the 20mm paper/checker square.

## Held-out mixed rotation chain
Withhold all B rolled poses. Learn only A roll observations and the pure M4 change D4 = R_B0 R_A0ᵀ; predict R_Broll = D4 R_Aroll. Four held-out mixed poses at nominal 10/20/30/40 have errors 1.17°, 2.12°, 1.78°, 0.72°, RMS **1.55°**. This checks rotational composition and axis transport, not command-only FK, full-chain zeros or physical link lengths. Holdout intent existed during acquisition, but exact analysis membership was saved after capture: treat this as exploratory validation, not strictly preregistered independent confirmation. No B rolled observations enter D4 or A predictions.

## Identifiability / adoption
Supported provisionally: M3/M4 are approximately oppositely signed parallel pitch rotations; M5 is a terminal roll transported by preceding pitch, about 41° for four outward steps in these configurations. Retain measured axes/step uncertainty for further calibration; **do not overwrite shared model constants** from these short arcs.

**Rotation matrices contain no link lengths.** Physical L1/L2, M5 axial station, marker mounting location and shoulder axis location are not estimated by this rotation-only trial. Stored marker translations are secondary 15mm-scale PnP trajectories only, not a length fit. Physical M3/M4 zeros and a free fixed mount orientation have rotational gauge freedoms; no unique mechanical zeros or physical wrist reference were fitted. Existing ~180mm L1 is neither validated nor refuted here.

## Evidence, counts and reproduction
- 91 retained original JPEGs, including preflight and successful final cleanup.
- 96 capture requests total = 94 SDK + 2 direct MCP. 4 failed requests; one early successful SDK image was not retained due an inline-image logger defect, documented separately. 90 SDK retained images + one direct preflight = retained originals.
- 57 full marker orientation observations; 77 accepted floor poses across F/G; 34 null marker views (G intentionally included).
- 22/23 consecutive step comparisons measurable; rejected/missing step is null, never zero.
- raw/*.jpg + raw/*.json: originals, hashes, detections, rejected counts, both marker PnP branches, board corners/residuals.
- log.jsonl: exact MCP calls, parameters, results, receipt time/exposure-null, camera identity/dimensions and dead-reckoned estimates.
- step_angles.csv / step_rotations.json: relative SO(3), axes, normal tilt, old-marker-z swing/twist and conditional intervals. Euler intrinsic XYZ is secondary presentation only in orientation-diagnostics.json.
- Reproduce: `calibration/arm-validation/.venv/bin/python calibration/arm-calibration/20261015T104441Z-marker68-rotation/test_rotations.py` (4 tests pass), then `.../diagnostics.py`. Analyzer has fixed output in its own fresh session directory; tests do not write archives. No old calibration archive or shared model was edited; no commit.

## Final cleanup
Final image raw/final-front.jpg visually shows pincher/attached marker absent and board still present. Final raw status state=3, error=0, battery=56%, no clean/return; pickup bits cleared, program_state=0. Front selected; both camera streams have empty consumer lists. Estimate IN_ROBOT is dead reckoning initialized by stow, not encoders. Stow and mode-exit ACKs alone were not used as proof: delayed raw status plus final image were checked.

Camera failure diagnosis: robot RPC link remained healthy, MQTT/PIN/ICE/DTLS connected, but acquisition reported zero video tracks/packets and timed out. Camera selection recovery did not restore frames. No direct robot transport, Docker/service restart or continuous polling was used. An initial pickup-mode exit was rejected (-10007); app_stop followed by settled stow restored idle and final capture. Cleanup now verified, camera root cause unresolved.
