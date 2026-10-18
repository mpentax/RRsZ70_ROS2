> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [current arm model](../../docs/z70-arm-model.md), [camera/calibration guide](../../research/cameras/FINDINGS.md).

# Limited front-camera pose-simulator check

**Outcome: coarse M5 wrist-motion check supported; full pose simulator remains unvalidated.**

Seven accepted two-frame capture pairs covered four nominal wrist settings, then the reverse path. All arm-marker corners remained in frame during this wrist sweep. The marker black side was 15 mm.

| Sequence | Simulator M5 change from start (deg) | Camera-estimated marker rotation from start (deg) | Stationary-pair angular difference (deg) |
|---|---:|---:|---:|
| baseline-low | 0 | 0.0 | 6.3 |
| m5-plus-01 | 10 | 6.1 | 2.6 |
| m5-plus-02 | 20 | 19.3 | 1.2 |
| m5-plus-03 | 30 | 30.2 | 0.6 |
| m5-minus-01 | 20 | 21.1 | 0.4 |
| m5-minus-02 | 10 | 8.9 | 1.6 |
| m5-minus-03 | 0 | 4.8 | 0.6 |

## What the evidence supports

- Nominal total excursion: **30 deg**; marker estimate: **30.2 deg**.
- Positive and negative command blocks rotated in opposite senses along the empirically fitted excursion axis: **True**. This does not independently calibrate the axis in robot coordinates.
- Return marker-centre difference: **0.53 px**, approximately **0.72 mm** in this camera model. This is marker position, not measured wrist position.
- Return orientation estimate differs by **4.8 deg**. The starting stationary pair itself differs by **6.3 deg**, showing that angular precision near this viewing configuration is weak. Do not identify all of this apparent return difference as mechanical backlash.

## What remains unverified

- M3/M4 joint motion, link lengths, absolute end-effector positions and complete workspace geometry were not validated by this wrist-only sweep.
- Missing marker-to-wrist transform prevents independently verified wrist positions and arm-derived camera extrinsics.
- The supplied intrinsics were fitted to floor boards; upper-image pose estimates are extrapolations. The above numbers are provisional camera estimates, not encoder measurements.
- The fast reset/replay of five forward and four down steps did not reproduce the previous marker framing; an extra down step was needed. Command timing, initial-state repeatability, target shifts or body/camera movement have not been isolated. ACKs must not be treated as proof that every simulator increment physically executed.
- No vendor constants were adopted or corrected; the original full-protocol decision remains inconclusive.

Evidence: `log.jsonl`, `LIMITED_POSE_SUMMARY.json`, `LIMITED_POSE_CONTACT_SHEET.jpg`, and `limited-*-analysis.json`. Cleanup stow and manual-exit commands returned `ok`; physical stowing has not been independently confirmed.
