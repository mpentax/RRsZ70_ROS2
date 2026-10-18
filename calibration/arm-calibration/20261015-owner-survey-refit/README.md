# Offline owner-survey refit

Read `RESULTS.md` first: **no geometry adoption**. This directory is separate from the untouched source archive `../20261015T061415Z-physical-base/`.

## Reproduce from the repository root

```sh
PY=calibration/arm-validation/.venv/bin/python
$PY calibration/arm-calibration/physical_base/survey_refit.py \
  calibration/arm-calibration/20261015T061415Z-physical-base \
  calibration/arm-calibration/20261015-owner-survey-refit/reproduction \
  --bootstrap 100
$PY calibration/arm-calibration/20261015-owner-survey-refit/verify.py
$PY calibration/arm-calibration/20261015-owner-survey-refit/diagnostics.py
$PY -m py_compile \
  calibration/arm-calibration/physical_base/survey_refit.py \
  calibration/arm-calibration/physical_base/test_survey_refit.py \
  calibration/arm-calibration/20261015-owner-survey-refit/verify.py \
  calibration/arm-calibration/20261015-owner-survey-refit/diagnostics.py
git diff --check
```

No installation or network is needed: the retained venv has numpy2.0.2, scipy1.13.1, OpenCV5.0.0. Commands use real retained images/corners. `reproduction/` is a fresh output location; it does not replace the retained run/report. `diagnostics.py` intentionally reads this directory's retained `pincher-fits.json`; it regenerates `residual-diagnostics.json`. `verify.py` checks this directory's pre-refit source snapshot and runs all12 offline tests. Temporary test copies live under the configured TMPDIR and are removed automatically.

**Do not run the existing unrestricted `python -m unittest` or old analyze/validate commands on the original session.** The old integration tests write `fit.json`, `validation.json`, overlays and other analyses into their session. `verify.py` redirects those two legacy test modules' session constants to a disposable full copy. The4 new tests independently check the mathematical constrained-pixel fit, content-sensitive provenance, whole-frame/pose split consistency and archive preservation, and alternate planar-start selection based on training pixels only. The numerical fixture is explicitly mathematical, not fabricated hardware evidence.

## Model and outputs

- `survey_refit.py` fixes optical-centre board-plane height inside the pixel objective (`C_z`), refitting orientation and in-plane camera centre; fixed intrinsics per scenario.
- Frozen original full corners and group files; all11 original pure-joint fits and conditional whole-pose holdouts recomputed.
- Scenarios: free;55/58/60/62/65mm; at60, front focal±1%,cy±20px,k1+0.012. These are scenarios, not measured uncertainty bounds.
- Same predetermined final whole-front-frame and pincher whole-pose holdouts in all cases. Two front starts use the first training exposure's alternate IPPE pose, never holdout errors. All-frame reconstruction and training-only validation are separately labeled.
- Separate100-draw front-frame bootstraps per block free/60 and200-draw model-free primary pincher pose bootstrap. Covariance/rank is conditional, not anatomy/absolute truth.
- No mutable scenario result cache: every run recomputes. Content provenance includes original JPEGs, capture log, frozen corners/grouping, original results, intrinsics, survey and implementation. Replacing image contents or log metadata with the same file count cannot silently preserve a scenario identity.
- `comparison.json` aggregates all scenarios; individual `scenario-*.json` survive intermediate progress. All scenario-heldout pincher identities are identical by construction. `archive-*-sha256.json` verifies every original archive file before/after, not only image counts.
- Rejected D cross-camera pairing remains rejected; no new chain or marker/optical-origin anatomy is claimed. Rough survey is an independent check only.

The output-path guard rejects the original source directory, any of its descendants, or any ancestor containing it. Read-only source evidence is never migrated or overwritten. No production file edits or live hardware access are part of these commands.
