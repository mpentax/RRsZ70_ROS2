# Offline owner-survey refit

Read [RESULTS.md](RESULTS.md): this optical refit was **not adopted**. Its failures and conditional identifiability are evidence about that experiment, not absence of a structural model today. Use [the current arm guide](../../../docs/z70-arm-model.md) for copied CAD anatomy and the optical/CAD/proxy consumer split.

## Safe reproduction from the repository root

The old `calibration/arm-validation/.venv/bin/python` environment is removed. The current manifest supplies Python, NumPy, SciPy and OpenCV through Pixi (macOS Apple Silicon); ensure `~/.pixi/bin` is on PATH and the environment is installed. Private retained images and JSON inputs must exist locally; Git alone is not a backup of them.

`survey_refit.py` reads its session and writes the separate second positional argument. Allocate a new scratch directory for each run; never choose this retained report directory or its `reproduction/` subdirectory:

```sh
SCRATCH_ROOT="${TMPDIR:-$HOME/.cache}/rrsz70-replays"
export TMPDIR="$SCRATCH_ROOT"
mkdir -p "$SCRATCH_ROOT"
S=$(mktemp -d "$SCRATCH_ROOT/z70-survey-refit.XXXXXX")
pixi run python -B calibration/arm-calibration/physical_base/survey_refit.py \
  calibration/arm-calibration/20261015T061415Z-physical-base \
  "$S/refit" --bootstrap 100
```

The analyzer rejects source/output ancestor overlap, but permits an existing unrelated output directory: the fresh name above is important. Its archive-before/after SHA-256 files and original JPEG manifest check must agree. Scratch is disposable; copy any desired new outputs to a separately named durable evidence directory, without replacing the archive.

`verify.py` and `diagnostics.py` are **historical fixed-location helpers**, not current commands. `verify.py` compares obsolete absolute checkout keys and writes this retained directory's `verification.json`; its legacy analyzer tests also require removed `ros2/src/...` resources. `diagnostics.py` writes `residual-diagnostics.json` beside itself. Do not run either in place. A full replay requires a separate complete evidence/code copy and deliberate adaptation of those historical roots/hashes, never replacement of retained manifests or restoration of old paths in this checkout.

### Historical command receipt (not executable current guidance)

```text
PY=calibration/arm-validation/.venv/bin/python
$PY calibration/arm-calibration/physical_base/survey_refit.py calibration/arm-calibration/20261015T061415Z-physical-base calibration/arm-calibration/20261015-owner-survey-refit/reproduction --bootstrap 100
$PY calibration/arm-calibration/20261015-owner-survey-refit/verify.py
$PY calibration/arm-calibration/20261015-owner-survey-refit/diagnostics.py
```

The retained `tests.txt` and `all-tests.txt` are distinct earlier/later receipts; do not rewrite either to a new count. The numerical fixtures are mathematical, not hardware observations.

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
