# Pincher intrinsics and conditional arm geometry

Offline analysis only; no robot IO. The [original lens/arm experiment](../20261015T080103Z-pincher-intrinsics-arm/RESULTS.md) remains unchanged as dated evidence. Its asymmetric pincher lens was **adopted later** in the installed [gripper CameraInfo](../../../src/z70_description/config/z70_gripper_camera_info.yaml); its non-adopted anatomical fits did not become validated by that lens adoption. [Current arm geometry](../../../docs/z70-arm-model.md) uses CAD structural anatomy and separate optical/contact registrations.

## Safe reproduction from repository root

The old calibration `.venv` is removed; use the installed Pixi environment (Python, NumPy, SciPy and OpenCV, `~/.pixi/bin` on PATH). Private retained images are required. These CLIs write observations, `lens/`, `arm/` and overlays **in the positional session directory**. Its `-pincher-intrinsics-arm` suffix check does not protect existing archives.

Copy the **complete calibration tree** into fresh scratch, retaining the directory layout and sibling baseline sessions. Run copied code from the copied root: `prepare.py` computes image paths relative to `D.parents[2]`, `lens_report.py` reads those relative paths and sibling old lens data, and arm reconstruction resolves older sessions relative to its own code. A lone session copy or output symlink is insufficient.

```sh
SCRATCH_ROOT="${TMPDIR:-$HOME/.cache}/rrsz70-replays"
export TMPDIR="$SCRATCH_ROOT"
mkdir -p "$SCRATCH_ROOT"
S=$(mktemp -d "$SCRATCH_ROOT/z70-pincher-replay.XXXXXX")
cp -R calibration "$S/calibration"
pixi run bash -c '
  set -eu
  cd "$1"
  CODE=calibration/arm-calibration/pincher_intrinsics
  D=calibration/arm-calibration/20261015T080103Z-pincher-intrinsics-arm
  python -B "$CODE/prepare.py" "$D"
  python -B "$CODE/lens_report.py" "$D" --bootstrap 200
  python -B "$CODE/arm_report.py" "$D" --bootstrap 200
' -- "$S"
```

All regenerated files remain in the disposable copy. Hash the original tree before/after, keep rejects, and retain desired new outputs only in a separately named durable evidence directory. No command above updates installed calibration.

`validation_report.py` is a **historical full-report helper**: its absolute `baseline-before-sha256.json` keys bind to the old checkout/content, so relocation or documentation curation can invalidate its assertion. It also writes `validation/` and `counts.json` in place. Do not present it or unrestricted legacy tests as a portable successful replay; preserve original manifests and stdout rather than rewriting hashes to make them pass. A full historical replay requires deliberate path/provenance adaptation solely in a complete disposable copy.

### Historical command receipt — removed environment / original archive target

```text
PY=calibration/arm-validation/.venv/bin/python
CODE=calibration/arm-calibration/pincher_intrinsics
D=calibration/arm-calibration/20261015T080103Z-pincher-intrinsics-arm
$PY $CODE/prepare.py "$D"
$PY $CODE/lens_report.py "$D" --bootstrap 200
$PY $CODE/arm_report.py "$D" --bootstrap 200
$PY $CODE/validation_report.py "$D"
(cd "$CODE" && ../../arm-validation/.venv/bin/python -m unittest -v)
```

## Method and original acquisition interpretation

`prepare.py` recomputes raw ChArUco corners without a lens/FK assumption. It records image SHA-256, detector code, log, board geometry and a cache key. For this actual session, regeneration produced the identical observation-file SHA-256 before and after (`1cd1b54a6a4d068d4d5dcb9c48b55aa94366a618fa981ee9c69d1f8a23e14499`).

- `calibrate.py`: raw board detection; pinhole/no-distortion/k1/k1k2 models; held-out views; extrinsic-nuisance-projected, dimensionless intrinsic Jacobian SVD; conditional checker-distance tests.
- `lens_report.py`: fixed whole-pose splits, training-only grouped 5-fold model selection, final holdouts, 200 whole-pose calibration refits, coverage overlays, proposed standard plumb_bob JSON. `lens/proposed-camera-info.yaml` is an offline ROS export, not deployed configuration.
- `arm_analysis.py`: fixed camera/marker registration, common-station axis separation and conditional translation profile. Anatomical L2/M4 zero/M5 station are not invented.
- `arm_report.py`: old/new lenses on the same new/archived corners; independent pure-joint SE3, position-circle and pixel-axis fits; whole-pose conditional geometry holdouts; F/G/G/F translation-and-orientation eligibility; marker mount with independent front-marker holdouts; SVD, multistart, random/systematic distinctions and effective second-assembly vectors.
- `validation_report.py`: independent historical lens test, image-region residuals, lens-bootstrap propagation, conditional L1 profile, analytic gauge-equivalent profiles, front-ID68 elbow crosscheck, actual observed roll/hysteresis, inventory and original/baseline hashes.
- Tests use actual retained images/artifacts and separately labelled mathematical solver fixtures. **No synthetic data are represented as hardware evidence.** The real-image tests need the local private archives.

## Interpretation safeguards

- Board: legacy10×14, 20mm checker /15mm black, DICT_4X4_1000. Elevated pincher ID68 also has **15mm black side**, not20mm; historical expected69 is not silently relabelled. Front floor-board mask excludes the elevated region; branch alternatives and full residuals are retained.
- Commands and filename angles are nominal inputs. Two reach-limited shoulder commands produced almost no measured rotation, so labels drifted. Grouping records those observed equivalences; intrinsics/axes never impose exact nominal angles or arm FK.
- Read `RESULTS.md` in the session, not a training RMS alone. A held-out board pose is free nuisance geometry; it is not an independent ruler. A conditional orientation-informed axis holdout is not command-only accuracy.
- F/G cameras are a shared selectable stream, not simultaneous. First-frame receipt is retained for the directly acquired preflight. Helper log `t` is completion time, **not an exposure or exact frame receipt timestamp**; other exposure/sequence metadata are unavailable. Preserve that limitation.
- `002-deployed-gripper.jpg` is an unsuccessful selection/setup filename: the actual MCP camera and1200-pixel height are **front**. It is excluded from pincher fitting. Subsequent gripper selection was confirmed and delivered896-pixel frames.
- The first attempted mode-entry/arm-out sequence did not extend the arm despite ACK. Later observed mode/action and a bounded arm-out recovered actual gripper video; no uncertain jog was replayed blindly.
- Print tolerance, rigid planarity and mechanical landmark stations are not precisely surveyed. The visible printed areas looked approximately planar; white margin curl is not automatically marker deformation. Rough tape values remain independent checks, including the rough60mm camera height.
- At acquisition/report time no production intrinsics, URDF, planner, firmware or shared mount changed. The L2/mount and M4-zero/mount null directions describe that optical objective, not absence of a CAD structural model. See the session's gauge examples instead of interpreting an effective optical lever as physical L2.

Evidence is local/private and git-ignored, **not a remote backup**.
