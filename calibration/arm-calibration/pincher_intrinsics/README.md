# Pincher intrinsics and conditional arm geometry

Offline tools only. The code does not contact the robot. Actual acquisition used the existing `../live.mjs` **MCP SDK** helper, bounded one-jog/capture/state blocks, and direct RoboClaw MCP tools. No robot HTTP/SDK/go2rtc fallback, ROS poller, Docker restart, grip/pick/clean/dock/navigation program or commit was used.

## Reproduce from the repository root

```sh
PY=calibration/arm-validation/.venv/bin/python
CODE=calibration/arm-calibration/pincher_intrinsics
D=calibration/arm-calibration/20261015T080103Z-pincher-intrinsics-arm
$PY $CODE/prepare.py "$D"
$PY $CODE/lens_report.py "$D" --bootstrap 200
$PY $CODE/arm_report.py "$D" --bootstrap 200
$PY $CODE/validation_report.py "$D"
(cd "$CODE" && ../../arm-validation/.venv/bin/python -m unittest -v)
```

Run from the repo root: retained image/log paths and baseline hash manifests are repo-relative. These CLIs write only the **new owned session's** `lens/`, `arm/`, `validation/` and observation files. Do not point them at an original baseline session. They do not call the old archive-writing `physical_base` command-line entry points or legacy integration tests. Original historical observations are read for identical-corner comparisons and independent longer-arc reconstructions.

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
- No production intrinsics, URDF, planner, firmware or shared mount changed. L2/mount and M4-zero/mount gauges remain exact null directions. See the session's gauge examples instead of interpreting an effective optical lever as physical L2.

Evidence is local/private and git-ignored, **not a remote backup**.
