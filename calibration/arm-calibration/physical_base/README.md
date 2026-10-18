# Offline physical arm-base calibration tools

These modules contain no robot IO. The retained [physical-base results](../20261015T061415Z-physical-base/RESULTS.md) are a rejected/conditional optical experiment; private JPEGs are local, Git-ignored evidence, not remotely backed up. [Current CAD-backed anatomy](../../../docs/z70-arm-model.md) is distinct from what this experiment could identify.

## Reproduction boundary

The historical `.venv` is removed. More importantly, `analyze.py` and `validate.py` still read the removed `ros2/src/z70_description/config/mount_calibration.yaml`. They write `fit.json`, `holdout.json`, `uncertainty.json`, `validation.json`, observations and overlays **inside their session**. Do not advertise them as working current commands or run unrestricted unittest discovery: `test_analysis.py` and `test_validation.py` select the original archive and launch writing analyzers.

A valid source/math-only check from repository root, using the installed Pixi environment (`~/.pixi/bin` on PATH), is:

```sh
pixi run python -B -m unittest discover \
  -s calibration/arm-calibration/physical_base -p test_geometry.py -v
```

For a future full legacy replay, first copy the complete calibration tree, including private images, logs, grouping, manifests and code, to a **fresh** scratch tree. Adapt old resource dependencies only in that copy, preserving the frozen experiment's numerical inputs; do not symlink outputs into the originals or silently substitute today's CAD geometry for its old gauge. Hash original files before/after. See [fresh-output survey refit](../20261015-owner-survey-refit/README.md) for an analyzer that accepts a separate destination. Scratch outputs are not durable evidence until copied to a new owned archive.

### Historical command receipt — not current reproduction

```text
# Formerly run from this directory; removed environment/resources required.
PY=../../arm-validation/.venv/bin/python
$PY analyze.py ../20261015T061415Z-physical-base --bootstrap 200
$PY validate.py ../20261015T061415Z-physical-base
$PY -m unittest -v
```

## Methods and retained acquisition scope

- `observations.py`: legacy dense 10×14 ChArUco, 20/15 mm; raw DICT_4X4_1000 IDs/corners, board-specific front mask, elevated-ID68 association by plane height, metric board-marker fallback, positive-depth PnP alternatives and quality/reject flags. The front mask was established from this session's actual floor region, not a universal wrist row.
- `geometry.py`: SE(3) screw line, independent position-only circle line, robust equal-pose-weight corner-space axis fit with latent angles, dimensionless Jacobian spectrum/rank, and frozen-axis pose prediction. Axis station along its own direction is deliberately not an anatomical joint centre.
- `analyze.py`: raw inventory/detection, per-chassis-block free front pose, independent pure-joint fits, whole-pose conditional holdouts and model-free whole-pose bootstrap. `groups.json` is the auditable pure-joint block registry. Full-corner residuals are retained even when PnP initialization trims corners.
- `validate.py`: camera-centre circle crosscheck, fixed-mount other-camera pixel holdout, full translation AND orientation stationarity rules, three multistarts, explicit lens/print sensitivity scenarios, repeat/deployment scatter, M5 observed traces/hysteresis and L2/mount gauge examples. A failed pairing stays rejected even when its diagnostic pixel result is low.
- Tests cover numerical transformations/projection with explicitly mathematical data and actual retained images. They never manufacture hardware observations. The local real-image tests require the private evidence directory.

## Live acquisition provenance

Acquisition used RoboClaw MCP tools exclusively. The then-current `../live.mjs` MCP SDK helper opened the historical `src/mcp-main.ts` and invoked `robot_rpc`, `select_camera`, `get_camera_frame`, `get_arm_estimate`; it did not call the robot SDK or go2rtc directly. Each bounded command was followed by a retained frame and deterministic offline check before the next uncertain step. The driver closes each MCP client in `finally`.

Some original helper capture records have completion timestamp `t`, not full MCP receipt metadata. Do not relabel these as exposure times or fabricate missing camera-session provenance. Three frames were captured through the directly exposed MCP tool and retain its exact receipt metadata. `manifest.json` retains SHA-256 and bytes for every untouched original; rejects are retained. No continuous preview/poller, ROS launch, Docker rebuild, grip, pick, clean or dock command was started.

## Scope

Pure-joint fits identify a camera-relative axis line without wrist registration, physical L2 or exact M4 zero. They do not replace the full independent survey/remount/multi-tilt/full-chain protocol. Read `RESULTS.md`, `analysis-policy.json`, `survey.json`, `fit.json`, `holdout.json`, `uncertainty.json`, `validation.json`, and `final-state.json` before reuse. No production geometry was adopted.
