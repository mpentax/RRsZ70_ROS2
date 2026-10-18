# Offline physical arm-base calibration

No robot IO is present in these Python modules. Retained evidence is local/private under `../20261015T061415Z-physical-base/`; JPEGs are git-ignored, not remotely backed up.

From this directory:
```sh
PY=../../arm-validation/.venv/bin/python
$PY analyze.py ../20261015T061415Z-physical-base --bootstrap 200
$PY validate.py ../20261015T061415Z-physical-base
$PY -m unittest -v
```

- `observations.py`: legacy dense 10×14 ChArUco, 20/15 mm; raw DICT_4X4_1000 IDs/corners, board-specific front mask, elevated-ID68 association by plane height, metric board-marker fallback, positive-depth PnP alternatives and quality/reject flags. The front mask was established from this session's actual floor region, not a universal wrist row.
- `geometry.py`: SE(3) screw line, independent position-only circle line, robust equal-pose-weight corner-space axis fit with latent angles, dimensionless Jacobian spectrum/rank, and frozen-axis pose prediction. Axis station along its own direction is deliberately not an anatomical joint centre.
- `analyze.py`: raw inventory/detection, per-chassis-block free front pose, independent pure-joint fits, whole-pose conditional holdouts and model-free whole-pose bootstrap. `groups.json` is the auditable pure-joint block registry. Full-corner residuals are retained even when PnP initialization trims corners.
- `validate.py`: camera-centre circle crosscheck, fixed-mount other-camera pixel holdout, full translation AND orientation stationarity rules, three multistarts, explicit lens/print sensitivity scenarios, repeat/deployment scatter, M5 observed traces/hysteresis and L2/mount gauge examples. A failed pairing stays rejected even when its diagnostic pixel result is low.
- Tests cover numerical transformations/projection with explicitly mathematical data and actual retained images. They never manufacture hardware observations. The local real-image tests require the private evidence directory.

## Live acquisition provenance

Acquisition used RoboClaw MCP tools exclusively. The existing `../live.mjs` MCP SDK helper opened `src/mcp-main.ts` and invoked `robot_rpc`, `select_camera`, `get_camera_frame`, `get_arm_estimate`; it did not call the robot SDK or go2rtc directly. Each bounded command was followed by a retained frame and deterministic offline check before the next uncertain step. The driver closes each MCP client in `finally`.

Some original helper capture records have completion timestamp `t`, not full MCP receipt metadata. Do not relabel these as exposure times or fabricate missing camera-session provenance. Three frames were captured through the directly exposed MCP tool and retain its exact receipt metadata. `manifest.json` retains SHA-256 and bytes for every untouched original; rejects are retained. No continuous preview/poller, ROS launch, Docker rebuild, grip, pick, clean or dock command was started.

## Scope

Pure-joint fits identify a camera-relative axis line without wrist registration, physical L2 or exact M4 zero. They do not replace the full independent survey/remount/multi-tilt/full-chain protocol. Read `RESULTS.md`, `analysis-policy.json`, `survey.json`, `fit.json`, `holdout.json`, `uncertainty.json`, `validation.json`, and `final-state.json` before reuse. No production geometry was adopted.
