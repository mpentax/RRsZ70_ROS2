# Ongoing chassis calibration

From the repository root, within separately authorized live acquisition, run normal observation bringup: `~/.pixi/bin/pixi run bringup`. No special trial, pulse schedule, per-step confirmations or calibration-only motion.

The private backend emits authenticated live dispatch events; `z70_driver` publishes typed `/z70/commands` without polling compatibility history. `chassis_calibrator` passively pairs them with timestamped `/odom` and typed `/z70/odom_status`. Source session/sequence gaps and UNKNOWN delivery remain explicit. Interrupted/overlapping moves, rejected odom, missing poses or changing/unknown camera offsets are **skipped**, never task errors or robot stops. Calibration is published as typed `/z70/chassis_calibration` and mutable state lives in durable application data, not installed shares.

Windowed least squares (last 500 valid samples) learns per-sign yaw gains, forward gain and camera_x, with sample counts and fit-only sigma. Estimates/seen IDs persist atomically in the configured durable application-data path. Installed immutable chassis records seed new state once; the repository measurement archive is never a runtime write target. `/z70/chassis_calibration` publishes the typed summary, also exposed by MCP `get_state` through the ROS gateway.

Odometry and mosaic consume the measured offset when camera fit has ≥4 turning samples and sigma ≤0.01 m (node parameters `min_camera_n` / `max_camera_sigma_m`). Gains never replace visual motion estimates. Sigma excludes floor-calibration, lateral-offset and slip bias: inspect the residual and offline cross-check rather than treating it as absolute accuracy. Continuous overlapping motion may provide no isolated settled windows; it remains observable but isn't incorrectly assigned to individual commands.

Offline cross-check: `~/.pixi/bin/pixi run python ../calibration/chassis/analyze.py ../calibration/chassis/<session>/pairs.json`. Offline tests: `~/.pixi/bin/pixi run test` and `~/.pixi/bin/pixi run chassis-calibration-test`. Calibration never issues robot commands.
