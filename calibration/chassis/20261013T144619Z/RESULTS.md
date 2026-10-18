# Chassis trial results

**DON'T ADOPT** — yaw_positive: need >= 4 measured samples; yaw_positive: scatter unavailable or >20%; yaw_positive: duration dependence untested; yaw_negative: need >= 4 measured samples; forward: need >= 4 measured samples; rotation-circle RMS unavailable or >2 mm

Primary: live_ros_odom. Attempted pairs: 1; live measurements: 1; whole-pair VO accepted/rejected: 1/0.

camera_x_m: 0.2666743758499333; fit sigma: 0.014756666683674757 m; rotation-circle RMS: 0.002119374196588659 m.

| Fit | n | Gain | Per-pulse gain std | Duration means (ms: gain) |
|---|---:|---:|---:|---|
| yaw_positive | 1 | 0.718726 | None | 400: 0.718726 |
| yaw_negative | 0 | unavailable | unavailable | unavailable |
| forward | 0 | unavailable | unavailable | unavailable |
| forward_positive | 0 | unavailable | unavailable | unavailable |
| forward_negative | 0 | unavailable | unavailable | unavailable |

Forward lateral RMS: None m; yaw RMS: None deg.

| Pulse | ms | Live/primary yaw deg | Fitted-base dx m | dy m | Pair VO yaw deg |
|---|---:|---:|---:|---:|---:|
| turn-plus-01 | 400 | 8.23599 | 0.002114 | 0.000152 | 8.41240 |

VO rejected/missing:

gains multiply command times seconds; camera_y=0; pure rotations assumed. Live camera motion recovered by undoing recorded ROS offset; fit uncertainty excludes calibration error/slip. VO failures do not remove live measurements.

Duration trend criterion: three durations; range of duration-group means <=20% of fitted gain. No constants are automatically changed.
Physical release: UNVERIFIED (RPC delivery is not physical completion).
