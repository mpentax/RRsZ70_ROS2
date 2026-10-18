# Chassis trial results

**DON'T ADOPT** — yaw_positive: need >= 4 measured samples; yaw_positive: duration dependence untested; yaw_negative: need >= 4 measured samples; yaw_negative: scatter unavailable or >20%; yaw_negative: duration dependence untested; forward: need >= 4 measured samples; rotation-circle RMS unavailable or >2 mm; positive/negative yaw gains differ >10%

Primary: live_ros_odom. Attempted pairs: 3; live measurements: 3; whole-pair VO accepted/rejected: 2/1.

camera_x_m: 0.27689338183032103; fit sigma: 0.005747794398940843 m; rotation-circle RMS: 0.0021460372513460823 m.

| Fit | n | Gain | Per-pulse gain std | Duration means (ms: gain) |
|---|---:|---:|---:|---|
| yaw_positive | 2 | 0.718568 | 0.07221610449547555 | 400: 0.647864, 600: 0.749993 |
| yaw_negative | 1 | 0.642029 | None | 400: 0.642029 |
| forward | 0 | unavailable | unavailable | unavailable |
| forward_positive | 0 | unavailable | unavailable | unavailable |
| forward_negative | 0 | unavailable | unavailable | unavailable |

Forward lateral RMS: None m; yaw RMS: None deg.

| Pulse | ms | Live/primary yaw deg | Fitted-base dx m | dy m | Pair VO yaw deg |
|---|---:|---:|---:|---:|---:|
| turn-plus-01 | 400 | 7.42397 | -0.001535 | 0.002843 | 7.64477 |
| turn-minus-01 | 400 | -7.35710 | 0.000511 | 0.001334 | -7.13427 |
| turn-plus-02 | 600 | 12.89142 | -0.000600 | -0.000987 | unavailable |

VO rejected/missing: turn-plus-02: floor VO rejected

gains multiply command times seconds; camera_y=0; pure rotations assumed. Live camera motion recovered by undoing recorded ROS offset; fit uncertainty excludes calibration error/slip. VO failures do not remove live measurements.

Duration trend criterion: three durations; range of duration-group means <=20% of fitted gain. No constants are automatically changed.
Physical release: UNVERIFIED (RPC delivery is not physical completion).
