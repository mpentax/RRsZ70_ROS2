# z70_odometry

Metric flat-floor visual odometry, calibrated command-model fallback, passive
chassis calibration and appearance-only floor mosaic. Coordinates are x-forward,
y-left, yaw-CCW. These nodes consume shared ROS evidence, not a new camera/vendor
connection.

## Build/run

From the repository root:

```sh
pixi run build
# Normal live observation, within its authorized scope:
pixi run bringup
# Or individual consumers in a sourced Pixi shell (need input publishers):
pixi run bash
source install/setup.bash
ros2 run z70_odometry floor_odometry
```

`floor_mosaic` and `chassis_calibrator` are separate executables also launched by
bringup. Installed ament resources supply default front calibration; no checkout
ancestor search or empty required calibration argument.

## floor_odometry

Input `/camera/front/image_raw` must match valid, nonstale FrameMetadata and
calibration identity/session through `z70_driver.frame_subscription.MatchedFrames`.
Dimensions/encoding must match calibrated `bgr8`; padded rows are supported.
Outputs are `/odom`, `/z70/odom_status` and `odom → base_footprint` TF.

| Parameter | Standalone default |
|---|---|
| `calibration` | Installed description `front_camera.json` |
| `camera_x_m` | Installed odometry.yaml initialization, 0.16 m |
| `max_range` | 1.5 m (near cutoff 0.06 m) |
| `n_features` | 3000, FAST threshold 10 |
| `ransac_thresh_m` | 0.008 m |
| `min_inliers` | 80 |
| `max_rms_m` | 0.003 m |

Normal bringup overrides camera_x with description mount calibration (0.275 m);
a ready typed passive fit can replace it in VO and mosaic. Keep initialization,
model mount and fitted offset distinct. See [calibration](../../docs/validation/chassis-odometry-trial.md).

Fit: ratio-matched ORB → calibrated floor points → two-point RANSAC → three tight
consensus refinements → camera-offset conjugation. Accepted pairs update pose;
zero covariance is unmodeled, not perfect accuracy. Rejected frames normally hold
pose and the last accepted image, allowing later overlap to recover displacement,
with planar covariance `1e6` so the mosaic excludes them.

Successful correlated RC pulses can propagate only intervals not covered by
accepted VO using a calibrated unicycle model. Pending intervals wait for motion/
reply completion; coverage is trimmed against pulse time, not just reply time.
Reference rebasing avoids double counting. Status `COMMAND_DEAD_RECKONED`,
`pair_accepted=false`, covariance `1e6` marks prediction, not measured travel.
A first returning image reference retains uncertainty until an accepted VO pair.
Unknown/rejected command delivery is not proof of displacement. Passive calibration
and mosaic exclude these model steps. TF broadcasts at 5 Hz, starting at zero even
without images; a fresh TF timestamp alone is not a new motion observation.

No loop closure, semantic floor segmentation or velocity estimate. Floor features
need overlap and a flat visible surface; synthetic rotation/translation limits
are test envelopes, not hardware motion guarantees.

## floor_mosaic

Matched front images join `/odom` at the exact image stamp using bounded
20-message caches. Frame must be odom; rejected/model-fallback covariance is
excluded. Per-cell running means map floor appearance, not occupancy/free space.

| Parameter | Default |
|---|---|
| `calibration`, `camera_x_m` | Same installed defaults/bringup override as VO |
| `resolution` | 0.005 m/cell |
| `size_m` | 4.0 m square, centred on initial odom origin |
| `max_range` | 1.2 m |

Outputs: `/z70/floor_mosaic` bgr8, `/z70/floor_mosaic_mask` mono8 (255 seen/0 unknown),
matching `/z70/camera_info`, and JSON `/z70/floor_mosaic_status`. Images/mask/info
are transient-local. Default 800×800 grid has origin (-2,-2) m; columns increase
odom x, rows increase odom y; cell centres are origin+(index+0.5)*resolution.
A normal image viewer does not automatically put robot-forward at image-up.
Out-of-bounds points drop; restart clears the mosaic. Exact-black warp samples
stay unknown, including genuinely black surfaces. Nonflat surfaces, projected
furniture, changing light and accumulated drift can smear appearance. No Nav2
map or traversability/collision guarantee is implied.

## Offline verification

```sh
pixi run test
pixi run bash -c 'source install/setup.bash && python -m unittest discover -s calibration/chassis -p "test_live_analysis.py"'
# Focused tests with installed ROS types:
pixi run bash -c 'source install/setup.bash && pytest src/z70_odometry/test -q'
```

Tests exercise synthetic translation/rotation, quality rejection, accepted-reference
recovery, command-pulse coverage and mosaic geometry/skip rules. Retained
[test results](test/RESULTS.md) distinguish synthetic from historical real floor
images. Private image audits may be unavailable in a clean checkout. Reproduction
scripts that emit reports need fresh output directories/test-owned XDG state;
never run them in place over retained evidence. See the
[passive calibration guide](../../docs/validation/chassis-odometry-trial.md) for
safe root-relative analyzer arguments and durable-state semantics.
