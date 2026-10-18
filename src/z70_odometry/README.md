# z70_odometry

Offline-tested metric floor VO and appearance mapping for the calibrated 1600×1200 front camera. Flat-floor rays set metric scale; x forward, y left, yaw CCW. `camera_x_m=0.16` is still a **guess**: use `calibration/chassis/analyze.py` and `docs/validation/chassis-odometry-trial.md` to measure it before treating base displacement as physically calibrated.

## Build / run

From the repository root:

```bash
~/.pixi/bin/pixi run bash -c 'colcon build --symlink-install --base-paths src'
# Terminal 1 (requires image publisher; these nodes do not contact the robot)
~/.pixi/bin/pixi run bash -c 'source install/setup.bash; ros2 run z70_odometry floor_odometry'
# Terminal 2
~/.pixi/bin/pixi run bash -c 'source install/setup.bash; ros2 run z70_odometry floor_mosaic'
```

### `floor_odometry`

Input `/camera/front/image_raw` paired with valid metadata: calibrated dimensions,
`bgr8`, supports padded rows. Outputs `/odom` (`nav_msgs/Odometry`), TF
`odom → base_footprint` and typed `/z70/odom_status` (`z70_interfaces/OdometryStatus`).
Calibration defaults resolve through installed ament resources, not cwd.

| Parameter | Default | Meaning |
|---|---:|---|
| `calibration` | empty | Required calibration JSON path |
| `camera_x_m` | 0.16 | Camera forward offset from base origin, metres |
| `max_range` | 1.5 | Geometric floor feature range, metres (near cutoff 0.06) |
| `n_features` | 3000 | ORB budget, FAST threshold 10 |
| `ransac_thresh_m` | 0.008 | Initial rigid-fit consensus radius |
| `min_inliers` | 80 | Accepted refined consensus minimum |
| `max_rms_m` | 0.003 | Refined inlier radius and final RMS ceiling |

Fit: ratio-matched ORB → metric floor points → 2-point RANSAC → three tighter consensus refinements → camera-offset conjugation. Rejected views hold pose and retain the **last accepted image**; an overlapping later view can recover the entire displacement. Rejections publish planar pose covariance diagonal `1e6` so the mosaic skips that view. Accepted covariance is zero/unmodelled, **not zero uncertainty**. TF also holds during rejection; consumers must read status. No twist/velocity estimate, loop closure, object segmentation or command integration. Keep relative views around ≤15° / ≤6 cm at the ~1 Hz snapshot rate; this is a synthetic overlap envelope, not a hardware motion guarantee.

### `floor_mosaic`

Exact image-stamp `/camera/front/image_raw` + `/odom` join, bounded 20-message caches. Odometry must share the image stamp and use `odom`; rejected held poses are not fused.

| Parameter | Default | Meaning |
|---|---:|---|
| `calibration` | empty | Same calibrated camera JSON |
| `camera_x_m` | 0.16 | Must equal VO offset |
| `resolution` | 0.005 | Metres per mosaic cell |
| `size_m` | 4.0 | Fixed square centred on initial odom origin |
| `max_range` | 1.2 | Floor-ray radial range per view, metres |

Outputs (mosaic/mask transient-local, depth 1):
- `/z70/floor_mosaic`: `sensor_msgs/Image`, `bgr8`, `frame_id=odom`; per-cell running mean appearance.
- `/z70/floor_mosaic_mask`: `mono8`, 255 seen / 0 unknown, same stamp/geometry.
- `/z70/floor_mosaic_status`: JSON with resolution, origin xy, dimensions, observed/overlap/total cells and overlap NCC before fusion.

Default is 800×800 cells. Columns increase odom x, rows increase odom y; cell centre is `origin + (index+0.5)*resolution`, origin `(-2,-2)` m. An ordinary image viewer therefore does not automatically display robot-forward as image-up; transform the raster using metadata. Out-of-map points are dropped. Restart clears the mosaic. Exact-black samples remain unknown (warp borders **and** truly all-black surfaces); the mask distinguishes unknown from dark observed floor. This is **appearance**, not occupancy or evidence of traversability. Furniture projected onto the floor, non-flat floors, drift and changing lighting can smear it; no Nav2/free-space map is claimed.

## Offline verification (no bridge / network / robot)

```bash
~/.pixi/bin/pixi run bash -c 'set -e; source install/setup.bash; python -m unittest discover -s src/z70_odometry/test -p "test_*.py"; python src/z70_odometry/test/synthetic_check.py; python src/z70_odometry/test/real_floor_check.py src/z70_odometry/test/real-floor-results.json; python src/z70_odometry/test/ros_mosaic_check.py; python ../calibration/chassis/test_synthetic.py'
```

ROS check uses isolated domain 87, starts installed **only these two executables**, sends rendered translation/rotation views and checks odometry, image/mask geometry, overlap NCC, rejection skip and accepted-reference recovery; kills its child nodes on exit. Result JSONs live beside the tests. Real-floor audit uses visually confirmed board-free kitchen frames (`camera-fixed-frame-{1,2}`, `deploy-frame-2`), plus a conservative floor-only pixel ROI to avoid counting shoes/furniture. Feature-rich full mask does not prove every feature is floor; the small ROI still rejects 6 cm/15° warps due to overlap. See `test/RESULTS.md` for measured limits and numbers. Photos must remain local/git-ignored.

## Bringup integration (not edited here)

Alongside the existing odometry node, use the same resolved calibration path and forward offset:

```python
Node(package='z70_odometry', executable='floor_mosaic',
     parameters=[{'calibration': calibration, 'camera_x_m': 0.16,
                  'resolution': 0.005, 'size_m': 4.0, 'max_range': 1.2}]),
```
