# Offline odometry / floor mosaic results

All measurements below are **offline**, not hardware chassis calibration. No robot MCP/HTTP calls, bridge launch, `.local/` access or git commit. JSONs retain full numeric output.

## Real board-free floor

Vision inspection identified `calibration/arm-validation/camera-fixed-frame-{1,2}.jpg` and `deploy-frame-2.jpg` as uncovered tiled kitchen floor **without boards**. Shoes, basin and furniture are present; `limited-baseline-low` and `marker-search-deploy` were excluded because boards are visible.

| Region | Features frame 1 / frame 2 / deploy | Ratio matches (same-session pair) | Refined inliers | RMS |
|---|---|---:|---:|---:|
| Full geometric floor mask | 3000 / 3000 / 3000 | 1342 | 1000 | 0.857 mm |
| Conservative near bare-floor crop, u=[80,1150), v=[900,1190) | 1796 / 1737 / 1850 | 175 | 150 | 0.351 mm |

No feature shortage in this tiled scene. The full geometric mask also includes above-floor objects: its feature count is **not** a pure floor count. The small crop better isolates tile and excludes the obvious shoes/furniture. The same-session pair is approximately stationary but has no measured ground truth; it tests repeatability, not commanded-motion accuracy.

Baseline full-mask rendered 3 cm / 6 cm / 15° views were rejected because broad noisy consensus inflated RMS. Three tighter metric consensus refinements now accept 3 cm, 6 cm, 5° and 15° on the full rendered board-free scene: translation errors 0.033 / 0.655 / 0.084 / 0.170 mm; yaw errors ≤0.022°. Original ORB/FAST settings and ≥80 inlier / ≤3 mm RMS limits are retained. This is a painted-flat-world rendering, including the objects; it does **not** prove real furniture parallax is handled.

On the floor-only crop, rendered 3 cm and 5° recover within 0.039 mm / 0.006°; 6 cm and 15° still reject for limited common crop support. Boards and small initial pulses are therefore useful for the later trial; no dense fallback is claimed.

## Original board synthetic regression

All seven in-envelope motions pass assertions (<0.4 mm translation, <0.08° yaw); measured worst translation rounds to 0.3 mm and worst yaw to 0.05°. The 30° view is rejected. Unlike the previous script, expected rejection does not make the reported worst error infinite and unexpected failures produce a nonzero exit.

## Metric mosaic through native ROS 2

Full `colcon build --symlink-install --base-paths src`: **4 packages built**. Installed `floor_odometry` and `floor_mosaic` ran in isolated ROS domain 87. Five rendered poses (forward/reverse and ±5° increments, up to 6 cm total and 10° absolute yaw) passed real image/odom/mosaic transport checks.

- Grid: 800×800 image/mask, 5 mm/cell, odom origin (-2,-2) m, per-view radial range 1.2 m.
- End-to-end overlap NCC: **0.9466–0.9937**; 62,133 observed cells after five views.
- Maximum odom sequence error: **0.1118 mm / 0.0167°**.
- Core geometric alignment test NCC **0.9951**; deliberate wrong-pose control NCC **0.6295**.
- A 40° out-of-envelope view was rejected, its held pose was not fused, and return to the initial view recovered from the retained accepted reference.
- Empty/black views, outside-map poses and invalid dimensions have explicit checks. Exact black is left unknown; truly black surfaces can therefore have holes. Visual inspection of `ros-mosaic-preview.png` found the board locally recognizable but the wider mosaic fragmented/radially stretched: non-floor imagery is still projected under the planar-floor assumption. High NCC verifies repeatability of that projection, not semantic floor correctness. Texture appearance is not occupancy; no loop closure or global drift correction.

## Chassis trial analysis

`calibration/chassis/test_synthetic.py` generates before/after PNG pairs from the real board photo and executes both library and CLI analysis. All **9 pairs** accepted (6 turns, 3 forward).

| Quantity | Synthetic truth | Recovered |
|---|---:|---:|
| Camera forward offset | 0.230000 m | 0.229706 m |
| Yaw gain | 0.600000 | 0.600762 |
| Forward gain | 0.550000 | 0.549307 |

Offset error **0.2943 mm**; rotational circle residual RMS **0.1412 mm**. Trial analyzer uses a 6000-feature budget because 3000 rejected two of the known-offset turn pairs; quality thresholds are unchanged. These constants are **not** the robot's measured gains. The real 0.16 m offset remains unchanged pending the separate supervised session.

## Reproduce

From `ros2/`, inside `~/.pixi/bin/pixi run bash -c '…'`:

```bash
colcon build --symlink-install --base-paths src && source install/setup.bash &&
python -m unittest discover -s src/z70_odometry/test -p 'test_*.py' &&
python src/z70_odometry/test/synthetic_check.py &&
python src/z70_odometry/test/real_floor_check.py src/z70_odometry/test/real-floor-results.json &&
python src/z70_odometry/test/ros_mosaic_check.py &&
python ../calibration/chassis/test_synthetic.py
```

All passed. Only the odometry package, chassis calibration directory and new chassis protocol were edited by this work; other concurrent workspace edits were not modified. Bringup still needs the mosaic node entry shown in the package README.
