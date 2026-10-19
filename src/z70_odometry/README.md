# z70_odometry

Metric flat-floor visual odometry (`floor_odometry`), online chassis pulse gain calibration (`chassis_calibrator`), and floor appearance mosaic (`floor_mosaic`). Coordinates follow REP-103 (`+x` forward, `+y` left, yaw CCW).

## Build and Run

From the repository root:

```sh
pixi run build
pixi run bringup
```

To run an individual node inside a sourced Pixi shell (`pixi run bash && source install/setup.bash`):

```sh
ros2 run z70_odometry floor_odometry
```

---

## `floor_odometry`

Consumes `/camera/front/image_raw` paired with `FrameMetadata` and `CameraInfo`, and publishes `/odom`, `/z70/odom_status`, and the `odom -> base_footprint` TF transform.

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `calibration` | Installed `z70_description` `front_camera.json` | Front camera intrinsics and floor homography |
| `camera_x_m` | `0.16 m` (`0.275 m` via `bringup.launch.py`) | Front camera forward offset from `base_footprint` |
| `max_range` | `1.5 m` (near cutoff `0.06 m`) | Floor projection range window |
| `n_features` | `3000` (FAST threshold `10`) | ORB feature budget |
| `ransac_thresh_m` | `0.008 m` | 2-point SE(2) RANSAC inlier threshold |
| `min_inliers` | `80` | Minimum inliers required to accept a frame pair |
| `max_rms_m` | `0.003 m` | Maximum RMS residual for an accepted frame pair |

- **Visual Odometry Fit:** Ratio-matched ORB features are projected onto the calibrated floor plane and solved with 2-point SE(2) RANSAC followed by three consensus refinements and camera-offset conjugation.
- **Unicycle Command Fallback:** When visual frame pairs are rejected (e.g., low floor texture or motion blur), correlated `BasePulse` commands propagate `/odom` using calibrated unicycle gains (`COMMAND_DEAD_RECKONED`, `pair_accepted = false`, covariance `1e6`).

---

## `floor_mosaic`

Accumulates matched front-camera frames and accepted `/odom` poses into a top-down metric floor appearance grid:

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `resolution` | `0.005 m/cell` | Grid resolution (`5 mm/px`) |
| `size_m` | `4.0 m` | Square map span (`800 x 800` cells centered at `(-2, -2) m`) |
| `max_range` | `1.2 m` | Maximum forward projection distance per frame |

Publishes `/z70/floor_mosaic` (`bgr8`), `/z70/floor_mosaic_mask` (`mono8`), `/z70/camera_info`, and `/z70/floor_mosaic_status` (`RELIABLE + TRANSIENT_LOCAL`).

---

## Tests

```sh
pixi run test
pixi run bash -c 'source install/setup.bash && pytest src/z70_odometry/test -q'
```
