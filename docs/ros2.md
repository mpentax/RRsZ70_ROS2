# ROS 2 Jazzy Guide

Commands run from the repository root. `pixi.toml` targets `osx-arm64` and pins RoboStack ROS 2 Jazzy and Python dependencies in `pixi.lock`.

```sh
pixi install
pixi run build
pixi run test
```

`pixi run test` builds the workspace and runs the Python/ROS package test suites in `ROS_DOMAIN_ID=91`. `npm run typecheck` and `npm test` cover the TypeScript packages (`adapters/vendor`, `apps/mcp`, `apps/web`).

---

## Launch

After starting the Docker sidecars (`docker compose -f deploy/compose.yaml up -d`) and completing setup at `http://127.0.0.1:57010`:

```sh
pixi run bringup web_gateway:=true
# With manipulation and navigation action servers enabled:
pixi run bringup web_gateway:=true manipulation:=true
```

| Launch Argument | Default | Description |
| :--- | :--- | :--- |
| `origin` | `http://127.0.0.1:57010` | Local vendor transport sidecar URL |
| `token_file` | `$Z70_REPO/.local/service-token` | Path to bearer token file (`Z70_REPO` defaults to `$PWD`) |
| `camera_hz` | `3.0` | Active camera polling rate in Hz (`0` disables polling) |
| `mapping`, `depth` | `true`, `true` | Enable vendor LiDAR map node and MPS monocular depth worker |
| `manipulation`, `web_gateway` | `false`, `false` | Enable manipulation action servers and HTTP gateway (`:57020`) |
| `web_gateway_port` | `57020` | Loopback port for `z70_web_gateway` |
| `foxglove_port` | `8765` | Loopback WebSocket port for `foxglove_bridge` |
| `camera_info`, `calibration` | Installed `z70_description` config | Front camera intrinsics and floor/extrinsics JSON |
| `depth_python` | `~/.venvs/roboclaw-depth/bin/python` | External Python interpreter for the MPS depth worker |

---

## Nodes and ROS Interfaces

`z70_driver` is the single ROS node communicating with the vendor sidecar. It owns command scheduling, `ArmEstimator` dead reckoning, and camera/map acquisition.

| Interface | Type / Role |
| :--- | :--- |
| `/z70/state` | `z70_interfaces/msg/RobotState` — robot state code, battery, `common_status`, `grab_status`, and source timestamps |
| `/z70/arm_estimate` | `z70_interfaces/msg/ArmEstimate` — dead-reckoned/anchored `M3..M6` joint state, validity, and uncertainty |
| `/joint_states` | `sensor_msgs/msg/JointState` — 8 URDF joints driving `robot_state_publisher` |
| `/z70/commands` | `z70_interfaces/msg/CommandEvent` — command dispatch/completion lifecycle stream |
| `/camera/{front,gripper}/image_raw`, `image_raw/compressed`, `camera_info`, `metadata` | Shared camera streams with paired `FrameMetadata` and calibrated `CameraInfo` (`RELIABLE + TRANSIENT_LOCAL` on compressed/info/metadata) |
| `/z70/camera/state`, `/z70/camera/select` | `CameraState` topic and `SelectCamera` service (`1` front, `2` gripper) |
| `/z70/driver/map_snapshot`, `/z70/map/refresh` | Raw vendor `v1` map bytes and `RefreshMap` service |
| `/map`, `/z70/map/safe_grid`, `/z70/map/markers`, `/z70/map/spatial_context` | Decoded `OccupancyGrid`, eroded safe interior grid, room/zone markers, and 8-sector clearance context |
| `/odom`, `/z70/odom_status`, `/z70/chassis_calibration`, `/z70/floor_mosaic` | Floor visual odometry, unicycle pulse fallback, online chassis gains, and floor mosaic |
| `/z70/stop`, `/z70/control/session`, `/z70/driver/raw_rpc`, `/z70/arm/anchor`, `/z70/tasks/session` | Priority stop, session enable/disable, raw vendor RPC, arm pose anchor, and task token ownership |
| `/z70/base/pulse`, `/z70/arm/step`, `/z70/arm/hand_step`, `/z70/arm/operation` | Driver motion primitives (`BasePulse`, `ArmStep`, `HandStep`, `ArmOperation`) |

See [`src/z70_manipulation/README.md`](../src/z70_manipulation/README.md) for the higher-level manipulation and navigation actions (`DeployArm`, `AlignToTarget`, `MoveArm`, `GraspAndLift`, `ReleaseAndStow`, `Pick`, `Stack`, `PinAndGo`, `DeliverGarment`, `PatrolCollect`).

---

## Installed Resources and Runtime State

- **Immutable package shares (`z70_description`, `z70_bringup`, `z70_odometry`):** CAD meshes, URDF/Xacro, camera intrinsics (`z70_front_camera_info.yaml`, `z70_gripper_camera_info.yaml`), mount/floor calibration (`mount_calibration.yaml`, `front_camera.json`, `pincher_geometry.yaml`), and the Foxglove layout (`z70-dashboard.json`).
- **Mutable runtime state (`$XDG_DATA_HOME/RRsZ70_ROS2`, default `~/.local/share/RRsZ70_ROS2`):**
  - `arm-estimate.json` (overridable via `arm_estimate_path`; pass `""` in tests to disable persistence)
  - `calibration/chassis/online.json` (overridable via `output_file`)

---

## Optional Depth Worker

The monocular depth node (`z70_mapping/depth_node.py`) runs a persistent worker process against `/camera/front/image_raw/compressed` using Apple Silicon MPS. Provision its isolated virtual environment and model weights once:

```sh
python3 -m venv ~/.venvs/roboclaw-depth
~/.venvs/roboclaw-depth/bin/pip install torch transformers opencv-python pillow numpy huggingface_hub
~/.venvs/roboclaw-depth/bin/hf download depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf --revision 8078d68a9c75a972131914f6afd0c1723be0da7f
```

The worker loads weights from the local Hugging Face cache (`local_files_only=True`) and publishes floor-scale-corrected 3D point clouds on `/camera/front/depth/points` and health telemetry on `/z70/depth/status`.
