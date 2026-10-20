# ROS 2 Lyrical Guide

Commands run from the repository root. `pixi.toml` targets `osx-arm64` and pins RoboStack ROS 2 Lyrical, Python, Go, and Node.js dependencies in `pixi.lock`.

```sh
pixi install
pixi run build
pixi run test
```

`pixi run test` builds the workspace and runs the Python/ROS package test suites in `ROS_DOMAIN_ID=91`. `npm run typecheck` and `npm test` cover the TypeScript packages (`adapters/vendor`, `apps/mcp`, `apps/web`).

---

## Launch

Start the full native stack (`go2rtc`, the vendor/web HTTP service on `127.0.0.1:57010`, `z70_web_gateway` on `127.0.0.1:57020`, `z70_driver`, `z70_mapping`, `z70_odometry`, `z70_manipulation`, and `foxglove_bridge` on `127.0.0.1:8765`):

```sh
pixi run bringup
# Or against the local MuJoCo physics & camera simulator (no hardware required):
pixi run sim
```

| Launch Argument | Default (`bringup.launch.py` / `pixi run bringup`) | Description |
| :--- | :--- | :--- |
| `origin` | `http://127.0.0.1:57010` | Local vendor transport service URL |
| `token_file` | `$Z70_REPO/.local/service-token` | Path to bearer token file (`Z70_REPO` defaults to `$PWD`) |
| `sidecars` | `false` (`true` in `pixi run bringup`) | Launch native `go2rtc` (`:57001` / `:57012`) and vendor/web HTTP service (`:57010`) |
| `sim` | `false` (`true` in `pixi run sim`) | Launch `z70_sim` MuJoCo simulator sidecar on `:57010` instead of hardware sidecars |
| `manipulation`, `web_gateway` | `false`, `false` (`true` in `pixi run bringup`) | Enable manipulation action servers and HTTP gateway (`:57020`) |
| `garment_perception`, `garment_backend` | `false`, `grounding_dino_clipseg` (`true`, `sim_hsv` in `pixi run sim`) | Enable `garment_perception` node and select model backend (`grounding_dino_clipseg`, `grounding_dino`, `clipseg`, `sim_hsv`, `contour_fallback`) |
| `towing_qualified`, `towing_artifact_uri` | `false`, `""` (`true`, `"sim:mujoco-qualified"` in `pixi run sim`) | Enable qualified garment towing and bind the qualification artifact URI on `pick_server` |
| `camera_hz` | `3.0` | Active camera polling rate in Hz (`0` disables polling) |
| `mapping`, `depth` | `true`, `true` | Enable vendor LiDAR map node and MPS monocular depth worker |
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
| `/map`, `/z70/map/safe_grid`, `/z70/map/dynamic_obstacles`, `/z70/map/markers`, `/z70/map/spatial_context` | Decoded `OccupancyGrid`, eroded safe interior grid, session-scoped dynamic obstacle grid, room/zone markers, and 8-sector clearance context |
| `/z70/world/objects`, `/z70/world/markers`, `/z70/world/update` | Session-scoped `map`-frame registry of world objects classified by mobility (`pickupable`, `draggable`, `stationary`) and objective relevance (`objective`, `completed`, `deferred`, `ignored`) with Foxglove 3D `MarkerArray` |
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
