# RRsZ70_ROS2

A **ROS 2 Jazzy** robotics stack, 3D visualization suite, web console, and **MCP** control plane for the stock **Roborock Saros Z70** (`roborock.vacuum.a179`).

`RRsZ70_ROS2` turns an unmodified Z70 with its 5-DOF deployable arm and dual cameras into a native ROS 2 platform—complete with a 3D digital twin, hybrid arm state estimation, visual odometry, LiDAR map decoding, monocular depth clouds, closed-loop manipulation actions, and agent-ready MCP tools.

> **Disclaimer:** This is an independent community project and is not affiliated with, endorsed by, or supported by Roborock.

---

## Getting Started

### Prerequisites
- **macOS (Apple Silicon)** or Linux with [Pixi](https://pixi.sh) in your `PATH` (manages ROS 2 Jazzy, Python 3.12, OpenCV, and `colcon`)
- **Node.js `>= 24.0.0`**
- **Docker** (runs the local protocol sidecar and `go2rtc` camera streamer)

### 1. Install & Build
```sh
npm ci && npm run build
pixi install
pixi run build
```

### 2. Pair Your Robot (Local-Only Auth)
Start the local sidecar and media containers:
```sh
docker compose -f deploy/compose.yaml build app go2rtc
docker compose -f deploy/compose.yaml up -d
```
Open **`http://127.0.0.1:57010`** and complete the setup form (Roborock account login, device selection, robot LAN IP — found in the Roborock app under **Settings > Product Info > Wi-Fi Name** — local protocol `L01`/`1.0`, and camera PIN).

**Authorization never leaves your machine:** credentials, device keys, PINs, and tokens are stored strictly on your host inside `.local/`. They never cross ROS topics or enter MCP/LLM context.

### 3. Launch ROS 2 Bringup
```sh
pixi run bringup web_gateway:=true manipulation:=true
```
This launches the ROS 2 driver, arm state estimator, floor visual odometry, LiDAR map decoder, monocular depth worker, manipulation action servers, Foxglove bridge (`ws://127.0.0.1:8765`), and web gateway (`127.0.0.1:57020`).

### 4. View the 3D Digital Twin
Connect **Foxglove Studio** to `ws://127.0.0.1:8765` and import `src/z70_bringup/foxglove/z70-dashboard.json`.

---

## Highlights

- **ROS 2–First Architecture:** Standard ROS 2 nodes, topics, services, and actions (`src/z70_*`) own command scheduling, kinematics, TF trees, camera pipelines, odometry, mapping, and manipulation.
- **Hybrid Arm State Estimator:** Because stock Z70 hardware does not stream joint encoder telemetry, `z70_driver` includes a custom ROS 2 estimator that fuses recovered vendor forward/inverse kinematics and command tracking with live ArUco fiducial registration on the pincher housing to publish real-time `/z70/arm_estimate` and `/joint_states`.
- **Closed-Loop Manipulation:** Composable ROS 2 actions (`Pick`, `Stack`, `DeployArm`, `AlignToTarget`, `MoveArm`, `GraspAndLift`, `ReleaseAndStow`) combine perception, visual chassis servoing, and collision-checked 10° joint planning:
  ```sh
  pixi run pick yellow
  pixi run stack yellow blue purple
  ```
- **SLAM Map, Depth & Visual Odometry:** Decodes the robot's native `v1` LiDAR map into `/map` with room contours and 8-sector clearance raycasts (`/z70/map/spatial_context`), paired with homography floor visual odometry (`/odom`), an online chassis gain calibrator, a live floor mosaic, and Apple Silicon MPS monocular depth clouds (`/camera/front/depth/points`).

---

## Connect Any AI Agent via MCP

`apps/mcp/` provides a standard stdio **Model Context Protocol (MCP)** server that connects **any MCP-compatible AI agent** (Claude Desktop, Cursor, OpenCode, Hermes, or custom agents) to your ROS 2 stack over loopback HTTP:

```json
{
  "mcpServers": {
    "rrsz70": {
      "command": "node",
      "args": ["/absolute/path/to/RRsZ70_ROS2/apps/mcp/src/mcp-main.ts"],
      "env": {
        "ROBOCLAW_MCP_ORIGIN": "http://127.0.0.1:57010",
        "ROBOCLAW_MCP_TOKEN_FILE": "/absolute/path/to/RRsZ70_ROS2/.local/service-token"
      }
    }
  }
}
```

**Exposed MCP Tools:**
- **State & Vision:** `get_state`, `refresh_status`, `get_camera_frame`, `get_camera_diagnostics`, `select_camera`, `get_arm_estimate`, `set_arm_anchor`, `get_map`
- **Manipulation & Motion:** `start_task` (`pick`, `stack`, `deploy_arm`, `align_target`, `move_arm`, `grasp_and_lift`, `release_and_stow`, `base_pulse`, `arm_step`, `hand_step`, `arm_operation`), `list_tasks`, `get_task`, `cancel_task`, `robot_rpc`
- **Safety:** `stop` (priority preemptive stop), `control_session` (pause/resume control and streams)

---

## Repository Structure

- `src/z70_interfaces` — Custom ROS 2 `.msg`, `.srv`, and `.action` definitions
- `src/z70_driver` — Telemetry, command scheduler, hybrid arm estimator, and camera publisher
- `src/z70_description` — URDF/Xacro, CAD STL meshes, camera intrinsics, and 5-DOF kinematic model
- `src/z70_odometry` — Homography floor visual odometry, online chassis calibrator, and floor mosaic
- `src/z70_mapping` — Roborock `v1` LiDAR map decoder, spatial context, and MPS depth node
- `src/z70_manipulation` — HSV perception, trajectory planner, and `pick` / `stack` action servers
- `src/z70_web_gateway` — Loopback `rclpy` HTTP bridge (`:57020`) for Web UI and MCP
- `src/z70_bringup` — Launch files and Foxglove Studio dashboard layout
- `apps/web` & `apps/mcp` — React operator console and stdio MCP server
- `adapters/vendor` & `deploy` — Local Roborock LAN/MQTT transport sidecar and Docker Compose setup

## Further Docs & License

- [`docs/ros2.md`](docs/ros2.md) — Launch arguments, ROS 2 topics, and MPS depth environment setup
- [`src/z70_manipulation/README.md`](src/z70_manipulation/README.md) — Manipulation actions and CLI guide
- [`src/z70_mapping/README.md`](src/z70_mapping/README.md) — LiDAR SLAM map decoder and spatial context
- [`docs/z70-arm-model.md`](docs/z70-arm-model.md) — Recovered 5-DOF arm kinematics
- Licensed under **Apache 2.0** ([`LICENSE`](LICENSE), [`THIRD_PARTY_NOTICES`](THIRD_PARTY_NOTICES)).
