# RRsZ70_ROS2

ROS 2 Jazzy stack, 3D visualization, web operator console, and MCP control plane for the stock **Roborock Saros Z70** (`roborock.vacuum.a179`).

> **Note:** Independent community project; not affiliated with, endorsed by, or supported by Roborock.

## Getting Started

Target platform: **macOS Apple Silicon (`osx-arm64`)** with [Pixi](https://pixi.sh) on `PATH`. Run commands from the repository root:

```sh
npm ci
pixi install
pixi run build
pixi run bringup
```

`pixi run build` compiles the native `darwin/arm64` `go2rtc` binary into `.local/bin/go2rtc`, builds the web frontend (`dist/web`), and builds all ROS 2 packages. `pixi run bringup` launches the entire stack natively (`go2rtc`, the local vendor/web HTTP service on `127.0.0.1:57010`, `z70_web_gateway` on `127.0.0.1:57020`, `z70_driver`, `z70_mapping`, `z70_odometry`, `z70_manipulation`, and `foxglove_bridge` on `127.0.0.1:8765`).

Open **`http://127.0.0.1:57010`**. On first run, click **Start local setup** to authenticate your Roborock account, select your robot, enter its LAN IP and local protocol (`L01` or `1.0`), and save your camera PIN. Credentials and session tokens are stored locally in `.local/` (mode `0600`) and are redacted from ROS topics and MCP responses. See [`deploy/OPERATIONS.txt`](deploy/OPERATIONS.txt).

- Connect **Foxglove Studio** to `ws://127.0.0.1:8765` and import [`src/z70_bringup/foxglove/z70-dashboard.json`](src/z70_bringup/foxglove/z70-dashboard.json).
- Monocular depth is enabled by default when `~/.venvs/roboclaw-depth` is provisioned ([setup instructions](docs/ros2.md#optional-depth-worker)); pass `depth:=false` to disable it.

## Architecture Overview

- **`src/z70_interfaces`** — ROS 2 `.msg`, `.srv`, and `.action` definitions.
- **`src/z70_driver`** — Command scheduler, status/camera/map acquisition, `common_status` grab-mode/action decoder, and `ArmEstimator` (`/z70/arm_estimate`, `/joint_states`).
- **`src/z70_description`** — CAD-backed URDF/Xacro, STL meshes, camera intrinsics/extrinsics, and kinematic models ([`docs/z70-arm-model.md`](docs/z70-arm-model.md)).
- **`src/z70_odometry`** — Homography floor visual odometry (`/odom`), online chassis pulse calibrator, and floor appearance mosaic (`/z70/floor_mosaic`).
- **`src/z70_mapping`** — Roborock `v1` LiDAR map decoder (`/map`, `/z70/map/safe_grid`, `/z70/map/spatial_context`) and Apple Silicon MPS depth point clouds (`/camera/front/depth/points`).
- **`src/z70_manipulation`** — HSV perception, coordinated multi-axis joint planner, and manipulation/navigation action servers ([`src/z70_manipulation/README.md`](src/z70_manipulation/README.md)).
- **`src/z70_web_gateway`** — Loopback `rclpy` HTTP bridge (`127.0.0.1:57020`) backing both the web console and MCP server ([`docs/thin-api.md`](docs/thin-api.md)).
- **`apps/web` & `apps/mcp`** — React operator console (with manual arm lifecycle, press-and-hold jogging, and live telemetry) and stdio MCP server.
- **`adapters/vendor` & `deploy`** — Local Roborock LAN/MQTT transport service (`127.0.0.1:57010`) and native `go2rtc` WebRTC/MJPEG media sidecar (`127.0.0.1:57001`, WebRTC `:57012`).

## MCP Configuration

`apps/mcp/` exposes the ROS 2 gateway over stdio MCP:

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

[Documentation index](docs/README.md) · [ROS 2 guide](docs/ros2.md) · [Arm model & protocol](docs/z70-arm-model.md) · [API contract](docs/thin-api.md) · [Apache 2.0](LICENSE) · [Third-party notices](THIRD_PARTY_NOTICES)
