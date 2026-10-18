# RRsZ70_ROS2

ROS 2 Jazzy for the stock Roborock Saros Z70: command/status-derived arm estimation,
CAD-backed arm geometry and TF, shared cameras, floor odometry, vendor LiDAR maps,
optional monocular depth, and manipulation actions.

**Independent community project; not affiliated with, endorsed by, or supported by Roborock.**

## Getting started

Supported manifest target: **macOS Apple Silicon (`osx-arm64`)**. Install
[Pixi](https://pixi.sh) on `PATH`, Node.js **24 or newer**, and Docker Desktop.
Run commands from the repository root.

```sh
npm ci
npm run build
pixi install
pixi run build
docker compose -f deploy/compose.yaml build app go2rtc
docker compose -f deploy/compose.yaml up -d
```

Open **http://127.0.0.1:57010** and choose **Start local setup** when no camera
PIN is saved. Request the Roborock email code, choose the region and device, supply
its LAN IP and local protocol (`L01` or `1.0`), and save the camera PIN. Saving a
PIN ends setup; sign in with that PIN. Configured installations use the same PIN login. Handle all setup
inputs in the browser, not an agent conversation.

Secrets are stored locally in private `.local/` files (plaintext, mode 0600),
excluded from ROS messages and MCP results. **Account authentication and camera
signaling communicate with Roborock services**; local storage does not mean
network-free authentication. See [operations](deploy/OPERATIONS.txt).

Once paired, start live observation and the web/agent ROS facade:

```sh
pixi run bringup web_gateway:=true
```

Depth is enabled by default; [optional depth setup](docs/ros2.md#optional-depth)
provides the external environment and cached weights. `depth:=false` disables it.
Bringup acquires live images/status/maps; it does not command motion just by
starting. Add `manipulation:=true` for idle manipulation servers; a live action
goal can move the robot.

Connect Foxglove to **ws://127.0.0.1:8765** and import
[`z70-dashboard.json`](src/z70_bringup/foxglove/z70-dashboard.json).
Ctrl-C stops the ROS launch; `docker compose -f deploy/compose.yaml stop` stops
the sidecars. Use the console's **Stop control & streams** to suspend robot
control and viewing explicitly.

## ROS capabilities

- `z70_driver` owns scheduling, acquisition and `/z70/arm_estimate`. The estimator
  tracks commands/status and observation-backed anchors, not encoder telemetry or
  continuous ArUco fusion. Task-local visual recovery is separate from that estimate.
- `z70_description` supplies copied CAD geometry, meshes, camera calibration and
  the canonical TF tree. See the [arm model](docs/z70-arm-model.md) for consumer boundaries.
- `z70_odometry` supplies floor VO, command-model fallback and a floor appearance mosaic;
  `z70_mapping` decodes vendor SLAM into `/map` and supplies optional depth clouds.
- Shared camera selection exposes **front and gripper streams**, not an exhaustive
  physical camera inventory. Both lenses have installed calibration.
- [Manipulation actions](src/z70_manipulation/README.md) implement pickup and stacking.
  One supervised pickup/hold is retained evidence, not repeatable pickup or stack qualification.

## Any MCP-compatible agent

`apps/mcp/` is a thin stdio MCP client of the same ROS gateway used by the web UI.
Configure your MCP host with absolute paths (replace the example checkout path):

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

State, images, map, task handles, raw RPC, anchors, stop and session control all
use shared ROS interfaces; MCP does not launch ROS or bypass its scheduler.

[Documentation index](docs/README.md) · [ROS guide](docs/ros2.md) ·
[API contract](docs/thin-api.md) · [Apache 2.0](LICENSE) ·
[Third-party notices](THIRD_PARTY_NOTICES)
