# Native ROS 2 Jazzy workspace

Run all commands from the repository root, not the retired `ros2/` subworkspace.
Pixi supplies native macOS ARM64 RoboStack Jazzy, Python, colcon, OpenCV/NumPy,
SciPy and pytest; `pixi.lock` pins its solved environment. Do not move old `.pixi`
environments: recreate with `pixi install` at the new absolute path.

## Standard commands

```sh
~/.pixi/bin/pixi install
~/.pixi/bin/pixi run build
~/.pixi/bin/pixi run test
~/.pixi/bin/pixi run view
```

`view` is the offline URDF/TF/RViz model viewer. `test` uses an isolated DDS domain
and fake backends; some optical regression tests use retained local calibration
images which are intentionally not distributed in a clean source checkout.
Do not reinterpret missing local evidence as a successful optical check.

For a fresh non-symlink install (no production services started):

```sh
~/.pixi/bin/pixi run colcon --log-base log-clean build --base-paths src --build-base build-clean --install-base install-clean
~/.pixi/bin/pixi run bash
source install-clean/setup.bash
ros2 launch z70_bringup bringup.launch.py --show-args
```

Package resources resolve via ament shares, not checkout ancestor walks. Description
owns immutable front/gripper lens JSON/YAML, mounts, model/meshes and floor geometry.
Bringup installs `share/z70_bringup/foxglove/z70-dashboard.json`. Explicit archive
paths are for analysis/tests; defaults do not search calibration directories.
Mutable arm/chassis state belongs under `$XDG_DATA_HOME/RRsZ70_ROS2` (fallback
`~/.local/share`), separate from installed shares and original observations.

## Live use — requires separate authorization

With the matching backend/sidecar deployed:

```sh
~/.pixi/bin/pixi run bringup web_gateway:=true
# Optional idle action server; no goal is implied by starting it:
~/.pixi/bin/pixi run bringup web_gateway:=true manipulation:=true
```

Ctrl-C shuts down owned ROS nodes. Compose services are stopped separately with
`docker compose -f deploy/compose.yaml stop`; never use `down -v` as rollback.
No custom supervisor/launcher is involved.

Bringup defaults contact live services, acquire camera frames, refresh vendor maps
and may persist chassis calibration. Default origin is `http://127.0.0.1:57010`;
`token_file` defaults to `$Z70_REPO/.local/service-token` (Pixi sets `Z70_REPO=$PWD`).
Use an absolute token-file override outside the checkout; never put token contents
in ROS parameters or command arguments. `mapping:=true`, `depth:=true`,
`manipulation:=false`, `web_gateway:=false` are defaults. `camera_hz:=0` disables
camera polling. Discovery and Foxglove remain localhost-only.

The ament gateway listens only on `127.0.0.1:57020`. On the qualified Docker Desktop
host, containers reach this native loopback listener at `host.docker.internal`;
Compose sets `Z70_ROS_GATEWAY_ORIGIN=http://host.docker.internal:57020`.
This does not bind the gateway to LAN or publish a new gateway port. The vendor
facade authenticates browser sessions and proxies the same ROS task registry;
setup/account secrets stay in the backend. A Linux host needs its own qualified
native/container arrangement; this macOS result is not a Linux networking claim.

## Principal topics and interfaces

- `/z70/state`: typed RobotState, known flags, stale/valid/source evidence.
- `/z70/arm_estimate`: typed command-derived ArmEstimate, not encoders.
- `/joint_states`: usable planning arm joints only; `/z70/display/joint_states`
  drives separate display TF placeholders.
- `/z70/commands`: dispatch-time CommandEvent with logical/attempt/session IDs.
- `/camera/{front,gripper}/image_raw`, `image_raw/compressed`, `camera_info`,
  `metadata`: one driver acquisition, paired stamps and calibration identity.
- `/z70/camera/state`, `/z70/camera/select`: shared selection, separate from pixels.
- `/z70/driver/map_snapshot`, `/z70/map/refresh`: private acquisition boundary.
- `/map`, `/odom`, `/z70/odom_status`, `/z70/chassis_calibration`: standard map/VO
  plus typed quality/calibration evidence. Snapshot/overlay is not localization.
- `/z70/pick` and bounded primitive actions, `/z70/stop`, raw RPC and anchor services:
  one driver scheduler, UNKNOWN retained, priority stop/cancel and no vendor fallback.
- `/diagnostics` and scalar dashboard topics: driver-owned presentation only;
  they do not poll an independent backend or become execution authority.

Foxglove: `ws://127.0.0.1:8765`. REP-103 base axes are x-forward/y-left/z-up;
optical axes are x-right/y-down/z-forward. Source image timestamps are host receipt,
not sensor exposure. Model/TF agreement is not independent physical validation.

## Optional depth: external cache and native MPS

The persistent out-of-process worker consumes existing compressed ROS images.
It never opens a second camera. Default interpreter stays
`~/.venvs/roboclaw-depth/bin/python`; Torch/Transformers remain outside Pixi/ROS.
No external environment/cache was removed or inspected by migration.

The retained model is `depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf`,
pinned to revision `8078d68a9c75a972131914f6afd0c1723be0da7f` (Hugging Face model API).
The worker uses `local_files_only=True` for processor and weights and defaults to
`mps`. Provision the optional environment/model separately, not on ROS startup:

```sh
python3 -m venv ~/.venvs/roboclaw-depth
~/.venvs/roboclaw-depth/bin/pip install torch transformers opencv-python pillow numpy huggingface_hub
~/.venvs/roboclaw-depth/bin/hf download depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf --revision 8078d68a9c75a972131914f6afd0c1723be0da7f
```

These optional setup commands were documented, not executed during offline cutover.
Weights use the standard Hugging Face external cache; never commit them. Availability,
finite cloud rate and off-floor metric accuracy are different qualification results.
Floor-fit agreement alone does not establish object clearance or collision avoidance.
