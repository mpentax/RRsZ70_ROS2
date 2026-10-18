# ROS 2 Jazzy

Commands run from the repository root. `pixi.toml` supports `osx-arm64` and pins
native RoboStack Jazzy/Python dependencies through `pixi.lock`.

```sh
pixi install
pixi run build
pixi run test
```

`test` builds first and runs package tests in DDS domain 91. Some optical tests
need retained private images; missing originals are not successful optical checks.
`npm run typecheck` and `npm test` cover the TypeScript surfaces separately.

## Launch

After [browser pairing and sidecar startup](../README.md#getting-started):

```sh
pixi run bringup web_gateway:=true
# Optional idle action servers; a live goal commands motion:
pixi run bringup web_gateway:=true manipulation:=true
```

Bringup reads live status, images and maps and can save arm/chassis state. Ctrl-C
stops owned ROS nodes; sidecars are stopped separately. Do not launch observation
outside the operator's live-access scope.

| Launch argument | Default | Meaning |
|---|---|---|
| `origin` | `http://127.0.0.1:57010` | Private sidecar |
| `token_file` | `$Z70_REPO/.local/service-token` | File path only; Pixi sets `Z70_REPO=$PWD` |
| `camera_hz` | `3.0` | Requested sampling rate; `0` disables polling |
| `mapping`, `depth` | `true`, `true` | Vendor map observer and optional depth worker |
| `manipulation`, `web_gateway` | `false`, `false` | Idle action servers and shared HTTP facade |
| `web_gateway_port` | `57020` | Loopback only |
| `foxglove_port` | `8765` | Loopback WebSocket |
| `camera_info`, `calibration` | Installed description config | Front CameraInfo and floor/lens JSON |
| `depth_python` | `~/.venvs/roboclaw-depth/bin/python` | External worker interpreter |

Use an absolute `token_file` when running outside the checkout; never pass its
contents as a parameter. Docker Desktop connects the app to the native gateway
through `host.docker.internal`; see [operations](../deploy/OPERATIONS.txt).

## Ownership and interfaces

`z70_driver` alone consumes private transport/acquisition APIs. It owns command
scheduling, the command/status-derived estimator and shared camera/map acquisition.
Mapping, odometry and manipulation consume ROS interfaces, not another vendor
client. Web/MCP use the [shared gateway](thin-api.md).

| Interface | Semantics |
|---|---|
| `/z70/state` | RobotState with source evidence, validity and stale flags |
| `/z70/arm_estimate` | ArmEstimate: planning usability is its typed joints/validity, not TF presence |
| `/joint_states` | All eight visualization joints, including retained deployed UNKNOWN poses; one canonical `robot_state_publisher` |
| `/z70/display/joint_states` | Compatibility presentation publication, not a second display TF tree |
| `/z70/commands` | CommandEvent with logical/attempt/session identities and delivery uncertainty |
| `/camera/{front,gripper}/image_raw`, `image_raw/compressed`, `camera_info`, `metadata` | Shared acquisition, paired stamp/frame identity and installed lens calibration |
| `/z70/camera/state`, `/z70/camera/select` | Shared physical selection state and SelectCamera service |
| `/z70/driver/map_snapshot`, `/z70/map/refresh` | Typed acquired map and driver refresh service |
| `/map`, `/odom`, `/z70/odom_status`, `/z70/chassis_calibration` | Vendor map, floor VO/model fallback and typed quality/calibration |
| `/z70/stop`, `/z70/control/session`, `/z70/driver/raw_rpc`, `/z70/arm/anchor`, `/z70/tasks/session` | Stop, acquisition/control lifecycle, raw RPC, non-moving anchor, task ownership |
| `/diagnostics` | Presentation of shared evidence, not independent command authority |

Driver primitives: BasePulse `/z70/base/pulse`, ArmStep `/z70/arm/step`,
HandStep `/z70/arm/hand_step`, ArmOperation `/z70/arm/operation`.
[Manipulation](../src/z70_manipulation/README.md) adds seven actions including Pick
and Stack. Standard `ros2 action list -t`, `ros2 interface show`, `ros2 topic info -v`
and `ros2 service list -t` expose the generated contracts.

REP-103 base axes: x-forward/y-left/z-up; optical: x-right/y-down/z-forward.
Image timestamps are host receipt, not measured exposure. TF/visual joints can
remain populated while a planning estimate is invalid. ArUco recovery in a task
is not continuous visual fusion into the authoritative estimator.

## Foxglove and measurement

Connect to `ws://127.0.0.1:8765` and import the
[dashboard](../src/z70_bringup/foxglove/README.md). Overview is map-fixed; odometry
views use odom and the arm close-up uses base_footprint. Map snapshot poses do not
prove current localization. Compressed images/CameraInfo/metadata retain the latest
sample across lens switches; inspect timestamps before treating a panel as live.

For isolated checks use a distinct DDS domain, unused Foxglove port **and test-owned
`XDG_DATA_HOME`**. DDS isolation alone does not isolate estimator/calibration writes.
Use RELIABLE + TRANSIENT_LOCAL for late `/tf_static`/`/robot_description` subscribers.
For large raw images, native Jazzy's best-effort `ros2 topic hz` can undercount;
measure with a reliable depth-10 subscriber and disclose that QoS. Count delivered
messages over a timed warm interval, not only configured timers. Stop only owned
validation processes.

`pixi run view` still launches the legacy RViz/sliders model viewer; it is not a
Foxglove command. Primary onboarding uses Foxglove; legacy viewer removal is a
separate runtime change.

## Installed resources and durable state

Description owns immutable model/meshes, front/gripper lens configs, mounts and
floor geometry; bringup installs its dashboard. Defaults resolve ament shares,
not checkout ancestors or historical calibration directories. Runtime arm history
is `$XDG_DATA_HOME/RRsZ70_ROS2/arm-estimate.json` (`arm_estimate_path` overrides;
an empty value disables persistence for tests); chassis state is
`$XDG_DATA_HOME/RRsZ70_ROS2/calibration/chassis/online.json` (XDG fallback
`~/.local/share`). Persisted estimates remain history until reconciled.

Standalone VO/mosaic initialize camera_x from installed `odometry.yaml` (0.16 m);
normal bringup passes `mount_calibration.yaml`'s front offset (0.275 m), and a ready
passive fit can replace it. These are distinct model/fit inputs, not surveyed truth.
Chassis `output_file` overrides its durable path. `initial_state_file` explicitly
imports a version-1 archive only if output is absent, atomically, byte-preserving
and no-clobber; no automatic archive search occurs. CalibrationState uses known
flags for unknown values and marks calibration ESTIMATED, not OBSERVED. Persisted
update timestamps and content-hash identity survive no-op republication; fit sigma
is not calibrated physical covariance. Compressed camera images, CameraInfo and
metadata are RELIABLE + TRANSIENT_LOCAL; raw images are volatile. Generated source
IDL under `src/z70_interfaces` is the field/constant authority.

For a fresh non-symlink resource check, use separate prefixes:

```sh
pixi run colcon --log-base log-clean build --base-paths src --build-base build-clean --install-base install-clean
pixi run bash -c 'source install-clean/setup.bash && ros2 launch z70_bringup bringup.launch.py --show-args'
```

This shows arguments without launching nodes. Always include `--base-paths src`
in direct colcon builds. To test ament Python packages directly, first source
`install/setup.bash` inside `pixi run bash -c`.

## Optional depth

Depth is default-enabled but degrades to unavailable status if its separate
interpreter/weights are missing. One persistent worker consumes existing ROS JPEGs;
it does not open another camera. Torch/Transformers remain outside Pixi.
Provision the environment and pinned model separately (network download):

```sh
python3 -m venv ~/.venvs/roboclaw-depth
~/.venvs/roboclaw-depth/bin/pip install torch transformers opencv-python pillow numpy huggingface_hub
~/.venvs/roboclaw-depth/bin/hf download depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf --revision 8078d68a9c75a972131914f6afd0c1723be0da7f
```

The worker defaults to MPS and loads the standard external Hugging Face cache with
`local_files_only=True`. No startup model download or committed weights. See
[mapping/depth](../src/z70_mapping/README.md) for floor correction and output limits.
