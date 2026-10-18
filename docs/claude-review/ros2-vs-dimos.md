> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md), [mapping/depth guide](../../src/z70_mapping/README.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

The observation-only/no-ROS recommendation and tf2-value verdict were superseded by implemented ROS execution, estimation and acquisition. Keep dated source comparisons/pins, not a mandate to remove current capabilities.

## Historical record

# Feasibility review: replace DimOS with ROS 2, move control into Hermes Agent

Historical source audit, not current architecture policy. The observation-only
recommendation below was superseded by the approved
[ROS ownership migration](../migration/README.md); ROS already includes a
discrete HTTP-backed pickup executor, perception, floor VO and vendor mapping.

Date: 2026-10-13 · Scope: read-only review of this repo (HEAD `efc8cf5`), the pinned DimOS checkout
(`db3d0ca9`, 2026-10-03), upstream python-roborock (HEAD `6a0aab05`, 2026-10-12) and public docs.
No robot, service, go2rtc, Docker or cloud access was used.

**Bottom line:** Don't replace DimOS with ROS 2 now. Remove the DimOS layer from the control path,
let Hermes Agent call the existing RoboClaw MCP server directly, and add a small deterministic safety
layer to RoboClaw (stop, motion caps, deadman). The Z70 publishes no odometry, lidar or IMU stream, so
the main reasons to use ROS 2 (nav2, tf2, SLAM) have no inputs. Keep ROS 2 as an optional,
observation-only sidecar for a later phase, and only build it if you start visual-SLAM work.

---

## 1. What DimOS provides here, and is it pulling its weight?

### What DimOS is
DimOS is a pre-release beta "agentive OS" for Python robotics. It provides Modules/Blueprints, typed
streams, `@skill` → MCP exposure, navigation/mapping (DimSlam, planners), perception, spatial memory and
VLM agents. Its first-class targets are Unitree quadrupeds, drones and manipulators, and the README
lists macOS as a partial (🟧) install target ([DimOS README](https://github.com/dimensionalOS/dimos);
local `source/README.md` lines 1-160, `dimos/robot/{unitree,unitree_webrtc,drone,manipulators}`,
`dimos/navigation/{go2,global_planner,local_planner}`).

### What the integration uses (evidence: `grep "from dimos"` over `integrations/dimos`)
| DimOS API used | Where | Purpose |
|---|---|---|
| `dimos.core.module.Module`, `dimos.core.stream.Out/In` | `module.py`, `live_slam.py` | Container for 4 skills and 2 streams |
| `dimos.agents.annotation.skill` | `module.py` | `z70_observe`, `z70_camera`, `z70_control(tool_name, args_json)`, `z70_supported_tools` |
| `dimos.web.relay_bridge.wt_client.RelayClient` / `protocol` | `headless.py`, controller | WebTransport/QUIC relay between host and container |
| `dimos.msgs.sensor_msgs.Image/CameraInfo` | `live_slam.py` | Message types for the (unqualified) mono VO stream |

**What it doesn't use:** DimOS navigation, planners, DimSlam/EKF ("Stock DimSlam typed LCM/EKF transport
… remain separate future qualification work", `native_slam/README.md`), perception, spatial memory, or
DimOS's own agent/LLM runtime. The owned `Controller` injects a custom `HostedCopilotModel` (OpenCode →
GitHub Copilot) that is **text-only**: "Hosted inference is text-only; media references are not vision
evidence" (`controller.py` SYSTEM prompt). Each of the four skills is a single pass-through to RoboClaw MCP
(`module.py`). `z70_control` is literally "One transparent MCP tool call".

### The current control path, and how many hops it has
Hermes → `hermes_plugin` (`roboclaw_chat`, which **explicitly enables no robot tools**:
"No robot tools or motion are enabled") → ACP subprocess → Docker container running DimOS controller →
TLS/QUIC relay → host inference bridge → OpenCode/Copilot (a second LLM agent) → MCP queue → host bridge →
RoboClaw MCP → RoboClaw HTTP → L01 TCP to the robot.

That makes two LLM agents in series, three IPC transports, a container built `FROM dimos-z70:test` (a local
trial image the README admits is "not an independently reproducible upstream release image"), and about
160 KB of integration source plus about 1.7k lines of tests. The result is receipts that always say
`physical_completion_verified: false`.

### The native SLAM side-project
cuVSLAM on Metal ran only on **synthetic** stereo/mono fixtures. On that fixture Metal was *slower* than CPU
(7.76 ms vs 1.07 ms median). Mono output is "up to scale", has no base transform, and has never seen a real
Z70 frame. The bundled NVIDIA licence restricts platforms, and the repo notes that "personal/noncommercial
use is not a documented exception" (`native_slam/README.md`, `CAPTURE_TEST_RESULTS.md`).

### Verdict: DimOS is not pulling its weight
Hermes can already call the same MCP tools directly. The only end-to-end hardware trial (2026-10-03) did
exactly that ("All robot interactions used configured RoboClaw MCP tools", `docs/hardware-trial-2026-10-03.md`).
DimOS adds latency, a second model that can't see images, and a large surface of gates and receipts, and it
contributes no capability the Z70 can use today. Its genuinely valuable pieces (nav/SLAM) need sensor
streams this robot doesn't publish.

---

## 2. ROS 2 on macOS Apple Silicon in 2026

| Topic | Fact | Source |
|---|---|---|
| Official tier | Jazzy (LTS to May 2029) and Kilted list **macOS amd64 only, Tier 3**. macOS arm64 isn't even in the table. Lyrical Luth (May 2026, newest LTS) keeps macOS at **Tier 3**, and "to use ROS Lyrical on any Tier 3 platform, you must build … from source". | [Jazzy platforms](https://docs.ros.org/en/kilted/Releases/Release-Jazzy-Jalisco.html), [Kilted](https://docs.ros.org/en/rolling/Releases/Release-Kilted-Kaiju.html), [Lyrical platforms](https://docs.ros.org/en/jazzy/Releases/lyrical/supported-platforms.html), [REP 2000](https://www.ros.org/reps/rep-2000.html) |
| RoboStack / pixi | Community conda binaries for Jazzy, Kilted and Lyrical on osx-arm64, with no sudo, Docker or VM. Pixi is the recommended installer. I checked the `robostack-jazzy` osx-arm64 repodata: it contains `ros-jazzy-rmw-zenoh-cpp` 0.2.9, `navigation2` 1.3.9, `foxglove-bridge` 3.6.0, `rosbag2` 0.26.9, `image-transport` 5.1.8 and `slam-toolbox` 2.8.5. This is community-supported, not OSRF Tier 1. | [RoboStack](https://robostack.github.io/index.html), [Getting started](https://robostack.github.io/GettingStarted.html), [pixi for robotics](https://prefix.dev/blog/reproducible-package-management-for-robotics), `conda.anaconda.org/robostack-jazzy/osx-arm64/repodata.json` |
| Docker Linux containers | Ubuntu 24.04 arm64 images are Tier 1 and run fine inside Docker Desktop's VM. However, Docker Desktop's host networking (opt-in, ≥ 4.34) "works on layer 4 … protocols below TCP/UDP are not supported", and "processes inside the container cannot bind to the IP addresses of the host". In practice DDS multicast discovery across the host/LAN boundary is unreliable. There is also no GPU passthrough on macOS (Apple Virtualization VM), so CUDA/Isaac stacks are out. | [Docker host driver – Desktop limitations](https://docs.docker.com/engine/network/drivers/host/), [Docker Desktop networking](https://docs.docker.com/desktop/features/networking/) |
| rmw_zenoh | Binaries exist for Humble, Jazzy and Rolling, and rmw_zenoh_cpp is Tier 1 in Kilted. It uses a router (`rmw_zenohd`) over unicast TCP instead of multicast discovery, which is the standard fix for DDS-in-Docker and DDS-across-NAT. One TCP port publish is enough. RoboStack ships it for osx-arm64, so a native-mac ↔ container ROS graph is feasible. | [rmw_zenoh binaries announcement](https://discourse.openrobotics.org/t/rmw-zenoh-binaries-for-rolling-jazzy-and-humble/41395), [ROS docs: Working with Zenoh](https://docs.ros.org/en/jazzy/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html), [Kilted middleware table](https://docs.ros.org/en/humble/Releases/Release-Kilted-Kaiju.html), [Isaac Sim pixi+zenoh example](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros_other_platforms.html) |

**Realistic choice if ROS 2 is ever adopted:** run RoboStack Jazzy natively on the Mac via pixi with
`RMW_IMPLEMENTATION=rmw_zenoh_cpp`. Avoid Docker DDS. The repo already built and then retired a Jazzy
Docker read-only bridge (`docs/robotics-ros.md`, marked "Historical/superseded"; `docs/thin-api.md`:
"The leased ROS action/read-only bridge is retired").

---

## 3. What the Z70 actually exposes (this decides the ROS question)

From `src/vendor-commands.ts`, `src/robot.ts`, `src/mcp-server.ts` and `docs/thin-api.md`:

- **Transport:** vendor RPC over local L01 TCP or cloud MQTT. Responses are ACK/result only;
  `physicalCompletion: "unverified"` is honest, because the vendor doesn't report physical completion.
- **Chassis:** `app_rc_start` / `app_rc_move {velocity, omega, duration, seqnum}` / `app_rc_stop` / `app_rc_end`,
  `app_charge` (dock) and `app_stop`. Values are vendor units, not SI. `duration` makes each move pulse bounded
  on the robot side, which is useful for safety.
- **Arm:** discrete, named steps only, e.g. `app_arm_move {m:[…]}` named steps, `app_arm_direction_move`
  up/down/forward/back, `app_arm_out/in_compartment`, `app_grip`/`app_open_gripper`, `app_pick_up`/`app_put_down`
  flags, `app_stop_grasp`. Joint feedback comes from `app_get_arm_joints_data`. There are no joint-position or
  trajectory commands.
- **State:** `get_status` (state codes, battery, errors), arm joints, room mapping, multi-maps.
- **Map/pose:** `get_map_v1` (encrypted L01 map blob). A `robot_data` position exists in map diffs, but the
  repo's own research says the capture clock and map frame are unproven
  (`src/native-map-diff-pose-evidence.ts`: "Never translate this result into … pose or a navigation target").
  Pose arrives at map-refresh rate, not as an odometry stream.
- **Camera:** WebRTC via the go2rtc `roborock://` source (front 1600×1200). go2rtc's documented working models
  are S6 MaxV, S7 MaxV and Qrevo MaxV, so the Z70 works but isn't listed upstream
  ([go2rtc Roborock](https://go2rtc.org/internal/roborock/)). Lens identity and exposure time are unknown.
- **Not exposed publicly:** raw lidar scans, wheel odometry, IMU, cliff/bumper streams and depth.

**python-roborock (option B check):** upstream HEAD has no `a179` / Saros Z70 constant (latest Saros entries
are `a144` 10R and `a147` 10). It has no arm commands (`app_pick_up`, `app_arm_*`, `app_grip` are absent),
although it has `APP_RC_MOVE/START`, L01 local protocol and V1 map parsing via `vacuum-map-parser-roborock`.
There is an open issue saying Saros 20 (L01) `GET_MAP_V1` frames aren't decodable
([python-roborock](https://github.com/Python-roborock/python-roborock),
[issues](https://github.com/Python-roborock/python-roborock/issues),
[devices API](https://python-roborock.github.io/python-roborock/roborock/devices.html)). Home Assistant's core
integration covers the Saros line for vacuum functions only
([HA Z70 review](https://smarthomeassistant.co.uk/articles/home-automation/roborock-saros-z70-review-uk)).
RoboClaw's TypeScript stack already implements L01, the A179 arm wire mappings and the go2rtc camera protocol.

---

## 4. Architecture options

### A) ROS 2 nodes wrapping RoboClaw HTTP/MCP
Nodes publish `/z70/status`, `/z70/arm_joints`, `/camera/image_raw` + `camera_info` and a map
`OccupancyGrid`, and expose `/z70/cmd` services/actions mapping to RPC.
- **Pros:** you get rosbag2, Foxglove/RViz, image_transport and a calibrated `camera_info` cheaply. It keeps
  RoboClaw as the single owner of the vendor session and is the lowest-risk way into ROS.
- **Cons:** it duplicates a thin API that already exists. On macOS it means a Tier-3/RoboStack or Docker/Zenoh
  toolchain. tf2 and nav2 still have no odometry or scan to consume. This was already built once and retired.
- **Use only for observation and recording**, and only once there's a reason (visual SLAM, datasets).

### B) Rewrite RoboClaw core as ROS 2 nodes using python-roborock
- **Pros:** one Python/ROS process.
- **Cons:** python-roborock lacks Z70/a179 and every arm command, and has known L01 map-decode gaps. You'd
  rewrite working TypeScript (L01 framing, map security, WebRTC/go2rtc camera, 311 passing tests) and inherit
  ROS-on-mac costs. There is no new capability.
- **Not recommended.**

### C) No ROS: thin RoboClaw service + Hermes Agent as planner over MCP, with skills for procedures
- **Pros:** shortest path. The hardware trial already proved Hermes → RoboClaw MCP works. Hermes provides MCP
  tool include/exclude filtering, timeouts and `supports_parallel_tool_calls: false`
  ([MCP config reference](https://hermes-agent.nousresearch.com/docs/reference/mcp-config-reference)), skills as
  procedural memory ([Skills](https://hermes-agent.nousresearch.com/docs/user-guide/features/skills)), cron with
  script pre-checks that wake the model only when needed
  ([hermes-cron](https://hermes-agent.nousresearch.com/docs/plugins/hermes-cron)), subagents and a vision model
  that can actually see frames. It removes about 160 KB of DimOS glue and the second LLM.
- **Cons:** LLM turn latency is seconds, so control is discrete step-and-look, never closed-loop servoing.
  Safety must not depend on the model; that's handled in section 5. No standard robotics visualization unless
  added (the Foxglove SDK works without ROS).
- **Recommended.**

### What ROS 2 would genuinely buy vs cost, for this robot
| Capability | Value for Z70 today | Reason |
|---|---|---|
| tf2 | ~none | No measured odometry or pose stream. The map pose frame is unproven. |
| nav2 | none now | Needs odom plus a laser/costmap at ~10 Hz or more. The Z70 navigates internally and exposes neither. You'd first need your own VO/SLAM from a mono camera with unknown exposure and lens. |
| image_transport + camera_info | moderate | Helps calibration and later VO. It can be done equally well in plain Python/OpenCV. |
| rosbag2 | moderate | Good recording format. JSONL plus JPEG with timestamps covers the current need. |
| Foxglove / RViz | moderate | Nice visualisation. Foxglove also accepts non-ROS WebSocket/MCAP. |
| Ecosystem (slam_toolbox, MoveIt) | none | Needs lidar. The arm has only named steps, with no joint control. |
| **Cost** | high | Tier-3 macOS (RoboStack community) or Docker without multicast/GPU, plus Zenoh router setup, a second build system and a second process model. |

---

## 5. What moves into Hermes, and what must stay deterministic

### Move into Hermes Agent
- **Planning and sequencing:** undock → look → move pulse → look → dock; arm deploy → step → grip → stow.
  Write these as **skills** (procedures with pre/postconditions and expected state codes), not code gates.
- **Perception judgements:** `get_camera_frame` already returns MCP image content, or a private file via
  `ROBOCLAW_MCP_IMAGE_HANDOFF=private-file` for `vision_analyze`. Hermes judges "clear path?", "object in
  gripper?" and "did the scene change?".
- **Verification:** use a fresh `refresh_status` plus an image diff after each action, and record a JSONL
  episode log. Optionally use a subagent as an independent verifier of the outcome.
- **Scheduling:** cron jobs (e.g. nightly status/battery check, patrol snapshots) with a script pre-check so
  the LLM wakes only on anomalies.
- **Tool surface:** connect Hermes `mcp_servers.roboclaw` (stdio `mcp-main`) with `tools.include` and serial
  execution. Retire the `roboclaw-acp` plugin.

### Keep in a deterministic, low-latency service (RoboClaw)
- **STOP:** an always-allowed, never-queued path that issues `app_rc_stop` + `app_stop_grasp` (+ `app_rc_end`).
  It should be reachable from MCP, HTTP and a UI button, independent of any LLM.
- **Motion invariants (small and fixed, not workflows):** clamp `velocity`, `omega` and `duration` (e.g. ≤ 1 s
  per pulse) and serialise commands per device. Add a deadman: if no new move arrives within N seconds of an
  RC session, send `app_rc_stop`/`app_rc_end`. The current thin API deliberately has none of this
  (`thin-api.md`: "Methods are not restricted…").
- **Streaming:** keep the go2rtc/WebRTC session, frame grabbing, timestamps and any future VO/recording at
  frame rate in code, not the LLM.
- **Vendor session:** L01 connection, reconnects, map decryption and credential custody.

---

## 6. Recommendation and phased plan

**Adopt option C. Freeze DimOS and archive `integrations/dimos` as historical, and don't start ROS 2 now.**

**Phase 1 — end-to-end validated loop (target: 1–3 days, supervised)**
1. Configure Hermes MCP directly to RoboClaw (`get_state`, `refresh_status`, `get_camera_frame`, `robot_rpc`)
   with `supports_parallel_tool_calls: false`. Disable the `roboclaw-acp` plugin.
2. Write one skill, `z70-step-and-look`: preflight status → frame → `vision_analyze` → `app_rc_start` → one
   `app_rc_move` pulse (small velocity, ≤ 500 ms) → `app_rc_stop` → `app_rc_end` → fresh status + frame →
   compare → `app_charge` → poll until the charging state code returns (8, observed in the 10-03 trial).
3. **Acceptance:** one JSONL episode with request IDs, vendor responses, before/after frames, a status-state
   transition away from charging and back to charging, and Michael's visual confirmation. That is the first
   genuinely validated loop.

**Phase 2 — safety floor (about 1–2 days):** add the STOP endpoint/tool, clamp move parameters, the RC
deadman, per-device serialisation and named MCP tools (`stop`, `move_pulse`, `dock`, `arm_step`) so the model
doesn't hand-craft raw RPCs for common actions. Keep `robot_rpc` for exploration.

**Phase 3 — arm procedures:** skills for deploy/step/grip/stow with joint-feedback checks
(`app_get_arm_joints_data`) and an arm-camera frame before and after. Run supervised trials and log episodes.

**Phase 4 — autonomy and recording:** cron patrols and status watch, a dataset recorder (JPEG + status JSONL,
optionally MCAP) and Foxglove via its SDK if you want visualisation.

**Phase 5 (optional, only for visual SLAM/navigation research):** RoboStack Jazzy natively via pixi with
rmw_zenoh. Build an observation-only A-style bridge that publishes image/camera_info/status/map, and try
monocular VO or slam on recorded bags. Promote it to control only if it produces a metric pose good enough
for nav2.

---

## Sources
- DimOS: https://github.com/dimensionalOS/dimos (pinned local checkout `db3d0ca9`)
- ROS 2 platforms: https://docs.ros.org/en/kilted/Releases/Release-Jazzy-Jalisco.html · https://docs.ros.org/en/rolling/Releases/Release-Kilted-Kaiju.html · https://docs.ros.org/en/jazzy/Releases/lyrical/supported-platforms.html · https://www.ros.org/reps/rep-2000.html
- RoboStack/pixi: https://robostack.github.io/index.html · https://robostack.github.io/GettingStarted.html · https://prefix.dev/blog/reproducible-package-management-for-robotics · https://github.com/RoboStack/ros-jazzy
- Zenoh: https://discourse.openrobotics.org/t/rmw-zenoh-binaries-for-rolling-jazzy-and-humble/41395 · https://docs.ros.org/en/jazzy/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html · https://index.ros.org/p/rmw_zenoh_cpp/
- Docker Desktop networking: https://docs.docker.com/engine/network/drivers/host/ · https://docs.docker.com/desktop/features/networking/
- go2rtc Roborock: https://go2rtc.org/internal/roborock/
- python-roborock: https://github.com/Python-roborock/python-roborock · https://github.com/Python-roborock/python-roborock/issues · https://python-roborock.github.io/python-roborock/roborock/devices.html
- Hermes Agent: https://hermes-agent.nousresearch.com/docs/ · https://hermes-agent.nousresearch.com/docs/reference/mcp-config-reference · https://hermes-agent.nousresearch.com/docs/user-guide/features/skills · https://hermes-agent.nousresearch.com/docs/plugins/hermes-cron
- Repo evidence: `integrations/dimos/{README.md,src/roboclaw_dimos/module.py,controller.py,live_slam.py,hermes_plugin/__init__.py,Dockerfile,native_slam/README.md}`, `src/{mcp-server.ts,vendor-commands.ts,robot.ts,native-map-diff-pose-evidence.ts}`, `docs/{thin-api.md,robotics-ros.md,hardware-trial-2026-10-03.md}`
