# Native dimOS first: living-room mapping → Z70 sock pickup

Research date: 2026-10-06. **No robot motion, map creation, camera session, arm, dock or stop RPC was issued. No deployment or implementation was changed. No claim is made that the living room is mapped.** The separate saved-map retrieval track must supply its own artifact and evidence.

## Revisions and evidence boundary

- Deployed-source checkout inspected: `/Users/mpentax/.openclaw/workspace/dimos-v1/source`, `db3d0ca9f31d726bfea3a78e8a57662292372d50`, commit date `2026-10-03T17:51:09-07:00` (`fix(sim): reject orphaned MuJoCo shared-memory buffers (#4337)`). Deployment pristine/parity is the provided baseline; this research did not inspect a running container to independently establish parity. Final checkout status showed existing/concurrent modifications in `web/relay/session.ts` and `web/sdk/src/session.ts`; this task did not touch them. Do not describe the entire host checkout as pristine merely from its commit ID.
- Upstream main resolved with `git ls-remote`: `4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0`, committer date `2026-10-06T05:34:13Z`, `perf: overlap model loads and heavy imports with the blueprint deploy (#4285)`. Full source archive inspected separately in `/tmp`; not substituted into the deployed checkout.
- Connector source inspected: `/Users/mpentax/Code/RoboRockZ70-Connector/src`. Its existing untracked research and robotics work was preserved. Source support is not proof that a hardware command has succeeded.
- Native `dimos/` comparison: 39 changed files, 1,279 insertions / 309 deletions. Relevant additions/changes include Livox/PointLIO/virtual MID360, Go2 3D blueprint and experimental perceive-loop skill. Core `OpenAIVlModel` and `ReplanningAStarPlanner` files compare identical between the two revisions. **The newer relocalization class structure below already exists at the pinned revision**; prose docs naming only `RelocalizationModule` conceal the actual composed subclass. Do not upgrade merely to gain a module already present.

Upstream sources below are immutable links at the inspected commit. Documentation starting point: [navigation overview](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/docs/capabilities/navigation/index.md). Documentation is useful architecture guidance, but source signatures take precedence.

## Decision

### Owner correction — 2026-10-06 16:40

The primary experiment is **independent native DimOS floor mapping, followed by comparison against the saved Roborock map**. Do not import the vendor map as the mapper's input, localization prior, or ground truth for this experiment. Keep it as a separate reference artifact. The raster-import branch below is an optional alternative, not the selected first milestone.

First establish an authentic sensor/pose pipeline supported by native DimOS (LiDAR or calibrated RGB-D/stereo); the current JPEG/RPC interface alone does not meet the mapping requirements. Then record a separately authorized living-room survey, construct/save the DimOS map with native modules, and compare the independent outputs offline. Estimate registration from measured common landmarks, report scale/orientation/translation uncertainty, observed overlap, walls/doorways and missing geometry, and distinguish dynamic furniture changes from mapping errors. Neither map is automatically ground truth. Comparison rendering may project the independently produced DimOS map to a floor view; it must not feed vendor geometry back into construction. No physical mapping run is authorized or performed by this research update.


Reuse native mapping, navigation, perception, recording and manipulation components. **Do not build bespoke SLAM, A*, object detector or grasp planner.** The immediate useful branch is existing 2D-map inspection plus stationary image perception, not a Go2 blueprint aimed at a vacuum. Only hardware/type/frame/unit bridges and a task-level composition are candidates for small custom work.

There are two independent missing-data bottlenecks:

1. Navigation: an occupancy prior does not provide fresh metric robot localization, obstacle sensing or a calibrated velocity interface.
2. Pickup: recognizing a sock does not provide its metric grasp pose, reachable/collision-free arm trajectory or verified gripper contact.

Z70 has sensors internally; the inspected connector does **not** expose their LiDAR/depth/encoder/IMU streams. Internal vendor capability cannot be assumed available to dimOS.

## Actual native pipeline and typed ports

These are dimOS ports, not assertions of ROS deployment or universal literal wire topic names. Blueprints autoconnect/remap ports; inspect their transport binding when integrating.

| Native component | Inputs → outputs / requirements | Z70 applicability |
|---|---|---|
| `VoxelGridMapper` | `lidar: In[PointCloud2]` → `global_map: Out[PointCloud2]`; clouds **already in world frame**; 5 cm column-carving occupancy, CPU fallback | Reusable only with actual posed 3D clouds. Not a localization estimator; an extruded vacuum raster is not observed 3D geometry |
| `CostMapper` | `global_map`, `merged_map`: `PointCloud2` → `global_costmap: OccupancyGrid`; height-cost defaults tuned to Go2 | Reuse for real clouds, configure vacuum clearance/height; do not use Go2 climb/underpass defaults |
| `ReplanningAStarPlanner` | `odom: PoseStamped`, `odometry: Odometry`, `global_costmap: OccupancyGrid`, goals `PoseStamped`/`PointStamped`, `stop_movement: Bool` → `nav_cmd_vel: Twist`, `path: Path`, costmap, goal status | Direct 2D `OccupancyGrid` input allows a raster-prior branch without inventing 3D mapping. Still needs correct frames, current pose/velocity and suitable embodiment settings |
| `MovementManager` | `nav_cmd_vel`, `tele_cmd_vel`: `Twist` → `cmd_vel: Twist`; goal/waypoint and stop ports | Native routing available, not a proof of calibrated robot control or physical cessation |
| `Go2Relocalization` | subclass of `LidarWindowRelocalization`; `lidar: PointCloud2`, `global_map: PointCloud2`; premap file → TF, `loaded_map`, `merged_map` | Actual Go2 relocalization blueprint component. FPFH/RANSAC coarse alignment + ICP refinement of live cloud window; requires overlapping metric 3D data |
| `DimSlam` | image/depth `Image`, `CameraInfo`, optional configured IMU/`odom_sources: Odometry` → `odometry: Odometry`, `depth_cloud: PointCloud2` | Native camera odometry branch exists; practical metric mapping requires calibrated stereo/RGB-D and rigid extrinsics, not an uncalibrated monocular JPEG |
| `RayTracingVoxelMap`, `MLSPlannerNative`, `LocalPlanner`, `BasicPathFollower` | ray-tracing global/local clouds, 3D surface planner, local planning/path-following and TF | Existing native alternative for equipped hardware; not a reason to fit an external MID360 assumption to Z70 |

Sources: [voxel mapper](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/mapping/voxels/module.py), [cost mapper](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/mapping/costmapper.py), [A*](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/navigation/go2/replanning_a_star/module.py), [Go2 composition](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/robot/unitree/go2/blueprints/smart/unitree_go2.py), [LiDAR relocalization](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/mapping/relocalization/lidar/module.py), [Go2 merge](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/mapping/relocalization/go2/module.py), [camera navigation guide](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/docs/capabilities/navigation/visual_nav_setup.md), [3D blueprint](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/robot/unitree/go2/blueprints/navigation/unitree_go2_nav_3d.py).

### Premap formats and live mapping

`unitree-go2` is live column carving; `unitree-go2-pgo` substitutes `PGOVoxelMapper` for loop closure. `unitree-go2-memory` records `lidar`, `odom: PoseStamped`, `color_image`, `camera_info` and TF. The Go2 source attaches the last odom pose to LiDAR observations for offline mapping, with a source TODO about already-world-frame clouds; do not blindly copy that convention to another sensor.

Native record/replay/export chain:

- SQLite memory `.db`: typed time-stamped streams/poses/TF, not a vendor raster archive.
- `dimos map global <recording> --export --no-gui`: offline PGO and voxelization; exports LCM-encoded `PointCloud2` as `<name>.pc2.lcm`.
- `<name>.rrd`: Rerun visual QA, not planner map interchange.
- Relocalization `map_file` accepts a stem/path with `.pc2.lcm` resolution. Inspect configured quality thresholds/window limits at the pinned source; do not equate stale prose constants to the refactored implementation.
- Habitat `habitat-nav` proves an existing no-robot simulation path, not compatibility with the living room's raster or Z70 controller.

Sources: [premap docs](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/docs/capabilities/navigation/relocalization.md), [map CLI](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/mapping/cli/map.py), [Habitat](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/docs/capabilities/navigation/habitat.md).

## Connector compatibility matrix

“Demonstrated” here means implemented source contract or explicit source evidence, **not a new physical demonstration**. “Proposed” requires separate validation; “unavailable” means absent/unproved through the inspected interface.

| Capability | Evidence/status | Consequence |
|---|---|---|
| Saved V1 raster parsing | Demonstrated source: `src/v1-map-summary.ts`, digest/block checks, image origin/dimensions, 0.05 m/pixel, walls/floor/room segments | Inspect colleague's actual map artifact; preserve map identity and coordinate provenance |
| Raster pose fields | Demonstrated parser: block 8 `vacuumPosition {x,y,angle?}`, block 1 charger; raw app indexing `floor(position/50 - image origin)` | `positionIsIndependentFreshTelemetry: false`; orientation/frame/freshness remain unverified, not odometry |
| Native map diff pose | `src/native-map-diff-pose-evidence.ts`: structural `robot_data.robot.x/y`, no established pose clock/sequence/V1 frame | Unavailable as trusted localization; arrival/nonce/map ID is not capture time |
| 2D raster → native grid | Proposed minimal representation adapter | Validate raw row-major versus display flip, axes, origin, metric conversion, unknown/wall semantics; don't infer angle units or synthesize covariance |
| Live LiDAR/depth/IMU/encoders | Unavailable in connector inspected | Cannot run real native live voxel mapping/PGO/3D relocalization from current data |
| JPEG/remote camera | Demonstrated source `/api/camera/frame` via go2rtc; per-frame/session identifiers and dimensions in `src/service.ts` | Camera capture may start a vendor session; no capture performed here. A decoded frame can become native `Image` |
| Camera metric calibration | `src/camera-observation.ts` has calibration/projection contracts, not measured installed calibration | Lens identity, capture clock, distortion, camera-to-base (especially moving arm mount) and floor plane need evidence |
| Chassis commands | `src/vendor-commands.ts`: `app_rc_start/move/stop/end`; move `{velocity,omega,duration,seqnum}` | Explicit vendor-valued interface, **not calibrated SI**. Proposed `Twist` adapter only after gain/sign/duration/sequence and feedback validation |
| Arm primitives | `app_arm_move {m:[...]}` named steps; `app_arm_direction_move`; deploy/stow/gripper; pickup/put-down workflow flags | Discrete buttons/workflows, not arbitrary joint targets or time-parametrized trajectory control |
| Arm feedback | `src/arm-feedback.ts`: numeric joint array, `jointUnits: unverified`, common-status mode/action hints, `physicalCompletion: unverified` | Cannot present zeros/raw values as calibrated `JointState`, tool pose, contact or success |
| Robot stops | RC stop/end, `app_stop`, `app_stop_grasp` distinct mappings | Request/ACK is not cessation; chassis, native cleaning task and arm cancellation are separate |

The source's legacy feedback `blockers` strings are descriptive historical content, not a proposal to restore service admission restrictions. This research adds no leases, readiness gates, operator challenges or persistent uncertainty latches. Connector remains a factual authenticated transport; orchestration/evidence belongs above it.

## Native perception, semantic memory and hosted vision

### Detect socks without inventing a detector

- `Detection2DModule`: native `color_image: Image` → `Detection2DArray` and crops; detector is configurable and defaults to `Yolo2DDetector`. Standard detector labels are not evidence of a sock class.
- Native `Yoloe2DDetector` supports prompt mode (`YoloePromptMode.PROMPT`, `set_prompts(text=[...])`); OWLv2 and Omdet implementations also exist. Prototype a sock prompt against fixed images and check precision/occlusion/segmentation; do not claim validated sock recognition from package presence.
- `Detection3DModule` takes `pointcloud: PointCloud2`, camera information and TF. It also contains helpers using **assumed depths** (1.0/1.5 m). Those coordinates are not measured sock/grasp positions.
- `WorldBeliefModule` in `dimos/experimental/world_belief/` is an existing experimental persistent object-identity/scan/recall branch, currently demonstrated through xArm6 RGB-D composition. Recorder inputs include color/depth images, both camera infos and coordinator joint state. It is not a drop-in calibrated Z70 world model.
- `dimos.memory.module` supplies SQLite recorder/replay and visual memory search; image similarity associated with robot observation pose is useful for semantic navigation recall, not proof of an object's exact location.

Sources: [2D detection](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/perception/detection/module2D.py), [3D detection](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/perception/detection/module3D.py), [YOLO-E](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/perception/detection/detectors/yoloe.py), [WorldBelief](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/experimental/world_belief/worldbelief_module.py), [memory](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/memory/module.py).

### Existing hosted image client: yes; this deployment's credential/vision route: unproved

`dimos.models.vl.openai.OpenAIVlModel` already JPEG/base64-encodes a native `Image` and sends a Chat Completions multipart-content message containing `image_url: data:image/jpeg;base64,...` plus text. Config exposes `model_name` (default `gpt-4o-mini`) and `api_key`, or `OPENAI_API_KEY`. It supports query/structured response request and batched images. This exact implementation is identical at pinned and latest revisions.

**This is real native upload support, not a text-only assumption.** However its client is constructed as `OpenAI(api_key=...)`: no provider-specific base URL/auth negotiation is exposed in that class. An OpenClaw hosted subscription/GitHub Copilot session is not automatically an OpenAI API credential, and a model name does not establish image-route support. No credentials were read and no image was uploaded. Validate the intended provider's existing supported client/config with a nonprivate fixture before choosing it; do not recreate an inference server or place tokens in the connector. A non-OpenAI provider may need its existing dimOS/provider client or a small supported configuration bridge, not a new vision implementation.

Connector `src/vision.ts` is a **separate local Ollama-only path** and currently rejects remote/cloud models. That restriction does not establish a project-wide prohibition on hosted native dimOS inference; this research does not propose copying it or changing the service. Use the native hosted client outside that local endpoint when provider capability is established.

Source: [OpenAIVlModel](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/models/vl/openai.py). VLM descriptions/detection boxes remain uncertain perception, never executable commands or physical-success evidence.

## Native manipulation: reuse the planner only if the hardware contract can be met

Native `PickAndPlaceModule` provides `scan_objects`, `get_object`, `pick_object`, `get_grasp_candidates`, `place_at`, with approach/servo, gripper settling and grasp verification. `ManipulationModule`, `ControlCoordinator`, `ManipulatorAdapter`, SDK `Arm`, RoboPlan world/planner and Pink IK are existing building blocks. Default current planning is RoboPlan; the custom-arm guide's Drake-only architectural wording is older than the main manipulation guide/source.

The adapter contract requires SI joint positions/velocities, limits, control modes, position/velocity writes and stop semantics. Motion planning additionally requires an authored/calibrated URDF and collision geometry, joint ordering/signs/zeroes/limits, base/tool transforms and execution feedback. Z70's named button steps and unverified joint array **do not meet that contract**. Never fabricate a generic 6-DOF arm or convert step codes into radians.

Two native-first branches:

1. **Vendor pickup workflow first:** reuse Z70's existing pickup/continue/pause and gripper primitives through a task-level dimOS skill composition, with native perception supplying evidence. This avoids pretending vendor motion is a planned Cartesian grasp. Requires understanding the workflow's actual target selection and separately observing outcomes; source RPC mappings alone do not prove a sock can be selected/picked.
2. **Full dimOS arm planning later:** only if calibrated joint/cartesian control and feedback become available, implement the small `ManipulatorAdapter` plus robot model; retain native RoboPlan/Pink/ControlCoordinator/PickAndPlace rather than inventing IK/planning.

Sources: [pick/place source](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/manipulation/pick_and_place_module.py), [adapter protocol](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/hardware/manipulators/spec.py), [custom arm guide](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/docs/capabilities/manipulation/adding_a_custom_arm.md), [planning guide](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/docs/capabilities/manipulation/index.md).

## Living-room-first pipeline options

### A. Existing vendor 2D map as prior — optional alternative, not selected

Receive colleague's verified saved map; inspect footprint/room segments and confirm which room is living room. Convert only the validated raw occupancy geometry to a native `OccupancyGrid` for offline native A* tests. Preserve original artifact/hash/map identity and explain row/axis/origin conversion. Do **not** extrude it into a `.pc2.lcm` and describe that as a voxel map or relocalization premap.

For live use, independently establish current robot pose in the same metric frame, its timestamp/uncertainty, and obstacle observation. Existing native fiducial detection/TF modules are a possible measured localization aid with calibrated camera and surveyed markers; they do not solve arbitrary vendor frame registration automatically. If proper 2D scans are later exposed and native 3D matching is inappropriate, maintained ROS `slam_toolbox`/Nav2 AMCL is an existing third-party branch to assess, not custom 2D SLAM; neither is needed merely to inspect a saved raster. Dynamic objects/socks are not reliably contained in a saved cleaning map.

### B. Native live LiDAR mapping / saved 3D premap

Requires actual metric timestamped clouds and sensor/world transforms plus odometry, not just Z70's possession of internal LiDAR. Reuse voxel/PGO → native costmap → native planner; record through native memory, export `.pc2.lcm`, QA and replay relocalization. Adding an external supported sensor is an explicit hardware choice, not an assumed capability. Prefer a coherent existing sensor pipeline (e.g. supported LiDAR + PointLIO) over custom SLAM.

### C. Native calibrated RGB-D/stereo pipeline

Use supported RealSense/image module + `DimSlam` + odometry history and existing map/planning components if hardware can be added and calibrated. An RGB JPEG can support recognition immediately, but mono scale, clocks and moving-camera transforms are not magically supplied by VLM inference. Metric floor projection is a restricted option for objects known to lie on a surveyed plane; it needs measured calibration and bounded uncertainty and is not general depth or volumetric mapping.

## Phased roadmap / next prototypes (not executed)

1. **Artifact and stationary perception.** Review saved-map artifact offline; keep private data private. Use already available images, or separately authorize a stationary camera session. Decode to native `Image`; run native YOLO-E prompt and/or native hosted VLM on a nonprivate fixture first. Output sock candidate image evidence only. Compare known sock/no-sock scenes; record model, frame and uncertainty.
2. **Map registration.** Validate raster axes/origin/scale/room identity against measured landmarks. Offline native `OccupancyGrid`/A* prototype only; test unknown space and vacuum-sized clearance. Deliver transform with provenance, not navigation commands.
3. **Localize.** Obtain an independently fresh metric pose stream or measured native fiducial/other supported localization pipeline. For LiDAR/RGB-D branch, replay native mapping/relocalization against authentic data. No generated odometry from commanded velocities and no stale map pose relabelled live.
4. **Approach.** Only after units/signs/current pose/obstacle observation are measured, prototype a thin `Twist` → vendor RC adapter with native planner retained. Validate in mock/replay first. Robot footprint and collision envelope must include deployed arm configuration. Actual supervised motion is a distinct authorization and demonstration.
5. **Grasp.** Choose vendor workflow composition unless the full SI adapter/model becomes supportable. Obtain metric sock location (depth or calibrated floor intersection), reachable target evidence and camera/base/tool registration before Cartesian interpretation. Cloth/deformability may defeat generic rigid-object grasp assumptions.
6. **Verify.** Independently observe pickup/retention and post-lift sock presence, optionally native gripper feedback when meaningful. RPC return, arm bitfield, image classification and physical completion are separate facts. A status hint alone cannot verify grasp.
7. **Stop/cancel is separate.** Plan distinct chassis RC stop/end, native-task cancel and arm cancellation. Test dispatch/ACK separately from measured cessation; no automatic replay after uncertain issuance. No docking/stow sequence is implicitly authorized by this roadmap.

**Next highest-value deliverables:** (a) colleague's map artifact + raw-coordinate validation report, (b) stationary native sock-perception results, (c) sensor/pose/velocity units evidence inventory, (d) explicit determination of whether vendor pickup can target the detected sock. These unblock a native composition; writing a new SLAM/planner/detector today does not.
