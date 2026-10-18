> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md), [arm model](../z70-arm-model.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

The TypeScript-estimator/separate-display-TF/RViz ownership plan is superseded by ROS command/status estimation, one canonical joint/TF stream and current CAD-backed model. The old no-full-measurement premise is not a claim that structural CAD is unavailable.

## Historical record

# ROS 2 + arm pose estimation plan

Historical 2026-10-13 plan, superseded as current policy by
[the approved ROS-first contracts](../migration/README.md). ROS now has an opt-in
HTTP-backed pickup executor, mapping and perception; the observation-only and
geometry-validation assumptions below are historical, not current guarantees.

Decided 2026-10-13: ROS 2 **Jazzy** (RoboStack via pixi, native macOS arm64) + **Foxglove**, RViz2 as a bonus.
RoboClaw owns every robot command and therefore the dead-reckoned arm pose. ROS reads it.

```
Hermes ──MCP──┐
              ├──> RoboClaw (Docker): commands + ArmPoseEstimator (Phase 2)
ROS 2 nodes ──┘     GET /api/arm/estimate
z70_bridge ── /joint_states, /z70/status, /camera/front/{image_raw,compressed,camera_info}
robot_state_publisher (z70.urdf.xacro) ── /tf      foxglove_bridge ws://127.0.0.1:8765
```

## Phase 0 — done
- `ros2/`: pixi workspace (Jazzy, rviz2, foxglove_bridge, xacro, robot_state_publisher, colcon).
- `z70_description`: URDF from the app's arm model (`docs/z70-arm-model.md`) plus the front-camera calibration.
  Its wrist position, through ROS TF, matches the app's forward kinematics to 0.00 mm (WORKING_POS, TRANSFER_POS, a custom pose).
- `z70_bridge`: read-only RoboClaw → ROS (state 2 Hz, camera 1 Hz, joints 10 Hz). Smoke-tested offline:
  all topics up, `/joint_states` at 10 Hz, Foxglove listening.
- Not yet run against the live robot. That needs the camera and state reads, so ask first.

## Phase 1 — validation session (GPT): `docs/validation/arm-chassis-session-for-gpt.md`
Not skipped. The app model gives geometry, poses and limits, so no full measurement campaign is needed. Still to
measure: whether the manual-mode commands really move 10° / 0.04 m, deployed-pose repeatability, chassis pulse
calibration, and the camera's position on the body. Decision rule in the protocol: adopt the app constants if
ratios fall within 0.85–1.15.

## Phase 2 — ArmPoseEstimator in RoboClaw (`src/arm-estimator.ts`) — done 2026-10-13
Deployed and wired: RPC dispatch → estimator → `GET /api/arm/estimate` / MCP `get_arm_estimate` → `z70_bridge` → `/joint_states`
(verified with a no-motion anchor: m3 −2.094 rad, m4 2.356 rad). Not done: detecting arm activity made outside RoboClaw.
Phase 1 outcome (GPT, `calibration/arm-validation/`): M5 cumulative 10/20/30° → 6.1/19.3/30.2°; hand/M3/M4/chassis
unmeasured (marker left the front view). Constants kept at vendor values.

- Port `arm_model.py` (FK/IK/state machine/checks) to TypeScript. Step constants come from the app model or `arm_model_measured.yaml`.
- Hook every arm RPC in the dispatch path:
  - `app_arm_out_compartment` → WORKING_POS;
  - `app_arm_in_compartment` → IN_ROBOT (re-anchor);
  - `app_arm_move` / `app_arm_direction_move` / `app_start_program` `arm_joints` / `arm_move` records → accumulate;
  - `app_open_gripper` / `app_grip` → M6.
- `unknown` on: timeout or unclear outcome, autonomous pickup/put-down, cleaning, arm activity in `get_status` we
  didn't command. Only stow re-anchors.
- Persist `/data/arm-pose.json` until stow; restore on restart, flagged `restored`.
- Expose `GET /api/arm/estimate` and an MCP tool `get_arm_estimate`; the bridge already consumes it.
- Tests: accumulation, IK branch continuity, invalidation, persistence. No format pinning.

## Phase 3 — chassis odometry → `/odom`
Pulse table from Phase 1, plus visual odometry from floor features (real metres, from the known floor plane and camera height).

## Phase 4 — mapping in ROS
Depth node (Depth Anything V2 Small, 0.05 s/frame on the Mac GPU, with the floor correction) → depth image / PointCloud2.
Then `depthimage_to_laserscan` + `slam_toolbox`, or RTAB-Map. Check osx-arm64 RoboStack availability first.
