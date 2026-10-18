# Brief: agentic pickup of a dodecahedron (GPT session; Michael present)

Michael placed several small dodecahedrons on the floor in front of the robot. Pick one up with **our own pipeline**:
find it, drive to it, place the gripper over it using geometry and then the pincher camera, grasp it, and verify it is held.
You run the whole thing; Michael has already authorized the session. Follow the operating rules in the `robot_rpc` tool
description: keep moving, resolve ambiguous outcomes by looking, and treat measurement failures as data, not stops.

## What you have (read these first, quickly)
- Front camera floor geometry: `calibration/front-camera/` (README, `floor_geometry.py`, `front_camera.json`). It maps a pixel to
  a metric floor point (camera at 68 mm, ~2.7° down, f≈823 px). Near edge of view ≈ 8.5 cm ahead of the camera.
- Chassis: `app_rc_start` → wait ≥5 s → `app_rc_move {velocity, omega, duration, seqnum}` (seqnum increments) → `app_rc_stop` →
  `app_rc_end`. Live calibration: MCP `get_state` → `chassisCalibration` (yaw gain ≈0.72 left / 0.64 right; camera_x ≈0.275 m;
  forward gain unknown, so measure it). Every move you make improves it. ROS `/odom` (floor VO, ~3 Hz) gives closed-loop feedback.
  If no bringup is running (`ros2 node list` in the root Pixi env shows no z70_driver), start one within the authorized live scope: `~/.pixi/bin/pixi run bringup` from the repository root.
- Arm: `docs/z70-arm-model.md`, `src/z70_description/z70_description/arm_model.py`, `calibration/arm-validation/{RESULTS.md,M3_M4_RESULTS.md,LIMITED_POSE_RESULTS.md}`
  have exact command formats and measured step sizes: `app_arm_out_compartment`, `app_arm_direction_move` (hand steps ≈4 cm),
  `app_arm_move` (joint steps ≈10°), `app_open_gripper`, `app_grip`, `app_arm_in_compartment`. MCP `get_arm_estimate` tracks the pose.
  The arm origin relative to the front camera is NOT measured (the URDF guesses it). An ArUco DICT_4X4_1000 ID0 marker (15 mm black square)
  is fixed on the arm/gripper, so you can measure the offset if the marker enters the front view.
- Pincher camera: pickup mode `app_pick_up_enter_exit {flag:2}` → `app_arm_out_compartment` → wait until `get_status`
  `common_status` has bit 8 (0x100) set (~15 s) → MCP `select_camera {camera:"gripper"}` (confirms in ~1 s; first frame ~10 s, then
  ~1.5 s each; 1600×896). `select_camera {camera:"front"}` to go back. Frames say which camera they came from.
- Vendor autonomous pickup exists (`app_pick_up {flag:2}` start, `{flag:3}` continue). Use it ONLY as a labelled fallback after
  our own pipeline has failed 3 grasp attempts, and report it separately. It doesn't count as success of this assignment.

## Process (adapt as you learn; log every step)
1. **Preflight (no prompts):** status, arm estimate, `get_state` calibration, ROS /odom alive, front camera selected.
2. **Find:** front frame → vision lists dodecahedrons (colour, pixel of the floor-contact point, size) → floor_geometry gives metric
   (x, y) relative to the robot. Pick the best candidate (nearest, isolated, reachable). Estimate its size from the projection.
3. **Approach (closed loop):** decide the standoff where the object should end up for the arm (from the arm model plus what you measure).
   Turn to bearing ≈0, then drive in small forward pulses, re-measuring from a fresh front frame after each move. Target bearing
   ≤3° and range error ≤2 cm. Prefer re-observing over trusting gains; note predicted vs observed for every move.
4. **Arm + pincher camera:** pickup mode, arm out, wait for ready, gripper camera. Open the gripper. Servo: move the hand until the object
   is centred in the gripper image (use small chassis turns for lateral error), descend in steps while checking the gripper image,
   then `app_grip`. Lift one step.
5. **Verify:** gripper frame (vision: object between fingers?), then front frame (object gone from its floor spot?), hold 10 s, then check
   the gripper frame again.
6. **Finish:** lower, `app_open_gripper` to put it down, stow (`app_arm_in_compartment`), front camera, `app_rc_end`/exit pickup if
   still active, final status.

Up to 3 grasp attempts; re-approach as often as useful. On a miss, look at why (gripper image, front image) and correct.

## Success criteria
- **SUCCESS:** the gripper frame shows the dodecahedron held between the fingers, AND after lifting, the front frame shows it gone
  from its pre-grasp floor position (±3 cm), AND the gripper frame still shows it after a 10 s hold.
- **Partial, reported as levels:** L1 localized (metric position from geometry); L2 approached within tolerance (bearing ≤3°,
  range ≤2 cm); L3 object centred under the gripper in the pincher camera; L4 gripper closed on the object but not lifted or held.

## Deliverables
`calibration/pickup/<UTC>/`: `log.jsonl` (every RPC/frame/decision with timestamps), key frames (git-ignored), any helper scripts,
and `RESULTS.md`: outcome level, timeline, predicted vs observed positions, measured arm-origin/standoff, forward gain, what to
improve. Commit RESULTS.md, scripts and log (no images) when done. End with the robot stowed and idle.

## Rules
Robot actions only through RoboClaw MCP tools (no raw HTTP/shell to the robot; never read `.local/`). Images via vision_analyze.
Stop motion only for the stop conditions in the operating rules. Don't ask Michael for confirmations mid-run; tell him what you are doing.
