# z70_manipulation

Deterministic HSV perception, nominal 10-degree arm planning, and closed-loop
manipulation actions through RoboClaw. **One supervised yellow-object pickup and >10 s hold
was verified on 2026-10-15**, after one failed attempt. Evidence and limitations:
`calibration/pickup/20261015T053609.715453Z-ros/RESULTS.md`.

## Run

From the repository root:

```sh
~/.pixi/bin/pixi run test
~/.pixi/bin/pixi run bringup manipulation:=true mapping:=false foxglove_port:=8765
# Separate terminal; allow approximately 60 s for bringup/discovery:
~/.pixi/bin/pixi run pick yellow --dry-run
~/.pixi/bin/pixi run pick yellow
~/.pixi/bin/pixi run stack yellow blue purple --dry-run
~/.pixi/bin/pixi run stack yellow blue purple
```

Ordinary bringup commands no motion, but reads live camera/state, refreshes maps
and may write chassis calibration.
`manipulation:=true` starts the manipulation action servers (`z70_pick_server`); only a live goal
commands motion. One active manipulation goal is accepted at a time. Cancellation and
`/z70/safety_stop` stop the active task.

## Curated ROS 2 Actions

When `manipulation:=true` is enabled, `z70_pick_server` exposes both the composite `/z70/pick` action and its composable building blocks (all wired through `z70_web_gateway` and Hermes MCP `start_task`):

- **`/z70/arm/deploy` (`z70_interfaces/action/DeployArm`, task kind `deploy_arm`)**
  - Goal: `{settle: bool}`
  - Enters remote pickup mode (`app_pick_up_enter_exit {flag: 2}`), unstows the arm (`app_arm_out_compartment`), waits for readiness (`WORKING_POS`), and optionally waits 15 s for the vendor obstruction-check spin to settle.
- **`/z70/align_target` (`z70_interfaces/action/AlignToTarget`, task kind `align_target`)**
  - Goal: `{color: string}`
  - Acquires and confirms the target on two fresh front frames and visually servos the chassis (`BasePulse` + `/odom` + `/z70/chassis_calibration` gains) until bearing (`<= 2 deg`) and standoff range (`<= 15 mm`) converge.
- **`/z70/arm/move` (`z70_interfaces/action/MoveArm`, task kind `move_arm`)**
  - Goal: `{m3_deg: float, m4_deg: float, m5_deg: float}`
  - Plans a legal sequence of 10-degree steps from `/z70/arm_estimate` using the recovered vendor kinematic envelope (`arm_model.check` + `planner.joint_plan`) and steps via RPC receipt without requiring per-step front-camera marker checks.
- **`/z70/arm/grasp_and_lift` (`z70_interfaces/action/GraspAndLift`, task kind `grasp_and_lift`)**
  - Goal: `{lift_steps: uint8, verify_color: string}`
  - Closes jaws (`app_grip`), executes `lift_steps` upward hand steps (`app_arm_direction_move {direction: 1}`), and optionally verifies object elevation and floor disappearance when `verify_color` is non-empty.
- **`/z70/arm/release_and_stow` (`z70_interfaces/action/ReleaseAndStow`, task kind `release_and_stow`)**
  - Goal: `{put_down: bool}`
  - Optionally runs `app_put_down {flag: 0}` with 20 s program settling, then stows (`app_arm_in_compartment`) and reconciles `IN_ROBOT`.
- **`/z70/pick` (`z70_interfaces/action/Pick`, task kind `pick`)**
  - Goal: `{color: string, dry_run: bool}`
  - Composes `align_front` -> `deploy` -> `post_deploy_align` -> `move_arm` -> optional `front_servo` -> `grasp_and_lift` -> `release_and_stow`.
- **`/z70/stack` (`z70_interfaces/action/Stack`, task kind `stack`)**
  - Goal: `{base_color: string, middle_color: string, top_color: string, dry_run: bool}`
  - Plans a 3-tier dodecahedron stack. Nominal 66 mm objects imply equator heights of 33/99/165 mm; those are planning targets, not measured achieved heights.
  - Normal in-place release uses Manual Arm Control (`app_pick_up_enter_exit {flag: 1}`, observed grab mode 1) followed by `app_put_down {flag: 0}`. In auto mode 2 the same put-down RPC can lower the arm and rotate the chassis; `app_open_gripper` is not the normal manual-release button.
  - Elevated release alone does not validate stacking. Placement must align the measured held object, lower to the support without a free drop, and verify detachment and stable support before counting a completed tier. The retained Gemini trials opened successfully but did not produce a stable stack.

## Safety and interfaces

HSV does not identify people/pets. Supervision or an external detector must
publish `/z70/safety_stop`. Robot errors (`error_code != 0`), unexpected clean/dock,
and external stops remain hazards. Battery threshold is 15%.

Perception publishes `/z70/objects` (MarkerArray) and `/z70/objects/json`
(String), in `base_footprint`, alongside bridge front/gripper image and CameraInfo
topics.
