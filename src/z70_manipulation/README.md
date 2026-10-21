# z70_manipulation

HSV object perception, CAD-backed coordinated multi-axis arm planning, and ROS 2 manipulation and navigation action servers. `z70_manipulation` communicates through `z70_driver`'s ROS interfaces and shared task scheduler.

## Quickstart

From the repository root:

```sh
pixi run bringup

# Separate terminal (dry-run validates state, observations, and plans without motion):
pixi run pick yellow --dry-run
pixi run stack yellow blue purple --dry-run

# Live execution:
pixi run pick yellow
pixi run stack yellow blue purple
```

Inside a sourced Pixi shell (`pixi run bash && source install/setup.bash`):

```sh
ros2 action list -t
ros2 interface show z70_interfaces/action/MoveArm
ros2 action send_goal /z70/pick z70_interfaces/action/Pick '{color: yellow, dry_run: true}' --feedback
```

---

## Manipulation Actions

All actions are available through ROS 2, the web console, and MCP `start_task` ([`docs/thin-api.md`](../../docs/thin-api.md)):

| ROS Action / Type | Task Kind | Goal Fields | Description |
| :--- | :--- | :--- | :--- |
| `/z70/arm/deploy` (`DeployArm`) | `deploy_arm` | `{settle: bool}` | Enters Manual Arm Mode (`app_pick_up_enter_exit {flag: 1}`), unstows the arm (`app_arm_out_compartment`), waits for `WORKING_POS` readiness, and optionally waits 15 s for the firmware obstruction check to settle. |
| `/z70/align_target` (`AlignToTarget`) | `align_target` | `{color: string}` | Confirms the target on fresh front frames and visually servos the chassis (`BasePulse` + `/odom` + `/z70/chassis_calibration`). |
| `/z70/arm/move` (`MoveArm`) | `move_arm` | `{m3_deg: float, m4_deg: float, m5_deg: float}` | Plans and executes collision-checked, coordinated multi-axis 10° steps (`[m3, m4, m5]`) from `/z70/arm_estimate`. |
| `/z70/arm/grasp_and_lift` (`GraspAndLift`) | `grasp_and_lift` | `{lift_steps: uint8, verify_color: string}` | Closes the jaws (`app_grip`), steps the hand upward (`app_arm_direction_move {direction: 1}`), and verifies object elevation and floor-spot clearance when `verify_color` is set. |
| `/z70/arm/release_and_stow` (`ReleaseAndStow`) | `release_and_stow` | `{put_down: bool}` | Optionally runs `app_put_down {flag: 0}`, stows the arm (`app_arm_in_compartment`), and anchors `IN_ROBOT`. |
| `/z70/pick` (`Pick`) | `pick` | `{color: string, dry_run: bool}` | Full pickup sequence: `align_front` -> `deploy` -> `post_deploy_align` -> single-sweep coordinated `move_arm` descent -> `front_servo` fine alignment -> `grasp_and_lift` -> `release_and_stow`. |
| `/z70/stack` (`Stack`) | `stack` | `{base_color: string, middle_color: string, top_color: string, dry_run: bool}` | Multi-tier dodecahedron stacking with visual carrier-to-support alignment, controlled descent (`planner.descent_step`), in-place manual jaw release, and post-withdrawal support verification. |

### Coordinated Multi-Axis Trajectory Planning (`planner.py` & `executor.py`)

- **Simultaneous Diagonal Joint Steps:** `planner.joint_plan` prioritizes simultaneous two-axis diagonal steps (`[1, 1, 0]`, `[2, 2, 0]`, `[1, 2, 0]`, `[2, 1, 0]`) ahead of single-axis steps while checking `safe()` and `safe_edge()` at `<= 0.5°` resolution across the CAD collision bodies and floor bounds. Moving `M3 +10°` and `M4 +10°` together (`[1, 1, 0]`) translates the hand downward and forward while holding the forearm world pitch (`M4 - M3 + 90°`) constant, reducing the descent from `WORKING_POS` `(150°, 45°)` to `(250°, 105°)` from 16 single-axis staircase steps to 10 smooth diagonal steps.
- **3-Axis `MoveArm` Folding:** When `m5_deg` also changes, `planner.trajectory` (called by `Executor.move_arm` and `GoalContext.plan_trajectory`) folds `M5` roll steps (`1` or `2`) into the `M3/M4` step vectors (`[m3, m4, m5]`) whenever the 3-axis `safe_edge` check passes, falling back to sequential `M5` steps when needed.
- **Coordinated Visual Servoing & Tower Descent:** `Executor.front_servo` and `planner.descent_step` evaluate all 8 single- and dual-axis `(M3, M4)` step combinations (`[c3, c4, 0]`), enabling constant-pitch reach adjustments over the object equator and finer vertical descent increments (down to ~18.7 mm with `< 30 mm` lateral drift) when seating a held dodecahedron onto a support tier.
- **In-Place Jaw Release:** Normal jaw release uses Manual Arm Mode (`grab_mode == 1`, `app_pick_up_enter_exit {flag: 1}`) followed by `app_put_down {flag: 0}`, which opens the jaws in place without lowering the arm or rotating the base.

---

## Navigation, Garment Delivery, and Patrol Actions

| ROS Action / Type | Task Kind | Goal Fields |
| :--- | :--- | :--- |
| `/z70/nav/pin_and_go` (`PinAndGo`) | `pin_and_go` | `{target_x_m, target_y_m, max_duration_s, dry_run}` |
| `/z70/deliver_garment` (`DeliverGarment`) | `deliver_garment` | `{category, drop_x_m, drop_y_m, max_duration_s, allow_towing, dry_run}` |
| `/z70/patrol_collect` (`PatrolCollect`) | `patrol_collect` | `{drop_x_m, drop_y_m, max_duration_s, max_items, allow_towing, dry_run}` |

### Pin-and-Go (`/z70/nav/pin_and_go`)

```sh
ros2 action send_goal /z70/nav/pin_and_go z70_interfaces/action/PinAndGo \
  '{target_x_m: 24.5, target_y_m: 26.0, max_duration_s: 60.0, dry_run: true}' --feedback
```

- Target coordinates are in the decoded vendor map frame (metres, bounds `±100 m`, duration `1..600 s`).
- Requires a stowed arm (`IN_ROBOT` and `grab_status == 0`), battery `> 15%`, fresh `/z70/driver/map_snapshot` with `active_floor_verified` and `current_pose_verified`, and target containment inside the eroded safe interior (`z70_mapping.boundary`, configurable via `pin_and_go_allowed_segments`, default `[1..6]`).
- Dispatches `app_goto_target` once in integer millimetres, monitors boundary clearance and hazards during transit, and verifies `ARRIVED` on a fresh post-stop map observation within `0.25 m` of the target.

### Garment Delivery & Patrol (`/z70/deliver_garment`, `/z70/patrol_collect`)

```sh
ros2 action send_goal /z70/deliver_garment z70_interfaces/action/DeliverGarment \
  '{category: sock, drop_x_m: 1.0, drop_y_m: 2.0, max_duration_s: 120.0, allow_towing: false, dry_run: true}' --feedback
ros2 action send_goal /z70/patrol_collect z70_interfaces/action/PatrolCollect \
  '{drop_x_m: 1.0, drop_y_m: 2.0, max_duration_s: 600.0, max_items: 10, allow_towing: false, dry_run: true}' --feedback
```

- Uses `/z70/garments/observations` and plans loaded transit over a `+0.10 m` deployed-arm safety margin using 4-connected A* and capsule corridor checks (`LOADED_ROUTE_BLOCKED` if no safe corridor exists).
- Stages arm deployment in open space (`>= 0.8 m` clearance), reacquires the garment after deployment, grasps and lifts with two upward hand steps, verifies elevation and floor-spot contrast, transports to `(drop_x_m, drop_y_m)`, releases in place (`grab_mode == 1`, `app_put_down {flag: 0}`), withdraws `0.15 m`, and verifies detachment before stowing.
- `PatrolCollect` persists mission progress atomically in `patrol_ledger.json` (`patrol_ledger_directory`, default `.local/patrol`), tracks waypoint observation states (`UNVISITED`, `VISITED`, `ADEQUATELY_OBSERVED`, `OCCLUDED_DEFERRED`, `CLEARED`, `UNREACHABLE`), and enforces a `25%` battery reserve (`SAFE_RELEASE_BEFORE_DOCK` before charging, resuming at `80%`).

---

## Authoring ROS 2 Goals (`z70_manipulation/goals.py`)

All manipulation, navigation, and driver action specs are defined in `BUILTIN_GOALS`, `DRIVER_GOALS`, and `ALL_GOALS` inside `src/z70_manipulation/z70_manipulation/goals.py`. Both `PickServer` (`z70_manipulation/node.py`) and `Gateway` (`z70_web_gateway/gateway.py`) consume `goals.py` directly, so adding or updating a `GoalSpec` registers both the ROS 2 `ActionServer` and the HTTP/MCP validation schema in one place:

```python
from z70_manipulation.goals import ALL_GOALS, BUILTIN_GOALS, GoalContext, GoalSpec, simulate_goal

spec = GoalSpec(
    name='my_goal',
    action_cls=MyGoal,
    topic='/z70/my_goal',
    timeout_s=lambda req: req.max_duration_s,
    validate=lambda server, req: req.max_duration_s > 0,
    run=_goal_my_goal,
    fields=('max_duration_s', 'dry_run'),
    description='Custom ROS 2 action goal.',
    validate_payload=lambda p: isinstance(p['max_duration_s'], (int, float)) and isinstance(p['dry_run'], bool),
)
```

- **Runtime Goal Catalog (`GET /api/ros/goals`, `GOAL_CONTEXT_PRIMITIVES` & `GoalSpec.describe()`):** `z70_web_gateway` exposes `GET /api/ros/goals` (and MCP `list_goals`) returning `{goals: [{kind, topic, action, fields, field_types, result_fields, timeout_s, navigation, description}, ...], context_primitives: [{name, signature, summary}, ...]}` (`GOAL_CONTEXT_PRIMITIVES` in `goals.py`).
- **Dry-Run Goal Validation & Simulation (`POST /api/ros/goals/validate` & MCP `validateOnly`/`simulate`/`context`):** `POST /api/ros/goals/validate` validates a `{kind, goal, simulate?: bool, context?: dict}` request payload against `GoalSpec.validate_payload` and the ROS message schema without dispatching to hardware, and when `simulate: true` is passed (or via MCP `start_task(..., simulate: true, context?: {...})`), executes `simulate_goal()` in memory with any `context` primitive overrides and returns `simulated: {ok, result, trace}`.
- **Pure-Python Goal Simulation (`simulate_goal(spec, request, **overrides)`):** `simulate_goal()` constructs a stateful in-memory `GoalContext` backed by `SessionWorldMap` and `planner.{grasp_goal,stack_tier_plan,trajectory}` (overridable via callables or static return values), records primitive invocations in `ctx.trace`, and invokes `spec.run(ctx, request)` so `GoalSpec` handlers can be unit-tested offline without spinning up DDS or hardware.

Inside a goal handler `def _goal_my_goal(ctx: GoalContext, req) -> tuple[bool, MyGoal.Result]`, `ctx` provides:
- **State, Spatial Context & Perception:** `ctx.status()`, `ctx.arm()`, `ctx.arm_msg()`, `ctx.odom()`, `ctx.map_pose()`, `ctx.map_snapshot(refresh=False)`, `ctx.safe_interior(refresh=False, deployed_arm_extension_m=0.0)`, `ctx.spatial_context(refresh=False)`, `ctx.plan_route(target_x_m, target_y_m, deployed_arm_extension_m=0.0, refresh=False)`, `ctx.corridor_safe(start_x_m, start_y_m, end_x_m, end_y_m, radius_m=0.0, deployed_arm_extension_m=0.0, refresh=False)`, `ctx.objects(camera='front', color=None)`, `ctx.wrist_pose()`, `ctx.fit_wrist_joints()`, `ctx.garments(category='any')`, `ctx.world_objects()`, `ctx.update_world_object(x_m, y_m, category='object', mobility='pickupable', relevance='objective', radius_m=0.12, detail='')`, `ctx.mark_obstacle(x_m, y_m, radius_m=0.12, category='obstacle', detail='OBSTACLE')`, `ctx.clear_world()`, `ctx.frame()`
- **Kinematics, Motion & Manipulation:** `ctx.arm_safe(m3_deg, m4_deg, m5_deg=0.0, m6_deg=90.0)`, `ctx.arm_kinematics(m3_deg, m4_deg, m5_deg=0.0, m6_deg=90.0)`, `ctx.plan_approach(x_m, y_m)`, `ctx.plan_grasp(x_m, y_m, size_m=0.066)`, `ctx.plan_stack_tier(tier, size_m=0.066)`, `ctx.plan_trajectory(m3_deg, m4_deg, m5_deg=0.0)`, `ctx.navigate(x_m, y_m, max_duration_s=60.0, dry_run=False)`, `ctx.pick(color, dry_run=False)`, `ctx.stack(base_color, middle_color, top_color, dry_run=False)`, `ctx.deliver_garment(...)`, `ctx.patrol_collect(...)`, `ctx.deploy_arm(settle=True)`, `ctx.align_target(color)`, `ctx.move_arm(m3_deg, m4_deg, m5_deg=0.0)`, `ctx.arm_step(m3=0, m4=0, m5=0)`, `ctx.hand_step(direction)`, `ctx.open_jaws()`, `ctx.set_arm_anchor(anchor, reason='')`, `ctx.grasp_and_lift(lift_steps=2, verify_color='')`, `ctx.release_and_stow(put_down=True)`, `ctx.pulse(velocity=0.0, omega=0.0, duration=300)`, `ctx.rpc(method, params, timeout_ms=5000)`, `ctx.stop()`
- **Lifecycle & Feedback:** `ctx.sleep(seconds)`, `ctx.hazard()`, `ctx.is_cancelled()`, `ctx.log(stage, **fields)`, `ctx.feedback(**fields)`, `ctx.outcome(success)`, `ctx.trace`

---

## Perception and Safety

- `/z70/objects` (`visualization_msgs/msg/MarkerArray`) and `/z70/objects/json` (`std_msgs/msg/String`) publish detected HSV objects in `base_footprint` from matched front-camera frames.
- Publishing `/z70/safety_stop` (`std_msgs/msg/Bool`), cancelling an action goal, or invoking driver priority stop (`/z70/stop`) immediately halts active base and arm motion.
