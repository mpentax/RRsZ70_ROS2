# Shared API and Private Transport Contract

## Ownership and Authentication

- **`z70_driver`** owns ROS command scheduling, `ArmEstimator` dead reckoning, and shared camera/map acquisition.
- **Vendor sidecar (`adapters/vendor`)** owns Roborock authentication, LAN/MQTT framing, and RPC correlation.
- **`z70_web_gateway`** is a loopback ROS client (`127.0.0.1:57020`) serving both the web console (`apps/web`) and stdio MCP server (`apps/mcp`).

Browser sessions use same-origin cookies (`POST /api/session {pin}`). When no camera PIN is configured yet, `GET /api/session` reports `setupAvailable: true` and `POST /api/session {setup: true}` opens a 20-minute localhost setup session for `/api/setup`, `/api/account/*`, and `/api/camera/pin`. Service-to-service calls use `Bearer <service-token>` from `.local/service-token`. Credentials are stored in `.local/pairing.json` (mode `0600`) and redacted from all API and MCP responses.

---

## Shared ROS HTTP Facade (`/api/ros/*`)

Proxied by the sidecar to `z70_web_gateway` (`127.0.0.1:57020`). Task IDs are lowercase hyphenated UUIDs matching the underlying ROS action goal UUID.

| Method & Path | Request / Response |
| :--- | :--- |
| `GET /api/ros/state` | `{available, session_enabled, state, camera, arm, calibration}` (`state.grab_status` decoded from `common_status` bits `6..9`, `grab_mode` in bits `10..11`). |
| `GET /api/ros/map` | `{map: OccupancyGrid \| null, dynamic_obstacles?: OccupancyGrid, world_objects?: dict}` |
| `POST /api/ros/frame` | `{}` returns the active camera JPEG; `{camera: "front" \| "gripper"}` filters by lens without switching cameras. Response headers include `X-Roboclaw-Camera`, `X-Roboclaw-Frame-Id`, `X-Roboclaw-Camera-Session`, and `X-Roboclaw-Received-At`. |
| `POST /api/ros/camera/select` | `{camera: "front" \| "gripper"}` -> calls `/z70/camera/select` (`SelectCamera`). |
| `POST /api/ros/arm/anchor` | `{anchor: "unknown" \| "stowed" \| "deployed", reason?: string}` -> calls `/z70/arm/anchor` (`SetArmAnchor`). |
| `POST /api/ros/stop` | `{scope?: 1 \| 2 \| 3}` (`1` BASE, `2` ARM, `3` ALL; default `ALL`) -> calls `/z70/stop` (`Stop`). |
| `POST /api/ros/control/session` | `{enabled: bool}` -> calls `/z70/control/session` (`SetControlSession`). |
| `POST /api/ros/rpc` | `{method, params, timeoutMs?: 1..30000, transport?: "local" \| "cloud", takeover?: bool}` -> calls `/z70/driver/raw_rpc` (`RawRpc`). |
| `GET /api/ros/goals` | `{goals: [{kind, topic, action, fields, field_types, result_fields, timeout_s, navigation, description}], context_primitives: [{name, signature, summary}]}` |
| `POST /api/ros/goals/validate` | `{kind, goal, simulate?: bool, context?: dict}` -> dry-run schema/field validation (and optional in-memory `simulate_goal()` execution with `context` primitive overrides returning `simulated: {ok, result}`) without dispatching to the robot. |
| `POST /api/ros/tasks` | `{kind, goal}` -> `202` `Task` after action acceptance. |
| `GET /api/ros/tasks` | `{tasks: [Task]}` |
| `GET /api/ros/tasks/<id>` | `Task` (`{id, kind, status, phase, feedback, result}`) |
| `POST /api/ros/tasks/<id>/cancel` | `{}` -> `{id, cancel_requested: true}` |

### Supported Task Kinds (`POST /api/ros/tasks` & MCP `start_task`)

| `kind` | ROS Action | Goal Schema |
| :--- | :--- | :--- |
| `base_pulse` | `/z70/base/pulse` | `{linear_mps: -0.1..0.1, angular_radps: -0.5..0.5, duration: {sec: 0, nanosec: 1..800000000}}` (non-zero on one axis only) |
| `arm_step` | `/z70/arm/step` | `{joint: 3 \| 4 \| 5 \| 34 \| 43, direction: -1 \| 1}` (`3..5` single joint `M3..M5`; `34` coordinated `M3 & M4` same-sign `[1,1,0]`/`[2,2,0]`; `43` coordinated `M3 & M4` opposite-sign `[1,2,0]`/`[2,1,0]`) |
| `hand_step` | `/z70/arm/hand_step` | `{direction: 1..4}` (`1` up, `2` down, `3` forward, `4` back; nominal 40 mm IK step) |
| `arm_operation` | `/z70/arm/operation` | `{operation: 1..9}` (`1` unstow, `2` stow, `3` recovery-open, `4` grip, `5` put-down / in-place release, `6` enter manual mode `flag:1`, `7` enter remote-pickup mode `flag:2`, `8` exit arm mode `flag:0`, `9` arm recover) |
| `deploy_arm` | `/z70/arm/deploy` | `{settle: bool}` |
| `align_target` | `/z70/align_target` | `{color: string}` |
| `move_arm` | `/z70/arm/move` | `{m3_deg: 90..270, m4_deg: 35..170, m5_deg: 0..155}` (checked against `planner.safe()`) |
| `grasp_and_lift` | `/z70/arm/grasp_and_lift` | `{lift_steps: 0..10, verify_color: string}` |
| `release_and_stow` | `/z70/arm/release_and_stow` | `{put_down: bool}` |
| `pick` | `/z70/pick` | `{color: string, dry_run: bool}` |
| `stack` | `/z70/stack` | `{base_color: string, middle_color: string, top_color: string, dry_run: bool}` (distinct colors) |
| `pin_and_go` | `/z70/nav/pin_and_go` | `{target_x_m: -100..100, target_y_m: -100..100, max_duration_s: 1..600, dry_run: bool}` |
| `deliver_garment` | `/z70/deliver_garment` | `{category: string, drop_x_m: -100..100, drop_y_m: -100..100, max_duration_s: 1..3600, allow_towing: bool, dry_run: bool}` |
| `patrol_collect` | `/z70/patrol_collect` | `{drop_x_m: -100..100, drop_y_m: -100..100, max_duration_s: 1..3600, max_items: 1..1000, allow_towing: bool, dry_run: bool}` |

### Task Lifecycle and Error Codes

- `Task.status` values: `pending`, `accepted`, `executing`, `succeeded`, `failed`, `cancelled`, `rejected`, `unknown`.
- Error responses use `{error: CODE, dispatch: "not_sent" | "unknown"}`:
  - `400`: `INVALID_INPUT`
  - `404`: `TASK_NOT_FOUND`
  - `409`: `REJECTED`, `CAPACITY`, `TASK_FINISHED`, `CANCEL_REJECTED`, `SESSION_STOPPED`
  - `503`: `ROS_UNAVAILABLE`, `OBSERVATION_GAP`
  - `504`: `RESPONSE_TIMEOUT`

---

## ROS Scheduling, Stop, and Private Driver Transport

- **Single Command Dispatcher:** `z70_driver` serializes ordinary write RPCs. When a manipulation task holds an active session token (`/z70/tasks/session`), manual writes without that token return `BUSY`. Starting a task with `takeover: true` revokes the prior task, runs priority `ALL` stop, and assigns the new token.
- **Priority Stop (`/z70/stop`):** Uses reserved transport capacity independent of in-flight RPCs. `BASE` (`1`) sends `app_rc_stop` + `app_rc_end`; `ARM` (`2`) sends `app_stop_grasp`; `ALL` (`3`) sends all three (plus `stop_goto_target` when a native navigation goal was active).
- **Private Driver Endpoint (`POST /api/driver/rpc`):** Bearer-authenticated endpoint used by `z70_driver` (`timeoutMs: 1..120000`). Returns `{requestId, commandId, attemptId, attemptIndex, dispatch, response, result}` and streams dispatch events on `GET /api/commands/events`.
