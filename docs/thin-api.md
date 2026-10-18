# Shared API and private transport contract

## Ownership and authentication

`z70_driver` owns ROS execution, command/status-derived arm estimation and shared
acquisition. The private TypeScript vendor sidecar owns Roborock authentication,
protocol framing and response correlation. `z70_web_gateway` is a loopback ROS
client; web and stdio MCP are thin clients of that same task/evidence registry.
There is no client-to-vendor actuation fallback.

The browser uses same-origin protected session cookies; configured-user login is
`POST /api/session {pin}` with the saved camera PIN. With no saved PIN,
`GET /api/session` reports `setupAvailable`; **Start local setup** posts
`{setup:true}` to obtain a same-origin, localhost-only setup cookie (20 minutes).
It can access only setup status and account/code/login/select/camera-PIN routes,
not ROS/control/acquisition. Saving a PIN revokes setup sessions; normal PIN login
is then required. Account/device/PIN input stays in the backend, never DDS. Service-to-service calls use `Bearer <service-token>` read
from a private file, not MCP tool arguments. Secrets are plaintext mode-0600 local
files, not encrypted at rest. Account login/camera signaling uses vendor cloud
services. Responses redact credential keys/known secret values; errors never
forward arbitrary upstream text or credential URLs.

Pairing updates publish one validated atomic `pairing.json`, preferred over
legacy split documents. Camera PIN/room updates use that document. Old
hold/authority/docking journals are ignored, not prerequisites to clear.

## Shared HTTP facade

The sidecar proxies `/api/ros/*` to the bearer-authenticated native gateway
(default `127.0.0.1:57020`; Docker Desktop origin `host.docker.internal:57020`).
JSON uses generated ROS snake_case fields; UUIDs inside messages retain their
ROS message shape. Task IDs are lowercase hyphenated UUID strings equal to the
actual ROS action goal UUID.

| Method/path | Request and response |
|---|---|
| `GET /api/ros/state` | `{available,session_enabled,state,camera,arm,calibration}`; messages may be null; availability is driver Stop service readiness, not cached telemetry presence |
| `GET /api/ros/map` | `{map: OccupancyGrid\|null}` |
| `POST /api/ros/frame` | `{}` follows shared selected lens; `{camera:"front"\|"gripper"}` filters shared cache **without selecting**; JPEG response |
| `POST /api/ros/camera/select` | `{camera:"front"\|"gripper"}` → generated SelectCamera response; no implicit deployment/mode entry |
| `POST /api/ros/arm/anchor` | `{anchor:"unknown"\|"stowed"\|"deployed",reason?:string}` → SetArmAnchor; reason ≤64 enum-like characters; changes estimate, not hardware |
| `POST /api/ros/stop` | `{scope?:1\|2\|3}` (BASE/ARM/ALL; default ALL) → Stop |
| `POST /api/ros/control/session` | `{enabled:bool}` → SetControlSession; false requests stop, viewing exit and acquisition suspension; inspect outcomes and `streams_released` |
| `POST /api/ros/rpc` | `{method,params,timeoutMs?:1..30000,transport?:"local"\|"cloud",takeover?:bool}` → RawRpc; default timeout 12000 ms/local/no takeover; params JSON ≤16384 UTF-8 bytes |
| `POST /api/ros/tasks` | `{kind,goal}` → 202 Task after bounded action acceptance, not task completion |
| `GET /api/ros/tasks` | `{tasks:[Task]}` |
| `GET /api/ros/tasks/<id>` | Task |
| `POST /api/ros/tasks/<id>/cancel` | `{}` → `{id,cancel_requested:true}` if accepted; read task result afterward |

JPEG headers `X-Roboclaw-Camera`, `X-Roboclaw-Frame-Id`,
`X-Roboclaw-Camera-Session`, `X-Roboclaw-Received-At` preserve the matched image/
metadata identity. Evidence must be valid/nonstale, matching stamp/frame/lens,
and received within five seconds. Missing data is 503 `OBSERVATION_GAP`;
suspended session is 409 `SESSION_STOPPED`. No hidden acquisition or lens restore.
Uncalibrated images remain viewable without inventing metric validity.

### Goals

Fields are required as listed; extra fields are rejected. Driver primitive units
are SI unless explicitly nominal arm selections.

| kind | ROS action endpoint | goal |
|---|---|---|
| `base_pulse` | `/z70/base/pulse` | `{linear_mps,angular_radps,duration:{sec:0,nanosec}}`; ≤0.1 m/s, ≤0.5 rad/s, not both axes nonzero, duration >0..800000000 ns |
| `arm_step` | `/z70/arm/step` | `{joint:3\|4\|5,direction:-1\|1}` |
| `hand_step` | `/z70/arm/hand_step` | `{direction:1..4}` |
| `arm_operation` | `/z70/arm/operation` | `{operation:1..5}`: out, in, recovery-open, grip, put-down (`commands.py` mapping) |
| `pick` | `/z70/pick` | `{color,dry_run:bool}` |
| `stack` | `/z70/stack` | `{base_color,middle_color,top_color,dry_run:bool}`; distinct colors |
| `deploy_arm` | `/z70/arm/deploy` | `{settle:bool}` |
| `align_target` | `/z70/align_target` | `{color}` |
| `move_arm` | `/z70/arm/move` | `{m3_deg:90..270,m4_deg:35..170,m5_deg:0..155}` |
| `grasp_and_lift` | `/z70/arm/grasp_and_lift` | `{lift_steps:0..10,verify_color}`; empty color disables optional verification |
| `release_and_stow` | `/z70/arm/release_and_stow` | `{put_down:bool}` |

Color tokens are 1–32 ASCII letters/digits/underscore/hyphen (verify_color may be
empty). HTTP range validity is not proof of model reachability or hardware motion.
See [manipulation](../src/z70_manipulation/README.md) for behavior and evidence.

### Tasks and errors

Task: `{id,kind,status,phase,feedback,result}`. Status can be `pending`, `accepted`,
`executing`, `succeeded`, `failed`, `cancelled`, `rejected`, `unknown`. Initial phase
is 0; generated feedback/result structures remain intact. One gateway retains at
most 256 terminal tasks, never evicts active/unknown tasks, and allows at most 32
nonterminal records. Restart loses the registry: missing IDs are `TASK_NOT_FOUND`,
not fabricated completion. A lost server/response yields unknown; no write replay.

Fixed errors are `{error:CODE,dispatch:"not_sent"|"unknown",physicalCompletion:"unverified"}`,
optionally `id` after goal allocation. Principal statuses: 400 `INVALID_INPUT`,
404 `TASK_NOT_FOUND`, 409 `REJECTED`/`CAPACITY`/`TASK_FINISHED`/`CANCEL_REJECTED`,
503 `ROS_UNAVAILABLE`, 504 `RESPONSE_TIMEOUT`. An accepted cancel is not cessation.
A timed-out task-start response may include an allocated UUID: inspect it before
considering another goal. HTTP/service success does not prove physical success.

## ROS scheduling and stop

One driver dispatcher serializes ordinary writes, with no pending FIFO. Manual
writes conflict with an active task token; task RawRpc must hold the current token.
TaskSession `/z70/tasks/session` begins/ends ownership; matching begin is idempotent,
end revokes future writes and publishes `/z70/tasks/cancel`. Explicit takeover
revokes the old task, completes priority ALL stop, then allows the replacement.
Failed stop latches ordinary dispatch disabled until explicit successful stop/
takeover. Current reasons include `BUSY`, `SESSION_STOPPED`, `STOP_UNVERIFIED`.
These are scheduler state, not legacy operator leases or backend admission policy.

Stop runs independently of stalled ordinary reply/camera waits with reserved
transport capacity. BASE sends `app_rc_stop` then `app_rc_end`; ARM sends
`app_stop_grasp`; ALL sends all three, total budget ≤15 s including backend safe
retry. Cancellation revokes future writes and attempts ALL stop. It does not
implicitly stow/open/put down, replay unknown motion or certify cessation.
Session disable also exits viewing and suspends acquisition until explicit resume.

## Private driver transport

Bearer-only `POST /api/driver/rpc` is for the driver, not browser cookies. Body:
`{method,params,timeoutMs,transport,commandId}`. Default transport is local;
cloud uses the saved account, never a credential-bearing media URL. Requests
cannot override the selected device's address/key. The private envelope timeout
range is 1–120000 ms (distinct from the gateway's 30000 ms maximum).
Method identifiers/payload/capacity bounds are structural, not a control allowlist.

Outcome includes request/logical command/attempt identities, attempt index,
`dispatch` (`not_sent`, `sent`, `unknown`), `response` (`received`, `vendor_error`,
`transport_error`), timestamps and `physicalCompletion:"unverified"`, plus result
or data-free error/vendor code. Transport reasons are closed categories, not raw
exceptions. HTTP 200 is correlated non-rejected response; 502 carries the vendor/
transport outcome; malformed input is 400. A correlated rejection does not prove
absence of all side effects. Response loss after possible write means UNKNOWN.
The driver allocates logical identity before HTTP; no received attempt evidence
means zero attempt UUID, not an invented backend event.

The private transport does not consult prior holds, missing measurements, old
journals, battery or arm mode before a new valid RPC. Safe reads/RC lifecycle or
`not_sent` may be retried once unless explicitly cancelled. Potentially written
motion is not automatically replayed; there is no silent local/cloud failover.
This permissive transport statement does **not** override ROS scheduler state.

Authenticated `GET /api/commands/events` is live NDJSON dispatch evidence with
logical/attempt/session IDs, sequence and source timestamps, no private params or
arbitrary results. Dispatch-call time does not prove the precise socket write
time, physical onset or image exposure. Blank heartbeats are not evidence. Slow readers disconnect;
sequence/session gaps stay explicit, with no history-based reconstruction.
`/api/commands/recent` is bounded compatibility history, not a ROS consumer.
Private camera selection `/api/driver/camera/select` retains underlying outcome
UUIDs; status-confirmed selection can still have UNKNOWN RPC delivery.

Private frame/offer/telemetry/room/map-cache APIs remain acquisition boundaries;
map cache receipt is not source capture or localization. Public clients only read
shared ROS frames. Legacy `/api/v1/rpc`, device actuator aliases, HTTP arm estimate/
anchor and `/api/camera/select` return 503 rather than bypass ROS; retired docking/
robotics workflow routes return 410. Old frame-route camera selectors are rejected.

## Browser setup payloads

`GET /api/setup` exposes selected-device metadata, accountConfigured/pinConfigured,
supported regions/protocols, not credentials. Setup uses `POST /api/account/code`
`{email,region}`, `/api/account/login` `{code}`, `/api/account/select`
`{id,host,localProtocol:"1.0"|"L01",pin?}`, `/api/camera/pin` `{pin}`. The browser
clears secret inputs after submission/session expiry; device selection remains a
web task, never an RPC device override. First-run admission is local browser setup;
configured login remains PIN-based. See [operations](../deploy/OPERATIONS.txt).
