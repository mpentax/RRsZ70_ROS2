# Phase 4 shared adapter contract

Implementation contract for offline phase 4. No deployment or hardware authorization.

The existing authenticated TypeScript web backend proxies `/api/ros/*` to the loopback ament Python `z70_web_gateway` node (default http://127.0.0.1:57020). The gateway is a ROS client, not a vendor client. It authenticates the same server-side bearer token read from a token_file ROS parameter. Secrets and account/device/PIN setup remain in the existing authenticated backend `/api/account/*` and `/api/camera/pin`; secrets never cross DDS. Existing TS stdio MCP is a thin protocol shim over the same web API; it owns no robot state, task registry, calibration files or workflows.

All shared JSON uses generated ROS snake_case message fields; UUIDs inside generated messages retain ROS shape. Task handles use lowercase hyphenated UUID strings equal to the ROS action goal UUID. One gateway owns the registry for both surfaces. Retain at most 256 terminal tasks; never evict active tasks. A gateway restart loses history and returns TASK_NOT_FOUND, never fabricated completion.

Routes on both gateway and backend (paths preserved):
- GET `/api/ros/state`: `{available:bool, state:RobotState|null, camera:CameraState|null, arm:ArmEstimate|null, calibration:CalibrationState|null}`. Original evidence timestamps/identities retained; availability reflects ROS driver service availability, not battery/cache presence.
- GET `/api/ros/map`: `{map:OccupancyGrid|null}`; standard mapping node owns interpretation.
- POST `/api/ros/frame` `{}` or `{camera:'front'|'gripper'}`: latest shared compressed JPEG, never acquisition HTTP or hidden selection. Explicit requested camera filters shared frames; switch only through select. JPEG headers X-Roboclaw-Camera, X-Roboclaw-Frame-Id, X-Roboclaw-Camera-Session retain shared sample identity. Missing/stale frame returns 503 OBSERVATION_GAP. Maximum age 5s.
- POST `/api/ros/camera/select` `{camera:'front'|'gripper'}`: SelectCamera service response.
- POST `/api/ros/arm/anchor` `{anchor:'unknown'|'stowed'|'deployed',reason?:string<=64}`: SetArmAnchor response.
- POST `/api/ros/stop` `{scope?:1|2|3}`: Stop response, independent of ordinary task requests.
- POST `/api/ros/rpc` `{method,params,timeoutMs?:integer,transport?:'local'|'cloud',takeover?:bool}`: RawRpc service response, no direct fallback or retry. Unavailable response after possible write reports unknown. No device-ID override: select devices in web setup.
- POST `/api/ros/tasks` `{kind:'pick'|'base_pulse'|'arm_step'|'hand_step'|'arm_operation',goal:object}`: 202 `{id,kind,status,phase,result:null}` immediately after bounded action acceptance (no waiting for execution). Goals use generated field names and SI units. base_pulse duration accepts `{sec,nanosec}`.
- GET `/api/ros/tasks`: `{tasks:[Task]}`.
- GET `/api/ros/tasks/<id>`: Task.
- POST `/api/ros/tasks/<id>/cancel` `{}`: `{id,cancel_requested:true}` only when cancellation accepted; subsequent task status/result remains authoritative.

Task: `{id,kind,status,phase,feedback,result}`. status is accepted/executing/succeeded/failed/cancelled/unknown; phase retains numeric feedback phase (initial 0). feedback/result retain generated ROS structures, with no interpretation of ACK as physical completion. Lost action response -> unknown, never automatic replay. Input rejection: 400 INVALID_INPUT; rejected goal: 409 REJECTED; missing task:404 TASK_NOT_FOUND; unavailable ROS:503 ROS_UNAVAILABLE; bounded response timeout:504 RESPONSE_TIMEOUT. Errors are fixed `{error:CODE,dispatch:'not_sent'|'unknown',physicalCompletion:'unverified'}`; never arbitrary exception strings. Read-only reconnect is passive; writes never retried.

Setup UI uses existing backend payloads: POST account/code {email,region}; account/login {code} -> {devices:[{id,name,model}]}; account/select {id,host,localProtocol:'1.0'|'L01',pin?}; camera/pin {pin}. Invalid/expired responses remain specific bounded backend errors. Browser clears secret input values after submission and on session expiry; no storage/logging. Setup status GET `/api/setup` returns selected device metadata, accountConfigured/pinConfigured and supported regions/protocols, never credentials. Robot state/camera/map/tasks come only from ROS routes.
