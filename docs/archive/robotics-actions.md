> Historical/superseded architecture. Current contract: [thin-api.md](thin-api.md).
> The service-owned admission, recovery and workflow policies described below have been removed.

# Bounded ROS actions — shared authority client

`roboclaw_actions` exposes `/roboclaw/bounded_operation` using the generated `BoundedOperation` action. Simulation is the default. Production requires both `mode:=REAL` and `enable_production:=true`, a private bearer-token file, and successful versioned capability negotiation. This adapter calls only the narrow robotics HTTP endpoints; arbitration, durable admission and physical bounds belong to the service-wide command authority.

## Modes and operations

| Mode | Operations / arguments in ROS `parameters_json` |
|---|---|
| `SIMULATION` (default) | `chassis_pulse`: `{linear,angular,durationMs}` (±0.2, ±0.5, 1–1000ms); `arm_operation`: `{operation:"stow"|"deploy"}`; `return_to_dock`: `{}`; `stop`: `{}` |
| `REAL` (explicit opt-in) | `arm_action`: `{action:"deploy"|"stow"|"open"|"close"}`; `chassis_pulse`: `{direction:"forward"|"backward"|"left"|"right",durationMs:100..500}`; `chassis_undock`: `{durationMs:100..500}`; `stop`: `{}` |
| `DISABLED` | Reject every goal |

Production rejects simulation velocity arguments, arbitrary joints/RPCs, and native dock. Native dock remains the separately supervised docking workflow. Simulation `return_to_dock` is only a legacy synthetic fixture.

Command IDs are unique `[A-Za-z0-9_-]{1,64}` strings. ROS `timeout_ms` is a 100–120000ms **observation deadline**, not a physical-motion limit, and is not sent as a production execution instruction. Cardinal pulse durations are enforced separately by the service. At most one motion and one independent STOP goal occupy execution workers, using the four-thread executor.

## Exact production contract for service integration

The existing read-only capabilities envelope remains intact, with this **optional** extension:

```json
{
  "apiVersion": "1.0",
  "kind": "capabilities",
  "generatedAt": 1790899200000,
  "data": {
    "readOnly": true,
    "commands": [],
    "actions": {
      "schema": "roboclaw.bounded-action.v1",
      "commandsAvailable": true,
      "simulation": false,
      "operations": ["arm_action", "chassis_pulse", "chassis_undock", "stop"],
      "binding": {"deviceId": "device-id", "session": 1790899200000},
      "endpoints": {
        "submit": "/api/robotics/v1/actions",
        "status": "/api/robotics/v1/actions/{commandId}",
        "cancel": "/api/robotics/v1/actions/{commandId}/cancel"
      }
    }
  }
}
```

Other existing capability fields are omitted from this example for space. Version, schema, full operation list, exact fixed endpoint templates, availability, simulation flag and binding are validated. Missing/disabled/mismatched extensions fail before writing. Simulation cannot authorize REAL and REAL capabilities cannot authorize simulation. Non-STOP goals renegotiate. Independent STOP can reuse an explicit prior grant bound to the exact endpoint, mode, opt-in, Host header, token path and token contents; failed explicit negotiation clears it.

Only these endpoints are used:

1. `GET /api/robotics/v1/capabilities` as above.
2. `POST /api/robotics/v1/actions` with exactly `{commandId,operation,arguments}`. Example: `{"commandId":"pulse_1","operation":"chassis_pulse","arguments":{"direction":"forward","durationMs":100}}`.
3. `GET /api/robotics/v1/actions/{commandId}` for correlated outcome observation.
4. `POST /api/robotics/v1/actions/{commandId}/cancel` with `{}`. Cancellation requests must also pass shared authority and device/session fencing.

Every production request has bearer authentication from the file. Every submit/poll/cancel additionally sends `X-Roboclaw-Device-Id` and `X-Roboclaw-Device-Session` captured from negotiation. **The service must compare these to the current connected binding before dispatch**, including independent STOP: it must never redirect a stale goal to a newly selected device/session. Device IDs must match `[A-Za-z0-9_-]{1,128}`; session is a nonnegative JS-safe integer (the connected session timestamp).

All action status responses have `Content-Type: application/json` and this shape:

```json
{
  "apiVersion": "1.0",
  "schema": "roboclaw.bounded-action.v1",
  "commandId": "pulse_1",
  "operation": "chassis_pulse",
  "binding": {"deviceId": "device-id", "session": 1790899200000},
  "writerLease": {
    "id": "lease-id",
    "writer": "robotics",
    "sessionId": "authority-scope-session",
    "expiresAt": 1790899260000
  },
  "state": "running",
  "vendorAck": true,
  "verifiedSuccess": false,
  "cessationVerified": false
}
```

`writerLease` is the service authority's ownership metadata, **not a new client-acquired authority**. The service owns acquisition, renewal and release. Lease ID and session ID are `[A-Za-z0-9_-]{1,128}`; writer is exactly `robotics`; expiry is Unix milliseconds. Motion records require lease metadata, except pre-admission `rejected` may use null. STOP may use null. Once a lease is observed it cannot disappear or change ID/writer/session; expiry may renew. `accepted`/`running` require an unexpired lease. Terminal records retain historical metadata. All responses must retain original goal ID, operation and device-session binding. Malformed, expired active, substituted or uncorrelated records become `unknown`.

Closed states: `accepted`, `running`, `cancel_requested`, `succeeded`, `rejected`, `failed`, `unknown`, `cancelled`. All three evidence flags are actual booleans. `succeeded` requires `verifiedSuccess:true`; successful STOP additionally requires `cessationVerified:true`; `cancelled` requires `cessationVerified:true`. Verified success on any other state is invalid. ACK alone proves neither physical completion nor stopped motion. Current service authority can honestly report `unknown` with `vendorAck:true` and both physical flags false. Physical evidence provenance/freshness is the service's responsibility; the client does not manufacture it from elapsed pulse time or a transport ACK.

Admission should return 202 with a status record. Definitive policy refusal may return a success HTTP status containing `state:"rejected"`; HTTP errors/timeouts/malformed bodies are conservatively ambiguous. Server deduplication must durably reserve IDs before possible issuance, reject reuse across restarts and never replay commands. Service integration must route every write/cancel through the existing shared authority and leave independent STOP available while motion is pending.

## Transport, cancellation and deadlines

Production permits only explicit `localhost`, `127.0.0.1`, `::1` or `host.docker.internal` service URLs. No discovery, proxy inheritance, redirect following, fallback endpoints, write retries or automatic replay. The token must be an owner-private regular file (no group/other permissions), owned by the adapter UID, not a symlink, containing only the bearer token. It is reread for each request; credential/transport identity changes prevent polling/cancelling an existing goal. Tokens and backend bodies are never republished in ROS results/logs.

Every goal ID is reserved before I/O for the process lifetime. Submit and cancel writes are each attempted at most once per ID, even after ambiguity; polling is GET-only. Backend restart durability remains essential. ROS cancellation acceptance merely accepts a request: unverified cancellation keeps polling until its observation deadline then aborts `unknown`, without claiming stopped motion. ROS terminality means the bridge stopped waiting.

The node uses a 1s request timeout capped to the remaining monotonic observation budget, checks the budget before and after each I/O, and discards late success. Responses are size-bounded. Socket timeout is not a hard wall-clock bound for trickled bytes or scheduling delays; physical execution can remain unresolved after observation expiry.

## Compose profiles

Simulation requires no credentials and runs the fake and adapter in a network-disabled container:

```sh
docker compose -f robotics/compose.actions-sim.yaml --profile actions-sim up --build
```

`robotics/compose.actions.yaml` is a separate explicit `actions-production` profile for the adapter only. It mounts `ROBOCLAW_ACTION_TOKEN_FILE` read-only at `/run/secrets/robotics-token`, with `create_host_path:false`. Its default transport is `http://host.docker.internal:57010`, Host `127.0.0.1:57010`; optional overrides are `ROBOCLAW_ACTION_BASE_URL` and `ROBOCLAW_ACTION_HOST_HEADER`. The mounted token must satisfy the runtime UID 10001/private-file check. No service startup, port publishing, host networking, restart policy, production deployment change or permission adjustment is included. This profile was configuration-validated only, never started against a live service.

## Isolated validation

```sh
python3 -m unittest discover -s robotics/tests -p 'test_*.py'
docker build -t roboclaw-ros:jazzy-actions-contract robotics
docker run --rm --network none --cap-drop ALL --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,uid=10001,gid=10001,mode=1777 \
  --tmpfs /home/rosdriver/.ros:rw,uid=10001,gid=10001,mode=0700 \
  -v "$PWD/robotics/tests:/tests:ro" roboclaw-ros:jazzy-actions-contract \
  python3 /tests/action_ros_smoke.py
```

Validated 2026-10-02: **30 Python tests passed** (including unchanged read-only tests); native ARM64 Jazzy image built all three packages. Both compose profiles passed configuration validation. Real ROS/DDS smoke passed the legacy simulation operations, duplicate/saturation rejection, independent STOP, cancellation evidence and late-result rejection; it also passed opt-in REAL mode against only a loopback production-contract fake with a temporary private token, all four production operations, ACK-only unknown, unverified cancellation with one cancel write, and mismatched device-session outcome rejection. Local synthetic simulation timing in the final rebuilt-image run: STOP 0.187s; cancel-through-cessation 0.208s, under the 1.5s test assertions. These are fixture timings, not robot stopping guarantees. No physical robot or deployed service was contacted.
