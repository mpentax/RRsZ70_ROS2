# Thin authenticated API (current service contract)

RoboClaw authenticates private service callers and Roborock account/device access,
encodes vendor RPCs, correlates responses and publishes facts. Phase-3 source makes
`z70_driver` the sole ROS backend client and command/estimate owner; manipulation
uses ROS interfaces. Phase-4 MCP/web clients now use the shared authenticated
`/api/ros/*` adapter; [phase4-api.md](migration/phase4-api.md) defines its routes,
goal IDs, JSON fields, errors and web input flows. This cutover is offline-qualified,
not deployed; do not apply it over an active controller.

## RPC

Private bearer-only `POST /api/driver/rpc`, used by `z70_driver` only.
Legacy `/api/v1/rpc`, device RPC paths, named actuator aliases, arm estimator/
anchor routes and `/api/camera/select` return explicit 503 migration errors;
they do not bypass ROS. Advanced vendor RPC remains available via ROS RawRpc.

```json
{"method":"app_rc_move","params":[{"velocity":0.07,"omega":0,"duration":400,"seqnum":1}],"transport":"local","timeoutMs":5000}
```

`transport` defaults to `local`; `cloud` uses the service's saved authenticated
account directly, not a credential-bearing go2rtc URL. The selected device's
stored address/key cannot be overridden by requests. This is vendor RPC, not an
arbitrary HTTP proxy. JSON payload and transport capacity bounds apply.
Methods are not restricted to a control allowlist. Finite timeout 1–120000 ms,
method identifiers and protocol framing limits are structural validation, not
supervision policy. Values are vendor wire values, not calibrated SI units.

A response contains `requestId`, `commandId`, `attemptId`, `attemptIndex`,
`dispatch` (`not_sent`, `sent`, `unknown`),
`response` (`received`, `vendor_error`, `transport_error`), timestamps,
`physicalCompletion: "unverified"`, and `result` or data-free `error` plus the
vendor error code where available. Transport errors include a data-free `transportReason`
(timeout/capacity/invalid-input/unavailable/transport/cancelled), not upstream exception text. HTTP 200 means a correlated vendor response
was received; HTTP 502 includes the outcome envelope for vendor/transport errors.
It does not mean physical task success. On TCP/socket/MQTT handoff followed by
timeout, delivery is unknown. Vendor errors prove a correlated response, not
that every possible side effect was absent. Invalid API envelopes receive 400.

No prior request state, ACK, uncertain outcome, missing measurement, old journal,
operator lease, motion hold, battery level or arm mode blocks a later request.
The service retries reads, RC start/stop/end or NOT_SENT once, except intent
explicitly cancelled by a priority stop. Potentially written
motion is not automatically replayed. Controllers may explicitly
submit the next request, including an intentional retry, without clearance.
Reconnect occurs before a new request; no silent local/cloud failover is used.

## Adapters and retired routes

MCP `robot_rpc` and browser controls use `/api/ros/*`, proxied to the loopback
`z70_web_gateway` ament node. Existing TS stdio MCP is a protocol-only client,
not another estimator, calibration reader, task owner or ROS launcher. Web and
MCP share the real ROS action goal UUID, feedback and terminal results. Long tasks
return handles, not a synchronous wait for execution. No direct actuation fallback
exists. Browser cookies cannot access private acquisition/offer/telemetry routes;
their camera snapshots come from matching shared ROS images/metadata. Account,
device and camera-pattern input remains authenticated and server-side. ROS bounded actions and
RawRpc share one scheduler, estimator and backend event stream. Stop has reserved
local/cloud transport capacity, cancels queued actuator intent and never replays
cancelled motion as an unsent retry. Private `/api/driver/camera/select` returns
underlying outcome UUIDs; a normal selector timeout remains UNKNOWN even when
status confirms the lens. Frame reads with `{}` follow the shared camera; a
camera selector in the old frame route returns 503 rather than changing mode.

`GET /api/commands/events` is an authenticated live NDJSON dispatch stream used
by `z70_driver`. Logical command/attempt/session UUIDs, sequence, dispatch-call
timestamps and uncertainty are preserved; no replay, arbitrary result or private
parameter payload is included. Blank heartbeat lines carry no command evidence.
Slow readers are disconnected rather than delaying dispatch. A reconnect exposes
sequence gaps; the driver never reconstructs missing commands by polling history.
`/api/commands/recent` remains bounded compatibility history, with no ROS consumer.
Other retired command-workflow routes,
`/api/docking*`, `/api/agent/docking/start`, and
`/api/robotics/*` return 410. No bypass flags or compatibility clearance gates
remain. The old leased ROS fixtures and DimOS integration are retired; the
current native Jazzy workspace has ROS-commanded pickup and the
single shared `z70_driver` command/estimate/observation owner (offline-qualified through phase 4).
Hermes currently calls the MCP server directly. This is a deliberate API migration, not schema-compatible
ROS support. The legacy unauthenticated cockpit entry point is removed.

Bearer-private acquisition state, camera frame/offer, room list and map-cache APIs
remain for the ROS driver; browser camera PIN and account/device onboarding use
web setup, never DDS. Service-owned local VLM description/navigation
endpoints are no longer part of this service. Generic vendor RPC remains available
regardless of camera configuration. Camera capture receipt time/dimensions are
reported without fabricating lens, capture time or physical pose.

## Authentication and migration

Bearer authentication and same-origin protected browser cookies remain. Stored
credentials are private plaintext mode-0600 files, not encrypted at rest. Browser
and MCP responses redact credential keys/known secret values. Operator helper
entry points must not expose tokens to robot controllers.

Old hold/authority/docking journals are left untouched and ignored. They do not
need clearing, recovery evidence or a successful filesystem checkpoint before a
command. Existing credential documents remain compatible for initial loading. New account selections
publish a single validated, atomic mode-0600 `pairing.json`; startup prefers it
over legacy split documents, preventing mixed-account persistence. Camera PIN and
room updates use that same pairing document after migration. An empty saved camera
PIN is allowed for account/control setup without enabling camera preview.

`research/` retains offline reverse-engineering tests outside `npm test`; historic
policy documents do not describe current command admission. Focused regressions
prove arbitrary vendor RPC, exact aliases, auth/redaction, codec correlation,
no automatic replay of potentially written motion, and a subsequent command after an unknown result.
