# Independent robotics safety/integration review

Reviewed 2026-09-30. Read-only code review except this report; no commits, live robot reads/actions, service restarts, Docker builds, or ROS starts. Findings refer to the inspected working tree, which other workers may subsequently change.

## Required corrections

### P1 — New robotics status publication permits arbitrary strings under allowlisted keys

**Locations:** `src/robotics-api.ts:35-37`, `src/credentials.ts:73-81`; published verbatim by `robotics/ros2/src/roboclaw_readonly/roboclaw_readonly/node.py:70`.

The new endpoint reuses `summarizeStatus`, whose field-name allowlist accepts arbitrary string values for every status field, including battery/state. A synthetic robot response `{battery: "https://private.example/?token=FAKE_SECRET"}` survives into `raw.status.value`; an offline invocation confirmed this. Thus the claim that the new ROS boundary cannot publish upstream URLs/credentials is stronger than its implementation. The property-name allowlist alone does not sanitize values. `fw_ver` also accepts unconstrained strings. Nonfinite numeric values can serialize as null while the observation is still labelled fresh.

**Correction:** use robotics-specific typed status validation: finite bounded numeric values for numeric fields and a narrowly validated/bounded version string (or omit it). Reject malformed values before marking observations fresh. Add malicious strings under each allowed key and nonfinite-number tests. This is an upstream-response privacy boundary issue, not an observed real credential exposure.

### P1 — Observation polling bypasses writer admission and consumes the same limited RPC pool

**Locations:** `src/service.ts:680-682`, `src/robotics-api.ts:25-38`; transport evidence `src/robot.ts:225-244,247-282`; ROS caller `robotics/ros2/src/roboclaw_readonly/roboclaw_readonly/client.py:143-158`.

Every state request submits two simultaneous ordinary RPCs with no busy check, coalescing, global request admission, or disconnected-client cancellation. Ordinary reads share four inflight slots and a 32-request pending pool. Nonpriority supervised motion refuses dispatch when any request is queued or four requests are inflight (`robot.ts:240-242`). Two overlapping robotics callers alone can fill all four slots; additional callers create a backlog that rejects legitimate arm/chassis commands. Existing arm observation explicitly refuses reads while the arm is busy (`service.ts:969-970`), but this new endpoint bypasses that policy. Priority stop has reserved capacity, so this does not establish that all stops are blocked.

The ROS HTTP timeout defaults to 3 seconds; robot reads allow a 15-second queue wait and 5-second response timeout. HTTP timeout/disconnection does not cancel reads, so abandoned polls can remain active while later calls accumulate. The new endpoint also uses the ordinary `request` path, whose implementation invokes `connect()`; its initial connected guard is not an atomic no-reconnect guarantee if the connection drops between guard and enqueue.

**Correction:** introduce shared read admission/coalescing with writer priority, reject/defer polling during active/claiming arm/chassis work, bound outstanding reads globally, and use an already-connected read API that cannot reconnect. Align HTTP deadlines with the complete queue-plus-response budget and cancel or account for abandoned reads. Add fake-transport tests demonstrating poll overlap cannot block movement admission and a disconnect cannot trigger connection creation. Do not authorize live ROS polling until this is resolved or explicitly accepted with evidence.

### P2 — Typed outcome propagation is not end-to-end

**Locations:** `src/action-outcome.ts:14-25`, `src/mcp-server.ts:32,87,130-144`; compare `web/src/outcomes.ts:7-18`.

The UI consumes a nested `actionOutcome` or direct typed outcome, but the server classifier ignores both. MCP drops those keys via `ACTION_REPLIES` before classification. A synthetic `{actionOutcome:{outcome:"rejected"},state:{busy:true}}` classifies as `in-progress`; a direct explicit rejection without busy becomes unknown. Error responses are discarded before body parsing (`mcp-server.ts:87`), so the dock route's `NEEDS_VERIFICATION` reason never reaches the MCP caller. This is a concrete propagation gap, although current service replies largely remain legacy ACK/state shapes.

**Correction:** define one sanitized typed outcome envelope and preserve/validate it through service, MCP and UI; explicit correlated outcome evidence should outrank heuristic phase inference. Parse bounded allowlisted non-2xx evidence rather than leaking arbitrary error bodies. Add MCP tests for nested/direct typed rejection, acknowledged:false with busy, and dock's staged reason. Keep legacy 409 ambiguous unless the service supplies explicit no-dispatch evidence.

### P2 — Dock tool now guarantees failure but still advertises a native docking request

**Locations:** `src/service.ts:887-918`, `src/mcp-actions.ts:20-23`, `src/docking-workflow.ts:17-55`.

The service always returns 409 after successful docking preflight and never invokes `DockingWorkflow`. Fail-closed behavior is appropriate while physical-stow evidence and shared reservations do not exist, but this is a deliberate loss of the previous docking capability, not a functioning stow-before-dock integration. The MCP tool description still offers a native return-to-dock request; generic MCP error conversion hides the actual disabled/staged reason. It also performs a robot status read before returning an inevitable disabled result.

**Correction:** retain the safety block, but explicitly mark the tool unavailable/staging-only and expose the machine-readable `NEEDS_VERIFICATION` reason. Consider returning the fail-closed result before unnecessary preflight reads. If functional docking is a required deliverable, identify it as incomplete until trusted evidence and a reservation covering every writer are implemented; do not solve this by trusting caller booleans or joint zeros.

### P2 — ROS validator accepts envelopes inconsistent with its own observation contract

**Locations:** `robotics/ros2/src/roboclaw_readonly/roboclaw_readonly/client.py:31-58,161-173`.

Validation checks a subset of fields but does not validate `connection`, enforce `physicalArmStowed:null`, validate `error`, require age for fresh observations, require null value/time/age for unavailable observations, or reject `observedAt > generatedAt`. `observation_health` uses wall-clock timestamps but ignores declared age/connection. A malformed v1 envelope can therefore pass validation, be published wholesale, and receive OK diagnostics despite contradictory provenance/availability fields. Capabilities validation checks only readOnly, not commands/cameraAccess/observation schema. This is not a TypeScript/Python version mismatch: normal generated v1 envelopes are compatible.

**Correction:** validate required v1 safety invariants and types while allowing genuinely additive fields. Reject contradictory observation states and forbidden physical-stow assertions, and test diagnostic behavior against malformed timestamps/ages/connection states. Document whether unknown additive fields are safe to republish or filter the published envelope.

## Reviewed boundaries / nonfindings

- Robotics routes are behind existing `auth.checkHost` and `auth.require`, reject query parameters, and permit GET only (`service.ts:650,678-682`). No newly introduced unauthenticated or host-check bypass was found.
- The Python client disables proxy inheritance, refuses redirects, and logs closed data-free errors. Host override does not itself bypass server bearer authentication. Arbitrary configured base URLs are an operator trust decision; use HTTPS outside the documented private local route.
- Receipt timestamps are explicitly documented as service receipt time, not independently verified device measurement time. Failed reads discard previous samples and a session change invalidates the complete snapshot. I found no fabricated physical-stow/joint calibration claim. Freshness still means successful on-demand receipt, not guaranteed fresh device measurement.
- Dock workflow checks evidence source/time and rechecks permission/readiness while holding the supplied reservation. It is not production-wired and supplies no real shared lock. `acquireExclusive` must cover all writers; adapters cannot use a no-op lock. The boolean ports also cannot establish physical completion, which remains explicitly unverified.
- Small robustness issue: `docking-workflow.ts:49` calls release before resetting `#running`; a throwing release escapes the method and leaves the instance permanently running. Nest the reset in a finally around release if adapters may throw; add a fake release-throws test.
- ROS publishes JSON String plus diagnostics rather than falsely calibrated JointState/Odometry/tf. The envelope version, timestamps and standard ROS message use are aligned for valid fixture/server responses. Container/ROS runtime proof was left to the ROS worker; this review did not build or run it.

## Offline verification

Executed only targeted offline tests:

- `node --test src/robotics-api.test.ts src/docking-workflow.test.ts src/action-outcome.test.ts src/mcp-actions.test.ts web/tests/outcomes.test.ts web/tests/api.test.ts`: **32 passed, 0 failed**.
- `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s robotics/tests -v`: **7 passed, 0 failed**. HTTP tests use only synthetic localhost fixture/test servers, not the application or robot.
- Standalone synthetic imports reproduced arbitrary allowed-key string passthrough and nested typed rejection being classified as in-progress. No real credentials were used.

Passing tests cover nominal boundaries but do not currently cover the above malformed-value, cross-caller resource-admission, typed-envelope, or contradictory-schema cases.
