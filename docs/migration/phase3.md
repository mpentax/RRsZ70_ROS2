# Phase 3 — ROS execution, estimation and calibration ownership

Task `t_16c0ca97`, checkout `/Users/mpentax/Code/RoboRockZ70-Connector`, branch
`claude/cleanup`, HEAD `de27d42dc098308949d6c6f122cadfcd8ac7a337` unchanged.
Implementation and offline qualification are complete. No hardware/camera access,
Docker/service deployment, checkout relocation, private-state inspection or commit
was performed. Existing geometry work and retained measurement originals remain.

## Implemented ownership

- `z70_driver` owns ordinary command scheduling, priority stop, command-derived arm
  estimation and shared observations. Manipulation sends ROS requests, not HTTP;
  its production Client no longer loads a backend token or opens a requests Session.
- Generated `z70_interfaces` defines BasePulse, ArmStep, HandStep, ArmOperation and
  Pick actions; Stop, SetArmAnchor, RawRpc and TaskSession services; and typed
  CalibrationState alongside phase-2 observation/evidence types. Pick server and
  its existing client switch together to `z70_interfaces/action/Pick`.
- One scheduler handles bounded primitives, task sequences, vendor programs and
  Advanced raw RPC. Ordinary conflicts return BUSY, without an unbounded FIFO.
  Task ownership is execution exclusion, not a permission/approval/lease system.
  Takeover revokes the old task and stops before new dispatch. Old-task cleanup
  cannot send a second stop after the new controller has begun.
- Standard action cancellation and an independent pickup watchdog work while
  commands, image reads or selection replies are stalled. Cancellation/deadline/
  failed task cleanup does not automatically open, put down or stow. Explicit
  successful pickup release/stow sequencing remains in the discrete executor.
- Stop runs independently of ordinary HTTP waits/camera locks. The local transport
  reserves stop capacity beyond four ordinary in-flight requests and cancels
  queued actuator intent. Cloud has reserved stop capacity too. Cancelled intent
  is NOT retried as an ordinary NOT_SENT opportunity. Reply budgets include
  connection/queue waiting and the backend's one allowed safe retry. Stop replies
  remain bounded to five seconds each and fifteen seconds total; ACKs never set
  cessation_observed or physical_completion to observed-complete.
- Advanced local/cloud RPC remains through that same scheduler/events/estimator;
  Stop uses the latest actuator transport. There is no automatic motion replay,
  transport failover, recovery agent or product-specific diagnostic command.

## Backend and identity cutover

The TS service remains a private credential/vendor/media backend. Driver uses
bearer-only `/api/driver/rpc` and `/api/driver/camera/select`. Public legacy RPC,
actuator aliases, TS arm-estimate/anchor and selector writes return explicit 503
ROS-cutover errors; explicit camera writes in the old frame route also fail.
Ordinary `{}` frame reads still follow the shared active lens. Browser cookie
callers cannot use private driver write routes.

The driver generates a logical command UUID before HTTP dispatch; backend reply
and dispatch-time events preserve it literally. Backend attempt UUIDs/indexes
remain separate across safe retries. Actual HTTP disconnection retains the
logical ID and UNKNOWN delivery; absent attempt evidence stays zero rather than
being invented. The driver never reconstructs commands from recent history or
emits a second copy of backend wire events. Selector replies include underlying
RPC outcomes: a status-confirmed lens change can still have UNKNOWN/TIMEOUT wire
evidence. Dispatch-call timestamps are not confirmed socket-write or exposure time.

`src/arm-estimator.ts` and its superseded TS suite were removed, and service/main
no longer instantiate or update an estimator. There is no second authority or
legacy estimator feature flag. Pure TS nominal geometry helpers are not a state
estimator and remain for later client/layout cleanup.

## Estimation and TF

`z70_driver.estimator.ArmEstimator` reuses the ROS description's nominal vendor
FK/IK/constants, not fitted physical dimensions. Versioned, atomic/fsynced
persistence defaults to `$XDG_DATA_HOME/RRsZ70_ROS2/arm-estimate.json` (fallback
`~/.local/share`). Startup without a session is UNKNOWN; restored state is history
only until an anchor/stow reconciliation. The old private TS session file is not
read or migrated automatically and remains untouched.

Replay covers deploy/stow, jog direction/limits, nominal hand IK, programs,
uncertainty, restart, rejected/unsent/unknown outcomes, logical duplicate filtering,
stale status, mode exit and official-app visibility limits. A sanitized fixture
contains 78 retained command/status records with original-source SHA-256 and
provenance; device identifiers/images/arbitrary results were excluded.

Corrections to unsupported legacy semantics are explicit: grip is NOT treated as
an auto-lift; unsupported post-grip/program pose remains unknown. Unknown Advanced
actuator effects conservatively invalidate estimation without rejecting/replaying
the RPC. An unchanged official-app mode cannot reveal an invisible joint jog.
Mode exit is not completed stow. Heuristic sigma is not measured covariance.

Only usable M3/M4/M5 command-derived joints enter `/joint_states`; invalid/restored
state publishes no fake planning joints. Folded/finger placeholders use
`/z70/display/joint_states` and a `display/`-prefixed RSP tree. Stable republication
retains the original joint header/evidence identity so late RSP subscribers can
initialize without fabricated fresh poses. TF history can remain after an
invalidation: consumers must check typed validity, never TF presence alone.

## Calibration resources

Description owns installed front JSON/CameraInfo, floor geometry code and existing
mount/pincher/gripper configuration. Odometry owns installed historical fit seeds
and durable mutable online state. Manipulation uses installed resources and typed
calibration gains; executor ancestor walks, checkout online.json reads and the
hard-coded held-verification camera offset are removed. Bringup no longer directs
online writes into the checkout.

The original archive is preserved, not replaced by runtime package copies.
Parent verification checked nine source/installed asset SHA-256 pairs and confirmed
non-symlink module origins/default projection outside the checkout cwd. Existing
mutable archive online.json retains SHA-256
`3df23b7849c31560918ebbccb1c19274d14805a4ff29e246672d5376e01d03f2`.
For schema, provenance, explicit no-clobber import and fit uncertainty semantics,
see [phase3-resources.md](phase3-resources.md). No calibration value or historical
optical tolerance was widened. The earlier phase-2 ten installed-resource failures
no longer occur. Isolated build/install/log prefixes are git-ignored.

## Verification actually executed

| Check | Result | Retained evidence |
|---|---|---|
| Fresh non-symlink colcon | All nine packages built; no missing-LC_RPATH diagnostics in the fresh build | phase3-clean-build-final.txt |
| Full installed ROS suite, latest source | 179 passed, 3 existing optical xfailed, no failures | phase3-ros-final.txt |
| TypeScript suite | 358 passed, no failures | phase3-npm-test.txt |
| TS noEmit, web typecheck and Vite build | Passed | phase3-npm-build.txt; explicit npx tsc also exited 0 |
| Installed origins/default projection/hash checks | Nine assets verified; no source checkout dependency for runtime defaults | phase3-installed-verification.txt; phase3-verification.json |
| Observation-interface generator preservation | All 27 interface/build files byte-identical in isolated copy | phase3-generator.txt |
| Whitespace | git diff --check passed | terminal execution |

Real DDS fake-backend tests cover conflicts, action results/event identities,
unknown/vendor-rejected outcomes, actual HTTP disconnect, no motion replay,
priority stop/cancel/takeover, all-stop reply deadlines, task revocation, cloud
selection, stalled images and independent pickup watchdog. Actual RSP tests cover
nominal/physical optical chain consistency, display/planning tree separation and
invalid-state handling. The existing provisional optical-model expected failures
remain at the historical 12px tolerance: these are not new physical validations.
Standard `ros2 action list -t`, `ros2 topic list -t --no-daemon --spin-time 1`, and
`ros2 param get` were exercised against the isolated fake graph. No custom doctor,
launcher or diagnostic agent was added.

Experiments/failures are not hidden: an importlib pytest mode did not suit existing
cross-test imports; the usual collection mode was used. Default ROS CLI daemon
caching initially missed the ephemeral fake graph; explicit no-daemon discovery
fixed that test. An existing MCP-reload timing test failed in a concurrent heavy
run and passed in the final sequential full suite without modifying that test.
Its failed output is retained in phase3-npm-concurrent-flake.txt. Initial TF testing
also exposed late-RSP initialization: stable samples now republish with their
original timestamps, verified against real RSP. Only owned validation processes
were stopped; the unrelated model preview publisher was left running. The
validation-created domain-92 CLI daemon was stopped and its PID disappearance
confirmed.

## Phase-4 handoff / deployment limits

Existing child `t_6d59fe24` owns ROS-aware MCP/web clients, shared task handles,
feedback/result/cancellation and web input flows. It must not restore direct
backend writes or instantiate a gateway estimator. Read
[phase3-command-api.md](phase3-command-api.md) and [interfaces.md](interfaces.md).
Task UUIDs are internal dispatcher ownership IDs; standard Pick goal IDs/results
are the public action boundary. Driver-local outcomes with no vendor attempt
must not be shown as physical completion or a received vendor reply.

Do not deploy this partial client cutover over an active controller. The running
production stack was not changed. Live camera throughput, pickup repeatability,
physical cessation, encoder feedback, continuous trajectories and verified
localization are NOT established by these tests. Live deployment/qualification
still requires separate authorization. Root layout/checkout relocation and unused
legacy package cleanup remain their existing downstream phases.

Collision hotspot: executor.py and test_executor.py include inherited phase-2 and
concurrent geometry edits. Changes were targeted; do not commit the whole dirty
workspace as if it belonged to phase 3. No commit or staging operation was made.
