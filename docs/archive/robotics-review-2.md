# Second independent review — supervised docking / ROS actions

2026-09-30, evolving working tree. Owned only this report. No robot calls, live observations, builds, restarts, or commits. Line references describe the inspected snapshot; parent service fault-case fixes may supersede findings.

## Remaining concrete findings

### P1 — Journal inspection failure blocks docking start, not other writers

`src/service.ts:276-277,314,819-821`: an unreadable/unsafe state directory or failed marker inspection sets journal=null but leaves docking IDLE/unreserved. Only `/api/docking/start` checks journal. Existing browser/MCP arm/chassis writers remain available, even when the failure prevents determining whether a previous dock dispatch is unresolved. The comment says fail closed, but only the capability fails closed.

Required: journal inspection uncertainty must create a global unresolved-writer quarantine (with independent STOP available), or refuse service startup. Distinguish a positively absent marker from an uninspectable marker. Test startup lstat EACCES/EIO and unsafe directories against every motion writer, not just docking start.

### P1 — STOP fence blocks zero velocity; native navigation cancel is excluded

`src/service.ts:403-408` allows only chassis-stop/chassis-end as independent stops. `src/chassis-session.ts:268-276` sends zero velocity, stop and end together; zero velocity is rejected by the docking fence, guaranteeing that forced browser STOP reports failure during a reservation even when the other two ACK. Transport priority already recognizes zero velocity, but service admission does not.

Furthermore native docking is now genuinely dispatched (`service.ts:308-311`), while STOP routes (`981-994,1096-1098`) only use RC stop/end. The task-cancellation route sends cancel-cleaning but is blocked by the reservation (`831-833,1103-1106`). This does not prove RC stop cannot stop navigation, but the implementation provides no evidence that it can. The UI says STOP remains accessible; availability alone is weaker than appropriate operation coverage.

Required: allow bounded zero-velocity braking under the independent-stop fence; preserve a permission-safe native-task cancel path while dock is pending/ACKed/unknown. Test actual emitted command kinds with fake transports during each stage, and do not claim physical cessation from their ACKs.

### P1 — Legacy GET observations still bypass admission during docking

`src/service.ts:765,770-772,791-792,571-574` execute state/map observations before the reservation fence. `robotTelemetry` (`153-177`) performs three simultaneous ordinary requests; `currentState` can add arm status reads. Ordinary `request` can queue/reconnect, unlike hardened robotics `requestConnected`. Concurrent GET /api/state or /api/chassis callers can fill the four-slot pool and queue more work; `robot.ts` rejects nonpriority supervised commands behind reads. Thus the new robotics read hardening does not protect docking from normal UI refresh/API observers. Its admit callback (`760-762`) also does not directly include docking.reserved; after arm mode exit, controller idle conditions may permit observation during readiness/dock.

Required: shared observation admission must cover all robot-reading GETs and docking reservation, with no-reconnect bounded connected reads, coalescing and accounting until settlement. Snapshot-only GET /api/docking is acceptable. Test multiple browser refreshes racing confirm, readiness and dock dispatch.

### P1 — Reservation recovery can be stranded by ordinary cookie reauthentication

`src/supervised-docking.ts:67-69` restricts recovery to the original owner while workflowId is non-null. `src/service-auth.ts:88-94` replaces a cookie's session identity on login; expiry/logout also destroys the old identity. The journal persists only a marker, not recoverable owner credentials. Once a started workflow outlives its cookie or the operator signs out/relogs in, the new authenticated browser cannot challenge/finish it. Restart happens to change id to null and permit takeover, but restart is not an acceptable recovery prerequisite.

`service.ts:743-745` also permits a session DELETE to invalidate a docking revision regardless of whether that session owns docking; this is before require. Origin protects cross-site use but not another local same-origin client.

Required: define explicit authenticated physical recovery takeover for orphaned/expired ownership without allowing replay or automatic clearance. Bind normal cancellation to owning identity; independent STOP may intentionally invalidate any workflow. Test logout/login, cookie expiry, second browser and restart recovery.

### P1 — Journal clear has a cancellation race and unlink/fsync uncertainty gap

`src/supervised-docking.ts:83-89`: finish validates revision before awaiting clearPending, then unconditionally sets PHYSICALLY_VERIFIED. STOP/logout/cancel can increment revision while clearPending is awaiting; finish then overwrites UNKNOWN and releases the reservation. Add a post-clear revision/binding check and specify safe behavior if clearing races cancellation.

`117-120`: clear unlinks first, then opens/fsyncs the directory. If directory open/sync fails after unlink, finish remains reserved in memory but the marker is absent; subsequent restart may permit writers. A crash between unlink and directory sync has inherently uncertain deletion durability; current code offers no persistent conservative state after an acknowledged clear failure. Required: carefully define crash-consistent pending/cleared records and failure handling; do not equate a failed clear with a durably retained marker. Inject unlink-success/open-failure/fsync-failure and cancellation during clear tests. Parent is actively fixing fault cases: recheck finalized implementation.

### P2 — Browser release contract does not match the implemented recovery handshake

`web/src/docking.ts:46-51` posts only workflowId and `physicallyStoppedAndStowed:true` to finish. Service requires a `/recovery` challenge, recoveryToken and `physicallyDockedStationaryAndStowed:true` (`service.ts:809-816`, supervised-docking.ts:75-80). There is no browser challenge helper. Therefore the release button always fails with 409, leaving ACKed/cancelled/unknown reservations held. Current UI text also allows stationary/stowed confirmation with arrival optional; backend requires docked inactive status plus arrival assertion.

Required: implement the two-step challenge/token flow, ensuring physical inspection occurs after the fresh challenge and consent is specifically docked+stationary+stowed. Test browser helpers against the actual service handler, not only mocked request fields. Preserve unknown latch until successful recovery and no automatic retry.

### P2 — Background owner is not fully covered by the reservation

`src/service.ts:490-526,1259-1264`: startup/retry/checkCameraSource can invoke driver.configure under cameraExclusive after docking reservation begins, since restore/checkCameraSource/cameraExclusive do not check reserved. Dock start only checks current cameraBusy/monitoringCamera, not future timer work. The robot proxy fences direct RPC writers but not go2rtc sidecar configuration. This is a fence completeness gap, not proof configure itself moves a robot or starts video.

Required: suppress/defer owner restoration and camera monitoring that can lead to configuration for the full reservation, including recovery; ensure previously admitted restoration is quiescent before start. Test a sidecar loss and restore retry timer during WAITING/DOCKING/ACKNOWLEDGED.

### P2 — ROS independent STOP can be starved by action executor occupancy

`robotics/ros2/src/roboclaw_actions/roboclaw_actions/node.py:40-59,75`: each execute synchronously occupies an executor thread with sleeps and blocking HTTP calls for up to 120 seconds. Four concurrent nonterminal goals can occupy all four executor threads, preventing a fifth STOP goal or cancellation callback from running until one exits. No admission limit protects a reserved stop thread. Client also renegotiates capabilities for every STOP; an unavailable capability fetch prevents its POST.

Required before REAL integration: separate stop/cancel scheduling from blocking motion waiters, use asynchronous/nonblocking polling or explicit motion concurrency admission plus a dedicated independent stop path. Offline executor-level test must hold four fake goals running and prove STOP/cancel latency bounded. Current ordinary two-goal simulation does not establish this guarantee.

### P2 — ROS deadline, evidence and fake terminal state contracts need stricter tests

Action execute checks deadline only around the loop, not after blocking submit/poll/cancel. A 100ms observation deadline can spend two one-second calls submitting; a poll begun before deadline can return succeeded after it and be accepted. This is bounded overshoot, but contradicts a strict timeout interpretation. Specify observation-budget semantics and check the deadline after each I/O; reduce per-request budget to remaining time where appropriate.

`roboclaw_actions/client.py:validate_status` accepts succeeded with verifiedSuccess=true and cessationVerified=false, including STOP. The documentation says stop ACK alone is not cessation and a trustworthy backend is required; the adapter should explicitly require cessationVerified for successful stop, or clearly distinguish verified stop semantics. It trusts backend boolean evidence without timestamp/provenance; REAL capability negotiation is necessary but not proof of trusted physical verification.

`fake_server.py:do_GET` re-writes any state except rejected/cancelled after 150ms, including unknown. Therefore an ACK-only terminal unknown can later become succeeded if scenario changes, rather than remaining terminal. Tighten fixture terminal immutability. Goal IDs/client IDs are process-local and unbounded; backend durable dedup remains an explicit production requirement, not implemented here.

## Acceptable boundaries / verified improvements

- Supervised docking now does real staged stow, explicit browser confirmation, exit/readiness and one dock dispatch, with device/session/revision checks. Agent booleans cannot invoke physical confirmation; cookie+Origin required.
- Journal creation uses exclusive no-follow private file, file fsync and directory fsync BEFORE stow. Marker survives ACK, cancel, expiry and restart; ACK does not release the reservation. Owner/session expiry cases above remain gaps.
- Account switching and other POST mutations are fenced; start admission tracks other in-progress mutating requests before reservation. Robot proxy fences direct writers outside docking async context, including background contexts. Account-switch race needs to remain tested with pending pre-admitted work.
- Hardened robotics state uses sequential connected reads, numeric status sanitization, per-robot coalescing, caller deadline with retained outstanding accounting. First-review string-value and no-reconnect issues are corrected for this endpoint; legacy GET paths are not.
- GET docking is snapshot-only. Browser actions do not automatically confirm/retry; cancellation never claims motor stop. Docking UI remains separate from STOP button.
- ROS defaults DISABLED. REAL refuses the simulation backend and current production read-only capabilities. Action schema intentionally differs from the production read-only v1 envelope; thus production actions are unavailable, not silently mapped to legacy routes. Both can coexist only with an explicitly designed capability contract.
- ROS simulation verification is explicitly synthetic. No JointState/odometry/tf or live command backend is fabricated. IDs are reserved before dispatch, ambiguous submit is not retried, cancel is sent at most once, unknown abort does not claim stopped motion. No real-mode production readiness is established.

## Offline evidence

- Node targeted suite: `node --test src/supervised-docking.test.ts web/tests/docking.test.ts src/robotics-api.test.ts`: 12 passed, 0 failed.
- Python with explicit package paths: `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=robotics/ros2/src/roboclaw_actions:robotics/ros2/src/roboclaw_readonly python3 -m unittest discover -s robotics/tests -v`: 18 passed, 0 failed. Initial command without PYTHONPATH could not import action package; corrected invocation passed. Only synthetic localhost tests, no application/robot connection.
- No container build or ROS runtime tests were run concurrently with the ROS worker. Report does not independently attest the parent's 517-pass/full-build claim.

Passing targeted tests do not currently establish journal fault containment, end-to-end browser recovery, legacy GET admission, or executor saturation STOP availability. Re-review the parent/worker's final changes before treating any evolving finding as resolved.
