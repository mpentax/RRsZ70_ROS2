> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

The stow-before-dock fault/recovery contract explains old trials only. These routes/browser attestations are retired; this page provides no current docking procedure.

## Historical record

# Supervised staged docking (offline implementation)

## Browser route contract — UI update required

All POST routes require browser cookies and existing exact Origin/CSRF checks. No bearer physical attestations. GET `/api/docking` returns the snapshot only: workflowId, stage, postStowAt, expiresAt, acknowledged, physicalDockVerified, actionOutcome, motorStopConfirmed:false, automaticRetryAllowed:false.

1. POST `/api/docking/start` `{}` reserves before any await, requires both device-bound permissions and fresh manual/waiting arm feedback without blockers, records durable pending marker, and uses existing validated ArmController stow. Returns WAITING_PHYSICAL_CONFIRMATION after ACK/post-command observation; no joint pose is invented.
2. Physically inspect folded arm after stow. POST `/api/docking/confirm` `{workflowId,postStowAt,physicalStowed:true}` binds owning cookie/device/LAN session/post-stow timestamp. Workflow expires at 120 seconds; freshness of physical verification starts at confirmation receipt, with 30 seconds to guarded mode exit, fresh native readiness and a single docking dispatch. ACKNOWLEDGED retains reservation and does not prove arrival.
3. Observe arrival, actual stationary robot and folded arm. POST `/api/docking/recovery` `{}` obtains fresh read-only native docked state 8, no cleaning/return/error/lock, arm off/not-started and no inflight local writers. Returns `recoveryToken`, `recoveryObservedAt`, and snapshot. Token binds current cookie/device/LAN session and expires after 30 seconds.
4. After recovery read and physical inspection, POST `/api/docking/finish` `{workflowId,recoveryToken,physicallyDockedStationaryAndStowed:true}`. Rechecks binding and corroboration, clears durable marker, returns PHYSICALLY_VERIFIED. This is operator observation, not calibrated arrival proof. **Older UI bare physicallyStoppedAndStowed payload is deliberately unsupported: add recovery call/token and docked/stationary attestation.**
5. POST `/api/docking/cancel` `{workflowId}` cancels later steps, not cessation. Cancellation, expiry, signout, unknown dispatch and docking ACK preserve post-stow quarantine. Independent STOP remains available.

Bearer `/api/agent/chassis/dock` returns 409 with current snapshot, browserWorkflowRequired and start/confirm endpoint links, without commands.

## All-writer boundary

Admission blocks every conflicting non-GET route, including device/account changes, camera, browser arm/chassis and MCP. Start rejects already admitted overlapping mutating requests and background setup/camera activity. Robot proxy guards every mutating RPC/supervised call synchronously at dispatch; only workflow-context commands and independent stop/end pass. Stops invalidate later steps. Existing active arm/chassis controllers prevent reservation acquisition. Robotics bounded observation rejects while reserved. Ordinary read-only telemetry does not reconcile uncertain motion. External vendor-app writers cannot be locked: operator supervision must exclude them.

## Durable recovery

Private existing state directory is inspected for owner, permissions and symlinks. Exclusive-create mode-0600 `supervised-docking.pending` marker is fsynced before potential dispatch. Existing marker is preserved; restart re-enters UNKNOWN before robot creation, never replaying. Unavailable/unsafe journal also enters failclosed quarantine. No credentials/payloads are stored. Cancel, signout, ACK and restart never clear marker. Recovered workflow ID is null: fresh authenticated operator obtains current-device/session recovery challenge before physical finish. Recovery requires docked inactive native status; cessation away from dock cannot clear quarantine here. Physical finish clearing failures re-create marker when possible and retain UNKNOWN; filesystem durability failures remain explicit safety blockers.

Preflight rejection before marker/dispatch is REJECTED; persistence or potential command uncertainty is UNKNOWN. No live calls, automatic retries, fabricated locks or joint-zero stow inference.

## Follow-up review fixes

- Legacy robot-reading GET routes return 409 while reserved; snapshot-only docking and robotics capabilities remain readable. Robotics state admits no RPC while reserved. Already admitted GET handlers are counted until settlement; start rejects their occupancy and `observationActive(robot)` even after a robotics caller deadline. This is conservative admission, not silently relabeling queued legacy reads as connected/bounded observations.
- Background camera retries are deferred without consuming bounded retry attempts; cameraExclusive and monitoring enforce reservation directly.
- Independent STOP permits validated zero-velocity braking, RC stop/end, and permission-checked native task cancellation. STOP during quarantine attempts native cancel in addition to RC stop; ACK still does not prove cessation. Browser cancel-task remains permission/confirmation protected and accessible during reservation.
- Recovery challenge permits **explicit authenticated browser takeover** after local writers quiesce and fresh docked inactive status corroborates. This authorizes physical settlement only, never resuming/replaying dock or bypassing original-owner stow confirmation. Reauthentication/cookie loss does not require restart. Challenge/finish bind the takeover cookie, workflow ID, current device/session and 30-second window.
- Workflow dispatch privilege is revoked synchronously at the robot proxy when revision/cancellation/session/deadline changes, even if a controller is still awaiting its own pre-send observations.
- Durable clear is followed by another revision check. Clearing failure or cancellation race re-latches UNKNOWN and attempts to recreate/fsync the pending marker. If the filesystem cannot persist either clearing or re-latching, restart crash consistency cannot be guaranteed; this remains an explicit storage failure safety blocker, not proof of successful durable settlement.

## Offline validation

`npm run typecheck` passed (source + web). Final merged `npm test` passed **526 tests, 0 failures** (23.8 seconds). Eight dedicated coordinator/journal tests cover binding, no replay, predispatch rejection, timeout uncertainty, cancellation/expiry, restart marker recovery, stale recovery/session changes, persistence/clearing faults and cancellation during durable clear. Service fake tests cover cookies/Origin, every conflicting route, legacy GET exclusion, already admitted GET occupancy, one-shot docking, takeover challenge/finish, native/RC independent STOP, background restore deferral and unavailable-journal startup quarantine. No real service/robot requests, movement, restart, permission changes, commit or push.

## MCP guarded start (2026-10-01)

`start_supervised_docking` sends one empty POST to `/api/agent/docking/start`.
It shares the browser start admission, durable journal and robot-proxy writer reservation;
it does **not** accept `physicalStowed`, perform mode exit, or dispatch navigation.
The returned workflow waits for a fresh authenticated browser operator to inspect the arm,
then POST the returned `workflowId` and `postStowAt` with `physicalStowed:true` to
`/api/docking/confirm` using the existing cookie/Origin protections. That browser claims
ownership atomically before mode exit; another operator cannot subsequently confirm it.
The existing UI can use the service snapshot to complete this workflow. MCP callers must
not supply or synthesize the physical confirmation. Existing `return_to_dock` remains a
non-actuating compatibility rejection; use the new staged tool.

If feedback reports vendor remote-pickup mode (mode 2), start returns `REJECTED` with
`guidance: EXIT_REMOTE_PICKUP_IN_APP`, before persistence or stow. **Exit remote pickup
in the vendor app under nearby supervision, then freshly establish supported idle manual
arm readiness.** There is no supported remote-pickup mode-exit command implemented or
inferred here. `MANUAL_STOW_REQUIRED` likewise requires the supported manual workflow;
neither an ACK nor all-zero joints permits proceeding. Never automatically retry a rejected
or uncertain workflow. After any dispatched stow, cancellation/expiry/ACK retains quarantine
until the existing browser recovery challenge and physical docked/stationary/stowed finish.

This addition was validated offline only; no deployment or physical docking validation.
