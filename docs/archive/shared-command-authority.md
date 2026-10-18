> Historical/superseded architecture. Current contract: [thin-api.md](thin-api.md).
> The service-owned admission, recovery and workflow policies described below have been removed.

# Shared service command authority

The service now owns one `CommandAuthority` across browser, MCP and ROS robot writers.
The robot proxy admits all supervised writes and mutable camera RPCs through it.
Owners come from authenticated browser sessions or the existing bearer identity;
the broad bearer represents one MCP writer, not individually identified MCP clients.
ROS bounded actions claim actual leases from this same instance and retain durable
correlated goal outcomes. The observation API stays backward compatible.

## Dispatch and recovery

- A private, atomic, fsynced `command-authority.json` checkpoint is written before
  transport issuance. It contains IDs and phases, never command payloads, map targets,
  credentials, or camera images. Restore preserves consumed IDs and converts in-flight
  and acknowledged operations into uncertainty. Checkpoints have a 16 MiB bound;
  reaching it blocks new actions rather than silently pruning history.
- Trusted service workflows may continue after earlier RPCs acknowledge, using the
  same exclusive lease. This supports RC startup/movement and deploy mode transitions;
  it does not expose continuation authorization through HTTP or MCP. Workflow ACKs
  remain physically unresolved. Ambiguous transport outcomes block continuation.
- Finite requests release their lease on settlement and retain unresolved uncertainty. Browser
  RC retains ownership across its heartbeat session. Docking retains its service-owned
  scope between stow and operator confirmation. Command leases are bounded to 60 seconds.
- Independent zero velocity, RC stop/end, and native task cancellation can dispatch
  without action ownership. End/explicit STOP fences continuation; ordinary browser
  braking preserves the owning RC workflow. Stop does not wait for action ACK, but
  checkpoint work is serialized and filesystem stalls can still delay issuance.
  When journal writes fail, independent STOP is attempted while new actions stay blocked.
- Camera lifecycle operations exclude active command ownership; robot writers also
  reject conflicting camera lifecycle work. Sidecar media effects themselves are not
  represented as physical motion completion in the command ledger.
- Correlated RPC method-not-found is a recognized nonexecution case. The old
  mechanical-camera settings fallback was removed after the primary Android trace
  showed no selector equivalence. Timeout, generic error and malformed ACK never
  authorize another write.

`/api/state.commandAuthority` exposes bounded ownership/counts and recovery status,
including through MCP `get_state`. Recovery is service-owned, not browser-owned.
Both authenticated bearer agents and same-origin dashboard users call
`POST /api/commands/recovery {}`; MCP exposes `reconcile_commands`.
The dashboard is an optional client, never a prerequisite for shared command recovery.
`POST /api/commands/confirm` is retired (409): a physical-observation checkbox cannot
certify absence of delayed commands and no longer clears uncertainty.

## Evidence and outcomes

The service first checks local quiescence and performs one bounded already-connected
`get_status` read, without transport reconnect, motion, replay or permission changes.
Concurrent recovery is refused; motion claims/dispatches are blocked during evidence
acquisition. Independent STOP remains admitted and invalidates the recovery result.
The status check is corroboration only, NOT terminal/controller evidence.

Responses include `outcome: resolved | unresolved | unsupported`, a stable `reason`,
`queueExclusionVerified`, `dashboardRequired: false`, and public authority counts.
Stock Z70 has no controller-terminal/queue-drained-including-delayed certificate and
no independent calibrated cessation adapter. Quiescent stock status therefore returns
`unsupported / CONTROLLER_QUEUE_EXCLUSION_UNAVAILABLE`, leaving the ledger intact.
Charging, docked100, ACK, zero joint arrays and a human's stationary observation do
not establish absence of future delayed execution. The separate [single-use supervised risk policy](supervised-risk-acceptance.md) may admit
one explicitly attested bounded action, without disguising acceptance as verified evidence.

`measuredCompletionSource` is the existing validated independent measurements contract.
It can also reconcile orphaned/restarted history against the exact unresolved command
IDs and a fresh current-session capture after recovery starts. Previous-device work
cannot be released from current-device evidence. The trusted in-process
`recoveryObserver` seam has the same cessation/controller guarantee requirement; it
is for real service adapters and network-free synthetic tests, not HTTP assertions.
Malformed/stale/wrong-ID/operator evidence returns unresolved. Observer reads are
bounded to ten seconds and retain admission if the underlying adapter remains active.
Journal checkpoint failure quarantines authority. IDs remain consumed; no replay occurs.
Recovery verifies safe release, not success of an old grasp/navigation task.

Human observations may be passed conversationally through an authorized agent as
context, but cannot become queue exclusion guarantees. The separately authorized supervised risk-acceptance policy preserves unresolved history
and measured recovery defaults; it is never autonomous queue-exclusion evidence.
This change does not alter docking's separate supervised reservation/stow protocol or
its physical-stow authority; it is not proof of unattended docking readiness.

Verification: `src/command-authority.test.ts` and production `src/service.test.ts`
with fake connected hardware and private temporary state, plus the full suite/build.
No physical robot validation, robot RPC fallback, permission edits or service restart.
Deploy only after independent review and current stationary/stowed/supervised readiness
checks. Existing ambiguity may remain unsupported after deployment, by design.
