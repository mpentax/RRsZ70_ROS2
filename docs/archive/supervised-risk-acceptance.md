> Historical/superseded architecture. Current contract: [thin-api.md](thin-api.md).
> The service-owned admission, recovery and workflow policies described below have been removed.

# Single-use supervised risk acceptance (review before deployment)

Measured service reconciliation remains the default. Stock unsupported queue exclusion
is not permanent prohibition of explicitly supervised work: the separate policy below
accepts risk without claiming cessation or queue-drained measurements.

## Agent/service protocol (no dashboard prerequisite)

1. Authenticated `POST /api/commands/supervised-challenge` with exactly
   `{action: "arm_joint_step", arguments: {control: "m3-left"}}` (MCP:
   `supervised_action_challenge`). Action names/arguments use existing bounded MCP schemas.
2. Read back device/session, authority revision, exact unresolved command IDs, action
   digest and 30-second expiry. Obtain explicit operator attestation of **presence,
   clear area, observed cessation, and readiness to physically power off**. Never infer
   these attestations from telemetry, an old message, or an unattended script.
3. Invoke the exact existing action with its original arguments plus
   `supervisedPermit: {challengeId, attestation: {operatorPresent: true, areaClear: true,
   cessationObserved: true, physicalPoweroffReady: true}}`. MCP action tools accept this
   optional envelope. The agent relays the operator's explicit attestation, not evidence.
4. Response identifies `admissionBasis: supervised-risk-acceptance` and
   `queueExclusionVerified: false`, including admitted errors. HTTP/ACK is not success.
   Observe physically. Never replay or automatically retry an expired/consumed attempt.

Challenges are in-memory, owner/device/session/revision/exact-history/action bound.
A new challenge supersedes the previous one. Every attempt reaching authority acceptance
consumes the current challenge, including wrong ID/digest/attestation. Malformed MCP
schema or HTTP envelope/arguments rejected before authority acceptance do not consume
it; this distinction is not permission to retry motion automatically.
STOP/session change, conflicting authority ownership/history and expiry invalidate it.
Audit decisions are fsynced in the existing private checkpoint **before admission**.
Old acknowledged records become physically unknown; latches and consumed IDs remain.
Only that action's finite service-owned workflow can ignore that exact old history;
new uncertainty still blocks it. Revision/binding/expiry are rechecked after both acceptance checkpoints and before dispatch;
independent STOP remains available. Restart discards all exemptions/challenges and retains
audit/history. The authority journal capacity and error quarantine still apply.

This does not enable raw RPCs, change persisted motion permissions, prove arm-stop,
or override vendor mode/readiness/docking checks. Finite chassis pulse and arm workflows
may issue multiple internal bounded RPCs for their *one* exact action. Mutable mechanical
camera selection is honestly treated as authority-controlled side effect, not motion
completion; camera stream hold is excluded (unbounded sidecar lifecycle). STOP and
supervised docking are excluded because they have independent protocols. No camera,
robot motion, permission edit, journal deletion or service restart was performed in testing.

## Architectural direction, not implemented blanket deregulation

Long-term RoboClaw should be an authenticated Roborock API wrapper with protocol and
transport validation, and DimOS should own behavior/mechanical risk policy. This slice
adds only explicitly authorized supervised permits; it does **not** wholesale remove
current service restrictions. DimOS z70 actuation configuration currently rejects `true`
because its existing policy lacks an independent STOP path. This is a candidate adapter
limitation, not a prerequisite imposed on the explicitly authorized supervised
single-action residual-risk policy. Integrate the exact challenge/attestation/action
interface without simply flipping broad actuation on. Bound priority chassis STOP/cancel
and honest arm-stop capability remain valuable follow-up improvements, not a claimed
verified capability of this slice. A host-only operator-attestation directory must not
silently recreate a dashboard/file prerequisite: relay fresh explicit operator consent
through a supported authenticated path, never synthesize operator booleans.

## Deployment gate

Parent independently reviews dirty-checkout diff, authority exemptions, checkpoint
failure/STOP races, MCP schemas, tests and physical limitations. Run `npm test` and
`npm run build` on the reviewed tree. Preserve private journals and existing permissions.
Only after fresh operator presence/clear-area/stationary/stowed readiness and poweroff
availability should the parent schedule the existing deployment procedure/service restart.
After restart, first perform read-only state/reconciliation; unsupported is expected on
stock firmware. Use a fresh explicit challenge only when the operator authorizes one
specific bounded action. This document is not authorization to deploy or move hardware.
