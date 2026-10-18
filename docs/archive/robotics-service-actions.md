> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

The registry/checkpoint and lease/command-ledger schema below is retired; current task ownership is ROS. Retain synthetic validation scope, not a permanent queue-exclusion or lifetime-ledger requirement.

## Historical record

# Service integration: durable bounded actions

`src/robotics-actions.ts` implements the production schema in `robotics-actions.md`. It is a registry, not an authority or HTTP server. There is no robot I/O in this module.

## Wiring

```ts
const actions = await RoboticsActions.create({
  journal: await roboticsActionJournal(privateStateDirectory),
  binding: () => connectedBinding, // null when disconnected
  acquire: claimRoboticsAuthority,
  execute: executeThroughSharedAuthority,
  cancel: stopThroughSharedAuthority,
  admit: policyAdmission, // optional; false rejects before issuance
  now: Date.now,          // optional
  maxPending: 1,         // optional motion limit; STOP has a separate slot
  maxRecords: 10_000,    // optional permanent history limit, no eviction
});
```

The private journal is `robotics-actions.json`, separate from `command-authority.json`, using owner-private regular files, exclusive temporary creation, atomic rename, file fsync and directory fsync. Injected `load/save` must provide equivalent durable semantics and snapshot isolation. There must be exactly one process/registry owner for a journal. Creation validates the complete checkpoint and saves restart conversion before becoming usable. Never replace an unreadable checkpoint with an empty one.

### Exact callback interfaces

```ts
type RoboticsBoundGoal = RoboticsActionGoal & {
  binding: { deviceId: string; session: number };
};
type RoboticsWriterLease = {
  id: string; writer: "robotics"; sessionId: string; expiresAt: number;
};
acquire?: (goal: RoboticsBoundGoal, scopeId: string)
  => Promise<RoboticsWriterLease | null>;
execute: (goal: RoboticsBoundGoal, scopeId: string,
  onLease: (lease: RoboticsWriterLease) => Promise<void>)
  => Promise<RoboticsActionResult | "rejected" | "failed" | "unknown">;
cancel: /* same signature as execute */;
type RoboticsActionResult = {
  vendorAck: boolean; verifiedSuccess: boolean; cessationVerified: boolean;
  state?: "succeeded" | "rejected" | "failed" | "unknown" | "cancelled";
};
```

`acquire` claims actual shared service authority **without issuing transport commands**. The registry-generated `scopeId` is an opaque workflow correlation key. `service.ts` stores its authenticated ROS scope under this key and calls `CommandAuthority.claim(scope, binding)`; returned metadata belongs to that actual authority lease. STOP returns null from acquisition and initially responds `unknown`, never lease-less `accepted`/`running`.

The service owns acquisition, renewal and cleanup/release, including scopes whose registry admission was cancelled or fenced after acquisition. The registry's `release(scopeId)` callback handles partial claims and execution/cancellation cleanup; it waits for callbacks using that scope before releasing. `close()` immediately fences admission/issuance, sends no automatic robot traffic, waits for existing callbacks and checkpoints unresolved outcomes. Unresolved acquisitions occupy a pending slot, while cancellation and STOP remain independent.

`execute` and `cancel` **must** route writes through the shared `CommandAuthority`, and recheck the original `goal.binding`, real scope epoch and writer ownership at the final authority dispatch boundary. The registry repeats local binding/lease checks around every issuance checkpoint, but cannot protect continuations inside an injected callback. `cancel` is a bound independent STOP through the same authority, not a replay of the original operation. STOP and cancel must fence other service writers/continuations through that authority too. Never wait for the original execute transport promise before cancellation.

Await `onLease(realLease)` when reporting ownership renewals. It checkpoints metadata and rejects expired leases, ownership substitution, decreasing expiry, stale binding and terminal updates. A callback can report vendor ACK but must leave physical flags false unless it has correlated fresh physical evidence. Missing/invalid results and thrown/ambiguous transport outcomes become `unknown`. No elapsed pulse-time success inference exists. STOP success and cancellation require cessation evidence. No client evidence endpoint exists.

## HTTP handler mapping

Read `X-Roboclaw-Device-Id` and `X-Roboclaw-Device-Session` into a validated binding; require authentication and the existing service origin/Host policy before registry calls. Do not derive request binding from the selected robot. Session headers must be canonically parsed nonnegative JS-safe integers.

* `POST /api/robotics/v1/actions`: `await actions.submit(parsedBody, requestBinding)`; exact body `{commandId,operation,arguments}` only.
* `GET /api/robotics/v1/actions/{commandId}`: `await actions.status(commandId, requestBinding)`.
* `POST /api/robotics/v1/actions/{commandId}/cancel`: `await actions.cancel(commandId, requestBinding, parsedBody)`; exact body `{}` only.

Serialize returned status directly with `Content-Type: application/json`. Admission normally returns 202; a definitive `rejected` record can use a success HTTP code. Preserve all fixed envelope/evidence fields, historical leases and original binding. `accepted` is the admission snapshot, not a promise that execution remains active when the HTTP response arrives; polling observes subsequent transitions.

`RoboticsActionError.code` mapping: `INVALID` → 400, `BINDING` → 409, `DUPLICATE` → 409, `NOT_FOUND` → 404, `PENDING` → 409, `UNAVAILABLE` → 503. Do not invent a status record for an error. During acquisition, status returns `PENDING` rather than publishing fake active ownership. Duplicate IDs always reject, including across devices/sessions and restarts; clients should GET rather than resubmit.

Admission durably consumes IDs before admission/acquisition/executor callbacks. Saturation and policy refusals also retain IDs. The journal intentionally stores only status/authority metadata and internal phase, not operation arguments. Restart turns accepted/running/cancel-requested or otherwise incomplete leased records into `unknown`; unleased pre-admission reservations become `rejected` because they never reached a permitted motion dispatch. Nothing is replayed. Cancellation during pre-admission settles `rejected`, with no cancellation transport required; cancellation during issuance waits only for the checkpoint barrier and fences the original send. Each dispatched cancellation is attempted once and late original results cannot replace its outcome.

Checkpoint failures poison the instance and prevent further registry issuance/status claims. Existing service-level independent STOP remains the integration's emergency path if this journal becomes unavailable. Full permanent history similarly prevents further registry admission; configure sufficient capacity and never prune IDs to enable replay. Capabilities should advertise commands only once both the registry and genuine authority integration are available.

## Validation

`node --test src/robotics-actions.test.ts` covers strict schema, checkpoint ordering, restart/no replay, actual lease requirements, acquisition/issuance cancellation races, independent STOP, at-most-once cancellation, late outcomes, device/session fences, renewal/substitution, persistence failures, history limits and private filesystem checkpoints. All executors are synthetic; no physical robot is contacted.
