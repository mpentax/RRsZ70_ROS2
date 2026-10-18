> Historical/superseded architecture. Current contract: [thin-api.md](thin-api.md).
> The service-owned admission, recovery and workflow policies described below have been removed.

# Measured completion for autonomous workflows

`src/measured-completion.ts` provides an in-process, bounded completion observer:

```ts
createMeasuredCompletionObserver(source, options)
  // (context: MeasuredCompletionContext) => Promise<CommandCompletionEvidence | null>
```

Exported APIs: `MeasurementSource`, `CompletionMeasurements`,
`MeasuredCompletionContext`, `CompletionClock`, `MeasuredCompletionOptions`, and
`createMeasuredCompletionObserver`. Evidence uses the existing
`CommandCompletionEvidence` from `command-authority.ts` and always includes an
explicit `verifiedSuccess` boolean when validation succeeds.

## Trust boundary

Inject a reviewed `MeasurementSource` directly into the service. It identifies
itself as `kind: "independent-measurements-v1"`, has a nonempty `sourceId`, and
implements `read(context, signal)`. An unknown source returns `null`.

The integration is now implemented: `ServiceOptions.measuredCompletionSource`
and `measuredCompletionOptions` instantiate this observer. `CommandAuthority`
records the actual pre-send issuance timestamp for every command and passes the
latest boundary into the observer. It also rejects settlement predating dispatch.
The Docker ROS/service end-to-end fixture uses this full validator (five advancing
stable measurement frames per action), rather than a direct completion assertion.
There is no stock hardware `MeasurementSource` implementation yet.

The adapter must authenticate the acquisition and controller channels, bind them
to the physical device and current connection session, and correlate controller
execution IDs with the exact authority command IDs and workflow scope. The
interface discriminants are runtime checks of a trusted adapter contract, **not
authentication of arbitrary objects**. Do not construct this source or its
measurements from caller HTTP/MCP booleans, request bodies, or assertions that a
task succeeded. There is no HTTP fallback or robot transport in this module.

Each read returns a detached measurement frame or `null` if no frame is available.
Joint encoder positions (radians), localized base position/yaw (metres/radians),
and at least three stable visual scene tracks (normalized image coordinates)
each carry their own acquisition timestamp and evidence ID. Track/joint IDs
identify the same physical quantities across the stationary window. The adapter
must ensure joint coverage, visual track validity/quality, localization validity,
calibration and a cadence appropriate to the physical controller. Sparse samples
cannot certify arbitrary unobserved motion.

Capture timestamps must be acquisition times mapped to the same clock domain as
service dispatch and the observer clock. Clock mapping must preserve ordering;
receipt time, polling time, ACK time and a cached telemetry timestamp are not
measurement times. A `timestampOrigin: "receipt"` or a stock/status origin fails
validation. Do not fill missing channels with stock zeros, derive a pose from an
ACK, or relabel stock values as encoders/localization. Real measured zero positions
are valid; numeric zeros alone cannot prove or disprove sensor provenance.

## Validation and outcomes

Validation requires all of the following:

- Identical device, connection session, scope, and exact command-ID set. Order is
  immaterial; duplicates, subsets and supersets fail. IDs are compared literally.
- Safe integer timestamps, strictly after `dispatchedAt`, no future timestamps,
  and bounded freshness for **every** channel and terminal certificate.
- Independently advancing capture timestamps and evidence IDs for every channel.
  Repeated or reordered captures fail closed, including across observer calls.
- A controller-origin terminal certificate covering every exact command,
  `execution: "terminal"` and `queue: "drained-including-delayed"`. The adapter
  must establish that scheduled/delayed execution and controller retries for those
  commands cannot occur. Idle telemetry, STOP/transport ACK and a momentarily empty
  queue are insufficient. `controller: null` denotes nonterminal progress; it
  keeps polling and resets the stationary window. An invalid certificate fails.
- A consecutive stationary window **after** that controller certificate. Motion,
  unavailable frames, gaps, changing joint/track identities or a new terminal
  certificate reset the window. All three channels must span the entire window,
  satisfy minimum sample count, and stay within tolerances both relative to the
  window anchor and the preceding sample. This prevents slow cumulative drift
  from passing by taking many small steps.

A passing observation returns `cessationVerified: true` and
`noPendingExecution: true`. `verifiedSuccess` is true only when **every** correlated
controller task outcome is `succeeded`. Failed, cancelled and unknown task
outcomes may establish cessation with `verifiedSuccess: false`; this can settle
execution uncertainty but must not turn a failed grasp/navigation task into
success. The adapter must report genuine task results, not infer success from
cessation or accepted commands. If the controller cannot certify the task result,
use `unknown`.

`observedAt` is the oldest timestamp among the terminal certificate and final
channel captures, never a new receipt timestamp. The certificate must remain
fresh throughout the window. A repeatedly refreshed certificate resets the
window; a fixed certificate older than the freshness bound cannot be rescued by
recent poses. Consume returned evidence promptly with `authority.settle`; the
authority independently rechecks current session, epoch and command correlation.

Defaults: 10-second timeout, 50-ms poll interval, 750-ms stationary window, three
minimum samples, 200 maximum reads, 2-second freshness, 250-ms maximum per-channel
gap. Default tolerances: 0.005 joint radians, 0.003 base metres, 0.01 base yaw
radians, 0.005 normalized visual distance. Deployment must select tolerances and
capture cadence based on calibrated sensors and the physical workflow.

Timeout, exceptions, clock rollback, invalid input, unavailable source and sample
exhaustion return `null`. The observer aborts the source on exit, cancels its
timers, serializes observation calls and quarantines a source whose pending read
ignores abort until that read settles. A deadline bounds asynchronous waits; a
synchronous adapter that blocks the event loop cannot be made bounded by a timer
and must not be used. Successful command IDs are consumed, and accepted captures
retain per-device/session/channel high-water marks even after unsuccessful calls.
Replay history is capped at 10,000 entries by default; exhaustion fails closed
without evicting history. This is process-local observation protection. Durable
command-ID consumption and restart uncertainty remain the authority/journal's
responsibility; do not recreate an observer per HTTP request.

## Service integration contract

The current authority `completionContext(scope)` returns only `{ binding,
commandIds }` after transport waits settle. The current service accepts
`completionObserver({ scopeId, binding, commandIds })` with no dispatch timestamp.
This change deliberately owns only the new module, its tests and this document:
`ServiceOptions.measuredCompletionSource` and dispatch-boundary tracking still
need to be wired by the service owner.

The intended service option is:

```ts
measuredCompletionSource?: MeasurementSource;
```

Create one observer for the service lifetime. At the actual authority transport
issuance boundary, record service-owned dispatch timestamps associated with each
authority command ID, device, session and scope. Immediately before observing,
obtain the current authority completion context and derive
`dispatchedAt = Math.max(...timestampsForExactContextCommands)`. Any missing,
ambiguous, invalidated or previous-session entry must return no evidence. Do not
derive this timestamp from an ACK, HTTP receipt, observer start, lease deadline
or an arbitrary caller value. Using the latest dispatch ensures all measured
channels and the controller certificate postdate every command in the context.
The current arbiter record has a deadline but no dispatch time, so it is not an
adequate source for this derivation.

Illustrative service-side wiring (the boundary tracker is an integration seam):

```ts
const observeMeasured = createMeasuredCompletionObserver(options.measuredCompletionSource);

async function observeCompletion(scope: CommandScope) {
  const context = authority.completionContext(scope);
  if (!context || context.commandIds.length === 0) return null;
  const dispatchedAt = dispatchBoundaries.latestForExactContext(scope.id, context);
  if (dispatchedAt === null) return null;
  const evidence = await observeMeasured({ scopeId: scope.id, ...context, dispatchedAt });
  if (evidence) await authority.settle(scope, evidence);
  return evidence;
}
```

An adapter that cannot correlate terminal controller work with authority-generated
command IDs, provide independent captures, or exclude delayed execution is not a
production measured source. Leave the injected source unavailable and retain
unverified completion until such an adapter exists.

## Offline verification

```sh
node --test src/measured-completion.test.ts src/command-authority.test.ts
npm run typecheck
```

The fake-clock suite checks each modality's motion, stationary-window reset and
drift, stale/future/pre-dispatch/reordered/replayed feedback, wrong session/device/
scope/commands, receipt timestamps, stock-zero provenance, idle/ACK/pending
execution, failed/cancelled/unknown task results, successful completion, gaps,
nonterminal-to-terminal transition, hung-source deadline/abort, bounded history,
and actual authority settlement versus timeout uncertainty without replay.
Tests use in-memory sources and journals and never contact a physical robot.
