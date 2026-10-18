> Historical/superseded architecture. Current contract: [thin-api.md](thin-api.md).
> The service-owned admission, recovery and workflow policies described below have been removed.

# UI action outcomes

The browser separates request transport, execution evidence, and physical verification. No action request is automatically retried.

## Evidence rules

- A signal already aborted before an actuation request is sent is a local pre-dispatch rejection; fetch is not called.
- A non-2xx HTTP reply is not by itself proof of pre-dispatch rejection. Legacy controller 409/502 errors can occur after dispatch. Without explicit execution evidence its outcome remains unknown.
- Network loss, timeout, cancellation after dispatch, malformed JSON, or unexpected content on an action response leave execution unknown: the request may have executed.
- A successful HTTP/state-only response does not establish a robot acknowledgment. Legacy `acknowledged: true` does; `acknowledged: false`, `stopped: true`, and idle/ready state do not.
- Accepted and in-progress responses are not completion. Acknowledgment is also not physical completion.
- Explicit execution failure is displayed as such only when supplied in an outcome envelope. Partial movement is still possible.

Arm actions display the returned evidence rather than always claiming acknowledgment. Cancellation retains a potentially-executed warning even though the generic task hook suppresses aborted-request errors. Cancel only prevents future requests, not motor motion. Chassis response handling rejects explicit failed/rejected outcomes, preserves ambiguous input errors, and ends the local lease/heartbeat after an input error. Its one-shot safety stop is not a replay of the original movement. Controller inactive/disabled state is never presented as proof of a physical stop. Existing hold and heartbeat supervision remain unchanged.

## Backend integration contract

`web/src/outcomes.ts` is intentionally browser-local and has no server imports. It currently accepts an optional `actionOutcome: { outcome: ... }` envelope, or a top-level string `outcome`. Recognized kinds are `unknown`, `rejected`, `failed`, `accepted`, `in-progress`, and `acknowledged`; legacy acknowledgment/accepted/in-progress fields remain supported. Unknown envelope extensions are not promoted to success.

The backend helper's `ActionOutcome` kinds fit this reader (`failed` is a forward-compatible UI extension, not an inferred backend status). Service action routes must attach `actionOutcome` to both successful replies and JSON HTTP-error replies to convey explicit evidence. If routes choose a different envelope key, update the reader and contract tests together. The UI does not infer pre-dispatch rejection from HTTP status: only the backend's explicitly evidenced rejection may establish it. Backend producers must not classify ambiguous busy/readiness/transport errors as rejected. A known execution failure requires affirmative execution evidence, not a timeout or negative/missing ACK.

Do not resolve an unknown outcome from a later uncorrelated state snapshot. Backend request-correlated reconciliation may update execution evidence; physical verification requirements must remain visible. No current browser logic consumes telemetry as physical success evidence.

## Offline validation

Run `node --test web/tests/*.test.ts` and `npm run typecheck`. Tests cover legacy response ambiguity, explicit envelopes, ambiguous HTTP failures, network/body loss, timeout/cancellation, zero-send pre-abort, and absence of API retries, alongside chassis lease/input ordering tests. No live robot requests or browser trials are required or performed.
