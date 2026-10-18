> Historical/superseded architecture. Current contract: [thin-api.md](thin-api.md).
> The service-owned admission, recovery and workflow policies described below have been removed.

# Browser-supervised docking

The chassis panel contains a separate supervised docking component. Its existing STOP button remains independently clickable during every docking wait. No component mount, polling callback, reconnect, or status read triggers an action.

## Operator flow

1. **Read docking workflow** (GET only) to discover existing reservations. Start is unavailable before this read.
2. **Start stow-for-docking** sends one POST. This asks for stow; it does not certify folding. Busy, reservation and unknown errors remain visible.
3. At `WAITING_PHYSICAL_CONFIRMATION`, physically inspect the arm after this request. Check the initially unchecked folded-arm box, then separately click **Confirm folded arm → check mode/readiness and request dock**. No automatic confirmation exists. The button expires with the server workflow deadline.
4. The service performs mode exit, fresh readiness checks and docking. `ACKNOWLEDGED` is displayed as **NOT verified docked**. Watch physical arrival and cessation; no state snapshot or ACK establishes physical completion.
5. Separately check the physical cessation/folded-arm box and click **Confirm physical cessation and folded arm / release reservation**. For normal docking completion the operator also observes arrival. Recovery may instead confirm cessation away from the dock. `PHYSICALLY_VERIFIED` is labeled as an operator confirmation, not a calibrated arrival measurement.

Cancel future docking steps is explicitly not a motor stop. Unknown and acknowledged reservations cannot be cleared by cancel. Use the physical stop when needed. A failed/lost mutation disables replay in this mounted component until an explicit physical finish succeeds; fresh reads do not clear the error or authorize replay.

## Route contract implemented

All calls use the existing `json()` API wrapper, same-origin browser cookies, no bearer credentials, JSON POST bodies, and its existing session-expiration handling. Server Origin checks remain in force; no custom CSRF bypass or headers were added.

- GET `/api/docking`: read the snapshot only.
- POST `/api/docking/start`: `{}`.
- POST `/api/docking/confirm`: `{ workflowId, postStowAt, physicalStowed: true }`, using exactly the snapshot binding values after fresh operator observation.
- POST `/api/docking/cancel`: `{ workflowId }`.
- POST `/api/docking/finish`: `{ workflowId, physicallyStoppedAndStowed: true }`, only after a separate physical observation and click. A recovered null workflow ID can be passed as null; the backend determines recovery authorization.

Snapshots contain `workflowId`, `stage`, `postStowAt`, `expiresAt`, `acknowledged`, and `physicalDockVerified`. The UI does not manufacture ownership: confirmation routes enforce the owning cookie identity and device/session binding. Another browser may read a reservation but cannot use that visibility as owner authority.

Every mutation explicitly uses `action: true`, including cancel and finish, to classify lost/timeout replies conservatively. Start/confirm use 45-second bounded waits; cancel/finish use 15 seconds. There are no automatic retries. The UI preserves unknown outcomes rather than promoting an HTTP response or ACK to arrival.

## Offline validation

`web/tests/docking.test.ts` uses fake fetch only. It verifies read versus write separation, cookie transport, exact confirmation payloads, physical-confirmation gates/expiry, ambiguity/no retries, explicit unchecked boxes, absence of mutation polling, and retention of STOP access.

Validated with `node --test web/tests/*.test.ts` (43 passing) and `npm run build` (full TypeScript checks and Vite production build). No robot requests, live browser clicks, restarts, permission changes, commits or pushes were performed.
