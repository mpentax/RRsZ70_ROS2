# Robotics observation admission and privacy

The v1 envelope is unchanged. Status contains only known bounded integers (battery 0–100; state/error_code 0–65535; cleaning/returning flags 0/1 or booleans). Malformed recognized fields invalidate status. Firmware strings are omitted entirely. Arm telemetry retains the existing bounded numeric-only sanitizer. No upstream exception text is returned.

## Required service wiring

Replace `roboticsState(robot)` with:

```ts
roboticsState(robot, Date.now, {
  admit: () => !stopped && !accountBusy && !armState.busy && !chassisState.busy,
  timeoutMs: 2500,
})
```

Use the **actual current** arm/chassis supervisor state getters, not the illustrative variable names above. Include every active **or claiming** writer/reservation (including account/device switch, dock reservation, and any cleaning/other writer paths). Missing admission fails closed. The callback must be synchronous and side-effect-free; exceptions refuse admission. Recheck it before every RPC. Set writer claiming state synchronously before any awaited preflight so observation cannot slip into that gap. Do not use `() => true` in production.

Required transport API: `requestConnected(method, params)` allowlists only the two observation methods, synchronously verifies connected session and idle queue/available inflight slot, and reserves/enqueues without invoking connect or waiting behind a backlog. The ordinary reconnecting `request` API is never a fallback. Missing connected API returns unavailable. Parent has implemented this method in RobotClient; service must pass the same stable RobotClient object across callers (do not construct per-call wrappers).

All callers share one WeakMap entry per robot. Reads are sequential: at most one observation RPC outstanding, no polling queue, one shared two-read operation. Busy callers are refused even if an operation is in progress. With transport no-backlog admission, this source cannot fill all four inflight slots or create a pending backlog. Other subsystem reads are outside this limiter and must have their own admission.

## Deadlines and disconnects

Default caller deadline is 2500 ms (range 1–2500), below the ROS client's 3000 ms HTTP timeout. Deadline returns an unavailable v1 envelope, **not cancellation**. The shared operation remains accounted until its underlying calls settle; later idle callers join that operation rather than start more reads. Disconnected HTTP consumers likewise do not release accounting. A transport promise that never settles holds the observation slot indefinitely, favoring safety over availability. The underlying transport response timeout remains its own responsibility.

Session identity is checked before and after each RPC and at complete snapshot generation. Disconnect/session change prevents the second read and invalidates the whole envelope. Admission is rechecked between reads so a newly claiming writer suppresses the second read. One already-dispatched read cannot be withdrawn; there is no claimed cancellation guarantee or atomic physical-writer reservation here.

## Offline verification

`node --test src/robotics-api.test.ts src/observation-admission.test.ts` covers 30 overlapping callers, a single outstanding RPC maximum, deadline accounting, writer becoming busy during the first read, missing API/admission, disconnect, fixed error strings, malicious status values, and firmware omission. `npm run typecheck` verifies the narrowed TS contract. No live robot calls or restarts are necessary.
