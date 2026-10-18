> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

Synthetic-only conclusions and residual storage caveats below remain historical. Recovery/docking acceptance and per-data-gap stop rules are retired, not current operating doctrine.

## Historical record

# Final targeted independent recheck

2026-09-30. Scope: review-2 findings only, final inspected source and targeted offline tests. No live robot calls, restarts, builds, commits or code changes. This report is the sole owned file.

## Readiness verdict

**The previously identified normal-path P1 blockers are corrected for an explicitly supervised browser deployment / controlled physical validation trial. This is not approval for unattended autonomy or production ROS REAL actuation.** The implementation now fails closed on uncertain execution, retains reservations after ACK, and exposes appropriate independent stop requests. Physical stop availability, supported stow/mode-exit behavior and actual docking/cessation still require operator-supervised validation; offline tests cannot establish them.

Two residual limitations below remain explicit. They do not authorize replay or relaxation of physical checks.

## Review-2 disposition

- **Journal inspection failure — resolved.** `service.ts:278-279,316` restores UNKNOWN if journal is missing/uninspectable as well as pending. Thus writer/legacy-read fences apply globally, not merely docking start. Test `unsafe journal startup quarantines motion and legacy robot reads without disabling independent stop` checks arm, pulse, account selection, state and native cancel dispatch.
- **Zero braking and native task stop — resolved in code.** `service.ts:407-412` permits zero velocity/omega, stop/end and a narrowly flagged native cancel. `stopNativeDock` (`730-740`) checks device permission and ACK; browser STOP invokes native and RC stops independently (`1121-1124`), and explicit cancel-task remains admitted. ACK remains unverified physical cessation. Real robot effectiveness is a physical-validation gate, not an offline claim.
- **Legacy observations — resolved for reservation scope.** `service.ts:779-780` blocks robot-reading legacy GETs while reserved. Robotics reads return unavailable because their admission includes !reserved (`784`). Start refuses pre-admitted GETs and outstanding coalesced robotics work (`842`); server accounts GET work until completion (`1245-1249` and finally). The previously admitted GET regression verifies start rejection before any command. Snapshot GET /api/docking and static capabilities remain allowed.
- **Recovery cookie ownership — resolved for recovery.** `supervised-docking.ts:73-81` permits an authenticated replacement operator to obtain a fresh, bound challenge for quiescent reserved workflows; finish binds token to that operator/device/session and corroborates status again. Stow-confirm remains original-owner-only. Service integration test successfully recovers from a second browser. No persistence of browser session secrets is required.
- **Clear cancellation race — resolved.** `supervised-docking.ts:96-107` checks permission, revision, binding and challenge age after durable clear; failure stages UNKNOWN and attempts to restore marker. The cancel-during-clear regression asserts reservation remains held and persist is called. Initial persistence failure likewise retains UNKNOWN and prevents retry. See residual multi-failure durability limitation below.
- **UI recovery mismatch — resolved.** `web/src/docking.ts:46-57` requests recovery, validates workflow/token, then submits exact recoveryToken and physicallyDockedStationaryAndStowed fields. `Docking.tsx:67-71` explicitly asks operator to observe docked, completely stationary and arm folded. No automatic physical assertion occurs without the operator checkbox/click. The helper fetches challenge after the click and uses the recent operator assertion; this is a bounded human assertion, not independent sensor evidence.
- **Background camera owner — resolved.** `service.ts:483` blocks cameraExclusive; restore defers without consuming retry count (`495-500`), monitor refuses reservation (`525-526`). Start checks existing camera work and hold/preview. Background restoration regression verifies defer. Direct robot writer proxy now requires both docking async context and `docking.mayDispatch` (`410`), immediately revoking dispatch privilege on cancellation rather than allowing delayed async-context writers.
- **ROS executor STOP starvation — resolved within simulation adapter admission model.** `roboclaw_actions/node.py` admits one non-stop and one stop slot, removing capability I/O from goal callback and limiting long execute occupancy to two of four threads. Cancel remains independently schedulable. Worker reports rebuilt synthetic saturation STOP 177ms/cancel 247ms; this review did not run container tests or independently measure those timings. Python tests include strengthened STOP authorization. Stop uses an exact client/token/endpoint/mode-bound cached successful grant to avoid renegotiation failure while motion is pending; initial unnegotiated STOP still requires capabilities, appropriately failing closed rather than guessing authority.
- **ROS deadlines, stop evidence and fake terminality — resolved.** Client applies remaining deadline budget before/after I/O; node checks after submit/poll/cancel and yields unknown on expiry. Successful stop explicitly requires cessationVerified. Fake only transitions nonterminal states. Real physical evidence provenance/durable backend dedup remains a documented backend obligation; no such real backend is silently supplied.

## Precise residual issues / operating limitations

### P2: session DELETE can still cancel another browser's workflow

`service.ts:761-767` authenticates optionally, checks Origin, then unconditionally calls docking.cancel if reserved. Therefore a same-origin session DELETE from a non-owning browser, even with no valid cookie, invalidates docking future steps. This cannot dispatch motion, release quarantine, or clear the journal; it is an availability/ownership defect rather than a motion-safety bypass. Existing review-2 ownership finding is therefore only partially resolved: recovery takeover is fixed, logout ownership is not. Prefer owner-bound logout invalidation, reserving cross-owner invalidation for explicit STOP. During supervised trials, use a single controlling browser and avoid unrelated sign-out actions. Not a blocker to a consciously single-operator trial, but should be corrected before multi-client deployment.

### Durability guarantee excludes compound storage failure during recovery settlement

`supervised-docking.ts:138-147` still unlinks then fsyncs; `104-107` attempts to re-persist on failed clear/post-clear validation, swallowing failure while retaining in-process UNKNOWN. If unlink succeeded, directory sync failed, AND marker recreation also fails, a later restart may find no marker and permit writers. The normal single-clear-failure path is repaired; arbitrary compound storage failure is not demonstrably crash-safe. Do not claim unconditional durable quarantine under filesystem failure. If settlement reports failure, keep the service/robot quarantined operationally, inspect the private pending marker/storage health, and do not restart into motion availability until physical recovery and durable state are established. Stronger failure guarantees require a durable multi-record settlement design; the current targeted tests establish restoration attempts, not persistence under failing storage.

A crash after marker deletion during finish occurs only after repeated fresh docked/inactive corroboration and explicit physical recovery assertion; this is not the old unchecked-clear race. Hardware/OS storage durability cannot be proved from mocks. For safety-sensitive deployment requiring fail-closed behavior through arbitrary storage failures, this remains a blocker; for a supervised prototype with healthy private storage and operator recovery procedure, it is a declared limitation.

## Inherent physical validation requirements, not unresolved source bugs

Before supervised activation: verify current device-bound permissions, clear work area, accessible physical stop, supported manual stow command, correct mode exit, and readiness interpretation on the actual robot. Observe arm folded after stow before confirm. Observe docking arrival, full cessation and folded arm before releasing reservation. Exercise independent software STOP/cancel under supervision, but never rely on Wi-Fi/host timers or ACK to prove cessation. Stop immediately and retain quarantine on missing ACK/session change/timeout.

ROS remains a **simulation milestone**: REAL is disabled by incompatible production read-only capabilities, with no fallback to legacy writes. Boolean verifiedSuccess/cessation fields are trusted-backend assertions; synthetic tests are not physical validation. Unattended planners, calibrated joints, navigation geometry and automatic recovery remain outside this readiness verdict.

## Independent offline evidence

- `node --test src/supervised-docking.test.ts src/service.test.ts web/tests/docking.test.ts`: **67 passed, 0 failed** (includes journal failure, recovery, cancellation, background restoration and legacy observation admission regressions).
- `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=robotics/ros2/src/roboclaw_actions:robotics/ros2/src/roboclaw_readonly python3 -m unittest discover -s robotics/tests -v`: **21 passed, 0 failed**. Synthetic localhost fixture tests only; no application or robot access.
- Parent reports 526 full-suite passes; ROS worker reports 21 Python tests and rebuilt saturation smoke. Those reports are supplementary, not independently rebuilt here.
