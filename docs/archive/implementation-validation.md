> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

Counts and synthetic timings below are reported historical software results, not current acceptance tests or measured physical-stop bounds. Named /tmp logs are ephemeral/unavailable; no original output has been reconstructed.

Temporary/scratch paths below are historical ephemeral references, not retained portable outputs; their original contents are unavailable from this checkout and were not reconstructed.

## Historical record

# Local robotics implementation validation ledger

## Scope

Owner authorized implementation on 2026-09-30. Development reuses the existing connector on the Mac mini. Parallel workers own disjoint paths. No live actuation, camera session, motion-policy change, or active deployment restart is part of initial software validation.

## Required software gates

1. Preserve existing offline regression suite and TypeScript checks.
2. Test versioned observation contracts with unknown/stale/missing data; do not synthesize measured joints, odometry, or calibration.
3. Build Linux ARM64 ROS image and test the Python transport independently of ROS.
4. Exercise ROS against a fake service, including disconnect and replay. Fake data must be unmistakably labeled.
5. Audit command outcomes across service, MCP and UI. Transport errors, command rejection, vendor acknowledgment and physical outcome are separate facts.
6. Test docking orchestration with fake arm/readiness feedback, including timeout, busy arm, stale feedback, missing physical verification and rejected dock. No automatic repeat after uncertain dispatch.
7. Review authentication, host validation, logging, queue limits and credentials boundaries before deployment.

## Physical validation gates (not satisfied by software tests)

- Nearby operator and accessible physical stop before motion.
- Confirm initial arm state independently; zero joint arrays are not pose evidence.
- Validate chassis startup timing, finite movement, explicit stop and disconnect behavior.
- Validate stow, mode exit and readiness before requesting dock.
- Verify physical stow and docking outcome independently of RPC acknowledgment.
- Do not unlock unattended movement until observable failure handling and stop behavior are established.

## Evidence status

- Existing checkout inspected before parallel work; latest starting commit: 91bf4c7.
- Full baseline npm test completed: 463 passed, 0 failed, 0 skipped/cancelled; 22.4 seconds. Log: /tmp/roboclaw-baseline-tests.log.
- ROS foundation, service observation/docking, MCP outcome semantics and UI outcome work delegated separately.
- Initial MCP outcome worker reported 481 full-suite passes; UI worker reported 39 UI test passes and typecheck success. These are intermediate shared-checkout results, not final integration certification.
- Service observation/docking worker supplied typed API and offline docking state machine. Legacy docking became fail-closed pending trusted physical evidence; this regression is under explicit integration review, not claimed as a completed docking fix.
- Follow-on service integration added typed action responses and stopped ambiguous duplicate camera-setting writes; fallback now requires explicit method-not-found RPC evidence.
- Independent review findings recorded in robotics-review.md. Observation sanitizer, coalescing/admission, connected-only reads, MCP outcome propagation and ROS schema hardening were implemented in response.
- Post-review integrated npm suite passed 509 tests, zero failures/skips/cancellations; source/web typechecks passed. Logs /tmp/roboclaw-final-tests.log and /tmp/roboclaw-final-types.log. Further changes still require another regression pass.
- Hardened Linux ARM64 ROS image built and network-disabled read-only/nonroot smoke passed, including five-message MCAP recording and isolated replay. No live service connection tested.
- Simulation-only ROS action layer implemented; parent independently verified 18 Python tests and network-disabled ROS smoke covering four operations and duplicate rejection.
- Supervised production docking workflow and browser panel implemented in working tree, not deployed. Second review found writer/STOP/recovery/persistence gaps; fixes remain under review. Parent corrected UI recovery-token handshake.
- Current second-review regression run: 522 Node tests passed, zero failures/skips/cancellations; production build including source/web checks passed. Logs /tmp/roboclaw-review2-current.log and /tmp/roboclaw-review2-build.log. Concurrent final fixes still need validation.
- Pure command arbiter is tested (11 tests including acknowledged-but-unverified pending ownership) but not yet the deployed all-writer authority.

## Stable-checkout final checks

- Parent independently reran full Node suite: 526 passed, zero failures/skips/cancellations; 23.9 seconds. Production build and source/web typechecks passed. Logs /tmp/roboclaw-final-526.log and /tmp/roboclaw-final-build.log.
- Parent independently reran 21 Python tests: all passed. Log /tmp/roboclaw-python-21.log.
- Parent independently reran final ARM64 network-disabled ROS action smoke: four operations and duplicate rejection passed; held-motion saturation test STOP 0.181s and cancel/cessation 0.246s under synthetic 1.5s assertions; post-submit deadline expiry returned unknown. Log /tmp/roboclaw-final-ros-smoke.log. These timings are not physical stop guarantees.
- Final independent review cleared normal-path blockers for a controlled supervised trial only; see robotics-review-final.md. Parent subsequently fixed residual cross-owner/unauthenticated logout invalidation and added service regression assertions. Full suite still 526 passed and production build/types passed; logs /tmp/roboclaw-owner-fix.log and /tmp/roboclaw-owner-build.log.
- Compound storage-failure durability remains a declared limitation. No unattended or ROS REAL actuation readiness claim.
- Active app deployment has not been restarted or changed. No physical tests performed in this implementation run. Controlled deployment requires current physical stationary/stowed confirmation and an available supervising operator.

This ledger describes validation requirements, not a claim of autonomous capability or a physical safety certification.
