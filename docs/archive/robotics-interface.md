> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

The v1 observation/docking envelope, always-disabled docking and reservation procedure are retired. Receipt timestamps and unmeasured-joint limitations remain evidence distinctions, not justification to restore that API.

## Historical record

# Robotics interface v1

Authenticated, host-checked, query-free GET endpoints:

- `/api/robotics/v1/capabilities`: static description; zero robot reads.
- `/api/robotics/v1/state`: at most one `get_status` and one `app_get_arm_joints_data` read per request, using the existing connection. No camera reads/frames, stream startup, mode changes, background polling, retries, or control commands.

Both use `{ apiVersion: "1.0", kind: "capabilities" | "state", generatedAt: <epoch-ms>, data: ... }`. See `src/robotics-contract.ts` for exact TypeScript schema.

Capabilities data: `readOnly: true`, `endpoints: { capabilities, state }`, `observations: ["status", "armJoints"]`, `commands: []`, `cameraAccess: false`, `backgroundPolling: false`, `physicalStowVerification: false`, `unknowns`.

State data: `connection: "connected" | "unavailable"`, `raw: { status, armJoints }`, `physicalArmStowed: null`, `unknowns`. Each raw observation is `{ value, observedAt, source: "robot-rpc", freshness: "fresh" | "unavailable", ageMs, error: null | "READ_FAILED" }`. Unavailable values and timestamps are null. Fresh means read within this request and unchanged connection session, not a guaranteed physical condition. A session change invalidates the whole snapshot. No previous samples survive a failed read.

Status values use the existing status allowlist (`battery`, `state`, `error_code`, `in_cleaning`, `in_returning`, `fw_ver`). Arm values use bounded recursive numeric/boolean telemetry sanitization, rejecting security/identity fields and arbitrary strings. `raw` means sanitized uninterpreted readings, **not** an upstream payload. Joint units, kinematic calibration, physical stow, robot pose, and dock contact remain unknown. A zero joint array is telemetry, never proof of stow. Clients must not synthesize ROS joint units/frames or navigation authority from these values.

## Guarded stow-before-dock

`DockingWorkflow` is an offline/fake-testable, single-attempt orchestration boundary:

1. Check existing permission and absence of active arm/chassis commands.
2. Acquire an exclusive reservation covering **all** arm/chassis writers; recheck admission.
3. Send a permission-checked stow action and wait for its completion/ACK.
4. Require fresh physical evidence acquired after stow began (`supervised-physical` or a future independently validated sensor). ACK and joints are rejected.
5. Recheck permission, exit the appropriate arm mode, then read fresh docking readiness.
6. Recheck permission, concurrency, and verification age; send exactly one return-dock command.

Every unavailable/unconfirmed physical outcome is staged `NEEDS_VERIFICATION`; busy/permission/readiness failures are `BLOCKED`. `DOCK_ACKNOWLEDGED` is command acceptance only; `physicalDockVerified` always remains false. There are no automatic retries or polling. Reservation adapters must cover every concurrent writer; the class's own running flag only protects its own runs.

The current service has no trusted physical-stow source and no full-sequence reservation shared with every writer. Therefore the existing compatibility dock endpoint now fails closed with HTTP 409 and `NEEDS_VERIFICATION` even when mode-off bits or a caller's `armStowed: true` claim look good. It does **not** automatically stow or dock, expand permissions, or expose a new write API. Existing separately authorized finite arm actions remain available. The reusable workflow is intentionally not wired to an unsafe/no-op reservation or fabricated evidence.

## Required supervised validation before production orchestration

With explicit operator authorization, a clear work area and physical stop ready: validate the supported stow command in manual and remote-pickup modes, characterize completion versus ACK and the observed arm-out dock failure, establish physical stow evidence, confirm correct mode-exit command and readiness feedback, and validate uninterrupted exclusive ownership/cancellation/session-change behavior. Then implement the trusted evidence/reservation adapters under existing per-device permissions. Verify actual dock contact separately. No live robot action, observation, permission alteration, or service restart is required for the offline tests in this change.
