> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

Shared MCP/UI/DDS testing below is reported historical software evidence, not deployment/hardware acceptance. The old cd ros2 recipe and handoff task are inactive; named missing verification artifacts are unavailable.

**Historical outputs unavailable in this checkout:** `docs/migration/phase4-cross-surface.json`, `docs/migration/phase4-npm-audit-final.json`, `docs/migration/phase4-npm-build-final.txt`, `docs/migration/phase4-npm-final.txt`, `docs/migration/phase4-release.txt`, `docs/migration/phase4-ros-pass.txt`, `docs/migration/phase4-verification.json`. Results citing them below are reported history only.

## Historical record

# Phase 4 — shared ROS MCP/web controls and web setup

Task `t_6d59fe24`. Implemented and qualified offline at unchanged HEAD
`de27d42dc098308949d6c6f122cadfcd8ac7a337`, branch `claude/cleanup`.
No robot/camera access, deployment, checkout relocation, private-state reads,
staging or commits. Prior phase-2/3 changes, unrelated geometry edits and retained
calibration evidence are preserved.

## Delivered

- New ament Python `z70_web_gateway` is a ROS client of existing generated
  actions/services/topics. It owns one bounded task presentation registry for
  both clients, not execution/estimation/acquisition. Handles are the actual ROS
  action goal UUIDs. Generated feedback/results retain their fields, numeric
  phases and physical-completion uncertainty.
- Existing authenticated TypeScript service proxies `/api/ros/*` to this node.
  Existing TS stdio MCP remains a protocol-only client of that same API, avoiding
  a second Python MCP server/ROS registry. This explicitly supersedes the phase-1
  separate `z70_mcp` process proposal; phase 5 places the thin shim in `apps/mcp`.
  The Python gateway stays in the ament source tree.
- MCP supports shared state/calibration, latest snapshots, camera selection,
  anchor, Advanced raw RPC, start/list/get/cancel task, priority stop and map.
  The start tool supports Pick and every bounded primitive. Direct calibration
  JSON reads and stale degree-based estimator claims are removed.
- React displays connection/battery and original state evidence, camera controls,
  shared task IDs/phases/feedback/results/cancellation, map data and priority stop.
  Raw RPC is under Advanced, not a parallel workflow. Stop is never disabled by
  a busy task or an outstanding action-start response.
- Web forms cover account authentication/reauthentication, region, verification
  code, authenticated device selection, literal IP/local protocol and existing
  camera pattern. Inputs are cleared after secret submission; session expiry
  unmounts the setup form and clears sensitive UI state. Credentials stay in the
  private backend and never enter ROS/MCP arguments, browser storage or logs.
- `/api/setup` exposes only nonsecret configuration. Account failure/expiry and
  invalid input have bounded field-specific recovery messages. Browser cookies
  cannot access old private acquisition, preview, telemetry or driver routes.
- No client writes are retried. Unavailable-before-call is NOT_SENT; missing
  response after dispatch is UNKNOWN. Late action acceptance/result reconciles
  the same UUID rather than submitting another goal. Driver/service reconnect
  restores reads without replaying uncertain/unavailable writes.
- Shared JPEGs require matching ROS image/metadata stamp/frame, source camera,
  valid receipt evidence and age <=5 seconds. Reads neither acquire a new frame
  nor select/restore a lens. Repeated reads retain sequence/session/receipt identity.
  Uncalibrated images may be viewed without claiming metric validity.

Exact API, bounds, enums and retention/restart behavior: [phase4-api.md](phase4-api.md).

## Verified results

- 365 npm tests pass, zero failures; TypeScript/service/web typecheck and Vite
  production build pass.
- Fresh non-symlink colcon build: 10 packages. Full installed ROS suite: 192 pass,
  3 pre-existing optical-model xfails, zero failures. The historical optical
  tolerances and calibration values are unchanged.
- Real MCP SDK + real authenticated HTTP proxy + rendered React (happy-dom) +
  real DDS + fake driver/action servers: MCP-started goal appears in the UI with
  identical ID/phase/result; the actual rendered web form starts a goal observed
  by MCP. Cancellation works from either surface; UI Cancel cancels an MCP goal.
  Stop responds during a long task and still reports cessation unobserved.
- Real driver over fake HTTP verifies unknown-after-write, vendor rejection,
  failed/cancelled terminal results and priority stop/cancel under a stalled RPC.
  Tests distinguish stop-terminated tasks from a later cancel request, rather than
  racing the two into an incorrect expectation of two independent acceptances.
- Actual DDS service discovery loss/recreation does not replay writes. Delayed
  action acceptance exceeds the HTTP acceptance budget, reports UNKNOWN with its
  UUID, then reconciles exactly one late goal/result.
- Actual shared JPEG/metadata and standard OccupancyGrid/calibration subscriptions
  preserve source fields/identities, reject old frames and make no acquisition RPC.
- Backend tests cover invalid account/configuration/PIN, bounded credential failure,
  server-only secrets and expired sessions. React tests cover expired-login recovery,
  session-secret clearing, stop while busy, shared task rendering/cancel and passive
  read-only reconnect.
- Installed gateway/driver module bytes match source SHA-256. `git diff --check`
  passes. npm audit reports zero vulnerabilities after semver-compatible MCP SDK
  and source-map-js lockfile updates. happy-dom is test-only.

Evidence: phase4-verification.json (historical path `docs/migration/phase4-verification.json`; unavailable),
phase4-cross-surface.json (historical path `docs/migration/phase4-cross-surface.json`; unavailable),
phase4-npm-final.txt (historical path `docs/migration/phase4-npm-final.txt`; unavailable),
phase4-npm-build-final.txt (historical path `docs/migration/phase4-npm-build-final.txt`; unavailable),
phase4-release.txt (historical path `docs/migration/phase4-release.txt`; unavailable) (fresh build and earlier test-race failure),
phase4-ros-pass.txt (historical path `docs/migration/phase4-ros-pass.txt`; unavailable) (final full installed suite),
phase4-npm-audit-final.json (historical path `docs/migration/phase4-npm-audit-final.json`; unavailable).

An earlier verification used a stale pre-existing domain-92 CLI daemon and failed
with InvalidHandle. The test now uses explicit daemon-free `ros2 topic/param`
commands and standard rclpy action graph inspection: this Jazzy `ros2 action list`
has no `--no-daemon` option and ignores the former environment flag. No unrelated
process/daemon was stopped. Incremental macOS rosidl reinstalls also emitted
install_name_tool delete_rpath warnings; the final fresh build is clean. Owned
fixture action clients/servers are explicitly destroyed before context shutdown.

These are offline fake-driver and rendered-DOM results, not a real-browser or
hardware trial, motor-cessation proof, camera recovery, calibrated accuracy or
successful physical pickup claim.

## Historical reproduction recipe (retired workspace)

The following is the original receipt, not a present-day command. Use the
[current ROS guide](../ros2.md) and root manifests for supported checks. The
old `ros2` subworkspace no longer exists.

Original working-directory description:

```sh
npm test
npm run typecheck
npm run build
cd ros2
~/.pixi/bin/pixi run colcon --log-base log-check build --base-paths src --build-base build-check --install-base install-check
~/.pixi/bin/pixi run bash -c 'source install-check/setup.bash && export ROS_DOMAIN_ID=96 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 && python -m pytest src --basetemp build-check/pytest -q'
```

Use a fresh prefix for clean native Mach-O install verification; do not delete
another run's build/install or stop its nodes. Test fixtures use isolated domains
92/94/95/97 and loopback fake servers. Standard ROS launch argument discovery,
colcon/Pixi and Compose remain the operational/developer tools; no bespoke launcher,
supervisor, CLI, doctor or recovery agent was introduced. The phase4 scripts in
`scripts/` and `docs/migration/` are test/evidence harnesses, not operator commands.

## Handoff to existing phase 5 (`t_220f88b3`)

- Root source relocation/cleanup remains its responsibility, including new gateway,
  MCP shim, tests/harness imports and npm/ament dependencies. The old path-map JSON
  is a historical phase-1 index snapshot, not an inventory of current new packages.
- Gateway defaults to loopback port 57020 and `web_gateway:=false` in bringup;
  enabling it reads the server-side token_file parameter. The web proxy defaults
  to `Z70_ROS_GATEWAY_ORIGIN=http://127.0.0.1:57020`. The current Docker backend
  cannot reach a native loopback listener automatically: coordinate native/container
  networking in phase 5 before deployment, without exposing credentials or the
  gateway to a LAN. Do not silently widen the bind or add a vendor fallback.
- Do not deploy this source over an active controller. No live services were
  restarted; source MCP watchers may observe source changes normally.
- `src/service.ts` is a dirty multi-phase hotspot: retain phase-3 private transport
  changes and phase-4 ROS proxy/setup isolation together. Do not restore TS arm
  estimation, per-client camera sessions or direct legacy control routes.
- Keep current RPC/unknown/cancel/stop/task identity tests and all retained evidence.
  Hardware qualification and checkout relocation still need their separately
  specified authorization/coordination.
