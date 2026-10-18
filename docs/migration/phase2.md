> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

Observation/acquisition identity and shared-driver test scope are retained. TypeScript estimator ownership, separate display tree, ros2 workspace and phase-3 TODOs are superseded. Reported test counts are historical; named missing output is unavailable, not a current independently verified pass.

**Historical outputs unavailable in this checkout:** `docs/migration/generate_observation_interfaces.py`, `docs/migration/phase2-clean-build.txt`, `docs/migration/phase2-clean-test.txt`, `docs/migration/phase2-installed-driver-test.txt`, `docs/migration/phase2-npm-build.txt`, `docs/migration/phase2-npm-test.txt`, `docs/migration/phase2-ros-test.txt`. Results citing them below are reported history only.

## Historical record

# Phase 2 — one ROS observation driver

Task `t_f0121e04`, existing checkout and branch. Offline implementation only: no
robot/camera access, deployment, service restart, checkout relocation, commit,
private-state read, or calibration/description asset changes were performed.

## Implemented ownership

- `ros2/src/z70_interfaces`: 13 generated observation/evidence messages and
  SelectCamera/RefreshMap services. The checked-in definitions are generated from
  `interfaces.md` by `generate_observation_interfaces.py`; rosidl generates real
  Python/C/C++ types during the standard colcon build. Phase-3 actions/stop/anchor
  and legacy Pick interface replacement are deliberately not implemented here.
- `ros2/src/z70_driver`: sole ROS HTTP observation client, backed by the existing
  private TypeScript service. State/arm, camera, map and live command-stream workers
  are owned, bounded and joined on shutdown. No child process or lifecycle wrapper
  is introduced. Standard ROS node/launch/parameters remain visible.
- Bringup launches `z70_driver/driver`. `z70_bridge/bridge` is a deprecated import/
  entrypoint alias to the same implementation, not another acquisition pipeline.
  Historical dashboard helpers remain offline utilities; the old bridge HTTP
  polling implementation is gone.
- Mapping subscribes to latched MapSnapshot, decodes it and publishes the usual
  latched grid/poses/markers. It performs no HTTP. Driver owns raw cache, refresh,
  room cache and read-only get_room_mapping access.
- Manipulation state/estimate/frame reads use the ROS-only Observations adapter.
  Camera selection uses the driver service, including during live pickup. Deployment
  readiness consumes shared status instead of issuing another get_status RPC.
  Dry-run sends no HTTP or selection and does not read a backend token.
- Perception, VO, mosaic and depth use a shared latest-only image/metadata join.
  Uncalibrated/mismatched/stale images remain available for visualization but never
  enter their metric processing callbacks. Driver acquisition is independent of
  subscriber count; ordinary frame requests send exactly `{}` and never select or
  restore a camera.

## Evidence semantics

- RobotState distinguishes connected transport from telemetry, exposes numeric
  common_status/grab_status only when supplied, and carries explicit known/value
  pairs for the existing in_cleaning/in_returning hazard flags. The TS status
  sanitizer now preserves the readiness bitfield; no state-code alias is guessed.
- ArmEstimate republishes the TS authority, not a second estimator. Only usable
  M3/M4/M5 command-derived joints enter `/joint_states`; heuristic degree sigma is
  converted to radians, never into fabricated wrist covariance or finger certainty.
  UNKNOWN/restored estimates are not planning state; command-derived stow is
  DISPLAY_ONLY with COMMAND_DERIVED_STOW, not observed physical stow. Folded/finger
  visuals go to `/z70/display/joint_states`. Existing cached TF is not validity.
- An unchanged estimate, including recovery of the same cached update after a
  data gap, retains its original source stamp/identity. Backend integer milliseconds
  are converted exactly, avoiding float-roundoff-induced false updates. Missing or
  stale state also marks arm availability stale; cached model data is not fresh
  transport evidence.
- Images/raw/JPEG/CameraInfo/FrameMetadata share acquisition stamps/frame IDs.
  CameraInfo is emitted only for matching dimensions and records the calibration
  file SHA-256. Receipt is host time; exposure and device source time remain unknown.
- Frame session_scope=DRIVER_BACKEND_PROCESS explicitly says the backend header
  identifies a service process, not a vendor preview. UUID namespace includes a
  fresh driver epoch and backend process epoch; restarts cannot reuse sequence
  identities. Counters are per stream/session. Cold-preview/vendor stream identity
  remains unknown. Backend command events retain their actual backend session UUID.
- Map cache receipt is retained, not relabeled as capture time or localization.
  Unchanged cache content preserves evidence; late subscribers receive the original
  typed map/state samples. Refresh errors preserve last geometry and mark gaps.
- Camera selection confirmation is separate from delivered-lens frames and from
  app_change_camera reply evidence. The selector HTTP route does not expose that
  wire outcome: its ROS wrapper therefore returns UNKNOWN/NONE with confirmed
  CameraState rather than inventing a vendor reply. These wrapper request UUIDs
  are not presented as underlying vendor-attempt identities. Full command/service
  outcome correlation is part of the phase-3 dispatcher cutover.

## Dispatch-time command stream

`GET /api/commands/events` is authenticated NDJSON, live without replay. Events
originate at TS dispatch, not recent-history polling. All accepted vendor methods,
including advanced raw RPC, share the stream. Logical command UUID survives the
backend's one safe retry; each actual attempt has a new UUID/index. Session and
sequence detect discontinuities. Dispatch-call start is NOT confirmed wire time:
write_time_known=false and physical_completion=UNVERIFIED throughout.

Only closed categories, numeric vendor codes and bounded numeric RC inputs are
exported; arbitrary params/results/errors/credentials are not streamed. Blank
heartbeats carry no command evidence. Slow readers are bounded/disconnected, and
service shutdown closes subscribers. `/api/commands/recent` remains compatibility
history but has no ROS consumer.

The chassis calibrator joins DISPATCHED/REPLIED by attempt/session, never counts
FINISHED twice, rejects unknown dispatch, and drops pending pairs on stream gaps.
Its approximate time origin is dispatch-call start, not guaranteed motor onset.

The phase-1 instruction removing advanced generic RPC was corrected: phase 3
RETAINS advanced raw RPC through the SAME ROS dispatcher/events/estimator, without
an active-action bypass. Operation width is 128 so accepted method identifiers
are never silently truncated. Other coordinated contract additions are explicit
hazard known/value fields and frame session_scope; no new permission layer exists.

## Verification and limits

Actual retained outputs:

| Check | Result | Evidence |
|---|---|---|
| TypeScript suite | 372 pass, 0 fail | phase2-npm-test.txt |
| TS noEmit + web typecheck/build | pass | phase2-npm-build.txt; explicit npx tsc -p tsconfig.json --noEmit also passed |
| Full normal Pixi/colcon ROS suite | 136 pass, 3 existing expected failures | phase2-ros-test.txt |
| Fresh non-symlink colcon build | all 9 packages build | phase2-clean-build.txt |
| Installed driver/map/odom/compat suites | 48 pass | phase2-installed-driver-test.txt |
| Full non-symlink workspace investigation | 10 fail, 126 pass, 3 expected failures; existing resource-path defect below | phase2-clean-test.txt |
| Whitespace check | git diff --check passed | terminal execution |

The real-DDS fake-backend test verifies late-subscriber typed state/raw map and
decoded OccupancyGrid, two readers sharing one acquisition, front/gripper service
selection and tags, matching CameraInfo/stamps, unknown exposure/session scope,
actual serialize/deserialize, stale/error and calibration-mismatch reporting,
restart epochs, display-only stow, command identities/gaps/duplicate filtering and
joined owned workers. Separate tests serialize every generated message/service,
check exact millisecond identity, command retry pairing, read-only delegation,
and production source observation ownership. HTTP geometry fixtures remain injected
synthetic observation providers for existing executor tests; they are not production
HTTP observation fallbacks and are not the shared-driver integration proof.

The three expected failures are pre-existing provisional optical-model checks,
explicitly retaining the historical 12px tolerance. No tolerance was widened and
no physical accuracy, repeatable pickup, current localization or camera throughput
is inferred from these tests.

The extra full installed-workspace experiment found 10 failures caused by existing
`Path(__file__).resolve().parents[4]` executor paths used to import front-camera
calibration code. Those same expressions exist at HEAD in jog/front_measure/
held_front. The clean driver/map/odom/compat suites pass; full resource ownership
and removal of these executor assumptions are already in phase-3 card
`t_16c0ca97` and the exact failure was posted there. This is NOT a globally passing
non-symlink workspace, nor a reason to silently expand this task into calibration
resource migration. An incremental native rosidl reinstall can also print
install_name_tool missing-RPATH diagnostics; the fresh non-symlink build has none.

## Explicit remaining transitions

Phase 3 (`t_16c0ca97`) owns authoritative ROS dispatch/estimation, command-only
executor HTTP removal, advanced raw RPC via that same dispatcher, stop/cancel/
takeover, action/service outcome correlation, remaining typed consumer cutovers,
installed calibration resources, and a separate display TF tree. Legacy Pick,
object JSON and odometry JSON status remain coordinated future cutovers, not
claimed typed replacements here. Current MCP/browser still talk to TS; they are
not yet ROS-only clients. Existing TS estimator semantics were not rewritten.

Root directory moves remain phase 5. Later, separately authorized live camera
checks must measure cold/warm throughput and owned-preview release; no such check
was run. No existing controllers or observation sessions were interrupted.

Collision hotspot: executor.py and test_executor.py already had concurrent
uncommitted pickup/geometry changes. Targeted observation/deploy-read edits preserve
those changes; do not commit the whole dirty workspace as if it belonged to phase 2.
