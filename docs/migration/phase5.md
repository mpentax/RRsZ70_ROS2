> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

The cutover and replay are historical, not current operator closeout or deployment status. Named missing qualification logs/manifests are unavailable, so numerical results below remain reported history, not newly verified passes. Do not regenerate pre-cutover evidence from HEAD.

**Historical outputs unavailable in this checkout:** `docs/migration/phase5-build.txt`, `docs/migration/phase5-clean-colcon.txt`, `docs/migration/phase5-clean-installed.txt`, `docs/migration/phase5-clean-npm.txt`, `docs/migration/phase5-installed.txt`, `docs/migration/phase5-mcp-connection.json`, `docs/migration/phase5-native-client-race.txt`, `docs/migration/phase5-network.txt`, `docs/migration/phase5-npm.txt`, `docs/migration/phase5-path-map.json`, `docs/migration/phase5-pixi-test.txt`, `docs/migration/phase5-policy-remainders.json`, `docs/migration/phase5-preserved-evidence.json`, `docs/migration/phase5-research.txt`, `docs/migration/phase5-ros.txt`, `docs/migration/phase5-verification.json`. Results citing them below are reported history only.

## Historical record

# Phase 5 — RRsZ70_ROS2 cutover and offline qualification

Task: `t_220f88b3`. Checkout: `/Users/mpentax/Code/RRsZ70_ROS2`.
Base HEAD: `de27d42dc098308949d6c6f122cadfcd8ac7a337`, branch `claude/cleanup`.
Origin is `https://github.com/mpentax/RRsZ70_ROS2.git`; the old checkout is absent.
Implementation is present and software-tested. The three operator/capability
remainders were resolved by the interactive parent. Umbrella final acceptance
remains separate; no production deployment or hardware acceptance is claimed.

## Layout and ownership

- Root `src/`: eight ament packages: `z70_interfaces`, `z70_driver`,
  `z70_description`, `z70_odometry`, `z70_mapping`, `z70_manipulation`,
  `z70_bringup`, `z70_web_gateway`.
- `adapters/vendor/src/`: necessary private transport, credentials, camera,
  map-wire integration and HTTP facade. It is not a second ROS estimator/executor.
- `apps/mcp/src/`: thin native stdio MCP client of the shared ROS gateway.
- `apps/web/`: React/Vite web controls and account/device/protocol/PIN input flows.
- `packages/http-contracts/`: transport-free camera names, JPEG dimensions and
  canonical ROS HTTP route selection. MCP does not import vendor transport.
- Root npm workspaces, one public-registry lockfile, root Pixi manifest/lock.
  `deploy/` owns Compose/Docker; `research/` owns retained experiments, including
  non-runtime legacy TS fixtures. Non-ament directories carry `COLCON_IGNORE`.
- Immutable calibration/URDF/meshes and the Foxglove layout install with their
  owning packages. Mutable estimator/chassis state stays in durable application
  data; retained measurement archives are not runtime write targets.

ROS retains execution, estimation and shared acquisition. Advanced raw RPC,
manual controls and tasks still use the same dispatcher. No vendor fallback,
second estimator, alternate camera producer, custom operator CLI/launcher/doctor
or new permission/lease layer was added. Superseded bridge/interfaces, direct
CLI/onboarding/probe paths, unused TS vision/preview machinery and standalone
Depth Pro point-cloud prototype were retired after tracing consumers. Optional
ROS depth remains separate; its hardware/metric accuracy was not qualified here.

## Preservation and cleanup

`phase5-path-map.json` covers all 3,393 base-HEAD paths: 209 moved/extracted,
553 retained, 50 retired and 2,581 environment entries removed. This is a
cumulative mapping of the existing multi-phase working tree, not a claim that
all prior-phase changes originated in this card. New files and edits remain
uncommitted; do not reset/clean the tree. Only the known environment index removals
were staged; other multi-phase work was preserved.

The old `calibration/arm-validation/.venv` is absent from checkout and index.
Dependency metadata/licenses with seven verified SHA-256 records are retained in
`retired-environment/`. Recursive deletion was denied by the headless policy, so
the environment was moved recoverably to
`~/.Trash/RRsZ70_ROS2-arm-validation-venv-t_220f88b3`; no shared cache was deleted.
Ignore rules prevent dependency binaries/checkpoints from being recommitted.

No tracked checkpoint or LFS pointer was found. Git LFS is not installed here;
canonical pointer headers were inspected directly in HEAD and the index using
Git, rather than treating the missing extension as an empty inventory.

`phase5-preserved-evidence.json` hashes 2,745 retained calibration files. It is an
explicit post-cutover inventory, not an invented pre-cutover image snapshot.
Six installed frozen resource hashes also match their provenance manifest.
Raw observations and prior results remain retained; new replay outputs go only
under `calibration/arm-validation/phase5-replay/`. Replay parsed 84 records with
zero parse exclusions, retained five command deltas and excluded 30 evidence
rows; its conclusion remains inconclusive. M3/M4 replay covers twelve increments
in four groups; mean changes against the historical report are approximately
1.6e-14 degrees or smaller, not new mechanical measurements or lens adoption.

## Qualification results

The machine-readable summary is `phase5-verification.json`. Canonical logs:

- `phase5-npm.txt`: 285 active workspace tests pass.
- `phase5-build.txt`: typecheck and production UI build pass.
- `phase5-research.txt`: 63 research tests pass, including moved legacy camera tests.
- `phase5-clean-npm.txt`: fresh `npm ci`, build and 285 tests in the clean-source export.
- `phase5-pixi-test.txt`: 195 passes and three existing optical expected failures.
- `phase5-ros.txt`: the same 195 passes/three xfails against the non-symlink install.
- `phase5-clean-colcon.txt`: eight-package non-symlink build with cwd explicitly
  inside `build-phase5/clean-checkout`, fresh `build-qualified`/`install-qualified`.
- `phase5-installed.txt`, `phase5-clean-installed.txt`: nine installed module paths
  including generated odometry interfaces, six resource hashes, real CDR
  serialization, default front-lens fallback, installed xacro and Foxglove layout
  resolve outside the checkout with `Z70_REPO` unset. Launch arguments also load.
- Full ROS tests include an actual standard `ros2 launch z70_bringup bringup.launch.py`
  against a fake backend on DDS domain 94, test-owned XDG data/logs and a synthetic
  1600x1200 image. State, image, map, typed calibration/odometry arrive, followed
  by bounded ordinary SIGINT shutdown. No live token/backend is used.
- Existing real-DDS/MCP/HTTP/rendered React tests still exercise shared goal UUIDs,
  cancellation both directions, UNKNOWN outcomes and priority stop while stalled.

The clean export excludes private state, node_modules, bytecode and environment
artifacts; its dependency environment is created from the root lock. This tests
the current uncommitted source, not an assertion that remote HEAD already
contains the migration. Historical image-based accuracy requirements remain
separate; no optical tolerance was widened to force these tests green.

Launch qualification exposed a remaining String/typed odometry topic mismatch.
The publisher, online calibrator and dashboard now use `OdometryStatus` with
matched-frame evidence. Explicit known/applied-camera-offset fields preserve
what each pair actually used, not a later fit. Initial/held poses are not new
measured motion. A real takeover test also exposed destruction of per-task ROS
clients while the shared executor enumerated QoS waitables. Command transport
entities are now node-owned; task-local UUID/cancellation/outcome state stays
separate and old task retirement cannot affect a replacement. Repeated takeover
and image-stall/cancel/deadline tests pass. The original native failure trace is
retained in `phase5-native-client-race.txt`, not presented as a final result.

Some native incremental builds emit install_name_tool RPATH-cleanup stderr,
and rclpy sometimes prints timer-jump teardown diagnostics. Build/test exit
statuses and installed import/CDR checks are retained; these messages are not
silently described as a warning-free run.

## Container/network/MCP evidence

Both qualification images build. Network-disabled, nonroot/read-only app and
patched go2rtc startup/shutdown fixtures pass. FFmpeg generates a synthetic
1600x1200 JPEG; this proves codec operation, not robot video. Old -> new -> old
app image rollback preserves the SAME synthetic token and hold bytes, with no
device and no network. Test containers/volume are removed afterward.

`phase5-network.txt` proves Docker Desktop can fetch an isolated native loopback
fake gateway using `host.docker.internal`; the native gateway stays loopback-only.
No LAN widening or vendor bypass was added. This is Mac Docker Desktop evidence,
not Linux networking qualification. Compose references now resolve from `deploy/`.

Supported Hermes configuration now points at the new absolute Node/MCP entry,
with the relocated existing private token-file reference and no explicit cwd.
Default MCP/private-facade paths are module-relative. `hermes mcp test roboclaw`
initializes and discovers fourteen tools both from the checkout and outside it.
Supported `manage_connections enable` also connected this worker's pool; the
`get_state` schema was described, never invoked. No robot tool was called.
See `phase5-mcp-connection.json`; worker-pool connection is NOT proof of Desktop
in-memory pool reload.

## Operator closeout

The interactive parent resolved all three remainders; see
`phase5-policy-remainders.json` and `phase5-verification.json`:

1. Supported `manage_connections enable` in the Desktop session returned
   `connected` and discovered 14 RoboClaw tools. `tool_describe` resolved
   `get_state`, `start_task` and `stop` without invoking any robot tool. This
   closes the earlier worker-pool-only discovery limitation.
2. The parent updated this card's workspace through the installed application
   helper and independently read back `/Users/mpentax/Code/RRsZ70_ROS2`. The
   dependent final audit remains scratch-based and must inspect the actual repo.
3. The isolated Compose fixture ran interactively: app checks passed and both
   containers exited with code 0. After explicit user approval, `down --volumes`
   removed only the fixture containers and test volume; subsequent readback
   found none remaining. The fixture used no network or published ports and
   test-owned state only. Historical approval denials were not bypassed.

For that isolated fixture only (NOT production deployment):

    docker compose --env-file /dev/null -f deploy/compose.yaml -f deploy/phase5-offline.compose.yaml up --no-build --abort-on-container-exit --exit-code-from app
    docker compose --env-file /dev/null -f deploy/compose.yaml -f deploy/phase5-offline.compose.yaml down --volumes

No production deployment/restart, robot/camera access or history rewrite occurred.
The existing production app was observed running with restarts=0 but its old
bind-source path; it has NOT adopted this source. A later separately authorized
rollout must use the relocated Compose file and rebuild/restart matching ROS
interfaces/packages together. Camera switching, uncertain-dispatch recovery,
cancellation/physical stop and pickup regression still require separately
authorized real observations. Offline passes do not establish physical cessation,
metric geometry/depth accuracy, localization or repeatable autonomous pickup.
