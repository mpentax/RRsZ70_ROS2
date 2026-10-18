# Repository layout, exact path map and cutover decisions

Phase-5 implementation supersession: the approved cutover has moved the checkout
to `/Users/mpentax/Code/RRsZ70_ROS2`, ament packages to root `src`, TypeScript vendor
code to `adapters/vendor`, UI/MCP to `apps/web` / `apps/mcp`, and deployment to `deploy`.
The small transport-free `packages/http-contracts` npm workspace prevents MCP from
importing vendor camera/robot internals. Root npm workspaces use one lockfile.
See [phase5.md](phase5.md) and its realized path inventory. The phase-1 snapshot
below is retained as historical plan/provenance, not a current relocation deferral.

Phase 1 is documentation/contracts/inventory only. No physical moves, deletion, checkout relocation, service operation or hardware contact occurs here. The repository is named **RRsZ70_ROS2**; **RoboClaw** remains the UI name and `z70_*` remains the ROS package prefix. Checkout stays `/Users/mpentax/Code/RoboRockZ70-Connector` until coordinated directory cutover. Uncommitted calibration, description and manipulation work is outside this phase's ownership.

## Final layout (decision, not alternatives)

```text
RRsZ70_ROS2/
  pixi.toml, pixi.lock                 native RoboStack Jazzy workspace
  src/
    z70_interfaces/                   ament_cmake + rosidl, msgs/srvs/actions
    z70_driver/                       ament_python; sole private-backend client
    z70_description/                  model, FK, adopted mounts/lens + mesh resources
    z70_odometry/                     floor VO, mosaic, chassis calibration
    z70_mapping/                      vendor map decode + optional DA2 Small worker
    z70_manipulation/                 perception, planner, verified discrete pickup
    z70_bringup/                      installed launch and Foxglove/RViz resources
    z70_web_gateway/                  rclpy HTTP client gateway, shared task handles
  adapters/vendor/                    private proven TS transport/auth/cache/go2rtc adapter
    src/, package.json, package-lock.json, tsconfig.json
  apps/web/                           React/Vite RoboClaw UI, tests/config
  apps/mcp/                           thin TS stdio MCP protocol shim, shared web API client
  deploy/                             Compose, Dockerfiles, go2rtc config/patches
  calibration/                        retained evidence, acquisition/analysis, writable outputs
  docs/                               current docs; migration/ binding decisions
    archive/                          explicitly historical notes
    claude-review/                    dated source audits, not current runtime policy
  research/                           offline protocol/depth experiments, never production imports
  tools/                              offline maintenance only, not a new operator CLI
```

`z70_driver` evolves the current bridge; `z70_interfaces` replaces `z70_manipulation_interfaces`. There is no parallel long-term `z70_bridge`. Keep transport TS private during migration, not a Python rewrite, DimOS revival or ROS distribution upgrade. Phase 4 retains the existing thin TS MCP protocol shim rather than a second Python MCP/ROS process; both surfaces use the same ament gateway registry. This supersedes the original z70_mcp target only, with the shim in apps/mcp at phase 5. Root colcon `src` must not contain legacy TS modules after directory cutover. Standard `ros2 launch`, `colcon`, Pixi tasks and `docker compose -f deploy/compose.yaml` remain visible tools. No custom launcher, doctor, diagnostic agent, leases or approval flow.

## Exact path map

`path-map.json` enumerates every path in the `tracked-assets.json` index snapshot exactly once. Each row has `old_path`, `destination`, `operation`, `cutover` and `rule`. `destination=null` means conditional retirement, never a deletion performed here. `split` is a deliberate extraction rather than a filesystem rename; this table resolves its owners. JSON mapping covers index membership only, not untracked or ignored files. Concurrent new geometry files are listed below and must be added to the directory-cutover inventory when accepted by their owning worker.

| Current path/group | Destination and required handling |
|---|---|
| `ros2/src/z70_bridge/` | `src/z70_driver/`; rename Python module/resource marker/package/setup/entrypoint/imports. Bridge acquisition goes to driver; dashboard ROS presentation stays a driver module initially. Remove legacy HTTP history polling after typed dispatch events exist. |
| `ros2/src/z70_manipulation_interfaces/` | `src/z70_interfaces/`; update Pick imports, CMake/package deps and regenerate, not stale copied build artifacts. |
| `ros2/src/z70_{description,odometry,mapping,manipulation,bringup}/` | Same packages under root `src/`, preserving internal relative paths and installed resources. |
| `ros2/pixi.toml`, `ros2/pixi.lock` | Root names; preserve the tracked lock and update base paths, activation `Z70_REPO`, test base dir and launch parameters. |
| `ros2/foxglove/` | `src/z70_bringup/foxglove/`; install layout JSON via ament share, generator/test stay package development source; historical live validation remains evidence, not a launch resource. |
| `ros2/README.md` | `docs/ros2.md` at directory cutover; root README links updated then. |
| `web/` | `apps/web/`; independent npm manifest/lock generated at web cutover; root current package.json split rather than blindly moved. Preserve tests, Vite root/output/proxy and TS imports. |
| `src/mcp-{main,server,actions,reload}.ts` and MCP tests | Phase-4 update: thin TS protocol-only shim → `apps/mcp/`, calling the same authenticated ROS web gateway. No separate z70_mcp process or duplicated task registry. Resolve shared HTTP-route validation dependencies explicitly; reload watcher must not remain dependent on the old TS root. |
| `src/service.ts`, `service-main.ts`, `service-config.ts`, `service-auth.ts` | Private transport/auth façade in `adapters/vendor/src/`; extract public ROS-aware HTTP/websocket handling to `src/z70_web_gateway/`. Browser advanced raw RPC is retained through the SAME ROS dispatcher/events/estimator, never a second direct-vendor bypass around active actions. |
| `src/arm-model.ts`, `arm-estimator.ts` + tests | Extract authoritative semantics to `src/z70_description/` and driver estimator (phase 3); retain compatibility TS until tests/consumers switch, then retire duplicate implementation. Existing Python model is the starting point, not a second authority. |
| `src/arm-joint-protocol-research.ts` | Split: `ManualJointControl` and `MANUAL_JOINT_CONTROLS` → `adapters/vendor/src/arm-joint-protocol.ts`; fixtures/parser/research flags → `research/arm-joint-protocol/fixtures.ts`. Update both command importers and research tests before deleting old file. Current runtime importers: `src/vendor-commands.ts:1–2`, `src/vendor-rpc.ts:1`. Do not delete production constants with research scaffolding. |
| `src/camera-observation.ts` | Split `jpegImageDimensions` to private adapter utility (MCP may use equivalent client parsing); calibration/projection semantics to ROS packages. Keep legacy module until export-by-export consumers move. |
| `src/telemetry.ts` | Split production numeric sanitization into adapter; typed telemetry semantics into driver. Standalone test-only class is conditional retirement, not runtime authority. |
| `src/*-research.ts`, research-only `src/map-raster-fixture.ts` | `research/legacy-ts/`; rewrite research/test imports. Arm protocol file is excluded and split as above; map raster fixture has service test consumer to update. |
| Remaining production `src/*.ts` + tests | `adapters/vendor/src/` initially; exact files in JSON. This is conservative retention, not a claim every file is production. Conditional utility removal below applies at its phase. |
| Root `package.json`, `package-lock.json`, `tsconfig.json` | Split into `adapters/vendor/` and independent `apps/web/` manifests/tooling; no root custom command wrapper. Preserve engine Node ≥24, SDK versions and reproducibility until consumers migrate. |
| Root `Dockerfile`, `docker-compose.yml`, `.dockerignore` | `deploy/Dockerfile`, `deploy/compose.yaml`, `deploy/.dockerignore`; update build context/dockerignore selection together (Dockerfile-specific ignore if required with root context). Context stays repo root, mount paths anchor to Compose file; never let private state enter build context. |
| Existing `deploy/*` | Stay in `deploy/`; update offline COPY/import/check paths, no live checks run as part of inventory. |
| `calibration/front-camera/floor_geometry.py` | Extract runtime module to `src/z70_description/z70_description/floor_geometry.py`; retained analysis scripts import packaged helper or a compatibility shim after tests. No immediate file move. |
| `calibration/front-camera/front_camera.json`, `z70_front_camera_info.yaml` | Keep original evidence/export at current paths; copy adopted runtime snapshots to installed `src/z70_description/config/` with provenance and digest. Runtime uses ament package share and explicit override path, not repo parent indexing. |
| `calibration/chassis/online.json` | Preserve existing writable state/evidence; calibrator output parameter remains explicit. Optional initial runtime seed belongs to package config but must not overwrite ongoing recordings. Do not write into read-only installed shares. |
| `calibration/pointcloud/` | Preserve evidence/results now; extract prototype code/README to `research/depth/` at cleanup after correcting result/calibration paths. Keep optional ROS DA2 Small path. No weight deletion: none identified in index. |
| `calibration/arm-validation/.venv/` | Conditional retirement only after replacement environment recipe/dependency/license provenance and offline analyze/report reproduction. All exact paths in asset inventory. No physical removal/untracking now. |
| Other `calibration/`, `.local/`, home weight cache, ongoing recordings | Stay put; private/untracked contents are not inventoried or moved here. Do not interpret a root checkout rename as permission to relocate evidence/state independently. |
| `docs/`, `research/`, notices/license, git metadata config | Retain current/history distinction and update active links at cutover; preserve dated historical paths in archived evidence. No history rewriting. |

Concurrent untracked package resources observed at task start: `ros2/src/z70_description/config/pincher_geometry.yaml`, `urdf/pincher_collisions.xacro`, `z70_description/pincher_geometry.py`, description tests and `z70_manipulation/test/test_operational_geometry.py`. Destination is corresponding root package path only after owner acceptance. They are not in the index snapshot and must never be silently discarded by index-only migration.

## Keep / extract / delete-at-cutover decisions

- KEEP proven LAN/cloud codecs, account/PIN web plumbing, secret storage, go2rtc patches/config, raw map acquisition and safe transport-outcome evidence in private adapter. Runtime source and imported dependencies stay until consumers move.
- EXTRACT semantic state/estimator/model/metric projection into ROS, auth input rendering into web flow, ROS-aware gateways into ament. Extraction is verified before duplicate retirement. Tests follow actual behavioral owners, not legacy format assertions.
- EXTRACT proven arm constants before retiring the misleading research filename. Preserve 0/1/2 directional wire values; never turn them into absolute joint positions.
- RETIRE direct-device status/probe/map/room/camera-login CLI entrypoints after equivalent needed account/device/PIN input is handled in web and status is exposed via ROS. Do not replace them with a new CLI. Retain protocol evidence in research. `open-ui`/service-access Docker credential extraction is not part of the final setup path.
- RETIRE standalone unused photo/vision/feedback/diagnostic/preview implementations only after runtime and test consumers are explicitly removed/replaced. See runtime inventory export-level distinctions; `photo-protocol.ts` is production map dependency, not in this deletion group.
- RETIRE tracked `.venv` and generated dependency binaries only under the replacement/reproduction condition in asset inventory. Never blanket-delete `.stl`, URDF snapshots, calibration JSON, depth code or result evidence.
- KEEP Depth Pro/DA2 Base experiments as research until separately retired; KEEP optional DA2 Small persistent ROS worker/MPS path and external environment. No claim of metric accuracy from finite point-cloud output.

## Ordered cutovers and regression gates

1. Phase 1: this map + interface contract + source/asset inventory; correct stale current docs. Nothing moved/extracted/deleted in source yet.
2. Phase 2: establish driver/interfaces and one acquisition pipeline in current `ros2/src`; migrate mapping/manipulation observations to topics/services. Preserve transitional executor command HTTP explicitly. Test typed serialization, late subscribers, acquisition identity and shutdown against fake backend.
3. Phase 3: ROS bounded actions/priority stop, command events and one authoritative estimator; replace executor command HTTP; thin MCP/web gateway. Stop path must work during blocked command/camera/inference and no direct vendor clients remain in public control paths.
4. UI/setup: account/device/PIN via web, ordinary startup/shutdown/recovery documented in standard tools. No custom CLI. Diagnostics are closed numeric/enums and bounded deterministic errors.
5. Directory/cleanup cutover: freeze relevant editors, inventory all dirty/untracked work with owners, regenerate index map, move TS out before moving ament packages into root src, update installations/imports/resources/manifests/deploy/docs/MCP/board references atomically. Test before conditional deletion. No deployment/robot check without separate authorization.
6. Offline acceptance: clean install-space build (not just symlink source), no repo-parent imports, TS test/typecheck/web build and whole ROS suite; ament share resource resolution, fake stop/cancel and shared observation; standard launch argument listing only if it makes no live requests. Authorized live checks later report physical outcomes independently.

## Rename impact checklist

Remote rename is complete: source inspection verified `origin` fetch/push `https://github.com/mpentax/RRsZ70_ROS2.git`. No remote changes needed here. Local checkout relocation is deferred, not a naming decision still open.

| Surface | Phase-1 finding / cutover requirement |
|---|---|
| Git remote/hosting | Preserve current remote; repository README title/name updated now. Old external links redirect today but active docs should use approved URL. Never rewrite archive evidence as if it used the new name then. |
| CI | No `.github/` paths in tracked index at this snapshot. When workflow automation exists/adds, update checkout/build working directories, caches, path filters and artifact names. Do not claim CI passes from local docs checks. |
| Documentation | Root/workspace current docs distinguish actual ROS executor from target ROS driver. `docs/claude-review/` remains dated audits with supersession notice; `docs/archive/` remains history. Search active docs for old repo URLs and root `src/mcp-main.ts` paths at cutover. |
| Hermes MCP | Existing config command/args/token-file absolute paths and watcher source glob pin checkout/source. Phase 4 retains the TS stdio shim; update entry/source-watch paths to apps/mcp at directory cutover, keeping server name `roboclaw` and token values out of config/args. Do not modify user configuration without its authorized cutover. |
| Compose | Moving Compose into deploy changes relative `.local` and YAML mount bases; set `../.local`/`./go2rtc.yaml` as appropriate while preserving actual state volume, port 57010 and media 57012; admin 57001 stays private. Pin Compose project `roboclaw` to avoid parallel stack/volume creation. Dockerfile paths/COPY/offline validators change together. |
| Pixi/colcon | Root workspace activation/build/test/launch/resource paths replace ros2-relative paths; regenerate no environment silently. Stop only owned test nodes; do not interrupt active controllers. |
| Board workspaces | Phase 5 must update every dependent migration card workspace/body absolute checkout reference when physical relocation occurs; inspect cards and active run ownership first. No DB edits outside tools. Existing phase-2 card remains pointed to current checkout until then. |
| Acquisition/evidence | Update reusable scripts to installed clients/helpers; historical session paths stay evidence. Preserve hashes/originals and all untracked recordings; no moving private state or cache for tidiness. |
| User launch references | Update operator-owned shell shortcuts, desktop open-project paths and local worktrees only during coordinated relocation, with user input for external unknown locations. Not guessed or silently edited here. |

Acceptance of phase 1 releases phase 2; it does not authorize deployment, camera capture or robot motion.
