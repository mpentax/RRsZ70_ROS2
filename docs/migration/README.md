# RRsZ70_ROS2 migration — contracts and implementation evidence

Directory cutover is implemented by phase 5 (`t_220f88b3`): see [phase5.md](phase5.md)
for the current root layout, measured qualification, preserved limitations and
relocation to `/Users/mpentax/Code/RRsZ70_ROS2`. The phase-1 through phase-4 inventories
and verification commands below are dated evidence of their original source tree,
not commands to regenerate those snapshots against the restructured checkout.

Task `t_7fd489d5`. This is the binding phase-1 architecture/interface/layout handoff for the authorized migration, not another proposal to start. Source snapshot: `f0484c2d594ed035240210b5795368272fa6c45f` plus concurrent working-tree changes, which remain owned by their existing workers. No source-directory relocation, runtime deletion, deployment, robot/camera access or private-state reads were performed.

Phase-2 implementation task `t_f0121e04` is documented in [phase2.md](phase2.md).
The generated observation interfaces and shared driver now exist in `ros2/src`;
phase-1 source citations below remain an explicitly historical pre-cutover inventory.

Phase-3 implementation `t_16c0ca97` is documented in [phase3.md](phase3.md):
ROS command/estimate authority, priority stop/cancel/takeover and installed
calibration resources. Phase-4 implementation `t_6d59fe24` is documented in
[phase4.md](phase4.md) and [phase4-api.md](phase4-api.md): shared authenticated
ROS gateway, real action goal IDs/feedback/results, thin TS MCP shim, React controls
and web account/device/PIN input. Legacy direct writes still fail rather than
bypass ROS. No deployment/live access was performed; phase 5 owns directory and
native/container networking cutover.

## Read in order

1. [Runtime inventory](runtime-inventory.md): current imports, entrypoints, scripts, launch graph and installed resources with source citations. ROS already executes pickup via HTTP; it is not observation-only.
2. [Interface contracts](interfaces.md): single driver, typed state/events/estimates and timestamp provenance, shared acquisition, bounded actions/cancel/priority stop, uncertainty and display-versus-planning separation. Generated ROS interfaces and shared MCP/web clients are implemented offline through phase 4.
3. [Layout and cutover](layout-and-cutover.md): settled package ownership, split/extract rules, rename impact checklist and phase order. Repository name is RRsZ70_ROS2, UI RoboClaw, package prefix z70_*. Checkout stays at its current path until coordinated phase-5 cutover.
4. [Asset inventory](asset-inventory.md): indexed model/dependency/calibration assets, actual depth consumers and conditional cleanup rules.
5. [Exact tracked assets](tracked-assets.json) and [exact path map](path-map.json): all indexed paths, not a sample or glob-only approximation. Index metadata is not a claim about concurrent dirty/untracked/ignored data.

## Implementation decisions downstream must preserve

- Phase 2 evolves `z70_bridge` into `z70_driver` and adds `z70_interfaces` within the current workspace. Driver owns all observation HTTP; mapping and manipulation read ROS observations. Existing manipulation command HTTP remains an explicit exception only until phase 3.
- Phase 3 consolidates command semantics/estimation in ROS, bounded actions, event identities, cancellable tasks and priority stop. Advanced raw RPC is RETAINED through the same ROS dispatcher/events/estimator, without bypassing active actions. Phase 4 implements one ament rclpy gateway shared by web/MCP; the existing TS stdio MCP is a protocol-only shim, not another ROS process or state owner. TS transport stays private.
- Final source layout is root `src/` for ament, `adapters/vendor/`, `apps/web/`, `deploy/`. Move only at directory cutover after active editor/recording coordination. Runtime resources install with their packages; evidence stays evidence, writable calibration stays writable.
- Extract proven arm wire constants before removing research scaffolding. A label containing “research” is not a dependency audit. No broad deletion of depth, calibration evidence or model resources.
- No custom CLI, launcher, doctor or diagnostic agent. Standard Pixi/colcon/ROS/Compose, web input flows and bounded deterministic status/errors. No new lease/approval bureaucracy.
- Hardware/deployment require separate authorization. Passing offline checks is not physical validation, camera recovery, accurate geometry or repeatable grasping.

## Current cutover audits

From the current repository root:

```sh
node --test scripts/migration-audit.test.ts scripts/phase5-preservation.test.ts
git diff --check
```

These are offline maintenance tests, not an operator runtime/doctor CLI. They read source/index metadata and hash retained calibration evidence, never private runtime state or services. Current artifacts are `phase5-path-map.json` and `phase5-preserved-evidence.json`; see [phase5.md](phase5.md) for measured results and remaining operator actions. The older `build_path_map.py`/`verify_phase1.py`, `path-map.json`, `tracked-assets.json` and [verification.md](verification.md) describe their original pre-cutover snapshot: do not regenerate that baseline against the restructured tree. Neither index metadata nor passing software tests prove physical calibration, camera operation or pickup repeatability.
