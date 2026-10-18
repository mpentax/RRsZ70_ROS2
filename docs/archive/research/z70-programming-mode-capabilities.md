> **Historical / superseded — retained in place.** Current owners: [ROS guide](../../ros2.md), [command/API contract](../../thin-api.md), [arm model](../../z70-arm-model.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

Later [editor recovery](android-ui-program-recovery.md) and [live trial](z70-programming-live-trial-2026-10-06.md) partly resolve the missing editor/firmware-acceptance discussion. AppStop maps to app_stop; full Blockly save, physical step size and stop/drain scope remain distinct. This is not an active DimOS integration plan.

External HASM/editor inputs below are private-owner provenance, unavailable from the public checkout; full original replay needs the authentic matching inputs. No private files were opened or copied during this curation.

## Historical record

# Z70 programming mode: candidate native discrete-arm interface

Research date: 2026-10-06. **Offline only.** Read primary Hermes instructions and local archive directories; did not execute the plugin, contact robot/camera/cloud, change modes, deploy software, or commit/push. Only this document is owned/edited by this workstream. Existing Oct 3 research was read first and its relevant primary windows re-read.

## Practical conclusion

The user's “M4 upward” programming action is a credible missing interface for **discrete relative joint actions and robot-side sequences**, not just a decorative editor. Android has a separate programming-session RPC, an executable `cmd_ids` object array, submission wrappers and runtime reporting. A DimOS skill adapter can plausibly use these without implementing full IK. **This does not establish absolute joint targets, calibrated increments, Cartesian trajectories, independently measured completion, or firmware acceptance on the user's device.**

Two useful additions to Oct 3 findings:

1. Programming mode is **`app_program_enter_exit {flag:1}` / `{flag:0}`**, not `app_pick_up_enter_exit` flag 2. The manage page enters once on focus and explicitly exits on back-to-home. Its `hasEnteredProgramMode` is a local optimistic latch, not confirmed firmware readiness.
2. The bundle itself resolves `Methods.AppStop` to **`app_stop`** for the ordinary/non-legacy device branch. The earlier “native expansion unresolved” statement can be narrowed: the constant is recovered here; firmware stop scope/queue flush/cessation remain unproven.

## Provenance and bounded fresh artifact search

H references below are **1-based disassembly text lines**, not JS source lines, in:

`/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s/Z70Android/targeted-plugin/index.android.hasm`

Fresh SHA-256 calculation:

| Artifact | SHA-256 |
|---|---|
| `index.android.hasm` (1,518,814 lines) | `5fa398d080e801a31b7d55598ce8c696535808fbbbba6b5bac13c413d7ba4a2c` |
| sibling `index.android.bundle` (previously identified as Hermes v96; not executed) | `bfd30c61c7969195251864d1296f464b732b1aefecd18d0ba86c51e1c511cc67` |
| `Roborock-4.74.08-100936-arm64-v8a.apk` (75,401,267 bytes) | `13dbd693115ab925af5dc6b3b4b3d100d777f6173952a3d9185eb533a5c07ee6` |

Fresh `os.walk` searched `/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s`, bounded to depth 6, 18,000 directories or 65 seconds; excluded `.git`, `node_modules`, trash/library/virtual-environment directories. It completed with **3 directories and 7 files**, not a bound hit. Filename keys: `tanos`, `roborock`, `blockly`, desired asset hash; archive/HTML names additionally selected in Z70/Roborock paths. Found the same APK and two Tanos parser `.jx` files, no newly downloaded editor/plugin package. This is a small available subtree, **not a scan of the entire WD16GOLD01 volume**.

Fresh APK central-directory inspection: **2,312 entries**, no `tanos`, `blockly` or `program/html` name match. Inspected all seven embedded `assets/plugin/**.zip` central directories in memory, no Tanos/Blockly/program HTML/any HTML names: pairing 192 entries; inner-feedback 29; intelligent 64; mower 120; wetdryvac 23; wm 114; vacuum-cleaner 99. No extracted plugin execution or credential/URL dump.

Exact editor asset registration H:1429912–1429929, especially H:1429923:

- `/assets/projects/com.roborock.tanos/Setting/Program/html/index.html`
- Metro asset hash `a346fef5730774bf6e71df54903bc7e4` (**not independently established as a file-content MD5**).
- WebView source is imported dependency index 22 at H:1425296–1425299; message handler H:1425300–1425301.
- RN supplies `LANGUAGE_DATA`, `programData`, `dataSoundList`, app language and localized resources through `window.handleDataFromRN(...)`, H:1424343–1424415.
- Web messages include `__WEB_READY__`, `noCommand`, `btnCancel`; JSON is parsed and forwarded to `dealWithDataFromBlockly`, H:1425335–1425385.

**Matching browser/editor package remains missing after a fresh search.** No public authenticated download URL was constructed/fetched. A matching already-downloaded Tanos plugin directory/archive or a privately exported app cache with this asset and companion JS is the next specific artifact. A similarly named editor from another revision would be comparative evidence only; match bundle/plugin revision and hash metadata before attributing its bounds/generator.

## Session entry, exit and prerequisites

| Path | Primary evidence | Established meaning / caveat |
|---|---|---|
| `appProgramEnterExit(flag)` #5523 | H:230922–230946; export H:224572 | Calls `f("app_program_enter_exit", {flag: suppliedArgument}, {})`; final `{}` is transport options. No clamp, status wait or arm deployment. |
| manage `enterProgramMode` #32779 | H:1431409–1431425 | If local latch false, sets `hasEnteredProgramMode=true` **before** calling `appProgramEnterExit(1)`. |
| manage focus path #32780 | H:1431430–1431450 | Gets program summary, adjusts screen, calls `enterProgramMode`. No pickup-session RPC is in this handler. |
| page wrapper #32635 | H:1423581–1423656 | Awaits entry/exit RPC, logs response/errors; catches errors. No inspected readiness predicate or result parsing. |
| editor exit #32658/#32659 | H:1424235–1424295 | Calls entry/exit with 0, restores UI; first path pops to top. Does not await confirmed exit. |
| manage back-home #32855 | H:1434037–1434058 | Calls flag 0. Separate #32851 H:1433941–1433968 also calls `goToCharge` before flag 0; this is not a universally inert exit procedure. |
| local transport check | H:1424053–1424103 | `localPingWithCallback` populates boolean `isLocal`. |
| start UI local check | H:1424653–1424685 | False `isLocal` shows network dialog and returns; arm-containing sequences receive local predictive validation. This is Android behavior, not a proposed API admission gate. |
| deployment guidance | English resources H:315728; rendered H:1425567–1425578 | “Please add \"Extend robotic arm\" action first … before running the program.” Indicates explicit `arm_out` action in program, not demonstrated automatic deployment on session entry. |

`app_pick_up_enter_exit` is a **different** wrapper #5564, H:231663–231676, `{flag:argument}`; existing manual/viewing research identifies flags 0 off, 1 manual, 2 remote pickup/auto tab. Do not translate programming entry to flag 2 or infer programming mode from common-status grab-mode bits. No dedicated programming-mode readback truth table, PIN/camera prerequisite, stowed-arm requirement, automatic mode exclusion or pose-preserving transition was established in these windows. App-local network checks do not prove firmware transport restriction. Mode entry/exit may have firmware side effects not visible in JS.

## Joint identities and executable records

**Do not renumber these as M1…M5.** Program vector names are exactly **M3, M4, M5, M6**, H:1426985 and H:1427283. `arm_joints.value[0..3]` maps to model `m3,m4,m5,m6`, H:1429772–1429791. M6 is gripper. No M1/M2 programming-vector identity was established; the separate five-input manual wrapper is not evidence that this program has five rotary-angle targets.

English resources H:315728: `M3 Action`, `M4 Action`, `M5 Action`, `Gripper Action`; gripper-position choices Upward/Downward/Forward/Backward. Display #32710 H:1427092–1427193 maps value 1 to upward units, 2 downward units for M3–M5; M6 1 release, 2 grip. Therefore **M4 upward is vector `[0,1,0,0]`** in the traced program model/display. The exact one-axis M4 record is inferred from this mapping, not an extracted M4-only firmware capture.

| Executable `cmd_id` | Proven record/semantics | Evidence |
|---|---|---|
| `arm_out` | `{cmd_id:"arm_out",id:5}` | H:1430234–1430235 |
| `arm_joints` | `{value:[0,0,1,0],id:6,cmd_id:"arm_joints",step:1}` in fixture | H:1430236–1430239 |
| `arm_move` | `{id:7,direction:3,cmd_id:"arm_move",step:1}` | H:1430240–1430241 |
| `delay` | `{cmd_id:"delay",time:3,id:8}` | H:1430242–1430243 |
| `arm_in` | `{cmd_id:"arm_in",id:12}` | H:1430251–1430252 |

Joint enums H:1428502: 0 no move, 1 up, 2 down. End-effector enums H:1428500: 0 no move, 1 up, 2 down, 3 front, 4 back. Local internal enum `ARM_JOINTS:2` etc at H:1428498 is a **model enum, not numeric wire `cmd_id`**. Wire operation names in these records are strings; numeric `id` is an entry identity, not a joint selector.

Blockly palette/type names `arm_m3`, `arm_m4`, `arm_m5`, `arm_m6`, `arm_breakrecover` occur H:1423020 / H:1430068; Blockly `arm_m4` nodes H:1430426–1430430. They are **not thereby proven executable `cmd_id` values**. Executable fixture/model use `arm_joints`; do not submit palette types as RPC commands without generator evidence.

### Relative, repeat, delay and timing limits

`step` becomes model `n_rep` H:1429777–1429791; model repeats `i<n_rep`, selecting first nonzero joint in M3→M4→M5→M6 priority, H:1429231–1429283. Local model changes M3/M4/M5 by ±10° per iteration and M6 predicted opening by 90/0; end-effector model shifts x/y by ±0.04 then solves IK, H:1429215–1429346. **Those are predictive model constants, not calibrated firmware encoder/displacement evidence.**

No recovered editor numeric min/max/integer contract. Fixture `step:1` proves an example only. Model `maxStep` counts valid predicted iterations until a geometric failure, not universal firmware/UI capacity. No evidence that arbitrary combinations, large/fractional/negative/zero counts or missing fields are accepted. Ordered array construction H:1430224–1430253 and ordered model iteration prove app order, not firmware motion barriers or overlap. `delay.time:3` is established; its firmware time unit, timing accuracy and whether delay starts after motors settle remain unvalidated. No per-joint speed, acceleration, duration, waypoint timestamp or absolute target angle in the arm records. UI loading delays are not motion periods.

## Save/start/runtime/stop are actual wire paths, model preview is not

Freshly re-read wrappers H:231015–231064 forward save/start objects unchanged to `app_save_program` / `app_start_program`; logging stringification does not replace the wire object. Query #5514 H:230734–230758 sends `app_get_program {program_id:id}`; runtime #5515 H:230763–230774 sends `app_get_program_runtime {}`. Manage launch retrieves stored `response.result.cmd_ids` then submits **only `{cmd_ids:...}`**, H:1431940–1432018 (detailed prior trace in `../arm-program-research.md`). There is no JS fan-out into manual jog RPCs: robot receives a program payload.

Save editor-copy path H:1425426–1425491 forwards/constructs `name`, `description`, `cmd_ids`, `blocks`, `program_type`; preset IDs 1/2/3 get name suffix `copy`, no copied `program_id`. `blocks` is editor structure; `cmd_ids` is executable content. Bundled fixtures are preset examples, but the separate live wrappers/consumer path establish more than decorative preset data. Static existence does not prove firmware support on every revision.

Runtime parser H:1427422–1427495 reads `response.result.program_state` and `response.result.result` (command records). Preview polls every **2,000 ms**, H:1427840–1427938, especially H:1427868. Prior directly documented labels H:1425838–1425888: task 0 no tasks, 1 starting, 2 executing, 3 error; 4 edit-again is locally synthesized too. Entry results 0 successful, 1 waiting, 2 executing, −1/−2 failed. Preserve raw values/unknown codes; JS `value || -1` loses state 0 and should not be copied. Last-entry success is not run-correlated full completion. Display defaults/time/speed/angle/distance are not measured encoders. Local `RecordRealPos` copies predicted state, not sensor telemetry.

### Stop expansion newly resolved

Preview stop→API `stop` chain already documented H:1426680–1426749; binding H:225166–225167; #5289 H:226366–226383 sends `Methods.AppStop`, `[]`, transport `{}`. Fresh constant window H:348750–348794 exports `Methods`; ordinary branch object H:348793 contains **`AppStop:'app_stop'`**, assigned/exported H:348794. Nearby branch logic H:348777–348790 recognizes legacy E2/C1 model list, selecting alternate method object H:348791; this report does not generalize ordinary mapping to every model.

Thus ordinary Z70 candidate preview cancel is **`app_stop []`**, not a newly discovered per-joint halt command. Manual UI separately checks capability and uses **`app_stop_grasp []`** else generic stop, H:859124–859155 / H:231069–231080. Neither static path proves arm-only versus chassis scope, cancellation latency, queue flush, delayed-command suppression, physical motor cessation, or mode exit. Treat generic task stop, manual grasp stop and physical emergency stop as distinct until verified.

## One small reconstructed envelope — NOT an execution recipe

This is a **static serialization specimen only**, inferred M4 action using proven vector mapping and fixture keys. It is intentionally not a complete deploy/run/stow procedure and must not be executed from this report:

```json
{"cmd_ids":[{"cmd_id":"arm_joints","id":6,"value":[0,1,0,0],"step":1}]}
```

Candidate method: `app_start_program`. Established: object `cmd_ids` array, string operation, four directional selections, count and fixture-style numeric identity. Unknown: whether single-record program allowed, required session/readiness, required explicit preceding `arm_out`, current deployed pose assumptions, permitted/custom IDs, additional required metadata, numeric step bounds, actual displacement/duration, firmware support, duplicate submission behavior, run identity, callback error/result schemas and stop/drain guarantees. **No claim this specimen is firmware-valid or safe.** Save metadata/Blockly fields are deliberately not invented; saved-program creation is a separate unresolved contract.

## Native DimOS implications (no implementation or new admission blocks)

- Existing candidate command transport is sufficient for a thin **discrete-joint/program-sequence adapter**: preserve original envelopes and expose factual submission ACK/error/unknown plus raw runtime. Controller chooses sequence and supervision; wrapper should not invent leases, IK requirements or gatekeeping policies.
- An initial skill vocabulary can name `M4 up one vendor unit` rather than `move_joint_to(45 degrees)`. Keep vendor units uncalibrated until measured. Separate deploy/stow/session primitives from program submission; do not hide potential side effects behind skill setup.
- Robot-side `arm_move` may offer directional end-effector steps without controller IK, but this is **not Cartesian pose tracking**. Full pose planning still needs calibrated axes, transforms, feedback and collision model; local app IK cannot be advertised as actual arm-state sensing.
- This interface does not by itself replace navigation/SLAM/map feedback. Program fixtures contain chassis relative move/turn records; that is neither independent mapping nor a streaming odometry/scan interface. Keep mapping sensor/pose research separate; do not synthesize public map data or absolute navigation feedback from program preview.
- Runtime polls need run correlation and terminal-contract work before claiming motion completion. A stop ACK is a dispatch outcome, not physical stop proof. No implementation/deployment performed here.

## Exact remaining physical-validation evidence (future separate authorization required)

These are experiment questions, **not authorization to perform actions**:

1. Privately recover matching editor HTML/JS; inspect Blockly field number bounds, step integer conversion, direction/vector generation, repeat expansion versus robot repeat fields and delay unit. Capture only sanitized generated program objects, never credentials/maps.
2. On the actual app/firmware, obtain a timestamped baseline status and runtime, then observe normal app programming entry with arm/workspace physically supervised. Correlate the app's entry RPC and any subsequent status changes/motion; determine whether camera/PIN/manual-mode setup is actually required. Isolate entry from deploy.
3. Use the vendor UI to construct the smallest accepted **single-axis M4 upward, one-step** program with its normal required deploy/stow context. Capture its exact sanitized payload (IDs and omitted fields included), command ACK and runtime transitions. Do not strip context based solely on the specimen above.
4. Independently measure M4 angle/displacement/duration before/after, with timestamped physical observation, and compare step 1 then an independently authorized step 2 trial. Establish axis sign and repeat relationship, limit behavior, collision/clamping/error response and whether adjacent records wait for completion. Do not extrapolate model 10° into hardware calibration.
5. Separately authorize stop trials in a clear controlled setup: program with observable single-axis action/delay boundary; record ordinary `app_stop` ACK, runtime transitions and independent cessation/absence of later queued motion. Test mode exit separately from cancellation. Compare `app_stop_grasp` only if explicitly authorized; establish scope rather than assuming equivalence.
6. After each trial require fresh observed terminal runtime **and** independent physical cessation; preserve ambiguous dispatch/timeout facts, do not auto-replay. Determine returned execution/run identifiers, stale-runtime behavior and whether state 0 means no buffered task. No “queue empty” claim from screen closure, local latch or last-entry result alone.

**Bottom line:** programming mode is a realistic discrete-action integration lead, with a distinct session entry/exit and an ordinary `app_stop` mapping now recovered. It can justify a vendor-unit skill adapter research slice—not a full absolute-angle/Cartesian/IK driver claim. Missing matched editor and live correlated acceptance/calibration/stop evidence are concrete gaps, not evidence that programming cannot work.
