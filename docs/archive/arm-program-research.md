> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md), [arm model](../z70-arm-model.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

Later editor recovery and the 2026-10-06 live trial partly resolve the missing-editor/acceptance questions below; see [editor recovery](research/android-ui-program-recovery.md) and [live trial](research/z70-programming-live-trial-2026-10-06.md). AppStop maps to app_stop; generated programs, runtime reports and physical completion remain separate. Full Blockly save replay and stop/drain guarantees were not established.

External HASM/editor inputs below are private-owner provenance, unavailable from the public checkout; full original replay needs the authentic matching inputs. No private files were opened or copied during this curation.

## Historical record

# Z70 Hermes program serialization, sequencing, telemetry and cancellation

Research date: 2026-10-03. Entirely offline: static Hermes instruction inspection and APK/ZIP directory inspection. No robot, camera, live HTTP, RPC, or motion. Only this document was edited by this workstream. Mode/camera behavior is outside this report.

## Verdict

**Programs are an ordered list of relative action records, with runtime task/result reporting. They are not a proven absolute-joint trajectory API or a measured physical-completion API.** `cmd_ids` contains command **objects**, not just numeric IDs. The primary Android meanings are `arm_joints` = four directional selections plus repetition count, and `arm_move` = end-effector direction plus repetition count. Do not swap these names based on the earlier adapter display implementation.

The decisive additional findings are:

- `maxStep` is a **local predictive checker's count of accepted model iterations before failure**, not firmware step capacity or executed-step telemetry.
- The preview polls runtime every **2,000 ms**, retains command fields and adds display defaults; it does not read encoders or compare measured pose against a target.
- The program preview's stop calls generic **`stop()` / `Methods.AppStop` with `[]`**. The manual manager separately selects **`app_stop_grasp []`** using a support capability. These are distinct traced call paths; neither supplies physical stop verification.
- Local controller cancellation revokes subsequent authorization but does not cancel an already submitted robot program or prove that arm motors stopped.
- The nearby APK and all seven directly embedded plugin ZIP directory listings do **not** contain the referenced Tanos program HTML/Blockly asset. UI numeric step bounds remain unresolved.

## Evidence identity and interpretation

**H** below means line numbers in:

`/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s/Z70Android/targeted-plugin/index.android.hasm`

Artifact identity recorded by the preceding primary research: 1,518,814 lines; SHA-256 `5fa398d080e801a31b7d55598ce8c696535808fbbbba6b5bac13c413d7ba4a2c`. Sibling `index.android.bundle` is Hermes bytecode v96, recorded SHA-256 `bfd30c61c7969195251864d1296f464b732b1aefecd18d0ba86c51e1c511cc67`. These identities are carried forward from `docs/arm-joint-control-research.md`; the instruction windows cited here were read directly in this follow-up. Function parameter counts include `this`.

JSON below reconstructs serializers and bundled fixtures. It is not a robot capture, a tested firmware schema, or an execution prescription. Unknown required fields, firmware rejection behavior and bounds remain explicitly unknown. Negative findings are limited to the inspected paths/artifacts, not claims about all firmware or native code.

## 1. Exact RPC parameter envelopes

| Android export / Hermes function | Method and parameters | Direct evidence |
| --- | --- | --- |
| `appSaveProgram(payload)` / #5527, `0x008a96bd` | `app_save_program`, **payload unchanged** | H:231015-231037; export H:224580 |
| `appStartProgram(payload)` / #5528, `0x008a970e` | `app_start_program`, **payload unchanged** | H:231042-231064; export H:224582 |
| `appGetProgram(id)` / #5514, `0x008a9421` | `app_get_program`, `{"program_id":id}` | H:230734-230758; export H:224554 |
| `appGetProgramRuntimeProgress()` / #5515, `0x008a9479` | `app_get_program_runtime`, `{}` | H:230763-230774; export H:224556 |
| `appStopGrasp()` / #5529, `0x008a975f` | `app_stop_grasp`, **`[]`** | H:231069-231080; export H:224584 |

Each wrapper also passes a separate `{}` transport-options argument to generic `f` (#5267). That object is not a nested program field. `JSON.stringify` in the save/start wrappers is used for logging; the argument passed to `f` is the original object, **not a JSON string**. No wrapper number conversion, clamp, defaulting, step validation, field whitelist, command expansion, or completion loop exists.

`f` takes method/parameters/options and dispatches through native `callMethod` or the spec path (H:225753-225797); another branch supports SDK retry machinery (H:225798-225832). This does not supply idempotency or execution correlation for a program. A resolved transport promise must not be interpreted as sequence completion.

### Save construction versus start construction

The Blockly bridge parses WebView message JSON and calls `dealWithDataFromBlockly` (H:1425383-1425385). Its bound #32683 (`0x00c6564a`, H:1425426-1425491) distinguishes:

1. No truthy `blocks`: forward the entire supplied object to `_remoteAppStartProgram`.
2. Truthy `blocks`, non-preset ID: forward the supplied object to `_remoteAppSaveProgram`.
3. Truthy `blocks`, preset ID 1/2/3: construct a copy containing exactly `name: original.name + "copy"`, `description`, `cmd_ids`, `blocks`, `program_type`, then save it. **No `program_id` is added to that copy.** H:1425460-1425489 proves the five keys.

Thus the proven save-copy shape is:

```js
{
  name: original.name + "copy",
  description: original.description,
  cmd_ids: original.cmd_ids,
  blocks: original.blocks,
  program_type: original.program_type
}
```

`blocks` is Blockly editor data, not the executable command array. The bundled preset constructs nested `blocks.blocks.languageVersion` and `blocks.blocks.blocks` with a `custom_procedure` / `inputs.BODY.block` / `next.block` chain (H:1430115-1430211). Numeric executable values coexist with **string** Blockly fields such as `{"dir":"1","step":"1"}` (H:1430128-1430135). The missing HTML generator prevents proving the complete string-to-number transformation or UI bounds. The pass-through save serializer alone does not establish which fields firmware requires, create/update ID semantics, or a returned new-ID schema. Save consumers #32673 and #32841 await then log the returned object, without an inspected new-ID parser (H:1424921-1425008; H:1433550-1433627).

For a stored program, the manage-page start path reads `appGetProgram(item.program_id)`, accesses **`response.result.cmd_ids`**, constructs **only** `{"cmd_ids": response.result.cmd_ids}`, then calls `_remoteAppStartProgram` (H:1431940-1432018). This proves that start need not send a saved `program_id`; the Android path resolves saved content first. Other editor/manager start paths forward their entire incoming object unchanged (#32668 H:1424625-1424776; #32878 H:1434695-1434790). Neither path fans out individual arm RPCs: it submits one program payload and opens the preview after the promise resolves.

## 2. Command records, order, repetitions and schedule

Primary bundled preset #32762 (`0x00c69606`) provides exact records:

| Command | Executable record | Evidence |
| --- | --- | --- |
| Deploy | `{"cmd_id":"arm_out","id":5}` | H:1430234-1430235 |
| Joint selection | `{"value":[0,0,1,0],"id":6,"cmd_id":"arm_joints","step":1}` | H:1430236-1430239 |
| End-effector selection | `{"id":7,"direction":3,"cmd_id":"arm_move","step":1}` | H:1430240-1430241 |
| Explicit delay | `{"cmd_id":"delay","time":3,"id":8}` | H:1430242-1430243 |
| Stow | `{"cmd_id":"arm_in","id":12}` | H:1430251-1430252 |

The array is assembled by index and placed under `cmd_ids` (H:1430224-1430253). `id` identifies an entry for the local validation map; it is distinct from the string operation `cmd_id` (H:1426582-1426626). The local checker iterates input order, maintaining predicted pose between entries, and pushes one result per command (#32758 H:1429715-1429863). No sorting by numeric `id` is shown. **Submitted array order and ordered model evaluation are proven; firmware dispatch timing, overlap and barriers are not.**

The first bundled preset contains repeated up/down commands as distinct array entries, with explicit delays and sound actions between them (H:1430083-1430114). It does not compress those records into a trajectory. The reconstructed start envelope for the two arm fixture records alone is:

```json
{"cmd_ids":[
  {"value":[0,0,1,0],"id":6,"cmd_id":"arm_joints","step":1},
  {"id":7,"direction":3,"cmd_id":"arm_move","step":1}
]}
```

This illustrates serialization only; removing surrounding deploy/delay/stow records is not demonstrated as admissible execution.

### Direction counts versus physical quantities

- `arm_joints.value[0..3]` maps **M3, M4, M5, M6**, and `step` maps directly to local `n_rep` (#32758 H:1429772-1429791). Enums: 0 no movement, 1 up, 2 down (H:1428502). The UI display treats M6 1 as release and 2 as grip; preceding primary display trace is H:1427092-1427193.
- `arm_move.direction`: **1 up, 2 down, 3 front, 4 back**; `step` is forwarded as the repeat count (H:1429793-1429823; enums H:1428500).
- Model #32745 loops `i < n_rep`; each accepted iteration advances the counter. Joint branch priority is **M3, then M4, then M5, then M6** (H:1429231-1429254). Only the first nonzero selection is modeled for that iteration; this is not proof that firmware implements that priority or simultaneous joint motion.
- Model M3/M4/M5 increment **+10 degrees for selection 1, −10 degrees otherwise in the selected branch**, with 0 excluded by branch selection (H:1429255-1429283). M6 sets predicted angle 90 for 1, otherwise 0. Normal enums define 2, but the checker is not a strict input validator for arbitrary nonzero codes.
- Model end-effector steps change predicted `.y` by ±**0.04** for up/down or `.x` by ±0.04 for front/back, then solve inverse kinematics (H:1429215; H:1429294-1429346). Degree/radian conversion is explicit (H:1429419-1429446). These are model quantities, not measured displacement or verified physical calibration.

### Step limits: what is actually established

There is **no fixed numeric min/max/integer contract for `step` in the inspected serializers or checker**. The checker performs a JavaScript-style loop comparison without an explicit integer/range check. Do not infer that zero, negative, fractional, missing or enormous counts are firmware-valid. Bundled examples establish `step:1`, not the full allowed range. The HTML numeric controls are missing.

`GetMaxStep` (#32746) returns local environment slot 19 (H:1429412-1429414); that slot starts at 0 for the command, increments **only after predicted-pose validation succeeds**, and returns early at the first failure (H:1429211-1429214; H:1429348-1429369). #32758 uses that count as `maxStep` **only on failure**; otherwise its initialized `maxStep` remains −1 (H:1429739,1429847-1429861). It returns `{id,cmd_id,currentResult,maxStep}`.

Model joint limits: M3 **90..270°**, M4 **35..170°**, M5 **0..155°** (H:1428543-1428548; checked H:1429075-1429100). Additional model checks include height `.y > .37`, robot/ground collision regions, projection `.x` outside ±.37, and `m4_angle - m3_angle + 90 > 15` (H:1429101-1429149). These depend on starting predicted pose, earlier actions and direction; there is no universal maximum repeat count derivable from them. Error enum H:1428506: 0 success; 1 joint limit; 2 height; 3 robot collision; 4 ground collision; 5 projection; 6 inverse kinematics; 7 A4 angle; 8 grasp-not-open. These codes are **local model faults**, separate from runtime command results.

Schedule evidence is limited to **ordered records, per-action counts, and explicit `delay.time` records**. No start timestamp, trajectory sample timestamps, fixed per-step duration, arm speed/acceleration, interpolation or deadline appears in these arm fixture records. The inspected delay fixture does not itself prove the time unit or a motor-completion barrier. The 500 ms timeout in the start flow hides a checker loading view (#32669, H:1424687-1424690 and H:1424863-1424875); it is not a step period. The 2,000 ms interval below is telemetry polling, not execution cadence.

## 3. Runtime telemetry and result meanings

`app_get_program` retrieves stored content by ID. It is **not** the execution-progress query. The dedicated runtime query returns an object consumed as:

```js
response.result.program_state
response.result.result // array of command records with per-record result
```

Evidence: #32698 requests the runtime wrapper and returns the transport response (H:1426205-1426246); #32715 reads both nested fields and maps the collection (H:1427422-1427495). This proves the Android-consumed layout, not every optional firmware field or a captured sample.

| Field | Android label | Evidence |
| --- | --- | --- |
| `program_state:0` | without tasks | H:1425863-1425868 |
| `program_state:1` | starting | H:1425869-1425873 |
| `program_state:2` | executing | H:1425874-1425878 |
| `program_state:3` | error | H:1425879-1425883 |
| `program_state:4` | edit again (**local preview/check state**) | H:1425884-1425888; H:1426587-1426589 |
| command `result:0` | successful execution | H:1425838-1425841 |
| command `result:1` | waiting for execution | H:1425842-1425846 |
| command `result:2` | executing | H:1425847-1425851 |
| command `result:-1` | execution failure | H:1425852-1425856 |
| command `result:-2` | execution failure 2 | H:1425857-1425861; signed comparison H:1427943-1427951 |

**Signed/unsigned caveat:** the disassembler prints the separate `PROGRAM_FAILED_1/2` constants as `4294967295/4294967294` at H:1425833, while the text map uses string keys `-1`/`-2` and other code explicitly compares signed −1/−2. Preserve raw values and unknown codes; do not silently normalize unsigned constants into a claimed firmware contract. Also, mapping `program_state` uses `value || -1` (H:1427453-1427456): state **0 is lost to −1 in that local state setter**. A new parser should not copy that falsy-default behavior. State 4's observed local synthesis is not proof firmware emits it.

Runtime record mapper #32716 (H:1427527-1427571) performs `Object.assign({}, record, displayFields)`: it preserves input fields and adds `statusText`, `cmdId`, `cmdText`, `time/speed/angle/distance` defaulted with `|| 0`, and `result`. These numeric fields are command/display data, **not established measurements of elapsed time, actual velocity, angular position or achieved distance**. No measured joint vector, measurement timestamp, per-step executed counter, motor-stationary flag, pending count or run-generation ID is read in this parser.

Preview lifecycle #32725 queries immediately when visible and not in local-check mode, then sets an interval of **2,000 ms** (#32727 calls the query again); hidden/cleanup clears the interval (H:1427840-1427938). No stop RPC is sent by the cleanup. Polling failures are caught/logged (#32698 H:1426247-1426271), not demonstrated as an empty queue.

### Why preview success and “RealPos” do not prove physical completion

The preview takes the **last returned collection entry's `result`** (H:1426413-1426427), sets a failure hint if any result is −2 (H:1426430-1426436), and offers Finish for last result 0 (H:1426478-1426484). It does not establish full expected-command coverage, run identity, freshness, or all-entry success.

`RecordRealPos` on last-result 0 copies predicted slot 15 to recorded slot 16 (H:1426813-1426822; H:1429629-1429632). `GetCurrentPos` and `GetLastEvaluatedPos` clone local slots 15/17 (H:1429649-1429672). Their names do not turn the model into encoder telemetry. The local-check preview even synthesizes result 0/−1 from `currentResult` before execution (H:1426586-1426603).

**Evidence available:** firmware-reportable task state and per-command execution labels, stronger than a bare submission ACK. **Not established:** a measurable sequence-completion predicate, physical target convergence, or a trustworthy proof that no robot-side execution remains pending.

## 4. Stop source and cancellation coverage

### Program preview stop

Preview `ct` → generator #32707 calls API object's **`stop()`**, awaits it, and logs `appStop response` (H:1426680-1426749). The API registration binds `stop` to **#5289** (H:225166-225167). #5289 (`0x008a6fb0`) calls generic `f(Methods.AppStop, [], {})` (H:226366-226383). The exact raw method string is supplied by the imported native `Methods` object: this inspected instruction path proves **the symbol `Methods.AppStop` and `[]`**, not its native string expansion. No completion/queue-drain loop is in that stop function.

### Manual manager stop

Manager `_onPressStopButton` generator **#19925**, `0x00aa1d07`, checks `isSupportApiAppStopGrasp()`; if true it awaits `appStopGrasp()`, otherwise it awaits generic `stop()` (H:859124-859155). The support accessor reads `supportApiAppStopGrasp` (H:240804). #5529 proves `app_stop_grasp []` serialization. The wrapper and manager do not check post-stop joint motion or program runtime. **This named consumer supplies manual-control stop intent; it does not prove that `app_stop_grasp` is the program cancel operation or a general all-arm-motors emergency stop.**

### Local service controller

At the inspected working-tree source `src/arm-controller.ts`:

- Lines 68-79 check revision/authorization before and after a supervised send and accept `"ok"`/`["ok"]` as ACK. A post-send cancellation exception cannot undo that submitted action.
- Lines 201-209 clear local busy/ownership in `finally` after the request flow. That is request lifecycle, not a robot queue-empty result.
- Lines 211-215 increment revision and revoke authorization on `cancel`/`close`; **no RPC is sent**.
- Lines 41-53 expose local busy/chassis blocking and `softwareStopVerified:false`; lines 61-65 observe status and joints, not runtime programs.

Do not label local `busy:false`, lease expiry, cancelled revision, disconnected transport, cleared repeat timer, closed preview, or a stop ACK as proof that a submitted program cannot still execute. No inspected source provides stopping latency/distance, affected motor list, flush semantics, or an arm-stationary handshake.

## 5. Nearby archive search: missing HTML remains a concrete gap

The target directory has only the Hermes bundle/disassembly and two parser `.jx` files. Parent directory contains the one nearby `Roborock-4.74.08-100936-arm64-v8a.apk`.

The precise desired asset is registered by #32760 at H:1429912-1429929:

`/assets/projects/com.roborock.tanos/Setting/Program/html/index.html`

Asset hash: **`a346fef5730774bf6e71df54903bc7e4`**.

Read-only `unzip -l` inspected the APK (2,312 entries) and selected `*tanos*`, `*Program*`, `*program*`, `*model*`, `*.html`, `*.zip` names. No matching program HTML or Tanos plugin package appears. The HTML names are captcha/recaptcha and Paho package documentation; `models_g3db`/`qrml_models` are not this program editor asset.

The APK contains seven direct `assets/plugin/**.zip` entries. ZIP central-directory inspection in memory, without extracting payload files, covered:

| Nested package | Entry count | Relevant name matches |
| --- | ---: | --- |
| `features/feature_device_pairing/feature_device_pairing.zip` | 192 | none |
| `features/feature_inner_feedback/feature_inner_feedback.zip` | 29 | none |
| `features/feature_intelligent/feature_intelligent.zip` | 64 | none |
| `home_card/roborock.mower/roborock.mower.zip` | 120 | only `android.zip.sign` for `.zip` name search |
| `home_card/roborock.wetdryvac/roborock.wetdryvac.zip` | 23 | none |
| `home_card/roborock.wm/roborock.wm.zip` | 114 | only a cloud-program PNG |
| `home_card/robot.vacuum.cleaner/robot.vacuum.cleaner.zip` | 99 | none |

Search keys were case-insensitive `tanos`, `program`, `blockly`, `.html`, `.zip`. The vacuum package's `index.android.bundle` is **1,156,616 bytes**; its directory is not a recovered Tanos editor asset. No entire APK decompilation, secret extraction, or execution occurred. The absence is bounded to this APK and these listed nested archives, not every possible downloaded plugin/cache.

## Recommendation and exact remaining evidence requirements

1. **Treat this as a candidate relative program interface, not calibrated waypoints.** Preserve Android command names/order/fields; do not convert model +10°/0.04 into claimed physical motions or assign a universal step limit. A single-axis selection per joint record is a conservative research restriction, not a proven firmware rule.
2. **Keep reported execution and measured completion separate.** A future runtime observer can expose the raw task state/results. A completion claim would require fresh, run-correlated coverage of every expected entry with terminal results plus a verified terminal-task contract. This bundle provides no run ID, freshness guarantee, full-queue contract or validated all-entry terminal predicate. Last-entry 0 alone is insufficient.
3. **“No pending execution” needs its own evidence.** State 0 is labelled without tasks, but static labeling alone does not prove active motors stopped, buffered commands were flushed, or delayed execution cannot resume. A suitable future contract would need validated start/stop correlation, task/queue drain semantics and fresh independent cessation evidence. Unknown, missing, failed or stale observations must retain uncertainty; do not infer empty execution from them.
4. **Do not route program cancellation through `app_stop_grasp` solely from its name.** Primary program UI uses `Methods.AppStop`; verify its native mapping, firmware scope and post-stop terminal behavior before advertising cancellation. Local cancellation currently blocks later local writes only.
5. The most targeted next offline artifact is the **matching Tanos plugin package/editor HTML and companion JavaScript** with hash above, then the native `Methods.AppStop` definition or firmware API schema. These can resolve Blockly limits/generator transforms and stop method mapping. Firmware documentation or trustworthy correlated runtime/physical evidence is still needed for actual repetition size, per-command timing, runtime terminal/drain semantics and physical completion.

**Bottom line:** exact wrapper envelopes and primary ordered command fixtures are established. Runtime task/result telemetry exists. Numeric UI/firmware step limits, physical step calibration, guaranteed serial completion barriers and general stop/queue-drain verification remain unproven. Nothing in this offline trace establishes that the current robot has completed a sequence or has no pending execution.
