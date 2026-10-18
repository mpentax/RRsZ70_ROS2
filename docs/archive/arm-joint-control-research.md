> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md), [arm model](../z70-arm-model.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

The exact three-input directional vectors and program predictive RealPos derivation remain useful protocol provenance. Local controller guards and the proposed camera trial are historical policy, not current firmware requirements or operating instructions.

External HASM/editor inputs below are private-owner provenance, unavailable from the public checkout; full original replay needs the authentic matching inputs. No private files were opened or copied during this curation.

## Historical record

# Z70 alternate arm modes and joint-control API research

Research date: 2026-10-03. **Primary Android follow-up performed entirely offline.** No robot tools, RPC, HTTP, camera request, mode change, or motion was performed. Only this document was edited by this workstream; no commit. The earlier adapter research below used public GitHub source retrieved using webfetch; no network retrieval was used for this follow-up.

## Finding

**The primary Android artifact proves that the traced manual UI uses directional jogging, not absolute joint targets.** It calls the three-input `appArmMove` wrapper with M3/M4/M5 selections of 0/1/2. A separate five-input `remoteManualArmPickUp` wrapper serializes five unchanged arguments, but no named caller was found in this bundle. Its five physical identities, limits and absolute/relative semantics remain unestablished. The inspected sources do not establish an absolute joint-position/waypoint API with calibrated feedback.

The proposal to use another mode for movement and temporarily enter remote pickup for the gripper camera remains a hypothesis. Separate mode and camera selectors exist; pose preservation, admissible deployed-arm transitions, and restoration of joint-control readiness are not established here. Camera/session internals are left to the separate camera research workstream.

**Correction to earlier notes:** `README.md:400-402` says the Android manual joint UI uses a five-input method. The now-available artifact supports the existence of that serializer, but the traced UI calls the **three-input** wrapper. The README was not edited. `getArmJointsData` has no named consumer in the artifact, and no joint-specific response parsing or slider population was found. Program `GetCurrentPos`/`RecordRealPos` are local model state, not demonstrated encoder telemetry.

## Source identities and references

### Primary Android artifact (H)

All **H line numbers** below refer to the exact file:

`/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s/Z70Android/targeted-plugin/index.android.hasm`

- 1,518,814 lines; SHA-256 `5fa398d080e801a31b7d55598ce8c696535808fbbbba6b5bac13c413d7ba4a2c`.
- Sibling `index.android.bundle`: `file` identifies **Hermes JavaScript bytecode version 96**; SHA-256 `bfd30c61c7969195251864d1296f464b732b1aefecd18d0ba86c51e1c511cc67`.
- Hermes function parameter counts include `this`: #5570's **6 params means five explicit inputs**, while #5505's 4 params means three explicit inputs. `LoadParam` indices 1 onward are the explicit arguments. Minified function names are retained below; semantic names come from export/method registration instructions.
- Dedicated grep/read searches were restricted to instruction lines (`^==>`) where necessary to avoid enormous embedded strings. Read windows covered wrappers, registrations, consumers, controls, program model and transport. `rg` counting was attempted but the executable is unavailable; complete narrow grep outputs supplied the occurrence inventory.
- These are **static bytecode traces**, not captured runtime command logs. JSON examples below are reconstructions of the proven serializers or built-in program fixtures, expressly not hardware acceptance evidence.

### Prior adapter references

Upstream repository: `copystring/ioBroker.roborock`, maintained default branch `main`, resolved during research to commit **`2b4f802d4267d55f8492c9e1d3ccc1d98179d086`**. All upstream line references below use that commit, not an unpinned branch.

- **A179**: [`src/lib/features/vacuum/a179_features.ts`](https://github.com/copystring/ioBroker.roborock/blob/2b4f802d4267d55f8492c9e1d3ccc1d98179d086/src/lib/features/vacuum/a179_features.ts).
- **A179 tests**: [`src/lib/features/vacuum/a179_features.test.ts`](https://github.com/copystring/ioBroker.roborock/blob/2b4f802d4267d55f8492c9e1d3ccc1d98179d086/src/lib/features/vacuum/a179_features.test.ts). These are mocks, not hardware captures.
- **Base**: [`src/lib/features/baseDeviceFeatures.ts`](https://github.com/copystring/ioBroker.roborock/blob/2b4f802d4267d55f8492c9e1d3ccc1d98179d086/src/lib/features/baseDeviceFeatures.ts), `getCommandParams` and `onCommandResult` are generic pass-through/no-op hooks.
- **Upstream docs**: [`docs/Commands.md`](https://github.com/copystring/ioBroker.roborock/blob/2b4f802d4267d55f8492c9e1d3ccc1d98179d086/docs/Commands.md) documents core V1/B01 commands, not the Z70 five-input schema or joint units.
- **Local**: `src/control-policy.ts`, `src/arm-controller.ts`, `src/arm-feedback.ts`, `src/control.test.ts`, `README.md` at the working-tree versions inspected. These files already contain other work; none was edited by this research.
- Earlier pinned upstream source cited locally: `1a3b5b79c2eaa33c1a257a7f1a17f749b1df59cb` (`src/arm-feedback.ts:1-2`). This report's new upstream inspection uses the newer commit above.

## Exact payloads proven in the inspected adapter

These describe static adapter serialization, not firmware acceptance in every mode.

| Method | Exact parameter shape | Source |
| --- | --- | --- |
| `app_pick_up_enter_exit` | `{"flag": n}`; labels 0 exit/off, 1 manual, 2 auto | A179:78-82, 399-405, 1334-1340 |
| `app_arm_move` | `{"m": [v0,v1,v2]}` or `{"m": [v0,v1,v2,v3,v4]}` | A179 `getCommandParams`:1379-1386; `normalizeArmMoveParam`:2915-2936 |
| `app_arm_move_preset` | Adapter-only alias; sends **`app_arm_move`** with `{"m": [...]}` | A179:1388-1394 |
| `app_arm_direction_move` | `{"direction": n}`; 1 up, 2 down, 3 go/forward, 4 back | A179:61-66, 1352-1359; local `control-policy.ts:56-58` |
| `app_get_arm_joints_data` | `{}` | A179:343, 1313-1318; A179 tests:195-198 |
| `app_arm_out_compartment`, `app_arm_in_compartment`, `app_open_gripper`, `app_grip` | `{}` | A179:337-340, 1313-1318; local `control-policy.ts:52-53,70-71` |
| `app_arm_recover` | `{}` by default, or an optional object passed through | A179:1306-1310, `normalizeOptionalObjectParam`:3060-3079 |
| `app_stop_grasp` | **`[]`**, not `{}` | A179:365-366, 1320-1325; A179 tests:199-202 |
| `app_exhibition_enter_exit` | `{"flag": n}`; labels 0 exit, 1 enter | A179:128-131, 404, 1334-1340 |
| `app_exhibition_action` | `{"type": n,"value": n}` | A179:1435-1439, `normalizeExhibitionActionParam`:3122-3135 |
| `app_program_enter_exit` | `{"flag": n}`; labels 0 exit, 1 enter | A179:404-405, 1334-1340; A179 tests:474-477 |
| `app_start_program` | `{"cmd_ids": [...]}`; an object with nonempty `cmd_ids` passes through | A179:1613-1617, `normalizeStartProgramParam`:3019-3042 |
| `app_get_program` | `{"program_id": n}` | A179 `getCommandParams` program branch; `resolveProgramStartPayload`:4170-4187 |
| `app_save_program` | Required object, passed through; no joint-specific schema validation here | A179:1606-1611 |
| `app_get_program_runtime` | `{}` | A179:344, 1313-1318; A179 tests:462-465 |

`app_arm_recover_payload` is another adapter-only alias to `app_arm_recover`, not a separate robot RPC. Recovery is not demonstrated as an ordinary move or stop.

## `app_arm_move`: values, transformation, limits, absolute versus relative

### Proven adapter behavior

`normalizeArmMoveParam`:

1. Parses JSON input with `parseJsonParam`.
2. Uses `payload.m` if the payload is an object containing `m`; otherwise uses the payload itself.
3. Requires an array of length **3 or 5**.
4. Converts each entry with **`Number(entry)`** and rejects nonfinite results.
5. Returns the converted array without scaling, rounding, offsets, sign conversion, saturation, or joint-specific range checking.

This is ioBroker input normalization. It is **not** the Android slider transformation. Acceptance of any finite number by this function does not establish a valid robot input or mechanical limit. A179's `app_arm_move` command registration at 487-491 contains no `min`, `max`, or `unit`.

### Proven preset definitions

A179:68-75 defines:

| `m` | Adapter label |
| --- | --- |
| `[2,0,0]` | M3 Left |
| `[1,0,0]` | M3 Right |
| `[0,1,0]` | M4 Left |
| `[0,2,0]` | M4 Right |
| `[0,0,2]` | M5 Left |
| `[0,0,1]` | M5 Right |

Thus, this source treats the **three-entry preset form** as per-axis directional commands. It does not label these entries as target angles. A179 tests:179-186 confirm `{m:[1,2,3]}` and the alias serialization; those test values are synthetic and are not a limits specification.

### Primary wrappers, all named callers, and transport

| Semantic name | Hermes function / body | Registration | Reconstructed call |
| --- | --- | --- | --- |
| `appArmMove(a,b,c)` | **#5505**, offset `0x008a920e`, H:230521-230539 | #5259 H:224535-224536 | `f("app_arm_move", {m:[a,b,c]})` |
| `remoteManualArmPickUp(a,b,c,d,e)` | **#5570**, offset `0x008a9c4f`, H:231765-231788 | #5259 H:224875-224876 | `f("app_arm_move", {m:[a,b,c,d,e]}, {})` |
| `getArmJointsData()` | **#5537**, offset `0x008a986e`, H:231211-231222 | #5259 H:224633-224634 | `f("app_get_arm_joints_data", {}, {})` |

Both move wrappers allocate an array of fixed length and put explicit arguments at indices in their original order. **Neither supplies input defaults, number conversion, rounding, offsets, clamping, sign conversion, or units.** For #5570, H:231773-231782 is the complete mapping `p1→m[0] ... p5→m[4]`. A missing argument is loaded as `undefined`, not converted to 0 by this wrapper; downstream serialization is outside that slot mapping. No call site demonstrating omitted five-input arguments was found.

The `f` callee is **#5267** (`0x008a693f`): #5259 binds it to environment slot 10 at H:224509-224510. H:225753-225797 dispatches through the generic `callMethod` promise path (#5260) or spec path (#5263). H:225798-225832 handles the SDK retry-capable path. Its `need_retry` injection applies only to a 13-method list at H:225238, which contains neither arm movement nor joint query. Thus the extra `{}` in #5570/#5537 is a transport argument, not another arm-command field. The wrappers return the transport result directly; they have no measured completion loop.

Complete narrow occurrence searches found:

- Exactly two instruction occurrences of literal `app_arm_move`: H:230537 and H:231785.
- `remoteManualArmPickUp` (without leading underscore): only its export H:224876. Function #5570 is created only for that export at H:224875. **No named consumer of the five-input wrapper** was found.
- `appArmMove`: export H:224536 and a single consumer H:858556, in **#19907** (`0x00aa174a`).
- `getArmJointsData`: only export H:224634; #5537 is created only there. Literal `app_get_arm_joints_data` occurs only in its wrapper H:231218. **No named query consumer/parser** was found.

These negative findings concern named/static references in this artifact. They do not exclude computed property access, native callers, another plugin revision, or firmware features absent from this bundle.

### Actual manual calling component and exact values

Call chain:

`MechanicalRemoteControlKeyView → _onPressDirButton → MAM._remoteManualArmPickUp(a,b,c) → appArmMove(a,b,c) → app_arm_move {m:[a,b,c]}`.

- Component identification: H:1283562-1283570 (`-MechanicalRemoteControlKeyView`, class factory **#29606**, constructor **#29607** at `0x00bf34b3`).
- Parent UI method `getMechanicalManualArmControlView` is bound to **#28147** at H:1214107-1214110; its body H:1223982-1224012 creates the child with `isVideoPage:true`, `isArmState:this.isArm` and `armStateChanged`. #28189 H:1225975-1225977 invokes that parent method. These component properties do not supply a joint vector or query feedback.
- Method registration: `_onPressDirButton` → generator #29617/#29618 at H:1283764-1283772. Body **#29618** (`0x00bf3a3d`) H:1284236-1284415.
- Manager registration: #19892 H:857985-857993 binds `_remoteManualArmPickUp` to #19906/#19907/#19908. **This underscored manager method has three explicit parameters; it is not the five-input RPC export with a similar name.**
- Manager body **#19907**, H:858530-858557, forwards the three parameters unchanged to `appArmMove`. The log text says `angle-` (H:858544), but the actual callers pass enum values, not angles.

| Input control identity | Exact `m` payload | #29618 branch / H evidence |
| --- | --- | --- |
| M3 left | `{"m":[2,0,0]}` | `0x1a9..0x1d3`, H:1284338-1284349 |
| M3 right | `{"m":[1,0,0]}` | `0x173..0x19d`, H:1284325-1284336 |
| M4 left | `{"m":[0,1,0]}` | `0x13d..0x167`, H:1284312-1284323 |
| M4 right | `{"m":[0,2,0]}` | `0x107..0x131`, H:1284299-1284310 |
| M5 left | `{"m":[0,0,2]}` | `0xd1..0xfb`, H:1284286-1284297 |
| M5 right | `{"m":[0,0,1]}` | `0x9b..0xc5`, H:1284273-1284284 |

Branch comparisons identifying those controls are H:1284261-1284272. Each command has one selected axis and zeroes for the others. The left/right polarity is axis-dependent, **not signed positive/negative input**. These values match the earlier adapter presets, now with primary Android corroboration. Physical clockwise/counterclockwise conventions, axis orientation in a robot/world frame and actual displacement per call remain unknown.

The same #29618 routes up/down/go/back to the distinct `app_arm_direction_move` path with direction 1/2/3/4 (H:1284351-1284397), not to extra slots in a five-angle vector.

### Input state, sliders, bounds and submission events

The traced manual controls are **buttons / paired directional controls**:

- `getTouchView` **#29631** H:1285980-1286025 wires `onPress` #29632, `onPressIn` #29633, and `onPressOut` #29634. Their bodies H:1286100-1286140 respectively call `_onPressDirButton`, `startIncrement`, and `stopIncrement`.
- **#29608** H:1283963-1284013 starts a per-control **100 ms interval**, guarded against an existing timer. #29609/#29610/#29611 H:1284018-1284076 increment a UI counter by 1 and call `_onPressDirButton` in the `setState` callback. This is button-repeat timing and a UI counter, **not a joint unit or calibrated speed**. No awaiting/backpressure in that interval callback is shown.
- **#29612** H:1284081-1284113 clears/deletes that timer and resets `buttonPressed`; it sends no all-zero arm move or motor-stop RPC. #29615 H:1284184-1284203 clears timers on unmount. No counter-to-RPC magnitude transform occurs in #29618.
- `renderJointOperationView` **#29637** H:1286267-1286364 constructs three paired controls with identities M5 right/left, M4 left/right, M3 left/right, passing press/start/stop handlers. **#29638** H:1286381-1286413 selects this view or `renderArmOperationView` using local `state.isArm`.

The constructor does initialize **five state fields**: `angle1=90`, `angle2=90`, `angle3=180`, `angle4=60`, `angle5=0` (H:1283921-1283929). A complete narrow instruction search found only those five writes plus reads of `angle3/4/5` at the beginning of **#29622**, H:1284486-1284488. No setters, slider handler, query-fed updates, or five-input submission using these fields was found. Their initialization does **not** identify #5570's five physical inputs or establish setpoint defaults.

**No slider min/max/step/default/transform can be assigned to the five-input RPC from this artifact.** Searches for `minimumValue`, `maximumValue`, `onSlidingComplete`, `onSliderComplete`, `Slider`, `FatSlider`, and `onAfterChange` found generic and unrelated slider implementations; the traced manual component renders directional controls and has no such binding to #5570. Its angle-named leftovers are insufficient to invent slider bounds. The program editor is a **WebView asset**, not a native joint-slider component: #32681 H:1425284-1425301 loads dependency 22; Metro module 3678's dependency 22 is **3682** (H:18503-18505), whose asset function **#32760** registers `/assets/projects/com.roborock.tanos/Setting/Program/html/index.html` with hash `a346fef5730774bf6e71df54903bc7e4` (H:1429912-1429929). That HTML is absent from the supplied `targeted-plugin` directory, so its numeric-field/slider bounds and transformations are not established here. #32682 H:1425333-1425385 parses WebView message JSON and forwards it to `dealWithDataFromBlockly`, not to `getArmJointsData`.

### Absolute versus directional verdict

**Resolved for the traced live UI: directional jogging.** M3/M4/M5 use finite selections 0/1/2, passed unchanged, with repeating button events. Neither command targets nor encoder positions are sourced from sliders. **Unresolved for unused five-input #5570:** only argument order and exact payload envelope are proven. In particular, do not relabel it as `[M1,M2,M3,M4,M5]`, `[M3,M4,M5,M6,step]`, or five absolute angles. No calibrated conversion between raw live-command entries and feedback is established.

## Modes and admissibility

### Mode entry flags versus feedback codes

The grab-mode command uses `app_pick_up_enter_exit {flag:0|1|2}`. Its upstream labels are off/manual/auto. Local Android-parity notes call flag 2 **remote-control-to-pick-up**, entered by `_setGrapMode(2)` (`README.md:383-402`). “Auto mode” and “remote pickup” are source-specific labels for that code, not evidence of another numeric mode.

Program and exhibition entry each have their **own method** and `flag` field. Do not equate their flag 1 with grab mode 1, or assume either has a particular grab-mode feedback code. No inspected source maps them to an absolute-waypoint mode or proves they improve joint telemetry.

### Local controller guards: proven software policy, not firmware specification

`src/arm-controller.ts` imposes these guards:

| Operation | Required pre-command feedback | Reference |
| --- | --- | --- |
| Manual deploy | Off/not-started, then sends chassis end + grab flag 1, waits 5 s, requires manual/state 39 before arm-out | 85-104 |
| Manual stow/open/close | Manual/waiting | 100-104 |
| Directional nudge / manual pickup | Manual/waiting | 136-154 |
| Remote-pickup entry | Off/not-started | 109-117 |
| Remote arm-out | State 39, mode 2, action not-started | 118-126 |
| Remote forward | State 39, mode 2, waiting or grasping | 127-135 |
| Remote stow/grip | State 39, mode 2, waiting/grasping/putting-down | 156-167 |
| Manual exit | Manual/not-started | 192-200 |

Remote workflow guards are separately encoded at 168-190: start from waiting; continue/pause grab from continuing/continuing-grasp/grasping; put-down from waiting/grasping; continue/pause put-down from continuing-put-down/putting-down. `control-policy.ts:64-69` maps start/continue/pause pickup to `app_pick_up` flags 2/3/0 and put-down to `app_put_down` flags 0/1/2. `src/control.test.ts:47` attributes the unchanged flag wrappers to Hermes v96 **#5568/#5569**; that comment is secondary provenance, not a new disassembly inspection.

The local common readiness policy also checks idle/manual robot state, battery, faults, lock and task flags (`arm-feedback.ts:58-64`). None of these local policies proves the full firmware admissibility matrix.

The inspected A179 `app_arm_move` serializer and normalizer do not check mode/action. That absence does not prove the firmware admits joint moves while off, changing mode, grasping, or in remote pickup. The local service does not expose raw joint moves (`control-policy.ts:72`; `arm-controller.ts:15-16,46`).

## Feedback: exact parsing and sample provenance

### Grab workflow feedback

A179 `updateDerivedCommonStatus`:3759-3800 decodes `get_status.common_status`:

- `mechanicalArmGrabStatus`: `(common_status >>> 6) & 0xf`.
- `mechanicalArmGrabMode`: `(common_status >>> 10) & 0x3`.
- `mechanicalArmGrabResult`: `(common_status >>> 12) & 1`.
- Mode booleans: 0 not-in-grab, 1 manual, 2 auto.
- Action labels: 0 not-started; 1 exiting compartment; 2 entering compartment; 3 waiting; 4 grabbing; 5 putting down; 7 continue; 8 mode changing; 9 continue grab; 10 continue put-down. Other codes have no mapping here.
- Result bit 1 is labelled `isGrabSucessful` by upstream. This is a workflow result hint, not a measured joint or physical grasp proof.

Local `arm-feedback.ts:49-55,67-74` implements equivalent mode/action extraction with division/modulo and expressly labels physical completion and joint units unverified. These bits describe workflow phases, not angular position or per-joint velocity.

### Joint query

**Primary Android parsing result:** #5537 H:231211-231222 constructs `{}` and returns the generic transport promise. There is no dedicated `data[0..n]` consumer reachable by a named reference to `getArmJointsData`, no query-fed manual state update, and no encoder units/measurement timestamp/velocity/validity declaration in the traced path. The non-spec generic callback **#5262** tests native callback success and `unknown_method` (H:225327-225335), then resolves with the returned response object unchanged (H:225449-225451). Its error branch constructs an error object with a `data` field (H:225390-225398); that is not the joint-response schema. **No Android sample response is encoded by the query wrapper.**

Consequently this source does not establish whether the real query's numbers are encoders, command selections, defaults, or unsupported telemetry. The absence of a consumer means it also does not show the app treating them as selections. The historical `{data:[0,0,0]}` below remains the only cited robot observation; it must not be conflated with UI enum vectors.

The exact query is `app_get_arm_joints_data {}`. The A179 source registers/serializes it, but has **no dedicated joint-response case** in `onCommandResult`, no joint-index calibration, and no joint units declaration.

Local parser `armFeedback` (`arm-feedback.ts:35-40,65-74`) accepts an object whose **`data` is a finite-number array of length 1..16**, copies the array, and does not transform its values. This is a local validation contract, not a vendor guarantee of that array length or joint identities. `observedAt` is the local receipt/processing timestamp, not a device measurement timestamp.

Existing historical observation recorded in `README.md:347-363`:

```json
{"data":[0,0,0]}
```

The README reports this response while physically deployed, including before/after an observed upward arm step. This research did not repeat those observations. The sample cannot be interpreted as joint angles, stationary state, physical stow, or evidence that another mode will produce usable positions. Whether the query returns command codes, unsupported/default values, or mode-dependent measurements is unresolved.

Local `arm-feedback.test.ts:10-20` has synthetic `[0,0,0]` and `[1,2,3]` inputs; they prove parser behavior only.

## Programs: sequencing lead, not a demonstrated joint trajectory interface

### Primary program identities and model step units

The Android program names differ importantly from the earlier adapter's display interpretation:

- **Android `arm_joints`**: `value:[M3,M4,M5,M6]` directional selections plus `step` repetition count. **Android `arm_move`**: `direction:1..4` end-effector movement plus `step`. The adapter section below previously describes the opposite labels. The Android distinction is independently proven in **#32758 `checkArmInvalidAction`** (`0x00c691d5`), H:1429727-1429823: `arm_joints` reads `value[0..3]` into `m3..m6`, and puts `step` into `n_rep`; `arm_move` converts direction to `HAND_MOVE_*` and passes `step` to #32736. This is a material naming discrepancy, not interchangeable payload evidence.
- Primary enums in **#32735**, H:1428498-1428503: `JOINT_NOT_MOVE=0`, `JOINT_MOVE_UP=1`, `JOINT_MOVE_DOWN=2`; `HAND_NOT_MOVE=0`, up/down/front/back=1/2/3/4.
- **#32709/#32710** H:1426942-1426988 and H:1427092-1427193 display `arm_joints` value entries as `M3/M4/M5/M6`; 0 is omitted; M3..M5 code 1=up, 2=down; M6 code 1=release gripper, 2=grip. `step` is supplied to the up/down text template unchanged. This is display and program selection evidence, not raw five-slot RPC labeling.
- **#32745 `q`** (`0x00c68b8c`), H:1429213-1429283 loops `n_rep` times; each model iteration adds **+10° for code 1 or −10° for code 2** to the selected M3/M4/M5 angle. The branch priority is M3, then M4, then M5, then M6 (it does not model all selected axes simultaneously). M6 maps release to model angle 90, grip to 0, without a proportional joint increment (H:1429247-1429253).
- For `ARM_MOVE`, the same model adds/subtracts **0.04** to `end_pos.y` for up/down and `.x` for front/back per iteration (H:1429215,1429294-1429328), then runs inverse kinematics. These are model length units, consistent with its link lengths `.1775` and `.205` in #32739 (H:1428740,1428763); no runtime measurement validates a physical 4 cm step.
- Degree interpretation is explicit inside this model: **#32747** computes `angle*PI/180` (H:1429419-1429430), and **#32748** computes `radians*180/PI` (H:1429435-1429446). **The model's 10° increment does not establish the live three-input RPC displacement per press.**
- Model bounds, not UI slider limits or tested mechanical limits: M3 **90..270°**, M4 **35..170°**, M5 **0..155°** (#32735 H:1428543-1428548; checked in #32744 H:1429075-1429100). The model also checks height/collision/projection and `m4_angle-m3_angle+90 ≤15` (#32735 H:1428496-1428497; #32744 H:1429135-1429149). Do not turn these model bounds into five-input acceptance limits.

Static built-in fixtures corroborate the exact format: **#32762** (`0x00c69606`) contains `{"id":6,"cmd_id":"arm_joints","step":1,"value":[0,0,1,0]}` construction at H:1430236-1430238 and `{"id":7,"cmd_id":"arm_move","direction":3,"step":1}` at H:1430240. These are vendor bundled examples, not command captures.

### “RealPos” is local estimation, not encoder feedback

The ROI/program model defines local slots 15=current, 16=recorded, 17=last evaluated. It starts with zero angles and an `IN_ROBOT` state (#32735 H:1428518-1428539), synthesizes working angles **150/45/0/90** when arm-out is modelled (#32742 H:1428907-1428933), and evolves angles from command selections (#32745 above).

- **#32754 `RecordRealPos`**, H:1429629-1429632, copies **slot 15 to slot 16** using the local JSON clone helper #32751 (H:1429498-1429514). It performs no RPC or encoder read.
- **#32753 `RoiCheckInit2RealPos`**, H:1429588-1429598, copies **slot 16 back into slots 15 and 17**; it also performs no RPC.
- **#32756 `GetCurrentPos`**, H:1429649-1429658, returns a copy of slot 15; **#32757 `GetLastEvaluatedPos`**, H:1429663-1429672, returns a copy of slot 17.
- The preview status handler **#32708 `gt`**, H:1426813-1426822, calls `RecordRealPos` when its last command result equals **0**. This makes program acknowledgment/result success advance a predicted position, not establish measured joint convergence.

Thus fields labelled `m3_angle` etc. and methods labelled “RealPos” in the program checker are **derived model state**. Their existence cannot fill the missing encoder-feedback contract.

### Prior adapter evidence (retain with naming discrepancy above)

A179 `normalizeStartProgramParam`:3019-3042 accepts nonempty `cmd_ids`, or resolves a numeric `program_id`. `resolveProgramStartPayload`:4170-4187 reads `app_get_program {program_id}` and sends its `cmd_ids` as `{cmd_ids:[...]}`. That resolution is source code only; it was not executed here.

Program command labels include `arm_out`, `arm_grasp`, `arm_putdown`, `arm_in`, `arm_joints`, and `arm_move` (A179:199-213). Crucially:

- `arm_joints` is interpreted by `getProgramRuntimeCommandText` as **`direction` codes 1..4** (up/down/head/back), not an absolute joint vector (4211-4219; 192-197).
- Program `arm_move` display uses **`value`** array and **`step`**, separate names from the live RPC's **`m`** (4239-4277).
- Program array labels are M3/M4/M5/M6. M3..M5 values 1/2 display as up/down by `step`; 0 entries are omitted. M6 values 1/2 display as release/grip (4253-4277).
- These are adapter display semantics. They do not prove the complete program submission schema, physical units of `step`, interpolation, joint synchronization, speed/acceleration bounds, or waypoint timing.

`app_get_program_runtime {}` is decoded by `applyProgramRuntimeStatus`:2346-2412. It reads `program_state` and a `result` collection of commands, exposes per-command results, flags negative results, and takes the last collection entry as its latest command. Upstream maps program state 0/1/2/3 to no tasks/starting/executing/error, and command result -2/-1/0/1/2 to failure 2/failure/success/waiting/executing (A179:171-184). This is a stronger sequencing feedback lead than a bare move ACK, but still not measured trajectory progress.

Explicit **synthetic upstream test fixture**, A179 tests:2046-2062:

```json
{
  "result": {
    "program_state": 2,
    "result": [
      {"cmd_id":"move_forward","cmdText":"Forward","time":3,"speed":2,"result":2},
      {"cmd_id":"arm_move","value":[1,0,2,1],"step":4,"result":-2}
    ]
  }
}
```

The adapter renders the second command as `M3 Up 4 | M5 Down 4 | Release Gripper` (test:2106-2108). The numbers are not a measured robot sample or a calibrated motion prescription.

## Callbacks, trajectory and stop support

- A179 `onCommandResult` dispatches program getters to the runtime/detail parsers (1779 onward), but has no `app_arm_move` or joint-query-specific completion callback. Base `onCommandResult` is a no-op. A returned request promise does not establish arm motion completion.
- Local `ArmController.#send`:73-80 accepts `"ok"` or `["ok"]` and explicitly handles it as acknowledgment. `currentState` obtains another observation; it does not compare a target vector or wait for measured joint convergence.
- Local cancel increments a revision and revokes future command authorization (`arm-controller.ts:211-215`). It sends no arm motor-stop RPC.
- **A static stop candidate exists:** A179 exposes `app_stop_grasp []`. Its name and serializer do not establish a generic joint-jog/program/trajectory stop, coverage of every arm motor, stopping distance, or cessation feedback. Local pickup/put-down pause flags likewise concern those workflows only.
- No absolute target, time-indexed joint trajectory, joint velocity/acceleration contract, interpolation contract, general motor-stop handshake, or calibrated feedback-completion condition was found in the inspected files. This is bounded negative evidence, not proof that firmware has no such API.

## Remaining evidence gaps after the primary trace

1. A consumer or another revision/native implementation of five-input #5570 is needed to name its slots, establish valid values and determine absolute/relative semantics. The present bundle has only the serializer/export.
2. The program HTML asset `Setting/Program/html/index.html` and its companion scripts are needed for Blockly numeric UI bounds/transforms. No native arm slider-to-five-input chain exists in the traced component.
3. A firmware/native schema or trustworthy captured query response with independently measured pose is needed to establish `app_get_arm_joints_data` identities, units, validity and whether it is encoder feedback. Android's unused wrapper supplies none of those guarantees.
4. Mode/camera transition research belongs to the separate workstream; primary joint conclusions here do not imply pose-preserving transitions or admissibility in another mode.

**Current research verdict:** the active Android manual path is direction-coded M3/M4/M5 jogging. Programs use relative command selections and a local predictive kinematic checker. The unused five-input raw format and actual query feedback semantics remain unresolved; reliable absolute waypoints plus measured joint feedback remain unproven.
