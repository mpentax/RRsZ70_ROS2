> **Supporting historical evidence — 2026-10-18 curation.** The original audit/replay body below is retained verbatim. Static negative findings and implementation statements are bounded to the inspected sources; replay is not a second physical acquisition. Current behavior: [camera guide](../cameras/FINDINGS.md) and [arm model](../../docs/z70-arm-model.md).

# Android plugin audit: gripper-camera selection and admission

## Scope and evidence convention

Primary source independently inspected: `/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s/Z70Android/targeted-plugin/index.android.hasm` (1,518,814 lines; 34,803 function headers). `H:line–line` below means exact, one-based lines in that disassembly, not JavaScript source lines. Function names such as `value` are the bytecode names; readable method names are recovered from their descriptor bindings. Pseudocode reconstructs bytecode dataflow and control flow, omitting Babel generator machinery, analytics, and most logging.

This is an **offline static audit**, not a firmware experiment. No robot, Docker, services, network, credentials, `.env`, `.local`, or token files were contacted/read. The historical archive report was not used as primary evidence. The user's physical observation is **three cameras: two chassis cameras and a gripper camera**. Nothing below substitutes a two-camera physical model for that observation.

## Bottom line

1. The vendor's actual camera-button denial toast explicitly says: **“Switching to the gripper camera is only available in Remote Control to Pick Up mode.”** This is not an unused localization string: function **#28053** reads it in the failed `RSM.canChangeCameraStatus` branch, immediately before returning without a selector RPC. Chinese independently agrees. [H:315715, H:315728; #28053 H:1220699–1220716]
2. The camera selection path is `changeCamera(wantMechanical)` → `appChangeCamera(wantMechanical ? 1 : 0)` → `app_change_camera({flag: value})`. There is **one direct appChangeCamera call site and three direct changeCamera callers** in this disassembly. The app emits **camera flag 1 for mechanical/gripper, flag 0 for non-mechanical**, not an enumerated three-camera index. The low-level wrapper itself passes its argument through unchanged, so this proves app behavior, not the firmware's full accepted value domain. [#28055 H:1220747–1220763; #5508 H:230590–230603]
3. Admission and selected-state feedback are **different status words**: `common_status` bit 15 supplies `canChangeCameraStatus`; `camera_status` bit 15 supplies `mechanicalCameraEnabled`. `common_status` bit 14 supplies the independent arm-activated gate. Selection is not proved merely by arm deployment or by an RPC acknowledgment. [#8428 H:342139–342154; #8583 H:347778–347782]
4. Crucial new corroboration beyond the toast: the Monitor's **arm-view entry action explicitly calls `_setGrapMode(2)`**, which forwards to `app_pick_up_enter_exit({flag:2})`. This is a **pickup-mode command**, not `app_change_camera({flag:2})`. The camera-button handler does not directly test `grabMode == 2` or robot `state == 39`; it tests firmware-provided permission and activation fields. [#28208 H:1226621–1226638; #19898 H:858228–858233; #5564 H:231663–231676]
5. No dedicated arm-camera-enable RPC, alternative camera ID/channel/type selector, or camera flag 2 caller was found in the inspected plugin. That is a bounded static negative finding; native SDK internals and firmware may support more than this plugin exposes.

## 1. Complete app selector call graph

The RPC export binding is function **#5259**, H:224541–224542: closure #5508 becomes `appChangeCamera`.

Monitor method descriptors in **#27920** bind:

- `onPressMechanicalCameraButton` → #28053, H:1213794–1213799.
- `changeCamera` → async generator wrapper #28054/#28055 and forwarding closure #28056, H:1213800–1213809. Wrapper #28054 creates generator #28055 at H:1220728–1220734; #28056 forwards `this` and arguments through `.apply`, H:1220824–1220837.

### RPC primitive: #5508

Evidence: H:230590–230603, especially H:230596–230602.

```javascript
function appChangeCamera(flag) {
  return rpc("app_change_camera", { flag: flag }, {});
}
```

`rpc` here is the shared environment-slot-10 transport; its readable name is a reconstruction. This wrapper does **not** coerce the argument, validate its range, add a camera index, or add an arm-enable parameter.

### UI-level selector: #28055

Evidence: H:1220739–1220819; functional core H:1220747–1220763.

```javascript
async function changeCamera(wantMechanical) {
  try {
    this.navbar.onCameraSwitch();  // start UI transition indicator first
    const flag = wantMechanical ? 1 : 0;
    const result = await api.appChangeCamera(flag);
    console.log(/* flag, result */);
  } catch (error) {
    console.log(/* error */);
  }
}
```

There is no status-bit check inside this method; checks belong to callers. There is no local assignment to `CSM.mechanicalCameraEnabled`, no preview restart, no dedicated second stream, and no second RPC in the selector's normal path. The error logger even references global `param` (#28055 H:1220795); it must not be mistaken for an additional RPC parameter.

### All direct callers

| Caller | Exact evidence | Selection and gates |
|---|---|---|
| #28053 `onPressMechanicalCameraButton` | H:1220646–1220723; gates H:1220683–1220716; call H:1220717–1220722 | Requires arm active, then `canChangeCameraStatus`; calls `changeCamera(!CSM.mechanicalCameraEnabled)` in either direction. |
| #28210 `onPressCloseArmView` | H:1226682–1226767; selector H:1226713–1226727 | Immediate close branch requires `mechanicalArmGrabStatus === 0`; `_setGrapMode(0)`, hides arm view; if permission and mechanical camera selected, calls `changeCamera(!selected)` (therefore false/flag 0). Otherwise constructs exit confirmation. |
| #28212 confirmation `onPress` created by #28210 | H:1226789–1226833; selector H:1226804–1226821 | `_setGrapMode(0)`; if permission and mechanical selected, calls `changeCamera(!selected)` (false/flag 0); hides view and toggles minimap animation. |

#28210 and #28212 do not await `_setGrapMode(0)` before examining current permission/selection. Their gates therefore use the currently cached status, not a guaranteed fresh post-exit status. Neither exit path adds a separate activation test.

```javascript
// #28053, H:1220683–1220723
function onPressMechanicalCameraButton() {
  if (!RSM.mechanicalArmActiveStatus) {
    showToast(strings.localization_strings_mechanical_robot_control_toast);
  } else if (!RSM.canChangeCameraStatus) {
    showToast(strings.localization_strings_mechanical_arm_can_not_change_camera_toast);
  } else {
    this.changeCamera(!CSM.mechanicalCameraEnabled);
  }
}

// #28210 immediate branch; #28212 confirmation has the same selector gate
MAM._setGrapMode(0);  // no await at this call site
if (RSM.canChangeCameraStatus && CSM.mechanicalCameraEnabled) {
  this.changeCamera(!CSM.mechanicalCameraEnabled); // select non-mechanical
}
```

### Exhaustiveness checks

Whole-file exact opcode-string inventory, programmatically counted:

- `appChangeCamera`: 2 occurrences, H:224542 export and H:1220760 invocation.
- `changeCamera`: 4 occurrences, H:1213801 descriptor and H:1220717, H:1226722, H:1226816 calls.
- `canChangeCameraStatus`: 7 occurrences, all accounted for below.
- `mechanicalCameraEnabled`: 20 exact property occurrences, all accounted for below.

These counts concern exact `# String: 'symbol'` opcode comments, excluding longer log strings and localization object-buffer contents. They do not prove absence of a hypothetical dynamically computed alias, though no such alias was identified in this call graph.

## 2. Decoder and status dependencies

### Robot/common status: #8428 `parseR50Status`

Descriptor binding: **#8384**, H:338317–338320. Decoder body H:342063–342226.

```javascript
function parseR50Status(status) {
  this.common_status = status.common_status;
  this.clean_tidyup_status = status.clean_tidyup_status;
  this.assist_clean_status = status.assist_clean_status;
  // These bytecode expressions preserve the JS && falsy-input behavior.
  this.mechanicalArmGrabStatus = status.common_status &&
    ((status.common_status >> 6) & 15);
  this.mechanicalArmGrabMode = status.common_status &&
    ((status.common_status >> 10) & 3);
  this.mechanicalArmGrabResult = status.common_status &&
    ((status.common_status >> 12) & 1);
  this.assistedTidyUp = status.common_status &&
    ((status.common_status >> 13) & 1);
  this.mechanicalArmActiveStatus = status.common_status &&
    ((status.common_status >> 14) & 1);
  this.canChangeCameraStatus = status.common_status &&
    ((status.common_status >> 15) & 1);

  this.notInGrabMode = this.mechanicalArmGrabMode == 0;
  this.manualGrabMode = this.mechanicalArmGrabMode == 1;
  this.autoGrabMode = this.mechanicalArmGrabMode == 2;
}
```

Exact field ranges:

| Field | Extraction | Source |
|---|---|---|
| `mechanicalArmGrabStatus` | bits 6–9 | H:342107–342114 |
| `mechanicalArmGrabMode` | bits 10–11 | H:342115–342122 |
| `mechanicalArmGrabResult` | bit 12 | H:342123–342130 |
| `assistedTidyUp` | bit 13 | H:342131–342138 |
| `mechanicalArmActiveStatus` | bit 14 | H:342139–342146 |
| `canChangeCameraStatus` | bit 15 | H:342147–342154 |
| mode-name booleans | mode 0/1/2 comparisons | H:342155–342166 |

The decoder **does not derive permission from mode, grab status, or main robot state**. It extracts permission directly. A missing/zero `common_status` cannot be treated as permission granted. The log's Chinese label at H:342221 explicitly identifies bit 15 as “whether the mechanical-arm camera can be switched.”

Other grab-status names are decoded independently: 0 notStart, 1 isExitingDoor, 2 isEnteringDoor, 3 isWaiting, 4 isGrabing, 5 isPuttingDown, 7 isContinue, 9 isContinueToGrab, 10 isContinueToPutDown, 8 isMechanicalModeChanging. [#8428 H:342167–342203] None is checked by #28053 beyond the permission bit.

`parseR50Status` is called in both ordinary status and mechanical-arm status paths: **#8399** H:339893–339896 and **#8396** H:339598–339601. These are not proof that a particular deployed service requests/receives the necessary fields.

### Camera status: #8583 `parseCameraStatus`

Descriptor binding: **#8578**, H:347434–347437. Decoder H:347708–347811.

```javascript
function parseCameraStatus(cameraStatus) {
  this.hasGot = true;
  this.cameraEnabled = !((~cameraStatus) & 1);
  this.petModeEnabled = ((cameraStatus >> 1) & 1) === 1;
  this.realTimeMonitorEnabled = ((cameraStatus >> 2) & 1) === 1;
  // ...settings / privacy / video flags...
  this.mechanicalCameraEnabled = ((cameraStatus >> 15) & 1) === 1;
  // emit real-time video notifications, then:
  this.triggerListeners();
}
```

Mechanical bit: H:347778–347782. Listener notifications: H:347798–347810. No assignment to `mechanicalCameraEnabled` was found outside this decoder; the similarly named assignment in #28298 constructs an **image-resource dictionary**, not camera manager state.

Two observed input paths:

- **#8581**, H:347645–347656: dedicated `getCameraStatus()` response's `result[0]` → `parseCameraStatus`. Its local feature branch checks `Camera_DualLine` at H:347635–347644; that feature label does not establish physical sensor count.
- **#8425 `parseMonitorStatus`** (binding #8384 H:338305–338308), H:341923–341935: the status object's `camera_status` → `CSM.parseCameraStatus`, only when the elapsed camera lock interval permits and `cameraStatusLocked` is false. Both ordinary and mechanical-arm status parsing call this method; the mechanical path's call is #8396 H:339590–339591. Locked paths explicitly skip this loop. Thus stale app feedback is possible during camera-status locks. [#8425 H:341937–341970]

### Mechanical-mode status polling

The app has a distinct mechanical-arm status request path, not just a different screen:

- **#8450**, H:343118–343146: when particular status is supported, arm mode is supported, and `isInMechanicalModeMonitorPage`, selects `getStatusForMechanicalArm()` instead of `getStatusCommon()`.
- **#8458**, H:343407–343463: marks requested fields, including `CameraStatus` and `RobotCommonStatus`, then calls `getParticulaStatus(this.getHexState())` (vendor spelling preserved). It also requests State, Battery, CleanTime, ErrorCode, InCleaning, InFreshState, HomeSecConnectStatus, EnableHomeSecPassword, MonitorStatus, AdbumperStatus, VoiceChatStatus, DockError, CleanTidyStatus, Events, InMotionState.
- **#8458**, H:343490–343527: compares `msg_seq`, discards old/equal sequence responses in the following branch, and passes accepted `result` to `parseStatusForMechanicalArm`; **#8396** subsequently calls #8428.
- **#8446**, H:342995–343022: in the usual non-special branch, arm-support + mechanical Monitor page changes loop delay from 2000 to 1000 ms. Earlier high-freshness/motion-state branch can instead return 20000 ms; do not collapse the whole function to a universal one-second timer.

This is a strong read-only diagnostic clue for a future authorized experiment: collect **raw State, RobotCommonStatus/common_status, and CameraStatus/camera_status together**, along with sequencing/freshness. Do not conclude failure from stale bits or from an incomplete status subset.

## 3. The overlooked vendor UI clue, and the three different meanings of “mode”

Primary localization objects:

| Function / location | Key | Exact value |
|---|---|---|
| #7964 H:315728 | `localization_strings_mechanical_arm_can_not_change_camera_toast` | `Switching to the gripper camera is only available in Remote Control to Pick Up mode.` |
| #7963 H:315715 | same key | `仅支持在「遥控取物」模式下切换至机械手摄像头` |
| #7964 H:315728 | `localization_strings_mechanical_robot_control_toast` | `Robotic Arm disabled. Enable to use this function.` |
| #7964 H:315728 | `localization_strings_Setting_mechanical_arm_fetch` | `Remote Control to Pick Up` |

These dictionary entries were parsed from the complete object-buffer line, not inferred from a log. The first two strings are operationally connected to #28053's denied permission branch (H:1220704–1220716); the third to its inactive-arm branch (H:1220686–1220698).

### A. Main robot state: raw `state = 39` → symbolic `REMOTE_PICKUP`

**#8379** builds a state lookup table: `RobotState.REMOTE_PICKUP` is assigned as the `.value` of entry **39** at H:337311–337319. `RobotState` itself is a string-valued enum, not the number 39 (object H:337031–337033).

This is more than an array order guess: **#8393**, H:339310–339325 and H:339332–339344, indexes that same environment-slot-18 table by `parseInt(status.state)` and stores the entry's `.value` as `RSM.state`. The specialized mechanical decoder **#8394**, H:339505–339530, performs the same numeric lookup directly.

```javascript
stateTable[39] = {
  name: strings.localization_strings_Setting_mechanical_arm_fetch,
  value: RobotState.REMOTE_PICKUP  // string "REMOTE_PICKUP"
};
RSM.originState = status.state;
RSM.state = stateTable[parseInt(status.state)]?.value;
```

**This raw robot state 39 is not grabMode 2 and not camera selector flag 1.** Monitor update **#27984**, H:1217425–1217444, computes `isRemotePickUpMode = (RSM.state == REMOTE_PICKUP && RSM.mechanicalArmGrabMode !== 0)`. It copies `grabMode` from `mechanicalArmGrabMode` at H:1217452–1217456. The predicate does not require mode exactly 2.

### B. Pickup-mode RPC argument: Monitor explicitly requests 2

Bindings: #27920 H:1214283–1214292 identify #28208 `onPressChangeToArmView` and #28210 `onPressCloseArmView`.

```javascript
// #28208, H:1226621–1226663
function onPressChangeToArmView() {
  if (this.showArmDialog()) return; // arm not activated
  MAM._setGrapMode(2);              // no await here
  if (this.state.grabMode == 0) {
    this.isEnteringRemotePickMode = true;
    // reset timer; clear loading flag after 20000 ms
  } else {
    this.miniMapWithRemotePickupEntry.current.toggleAnimation();
    this.setState({showMechanicalArmView: true});
    this.isEnteringRemotePickMode = false;
  }
}

// #19898, H:858228–858256
async function _setGrapMode(flag) {
  await api.remoteArmAutoPickUpEnterExit(flag);
  return true; // catch returns false
}

// #5564, H:231663–231676
function remoteArmAutoPickUpEnterExit(flag) {
  return rpc("app_pick_up_enter_exit", {flag}, {});
}
```

`showArmDialog` **#28195**, H:1226185–1226225, returns true and constructs an activation dialog when `mechanicalArmActiveStatus` is false; otherwise false. The entry action therefore does not blindly enter arm mode when activation is absent.

**#28207 `handleLoadingStatus`**, H:1226550–1226579, completes UI entry when the entry-loading flag is set, reported `grabMode != 0`, and the minimap's current ref exists: switches minimap view and sets `showMechanicalArmView: true`. Again, it is a nonzero-status transition, not an equality-to-2 admission rule.

### C. Reported grab-mode names are internally inconsistent with some UI names

The decoder #8428 calls **reported mode 1** `manualGrabMode` and **reported mode 2** `autoGrabMode` [H:342159–342166]. However, navbar constructor **#28273** sets a local state named `manualArmMode` from **reported mode == 2** [H:1229905–1229909]. Therefore casual interpretation of the word “manual” in this plugin is unsafe.

The exact Monitor entry argument 2 is proven by bytecode, independently of the toast. It suggests that generic/manual arm deployment or an argument-1 route is **not a sufficient substitute for reproducing this particular vendor Remote Control to Pick Up UI flow**. It does **not** prove that firmware grants camera admission iff grabMode == 2, nor that camera flag 2 is required, nor that mode 1 can never permit switching. The app's ultimate switch admission is the returned common-status permission bit.

**Against the historical manual-mode recommendation:** the archive's reported recommendation to prefer manual mode must not be carried forward as a verified camera prerequisite. The primary vendor evidence instead names Remote Control to Pick Up and shows its Monitor entry requesting pickup argument 2. This corrects the evidentiary basis, not the firmware truth table. It remains possible that more than one pickup submode admits camera switching; the returned permission bit, not an English mode label, is the direct app gate.

## 4. Remaining UI gates and every property use

### Camera button wiring, visibility, and debounce

- **#28060** H:1221038–1221041 supplies `onPressMechanicalCameraButton` to the navbar; H:1220962–1220965 supplies `showMechnicalArmNav` from `showMechanicalArmView`. H:1220952–1220953 supplies `iconEnable` from `isMonitoring`.
- **#28288 `clickCameraSettingButton`**, H:1230780–1230785, invokes the callback and hides the settings popup. There is no additional selector argument.
- **#28303 `getLandscapePatrolView`** (descriptor #28272 H:1229837–1229840), H:1231266–1231268, copies `showMechnicalArmNav` to `showMechnicalArmSwitch`. The camera-button rendering branch H:1231816–1231824 requires that switch, a passed UI flag (register 18 / fourth explicit parameter), and `isSupportMechanicalArmMode()`. These exact rendering gates are proved for this landscape renderer, not generalized to an uninspected portrait implementation.
- Feature method **#5839**, H:240762–240773, checks `isNewFeatureStrSupport(MechanicalArmMode)`; this is a feature capability, separate from current arm activation.
- **#28303** H:1231844–1231851 disables the button while `isMechanicalCameraChanging` and renders its changing icon. It does **not** disable it based on `canChangeCameraStatus`; denial is handled by #28053's toast.
- **#28300 `onCameraSwitch`**, H:1231196–1231219, sets the changing flag and resets a 20000 ms timeout. #28301 H:1231224–1231233 clears the flag on timeout.
- **#28274**, H:1229916–1229924, calls `handleMechanicalCameraChanging`. **#28299**, H:1231169–1231191, clears changing when the cached prior camera boolean differs from the new manager boolean, then caches the new value. An RPC success result alone does not clear changing through this method.

```javascript
// #28299
if (this.isMechanicalCameraChanging &&
    this.isMechanicalCamera !== CSM.mechanicalCameraEnabled) {
  this.isMechanicalCameraChanging = false;
}
this.isMechanicalCamera = CSM.mechanicalCameraEnabled;
```

### All `canChangeCameraStatus` uses

| Function | Locations | Role |
|---|---|---|
| #8428 | H:342154, H:342212 | Assignment from common bit 15; diagnostic logging. |
| #28002 | H:1218503 | Exit/charge confirmation condition; not a selector caller. |
| #28053 | H:1220670, H:1220704 | Diagnostic log; camera-button admission gate. |
| #28210 | H:1226716 | Switch-back-on-close gate. |
| #28212 | H:1226808 | Switch-back-on-confirmed-close gate. |

**#28002** H:1218492–1218526 computes two blocking/confirmation conditions:

```javascript
const blockedSelectedMechanical =
  !this.state.showMechanicalArmView &&
  !RSM.canChangeCameraStatus && CSM.mechanicalCameraEnabled;
const busyVisibleArm =
  this.state.showMechanicalArmView && RSM.mechanicalArmGrabStatus != 0;
if (blockedSelectedMechanical || busyVisibleArm) {
  // special exit confirmation branch at bytecode address 0x28c
} else {
  // ordinary charging/exit flow
}
```

This is evidence the UI separately handles being stranded on a mechanical selection while switching is not admitted. It does not prove which firmware states create that condition.

### All exact `mechanicalCameraEnabled` property occurrences

| Function | Locations | Role |
|---|---|---|
| #8583 | H:347782, H:347791 | Decoder assignment; log. |
| #28002 | H:1218509 | Special exit confirmation condition above. |
| #28053 | H:1220673, H:1220720 | Log; toggle target. |
| #28055 | none | Receives desired boolean; does not assign/read manager selection. |
| #28181 | H:1225240, H:1225632 | Hides rocky toast / arm-target background when mechanical selected; surrounding gate requires monitoring, arm view, and arm support. See H:1225230–1225256 and H:1225621–1225636. |
| #28210 | H:1226720, H:1226725 | Selected-state gate; false toggle target on close. |
| #28212 | H:1226812, H:1226819 | Same in confirmation callback. |
| #28273 | H:1229898 | Initial copy into local `settingMechanicalCameraEnabled`. |
| #28298 | H:1231133, H:1231142, H:1231147, H:1231150 | Builds icon-resource dictionary and chooses displayed normal/transition icon. H:1231133 is **not** a manager state mutation. |
| #28299 | H:1231180, H:1231188 | Transition completion via status change and cached selection. |
| #28303 | H:1231873 | Loading-spinner placement according to current selection. |
| #28544 | H:1240869 | Rendering-local value, followed by `if (value) value = false` at H:1240870–1240871. This is not a camera change or admission command; no stronger meaning is inferred. |

## 5. Alternatives: what exists, and what was not found

### `set_camera_status` is a settings writer, not a demonstrated gripper selector

**#5342**, H:227564–227577, sends `set_camera_status([argument])`, not `{flag: argument}`. Its only exact `setCameraStatus` invocation is **#8585**, H:347963–347966.

**#8585 `updateCameraStatus`** constructs a packed settings value from camera enable, pet mode, monitoring, LED setting, privacy agreement, exploration, shown-alert, real video quality, object photo, object-photo privacy, real-time-two-keys status, and pet snapshot. Full packing H:347854–347927. Highest composed field is **petSnapshotEnabled shifted 14**; the writer never reads `mechanicalCameraEnabled` and never packs a mechanical-camera bit 15. [H:347925–347927, H:347963–347966]

Thus `mechanicalCameraEnabled` is readable in a status word but **not an editable gripper-selection field in this settings writer**. Hand-setting bit 15 with `set_camera_status` would be an unsupported firmware experiment, not reconstruction of an observed vendor UI route. Likewise, the fact this writer omits bit 15 does not establish whether firmware ignores, preserves, or resets that bit.

### Preview and native SDK boundaries

- **#5371** H:228084–228095 forwards its argument unchanged as `start_camera_preview` params. It is a preview primitive, not a demonstrated camera selector.
- **#29879** H:1296034–1296068 calls native `RRVideoViewManager.startCameraPreview(params)` or `startCameraPreviewWithTag(getHandle(), params)` depending on SDK API level. The additional “tag” is obtained from a UI handle; it is **not evidence of physical-camera ID selection**.
- The actual #28055 camera-switch path contains neither this native preview restart nor a new camera-specific URL or stream request.

Whole-file searches of camera-related opcode strings found camera RPC literals `get_camera_status`, `set_camera_status`, `start_camera_preview`, `stop_camera_preview`, and `app_change_camera`, but no dedicated mechanical/gripper-camera-enable RPC or alternate camera ID/index/type/channel selector. Exact selector references were individually traced rather than inferred from the total number of “camera” strings. This audit does not exhaustively reconstruct unrelated preview authentication or native implementation internals.

### Physical versus logical cameras

A boolean mechanical/non-mechanical selection is entirely compatible with **three physical sensors**. It does not explain how the two chassis sensors are used, whether both are externally streamable, or whether the non-mechanical mode changes internally. It also does not prove any native/P2P/go2rtc stream maps permanently to a particular sensor. `Camera_DoubleRGB` / `Camera_DualLine` feature names alone are not a topology map.

## 6. What is proven versus still a firmware hypothesis

### Static proof

- The app's camera-button target is mechanical/gripper, not merely an unnamed “second camera,” established jointly by the exact used toast, manager field, and selector code.
- App camera selector outputs are 0/1; all observed callers use booleans, not a third enumerated selector.
- Activation and switch permission gates are common-status bits 14 and 15; selected feedback is camera-status bit 15.
- Raw state 39 maps to symbolic REMOTE_PICKUP.
- Monitor's arm-view entry requests pickup-mode argument 2 via `app_pick_up_enter_exit`, while closing requests 0 and may switch back to camera flag 0.
- Reported grab-mode labels and a navbar `manualArmMode` variable cannot safely be treated as a consistent naming convention.

### Not established offline

- The exact firmware truth table relating main state 39, reported grabMode 1/2, door position, grab substatus, and permission bit 15.
- Whether pickup argument 2 is necessary in every firmware version or merely the vendor Monitor's chosen route.
- Whether a gripper camera request is rejected, acknowledged-but-ignored, or delayed when admission is absent.
- Firmware behavior for `app_change_camera({flag:2})` or nonstandard `set_camera_status` bit writes. No observed app caller justifies these experiments.
- Whether a selected-status transition guarantees a new gripper frame; whether a preview session must be refreshed externally; how chassis cameras multiplex; native SDK capabilities beyond this bytecode.

### Best new investigative lead

The vendor path points first to **entering the actual Remote Control to Pick Up workflow**, then examining **both raw status words and raw main state**, rather than assuming programming/manual arm deployment is enough or guessing a third camera selector number. A future authorized experiment could reproduce the observed pickup argument-2 entry, wait for fresh common-status activation/permission, request camera flag 1, then separately verify camera-status bit 15 and the actual frame viewpoint. This report authorizes no live action, and it does not claim that sequence has worked on the user's firmware.
