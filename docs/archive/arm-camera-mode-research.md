# Z70 arm/camera mode-transition compatibility

Research date: 2026-10-03. Scope: offline Android disassembly and static repository
review; public-documentation references below are inherited from the earlier review,
not fetched again for this update. No robot RPC, camera capture, HTTP request, mode/lens
write, or hardware trial was performed. Only this document was edited. Joint
command encoding and raw `app_arm_move` payload research belong to the separate
workstream and are deliberately not reproduced here.

## Conclusion

**Prefer `app_change_camera({flag:1})` within the existing manual/viewing session;
do not use a mode-2 detour as a camera prerequisite.** The primary Android trace
proves that the camera-button handler checks arm activation and the robot's
`canChangeCameraStatus`, **not `grabMode === 2`**, and sends the camera selector
without entering pickup mode, deploying/stowing, or restarting preview. This
establishes an app-side gripper-camera selection path without a flag-2 request;
actual gripper frames in mode 1 remain a firmware/hardware question.

**Android does implement direct 1↔2 switching inside `MonitorPage`.** Its tab
handlers send one mode flag, with no explicit intervening flag 0, compartment
request, recover request, camera selection, or preview restart. Thus the earlier
claim that no Android evidence supported a direct transition is superseded.
However, a transparent **pose-preserving** detour is still unproven: the app has
a changing-mode action and even anticipates arm-out loading for an action-0 2→1
switch. No physical-pose save/restore contract is present in these handlers.
The local controller still rejects manual→mode-2 entry.

The first useful compatibility experiment is **camera selection while remaining
in manual mode with an already-established viewing session**. Whether firmware
allows that combination, automatically changes lens with an action, or moves the
arm when starting/stopping viewing remains a hardware question. Only after those
variables are isolated should a deployed-arm mode detour be considered.

“Back camera” is treated here as the user's name for the gripper/arm camera.
Vendor documentation calls the two cameras chassis/front and gripper/arm; no
third rear-facing RGB feed was established.

## Evidence levels and provenance

- **Static local fact:** behavior in the current working-tree implementation.
  This checkout contains pre-existing edits; source line numbers below describe
  the inspected snapshot, not a released service or firmware contract.
- **Maintained upstream implementation:** third-party command/status mappings,
  useful evidence but not a vendor protocol specification.
- **Primary Android static fact:** independently inspected instructions in the
  user-provided `targeted-plugin/index.android.hasm`. Method names are recovered
  from export/property binding tables, then traced through generator bodies and
  callers. Function numbers and **1-based disassembly text lines**, not original
  JavaScript source lines, identify this exact artifact. Absence of an explicit
  JS request in a handler does not establish absence of firmware-side motion.
- **Vendor fact:** documented user-facing capability, not RPC semantics.
- **Hardware unknown:** requires an isolated, physically observed trial.

Primary sources consulted:

0. Primary offline artifact:
   `/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s/Z70Android/targeted-plugin/index.android.hasm`
   (1,518,814 lines), SHA-256
   `5fa398d080e801a31b7d55598ce8c696535808fbbbba6b5bac13c413d7ba4a2c`.
   Adjacent `index.android.bundle` SHA-256
   `bfd30c61c7969195251864d1296f464b732b1aefecd18d0ba86c51e1c511cc67`;
   it was hashed for provenance, not executed. All `H:` references below mean
   this `.hasm` file. No app/firmware version was independently established.
1. Local `README.md:339–414` (finite arm observations and Android parity),
   `README.md:178–220,468–494` (viewing and lens caveats);
   `robotics/ARM_CAMERA_FEASIBILITY.md:19–36`;
   `robotics/SOCK_PICKUP_TEST_PLAN.md:40–42`;
   `docs/camera-observations.md:7–18` and `docs/outcome-audit.md:29–31`.
2. [ioBroker A179 implementation](https://github.com/copystring/ioBroker.roborock/blob/2b4f802d4267d55f8492c9e1d3ccc1d98179d086/src/lib/features/vacuum/a179_features.ts),
   maintained `main` head resolved on the research date to
   `2b4f802d4267d55f8492c9e1d3ccc1d98179d086`. The retrieved `master` raw URL
   served the A179 implementation; `main` is the resolvable current branch.
   Relevant symbols: `Z70_GRAB_MODE_STATES`, `Z70_CAMERA_STATES`,
   `A179Features.setupProtocolFeatures`, `getCommandParams`,
   `updateDerivedCommonStatus`, `updateDerivedCameraStatus`,
   `normalizeCameraPreviewParams`. Historical local mapping reference:
   [1a3b5b79c2eaa33c1a257a7f1a17f749b1df59cb](https://github.com/copystring/ioBroker.roborock/blob/1a3b5b79c2eaa33c1a257a7f1a17f749b1df59cb/src/lib/features/vacuum/a179_features.ts).
3. [go2rtc `pkg/roborock/client.go`](https://github.com/AlexxIT/go2rtc/blob/c245815e75e2a5fd60b4290f12bfc04e55a984d3/pkg/roborock/client.go),
   maintained `master` head resolved to
   `c245815e75e2a5fd60b4290f12bfc04e55a984d3`. Relevant functions:
   `Dial`, `Client.Dial`, `Client.Connect`, `StartCameraPreview`,
   `StopCameraPreview`, `GetHomesecConnectStatus`, `Request`.
4. [Roborock Z70 product page](https://global.roborock.com/pages/roborock-saros-z70),
   “Dual Cameras, Multiple Home Views”: one arm camera and one front camera,
   with switching; “App Compatible”: manually controllable arm and chassis.
5. [Vendor FAQ: What can the robotic arm do?](https://support.roborock.com/hc/en-us/articles/48039609125017-What-can-the-robotic-arm-do),
   item 4: **when Remote Viewing is enabled**, switch chassis/gripper cameras.
   It separately describes Remote Control to Pick Up in item 3. It does not say
   mode 2 is required for every gripper-camera view or that mode entry preserves
   a deployed pose. The generic “retracts … when idle” description is not a
   mode-transition specification.
6. [Vendor robotic-arm tutorial page](https://support.roborock.com/hc/en-us/articles/47174912370713-Roborock-Saros-Z70-Robotic-Arm-Tutorial-Video)
   lists automatic pickup, manual pickup, and camera switching in remote control
   mode, and links [the tutorial video](https://www.youtube.com/watch?v=D9uEes3_SCQ).
   The page was read; the video was not frame-by-frame analyzed.
7. [Vendor safety FAQ](https://support.roborock.com/hc/en-us/articles/48039636271513-Is-the-robotic-arm-safe-Will-it-pinch-me-when-retracting)
   describes the physical Emergency Stop as stopping all robot operations.
   The README's [manual PDF attachment](https://support.roborock.com/hc/en-us/article_attachments/46800475911961)
   was reachable but returned binary PDF through the text fetcher; its contents
   were not used to assert transition semantics.

## Mode, action, and camera are different state dimensions

| Interface/state | Static meaning | What it does not establish |
| --- | --- | --- |
| `app_pick_up_enter_exit`, flag 0 | Exit/off mapping | Physical stow, stop, or pose reset semantics |
| Same method, flag 1 | Manual grab/control mode | Calibrated joint control or gripper video |
| Same method, flag 2 | Android auto tab / remote-pickup entry; separate from pickup-start action | Autonomous pickup has started, arm deployed, or selected lens |
| `app_arm_out_compartment` / `app_arm_in_compartment` | Separate deployment/stow operations | Mode entry is physically inert |
| `app_change_camera`, flag 0/1 | Android viewing-page camera switch: false/true mechanical camera | Verified sensor frames, two simultaneous feeds, or no firmware-side arm motion |
| `common_status` bits 10–11 | Grab mode code 0/1/2 | Pose |
| `common_status` bits 6–9 | Action code; 0 not-started, 3 waiting, 8 changing-mode; 1/2 leaving/entering compartment | Completion or absence of intermediate motion |
| `common_status` bit 15 | Upstream `canChangeCameraStatus` | Which sensor produced a frame |
| `camera_status` bit 15 | Upstream `mechanicalCameraEnabled` | Same meaning as common-status bit 15, verified active lens |

Local mappings: `src/control-policy.ts:47–53`; decoding:
`src/arm-feedback.ts:35–75`. That decoder labels mode 2 `auto`, while the remote
pickup methods correctly test `modeCode === 2`. Do not conflate this enum label
with a pickup action. Maintained A179's `updateDerivedCommonStatus` and
`updateDerivedCameraStatus` independently decode the two bitfields. The existence
of a “can change” hint and action 8 suggests state-dependent transitions, but does
not reveal the firmware's lens-admission truth table.

## Primary Android trace

### RPC bindings and mode semantics

The export table connects `appChangeCamera` to #5508 (`H:224541–224542`),
`remoteArmAutoPickUpEnterExit` to #5564, `remoteArmGoBack` to #5565 and
`remoteArmOut` to #5567 (`H:224861–224868`). Their complete wire wrappers are:

| Function | Lines | Dispatch |
| --- | --- | --- |
| #5508 `appChangeCamera` | `H:230590–230603` | `app_change_camera`, object `{flag: argument}` |
| #5564 `remoteArmAutoPickUpEnterExit` | `H:231663–231676` | `app_pick_up_enter_exit`, object `{flag: argument}` |
| #5565 `remoteArmGoBack` | `H:231681–231692` | `app_arm_in_compartment`, `{}` |
| #5567 `remoteArmOut` | `H:231713–231724` | `app_arm_out_compartment`, `{}` |
| #5506 `appArmRecover` | `H:230544–230557` | `app_arm_recover`, supplied object or `{}` |

`MAM._setGrapMode` is bound in #19892 (`H:857955–857962`) through #19897/#19899
to generator **#19898** (`H:858220–858291`). It awaits exactly
`remoteArmAutoPickUpEnterExit(argument)`, logs, returns `true` on a normally
resolved promise and `false` after a caught exception. It does not inspect
`result[0]`, read status, wait for mode/action completion, or call arm-out/in,
recover, camera selection, preview, or joint restoration. Its boolean is
**promise-resolution UI feedback, not physical completion**.

The primary decoder #8428 (`H:342063–342226`) independently establishes:
`grabStatus = (common_status >> 6) & 15` (`H:342107–342114`),
`grabMode = (common_status >> 10) & 3` (`H:342115–342122`),
arm activation bit 14 (`H:342139–342146`), camera-switch admission bit 15
(`H:342147–342154`). Mode 0/1/2 becomes not-in/manual/auto at
`H:342155–342166`; actions 0/1/2/3 become not-started/out-of-compartment/
into-compartment/waiting at `H:342167–342179`; action 8 is
`isMechanicalModeChanging` (`H:342200–342203`). **“Active” means activated,
not physically deployed**, and “auto mode” does not mean a pickup has begun.

### Camera selection without flag 2: what is proved

`MonitorPage.onPressMechanicalCameraButton` **#28053**
(`H:1220646–1220723`; name binding `H:1213795–1213798`) implements:

```text
if !RSM.mechanicalArmActiveStatus: activation toast; return
if !RSM.canChangeCameraStatus: cannot-switch toast; return
changeCamera(!CSM.mechanicalCameraEnabled)
```

There is **no grab-mode or deployment/action-code check** in this handler.
`MonitorPage.changeCamera` generator **#28055**
(`H:1220739–1220819`; binding `H:1213801–1213808`) marks the navbar as
switching, converts its boolean to 0/1 (`H:1220751–1220757`), and awaits
`appChangeCamera(flag)` (`H:1220758–1220765`). It only logs/catches afterwards;
no mode or pose command and no preview teardown/start is bundled.

The button is wired into the monitor navbar at `H:1221038–1221041` and
navbar `clickCameraSettingButton` **#28288** invokes that prop
(`H:1230764–1230787`). The monitor renders manual controls with
`isVideoPage:true` in **#28147** (`H:1223982–1224012`), and the same page
contains both manual and auto tabs (**#28189**, `H:1225900–1225983`). Thus
“manual control” and “video/remote pickup” are not mutually exclusive UI pages.
The separate settings manual-control screen is a different lifecycle.

Visibility is also traced: `getNavbar` **#28060** passes
`showMechanicalArmView` as `showMechnicalArmNav` (`H:1220963–1220965`), and
navbar renderer **#28303** displays the camera switch when that arm-panel prop,
its enabled-icon input and mechanical-arm feature support hold
(`H:1231816–1231849`). This condition is **arm-panel visibility, not mode 2**:
#27990/#27991 show the panel for REMOTE_PICKUP + activated arm + **any nonzero
grabMode**, including mode 1. The camera-button handler itself has the two
admission checks above. UI visibility should not be mistaken for a firmware
mode-2 requirement.

The navbar tracks the old `isMechanicalCamera` and clears its changing flag
when decoded `mechanicalCameraEnabled` differs: **#28299**
(`H:1231169–1231191`). **#28300/#28301** also clear loading after 20 seconds
(`H:1231196–1231233`). The app uses a status transition/timeout, not sensor
identity in decoded video, as its switching indication.

**Proof boundary:** flag 2 is not an app-side prerequisite for the selector.
This is not proof that this firmware will set `canChangeCameraStatus` in every
mode-1 pose, accept the selector there, expose a third rear RGB camera, or
actually route fresh gripper frames to go2rtc. Treat “back view” as the arm/
gripper camera unless independently identified otherwise.

### Camera-status bit versus stream routing

CameraStatusManager decode **#8583** (`H:347708–347811`) reads bit 15 as
`mechanicalCameraEnabled` specifically at `H:347778–347782`. The same field
drives the navbar icon and switch-completion state. This supports its role as
**camera-selection feedback in the viewing UI**, rather than merely an unrelated
configuration preference; it is still not image-level lens verification.

Crucially, CameraStatusManager `updateCameraStatus` generator **#8585**
(`H:347827` onward) reconstructs settings only through bit 14:
`H:347843–347927`, then calls `setCameraStatus(value)` at
`H:347963–347966`. It never reads `mechanicalCameraEnabled` or shifts it into
bit 15. The wire wrapper **#5342** sends `set_camera_status([value])`
(`H:227564–227577`). The Android camera-switch path instead calls #5508.
Therefore **writing camera-status bit 15 is not established Android parity for
stream selection**, nor an evidenced equivalent fallback for a missing selector.
The serializer omitting bit 15 does not prove firmware clears it on settings
writes: writable/read-only semantics belong to firmware.

### Manual-control screen versus monitor/pickup lifecycle

| UI event | Exact function / lines | App-side writes and implications |
| --- | --- | --- |
| Settings `MechanicalManualRemoteControlPage` mount | `componentDidMount` #33252, `H:1451879–1451938`; screen-specific telemetry #33302 at `H:1453425` | Calls `checkLocal(true)`, displays `showNotLocalMsg`, registers status/app-state listeners, and `_pauseRobot` (#33256 sends `pause`, `H:1452037–1452078`). No preview or explicit compartment/recover request in mount. |
| Manual prompt confirmation | `showNotLocalMsg` #33258 / `onPress` #33259, `H:1452100–1452156` | Prompt's confirmation sends `_setGrapMode(1)`. It is not an arm-out request appended to flag 1. |
| Settings manual screen back | #33253, `H:1451943–1451969`; dialog #33260/#33262, `H:1452161–1452227` | Action 0 pops immediately; other actions show exit dialog, whose confirmation pops. |
| Settings manual screen unmount | #33263, `H:1452232–1452274` | Unconditionally sends flag 0, calls `_stopMotion(0,true)`, saves chassis-control UI index, removes listeners. No explicit compartment-in/recover, no pose restoration. |
| Monitor mount / existing arm state | #27982, `H:1217217–1217367`; `updateArmStateView` #27990, `H:1218005–1218095`; `updateArmModeView` #27991, `H:1218100–1218181` | Reflects existing robot state/activation/nonzero grab mode in arm UI and chooses manual/auto tab; invokes preview-start admission through network/dock handling. Mount does not itself send flag 2 or arm-out. |
| Monitor arm-view entry | `onPressChangeToArmView` #28208, `H:1226604–1226663` | After `showArmDialog` #28195 activation gate (`H:1226171–1226225`), sends flag 2 without awaiting it. If old grabMode is 0, starts 20-second entry-loading UI; otherwise opens arm UI immediately. No explicit arm-out/in/recover or camera switch. |
| Monitor mode-change UI completion | `handleLoadingStatus` #28207, `H:1226546–1226599` | Entry-loading clears on **any nonzero** grabMode, not specifically mode 2. Arm-out loading clears on **any nonzero** grabStatus; timeout is not pose proof. |
| Monitor close arm panel | `onPressCloseArmView` #28210 and confirmation #28212, `H:1226682–1226833` | Action 0 exits directly; otherwise confirms. Both send flag 0, hide arm panel, and conditionally switch mechanical camera off if admission and bit are true. They do not explicitly stop preview or call arm-in/recover. |
| Monitor full back | #28002, `H:1218466–1218692`; callbacks #28003/#28004, `H:1218697–1218742`, #28007, `H:1218785–1218852` | Nonzero arm action with panel visible, or mechanical camera enabled while panel hidden and camera switching unavailable, uses arm-exit confirmation; confirmation stops chassis remote lifecycle and pops. Other idle-state back paths can offer return-to-charge; confirming that separate prompt calls `charge` then pops. Neither dialog is a pose-preserving detour. |
| Monitor unmount | #27961, `H:1216201–1216452` | Stops chassis-motion lifecycle; when robot state is REMOTE_PICKUP sends flag 0 (`H:1216264–1216284`); stops viewing via `stopMonitor` (`H:1216389–1216390`). No saved-pose restoration or explicit compartment-in/recover in this function. |
| Arm settings page mount | #33453, `H:1459431–1459466` | If arm activated, sends flag 0. Merely navigating to settings can therefore change control mode. |
| Arm settings recover/stow operation | `offMechanicalArm` generator #33442, `H:1459064–1459149` | Sets `isEnteringDoor`, calls `appArmRecover({confirm:1})` at `H:1459109–1459115`, with a 10-second UI timeout. This is a separate deliberate operation, not camera-toggle/tab-switch cleanup. |

Explicit compartment buttons are distinct: MAM `_onPressArmOutButton` generator
**#19910** (`H:858643–858709`) calls #5567; `_onPressArmGoBackButton`
**#19913** (`H:858742–858808`) calls #5565. Shared manual control button
**#29624** (`H:1285814–1285836`) calls arm-in; **#29625**
(`H:1285841–1285879`) calls arm-out only when its local arm-in condition holds
**and `!isVideoPage`**, otherwise grip. Its mount **#29613**
(`H:1284118–1284140`) only installs a status listener; unmount **#29615**
(`H:1284184–1284203`) removes it and clears input timers. These are not automatic
mount deployment/stow requests.

**Does UI automatically start arm out or in?** At the JS-command level, mode
entry/tab selection does not append either compartment RPC. The monitor's
`changeToManualControlLoading` **#28190** (`H:1225988–1226024`) specifically
anticipates arm-out/loading when switching from **mode 2, action 0** to manual;
this flag clears on action becoming nonzero or after 10 seconds (#28207/#28191).
That is evidence of expected firmware-side deployment during that transition,
not a separate JS arm-out dispatch or proof of its final pose. Mode-2 entry opens
the pickup controls; **starting pickup is a separate button** (#28230 calls
`_remoteArmPickUp(2)`, `H:1227789–1227898`). Likewise `app_pick_up(flag:1)` is a
separate manual-pickup workflow (manual button #28521→#28523→#28532→#5568,
`H:1240214–1240278,1240505–1240527,231729–231742`), not mode-entry flag 1.
Exit flag 0 has UI intent consistent with ending arm operation, but firmware
stow/retract timing cannot be deduced from that intention.

Recover is not a pose-preservation primitive: the settings operation above sets
enter-compartment UI state; the separate manual-reset guide #15607 also issues
`appArmRecover({confirm:1})` (`H:649391–649463`). The main-page recovery handler
#22690 instead calls `appArmRecover()` with no argument
(`H:987968–987993`). None of those call chains supplies a previous pose.

For completeness, the other `_setGrapMode` call sites found in the full artifact
are lifecycle boundaries, not hidden camera-selection callbacks:

- `RemoteControlPageNew.stop` generator **#33563** first awaits `remoteEnd`,
  then sends flag 0 if state is REMOTE_PICKUP (`H:1464211–1464291`). Its
  unmount **#33577** stops motion and conditionally sends flag 0 on that same
  state (`H:1464650–1464688`). This is distinct from the settings *manual arm*
  screen's unconditional flag-0 unmount #33263.
- The exhibition-control lifecycle **#33716** sends flag 0 on mount when the
  mechanical-arm feature is supported (`H:1470455–1470528`); its back callback
  **#33717** sends flag 1 before popping (`H:1470533–1470574`). This is a
  special exhibition workflow, not evidence of an ordinary camera detour.

Together with #27961, #28208/#28210/#28212, #29718/#29721, #33259/#33263,
and #33453 above, these account for all direct `_setGrapMode` property-call
sites returned by the artifact-wide search. No such call is present in the
camera selector or inspected preview dispatch bodies.

### Direct manual↔remote-pickup switch and pose preservation

The auto/manual switch component **#29710** receives `grabMode`,
`changeToManualControlLoading`, and `onToggle` (`H:1289104–1289176`). Its
auto button is bound to **#29718** (`H:1289238`, auto label `H:1289286`), and
manual button to **#29721** (`H:1289327`, manual label `H:1289373`).

```text
auto tab (#29718): if already auto-selected, return
                  mark request 2; MAM._setGrapMode(2).then(#29719)
manual tab (#29721): if grabMode === 0, show preparing toast; return
                    changeToManualControlLoading()
                    if auto-selected: mark request 1;
                                      MAM._setGrapMode(1).then(#29722)
```

Complete handlers: `H:1289615–1289673` and `H:1289692–1289768`.
The success callbacks update local tab selection and set a 12-second request
timeout; **#29716** (`H:1289559–1289597`) reconciles reported `grabMode` and
clears pending UI state. `onToggle` parent **#28192**
(`H:1226043–1226065`) only changes `currentGrabMode` and the displayed controls.
`selectTab` animation **#29712** (`H:1289454–1289506`) similarly calls the
parent with 1/2; it is not another mode RPC.

**There is no action-0/stowed admission check for manual→auto in #29718.**
The handler has no flag-0 detour, arm-in/out, recover, camera selector, preview
start/stop, or joint snapshot/replay. Thus direct **app-requested** 1→2→1 is
supported by primary evidence within an already-open monitor. Its changing-mode
feedback and mode-2/action-0→manual arm-out loading make **physical neutrality
an unjustified assumption**. Switching whole screens adds the flag-0 exit and
preview/session lifecycle described above and is not equivalent to these tabs.

This supersedes the earlier inherited interpretation in `README.md:383–406`:
the inspected entry handler sends flag 2 alone; a separate arm-out command
exists, but is **not chained by `_setGrapMode` or #28208**. README is not edited
in this docs-only workstream. Also read “no mode entry automatically selects a
gripper lens” narrowly: there is no explicit selector in these entry handlers;
implicit firmware routing remains unknown.

### Preview start/stop and app state

`MonitorPage.startMonitor` generator **#27969** (`H:1216589–1216847`)
queries `getHomeSecConnectStatus`, refuses a foreign running owner, leaves an
already-running same-client session in place, and starts only from disconnected
status. Its parameters are **quality, password, client_id**, not lens or arm mode
(`H:1216800–1216843`). It calls `videoView.startCameraPreview_IoT`; view adapter
**#29844** (`H:1294811–1294838`) delegates to
`VideoManager.startVideoPreview`, generator **#29905**
(`H:1297009–1297157`), which dispatches preview and then negotiates TURN/SDP/ICE.
Wire wrapper **#5371** (`H:228084–228095`) forwards those parameters to
`start_camera_preview`. None of these inspected start bodies selects a camera
or sends an arm mode/compartment/recover request.

`MonitorPage.stopMonitor` generator **#27975**
(`H:1216983–1217051`) is gated by `hasCallStart`; marks disconnecting, ends
video operation and calls `videoView.stopCameraPreview_IoT({client_id})`.
Adapter **#29841** (`H:1294753–1294778`) stops local rendering and delegates
to VideoManager stop **#29908** (`H:1297190–1297252`), which resets its configs,
guards duplicate stop with `hasCallStopPreview`, and forwards to wire wrapper
**#5372** (`H:228100–228111`) → `stop_camera_preview`. These bodies have no
explicit arm-mode/stow/lens command. `hasCallStart` means adoption of an already
running same-client stream and teardown are not necessarily symmetric.

Navigation blur **#27925** (`H:1214911–1214950`) changes orientation/fullscreen
only; it does not itself stop preview or exit arm mode. App background is
different: `_handleAppStateChange` **#27932** (`H:1215103–1215182`) calls
`stopMonitor` and `remoteEnd`; foreground checks connectivity and schedules
preview-start admission. Network/dock handling **#28015**
(`H:1219000–1219108`) can also stop/start admission. `checkShouldAutoRetry`
**#27972/#27973** (`H:1216892–1216967`) schedules repeated start admission
at 500 ms with a retry counter. Consequently UI tab switching contains no
intentional preview restart, but background/network failure can break continuity.
Native renderer implementation and firmware effects of preview RPCs remain
outside this JS disassembly.

## Supported local transition lifecycle

| Transition | Exact local function and precondition | Local side effects |
| --- | --- | --- |
| Off → manual → deploy | `ArmController.action("deploy")`, `src/arm-controller.ts:85–105`; off/not-started | Ends chassis RC, sends flag 1, waits `CONTROL_LIMITS.startupSettleMs` (5 s), reads fresh feedback requiring manual/state 39, then separate arm-out |
| Off → remote pickup | `enterRemotePickupMode`, same file:109–117; off/not-started plus `#ready` blockers | Sends flag 2 once, then reads state; no settle/positive mode-2 postcondition, no arm-out/in, recover, or joint restoration |
| Mode 2 → deploy | `deployFromRemotePickup`:118–126; state 39, mode 2, not-started | Separate arm-out once; ACK and state read do not verify pose |
| Manual → mode 2 | `enterRemotePickupMode`:112–113 | Rejected before dispatch; synthetic regression at `src/arm-controller.test.ts:64–69` |
| Manual → off | `exitManual`:192–200; manual/not-started | Flag 0 once, then read; a deployed manual/waiting arm fails this precondition |
| Mode 2 → manual/off | No generic local transition operation | `exitManual` rejects mode 2; supervised docking explicitly requires remote pickup exit in vendor app (`src/service.ts:384–396`) |
| Manual waiting → stow | `action("stow")`:100–103 | Separate arm-in; mode exit is not bundled |
| Mode 2 compatible action → stow | `remoteCompartment`:156–167; state 39, waiting/grasping/putting-down | Separate arm-in; no mode exit bundled |

**Off/not-started is a local admission requirement, not a proven vendor firmware
requirement.** The primary Android tabs request direct 1→2→1 without a stowed
check, but do not establish firmware acceptance or physical pose preservation.
Conversely, a single flag-2 wire request with no explicit stow/recover in the
caller does not prove the firmware preserves joints. There is no local save and
restore of a joint state. The supported manual→stow→off→mode-2 path already
changes physical pose and therefore cannot implement a transparent detour.

Mode/status reads are not measured pose. Historical owner-confirmed deployment
and an upward step both returned `[0,0,0]` from `app_get_arm_joints_data`
(`README.md:347–363`). Equality before/after cannot prove joint preservation.

## Viewing startup/teardown can confound a mode test

- `Go2Rtc.frame`, `offer`, `preload`, `unload` in `src/go2rtc.ts:100–185`
  respectively request a frame, create a viewer, retain a consumer, or remove a
  retained consumer. Frame acquisition can start a producer if none exists;
  removing the hold does not close other consumers. Source `configure`:201–215
  sets an in-memory source, not an explicitly retained viewing session.
- Maintained go2rtc `Client.Connect` queries preview ownership, **stops the
  existing returned client ID** until it sees `none` (bounded loop), then starts
  its own HD preview, obtains TURN configuration, and negotiates WebRTC. It sends
  no explicit arm mode, deploy/stow, or lens-selector request. The firmware's
  consequences of these preview RPCs are not defined by this client. Its built-in
  preview-stop loop means a new producer is unsuitable for a strict single-write,
  no-retry experiment without first controlling the session boundary.
- The local service checks that ownership is `none` or go2rtc's ID before offer/
  frame (`src/service.ts:1414–1456`), reducing but not eliminating the race with
  another app. A timeout may leave remote viewing active. A frame response labels
  the lens and exposure time unknown.
- The alternate LAN diagnostic `CameraSession.start` in `src/camera.ts:240–281`
  refuses foreign ownership, checks PIN and starts preview once. `stop`,
  `#cleanup`, `#cleanupLateStart`:367–418 include a later scoped stop after an
  unresolved late start. This is different from go2rtc and is not proof of arm
  neutrality. `CameraPreview` in `src/preview.ts` wraps this diagnostic lifecycle.
- Repo guidance explicitly says viewing may disturb arm mode
  (`README.md:212–218`; `docs/robotics-next-milestones.md:18`). This run found no
  reproducible hardware trace establishing exactly which start/stop event moves,
  stows, resets, or changes mode. Historical retained-stream/off/on experiments
  did not identify a gripper view (`robotics/SOCK_PICKUP_TEST_PLAN.md:40`).

The mechanical-camera handler (`src/service.ts:1163–1201`) sends flag 0/1 and
reports `feedLensVerified:false`. Only explicit RPC method-not-found `-32601`
allows a fallback read-modify-write of camera-status bit 15; timeout, transport
error or missing ACK does not authorize a second write. This setting fallback is
not evidence that it selects the active stream identically. A controlled trial
must identify which path actually ran; it must not unknowingly combine a failed
selector and fallback into a supposed one-variable test.

`cameraExclusive` (`src/service.ts:602–615`) rejects an active authority writer,
arm/chassis operation, or docking reservation; the robot proxy also blocks
conflicting writes while camera lifecycle owns the boundary (:509–531). The
mechanical handler rejects an unexpired browser preview lease, while an explicit
hold can retain the feed between operations. These checks serialize local RPC
activity, **not the whole physical deployed-arm interval**. They do not lock the
vendor app or prove the producer will never reconnect. Persistent hold is a
consumer-lifetime aid, not an arm-mode guarantee.

## Cancellation and controller implications

`ArmController.#run`, `#assert`, `#send`, `cancel`, `close`
(`src/arm-controller.ts:68–80,201–215`) use a revision, permission checks, a 45 s
authorization window and the current connection identity. Cancellation invalidates
future sends; it sends no arm stop, stow, mode exit, or camera restoration.
Cancellation **after** a successful wire ACK can still fail the post-send check,
so a cancelled response can describe an already-executed transition. `finally`
clears local busy/ownership, not physical motion.

`web/src/Arm.tsx:18–43` cancels a pending operation on blur, offline, pagehide,
hidden visibility or unmount. Opening the vendor app during a pending browser
transition can therefore invalidate subsequent local steps without reversing the
first step. Service disconnect handling (`src/service.ts:492–499`), logout
(:889–895), robotics cancel (:365–369), and controller replacement (:480–483)
similarly cannot establish cessation or restore an old pose.

A future detour controller would need one explicit transition owner across the
whole sequence, measured physical baseline, fresh action/mode observations after
each step, and session/lens continuity tracking. It must never replay old joint
commands or “restore” saved zero telemetry after a cancelled/uncertain transition.
The present finite controller does not provide that detour lifecycle.

## Hardware questions still open

1. Can gripper video be selected in mode 1 while holding a deployed pose? Is
   admission tied to mode, `canChangeCameraStatus`, deployment, action, or a
   combination? Does action 8 temporarily reject switching?
2. The Android UI requests direct entry from manual without a stowed check; does
   this firmware accept it from manual/waiting? Does firmware stow, move to
   a canonical pose, reset its joint target state, release a grasp, or only change
   control semantics? What does the return to flag 1 do?
3. Does preview startup/stop/reconnection cause those same side effects? Does lens
   choice persist across mode/action changes, preview restart, or arm stow?
4. The app treats mechanical-camera bit 15 as selection feedback, but is routing
   immediate or action-dependent? What are the firmware's writable-bit semantics
   for `set_camera_status`, whose Android serializer omits bit 15? There is no
   primary evidence that the local status-bit fallback is selector-equivalent.
5. What are latency, frame discontinuity, exposure freshness, and independent
   physical stop behavior during these transitions?

Static sources cannot settle these questions. A successful ACK, mode 2, enabled
bit, or unchanged uncalibrated joint array is insufficient evidence.

## Evidence-backed integration recommendation

1. **Separate three operations:** viewing-session lifetime, camera selection, and
   arm-mode transition. Expose front/gripper *selection request* through
   `app_change_camera({flag:0|1})`; use common-status activation bit 14 and
   camera-switch admission bit 15 as the
   Android admission evidence, and camera-status bit 15 as reported selection
   feedback. Only identifiable fresh scene content establishes the active lens.
2. **Use the current manual mode and retained producer first.** Android has a
   camera-button path without flag 2 and manual controls within the viewing page.
   Do not automatically enter mode 2 or deploy the arm to obtain a camera frame.
   If camera admission is false, report that state rather than changing mode as
   a hidden recovery step.
3. **Do not claim `set_camera_status` bit-15 fallback is parity.** The primary
   Android settings serializer ends at bit 14; its live camera
   switch dispatches the distinct selector. The current service's fallback
   should be treated as a separately unvalidated operation in future integration
   work, not evidence that gripper routing succeeded. No code change made here.
4. **Keep direct mode switching experimental until measured.** The trace now
   provides a concrete vendor UI path: switch tabs inside one existing
   `MonitorPage`, with a direct mode request and reported mode/action monitoring.
   It does not justify relaxing the local controller's admission policy yet or
   implementing a momentary camera detour that promises pose preservation.
   Especially avoid interpreting mode-2/action-0→manual as inert: the UI expects
   arm-out loading there. Whole-screen navigation additionally sends flag 0 and
   may end viewing; it is not a transparent mode detour.
5. **Keep physical operations explicit and distinct.** Compartment in/out,
   recover `{confirm:1}`, pickup workflow flags, and mode-entry flags have separate
   meanings/call chains. UI loading flags, resolved promises, or a mode match must
   not be reported as stow/deployment/pose completion. Do not restore a prior
   pose by replaying unvalidated telemetry after a transition.

## Small controlled trial sequence (proposal only; no automatic retries)

Use an empty gripper, stationary robot off the dock, clear arm envelope, nearby
operator with the physical stop, and external visual/measurement evidence of arm
pose. Exclude competing app/service writers and automatic producer restarts.
Record firmware/app versions, transport path, preview owner/session, command and
ACK times, mode/action/common/camera bits, and visible physical changes. A
separate target visible to the gripper lens should distinguish it from the front
view without touching or obscuring the mechanism.

1. **Establish viewing while stowed/off.** Start one explicitly owned viewing
   session once; observe whether startup changes mode/action or arm pose. Retain
   that same producer. If readiness is uncertain, stop the experiment and inspect
   fresh state; do not reopen/retry it automatically.
2. **Manual baseline.** Use the already-supported manual/deploy workflow once.
   Wait for fresh manual/waiting feedback and independently stationary arm.
   Record physical pose and current identifiable view. Do not send joint payloads
   or pickup/put-down workflow commands in this camera experiment.
3. **Selector-only test.** With local command ownership ended and the same viewing
   session retained, request mechanical-camera selection once through an audited
   single-selector path. Avoid the unexpired browser lease restriction and
   account for the existing handler's method-not-found fallback. Observe mode,
   action, pose and genuinely new scene content. If it gives a verified gripper
   view without motion in mode 1, the mode detour is unnecessary for that tested
   configuration. Do not equate enabled status with a successful view.
4. **Separate supported mode-2 baseline if step 3 is inconclusive.** After an
   operator-directed, verified stow and manual exit, enter mode 2 once from
   off/not-started; observe before any deployment. Deploy separately once only
   with compatible fresh feedback, then test the selector once with the same
   retained producer. This characterizes mode-2 viewing, not pose preservation.
   Remote pickup exit uses the vendor app's supported lifecycle, with other
   writers paused and each physical effect recorded.
5. **Direct detour is a separate hardware question, not a current tool sequence.**
   The primary Android in-monitor tab path establishes an app-requested direct
   manual→pickup transition with no JS stowed check. If separately authorized
   for hardware characterization, conduct a single transition
   from an externally measured stationary pose, inspect, then a separately
   authorized return and inspect again. The current service rejects this path;
   do not bypass it with raw HTTP/RPC. No repeated toggle loop or pose restoration
   from joint telemetry. If the UI bundles stow/deploy or camera restart, report
   that bundle rather than attributing its effects to the mode flag alone.
6. **Cleanup as a measured event.** Operator-directed stow/exit as supported,
   physically verify, then release only the trial's viewing consumer/session and
   observe whether teardown changes mode/pose. Do not dock as part of this test.

At every stage: busy, timeout, missing ACK, unexpected motion/mode, loss of video
continuity or cancellation ends the planned sequence. Use fresh read-only state
and independent physical observation to resolve uncertainty; use the physical
stop if needed. No automatic command retry, fallback mode detour, compensating
stow, or replay. Trial success would establish compatibility only for the tested
firmware, lifecycle, action and measured pose, not arbitrary joint control.
