# Android plugin camera / stream audit (offline)

## Result

**No additional camera-channel RPC, native camera/track selector, or dedicated gripper-image endpoint was found in the inspected JavaScript bytecode.** This is a bounded negative finding, not a claim that the robot has only two physical cameras. The user's physical observation is **three cameras: two chassis, one gripper**. The plugin exposes a binary *viewing selection* and generic sensing/debug features; those are not a physical sensor inventory.

The useful additional leads beyond `app_change_camera` are:

1. Reproduce the app's **preview session parameters and native signaling lifecycle**, rather than treating a selector acknowledgment as a new camera session. The app supplies `{quality, password, client_id}` to the robot preview RPC; no camera/channel field is constructed by the examined callers.
2. The native `WithTag` variants select a **React Native view handle**, not an evidenced camera number. Do not try tag `1`/`2` as lens selectors.
3. The plugin has a **stored object/tidy-up photo transport**: `get_photo` with `data_filter:{img_id,type}`, an RSA/security envelope and gzip processing. This could provide independent images associated with arm tidy-up objects, but neither live capture nor gripper sensor provenance is established.
4. `getTofStatus` / `setTofStatus` are surprisingly **debug voxel-map** wrappers, not arm-end-camera streaming RPCs. They lead to `get_debug_voxel_map` / `set_debug_voxel_map` and a point-cloud web viewer. This is a separate sensing/debug lead, with no demonstrated Z70 support or sensor identity.
5. The `end_tof_error73` identifier really maps to the arm-end-camera error UI, but **does not identify RGB versus depth, a distinct sensor, or a stream channel**.

No robot, Docker, service, network, or native application execution was performed. No credential files were read. Only this report was written; existing checkout changes were not altered.

## Source and citation conventions

`H:` means 1-based text lines in:

`/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s/Z70Android/targeted-plugin/index.android.hasm`

Verified artifact: 107,407,171 bytes; SHA-256 `5fa398d080e801a31b7d55598ce8c696535808fbbbba6b5bac13c413d7ba4a2c`.

Adjacent `index.android.bundle`: 13,330,092 bytes; SHA-256 `bfd30c61c7969195251864d1296f464b732b1aefecd18d0ba86c51e1c511cc67`. Hashed/read for provenance, not executed. All behavioral findings below are from disassembly instructions, not interpretation of raw bundle string proximity. The historical `docs/archive/arm-camera-mode-research.md` was read for comparison; primary findings were independently located in the disassembly.

Function numbers identify this exact build; many disassembly function names are `value` or anonymous. Semantic names below are derived from export/property binding tables and call sites, not original source symbols.

Broad scans covered camera/preview/stream/channel/track, gripper/pickup/arm images, RGB/depth/TOF, native bridges, photo and debug functions. Long locale-object lines were handled as bounded fragments separately, not mistaken for executable channel-selection code. Framework hits such as color `rgb`, zlib streams, telemetry tracks, UI slider tracks and `maxDepth` animation geometry were separated from camera evidence.

## 1. Preview parameters: robot command versus native renderer

### Robot-side request

| Evidence | Exact function and lines | Finding |
|---|---|---|
| Preview wire wrapper | **#5371 `ue`**, `H:228084–228095`, bytecode `0x09–0x19` | Loads `start_camera_preview`, passes its sole supplied argument unchanged to the shared RPC helper, with a separate empty options object. This wrapper adds no camera or channel field. |
| Monitor caller | **#27969 `?anon_0_`**, object construction and call `H:1216800–1216843` | Creates a new object, sets `quality`, `password` from `MC.gesturePassword`, and `client_id` from `appClientID`, then calls `videoView.startCameraPreview_IoT(params, callback)`. Quality is `SD` when `isVideoMonitorDefinitionSupported()` is true, otherwise `HD`. |
| Other viewing caller | **#30039 `?anon_0_`**, `H:1302832–1302866` | Same three fields, with `quality:SD`. No sensor selector. |
| Quality literals | **#8329**, `H:334081` | Object is exactly `{'SD':'SD','HD':'HD','FHD':'FHD','AUTO':'auto'}`. These are string values, not channel numbers. |
| Preview manager | **#29905 `?anon_0_`**, `H:1297009–1297072` | Passes first parameter to `startCameraPreview` (`H:1297033–1297034`). On returned `result[0] === 'ok'`, starts TURN retrieval, device SDP polling and device ICE polling. |

Recovered caller shape (symbolic credentials/identity only):

```text
start_camera_preview({
  quality: "SD" or "HD",
  password: MC.gesturePassword,
  client_id: appClientID
})
```

**Actionable:** compare a future authorized capture session's field names/types, client identity and signaling lifecycle with this exact request. There is no app-side evidence here for adding `camera_id`, `channel`, `track`, or a flag to preview parameters. Because #5371 forwards its argument unchanged, this does not prove firmware would reject extra fields; it merely does not justify inventing them.

### Native start is a different operation with the same name

**#29852 `value`**, `H:1294967–1294980`, calls the native-facing `videoView.startCameraPreview` with a newly created callback (**#29853**), not the robot request object. **#29879 `value`**, `H:1296018–1296070` (instructions `H:1296022–1296068`), forwards that argument to:

- SDK API level < 2: `RRVideoViewManager.startCameraPreview(callback)`;
- API level >= 2: `RRVideoViewManager.startCameraPreviewWithTag(getHandle(), callback)`, if the method exists.

The app's TURN-to-SDK callback **#29835**, `H:1294647–1294665`, calls its own `startCameraPreview()` after TURN delivery. Robot preview authorization and native decoder/peer startup therefore must not be conflated.

## 2. Signaling and native bridge routing

### Signaling wrappers

| Wire operation | Function | Instruction references |
|---|---|---|
| `get_turn_server` | **#5373 `ie`** | `H:228116–228128`, method literal `H:228123` |
| `send_sdp_to_robot`, `{app_sdp:argument}` | **#5374 `se`** | `H:228132–228153`; object write `H:228140`; explicit `callMethodFromCloud` reference `H:228147` and method literal `H:228148` |
| `send_ice_to_robot`, `{app_ice:argument}` | **#5375 `_e`** | `H:228158–228171`; object write `H:228166`, method literal `H:228168` |
| `switch_video_quality`, `{quality:argument}` | **#5376 `pe`** | `H:228176–228189`; object write `H:228184`, method literal `H:228186` |
| `get_device_sdp` | **#5377 `ce`** | `H:228194–228205`; literal `H:228201` |
| `get_device_ice` | **#5378 `le`** | method literal `H:228217` |
| `get_homesec_connect_status` | **#5381 `me`** | method literal `H:228265` |

Device SDP retrieval **#29917 `?anon_0_`** reads `result.dev_sdp` and calls `sdpCallback` (`H:1297545–1297577`). ICE retrieval **#29923 `?anon_0_`** reads `result.dev_ice` and calls `iceCallback` (`H:1297694–1297724`). Neither examined function parses a camera identifier, selects a media track, or constructs an alternate camera session. No actual negotiated SDP is in this evidence, so video media-section count, SSRCs, track identity and codec cannot be established offline.

### `WithTag` is a UI-instance target

**#29889 `value`**, `H:1296409–1296420`, implements `getHandle` as `findNodeHandle(this.root)`. Thus the extra first argument in the `WithTag` calls is a React Native node handle.

The same version-dependent handle routing appears in:

- **#29880 `value`**, TURN to SDK: `H:1296075–1296118`;
- **#29881 `value`**, SDP to SDK: `H:1296123–1296166`;
- **#29882 `value`**, ICE to SDK: `H:1296171–1296214`;
- **#29883 `value`**, current SDK status: `H:1296219–1296269`;
- **#29877 `value`**, current-frame capture: `H:1295915–1295957`.

**#29859** binds the public video-view methods at `H:1295224–1295373`: fullscreen, microphone, recording, current-frame capture, preview start/stop, TURN/SDP/ICE delivery, SDK status/version/handle and render. No camera/channel/track-selection method is present in this inspected wrapper class. **#29890 `value`**, `H:1296425–1296483`, forwards `this.props` and registers native events including SDP, ICE candidate, first rendered frame and frame-stuck (`H:1296460–1296473`); because props are forwarded, this wrapper alone cannot rule out a native prop not used by the inspected callers.

**Actionable:** a future native-app comparison can use `getCurrentSDKStatusInfo[WithTag]`, `onFirstFrameRendered`, `onFrameStuck` and `captureCurrentFrame[WithTag]` to distinguish renderer/transport failure from selector state. Native capture samples the current decoder; it is not an evidenced fresh gripper-camera acquisition RPC.

**Limit:** **#6133**, `H:249928–249929`, obtains `NativeModules.RRVideoViewManager`; **#6134**, `H:249944–249974`, refers to `RRVideoViewManagerTurboModule`. Native module implementations are outside this Hermes JS disassembly. Additional capabilities inside native binaries remain possible. No conclusion about all native APIs or robot firmware APIs is justified.

## 3. Independent still-image route: tidy-up/object photos

This is a genuine route other than preview video, but **stored photo retrieval**, not a discovered live gripper camera command.

### Photo request and transport

**#5022 anonymous**, `H:218401–218429`, is bound as `getPhotoBase64Data` (export reference `H:205662`). It builds:

```text
get_photo({
  data_filter: {img_id: suppliedPhotoId, type: suppliedType},
  ...security envelope added by helper
})
```

The exact `data_filter` construction is `H:218410–218416`; `DecryptorType.gzip` and `get_photo` are `H:218417–218422`. Its success callback **#5023**, `H:218434–218453`, calls `parsePhotoData` and reports failure if parsing/decryption fails. Do not expect an ordinary plaintext JPEG RPC response.

The helper chain is independently recoverable:

- **#4657** stores **#4673** in environment slot 48 at `H:205334–205335`; #5022 loads that slot at `H:218408–218409`.
- **#4673 `Q`** forwards to **#4674 `Z`**, `H:207243–207277`, which creates **#4675 → #4676**.
- **#4676 `?anon_0_`**, `H:207294–207359`, awaits `MC.checkRsaKey()` (`H:207315–207319`), constructs `endpoint:'xxx'`, and `security:{pub_key:JSON.parse(MC.rsaKey.pub), cipher_suite:1}` (`H:207322–207340`), then merges supplied request fields (`H:207342–207346`). These are program constants/property names, not accessed real credentials.

### Link to arm tidy-up objects

**#18419 `value`**, `H:791858–791925`, passes `tidyStatus`, `tidyupPhotoVisible`, `props.photoId` and `obstacleShowFlag` into the tidy-up object bubble (`H:791909–791918`).

The bubble's photo hook **#18465** exposes `requestPhoto` (`H:794537–794631`). Its implementation **#18469**, `H:794675–794773`, checks `mapObjectPhotoEnabled`, uses the `MC.mapObjectPhotos` cache, and on a cache miss calls `getPhotoBase64Data(photoId, 1, callback)` (`H:794702`, `H:794714–794732`). **#18470**, `H:794779–794840`, consumes `photoData` and caches/displays it. This is real data retrieval, not simply an illustration asset.

For comparison, the larger object-photo path **#19667**, `H:849133–849230`, calls `getPhotoBase64Data(photoId, 0, callback)` at `H:849176–849182`, and **#19668**, `H:849235–849280`, consumes `photoData`, `photoSize`, `rectInfo` and `photoNonce`. **Types 1 versus 0 are used by thumbnail/bubble versus larger-photo retrieval; there is no evidence that they mean chassis versus gripper.**

The tidy-up photo UI toggle **#27525**, `H:1196083–1196101`, only changes local `isHiddenPhotos`; **#27526**, `H:1196105–1196131`, shows the corresponding arm tidy-up-photo toast. This is not a camera activation write.

**Actionable:** retain real map/tidy-up `photoId` values from existing authorized data, implement the complete photo security/decompression/parsing path, and compare image content with camera locations. This can provide an independent sensing-image clue when live preview does not change. It cannot identify the source sensor from its API name or request `type`, and does not provide a way to trigger a current gripper frame.

## 4. TOF / debug map: a misleading name with a real separate route

The RPC exports **#5259** bind:

- `getTofStatus` → **#5554** at `H:224791–224792`;
- `setTofStatus` → **#5600** at `H:225088–225089`.

Their actual wrappers are:

- **#5554 anonymous**, `H:231503–231514`: `get_debug_voxel_map`, empty array parameters;
- **#5600 anonymous**, `H:232360–232373`: `set_debug_voxel_map`, `{status:argument}`.

Debug UI **#25403 `?anon_0_`** converts a toggle to 0/1 and calls `setTofStatus` (`H:1114990–1114997`). This is a debug-map switch. It is not an evidenced gripper camera power, RGB stream switch or end-TOF reset.

**#26524 `value`**, `H:1156579–1156651`, renders a WebView using the fixed URL prefix:

```text
https://qomolangma.roborock.com:56443/maps/pcdviewer/index.html?key=
```

followed by its `localKey` and `&pcdid=` plus `sn` (`H:1156599–1156606`), then places that computed URI in the WebView (`H:1156644–1156647`). No URL was opened and no actual key/serial value was inspected. This is a point-cloud/debug viewer transport lead, not a video-camera channel.

**Limit:** this is a multi-model plugin. The presence of a debug feature does not show Z70 firmware implements it, which physical sensor supplies it, whether it is current data, or whether there is an accessible local PCD download. Treat this as a separate bounded investigation, not a gripper-stream activation recipe.

## 5. Arm-end-camera error 73 and physical sensor identity

English locale object `H:315728` contains:

```text
localization_strings_Setting_mechanical_arm_end_tof_error73
  = "Robotic arm's end camera error. Please try again."
```

The executable association is **#16042 anonymous**, `H:674406–674412`, bytecode `0x725–0x741`: obtains that locale property, puts it into a one-element array, and assigns it to the `error73` key. This is an arm error-message mapping, not an activation/transport function.

The name `end_tof` is worth preserving for a firmware/native trace, but **does not prove the visible gripper stream is depth, prove a separate TOF versus RGB imager, or equate error73 with one of the user's three physical cameras**. No end-TOF pixel format, image buffer, depth-array transport, or sensor ID is attached to this mapping.

Generic model-feature definitions in **#8011** should not be applied to the Z70 without tracing model assignment; directly observed feature literals in the disassembly include `mf.camera_singleline`, `mf.camera_dualline`, `mf.camera_rgb` and `mf.camera_doublergb` at `H:316880–316887`. These are capability names, not a Z70 physical-layout or stream enumeration. The generic end-camera error and generic model features must remain separate from verified hardware identity.

## 6. Why the original selector lead is still incomplete

The independent status decoder **#8583 `value`**, `H:347778–347782`, computes:

```text
mechanicalCameraEnabled = (((camera_status >> 15) & 1) === 1)
```

That is selection-state feedback, not decoded image provenance. Native current-frame capture and first-frame events likewise only demonstrate the selected renderer receives video, not which physical camera produced it.

`app_change_camera` is the observed binary viewing selector (**#5508**, method literal `H:230600`), but a binary viewer does not enumerate the three physical cameras. No second camera-channel wire wrapper surfaced in the full symbol scans. In particular, the only remaining camera-like bridge family `RRPluginCamera` inspected here is used for QR image generation (**#4692**, `H:207925–207998`; **#5080**, `H:219871–219899`) and saving images to the phone (**#4971**, `H:216970–217029`). Phone camera permission and `getSystemCameraForPhoto[V2]` are not robot stream selectors.

### Prioritized next investigations (not executed)

1. **Native binary offline audit:** find the implementation of `RRVideoViewManager`, preview start, SDP/ICE handoff, `captureCurrentFrame`, and incoming rendering props; inspect camera/track/channel APIs there. The JS wrapper stops at this boundary.
2. **Session-parity comparison in a separately authorized trial:** correlate admission state, camera bit, preview request, media/first-frame callbacks and actual image identity. Separate command acceptance, selected status, usable transport and proven gripper pixels. Preserve a single active viewer/session when testing the existing in-stream selector; this bytecode does not establish a restart requirement.
3. **Stored photo decoding:** use existing tidy-up photo IDs with the evidenced type/security/gzip path; examine whether image content exposes gripper viewpoint. Do not relabel the two `type` values as physical camera selectors.
4. **Debug voxel-map/native firmware research:** inspect the response schema and native/cloud point-cloud handling offline before any authorized query or switch. No end-camera RGB/depth relationship is proved.

### What remains unknown

- Which of the three physical cameras are preview-capable, and which are sensing-only.
- Whether two cameras share a combined hardware module or imaging pipeline.
- Whether the live stream changes pixels after selection, whether SDP/SSRC changes, or whether firmware additionally requires a specific pickup/control state.
- Whether stored tidy-up photos come from the front, another chassis camera, gripper camera, or a composed/cropped pipeline.
- Whether the debug voxel-map operations are implemented on this robot.
- Native implementation and firmware-only selectors not invoked by this plugin.

**Bottom line:** new transport/verification and stored-sensing-image clues are real; a second or third independently addressable live camera channel is not established by this artifact.
