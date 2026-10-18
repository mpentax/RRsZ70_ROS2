# Gripper-camera activation: Android source findings

## Result

**The strongest evidenced route is the official Remote Control to Pick Up viewing workflow, followed by `app_change_camera({flag:1})` when the robot reports that switching is allowed.** This is a recovered app path, not a hardware-verified activation recipe. No live robot/service access or motion was performed.

The owner reports **three physical cameras: two chassis cameras, one built into the gripper**. The plugin's binary viewing selector does not enumerate the hardware or establish whether both chassis sensors are streamable.

## Recovered app flow

Primary source: `/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s/Z70Android/targeted-plugin/index.android.hasm`. `H:` references are one-based disassembly text lines.

1. The Monitor's arm-view entry calls `_setGrapMode(2)` (#28208, H:1226621–1226632). Its wrapper sends **`app_pick_up_enter_exit({flag:2})`** (#5564, H:231663–231676). This flag belongs to pickup mode, NOT camera selection. It is separate from the start-pickup action.
2. The camera button requires **arm activated** (`common_status` bit 14) and **switch admitted** (`common_status` bit 15). When permission is absent it displays the exact vendor text: **“Switching to the gripper camera is only available in Remote Control to Pick Up mode.”** The English and Chinese locale entries are used by the executable denial branch, not merely unused strings. (#28053, H:1220683–1220723; locale H:315715/315728; decoder #8428, H:342139–342154.)
3. With permission, the app passes its desired mechanical-camera boolean to `changeCamera`, which sends **`app_change_camera({flag:1})`** for mechanical selection and flag 0 for non-mechanical. It does not restart preview, deploy/stow, or send another mode command inside this selector. (#28055, H:1220747–1220763; #5508, H:230590–230603.)
4. The app observes **`camera_status` bit 15** as `mechanicalCameraEnabled`, and clears its switching indicator after that reported boolean changes. An ACK alone is not the app's transition feedback, and neither the bit nor the ACK identifies the physical source of a decoded frame. (#8583, H:347778–347782; #28299, H:1231169–1231191.)

### Keep the three state dimensions separate

- Raw robot `state = 39`: Remote Control to Pick Up state (#8379, H:337311–337319).
- `common_status` bits 10–11: reported grab submode; decoder labels 1 manual and 2 auto.
- `app_change_camera` flag: binary desired viewing selection, 0/1.

The button does not directly require `grabMode == 2`; its actual gate is the robot's permission bit. Thus the entry argument 2 is proven app behavior, but an exclusive firmware mode-2 requirement is not proven. Generic manual/programming arm deployment must not be treated as sufficient camera preparation without checking admission.

## Why no third camera number was found

The full plugin selector call graph contains no observed camera-flag-2 caller or independent gripper-camera-enable method. Preview callers construct **`{quality, password, client_id}`**, with no camera/channel parameter (H:1216801–1216828). Native `WithTag` arguments are React Native view handles (`findNodeHandle(this.root)`, #29889, H:1296409–1296420), not camera IDs.

The APK's recovered receive path is `RRVideoViewManager` → Roborock home-security client → WebRTC. In `classes4.dex`, `com.roborock.homesec.OooOO0O.onAddTrack`, code offset `0x1a292c`, enables the received VideoTrack and installs a sink without selecting a track by camera name, ID, stream ID or MID. Robot-side source switching behind a stable video track is consistent with this implementation, but not established by static analysis.

**Do not infer `app_change_camera({flag:2})` from three physical cameras.** Do not use `set_camera_status` bit 15 as an equivalent selector: the inspected settings serializer does not write the mechanical-camera field.

## Additional leads, not activation commands

- **Arm-end sensor terminology:** error key `end_tof_error73` maps to “Robotic arm's end camera error.” It does not establish whether RGB, depth, or a combined module supplies the preview.
- **Stored object photos:** `get_photo` with `data_filter:{img_id,type}` has an RSA security envelope, gzip and photo parsing. It may supply independent image evidence, but no live gripper capture or sensor provenance is established. Types 0/1 are not evidenced physical camera indexes.
- **Misleading TOF API names:** `getTofStatus` / `setTofStatus` actually send `get_debug_voxel_map` / `set_debug_voxel_map`. These are separate sensing/debug leads; Z70 support is unknown.

## Best next hardware test — not performed

A separately authorized supervised test should reproduce the vendor Monitor workflow rather than guess IDs:

1. Establish one working preview session and record fresh raw state, common_status and camera_status.
2. If not already in the evidenced pickup-view workflow, request pickup entry argument 2. This can change arm mode/pose; do not assume physical neutrality or automatically restore a saved pose.
3. Read fresh status; proceed with selection only when arm activation and camera-switch permission are reported. Record grab submode and action as well.
4. Send camera flag 1 once, observe camera-status selection feedback, and independently verify frame identity against the actual gripper location/viewpoint. Keep the preview session intact initially; the selector does not include a restart.
5. Stop on unexpected motion or ambiguous delivery. If selection feedback changes but pixels remain chassis-view, investigate session/codec/track behavior separately. The app supports conditional SD/HD startup; an HD-only client is not exact quality parity, but there is no evidence that quality is a gripper prerequisite.

An image visibly originating at the gripper is required before claiming access. The app warns that ending the arm task automatically retracts it; exit is not a pose-preserving cleanup primitive.

## Audit artifacts and coverage

- [Selector and status audit](plugin-selector-audit.md): complete observed selector callers, admission, feedback, workflow and polling.
- [Stream and sensing-image audit](plugin-stream-audit.md): preview/native bridge, photo transport, TOF/debug paths.
- [Native APK audit](native-apk-audit.md): DEX bridge, WebRTC receive path and resolved false positives.
- [Locale evidence](locale-evidence.md): independent extraction and current local alias check.

Parent independently spot-checked the primary camera handler, selector and pickup wire wrappers, entry argument, both status-bit decoders, preview object and native track-handler disassembly. APK inventory independently reproduced: 2,312 entries, five DEX files, 36,727 class definitions, 90 native libraries. Native machine-code bodies and nested plugin bundles were not exhaustively reverse-engineered.

Verified source hashes:

- `.hasm`: `5fa398d080e801a31b7d55598ce8c696535808fbbbba6b5bac13c413d7ba4a2c`
- APK: `13dbd693115ab925af5dc6b3b4b3d100d777f6173952a3d9185eb533a5c07ee6`

The historical archive's recommendation to prefer manual mode as the first camera route, its physical two-camera assumption, and its older controller/service/fallback references are superseded as current guidance by this audit. Its underlying historical bytecode citations may still be useful.
