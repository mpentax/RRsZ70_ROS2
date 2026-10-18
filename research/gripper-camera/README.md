# Gripper-camera research and current status

## Current status

Gripper activation is no longer an unperformed proposal. Retained supervised acquisition delivered **1600×896 gripper** pixels (front **1600×1200**) through the selected stream with pickup mode `app_pick_up_enter_exit {flag:2}` and the arm out. See [current camera behavior and calibration](../cameras/FINDINGS.md), [ROS handoff](../cameras/ROS_HANDOFF.md), and [retained pincher lens/arm experiment](../../calibration/arm-calibration/20261015T080103Z-pincher-intrinsics-arm/RESULTS.md).

The owner inventory remains **three physical cameras: two chassis and one gripper**. The two implemented views do not enumerate every physical sensor or prove an additional chassis stream is accessible. No flag-2/side/rear/depth selector should be guessed.

MCP `select_camera({camera:"gripper"})` changes shared selection and leaves it selected. `get_camera_frame({camera:"gripper"})` only filters the paired ROS cache; it does **not** switch or open preview. ROS driver owns acquisition and publishes matching CameraInfo for both lenses. The [adopted gripper resource](../../src/z70_description/config/z70_gripper_camera_info.yaml) records exact asymmetric K/D and adoption provenance; camera/arm anatomy now comes from [the CAD-backed model](../../docs/z70-arm-model.md), distinct from historical optical marker registration and contact proxies.

The selector tolerates the expected reply timeout and confirms `camera_status` bit 15; status/dimension agreement is not independent exposure or physical-lens verification. It does not deploy the arm or enter pickup mode. Static UI permission bits below are app provenance, not extra current service prechecks. This page is not a standing hardware authorization or a claim of a newly executed live check.

## What the static audits established

The original owner-supplied HASM/APK inputs are external/private, not portable repository dependencies. `H:` refers to one-based disassembly lines of the hashed plugin. Detailed findings and negative-search limitations are preserved in:

- [Plugin selector/status audit](plugin-selector-audit.md): Monitor entry `_setGrapMode(2)` → `app_pick_up_enter_exit {flag:2}` (#28208/#5564); app button checks `common_status` bits 14/15 (#28053/#8428); selector `app_change_camera {flag:0|1}` (#28055/#5508); feedback `camera_status` bit 15 (#8583/#28299).
- [Stream and sensing-image audit](plugin-stream-audit.md): preview `{quality,password,client_id}`, native `WithTag` as React Native view handle, explicit cloud SDP wrapper, stored-photo transport, and TOF/debug leads. `getTofStatus` maps to `get_debug_voxel_map`, not a live depth image API.
- [Native APK audit](native-apk-audit.md): inspected DEX/WebRTC receive path, search coverage, hashes/offsets and false positives. A received track without camera-name selection is consistent with shared source switching, not an encoder/hardware census.
- [Locale evidence](locale-evidence.md): exact denial/exit wording and independent extraction. The denial text restricts switching to Remote Control to Pick Up; ending the arm task is not a pose-preserving operation.

Keep raw state 39, grab submode (`common_status` bits 10–11), app admission bits, and vendor camera flag 0/1 distinct. No inspected flag-2 caller was found. `set_camera_status` bit 15 is not an equivalent selector. Those are bounded client-source findings, not a complete firmware truth table.

Verified source hashes:

- `.hasm`: `5fa398d080e801a31b7d55598ce8c696535808fbbbba6b5bac13c413d7ba4a2c`
- APK: `13dbd693115ab925af5dc6b3b4b3d100d777f6173952a3d9185eb533a5c07ee6`

The older archive's manual-first/no-mode-2 recommendation and two-physical-camera assumption are superseded. Preserve its underlying bytecode citations as dated provenance, not current operating instructions.
