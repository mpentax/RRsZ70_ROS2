> **Supporting historical evidence — 2026-10-18 curation.** The original audit/replay body below is retained verbatim. Static negative findings and implementation statements are bounded to the inspected sources; replay is not a second physical acquisition. Current behavior: [camera guide](../cameras/FINDINGS.md) and [arm model](../../docs/z70-arm-model.md).

# Gripper-camera locale evidence (offline primary-source extraction)

Source: `/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s/Z70Android/targeted-plugin/index.android.hasm`.
SHA-256 verified during this investigation: `5fa398d080e801a31b7d55598ce8c696535808fbbbba6b5bac13c413d7ba4a2c`.
All line numbers are disassembly text lines, not original JavaScript.

## Camera-specific admission message

English locale object, line 315728:

- `localization_strings_mechanical_arm_can_not_change_camera_toast`: **Switching to the gripper camera is only available in Remote Control to Pick Up mode.**

Chinese locale object, line 315715:

- Same key: `仅支持在「遥控取物」模式下切换至机械手摄像头`.

Primary handler #28053, lines 1220683–1220723, uses this exact key when `RSM.canChangeCameraStatus` is false (1220704–1220715). Earlier in the handler it rejects an unactivated arm (`mechanicalArmActiveStatus`, 1220686–1220698). Otherwise it invokes `changeCamera(!CSM.mechanicalCameraEnabled)` (1220717–1220722).

The message specifically identifies the target as the **gripper camera**, not an unspecified second chassis camera. It is stronger evidence of intended product behavior than an unlabeled flag alone. However, the handler does NOT compare `grabMode` to 2; firmware supplies the admission bit. The user-facing Remote Control to Pick Up workflow name cannot by itself establish the complete mode/action/pose requirements or prove a received image came from the gripper.

## Arm-end sensor clue

English locale object, line 315728:

- `localization_strings_Setting_mechanical_arm_end_tof_error73`: **Robotic arm's end camera error. Please try again.**

The key associates an arm-end camera error with TOF terminology. This is a lead for sensor identity, not proof whether a separate RGB sensor, a depth sensor, or a combined module supplies the user-viewable stream. The physical three-camera inventory is supplied by the owner; a binary selector in the application does not disprove it or establish exposure of all physical cameras.

## Exit warning

Same English locale object:

- `localization_strings_video_mechanical_arm_control_exit_dialog`: **End current task, and the robotic arm will automatically retract.**

Treat camera experiments involving UI exit or arm-mode changes as potentially moving the arm. Do not assume selector/mode/session transitions preserve pose.

## Current local implementation check

`src/vendor-rpc.ts:39–41` maps `/api/agent/camera/mechanical` with explicit boolean `enabled` to `app_change_camera` and `{flag:1}` / `{flag:0}`. It does not write `set_camera_status` as a fallback. The archived report's older service/controller line references and fallback behavior are not current implementation evidence.

No live robot, camera, Docker, service or network access was performed for this evidence extraction.
