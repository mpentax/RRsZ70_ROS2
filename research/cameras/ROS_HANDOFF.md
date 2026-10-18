# Shared camera mode: HTTP, MCP and ROS

One physical robot selection feeds the existing shared go2rtc preview. Consumers do not own separate cameras.

## HTTP

- `POST /api/camera/select {"camera":"front"|"gripper"}` returns `{"camera":"<confirmed>"}`. It warms preview before a needed switch, sends `app_change_camera` with a 500 ms reply timeout, accepts that timeout as normal, then polls `get_camera_status` bit 15 for roughly 3 seconds. A missing bit flip returns 409 with pickup-mode/arm-out guidance. Other transport failures remain errors. No pickup/arm commands are sent.
- `POST /api/camera/frame {}` reads whichever camera is active; **never switches or restores front**. An explicit `camera` switches when needed and stays switched, including after a capture failure.
- Selection/capture requests are serialized. Status is live feedback cached for at most 1 second, refreshed on selection and known JPEG-size disagreement. Official-app switches are reflected.
- `X-Roboclaw-Camera` reports the live status camera. `X-Roboclaw-Size-Camera` independently infers front for 1600x1200 and gripper for 1600x896, otherwise `unknown`. Existing width/height headers come from actual JPEG bytes. Known mismatching frames are drained briefly, then rejected with 502 rather than mislabeled. There is no default-restored header.
- Lens/exposure verification remains unknown; bit/size agreement is a cross-check, not visual proof or calibration.
- Unsupported left/right/back/depth/ir/structured-light selectors return 422; invalid names return 400.
- Browser offers use the same shared active stream. Use the select endpoint to switch, not a camera argument on the offer.

## MCP

`select_camera({camera:"gripper"})` switches and stays. `get_camera_frame({})` is a non-switching read; an explicit camera argument switches and stays. Frame metadata includes `camera`, optional `sizeCamera`, and JPEG dimensions; no `requestedCamera`/`defaultRestored` claims. Gripper requires pickup mode with the arm out, established by the operator/controller, not these tools.

## ROS / Foxglove

The bridge keeps its default 3 Hz single poller and always posts `{}`. It routes each reported camera to:

| Camera | Compressed | Raw | Frame ID |
|---|---|---|---|
| front | `/camera/front/image_raw/compressed` | `/camera/front/image_raw` | `camera_optical_frame` |
| gripper | `/camera/gripper/image_raw/compressed` | `/camera/gripper/image_raw` | `gripper_camera_optical_frame` |

Only front frames publish `/camera/front/camera_info` using the measured front calibration. No gripper calibration or extrinsics are invented. A missing/invalid tag is not published as front. The dashboard already uses `/camera/gripper/image_raw/compressed`; unavailable-camera panels remain placeholders, not invented feeds. Previously received front images may remain displayed but their stamps stop advancing in gripper mode.

Live switch validation: `docs/validation/shared-camera-mode.md`.
