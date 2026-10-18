# Supported camera views and evidence

## Current contract

The owner reports **three physical cameras: two chassis cameras and one gripper camera**. The implemented selectable video views are **front** and **gripper**; that API inventory is not a census of physical sensors. No rear/side/depth/IR selector is evidenced by the inspected clients, and unsupported named feeds must not become placeholder panels or guessed flags.

| View | Delivered JPEG | ROS optical frame | Installed intrinsics source |
|---|---|---|---|
| front | 1600×1200 | `camera_optical_frame` | [front CameraInfo](../../src/z70_description/config/z70_front_camera_info.yaml) |
| gripper | 1600×896 | `gripper_camera_optical_frame` | [gripper CameraInfo](../../src/z70_description/config/z70_gripper_camera_info.yaml) |

The adopted gripper model has **separate fx/fy**, an asymmetric principal point and `plumb_bob` with fitted k1 (remaining D coefficients zero). Use the exact K/D in the linked resource, not the old approximate symmetric initialization. Its `calibration_source` and `adoption_status` identify the [retained lens fit](../../calibration/arm-calibration/20261015T080103Z-pincher-intrinsics-arm/RESULTS.md) and later adoption. Front K/D remain the original lens; [front floor extrinsics](../../calibration/front-camera/README.md) are provisional. Lens adoption is not anatomical/contact/precision validation. CAD now supplies arm/camera anatomy; [the arm guide](../../docs/z70-arm-model.md) distinguishes CAD camera/marker TF from historical optical marker reconstruction and contact proxies.

## Shared selection and acquisition

- ROS driver owns acquisition, scheduling and selection; web/MCP clients are thin readers/controllers of that shared state. See [ROS handoff](ROS_HANDOFF.md).
- MCP `select_camera({camera:"gripper"})` (or `front`) calls `/api/ros/camera/select`, backed by `/z70/camera/select`. Selection stays shared; there is no automatic restoration to front.
- MCP `get_camera_frame({camera:"gripper"})` calls `/api/ros/frame` and **filters the paired ROS cache** (maximum age five seconds). It never selects or opens preview; `{}` is also a non-switching read. A camera filter is not a selector.
- The private backend frame route is not the public cache contract. The driver alone polls `/api/camera/frame` with `{}` at a **target** 3 Hz; slow calls can reduce delivered cadence. The retired public `/api/camera/select` is not the current route.
- Gripper selection has been observed with pickup mode `app_pick_up_enter_exit {flag:2}` and the arm out. The selector does not enter mode or deploy. The [current selector](../../adapters/vendor/src/camera-selection.ts) tolerates only the expected 500 ms reply timeout, then confirms `get_camera_status` bit 15 within its polling window. Unrelated transport failures remain failures. A confirmed bit flip does not fabricate an RPC reply or prove lens pixels.
- `common_status` app UI admission bits, pickup submode, raw robot state 39 and `camera_status` selection bit are different fields. They are not a physical-camera count, nor an instruction to enforce static app button gates in the service.
- Dimension/status mismatch is drained then rejected, not relabeled. [go2rtc configuration](../../deploy/go2rtc.yaml) keeps two dimension-gated MJPEG receivers (`z70_front_jpeg`, `z70_gripper_jpeg`) on the configured `z70` producer. Source-dimension selection must precede scaling so stale frames cannot masquerade as the new lens.

Selection/status/dimensions are cross-checks, not exact exposure timestamps or independent physical lens verification. Gripper/front are sequential views of one selected stream, not synchronized stereo. See [shared-camera validation](../../docs/validation/shared-camera-mode.md) and [throughput scope](../../docs/validation/camera-throughput.md).

## Troubleshooting boundaries

Use current [deployment camera triage](../../deploy/go2rtc-login-fix.md), not an incident's old restart recipe. Distinguish fixed-ID-only MQTT denial from both IDs denied; preview rejection from connected-but-zero-RTP; and JPEG decode from acquisition admission. Empty consumers prove cleanup only. Offline patch tests/builds do not prove restored video, and official-app recovery does not prove MCP recovery. Camera diagnostics read shared ROS evidence; they do not initiate a broker probe.

## Historical/static provenance

The offline app/APK findings are bounded to inspected artifacts; their negative searches do not prove firmware/hardware absence. Their original bodies remain intact:

- [Selector/status call graph](../gripper-camera/plugin-selector-audit.md)
- [Preview, photo and debug-stream audit](../gripper-camera/plugin-stream-audit.md)
- [Native APK/DEX audit](../gripper-camera/native-apk-audit.md)
- [Locale extraction](../gripper-camera/locale-evidence.md)
- [Corrected L01/LAN explainer](LOCAL_API_AND_LAN_STREAMING.md)
- [Gripper research entrypoint](../gripper-camera/README.md)

The prior per-request switch/restore implementation and “no gripper calibration” handoff are superseded, not current instructions. No new live verification is asserted by this documentation curation.
