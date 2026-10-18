# Camera handoff to ROS and thin clients

## Ownership and interfaces

[`z70_driver`](../../src/z70_driver/z70_driver/driver.py) is the shared acquisition/selection owner. The private vendor sidecar supplies transport; [`z70_web_gateway`](../../src/z70_web_gateway/z70_web_gateway/gateway.py) and [MCP](../../apps/mcp/src/mcp-server.ts) expose ROS state/cache and services, not independent preview sessions.

| Operation | Current interface | Meaning |
|---|---|---|
| Select | `/z70/camera/select` (`z70_interfaces/srv/SelectCamera`) | Shared persistent choice; enum 1 front / 2 gripper in this ROS service, **not vendor camera flags**. |
| Web/MCP selection | `POST /api/ros/camera/select`, MCP `select_camera` | Body `{camera:"front"}` or `{camera:"gripper"}`; forwards to ROS. |
| Frame read | `POST /api/ros/frame`, MCP `get_camera_frame` | Optional camera filters paired cache only; never selects or opens preview. Maximum accepted cache age is five seconds. |
| Diagnostics | MCP `get_camera_diagnostics` | Reads shared ROS evidence; no broker/session probe. |

The driver privately reads `/api/camera/frame` with `{}` and selects via `/api/driver/camera/select`. Do not direct clients to those routes or the retired `/api/camera/select`. Selection is never implicitly restored by a frame reader. The selector does not enter pickup mode or move the arm. [Current selection constraints and evidence](FINDINGS.md) describe the observed pickup-mode/arm-out prerequisite and expected reply timeout; confirmed selection is distinct from the command's UNKNOWN/TIMEOUT outcome.

## Published views

For each `front` and `gripper` prefix:

| Topic suffix | Type / behavior |
|---|---|
| `/image_raw/compressed` | `sensor_msgs/CompressedImage`, reliable transient-local latest JPEG |
| `/image_raw` | `sensor_msgs/Image`, decoded `bgr8`, depth 2 (not latched) |
| `/camera_info` | `sensor_msgs/CameraInfo`, reliable transient-local, only when installed calibration dimensions match the frame |
| `/metadata` | `z70_interfaces/FrameMetadata`, reliable transient-local paired evidence/identity and calibration hash |

The full prefixes are `/camera/front` and `/camera/gripper`; frames are `camera_optical_frame` and `gripper_camera_optical_frame`. Front is 1600×1200, gripper 1600×896. The target acquisition rate is 3 Hz, not a delivered-throughput promise. After switching, a previous lens's latched JPEG may remain visible; inspect metadata/validity and advancing stamps, not just pixels on a panel. Receipt/ROS publication time is not camera exposure time.

Both lenses have installed calibration defaults under `z70_description/config`: [front CameraInfo](../../src/z70_description/config/z70_front_camera_info.yaml) and [adopted asymmetric gripper CameraInfo](../../src/z70_description/config/z70_gripper_camera_info.yaml). The driver parameters `camera_info` and `gripper_camera_info` can override them. Mismatched/missing calibration produces `CALIBRATION_MISMATCH` evidence rather than invented CameraInfo. Use the linked exact K/D; do not force fx=fy or substitute the old initialization.

[Current arm documentation](../../docs/z70-arm-model.md) owns the CAD camera/marker TF, optical registration and collision/TCP distinctions. Published TF and an adopted lens do not imply encoder feedback, exact pad calibration or precision manipulation qualification.

## Supported views versus physical sensors

The owner inventory is three physical cameras (two chassis, one gripper). Only front/gripper selectable streams are evidenced. Do not add left/right/back/depth/IR placeholder feeds or infer vendor flag 2 from ROS enum 2 or the physical count. Front and gripper are time-multiplexed, not a synchronous stereo pair.

See [current camera summary](FINDINGS.md), [shared-mode validation](../../docs/validation/shared-camera-mode.md), [Foxglove layout guide](../../src/z70_bringup/foxglove/README.md), and the preserved [static audits](../gripper-camera/README.md). No live launch/capture is authorized by this handoff.
