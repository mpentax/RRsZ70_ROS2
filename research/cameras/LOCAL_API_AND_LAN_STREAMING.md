# Z70 L01 and camera LAN transport

This is a source-bounded explainer for Saros Z70 (`roborock.vacuum.a179`, R50) and the inspected clients. It is **not** an exhaustive robot firmware, sensor, port or encoder inventory. No robot/network probe was performed for this documentation update.

## Control framing: TCP 58867

The current implementations are [local.ts](../../adapters/vendor/src/local.ts) (`header`, `prefix`, frame parser) and [robot.ts](../../adapters/vendor/src/robot.ts) (persistent session). Offsets below are **zero-based, end-exclusive within the frame body**, excluding the four-byte TCP length prefix. Multi-byte fields are big-endian.

| Body bytes | Width / type | Field |
|---|---|---|
| `[0:3]` | 3 ASCII bytes | version (`L01`) |
| `[3:7]` | uint32 BE | sequence |
| `[7:11]` | uint32 BE | random / session nonce material |
| `[11:15]` | uint32 BE | timestamp |
| `[15:17]` | uint16 BE | frame type |

The prefix is uint32 BE **body length**. For encrypted PUBLISH, `[17:19]` is uint16 BE encrypted payload length, followed by that payload and a final four-byte CRC32 IEEE over the body excluding the CRC. Header size is 17 bytes; **not every frame includes payload length or CRC**. Current CONNECT uses the header plus a four-byte value (`10`); acknowledgments/control frames have their own forms. Follow executable parsers rather than imposing the PUBLISH shape on every type.

Sequence is a **32-bit wire field**. The persistent sender's 16-bit outgoing wrap policy does not make replies or the wire field 16-bit; `local.ts` also contains a narrower one-shot-probe acceptance check, not a wire-layout definition. The previous table's field4/protocol/seq16 interpretation and timestamp offset 13 were incorrect.

The inspected `SocketFrameType` enum has these eight entries; that is an enum finding, not proof of every possible future firmware type:

| Value | Name | Inspected purpose |
|---:|---|---|
| 0 | CONNECT | local session initiation / connect nonce |
| 1 | CONNACK | session response / acknowledgment nonce |
| 2 | PINGREQ | keepalive request |
| 3 | PINGRESP | keepalive response |
| 4 | PUBLISH | encrypted DPS JSON, request DPS 101 / reply DPS 102 |
| 5 | PUBACK | transport acknowledgment, not physical completion |
| 6 | BLOB | inspected binary-transfer path |
| 7 | MAP | inspected local-map-transfer path |

L01 uses AES-GCM session encryption with local-key derivation and connect/ack nonce context; legacy `1.0` transport is a separate codec. Inspect the implementation for exact encryption/CRC/type handling; do not infer a public unencrypted HTTP/RTSP endpoint from local control support.

## Discovery and bounded client findings

The original APK investigation identified UDP **58866** discovery and TCP **58867** local control (`classes4.dex`: discovery `Lo0O000/OooOOOO.run` at `0x524db8`; frame enum at `0x1bbcc4`; socket handler at `0x52efc4`). WebRTC media uses negotiated UDP candidates, not a universal fixed media port.

The searches did not establish an additional usable RTSP/HTTP/ONVIF live-camera route. They do **not** prove that no such service exists anywhere in firmware. Host-side go2rtc ports/configuration are not robot port inventory. The native machine-code bodies and nested bundles were not exhaustively analyzed; see the full [native audit and exclusions](../gripper-camera/native-apk-audit.md).

## RPC routing is not media routing

The SDK entrypoints encode different preferences (`classes4.dex`):

- `callMethod` (`0x20ef40`): `locally=null`, SDK chooses local socket when locally online, otherwise MQTT.
- `callMethodFromLocal` (`0x20f118`): forces local and fails when the socket is unavailable.
- `callMethodFromCloud` (`0x20f0dc`): forces cloud MQTT.

The inspected native routing chain (`0x21c6cc`, `0x321668`, `0x321870`, `0x321788`) establishes those choices, **not** that every camera wrapper uses the default. In particular the [plugin stream audit](../gripper-camera/plugin-stream-audit.md) identifies an explicit `callMethodFromCloud` wrapper for `send_sdp_to_robot`. The previous claim that all camera signaling becomes local automatically was overbroad.

Local camera-selection/status RPC support in [camera-selection.ts](../../adapters/vendor/src/camera-selection.ts) does not establish cloud-free preview startup. The [pinned go2rtc build and patches](../../deploy/go2rtc.Dockerfile) use cloud MQTT signaling/admission for camera setup. Asynchronous SDP/ICE delivery and media transport must be analyzed separately; neither an L01 acknowledgment nor static local-capable SDK routing proves a fully local live-signaling path.

The inspected photo paths are separate from live video: [photo-protocol.ts](../../adapters/vendor/src/photo-protocol.ts) distinguishes local thumbnail handling from full-photo MQTT binary handling, and [split-map-reader.ts](../../adapters/vendor/src/split-map-reader.ts) distinguishes local request replies from map transfer. Photo `type` values are not live camera indices. Preserve their actual protocol handling rather than equating every binary path to the L01 frame-type enum above.

## Retained session evidence: a host–host media path

The original audit cited retained state snapshots:

- [M3 retry before idle](../../calibration/arm-calibration/20261015T121045Z-m3-rotation-retry/final-state-before-idle.json) and [final state](../../calibration/arm-calibration/20261015T121045Z-m3-rotation-retry/final-state.json)
- [Marker rotation final state](../../calibration/arm-calibration/20261015T104441Z-marker68-rotation/final-state.json)
- [Blocked coverage deployed state](../../calibration/arm-calibration/20261015T120106Z-m3-rotation-coverage/deployed-state.json)

Reported acquisition diagnostics included `localCandidateType:"host"`, `remoteCandidateType:"host"`, `candidateProtocol:"udp"`, `videoTracks:1`, connected ICE/DTLS, and nonzero video packets/bytes. These support a **selected host–host UDP media route in those retained sessions**, not an always-direct or cloud-free guarantee. A successful TURN-server query does not mean that TURN carried media. The historical ~5–7 s cold-start observations are not current latency promises; see [camera throughput scope](../../docs/validation/camera-throughput.md).

The pinned producer startup includes MQTT connection/admission, PIN/preview checks, TURN lookup, SDP/ICE exchange and connection wait. That explains possible setup overhead but does not prove a universal causal timing breakdown. Current [go2rtc configuration](../../deploy/go2rtc.yaml) shares the configured `z70` producer with dimension-gated front/gripper MJPEG receivers. This describes **software topology**, not the number of hardware encoders, sensor pipelines, or peer connections firmware could support.

## Supported views versus physical cameras

The owner reports **three physical cameras (two chassis, one gripper)**. Only front/gripper selectable streams are evidenced in the inspected implementation; `app_change_camera {flag:0|1}` and `camera_status` bit 15 select/describe that shared view. Dimensions are front 1600×1200 and gripper 1600×896. No inspected flag-2 caller or independent rear/side/depth/IR selector was established; that negative finding does not reduce the physical inventory to two.

The R50/a179 feature registration (`#8012`, H:317516–317534; parent #8011) distinguishes `RGBCameraFeatures` / `MechanicalArmFeatures` from another product's `DoubleRGBCameraFeatures`. Feature names are not a physical-camera census. `Remote_Back` (H:316856) names backward driving, not a rear video feed; the audit did not establish the rear sensor's complete electrical/encoder topology. `getTofStatus` / `setTofStatus` map to the debug voxel toggle, not a depth-video selector.

See [current camera summary](FINDINGS.md), [ROS handoff](ROS_HANDOFF.md), and [preserved gripper audits](../gripper-camera/README.md) for supported interfaces, adopted intrinsics and exact source hashes. Shared selection/no implicit restoration is current software behavior; frame-cache filtering never switches lenses. No task-specific future streaming plan or obsolete per-request selector recipe is needed to use the implemented stack.
