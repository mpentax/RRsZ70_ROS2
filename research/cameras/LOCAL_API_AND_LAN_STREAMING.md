# Z70 Local API, LAN Transport & Camera Streaming Research

## Executive Summary

This document records the offline reverse-engineering and repository evidence audit for the Roborock Saros Z70 (`roborock.vacuum.a179`, product family `R50`) local network protocol (`L01`), LAN ports, RPC transport routing, and WebRTC/ICE camera media path.

### Key Conclusions

1. **Two Distinct Layers: Signaling vs. Media Transport**
   - **Media Transport IS Already 100% Direct Local LAN (`host -> host` UDP):**
     Verified live acquisition diagnostics in this repository (`calibration/arm-calibration/20261015T121045Z-m3-rotation-retry/final-state-before-idle.json` and `final-state.json`) prove that once WebRTC SDP/ICE signaling completes, the selected ICE candidate pair is:
     - `localCandidateType: "host"`
     - `remoteCandidateType: "host"`
     - `candidateProtocol: "udp"`
     The H.264 video RTP packets (`videoTracks: 1`, `1600×1200` for `front` and `1600×896` for `gripper`) travel directly over the local Wi-Fi LAN between the Z70's ephemeral WebRTC UDP port and `go2rtc` on the Mac—**not** through a cloud TURN relay.
   - **Signaling & Camera Selection Transport (`MQTT` vs. Local TCP `58867`):**
     The Z70's local LAN socket (`TCP 58867`, protocol `L01`) uses AES-256-GCM encryption (`deriveL01Key(localKey)` + MD5/connectNonce/ackNonce salt) and supports an 8-type wire framing (`SocketFrameType`: `CONNECT=0`, `CONNACK=1`, `PINGREQ=2`, `PINGRESP=3`, `PUBLISH=4`, `PUBACK=5`, `BLOB=6`, `MAP=7`).
     - **`app_change_camera({ flag: 0 | 1 })` and `get_camera_status`:** Already work over the local `L01` TCP `58867` socket (`PUBLISH=4`, inner DPS `101` -> `102`) in `RoboClaw` (`adapters/vendor/src/camera-selection.ts` via `robot.ts`).
     - **WebRTC Signaling (`start_camera_preview`, `send_sdp_to_robot`, `get_device_sdp`, `send_ice_to_robot`, `get_device_ice`):** In both the official Android plugin (`index.android.hasm`) and `go2rtc` (`pkg/roborock/client.go`), these RPCs are sent over `DPS 101` with `callMethod` (`fromLocal = null`), which routes to the local `L01` socket when locally connected (`sendDpsBySocket`), **except** that the robot's asynchronous `dev_sdp` (`{"type":"answer","sdp":"..."}`) and `dev_ice` notifications are delivered as encrypted binary blobs on MQTT protocol `301` (`rr/m/o/<user>/<duid>`), or via `get_device_sdp` / `get_device_ice` RPC polling.

2. **Why There Are No Hidden RTSP/HTTP Ports or Additional Camera Streams on the LAN**
   - **Local Port Inventory:** Across the entire Android APK (`classes.dex`–`classes5.dex`), native libraries (`librrcodec.so`, `libjingle_peerconnection_so.so`, `libffavc.so`, `libreact_codegen_RTNNativeDeviceSpec.so`), and the Z70 React Native plugin (`index.android.hasm`), the **only** local LAN ports used by the robot are:
     - **UDP `58866`:** Local device broadcast discovery (`Lo0O000/OooOOOO;`, `SocketWireFrame$FrameType.SOCK_DISCOVERY`, `L01` SHA-256 / AES-GCM broadcast).
     - **TCP `58867`:** Local `L01` encrypted control/RPC/blob socket (`LocalClientService`, `SocketFrameType` `0..7`).
     - **Ephemeral UDP port (ICE `typ host`):** Negotiated dynamically per WebRTC session via SDP/ICE (`libjingle_peerconnection_so.so`).
   - No RTSP (`554`/`8554`), HTTP/MJPEG, ONVIF, or secondary raw video sockets exist anywhere in the client SDK or native libraries.
   - **Single Hardware Encoder / Single WebRTC Peer / Two Physical RGB Cameras:**
     - Product model config (`#8012`, `H:317516–317534`, `R50` / `a179`) registers `RGBCameraFeatures` + `MechanicalArmFeatures` (front RGB + gripper RGB), **not** `DoubleRGBCameraFeatures` (which is for dual-lens stereo front cameras on `T20`/`Coral`).
     - The robot's `homesec` daemon enforces a single active preview session (`get_homesec_connect_status` -> single `client_id`) and a single WebRTC video track (`videoTracks: 1`).
     - Switching `app_change_camera({ flag: 0 | 1 })` switches the active physical sensor feeding that single H.264 encoder/track in-place (`1600×1200` <-> `1600×896`) without renegotiating WebRTC.

---

## 1. Complete Z70 Local LAN Protocol (`L01`) Architecture

### 1.1 Discovery: UDP Port `58866`
Decompiled from `classes4.dex` (`Lo0O000/OooOOOO.run` at `0x524db8`):
- The robot broadcasts discovery packets on **UDP port `58866`** (`0xe5f2`).
- Frame header starts with the 3-byte ASCII protocol version `"L01"` (`ProtocolVersion.L01`).
- Discovery payload is decrypted using the SHA-256 digest of a fixed salt (`LOooOoO0/o00O0O;`) and parsed as `SocketWireFrame$FrameType.SOCK_DISCOVERY`, yielding the robot's LAN IP (`localAddress`), `duid`, and `localPv = "L01"`.

### 1.2 Control & Binary Transport: TCP Port `58867`
Decompiled from `classes4.dex` (`Lcom/roborock/internal/common/socket/internal/wire/SocketFrameType;` at `0x1bbcc4` and `Lo0O00O/OooOo.OooO00o` at `0x52efc4`) and verified against `adapters/vendor/src/local.ts`:

Every frame on TCP `58867` has a 17-byte outer header followed by an optional encrypted payload and a 4-byte CRC32 (`IEEE`):
- `0..3`: `"L01"` (3 bytes ASCII)
- `3..5`: `field4` (`uint16 BE`, typically `0x0000`)
- `5..7`: `protocol` (`uint16 BE` — `SocketFrameType`)
- `7..9`: `seq` (`uint16 BE`)
- `9..13`: `random` (`uint32 BE`随机数 / salt component)
- `13..17`: `timestamp` (`uint32 BE`, unix seconds)

#### Exhaustive `SocketFrameType` Enum (`classes4.dex` @ `0x1bbcc4`)
There are **only 8 frame types** (`0..7`) in the `L01` wire protocol:

| Value | Enum Name | Wire Handler (`Lo0O00O/OooOo`) | Purpose |
|---:|---|---|---|
| `0` | `CONNECT` | `SocketConnect` (`Lo0O00O/OooO0o`) | 21-byte frame (`connectNonce` in `random` field) initiating the `L01` session. |
| `1` | `CONNACK` | `SocketConnack` | 21-byte response from robot providing `ackNonce` (`connectAckNonce`) required to derive the session AES-256-GCM key. |
| `2` | `PINGREQ` | `SocketPingReq` (`Lo0O00O/OooOOO0`) | 17-byte keepalive request (sent every 10s). |
| `3` | `PINGRESP` | `SocketPingResp` | 17-byte keepalive response. |
| `4` | `PUBLISH` | `SocketPublish` (`Lo0O00O/OooOo00`) | Encrypted DPS JSON envelope (`{"dps":{"101":"{\"id\":...,\"method\":...,\"params\":...}"}}` for requests, `{"dps":{"102":"..."}}` for RPC replies). |
| `5` | `PUBACK` | `SocketPubAck` | 17-byte transport acknowledgment for `PUBLISH`, `BLOB`, or `MAP` frames. |
| `6` | `BLOB` | `SocketBlob` (`Lo0O00O/OooO0O0`) | Encrypted Protobuf (`MessageProto$AppToRobotMsg` / `RobotToAppMsg`) and chunked binary transfer (`BlobTransferAssembler`). |
| `7` | `MAP` | `SocketMap` | Encrypted local map binary transfer (`DpsMapReceivedEventModel`, `source=lan`). |

### 1.3 How `RRPluginSDK` Routes RPC Calls (`callMethod` vs `callMethodFromLocal` vs `callMethodFromCloud`)
Decompiled from `classes4.dex`:
- `PluginSDKModule.callMethod` (`0x20ef40`) passes `locally = null` to `o00OO00O.OooOo` / `OooOoO0` (`0x21f964` / `0x21fa84`).
- `PluginSDKModule.callMethodFromLocal` (`0x20f118`) passes `locally = Boolean.TRUE`.
- `PluginSDKModule.callMethodFromCloud` (`0x20f0dc`) passes `locally = Boolean.FALSE`.
- Inside `oo00o.OooOoo` (`0x21c6cc`):
  - When `locally == null`, it calls `IDevice.publishDps` -> `o000O0O0.sendDps` (`0x321668`).
  - `o000O0O0.sendDps` checks `DeviceBean.isLocalOnline()`:
    - If **`true`**, it logs `"sendDps by sendDpsBySocket"` and sends the RPC over **local TCP `58867`** (`SocketFrameType.PUBLISH = 4`).
    - If **`false`**, it logs `"sendDps by publishDpsByMqtt"` and sends the RPC over **cloud MQTT**.
  - When `locally == Boolean.TRUE` (`callMethodFromLocal`), it calls `IDevice.sendDpsLocal` -> `o000O0O0.sendDpsBySocket` (`0x321870`), which **fails closed** with `"socket.err.device.offline"` if the local TCP `58867` connection is not active.
  - When `locally == Boolean.FALSE` (`callMethodFromCloud`), it calls `IDevice.publishDpsMqtt` -> `o000O0O0.sendDpsByMqtt` (`0x321788`), forcing cloud MQTT.

In the Z70 React Native plugin (`index.android.hasm`):
- `callMethodFromLocal` (`#4923`, `H:215847`) is only called by **4 methods**, all in `R50` (`a179`) photo/obstacle image helpers (`H:218762–218995`):
  1. `getR50ImageDes` (`#5031`) -> `get_image_des`
  2. `getR50ImageTranslation` (`#5035`) -> `get_image_translation`
  3. `getR50UploadFileSignUrl` (`#5039`) -> `get_upload_file_sign_url`
  4. `getR50UploadFileSignUrlV2` (`#5043`) -> `get_upload_file_sign_url_v2`
- All other RPCs—including `app_change_camera`, `get_camera_status`, `start_camera_preview`, `stop_camera_preview`, `send_sdp_to_robot`, `get_device_sdp`, `send_ice_to_robot`, `get_device_ice`, and `get_photo`—use the default `callMethod` (`locally = null`), which automatically uses local TCP `58867` whenever the phone is on the same LAN.

---

## 2. How Camera Streaming & Signaling Actually Work on the LAN

### 2.1 Live Evidence: Media Is Already Direct LAN UDP (`host -> host`)
In `deploy/patch-go2rtc-diagnostics.go`, `go2rtc` was instrumented to record the selected Pion WebRTC `ICECandidatePair` (`LocalCandidateType`, `RemoteCandidateType`, `CandidateProtocol`) without logging IP addresses or secrets.

Inspection of the saved live hardware state snapshots in `calibration/arm-calibration/20261015T121045Z-m3-rotation-retry/final-state-before-idle.json` (and `20261015T104441Z-marker68-rotation/final-state.json`, `20261015T120106Z-m3-rotation-coverage/deployed-state.json`) shows:
```json
"acquisition": {
  "available": true,
  "vendorCode": 0,
  "attempt": 224,
  "elapsedMs": 6708,
  "videoTracks": 1,
  "videoPackets": 386,
  "videoBytes": 443366,
  "stage": "connected",
  "failure": "none",
  "vendorReason": "none",
  "mqtt": "ok",
  "pin": "ok",
  "turn": "ok",
  "pcState": "connected",
  "iceState": "connected",
  "dtlsState": "connected",
  "localCandidateType": "host",
  "remoteCandidateType": "host",
  "candidateProtocol": "udp",
  "readerExit": "none"
}
```
**What this proves:**
1. When `go2rtc` sends its SDP offer (`send_sdp_to_robot`) and ICE candidates (`send_ice_to_robot`), it includes its local LAN `typ host` UDP candidate, and the Z70 replies in `dev_sdp` / `dev_ice` with its own LAN `typ host` UDP candidate.
2. Pion WebRTC and the robot's `libjingle` peer connection establish DTLS/SRTP **directly over local LAN UDP** (`localCandidateType: "host"`, `remoteCandidateType: "host"`, `candidateProtocol: "udp"`).
3. Even though `get_turn_server` is queried during setup (`turn: "ok"`), **no video traffic is relayed through the cloud TURN server** when the Mac and robot are on the same LAN.

### 2.2 Why Cold Start Takes ~5–7 Seconds (`go2rtc` Signaling Path)
In `go2rtc` (`pkg/roborock/client.go` + `pkg/roborock/iot/client.go`), opening the `z70` producer (`roborock://...`) performs the following sequence on every cold start:
1. Opens a fresh TCP+TLS connection to the cloud MQTT broker and authenticates (`mqtt.Connect` + `mqtt.Subscribe("rr/m/o/...")`).
2. Sends `check_homesec_password` (PIN check).
3. Sends `get_homesec_connect_status` and, if `status != 0`, `stop_camera_preview`.
4. Sends `start_camera_preview`.
5. Queries `get_turn_server`.
6. Creates a Pion `PeerConnection`, gathers local ICE candidates (`GatheringCompletePromise`), and sends `send_sdp_to_robot` (with `{"type":"offer","sdp":"..."}`).
7. Polls `get_device_sdp` in a loop (100 ms interval, up to 5 s) waiting for the robot's `dev_sdp` answer blob on MQTT protocol `301`, then exchanges ICE candidates (`send_ice_to_robot` / `get_device_ice`) and waits on a 2-second ticker (`wait_connected`) until `pc.ConnectionState() == Connected`.

### 2.3 Split-Transport Behavior on `L01` (Why Signaling Uses MQTT `301`)
Just as documented in `adapters/vendor/src/photo-protocol.ts` and `split-map-reader.ts` for `get_map_v1` and `get_photo`:
- The Z70 accepts RPC requests (`101`) over local TCP `58867` (`L01`) and returns the JSON RPC acknowledgment (`102`, e.g. `["ok"]`) locally.
- However, asynchronous binary protocol `301` payloads pushed by the robot (`dev_sdp`, `dev_ice`, `get_map_v1` blobs, and `get_photo` `type: 0` full-image blobs) are published by the robot to its MQTT output topic (`rr/m/o/<user>/<duid>`) encrypted with `devKey` (protocol `1.0` AES-128-ECB) rather than sent back as `L01` `PUBLISH` frames on TCP `58867` (whereas `get_photo` `type: 1` thumbnails return on local `L01` protocol `300`, as implemented in `adapters/vendor/src/photo-protocol.ts`).
- Therefore, keeping the `go2rtc` WebRTC producer warm (or sharing a single warm `z70` WebRTC producer across multiple downstream consumers, as planned in Kanban task `t_7b37b8e7`) avoids repeating the 5–7 second MQTT signaling handshake while keeping all video media on direct local LAN UDP.

---

## 3. Exhaustive Audit for Additional Cameras (Reverse/Rear, Side, Depth/TOF)

### 3.1 Product Feature Configuration (`R50` / `a179`)
In `index.android.hasm` (`#8012`, `H:317516–317534`), the Saros Z70 (`shortModels: ['a157', 'a179']`, model code `R50`) inherits from `CoralProSAw` (`#8011`, `H:317494–317513`):
- **Enabled camera/sensor features:**
  - `RGBCameraFeatures` -> `[Camera_RGB, Obstacle_Picture, Preview_Pin, Video_Call, Cruise]` (single front RGB camera pipeline, not `DoubleRGBCameraFeatures`).
  - `MechanicalArmFeatures` -> `[Mechanical_Arm, Mechanical_Arm_Guide]` (adds the gripper/pincher RGB camera and pickup controls).
- **What `Remote_Back` (`mf.remote_back`, `H:316856`) Actually Is:**
  - String literal at `H:316856`: `"遥控--后退能力"` ("Remote control — reverse driving capability").
  - Used by the manual remote-control joystick UI to allow driving the chassis backward; it is **not** a rear/reverse camera feature flag.
  - The Z70's rear sensor is a navigation/docking sensor, not a video-streaming camera connected to the `homesec` H.264 encoder.

### 3.2 Native & Plugin Camera RPC Inventory
Across the entire plugin bundle (`index.android.hasm` `H:224500–232700`), `RRVideoModule` (`classes4.dex` `0x2486b0`), `librrhomesec/RtcClient v2.21` (`classes4.dex` `0x1a3db4`), and `librrcodec.so`:
1. **Live Video Camera Selection (`app_change_camera`):**
   - Takes `{ flag: 0 | 1 }` (`0` = `front` `1600×1200`, `1` = `gripper` `1600×896`).
   - Confirmed via `get_camera_status` bit 15 (`0x8000`, `mechanicalCameraEnabled`).
   - There are no other camera selector RPCs, channel indices, or WebRTC video tracks (`videoTracks` is always `1`).
2. **Obstacle / AI Snapshot Photos (`get_photo` + `get_image_des`):**
   - `get_photo` (`data_filter: { img_id, type }`) fetches stored obstacle/pet snapshots by `img_id`.
   - As verified in `adapters/vendor/src/photo-protocol.ts`, `type: 1` (thumbnail) is delivered locally over TCP `58867` on `L01` protocol `300`, while `type: 0` (full image) is delivered on MQTT protocol `301`. Neither accepts a live camera index.
3. **Debug Point-Cloud / TOF (`get_debug_voxel_map` / `set_debug_voxel_map`):**
   - Bound to `getTofStatus` / `setTofStatus` (`H:231503–231514`, `H:232360–232373`), used only for the internal debug voxel/TOF toggle and WebView point-cloud viewer, not an RGB/video stream.

---

## 4. Architectural Implications for Task 2 (`t_7b37b8e7`)

Because:
1. The Z70 hardware/firmware only exposes **one active WebRTC preview producer** (`z70`) at a time (over direct LAN UDP `host -> host`),
2. Switching `app_change_camera({ flag: 0 | 1 })` over local TCP `58867` switches the resolution of that single H.264 RTP stream in-place between `1600×1200` (`front`) and `1600×896` (`gripper`),

The optimal local streaming architecture (tracked in Kanban task **`t_7b37b8e7`**) is:
- Keep the single `z70` WebRTC producer (`roborock://...`) connected over direct LAN UDP without resetting it on camera switch.
- Run two dedicated downstream stream/transcoder consumers in `go2rtc` / the vendor adapter (or resolution-aware dual streams for `front` `1600×1200` and `gripper` `1600×896`) so switching cameras via local `L01` `app_change_camera` does not tear down or reset the producer/transcoder pipeline.
