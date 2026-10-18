> **Supporting historical evidence — 2026-10-18 curation.** The original audit/replay body below is retained verbatim. Static negative findings and implementation statements are bounded to the inspected sources; replay is not a second physical acquisition. Current behavior: [camera guide](../cameras/FINDINGS.md) and [arm model](../../docs/z70-arm-model.md).

# Offline APK audit: gripper-camera native leads

## Outcome

**No gripper-specific camera selector, activation command, camera-count declaration, or physical camera/channel mapping was recovered from the APK's DEX/native code.** A concrete React Native → Roborock home-security client → WebRTC receive path was recovered. Its application-level observer stores an incoming remote video track in **one field**, without testing track ID, stream ID, MID, camera index, or camera name. That is a useful transport lead, not evidence that the robot has only one camera or that only one camera can be streamed.

User-provided hardware context remains authoritative: the Saros Z70 has **three physical cameras: two chassis cameras and one gripper camera**. This audit neither confirms nor contradicts that count. The previously identified JS `app_change_camera` values `0`/`1` remain **physically unverified**; this APK audit provides no justification to label either value “gripper,” invent value `2`, or equate a binary app selector with the number of physical sensors.

## Provenance and inspection boundary

- APK: `/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s/Z70Android/Roborock-4.74.08-100936-arm64-v8a.apk`
- SHA-256: `13dbd693115ab925af5dc6b3b4b3d100d777f6173952a3d9185eb533a5c07ee6`
- ZIP inventory: **2,312 entries**, **90 `.so` libraries**, **5 DEX files**, **36,727 DEX class definitions**. Counts were aggregated programmatically.
- All top-level DEX string pools and defined method names were scanned. All top-level `.so` files were scanned for printable ASCII camera/video/channel/track/arm-related strings; ELF dynamic symbols were inspected. Selected DEX methods were disassembled, not executed.
- Seven embedded plugin ZIPs were inventoried, totaling 641 member entries. Their bundles were **not decompiled** in this native audit.
- No robot, Docker, service, account, or app execution. No `.env`, `.local`, token files, credential stores, or user-data files were read. Authentication material was not extracted; codec/authentication references below are class/method symbols only.
- Temporary tooling and extraction/index artifacts were created **only** under `/Users/mpentax/.hermes/cache/scratch/gripper-camera-native/`. This report is the sole intended repository write; existing dirty files were left untouched.

### DEX inventory

| APK entry | Uncompressed bytes | Defined classes |
|---|---:|---:|
| `classes.dex` | 10,677,480 | 13,706 |
| `classes2.dex` | 284,556 | 155 |
| `classes3.dex` | 8,070,408 | 8,416 |
| `classes4.dex` | 10,947,572 | 10,407 |
| `classes5.dex` | 3,612,736 | 4,043 |

All DEX locations below are **`code_item` offsets within the named uncompressed DEX entry**. Instruction offsets, when given, are byte offsets relative to that method's instruction stream. They are not ZIP offsets. Native string offsets are file offsets within the named uncompressed ELF entry; symbol values are ELF virtual addresses.

## 1. Actual native bridge: `RRVideoViewManager`

Entry: `classes4.dex`.

Class: `Lcom/roborock/smart/react/module/RRVideoModule;`.

`getName()Ljava/lang/String;`, `code_off=0x2485a4`, returns the literal **`RRVideoViewManager`**. The Java class name is therefore not the JS module name.

DEX annotations were parsed directly at the class annotation-directory offset `0xa216c0`. The following video-relevant methods are annotated `Lcom/facebook/react/bridge/ReactMethod;`:

- `startCameraPreview(Lcom/facebook/react/bridge/Callback;)V`
- `stopCameraPreview()V`
- `sendTurnServerInfoToSDK(Lcom/facebook/react/bridge/Dynamic; Lcom/facebook/react/bridge/Callback;)V`
- `sendSdpInfoToSDK(Lcom/facebook/react/bridge/Dynamic; Lcom/facebook/react/bridge/Callback;)V`
- `sendIceInfoToSDK(Lcom/facebook/react/bridge/Dynamic; Lcom/facebook/react/bridge/Callback;)V`
- `captureCurrentFrame(Lcom/facebook/react/bridge/Callback;)V`
- `startRecordVideo(Lcom/facebook/react/bridge/Callback;)V`
- `stopRecordVideo(Lcom/facebook/react/bridge/Callback;)V`
- `enableAVCall(Z)V`, `enableMicrophone(Z)V`
- `setMaxFramerate(I)V`
- `getCurrentSDKStatusInfo(Lcom/facebook/react/bridge/Callback;)V`

The remaining exposed methods concern permissions and full-screen UI. **No camera-ID/index/channel argument or camera-switch method appears in this module's defined API.** That is a statement about this bridge, not all robot capabilities.

### Preview activation is client/session activation, not demonstrated gripper activation

Verified chain:

```text
RRVideoModule.startCameraPreview(Callback)        code_off=0x249888
  → UI-thread runnable
  → lambda$startCameraPreview$2(Callback)          code_off=0x2491c8
  → startPreview()Z                              code_off=0x248300
  → homesec.Oooo000.OooO0o(RRVideoModule)Z         code_off=0x1a36bc
  → PeerConnectionFactory.createPeerConnection(...)
  → PeerConnection.createOffer(...)
```

`startPreview` requires `mICEServer`, creates/reuses the home-security client, supplies a singleton ICE-server list, and starts the call. It does not contain a robot RPC name, arm state, or physical camera selector.

The builder's integer could superficially look like a channel ID. It is instead demonstrably a frame-rate setting: `RRVideoModule.<init>` (`0x2486b0`, instructions `0x004a–0x004e`) sets `homesec.OooOOOO.OooO0Oo` to **15**; `setMaxFramerate(I)` (`0x249804`) writes the same field, and the client constructor stores the passed integer in `OooOO0O`. **Do not interpret this integer as a camera ID.**

`enableAVCall(Z)` (`0x248b74`) writes the builder's audio boolean `OooO0OO`. Its name is not proof of an alternative camera or arm-video mode.

## 2. WebRTC transport and track identity behavior

Actual obfuscated client: `Lcom/roborock/homesec/Oooo000;`, in `classes4.dex`. Strings in `startPreview` identify **`librrhomesec/RtcClient version: 2.21`**. No APK entry named `librrhomesec.so` exists in the native-library inventory: this string is an SDK label, not proof of a separate ELF transport implementation.

### Negotiation

In `Oooo000.OooO0o(RRVideoModule)Z`, `code_off=0x1a36bc`:

```text
0x0200–0x0204: RTCConfiguration.sdpSemantics = UNIFIED_PLAN
0x0272:       "OfferToReceiveVideo"
0x02ac:       "OfferToReceiveAudio"
0x02de:       "DtlsSrtpKeyAgreement"
0x02f6:       "RtpDataChannels"
0x0314:       PeerConnection.createOffer(...)
```

Video receive is enabled by the client's boolean `Oooo0o0`; the constructor (`0x1a3db4`, instructions `0x0124` onward) initializes it to true. There is no physical camera-count constant in this receive-enable boolean.

`Lcom/roborock/homesec/OooO0o;->onCreateSuccess(SessionDescription)V`, `code_off=0x1a242c`, emits:

```text
JSON keys: "sdp", "type"
"type" value: "offer"
UTF-8 JSON → Base64.encodeToString(..., 2) → onSdpCreated(String)
```

`RRVideoModule.setRemoteSdp(Dynamic)Z`, `code_off=0x248030`, Base64-decodes the input, parses the JSON `sdp`/`type`, and applies an **answer** through `PeerConnection.setRemoteDescription`. It explicitly rejects a remote offer with the string `Please create an answer rather than an offer on the remote peer`.

`sendSdpInfoToSDK` (`0x249764`) delegates to `setRemoteSdp`; `sendIceInfoToSDK` (`0x2496e8`) iterates an array and calls `addRemoteICE`; `sendTurnServerInfoToSDK` (`0x2497a4`) delegates to `setICEServer`.

**Lead:** any later, separately authorized comparison of camera modes should distinguish a camera switch within the same RTP track from replacement tracks or a renegotiated SDP. This APK supplies the negotiation shape but supplies no captured SDP, negotiated SSRC/MID/MSID, or robot camera identity.

### Incoming track handling: one active field, no identity selection

Observer: `Lcom/roborock/homesec/OooOO0O;`.

`onAddTrack(Lorg/webrtc/RtpReceiver; [Lorg/webrtc/MediaStream;)V`, `code_off=0x1a292c`:

```text
0x000c: receiver.track()
0x0014: instance-of ... org.webrtc.VideoTrack
0x0020: literal "onAddVideoTrack"
0x0032: client.OooOO0 = received VideoTrack
0x0038: track.setEnabled(true)
0x006c: track.addSink(client.OoooO00)
```

The `MediaStream[]` parameter is unused in this body. There is **no track.id(), stream.id(), MID lookup, camera name/index comparison, or indexed track selection**. A subsequent video-track callback can overwrite the same field. The code does not show removal of the prior sink in this callback, so “one stored active field” is more precise than “only one possible video stream.”

`onAddStream(MediaStream)V`, `code_off=0x1a28e8`, only logs `onAddStream: ` plus `stream.videoTracks.size()`. This is a **runtime media-track count**, not a physical sensor count.

`onDataChannel(DataChannel)V`, `code_off=0x1a2ab8`, consists solely of `return-void`. No mechanical-arm command channel was recovered from this observer. This does not rule out data-channel code elsewhere or non-WebRTC robot control.

`RRVideoModule.captureCurrentFrame(Callback)V`, `code_off=0x2489fc`, eventually calls `PeerConnection.CaptureRemoteVideoFrame(CaptureFrameCallback)` at instruction `0x00ae`. This call has **no camera/channel selector**. The native implementation's internal track choice was not machine-code reversed.

The constructor contains `ARDAMSa0` at instruction `0x02e4`, immediately passed to `createAudioTrack`; it is a **local audio-track ID**, not a gripper/video-camera identity.

## 3. Native libraries: concrete symbols, no gripper mapping

### `lib/arm64-v8a/libjingle_peerconnection_so.so` — 5,576,672 bytes

This is the identifiable WebRTC JNI layer. Examples:

| Evidence | Location |
|---|---|
| `Java_org_webrtc_PeerConnection_nativeCaptureRemoteVideoFrame` | printable string file offset `0x6412`; dynamic symbol VA `0x3cb424`, size 204 |
| `Java_org_webrtc_PeerConnection_nativeGetTransceivers` | string offset `0x6145` |
| `Java_org_webrtc_RtpTransceiver_nativeGetMid` | string offset `0x6d4c` |
| `Java_org_webrtc_VideoTrack_nativeAddSink` | string offset `0x52b8`; dynamic symbol VA `0x2a6e30`, size 72 |
| `H264` | string offset `0xd9601` |
| ` receiver for track_id=` | string offset `0x87ec1` |
| `GetTransceivers is only supported with Unified Plan SdpSemantics.` | string offset `0x917af` |

`classes5.dex` also defines `org.webrtc.PeerConnection.getTransceivers()Ljava/util/List;` at `code_off=0x1d6208` and `addTransceiver(...)` overloads. These are **generic WebRTC capabilities**, not evidence of a robot multichannel API or an application path using three camera tracks. The selected home-security client does not identify a gripper track through them.

### `lib/arm64-v8a/libffavc.so` — 1,161,456 bytes

- Export `Java_org_ffavc_DecoderFactory_GetHandle`: VA `0x56548`, size 4.
- Export `_ZN5ffavc14DecoderFactory9GetHandleEv`: VA `0x564a8`, size 88.
- Strings: `H264 Decoder` at `0x25a82`; `libavcodec/h264dec.c` at `0x1d49d`; `h264` at `0x26cb3`; `hevc` at `0x25ee4`.

These are decoder/library clues. **Codec-name tables, FFmpeg source-path strings, and the presence of `hevc` do not prove HEVC support is reachable, negotiated on the Z70, or associated with a particular camera.** No camera/channel selector was identified here.

### `lib/arm64-v8a/librrcodec.so` — 812,840 bytes

- String `com/roborock/internal/RRCodecApi` at file offset `0x2643a`.
- Export `JNI_OnLoad`: VA `0x49060`, size 284.
- `classes4.dex` defines `Lcom/roborock/internal/RRCodecApi;` with native methods `apiCodec`, `apiCodec2`, `apiCreateMallKey`, `codec`, `codec2`, `codec3`, `codecB1`, and `init`.
- DEX call sites include user/account repositories, an HTTP interceptor, and generic byte-packet classes. No selected RTC receive path calls this API.

**Do not mistake “codec” for a camera-video codec.** These references point to byte/API/packet coding and cryptographic functions; no camera identity was recovered. `armv8_*` symbols and `OPENSSL_armcap` denote CPU architecture/crypto acceleration, not the robot's mechanical arm. No authentication values or packet encryption material were retained.

### `lib/arm64-v8a/libreact_codegen_RTNNativeDeviceSpec.so` — 475,280 bytes

Generated bridge/UI symbols include:

- `_ZNK8facebook5react23RRVideoViewEventEmitter13onReceivedSdpENS1_13OnReceivedSdpE` — string offset `0xa5e9`.
- Corresponding events `onReceivedCandidate`, `onReceivedState`, `onFirstFrameRendered`, `onFrameStuck`, and `onVolumeChanged`.
- `RRVideoView` — string offset `0x2499c`.
- `openCameraScan` — string offset `0x22424`.

This corroborates a video-view/signaling bridge, **not a hidden mechanical-arm camera control**. `openCameraScan` is a scan/UI name; no robot camera-selector interpretation was established.

## 4. Generic robot command bridge, not a discovered arm-video command

`classes4.dex` defines:

- `Lcom/roborock/smart/fbreact/NativeRROmniDeviceTurboModuleSpec;`
- `Lcom/roborock/smart/refactor/ui/plugins/modules/omni/device/RROmniDeviceTurboModule;`

Both expose generic `callMethodWithMap`, `callMethodWithArray`, `callMethodMapWithStrategy`, `callMethodArrayWithStrategy`, `callMethodPb`, and `callMethodPbWithStrategy` APIs. For example:

```text
callMethodWithMap(
  String, String, String, ReadableMap, Callback
)V
```

The concrete module additionally defines `blueCallMethodInner`, `blueCallMethodDpsInner`, and `callMethodPbInner`.

**Lead:** a JS-defined camera/arm RPC can travel through a generic native RPC bridge without its command name appearing in DEX. Therefore the native string absence below does not disprove `app_change_camera` or an arm-program RPC. Parameter names/semantics and the complete MQTT/LAN/Bluetooth routing chain were not established here; do not invent a payload from these signatures.

## 5. False positives explicitly resolved

| Hit | Verified context | What it does NOT prove |
|---|---|---|
| `selectVideoTrack` | `classes4.dex`, `Lcom/shopify/reactnative/skia/RNSkVideo;->selectVideoTrack(MediaExtractor)I`, `code_off=0x39d564`: iterates local extractor tracks and returns the first whose MIME starts with `video/` | A robot camera/remote WebRTC track selector |
| `getAvailableCameraCount` | `classes.dex`, `Landroidx/camera/core/impl/CameraValidator$CameraIdListIncorrectException;`, `code_off=0x2385e8` | The number of cameras on the robot |
| `getCameraIdList`, `cameraId` | Android camera/projector classes and CameraX/Android SDK references | Z70 physical camera IDs |
| OpenCV `CAMERA_ID_FRONT`, `CAMERA_ID_BACK`, `setCameraIndex` | `classes5.dex`, `org.opencv.android` phone-camera capture layer | Chassis versus gripper sensor mapping |
| WebRTC `switchCamera` | `classes5.dex`, `org.webrtc.CameraCapturer`, `Camera1Capturer`, `Camera2Capturer` | Remote robot camera switching; these are Android capture abstractions |
| `EC_InitVideoTrack` | `classes4.dex`, `RtcClient$ErrorCode.<clinit>`, `code_off=0x1a2f54`, instruction `0x009a` | A camera-ID enum or a gripper activation mode |
| `VideoChannel`, RTP `track_id`, transceiver `MID` strings | Generic WebRTC library diagnostics and API | Named physical robot-camera channels |
| FFmpeg `%d channels` / channel-layout strings | Audio/codec library vocabulary | A video-camera count |

## 6. Negative findings and exact coverage

The following case-insensitive substrings occur **zero times in all five DEX string pools** and **zero times in raw bytes of all 90 native libraries**:

```text
gripper
mechanical
app_change_camera
robot_arm
arm_camera
camera_count
camera_num
camera_channel
video_channel
app_start_program
```

This was verified programmatically, separately for every entry. It is **not** a comprehensive semantic absence proof: obfuscated method names, computed commands, integer/protobuf fields, non-ASCII terminology, robot firmware, downloadable plugins, and un-reversed machine code can encode these capabilities without these strings.

No verified mapping was found for:

- camera IDs `0`, `1`, or any additional index;
- physical camera count or a physical sensor-to-stream/channel table;
- a gripper-specific SDP track/MSID/MID/SSRC;
- an arm-video activation prerequisite or deployment-dependent video state;
- a gripper-specific RTSP URL/path;
- a control message enabling a third feed.

### Embedded plugin boundary

The APK includes the nested entry:

```text
assets/plugin/home_card/robot.vacuum.cleaner/robot.vacuum.cleaner.zip
  → index.android.bundle  (1,156,616 bytes)
```

Its 99 members were inventoried. This is a home-card plugin archive, not automatically the same artifact as the adjacent targeted robot-plugin disassembly `/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s/Z70Android/targeted-plugin/index.android.hasm`. The latter was **not re-audited** here. Other nested bundles and assets were not exhaustively searched or decompiled. No claim is made that all bundled or remotely delivered JS has been covered.

## 7. Highest-value concrete follow-up leads

1. **Correlate native signaling with the targeted JS camera/arm feature.** The proven client module name is `RRVideoViewManager`; preview and remote SDP are distinct APIs. Establish whether `app_change_camera` is sent before/after preview, whether it leaves the peer connection intact, and whether any arm-programming UI uses a different native view/module. This can first be investigated offline in the adjacent plugin disassembly.
2. **Treat received video as remotely selected until proven otherwise.** The application observer enables whatever `VideoTrack` arrives and does not select by identity. A robot-side switch behind a stable track is consistent with this code, but not proven.
3. **Preserve track identity if later authorized to instrument.** The existing observer does not expose IDs. A diagnostic should separately label receiver track ID, stream IDs, transceiver MID, negotiated codec, and frame provenance. Do not call a newly observed track “gripper” without physical corroboration.
4. **Do not prioritize `RRCodecApi` or `RNSkVideo.selectVideoTrack` as camera-selector leads.** Their contexts were resolved and are not the robot receive selector.
5. **If native machine-code analysis continues**, start from `libjingle_peerconnection_so.so`'s `nativeCaptureRemoteVideoFrame` (VA `0x3cb424`, size 204) to determine snapshot track choice, and inspect the native receive/transceiver implementation. This audit did not reverse those function bodies, and their generic presence is not evidence of a third channel.

No live calls, speculative camera-ID probes, or arm motions are recommended solely on this evidence.

## Tooling, artifacts, and limits

Available host tools: Python `zipfile`, `/usr/bin/strings`, `javap`, and `objdump`; `jadx`, `apktool`, `dexdump`, and `llvm-objdump` were not found. `java`/`javap` paths exist, but no JVM-based decompiler was run and a usable JVM was not verified.

Installed **Androguard 4.1.4** and **pyelftools 0.32** into a scratch-only virtual environment. Androguard decoded DEX strings/classes/instructions. pyelftools read ELF symbol tables. Printable native strings were located using Python byte regexes so their file offsets are reproducible. No native ELF or APK code was executed. No full DEX cross-reference graph or Java source decompilation was produced. Obfuscation limits semantic naming; ARM64 machine-code bodies, resources.arsc, Android binary XML, and nested JS bundles were not exhaustively analyzed.

Scratch artifacts (ephemeral; Hermes scratch may be pruned):

- `inventory.json`: complete ZIP names/sizes/CRCs and APK SHA-256.
- `summary.json`, `classes*.dex.index.json`: DEX class counts and filtered string/class indexes.
- `classes4.dex.targeted-disassembly.txt`: selected home-security/video methods with code offsets; 179 retained methods.
- `classes4.dex.extra-disassembly.txt`: supplemental codec and resolved Skia selector disassembly.
- `rrvideo-annotations.json`: method annotations proving the React bridge exposure.
- `camera-selector-methods.json`, `device-bridge-methods.json`, `rrcodec-callers.json`: method-symbol indexes and symbolic call sites.
- `native-expanded.json`, `native-libraries.json`: filtered ELF string offsets and symbols.
- `dex-negative-terms.json`, `native-negative-terms.json`: per-entry negative-check counts.
- `nested-zip-inventory.json`: nested archive member inventories.
- Reproduction scripts: `audit.py`, `disassemble.py`, `expand.py`, `verify_scope.py`; environment `venv/`.

Reproduce from `/Users/mpentax` with the scratch virtual-environment Python, e.g.:

```sh
/Users/mpentax/.hermes/cache/scratch/gripper-camera-native/venv/bin/python \
  /Users/mpentax/.hermes/cache/scratch/gripper-camera-native/audit.py
```

Then run `disassemble.py`, `expand.py`, and `verify_scope.py` with the same interpreter. These scripts read only the named APK and create inspection artifacts in that scratch directory. A first annotation-discovery attempt used a nonexistent Androguard convenience API; it was replaced with direct DEX annotation parsing and the complete scope verification reran successfully. No unresolved extraction/decode error remains in the executed scans.
