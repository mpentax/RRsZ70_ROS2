# Z70 camera access — evidence, not a guessed channel map

Scope: Saros Z70 (`roborock.vacuum.a179`), Android 4.74.08. Offline investigation precedes any live selector experiment. No motion, arm deployment, pickup entry/exit, cleaning, docking or sensor-debug writes are authorized in this investigation.

## Camera inventory / access matrix

Physical locations named by Michael are preserved below; a physical aperture/sensor does **not** imply a remotely addressable video stream. Earlier reports mention two chassis cameras plus one gripper camera; that is historical context, **not** an exhaustive hardware inventory.

| Camera / modality | App selection | Preconditions | Exposure / verification |
|---|---|---|---|
| Front RGB / ordinary monitor | `app_change_camera {flag:0}` = non-mechanical selection; ordinary preview | Enabled camera, monitor permission, existing pattern / broker admission | Existing go2rtc `z70` -> `z70_jpeg`; live result recorded below |
| Left side | No selector or camera ID recovered | Unknown | Not exposed; do not invent a flag |
| Right side | No selector or camera ID recovered | Unknown | Not exposed; do not invent a flag |
| Back | No selector or camera ID recovered | Unknown | Not exposed; do not invent a flag |
| Pincher / gripper | `app_change_camera {flag:1}` = mechanical selection | App checks `common_status` bit 14 (arm activated), bit 15 (selection admitted); vendor toast requires Remote Control to Pick Up. Arm pose required for useful pixels is unverified | Selection plumbing can use an **already supervised / entered** pickup session; no automatic pickup entry or arm movement. Physical lens not live-verified here |
| Depth / TOF | No live video selector recovered | Unknown | `getTofStatus` (`get_debug_voxel_map []`) returned `{"status":0}` live (boolean cloud-debug upload switch, not depth data); no local depth feed established |
| IR / structured light | No preview selector / pixel format recovered | Unknown | No independently accessible stream established; RGB video cannot be relabelled depth/IR |

## Primary evidence (one-based `H` lines)

Source: `/Volumes/GOLD16-HFSP/WD16GOLD01/Michael/s/Z70Android/targeted-plugin/index.android.hasm`.

- H230590–230603 (#5508): argument passed unchanged as `{flag}` to `app_change_camera`.
- H1220747–1220763 (#28055), directly spot-checked: boolean mechanical selection becomes **1 or 0**, then `appChangeCamera(flag)`. No channel array, extra stream, or preview restart.
- H1220683–1220722 (#28053): inactive-arm / denied-camera gate; toggles `!mechanicalCameraEnabled`. H315728 English denial: “Switching to the gripper camera is only available in Remote Control to Pick Up mode.”
- H342139–342154 (#8428), directly spot-checked: `common_status >> 14 & 1` activation, `>> 15 & 1` switch permission. H342115–342122: pickup submode bits 10–11. These are **not** camera indexes.
- H347778–347782 (#8583), directly spot-checked: `camera_status >> 15 & 1` mechanical selected feedback; feedback is not proof of pixel provenance.
- H1226621–1226632 (#28208), directly spot-checked: Monitor arm-view entry `_setGrapMode(2)`. H858228–858233 and H231663–231676: forwarded to **`app_pick_up_enter_exit {flag:2}`**, not camera flag 2.
- H1216801–1216828 (#27969), directly spot-checked: preview object fields are `quality`, `password`, `client_id`; no camera/channel index. H1302832–1302866 other caller uses same fields.
- H228132–228171: `send_sdp_to_robot {app_sdp}` and `send_ice_to_robot {app_ice}`. H1297545–1297577 / H1297694–1297724 consume `dev_sdp` / `dev_ice` without camera identity.
- H1296409–1296420: native `getHandle()` = `findNodeHandle(this.root)`; `WithTag` is a React Native view tag, **not a camera selector**.
- H231503–231514 / H232360–232373: `get_debug_voxel_map` / `set_debug_voxel_map {status}` under `getTofStatus` / `setTofStatus`. No debug write executed.
- H1156599–1156606: debug WebView builds a PCD viewer URL with local key / serial; no local depth transport or sensor identification established. Do not log/open the credential-bearing URL.
- H674406–674412: locale `end_tof_error73` = arm-end camera error. Name alone does not establish a depth feed.
- H316880–316887: generic `mf.camera_singleline`, `mf.camera_dualline`, `mf.camera_rgb`, `mf.camera_doublergb`; generic multi-model capabilities, not this robot's channel map.

Full-file opcode-string scan independently repeated: `app_change_camera` H230600 only; `appChangeCamera` H224542/H1220760; `changeCamera` H1213801/H1220717/H1226722/H1226816. No `cameraId`, `camera_id`, `camera_index`, `cameraIndex`, `backCamera`, `leftCamera`, `rightCamera`, or `depthCamera` matches. Broad camera/preview/TOF/depth scan includes phone camera permissions, UI animation depth, photo saving, and unrelated dry-mode depth; these are not robot feeds.

Corroborating offline native APK audit: `research/gripper-camera/native-apk-audit.md`. `RRVideoViewManager` exposes preview/render/capture, not camera ID / channel selection; incoming remote video stored in one field without track-ID/MID selection. This does not exclude hidden firmware capabilities. Stored `get_photo` images (H218401–218429) use a security envelope/gzip and **photo IDs**, not live sensor indexes (details: `plugin-stream-audit.md`).

## Pinned go2rtc signaling source

Inspected existing offline checkout of commit `b5948cfb25404cc5cb37b166ecaa2dca20b11d4b`, `pkg/roborock/client.go` (cached under Hermes scratch; source pin in `deploy/go2rtc.Dockerfile`).

- L140–145 offers **one recvonly video media section** plus sendrecv audio. There is no camera index or channel parameter in the offer construction.
- L123 / L280–281: `"0"` in `SendICEtoRobot(candidate,"0")` is `sdpMLineIndex` / `sdpMid`, a media-section ID, **not camera 0**.
- L235–237 preview fields: `client_id`, `quality:"HD"`, `password`; no camera selector.
- L349–352 switches HD/SD quality, not physical camera.
- URL parsing L49–65 consumes PIN; no camera query parameter. Adding `camera=left` to this source would be ignored, not expose another lens.

No speculative go2rtc stream-parameter patch is justified by these artifacts. Separate sensor pipelines might be internal-only; access remains unknown rather than claimed impossible.

## Historical proposed gripper session — superseded by shared mode

With Michael present and workspace clear:
1. Enable arm in official app if necessary (activation can have physical effects; no undocumented enable RPC guessed here).
2. Enter the app's Remote Control to Pick Up: evidenced `app_pick_up_enter_exit {flag:2}`. **This may move the arm; it must not be run in this investigation.** Prefer official UI for any required deployment / pose setup, observed by Michael.
3. Read `get_status []`; inspect main state, `common_status` bits 10–11/14/15 and `camera_status` together. Activation/permission do not prove the arm is out or stationary.
4. Keep one preview session; select `app_change_camera {flag:1}`; read `get_camera_status []` until bit 15 is 1. Acquire/vision-check JPEG; document resolution and actual physical viewpoint, not only ACK/bit.
5. Historical proposal used an automatic restore; this is superseded. Camera mode now stays selected until explicitly switched. Pickup exit and arm stow remain supervised actions, not implicit camera cleanup.

## Live evidence / implementation

### Live outcome (2026-10-14)

- Before preview: raw `get_status` reported `state:2`, `error_code:0`, `common_status:49154`, `camera_status:11685`. Programmatic decoding: pickup submode **0**, activation bit **1**, switch-permission bit **1**, mechanical-camera-selected bit **0**. This is important counterevidence: **both common bits can be set outside pickup mode**. Do not use them alone to initiate a gripper experiment.
- A single non-motion `app_change_camera {flag:0}` outside pickup returned **vendor -10007**, even though the ordinary camera was already selected. No flag 1 or guessed flags were sent. This code therefore avoids redundant flag-0 writes; exact semantics of -10007 remain unknown.
- `get_camera_status []` returned `[11685]` after that rejection; mechanical bit remained **0**. Front capture succeeded through existing MCP: **1600×1200 RGB JPEG** (local evidence `frames/front-before.jpg`). Vision saw a near-floor forward/upward view: floor seams/scuffs, fiducial grid lying flat, low shelving, colored objects and wall-mounted TV; no identifiable gripper. This is consistent with the known front chassis viewpoint, not a new sensor discovery.
- After implementation/deployment: named HTTP `{camera:"front"}` returned **1600×1200**, **108016-byte JPEG**, `requestedCamera:"front"`, `defaultRestored:"front"` (`LIVE.json`, `frames/front-after.jpg`). Vision independently saw the same low floor-level RGB viewpoint, floor target, storage shelves and TV; no clear gripper or depth/IR map. Camera status read **before and after** was `[11685]`; ordinary/front remained selected throughout. No alternate lens was selected, so no physical switch-back was necessary.
- Named left/right/back/depth/ir/structured-light requests each returned **HTTP 422**, live, rather than silently delivering front pixels. Unsupported names are rejected before any robot RPC/preview. A fresh real stdio MCP child advertised the new eight-name `camera` argument and returned 422 for `get_camera_frame({camera:"left"})` against the deployed service (`MCP_LIVE.json`); the validation client/child were closed in `finally`. Existing Hermes sessions with a cached old empty tool schema may need MCP tool-list reload.
- **Gripper was not selected or captured.** No pickup-mode, arm, chassis motion, cleaning, docking, sensor-debug or preview-mode RPC was sent manually. The existing authorized front-preview driver handles normal preview signaling internally. Its ordinary warm-consumer idle cleanup is unchanged.

### Historical per-request implementation (superseded)

`POST /api/camera/frame {camera:...}` and MCP `get_camera_frame` argument: front default; researched gripper selector for an **existing supervised pickup session only**, requiring nonzero reported pickup submode plus app activation/admission bits. Never enters pickup, deploys/moves/stows the arm. Camera frame requests serialize on one pipeline; gripper uses existing preview, confirms flag-1 feedback, permits decoder settling/discards a frame, restores flag 0 and verifies feedback in `finally`. Restoration failure propagates rather than returning a JPEG success. Status/settling do not automatically verify physical pixel identity; metadata keeps lens verification unknown. Unknown locations/modalities are honestly unsupported until evidence exists. Details/topic hooks: `ROS_HANDOFF.md`.

Offline behavioral coverage: unsupported/malformed names have no RPCs, outside-pickup permission bits cannot trigger gripper, admitted selection order and front restoration, capture/uncertain-selection failure restoration, failed restoration cannot report success, concurrent front requests wait, redundant front selection avoided, MCP argument/metadata forwarded through real in-memory SDK transport. `npx tsc -p tsconfig.json --noEmit` passed; `npm test`: **358 passed, 0 failed** (`tests.log`). Existing camera-diagnostics fixture now provides read-only camera feedback so startup-error evidence remains exercised.

Only RoboClaw **app** rebuilt/recreated; go2rtc unchanged. Both read back healthy/running with **RestartCount 0**. No bridge/dashboard edits by this camera agent, no git commit, no retained agent background processes.

### Additional bundled-plugin scan

Seven embedded APK ZIPs, eight `.bundle/.js/.jx` code artifacts scanned offline for exact byte strings `app_change_camera`, `camera_id`, `cameraId`, `cameraIndex`, `camera_index`, `leftCamera`, `rightCamera`, `backCamera`, `depthCamera`, `structured_light`, `infrared`: **zero hits** in those bundled artifacts. The separately downloaded/decompiled targeted plugin is the richer robot source above. A byte-string absence is a bounded lead check, not disassembly or proof of firmware absence.

### Still unknown / supervised follow-up

Physical gripper pixels/resolution and precise arm pose requirement; independently accessible side/back/depth/IR/structured-light feeds; hidden firmware selectors; whether debug voxel maps have genuine depth geometry on this firmware. Do not guess flag 2 or camera-index fields from the ICE MID `0`. The supervised sequence above is the next justified gripper experiment; unavailable cameras are not fabricated or marked working.


## Shared-mode correction (2026-10-14)

Claude's supervised live investigation (evidence: `research/cameras/live/`) confirmed gripper selection in pickup mode with the arm deployed: state 39, `common_status:51202`; `get_camera_status` changed 11685 -> 44453 (bit 15 selected) despite **no app_change_camera reply** on local/cloud transports. The same `z70` stream supplied 1600x896 gripper JPEGs versus 1600x1200 front JPEGs; vision confirmed a different higher viewpoint. Stowed-arm tests were inconclusive because the old bridge/default reads could switch it back.

The old per-request select/restore above is no longer the API contract. Selection is one shared physical mode, exposed by `POST /api/camera/select` and MCP `select_camera`. Missing selector ACKs are expected: 500 ms reply timeout, then roughly 3 seconds of bit-15 confirmation. `{}` frame reads never switch; explicit frame selection stays switched. Live status is cached for ~1 second and JPEG dimensions provide an independent cross-check; official-app changes are reflected. No automatic front restore, pickup entry or arm motion exists in camera acquisition. ROS publishes frames only on their reported front/gripper topics and front CameraInfo only for front frames. See `ROS_HANDOFF.md` and `docs/validation/shared-camera-mode.md` for current contracts and Claude's live validation steps.

Physical gripper availability is now evidenced; independent side/back/depth/IR streams and gripper calibration/extrinsics remain unknown. The shared-mode coding agent did not send selector, pickup or arm commands live.
