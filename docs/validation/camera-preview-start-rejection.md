# Camera failure mode: robot-side preview-start rejection

## Observed incident — 2026-10-13

Camera acquisition had worked, then failed after an idle interval and manual robot relocation. A Mac reboot preceded the incident, but a real frame was received after that reboot; a host-boot-only failure is not established.

Instrumented RoboClaw/go2rtc acquisition established the failing boundary:

- Actual camera MQTT admission: `ok`.
- Actual camera PIN check: `ok`.
- `start_camera_preview`: vendor RPC rejection **`-10013`**.
- TURN, ICE and DTLS: not reached in that attempt.
- Incoming video: zero tracks, packets and bytes.
- Snapshot result: HTTP 502, `no_camera_image`, upstream HTTP 200 with no image.
- No active preview afterward: `get_homesec_connect_status` returned `status:0`, `client_id:none`.

Repeated Mac-side service restarts and owned-preview cleanup did not restore acquisition.

Michael then independently tried the **official Roborock app**, which also could not display the camera. **After Michael restarted the robot, video worked in the official app.** These two app observations and the robot restart are user-reported, not agent-measured.

## Conclusion and limits

Treat this as a **robot-side camera/preview failure state that was cleared by a robot restart in this incident**, rather than a demonstrated Docker, FFmpeg, host-network or credential regression. The cross-client failure and recovery narrow the failure to robot-side or robot-associated session state; they do not identify its firmware-internal mechanism.

The meaning of `-10013` remains unmapped. Do not turn it into a universal permission, PIN, busy or network diagnosis. The reason category was `unknown`.

The trigger remains unknown: idle, relocation, prior preview/session activity and the host reboot were not isolated experimentally. Robot restart is an observed recovery for this incident, not a validated universal cure. RoboClaw/MCP capture **after** the robot restart was not re-tested because Michael asked to document the failure and move on.

## Triage for recurrence

1. Locate the actual failing boundary with the retained acquisition diagnostic. MQTT/PIN success plus `start_preview` rejection precedes media transport and decoding; do not tune FFmpeg or expose ports on that evidence.
2. Stop RoboClaw's preview/retry activity and test the official app **sequentially**, not concurrently.
3. If the official app also fails, investigate robot-side camera/session state before repeatedly restarting Mac services, changing credentials or modifying networking.
4. An owner-performed robot restart is a reasonable incident-recovery option after the cross-client check. Do not reboot the robot automatically. Record whether official-app viewing recovers, then separately verify RoboClaw only if further live acquisition is authorized.
5. If only RoboClaw fails while the app works, investigate third-party preview parameters/admission separately; do not assume this robot-side failure mode.
6. Keep the firmware privacy announcement intact. Avoid automatic or hidden preview-start retries instead of suppressing it.

## Separate client defects found during investigation

These aggravated failures but were **not shown to cause the robot-side camera state**:

- The MCP camera deadline expired before the driver's bounded work could return diagnostic evidence.
- Pinned go2rtc pipe startup could wait indefinitely for FFmpeg output.
- Silent MJPEG/keyframe consumers were not removed on request cancellation.
- Generic producer reconnect could automatically start Roborock preview again.
- Failed warmup was swallowed and followed by a second startup through the snapshot path.
- Upstream `consumers:null` was initially misclassified as an unavailable stream.

Corrections were implemented and exercised offline. The last instrumented live capture confirmed one acquisition attempt per MCP capture after removing hidden fallback. See Git history for review/publication status; no further live action was taken for this documentation update.

## Evidence

- [Retained vendor rejection](../../calibration/chassis/20261013T153449Z/acquisition-vendor-rejection.json)
- [Earlier instrumented boundary](../../calibration/chassis/20261013T153449Z/acquisition-live-boundary.json)
- [Acquisition timeline](../../calibration/chassis/20261013T153449Z/acquisition-timeline.md)
- [Parent verification](../../calibration/chassis/20261013T153449Z/camera-fix-verification.md)
- [Offline lifecycle verification](../../calibration/chassis/20261013T153449Z/go2rtc-offline-fix.md)
- [No-automatic-reopen verification](../../calibration/chassis/20261013T153449Z/go2rtc-no-reopen.md)
- [Retained acquisition diagnostics](../../calibration/chassis/20261013T153449Z/acquisition-diagnostics-build.md)

Images remain local and git-ignored. Recovery in the official app is recorded from Michael's report; it must not be presented as an agent-verified recovered RoboClaw frame.
