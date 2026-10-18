# Image acquisition: established timeline and missing evidence

## User-reported resolution

Michael subsequently reproduced the camera failure in the official Roborock app. Restarting the robot restored video in that app. This supports a robot-side/robot-associated preview failure state; it does not prove a firmware trigger, universal meaning of -10013, or recovered RoboClaw capture. The final instrumented client attempt passed MQTT/PIN and failed at start_camera_preview with vendor code -10013, before TURN/ICE/DTLS. See acquisition-vendor-rejection.json and [the failure-mode note](../../../docs/validation/camera-preview-start-rejection.md). Earlier hypotheses below remain the investigation history, not the final ranking. No further live test was run after Michael asked to document and move on.


## Working-to-failing transition

- Commit `9fcd31d` replaced direct `z70` snapshots with a continuously decoded `z70_jpeg` path and a short-lived MJPEG consumer. The retained live measurements establish 265 JPEGs over 60.141832875 s and 181 raw/compressed ROS frames. They establish warm throughput and cleanup, not a successful real reopen after the idle interval.
- The Mac's boot time, retrieved from `kern.boottime`, is 2026-10-13T15:24:15Z.
- A real 1600x1200 MCP JPEG was delivered at receipt 2026-10-13T15:34:31.401Z, after that boot. The retained file is setup.jpg. Therefore 'no camera ever worked after reboot' is not supported.
- After Michael manually relocated the robot, fresh status at 15:36:47.574Z showed idle state 3. The subsequent setup camera read failed. There was both an idle interval and a physical relocation between the successful and failed reads; this timeline cannot distinguish them causally.
- Repeated read-only captures, service restarts and owned-preview stop did not restore video. No chassis pulse was sent in this session.
- MQTT diagnostic evidence separately showed fixed shared client CONNACK5 and unique client CONNACK0. All sourceAudit fields matched. This does not establish that every actual preview dial passed broker/PIN/SDP/ICE, only that the diagnostic broker probe succeeded and configured fields matched.
- Some original-stream snapshots showed negotiated receiver/sender metadata without received counters. Such receivers can be created from SDP; counters omit zero values. After cleanup, metadata-only producers and empty consumers do not establish what occurred before cleanup.

## Established defects, distinguished from original media failure

1. MCP canceled the camera request at 20s although the driver could spend 30s opening a consumer plus 30s waiting for a frame. This hid the sanitized driver error.
2. The pinned exec pipe path did not apply starttimeout to magic.Open, so silent media could indefinitely hold the derived producer's startup lock.
3. Silent MJPEG/keyframe HTTP requests did not remove consumers on cancellation, including close-before-WriteTo.
4. Generic go2rtc worker exit automatically re-dialed a Roborock source, which starts preview again.
5. Our initial stream diagnostic rejected upstream's legitimate consumers:null shape.

Each implementation defect was independently reproduced offline and corrected. These explain misleading errors, poisoned retries and a path to repeated preview openings; none proves why incoming video initially stopped.

## Authorized live checks after lifecycle changes

First check: docked state8, battery87, one capture returned502/no_camera_image/upstream200. Owned preview was explicitly stopped and read back status0/clientnone; both services stopped.

Later Michael explicitly authorized live images. One further controlled capture with existing production images returned the same502/no_camera_image/upstream200. A Docker attach HTTP101 handshake was verified before capture; no raw upstream logs were retained. Safe events were EOF, timeout, EOF (see camera-observer-result.json). No matching wrong-login/PIN/SDP categories appeared, but category absence is not proof that those boundaries passed. Fresh robot status was docked state8, battery100; preview readback was already status0/clientnone. Both services were stopped afterward. No movement or same-run capture retry.

Earlier camera-log-categories.json contains one error/permission category but never established successful observer attachment. Do not treat its zero category counts as clean-driver evidence.

## Ranked, falsifiable remaining explanations

1. A preview-control or SDP exchange fails during the actual producer dial. Prediction: retained diagnostic stops at that stage, before a connected media transport.
2. The robot-side ICE/DTLS path is admitted transiently or fails after reboot/idle. Prediction: preview and answer succeed, but ICE/DTLS never remain connected; selected candidate type/protocol and state transitions distinguish it.
3. Transport connects but no video is emitted/received, or the media reader exits. Prediction: connected ICE/DTLS with zero video OnTrack/RTP versus positive OnTrack followed by a categorized reader failure.
4. Video reaches the driver but decoding/JPEG delivery fails. Prediction: incoming video bytes/packets rise while derived JPEG startup/output fails. Genuine zero incoming RTP would weigh against this explanation.

Browser-facing webrtc.candidates/port57012 are not the Roborock producer's independent Pion API, so changing browser LAN exposure would not test these hypotheses.

## Next discriminating evidence

Retain the latest acquisition attempt's stage, bounded failure category, ICE/DTLS states, selected candidate types/protocol, video track/packet/byte counters and reader-exit category across cleanup. No source URLs, identifiers, SDP, candidate addresses, TURN credentials or arbitrary error text may be retained. Instrumentation is being built offline, without changing acquisition behavior, retries or timeouts. A later live acquisition under Michael's authorization must read this retained record, not just post-cleanup stream state.
