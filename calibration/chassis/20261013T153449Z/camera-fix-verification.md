> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [odometry guide](../../../src/z70_odometry/README.md), [camera guidance](../../../research/cameras/FINDINGS.md).

# Camera recovery — parent verification

Verified at 2026-10-13T16:30:07Z. Production app/go2rtc containers remain intentionally stopped. No chassis or arm motion was issued in this session, and no camera request or service start was issued during offline debugging.

## Implemented changes

- MCP capture deadline now covers the driver's bounded 30-second consumer startup plus 30-second snapshot work, preserving sanitized driver failures rather than replacing them with generic dispatch uncertainty.
- MCP-visible stream diagnostics expose only numeric counters. Corrected legitimate upstream `consumers:null` handling; availability describes a successful state read, not evidence of video.
- Pinned go2rtc exec pipe startup now honors starttimeout, cancels/kills/reaps the silent child, releases the opener and producer startup lock, and disarms the startup timer after success.
- MJPEG/keyframe HTTP consumers are removed exactly once on request cancellation, including close-before-WriteTo. Silent disconnected consumers no longer remain stuck indefinitely.
- Roborock producers no longer automatically reopen preview when Start exits or transport fails. Explicit subsequent demand remains possible after caller cleanup. Other producer schemes retain upstream reconnect behavior; stale workers cannot tear down a newer generation.
- Added new Go regression inputs to the deny-by-default .dockerignore, so ordinary Compose builds include them without admitting secrets/images.

## Independently executed verification

- Original lifecycle baseline failed for silent/partial pipe startup, pending AddConsumer, close-before-WriteTo and canceled silent MJPEG/keyframe handlers. See parent-go2rtc-red.log.
- Original streams baseline failed for both normal and error Start exit: automatic preview-producing reconnect, Dial count=2 rather than 1. See parent-final-red.log.
- Patched Go package tests passed for roborock/iot, shell, core, exec, mjpeg and streams; MQTT has no upstream test files. Full binary compiled.
- Three race-detector repetitions passed for exec, mjpeg and streams lifecycle regressions, with no race reports. Tests/patch/build stages use network=none; fixture tests use no robot, broker or live media service.
- TypeScript typecheck passed; npm test: 342 passed, 0 failed. See node-tests-final.log.
- Ordinary `docker compose build app go2rtc` returned 0. `git diff --check` returned 0.
- Both existing production containers read back as exited, restarts=0.

Build identifiers:
- Independently rerun race stage: sha256:a4c14decc3337db1b75fa74a270bbb27328bdf895566265d499f161d1927d493.
- Restricted-context verified runtime: sha256:35460c13183bc0f7ae92734a6731741662894fdacb356293066e3e83221d798b.
- Ordinary Compose-built roboclaw-go2rtc:local: sha256:bab84544b5ba8cf081c1688da7017879ca0e8cb6737fdd98e3c932dd8e5911de.
- Ordinary Compose-built roboclaw-app:local: sha256:8a8e8784c548dc317e7853e4738416920500640794f8978ae828b7dea81a21f5.

## One authorized hardware check — 2026-10-13T16:33:40Z

Michael authorized exactly one bounded capture, no movement/retries, then stack shutdown. Recreated both services from the verified Compose-built images above. Robot reported docked/charging (state 8), battery 87%, error 0.

The single `get_camera_frame` returned HTTP 502, `cameraFailure:no_camera_image`, `upstreamStatus:200`; no frame was produced and no capture retry was attempted. Fresh MCP state afterward reported both stream state reads available, each with configured producer metadata only and empty consumer lists. This demonstrates cleaned-up failure rather than the earlier indefinitely pending consumers; it does not identify the original media failure.

Owned preview cleanup: `get_homesec_connect_status` returned status 0/client_id 676f32727463; `stop_camera_preview` for that owned client returned received ok; the exact status read-back then returned status 0/client_id none. Both services were stopped and independently read back as exited, restarts=0, with the expected image IDs. No chassis/arm commands were issued. No restoration of hardware video is claimed, and the agent cannot hear/count the robot's announcements.

Remaining investigation is robot-side media transport/track admission, distinct from the now-reproduced and patched pipeline cleanup/reopen defects. Any further live capture requires new authorization; do not restart from this report.

## Resolution reported by Michael

Michael confirmed that the official Roborock app also failed to display the camera, then recovered after he restarted the robot. This supports a robot-side/robot-associated preview failure state, rather than a client-only regression. Recovery is user-reported in the official app; RoboClaw/MCP recovery was not retested. The firmware-internal cause and the meaning of vendor code -10013 remain unknown. See [the failure-mode note](../../../docs/validation/camera-preview-start-rejection.md). Investigation ended at Michael's request; no further live action is authorized by this report.

## Limits

The root cause of the initial robot-side media silence is still unresolved. Offline lifecycle fixes and a passing build do not prove restored camera input, physical preview release or elimination of every firmware privacy announcement. Original producer counters showed no received RTP, despite negotiated receiver metadata. The browser-facing loopback candidates are not the robot producer's independent Pion ICE/TURN configuration and were left unchanged.

Two pinned upstream legacy streams tests do not compile against their current API; the build retains their original file outside the test glob identically in RED/GREEN. Existing API tests and local behavioral regressions remain enabled. These two legacy tests are not claimed as passed. No production constants or camera_x defaults changed.

Review and publication status is recorded in Git history. Existing unrelated ros-latest.json files and opencode.json are untouched. RoboClaw recovery after the owner-performed robot restart remains untested; no additional capture or restart is authorized by this report.
