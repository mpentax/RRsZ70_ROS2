# go2rtc camera MQTT login compatibility fix

## Hardware finding

The cached camera account is valid: cloud `get_status` succeeded through RoboClaw MCP. A credential-safe source audit confirmed go2rtc's broker, account, device key and PIN match that account.

CONNECT-only diagnostics against the same broker and credentials produced:

| Client identity | CONNACK return code | Result |
|---|---:|---|
| Legacy `com.roborock.smart:mbrriot` | 5 | Not authorized |
| Unique `roboclaw-<UUID>` | 0 | Accepted |

This isolates the failure to the fixed MQTT client identity. Why the broker began rejecting that identity is not established; no camera-account credentials, PIN or robot settings were changed to restore capture.

## Patch and deployment

`deploy/go2rtc.Dockerfile` verifies and builds upstream v1.9.14 commit `b5948cfb25404cc5cb37b166ecaa2dca20b11d4b`, using a SHA-256-checked source archive and pinned Go builder. `deploy/patch-go2rtc.go` changes only the camera MQTT client-ID callsite and adds its unique-ID helper. The final image retains the pinned upstream runtime, FFmpeg, certificates and init process, replacing only the go2rtc binary.

The Go regression test fails with the original shared identity and passes with the patch. `docker-compose.yml` builds and uses `roboclaw-go2rtc:local`; a normal rebuild preserves the fix:

```sh
docker compose up -d --build go2rtc app
```

No go2rtc admin port is published. Private state stays mounted only into the app. go2rtc logging remains disabled to avoid retaining credential-bearing URLs.

## Verification

- Original-identity Go regression build: failed as expected, `camera connections reuse a shared broker identity`.
- Patched Go MQTT/Roborock tests and binary build: passed.
- Node focused camera/MCP tests: 17 passed.
- TypeScript: passed.
- Full Node suite: 316 passed; the pre-existing `local-selection.test.ts` persistence test remains failing (also failed before edits).
- App and patched sidecar: healthy.
- Two fresh 1600×1200 JPEGs acquired through `get_camera_frame`; the first was visually inspected. Local evidence: `calibration/arm-validation/camera-fixed-frame-{1,2}.jpg` and `session.json`.

`get_camera_diagnostics` returns bounded, numeric MQTT CONNECT evidence without subscriptions, preview startup or robot RPCs. `get_state.camera.sourceAudit` returns only consistency booleans. Neither exposes credentials. Camera frame errors retain allowlisted failure categories and numeric upstream HTTP status; arbitrary upstream text is never forwarded.
