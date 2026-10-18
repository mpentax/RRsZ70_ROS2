# Pinned go2rtc camera compatibility and diagnostics

## Maintained build

[`go2rtc.Dockerfile`](go2rtc.Dockerfile) verifies upstream v1.9.14 commit
`b5948cfb25404cc5cb37b166ecaa2dca20b11d4b` with source SHA-256
`78aa79bcedec8f155e4060a379613979b0b3ee48ff62ee5164bafc0ac6532386` and pinned
builder/runtime images. It retains upstream FFmpeg, certificates and init,
replacing the go2rtc binary. Dependency acquisition is separate from offline
patch/test/compiler stages.

`patch-go2rtc.go` covers unique MQTT client identities, bounded exec/pipe startup,
request-cancelled MJPEG consumers/write-buffer lifecycle and no automatic
Roborock preview reopen. `patch-go2rtc-diagnostics.go` adds bounded acquisition
stage/status counters for MQTT, PIN, preview, ICE/DTLS and media delivery. This is
not only a client-ID patch. Offline baseline/race stages exercise these boundaries.

From the repository root, within an authorized deployment window:

```sh
docker compose -f deploy/compose.yaml build go2rtc app
docker compose -f deploy/compose.yaml up -d go2rtc app
```

An image build is not deployment. Do not restart services underneath a live
controller. Admin port 57001 remains private; only media is published. Source
URLs contain credentials: go2rtc logs are not persisted. Private state is mounted
only into the app, not copied into images or exposed through diagnostics.

## Credential-safe triage

Current MCP `get_camera_diagnostics` reads shared ROS camera/connection evidence;
it does **not** run a broker CONNECT probe or open a diagnostic preview. Inspect
source timestamps/stale flags before drawing conclusions from cached evidence.

Find the first failing boundary: MQTT admission → PIN admission → preview start →
ICE/DTLS → incoming RTP → decode/JPEG. A camera error is not automatically a bad
PIN. MQTT `wrong login` means broker admission; compare sanitized account/session
consistency before recommending browser reauthentication. Neither negotiated
tracks nor configured state proves pixels. Suppress repeated consumer polling
while diagnosing repeated failures; do not create hidden preview retries.

If the official app is compared, run clients sequentially after releasing the
owned preview. The [preview-start rejection incident](../docs/validation/camera-preview-start-rejection.md)
shows why a robot-side rejection must not be diagnosed as FFmpeg/host networking.
Robot restart was one owner-reported recovery, not a universal automatic cure.

## Historical fixed-ID incident

The original finding/test receipt below is dated pre-cutover evidence. Its
Compose filename, patch-only description, active MCP CONNECT diagnostics and
suite counts describe that implementation, not the maintained contract above.

<details>
<summary>Original login-compatibility finding and verification</summary>

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

</details>
