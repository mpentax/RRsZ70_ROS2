> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [current arm model](../../docs/z70-arm-model.md), [camera/calibration guide](../../research/cameras/FINDINGS.md).

# Camera regression — resolved

## Root cause established on hardware

| Check | Observed result |
|---|---|
| Roborock app live viewing | User confirmed working |
| Camera account via RoboClaw cloud `get_status` | Request `1849dc6c-c292-4847-bf50-330ecf0ed827`: response received |
| go2rtc source vs cached account/device/PIN | Broker, account, device and PIN all match |
| MQTT CONNECT with legacy fixed client ID | Valid CONNACK, return code 5 (not authorized) |
| MQTT CONNECT with unique client ID | Valid CONNACK, return code 0 (accepted) |
| Patched go2rtc capture | Two fresh 1600×1200 JPEGs received through MCP |

The earlier credential-refresh hypothesis is superseded: cached credentials worked unchanged. The blocking difference was go2rtc's fixed MQTT client identity `com.roborock.smart:mbrriot`. The broker accepts a unique identity. The cause of the broker's changed policy is not established.

## Applied fix

The Compose sidecar now builds the exact previously pinned v1.9.14 source/runtime with a narrowly scoped patch generating `roboclaw-<UUID>` for each camera MQTT connection. Source archive and builder/runtime images are pinned and verified. See `deploy/go2rtc-login-fix.md`.

Diagnostic improvements preserve safe failure categories through driver, service and MCP. The upstream JPEG handler returned empty HTTP 200 after a producer-start failure; the old service/MCP path hid it behind 502. The new path reports `cameraFailure` and `upstreamStatus` without upstream credential-bearing text. Added source consistency and bounded CONNECT-only diagnostics remain available via MCP.

## Execution verification

| Verification | Result |
|---|---|
| Original fixed-ID Go regression test | Failed as expected |
| Patched Go tests and binary build | Passed |
| TypeScript | Passed |
| Focused Node camera/MCP tests | 17 passed |
| Full Node suite | 316 passed, one pre-existing local-selection persistence failure |
| Docker app and patched sidecar | Healthy |
| First frame | `a7aa3cbe-1b47-45ea-a2ee-b92732d451d6`, 81,933 bytes, SHA-256 `dc4b80ba40cf5c1538ba6d7cbb79997f2f07a6c72256b6b9b90c0a5bba5cf3dc` |
| Second frame | `9d0c8de7-36a7-4869-a991-c5c481dcfee6`, 81,482 bytes, SHA-256 `d48ad83508ae975be85672c3dcfb731bd9fc6faf98b20801681681bbdf6d9238` |
| Visual inspection | Usable floor-level scene; no visible calibration boards or identifiable deployed arm |
| Arm/chassis movement during diagnosis | None |

Frames remain local and git-ignored. Receipt time is not exposure time; lens identity remains unverified. Calibration cannot be reported until the marker/boards and measured offsets are established. Programming mode expired during diagnosis (last observed state 3); reacquire status before resuming the measurement protocol.
