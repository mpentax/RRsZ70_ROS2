> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [current arm model](../../../docs/z70-arm-model.md), [camera/calibration guide](../../../research/cameras/FINDINGS.md). Optical identifiability limits below apply to this experiment; copied CAD now supplies structural anatomy, not independent physical registration or contact validation.

# M3 rotation coverage — BLOCKED

No new M3 angle or axis measurement was acquired. No shoulder, elbow, wrist or chassis jogs were attempted. The authorized forward/return sweeps, multiple fixed-elbow configurations, stationary pilots, pincher crosschecks and prospective holdout evaluation remain unavailable, not zero-error results.

## Acquisition and failure boundaries

- Fresh MCP status/get_state preceded ONE retained preflight front frame. The full 1600x1200 image shows the stationary board and no arm marker. The board fit accepted with full RMS 2.4750px and optical angle to floor 1.6811deg; these are optical observations, not a new camera-height constraint.
- Raw get_status established idle state3, battery55%, error0; estimator IN_ROBOT was dead reckoning. Board was reasonably centred, so no remembered deployment-yaw correction or chassis move was sent.
- Pickup entry flag2 and arm-out were ACKed. Deployment status was state39, common_status51266, then51458; deployment/clearance was NOT proven by those fields. No jog followed without pixels.
- First deployed capture failed MCP502/no_camera_image/upstream200. Retained attempt198 failed at get_answer/answer_timeout after7645ms: MQTT/PIN/ TURN ok, pc/ICE/DTLS new, zero tracks/packets/bytes. This is earlier than the prior trial's connected-but-no-media boundary.
- Inspected the failure BEFORE any new capture. Released only observed owned go2rtc client676f32727463; readback status0/client_id:none. After20s quiet, ONE controlled same-camera retry failed. Attempt199 failed at start_preview/rpc_error with signed vendor code **-10013**, reason unknown, MQTT/PIN ok, zero tracks/packets. ICE/DTLS were unknown/not established; do not diagnose decoder or credential failure from this.
- Three capture requests total: one retained original, two failures. No lost successful SDK images. No camera-switch/reopen loop, random mode experiments, robot reboot, direct HTTP/SDK fallback, Docker/service restart or ROS/live-preview changes.

## Cleanup — NOT FULLY VERIFIED

Stow ACKed, waited14s, app_program_enter_exit flag0 ACKed and pickup exit flag0 ACKed. Raw status remained39/common_status51458 after a further10s, so those ACKs did NOT establish mode exit or physical stow. Attempted the previously useful app_stop correction: twice returned not_sent/transport_error unavailable. The second attempt was an explicitly allowed stop retry, not motion replay. Physical-stop warning was sent immediately after the first failure.

Final MCP get_state reports disconnected/tcp-connect and null telemetry with Device transport failed. Empty original/decoded consumer lists verify cleanup only; they do not restore video. Final estimate IN_ROBOT was initialized by stow dispatch and is NOT encoder or image proof. Front camera was last confirmed11685 before disconnect. No final frame was requested into the failed link. **Parent/operator must resolve physical stow and mode exit; no claim of safe completed cleanup is made.** Last valid battery55%, error0, no cleaning/return.

## Evidence and scope

- live.mjs: MCP SDK only, durable full JSON log and copies/decodes successful image bytes before another request; observe.py reused read-only from prior trial.
- protocol.json: frozen prospective holdouts before deployment (A third-forward/all-return and B configuration excluded from A training); they were never populated/evaluated.
- raw/preflight-front.jpg and adjacent full detections/metadata; log.jsonl; stage-specific failure states; final-state.json/final-estimate.json.
- results.json uses null for unobserved angles, axis, closure/backlash/noise/crosscheck/holdout errors. Zero observed settings is not a physical zero-angle finding.
- finalize.py verifies every prior artifact hash and retained new image hash, then writes verification.json and artifact-sha256.json. Prior archive has202 files/91 original images unchanged.
- One executed SO3 fixture confirmed cancellation of an arbitrary fixed marker mount; it is mathematical, not hardware validation. Detector imported/executed against real preflight pixels.
- No production files, shared libraries, prior archives, planners, firmware, model geometry or live visualization modified; no commits. Rotation-only data would not identify physical L1/L2, axis station or unique mechanical zeros even if acquired.

Reproduce: `calibration/arm-validation/.venv/bin/python calibration/arm-calibration/20261015T120106Z-m3-rotation-coverage/finalize.py` (offline only).
