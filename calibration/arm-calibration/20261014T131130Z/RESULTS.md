> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [current arm model](../../../docs/z70-arm-model.md), [camera/calibration guide](../../../research/cameras/FINDINGS.md). Optical identifiability limits below apply to this experiment; copied CAD now supplies structural anatomy, not independent physical registration or contact validation.

# Resumed arm calibration — camera acquisition blocked

## Outcome
No front JPEG was delivered, so the mandatory initial board-visibility check could not be completed. **0 usable front frames, 0 usable gripper frames, 0 measured poses.** No deployment, joint jog, chassis move, grip, or pick goal was sent. The only arm command was the final redundant stow.

Two capture paths were exercised: the available Hermes RoboClaw MCP tool, then the retained incremental `calibration/arm-calibration/live.mjs` helper through RoboClaw MCP. Both returned HTTP 502, `cameraFailure: no_camera_image`, upstream HTTP 200. No image files could be created.

## Diagnosis
The bounded CONNECT-only camera diagnostic returned valid MQTT CONNACK **returnCode 5 / accepted:false for both fixed and unique client identities**. This is broker admission evidence, not a board-placement failure and not proof of an incorrect robot PIN. Source audit still reports matching broker/account/device/PIN fields, which does not establish working authentication. Camera state snapshots show TCP/TLS stages, zero video tracks/packets/bytes, empty consumers. ROS continues polling frames; snapshot attempt counters are therefore not counts of this agent's requested captures.

Both app and go2rtc containers are running, restarts=0, OOM=false. Neither was restarted; no credentials were read, printed, reset, or modified. The specific account-side reason for the broker refusal remains unknown. Live frame delivery must be recovered before calibration can proceed.

## Geometry / requested fit
No new shoulder, L1/L2, joint-offset, camera-mount, sigma, RMS, wrist or jaw measurement is available. Requested jaw positions at (240,105)/(230,95) and recommended 3.3-cm-equator poses remain unknown. `fit.json` records nulls, not synthetic fit values. Vendor baseline remains shoulder (0.0475,0.14) m, L1=0.1775 m, L2=0.205 m in the vendor frame; it is not validated by this session.

No planner constants/tests were changed: the old gripper-only fit cannot independently resolve L2 versus camera translation along link2, or M4 zero versus free mount orientation. Wrist ID0 black side is 15 mm, but independent marker-to-wrist registration is still needed for physical wrist-axis geometry rather than marker-space motion alone.

If new measured geometry is adopted later, review `src/arm-model.ts`, `ros2/src/z70_description/z70_description/arm_model.py`, arm URDF geometry and `mount_calibration.yaml` consistently with `planner.py`. This session establishes no new change to propagate.

## Tests and cleanup
`cd ros2 && ~/.pixi/bin/pixi run test src/z70_manipulation`: **27 passed in 12.47 s**, 7 packages built successfully.

Sent `app_arm_in_compartment []`, received `ok`, waited 15 seconds and read exact status back: state=3 (idle), battery=51%, error_code=0, common_status=49154, arm-ready bit cleared. Estimate read back IN_ROBOT, joints all zero, wrist=null, stepsSinceAnchor=0. Front selection confirmed. This corroborates stow through status plus dead reckoning; no physical encoder or final image is available. Read-back status, not just command ACK, is retained in `log.jsonl`.

## Files
Created only this dated evidence directory: `log.jsonl`, `diagnostics.json`, `fit.json`, `RESULTS.md`. No code changes and no commit. Existing unrelated working-tree changes were left untouched. Running ROS observation and app/go2rtc services were left running.
