# Z70 arm/chassis validation session — protocol for GPT

**Audience:** the Hermes session that runs this (GPT), with Michael physically present. Follow it as written.
**Goal:** measure whether the vendor app's arm model (`docs/z70-arm-model.md`) matches the real robot, and calibrate
the chassis remote-control pulses. Produce numbers, not process.

## Rules
- Michael is present for the whole session and watches the robot. Don't ask for confirmation before every
  command. Ask once at the start, then run the protocol. Stop and report if motion is unexpected, a frame can't
  be fetched, or a command's outcome is unclear (timeout). Don't retry ambiguous commands.
- Use the RoboClaw MCP tools only (`mcp_roboclaw_robot_rpc`, `mcp_roboclaw_get_camera_frame`,
  `mcp_roboclaw_refresh_status`). No shell/HTTP calls to the robot. Never read `.local/` secrets.
- Don't add gates, wrappers, permission layers, or tests that pin formats. The only code to write is the
  logger/analysis script below.
- Save everything under `calibration/arm-validation/` (images git-ignored). Never use Hermes scratch, which is pruned.

## Prerequisites (Michael)
1. RoboClaw rebuilt with the timeout fix: `cd ~/Code/RoboRockZ70-Connector && docker compose up -d --build`.
   Then start a new Hermes session (MCP tools load at session start).
2. **Marker:** cut one full **40 mm** square from the 5×7 board (the black ArUco marker inside is 30 mm), keeping
   the white margin around the marker. Tape it flat on the gripper's outer face. Measure with a ruler: the marker
   centre's offset from the wrist joint (M5 axis), and from the fingertips. Write the numbers into `session.json`.
3. Floor clear in front of the dock. Leave the floor ChArUco boards in place for Part B.
4. If `get_status` reports state 42 (programming mode, left over from 2026-10-06), exit it first with
   `app_program_enter_exit {flag:0}`. This command has never been tested. Note the response and the robot's behaviour.

## Measurement method
- **Marker pose:** OpenCV `ArucoDetector` (DICT_4X4_1000) plus `solvePnP` with `calibration/front-camera/front_camera.json`
  (fx=fy=f_px, cx, cy, k1, k2) and a marker side of **0.030 m**. Reuse `calibration/front-camera/detect.py` settings.
- **Analyse step displacements:** use the change in marker position between consecutive frames, in camera
  coordinates. This doesn't depend on the unknown camera position on the body. The rotation change gives the
  joint-angle change for pure single-joint moves.
- **Settling:** after each arm command, wait for the RPC reply plus **2 s**, then take **2 frames**. Use the second;
  discard both if they differ by more than 1 px in marker centre.
- **Logging:** one JSONL line per command in `calibration/arm-validation/log.jsonl`:
  `{t, method, params, rpc_reply, frame_file, marker:{id, corners_px, rvec, tvec_m} | null, app_model_state:{m3,m4,m5,m6}}`.
  Track `app_model_state` with `src/z70_description/z70_description/arm_model.py` (`ArmState`); it is a nominal model, not joint feedback.

## Part A — arm (manual mode, no programming mode needed)

1. **Deploy:** `app_pick_up_enter_exit {flag:1}`, wait 5 s, `app_arm_out_compartment {}`. App model: `WORKING_POS`.
2. **Find the marker:** send `app_arm_direction_move {direction:3}` (forward) one at a time, up to 6, then
   `{direction:2}` (down) if needed, until the marker is detected. Log every step; these are data too.
   If it never appears, stop Part A and report. The fallback is Michael filming side-on with a phone.
3. **End-effector steps:** from the first pose where the marker is visible:
   - 3× forward, 3× back, 3× down, 3× up (`direction` 3, 4, 2, 1), with each direction run as a block;
   - repeat the whole block twice.

   The app model predicts **0.040 m** per step.
4. **Joint jog steps:** `app_arm_move {m:[M3,M4,M5]}` with one joint non-zero (1 = +, 2 = −). The app model predicts **10°** per step.
   - **M5 (wrist):** 3× +, 3× −, ×2. Measure marker rotation about the joint axis.
   - **M4:** 2× +, 2× −, ×2.
   - **M3:** 2× +, 2× −, ×2.

   Watch for the app's limits; stop a direction if the arm stops moving.
5. **Gripper:** `app_open_gripper {}` / `app_grip {}` once each (already confirmed physically). Log only.
6. **Stow:** `app_arm_in_compartment {}`, then `app_pick_up_enter_exit {flag:0}`. Confirm with Michael that it's stowed.
7. **Deployed-pose repeatability:** deploy → forward ×N (same N as step 2) → frame → stow, **3 times**. The spread of
   the marker position at the same N tests whether "stowed → deployed" is a reliable reset point.

## Part B — chassis (floor boards in view)

Floor boards give the camera pose by `solvePnP` on ChArUco corners (see `calibration/front-camera/detect.py`). For each
move: frame before → command → frame after.

Sequence: `app_rc_start`, wait **5 s** (required), then single `app_rc_move [{velocity, omega, duration, seqnum}]` pulses:
- **Rotation:** omega ±0.5 at duration 400, 800, 1200 ms; each twice, alternating sign so the boards stay in view.
- **Translation:** velocity 0.1 at 400, 800 ms forward, then the same backward (velocity −0.1); each twice.

Finish with `app_rc_stop`, `app_rc_end`. Measure yaw (deg) and translation (mm) per pulse from the board pose change.

## Analysis — write `calibration/arm-validation/analyze.py` and `RESULTS.md`
Report tables, nothing else:
1. **End-effector step (m), per direction:** mean, std, n, measured / app-predicted (0.04).
2. **Joint step (deg), per joint and sign:** mean, std, n, measured / 10.
3. **Slack:** the extra command needed after a direction reversal (difference between the first step after a reversal and the steady steps).
4. **Deployed-pose repeatability:** marker position spread (mm) over 3 cycles.
5. **Chassis:** deg per (omega · s), and mm per (velocity · s), with spread. Is the relation linear in duration?
6. **Camera extrinsics:** fit `camera_x` (and check `camera_z`, pitch) on the body. This combines the camera-frame
   marker positions with app-model wrist positions, plus the measured marker-to-wrist offset. Least squares;
   report residuals.

**Decision rule** (state it explicitly in RESULTS.md):
- **Adopt the app model as-is** if every step ratio is within 0.85–1.15 and the std is ≤ 20 % of the mean.
- **Otherwise,** write corrected constants (per joint, per sign) to `calibration/arm-validation/arm_model_measured.yaml`.

Then commit the scripts, `log.jsonl`, `RESULTS.md` and the YAML (not images), and stop.
