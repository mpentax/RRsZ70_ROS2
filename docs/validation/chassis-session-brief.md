# Historical brief: supervised chassis odometry trial

Superseded by the ongoing, passive calibration described in `chassis-odometry-trial.md`. This brief is historical context, not a current motion protocol.

Follow `docs/validation/chassis-odometry-trial.md` (the protocol), with these additions:

1. **Camera is now ~3 Hz warm** (`docs/validation/camera-throughput.md`). The cold first frame takes about 7 s; after that, frames arrive in about 0.2 s.
   The ~1 Hz notes in the protocol are outdated. Keep the pulse schedule unchanged.
2. **Live ROS end to end.** Before the first pulse, start the stack in the background:
   `~/.pixi/bin/pixi run bringup` from the repository root. Record `/odom` and typed `/z70/odom_status` to
   `calibration/chassis/<session>/ros-odom.jsonl` for the whole session, for example with a small rclpy recorder script.
   After each pulse, note the ROS odom delta next to the offline pair analysis. At the end, save one
   `/z70/floor_mosaic` image to the session folder. Stop every process you started.
3. **Results:** write `calibration/chassis/<session>/RESULTS.md` with the fit
   (camera_x_m and per-sign gains, against the adoption criteria), how ROS live odom compared with pair analysis, and the
   VO rejection rate on live frames. End it with an explicit "robot released" line with a timestamp. Commit results
   without images. Don't change `camera_x_m` defaults in code; report the value and Claude will adopt it.
4. **Images: deterministic checks first, VLM by exception.** Between pulses, judge each pair with
   `calibration/chassis/analyze.py` (accepted/rejected, inliers, rms) and the live `/z70/odom_status`. Don't send
   per-pulse frames to vision_analyze. Use vision_analyze only for the setup check, and when the analyzer rejects a
   pair or the numbers look wrong (e.g. a board was displaced or something entered the view). Keep the
   capture → pulse → capture loop tight: one pulse every ~10 s is fine.
5. **RoboClaw fix since the last attempt:** the previous abort was a RoboClaw bug, the robot sequence > 0xffff, fixed in
   `947b999` (see `calibration/chassis/20261013T144619Z/RESULTS.md`). Use a new session folder and seqnum starting at 1.
   Still treat any `dispatch: unknown` as a stop condition.

## Rules
Use MCP tools only for robot actions (`robot_rpc`, `get_camera_frame`, `refresh_status`). Don't use raw HTTP or shell for the robot. Never read `.local/` secrets.
Ask Michael once at the start, then run the pulses without per-command prompts. Stop on the protocol's stop rules.
No retries of ambiguous pulses, no compensating moves. Arm stays stowed: no arm commands.

## First turn
Read-only preflight only:
- run `refresh_status` and `get_camera_frame`, then `get_arm_estimate`;
- report state, dock and battery, and describe what the camera sees.

Then ask Michael, in one message, to confirm all of these:
- he is present;
- the arm is stowed;
- the boards are flat in view (0.15–0.8 m ahead);
- there is about 1 m of clearance around the robot;
- he accepts undocking/RC mode.

Then stop. Don't move anything in the first turn.
