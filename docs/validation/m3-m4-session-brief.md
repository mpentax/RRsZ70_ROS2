> **Historical session record — not current operating instructions.**
> The original body below is preserved verbatim, including dated commands,
> evidence and session-specific authorization. Old paths, numerical models and
> data-gap stop rules are not current guidance or standing hardware permission.
> Current: [ROS guide](../ros2.md), [API contract](../thin-api.md),
> [arm model](../z70-arm-model.md), [shared cameras](shared-camera-mode.md),
> [passive chassis calibration](chassis-odometry-trial.md),
> [manipulation/measurement practice](../../src/z70_manipulation/README.md).

---

# Brief: M4 then M3 calibration + supervised estimator check (GPT session)

You are running a supervised robot session with Michael present. Your own plan, approved with additions:
`calibration/arm-validation/M3_M4_PLAN.md`. Read it, `docs/z70-arm-model.md`, and the README "Arm pose estimate" section.

## Additions to your plan (approved by Claude)
1. **Use RoboClaw's estimator as the simulator.** RoboClaw now dead-reckons the arm from every arm RPC
   (`mcp_roboclaw_get_arm_estimate`, `mcp_roboclaw_set_arm_anchor`). Before deploying, confirm with Michael
   that the arm is physically stowed, then `set_arm_anchor {anchor:"stowed"}`. After every command, log
   `get_arm_estimate` (joints, wrist, sigmaDeg, state) next to your marker measurement. That makes this session the
   end-to-end check of the estimator too. If its state turns UNKNOWN, note why (history field) and stop motion.
   Your earlier local simulator stays as a cross-check only.
2. **Stationary noise first.** At the reference pose take 3 settled pairs before any jog. Last time stationary frames
   differed by about 6°. A 10° step is only resolvable if that's smaller. If the 40 mm square fits flat on the gripper, use
   it instead of the 20 mm one (twice the pixels → much steadier orientation).
3. **No chassis commands.** Arm only; the robot stays where it is. Claude works on startup/chassis code in parallel but
   won't run any trial while you are controlling the robot.
4. **Finish with a handoff line.** End the session by stowing, calling `set_arm_anchor {anchor:"stowed"}` after
   Michael confirms, and writing `calibration/arm-validation/M3_M4_RESULTS.md`. It should contain per-joint/sign tables
   (measured vs estimator), noise, coverage gaps, and an explicit "robot released" line with a timestamp.
   Commit results (no images).

## Rules (unchanged)
MCP tools only for the robot; never read `.local/` secrets; don't add gates or format-pinning tests; stop on unclear
outcomes, unexpected motion, lost target or animals in the workspace; no retries of ambiguous moves.

## First turn
Do read-only preflight only: `refresh_status`, `get_arm_estimate`, one `get_camera_frame`. Report robot state
(programming mode? docked?), the estimate, and what the camera sees. Then ask Michael to confirm:
- the arm is stowed;
- the marker is attached (which size);
- the area is clear.

Then stop and wait for his reply. Don't move anything in the first turn.
