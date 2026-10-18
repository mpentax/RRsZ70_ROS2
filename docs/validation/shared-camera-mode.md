# Shared camera mode: Claude's supervised live validation

## Scope / setup

The coding agent must not send `app_change_camera`, pickup or arm RPCs live. Those tests below belong to Claude in Michael's authorized supervised session. Stop only for an actual hazard or Michael's instruction; an acquisition/measurement gap is not a safety abort. No automatic camera restore, pickup entry, arm deployment or stow is part of camera acquisition.

Use the updated RoboClaw app, unchanged go2rtc, and the rebuilt ROS bridge. Restart only an existing bridge that belongs to your session; do not kill other publishers. If creating a validation bridge, isolate its ROS domain/port and close it afterwards. The source has been rebuilt with `pixi run test`, but an already-running Python bridge must be relaunched to load changes.

Use a fresh MCP tool list (`select_camera` must appear). `robot_rpc` is only needed for read-only `get_camera_status []` / `get_status []` and authorized setup; do not use raw camera writes when checking the select endpoint.

## Exact checks (Claude only for writes)

1. Read `robot_rpc({method:"get_camera_status",params:[]})` and `robot_rpc({method:"get_status",params:[]})`. Record raw `camera_status`, bit 15, state and `common_status`. A current gripper selection is valid; don't reset it merely for testing.
2. Warm the existing stream with `get_camera_frame({})`. Metadata must include `camera`, JPEG `dimensions`, and (known sizes) `sizeCamera`; no `defaultRestored`. Cross-check front = 1600x1200 / bit 15 clear, gripper = 1600x896 / bit 15 set. Save a local image under `research/cameras/live/` and visually identify the viewpoint.
3. Michael/Claude establishes pickup mode with arm visibly out using the already-authorized official-app/session setup. The known setup RPCs are `app_pick_up_enter_exit {flag:2}` and `app_arm_out_compartment`; these are **not** automatic camera prerequisites or commands for the coding agent. Observe rather than inferring arm deployment from status bits.
4. With front selected, call `select_camera({camera:"gripper"})` (HTTP equivalent: authenticated `POST /api/camera/select` with `{"camera":"gripper"}`). Expect `{"camera":"gripper"}` despite the missing selector ACK. The switch RPC reply budget is 500 ms, followed by up to ~3 s of bit confirmation; cold preview warmup can add driver startup time. Read `get_camera_status []` independently and confirm bit 15 is set. Do not replay a selector solely because its vendor RPC never replied.
5. Keep the 3 Hz bridge running and call `get_camera_frame({})` repeatedly without a camera argument for at least 10 seconds. All delivered frames should remain gripper with 1600x896 JPEGs. Independently read the status bit at the end: it must remain set. In Foxglove, `/camera/gripper/image_raw/compressed` and `/camera/gripper/image_raw` advance; `/camera/front/image_raw`, its compressed topic and `/camera/front/camera_info` do not advance. Measure delivered compressed-image cadence, not just the configured timer (target ~3 Hz warm).
6. Switch to front via `select_camera({camera:"front"})`. Confirm bit 15 clears, frames report front/1600x1200, and only front image topics and front CameraInfo now advance. Gripper topics stop advancing. Explicit `get_camera_frame({camera:"gripper"})` must also switch and stay; follow it with `{}` reads to verify no restore.
7. While the bridge keeps posting `{}`, change the camera through the official app. Read frames/status without a camera argument. New status/size tags and ROS topic routing must follow the app within approximately one status-cache interval (1 second, plus queued frames). During a transition, a bounded frame/status mismatch error is acceptable; mislabeled pixels or any corrective selector sent by default polling is not.
8. Once Michael has ended pickup/stowed the arm as part of normal authorized session cleanup, if front is active, call `select_camera({camera:"gripper"})` once. If firmware does not flip bit 15, expect a clear 409: not confirmed / gripper needs pickup mode with the arm out. A vendor rejection can return 409 immediately. Verify the bit stays clear. Do not deploy the arm or retry merely to eliminate this expected negative result.

## Evidence / adoption

Record request duration, select response, independent status bit, JPEG dimensions and camera tags, ROS message counts/stamps/cadence, and visual viewpoint. A tool listing or bit flip alone does not verify gripper pixels. No hidden side/back/depth streams or gripper calibration are claimed.

Adopt if timeout-with-bit-flip succeeds, no-flip is actionable, default bridge reads never change selection, app switches are reflected, frames/topics are correctly tagged, and front CameraInfo never accompanies gripper frames. Correct any mismatch from the first failing boundary; do not add automatic front restoration or app-mode guards.

End only validation processes you own. Leave the final camera mode/session state explicitly reported; switching to front, pickup exit or arm stow is an explicit supervised action, never camera-request cleanup. Verify `docker inspect roboclaw-app-1 --format '{{.State.Status}} restarts={{.RestartCount}} health={{.State.Health.Status}}'` still reports restarts=0.

## Coding-agent verification completed

- `npx tsc -p tsconfig.json --noEmit`: passed. `npm test`: 367 passed, zero failures.
- `pixi run test` in `ros2/`: 45 passed. Foxglove dashboard layout tests: 2 passed; existing gripper panel topic already matches, so no dashboard rewrite was needed.
- Rebuilt/recreated only the app with `docker compose build app && docker compose up -d --no-deps app`; readback: running, healthy, RestartCount 0. go2rtc and existing ROS processes were not restarted.
- Authorized read-only live smoke: `get_camera_status []` returned `[11685]` both before and after a `{}` frame request. Delivered JPEG: 40316 bytes, 1600x1200, metadata `camera:front`, `sizeCamera:front`; HTTP acquisition took 213 ms. No camera selector, pickup or arm command was sent live. The build process exited; no coding-agent process remains running.
- Live gripper switching, official-app changes and ROS routing/cadence across switches remain for Claude under the steps above. No git commit was made; pre-existing calibration/other untracked changes were preserved.

