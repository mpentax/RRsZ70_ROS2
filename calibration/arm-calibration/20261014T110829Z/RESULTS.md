> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [current arm model](../../../docs/z70-arm-model.md), [camera/calibration guide](../../../research/cameras/FINDINGS.md). Optical identifiability limits below apply to this experiment; copied CAD now supplies structural anatomy, not independent physical registration or contact validation.

# New live arm calibration — blocked by missing target

## Outcome
A new live acquisition was performed, but **zero usable metric frames** were obtained. Neither the front camera nor the gripper camera sees the dense ChArUco board or wrist ID0 marker. The live scene is a rug, yellow object, mop, bags and cart; the retained earlier sweep scene contains a floor ChArUco board, wooden panels and turquoise bottle. This is a real setup/context mismatch, not a failed PnP fit. No chassis command was sent.

## Acquired evidence
- 5 preserved JPEGs: 3 front, 2 gripper, plus detection overlays/JSON.
- Every frame has zero ChArUco corners and zero decoded ArUco markers.
- Deployment from owner-confirmed stow completed; common_status became 51458 (arm-ready bit set).
- Exactly one arm jog: app_arm_move {m:[1,0,0]}, nominal (150,45,0) -> (160,45,0), 3 seconds settle.
- Gripper-view optical flow median 109.83 px confirms image movement. It does not metrically confirm a 10-degree angle. No refused jog was counted and no compensating jog was sent.
- All RPC/capture/estimate evidence is in log.jsonl. No grip or lift was issued.

## Fits and planner
Shoulder position, height, L1, L2, joint offsets/gains, camera mounting, sigmas, reprojection RMS and jaw closing point are **not measurable from these frames**. They are null in fit.json. No changes were made to planner.py or its tests; existing provisional geometry remains provisional. Jaw positions at (240,105)/(230,95) and a 3.3-cm-equator approach sequence cannot responsibly be supplied.

Even with a varied camera trajectory, freely fitting both L2 and the camera translation along link2 has an exact gauge ambiguity: camera positions depend on L2+camera_along. M4 zero offset likewise trades against camera mount orientation/translation when that mount is unconstrained. A independently located wrist axis or known marker-to-wrist transform is needed to report physical L2 and joint zero separately, rather than only an effective link2-to-camera vector.

## Verification / cleanup
`cd ros2 && ~/.pixi/bin/pixi run test src/z70_manipulation`: **27 passed in 12.35 s**; 7 ROS packages built successfully.
Stow sent using app_arm_in_compartment []; after 15 seconds status was state 39, error_code 0, battery 52%, common_status 51202 (arm-ready bit cleared, matching initial stowed status). Service estimate read back IN_ROBOT, zero joints, null wrist; camera selected front and final frame preserved. These are status/model evidence, not encoder feedback.

## Resume prerequisites
Restore robot/board placement so the dense legacy 10x14 board (20-mm squares / 15-mm markers, DICT_4X4_1000) is visible ahead of the front camera and below the gripper camera; confirm the rigid wrist ID0 marker is still attached. No need for another permission gate: existing calibration authorization covers resuming after this setup correction. Then collect independently varied M3/M4 poses and front-camera jaw views before changing constants.

If measured geometry changes, src/arm-model.ts, ROS z70_description arm_model.py, URDF geometry and mount_calibration.yaml must be reviewed together with planner.py. None were modified during this blocked run. No commit made.

## Files created
- This dated directory: log.jsonl, five raw JPEGs, five overlays, five observation JSON files, fit.json, RESULTS.md.
- calibration/arm-calibration/live.mjs: MCP-only incremental acquisition helper (not a blind sweep; caller must inspect each jog before issuing another).
- calibration/arm-calibration/observe.py: legacy dense ChArUco and ID0 detection, PnP when target exists, optical-flow comparison and overlays.
