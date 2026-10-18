# Z70 arm model (recovered from the Roborock Android app)

Source: Roborock app 4.74.08 plugin, `targeted-plugin/index.android.hasm` (Hermes bytecode disassembly) on the
`GOLD16` volume, arm-checker module **Function #32735** (lines ~1428440–1429890). It exports
`ForwardKinematics`, `InverseKinematics`, `checkArmInvalidAction`, `RecordRealPos`, `GetMaxStep`.

These are the **vendor app's predictive constants**. The app uses them to preview and validate programs before
sending them. They are internally consistent: inverse-kinematics constants are exactly L1+L2, L1², L2² and 2·L1·L2.
These are nominal vendor coordinates, not measured physical link stations. Executable copy: `src/z70_description/z70_description/arm_model.py`; the effective measured/provisional model is separately recorded in `src/z70_description/config/mount_calibration.yaml`.

## Geometry (2-D sagittal plane, metres; x forward, y up, y≈0 floor)

| Item | Value | Where |
|---|---|---|
| Shoulder pivot (M3) `m2_pos` | (0.0475, 0.14) | init `W` #32741 |
| Link 1, M3→M4 | **0.1775** | FK `B` #32739, IK `K` #32740 |
| Link 2, M4→M5 (`end_pos`) | **0.205** | FK `B`, IK `K` |
| Link 1 direction | 270° − M3 | FK |
| Link 2 direction | M4 − M3 + 90° | FK |
| Joint limits | M3 90–270°, M4 35–170°, M5 0–155° | #32735 |
| M6 (gripper) | 90 = open, 0 = closed | `q` #32745 |

`end_pos` is the wrist (M5) point. The app does not model the gripper length; M5 is not used for position.

## Validity checks (`j` #32744), in order
1. Joint limits → `FAULT_JOINT_LIMIT`
2. `end.y > 0.37` → `FAULT_ARM_HEIGHT`
3. `|end.x| < 0.24` and `end.y < 0.14` → `FAULT_ROBOT_COLLISION` (body envelope)
4. Otherwise `end.y < 0.01` → `FAULT_GROUND_COLLISION`; `|end.x| > 0.37` → `FAULT_PROJECTION`
5. `M4 − M3 + 90 > 15` → `FAULT_A4_ANGLE` (link 2 may not point more than 15° above horizontal)

## Named poses and state machine (`w` #32743, `b` #32742)

| State | M3 | M4 | M5 | M6 | Wrist point (x, y) | Entered by |
|---|---|---|---|---|---|---|
| `IN_ROBOT` (stowed) | 0 | 0 | 0 | 0 | collapsed placeholder | init, `ARM_IN` |
| `WORKING_POS` (deployed) | **150** | **45** | 0 | 90 | **(+0.157, 0.241)** | `ARM_OUT` from stowed; `ARM_PUTDOWN` with gripper closed |
| `TRANSFER_POS` | 150 | 75 | (kept) | 0 | (+0.157, 0.347) | `ARM_GRASP` with gripper open |
| `IN_PROCESS_POS` | (accumulated) | | | | | any `ARM_JOINTS` / `ARM_MOVE` |

## Step model (`q` #32745)
- `arm_joints` (program records `value:[M3,M4,M5,M6]`, `step:n`): first non-zero joint in M3→M4→M5→M6 priority;
  **±10° per repetition** (1 = +, 2 = −). M6: 1 → 90 (open), otherwise 0.
- `arm_move` (hand front/back/up/down): wrist point **±0.04 m** in x or y per repetition, then 2-link IK
  (`K` #32740), keeping the current branch.
- Checked after every repetition. The first failure is reported, and `maxStep` = valid repetitions so far.

## Implications
- The deployed wrist sits about 24 cm above the floor in this frame, and "hand forward" keeps the height constant. With the
  front camera at 68 mm, the arm can only enter the camera's view once it is lowered, or through the gripper hanging
  below the wrist. To be confirmed in the validation session.
- The app frame origin relative to the robot centre and to the front camera is **unknown**. The URDF puts it at the
  robot centre on the floor, and the camera x offset (0.16 m) is a placeholder.
- The manual-mode commands (`app_arm_move` M3/M4/M5 jog, `app_arm_direction_move` 1–4) may not use the same step
  sizes as program records. They must be measured separately.
