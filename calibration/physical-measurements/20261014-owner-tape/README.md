> **Supersession pointer — 2026-10-18.** Original photo hashes, owner landmark explanations and dated implementation receipts below are unchanged. Current anatomy and consumers are documented in [the CAD-backed arm model](../../../docs/z70-arm-model.md); bent-tape exterior envelopes are not surveyed joint-centre lengths.

# Owner tape-measure photographs — retained originals

These original photographs are retained in the project, outside Hermes session/composer caches and scratch cleanup. Keep them: they are physical reference evidence, not disposable trial captures. PNGs are git-ignored for privacy; this is a local archive, not a remote backup.

## Files and integrity

- `IMG_0638_746ea6.png` — annotated pincher/wrist and lens locations; 4,527,277 bytes; SHA-256 `364819db6dc841cff43199080402dc33a294e6caceb0b3a5958bef089d843bf8`.
- `IMG_0641_ef94b3.png` — base-to-elbow segment; 2,347,612 bytes; SHA-256 `86d9b6788fb1c3192d5797471911910ed2080a7da15a143e554b63a9f80dd86b`.

Copied from `~/Library/Application Support/Hermes/composer-images/`; source and destination hashes matched on 2026-10-15.

## Owner explanation (session 20261013_093607_1cd339, message 11176)

The hook is tied to the pincher end, with the tape wrapped around the wrist joint. Pincher/wrist separation is around the 2.5-inch reading; open pincher plus wrist is about 7 inches (177.8 mm). The other photograph shows the base-to-elbow segment, also about 7 inches. The tape bends around housings: these are rough assembly measurements, not precise axis-to-axis distances. The pincher lens is recessed underneath the pincher.

## Foxglove geometry audit — 2026-10-15

Read the actual latched `/robot_description` from the running ROS publisher, not only source files. No robot command, camera capture or restart performed.

Live visual dimensions:
- `arm_link1`: 177.3567846500386 mm (6.983 inches), consistent with the first-segment tape estimate.
- `arm_link2`: 205 mm (8.071 inches), still the unmeasured vendor coordinate gauge.
- `arm_wrist_link`: an additional 50 mm (1.969 inches), placeholder.
- Each finger: an additional 70 mm (2.756 inches), placeholder, hinged 50 mm beyond the nominal wrist.
- Elbow to closed visual finger tip: 325 mm (12.795 inches). At a nominal 45-degree finger opening, longitudinal tip extent is 304.5 mm (11.988 inches). These are URDF visual extents, not measurements of the robot.

The owner's roughly 178-mm second assembly is NOT represented by these appended visual boxes. Source jaw calibration uses a separate effective closing-point vector of [0.165, 0.0011888505920638046, 0.02] m from the elbow; it did not shorten the drawing.

The running publisher is also stale for `gripper_tcp`: live wrist-relative x=+0.00795563691400302 m (elbow-relative x=0.21295563691400302 m); current source has wrist-relative x=-0.04 m (elbow-relative x=0.165 m). A source edit does not reload the running robot description.

Do not fix this by blindly changing vendor L2: camera and marker mounts currently compensate for that coordinate gauge. A visual correction should preserve calibrated camera/marker/contact transforms and separate rough physical display anatomy from nominal vendor wrist coordinates. No simulation/control files changed in this audit.

## Offline source correction — 2026-10-15

`ros2/src/z70_description/urdf/z70.urdf.xacro` now draws the second assembly within a **177.8 mm longitudinal open envelope**, rather than appending housings and fingers after the 205 mm vendor gauge. This implements the owner's approximate exterior measurement; it is not new axis metrology.

| Display component | Source geometry | Status |
| --- | --- | --- |
| First segment | 177.3567846500386 mm | Existing calibrated fit retained; consistent with rough 7-inch tape |
| Second assembly, elbow to open tip | 177.8 mm including box thickness, nominal M6=90° (±45° fingers) | Rough owner measurement used as display target |
| Proximal second-assembly box | 114.3 mm | Provisional 177.8−63.5 split, not physical L2 |
| Terminal housing span | 20 mm | Display placeholder within the terminal 63.5 mm, not an extra extension |
| Finger box | 57.518289963230 mm long, 8 mm wide | Derived display shape so the open envelope, including corners, reaches 177.8 mm |
| Closing-point surrogate | x=165 mm, y=1.188850592064 mm, z=20 mm from elbow at M5=0 | Existing pickup registration retained; **not** the open visual tip and not a tape-measured pad centre |

The 2.5-inch reading is a rough hook-to-housing landmark. Using its 63.5 mm as a display split does not establish a physical wrist axis or finger hinge. The provisional fingers extend to 191.818289963230 mm when drawn closed; no closed-tip measurement was supplied, so this extent and the M6 opening mechanism remain unvalidated. Widths, thicknesses, hinge gaps, and the shoulder housing are still illustrative.

### Why the calibrated transforms do not move

- The nominal `arm_wrist_link`/M5 frame remains at x=205 mm from the elbow. The physical M5 axis station remains **unknown** (`physical_M5_axis_station_m: null`), as do physical L2 and M4 zero.
- The wrist drawing and finger hinges move backwards **along link2 x**, inside the tape-sized assembly. Translation along the M5 roll axis commutes with its rotation: the shortened drawing rolls with the same camera/TCP chain at nonzero M5 without claiming a newly measured pivot.
- Camera, marker, and contact calibration vectors and rotations are unchanged. The marker is a calibrated registration, not a separately published URDF frame. Its base pose is checked through the actual optical TF.
- Firmware FK in `arm_model.py` and all command/planning code are unchanged. The effective 165 mm contact surrogate remains distinct from the 177.8 mm open drawing.
- `mount_calibration.yaml` links these original photographs and this README, and labels the visual-only dimensions and unknown physical axes explicitly.

### Offline verification

TDD regression first failed in all three M5 cases (0°, 35°, 155°): the old expanded box-corner envelope was **307.325901807805 mm**, not 177.8 mm. The earlier audit's 304.5 mm was the rounded centreline tip; including the rotated finger box's half-width explains the difference.

After correction:

- Physical-display regression: **4 passed**, including retention of calibrated L1.
- `~/.pixi/bin/pixi run test src/z70_description` from `ros2/`: **18 passed**.
- `~/.pixi/bin/pixi run test src/z70_description src/z70_manipulation` from `ros2/`: **63 passed**.
- `~/.pixi/bin/pixi run test` from `ros2/`: **103 passed**.
- Real, isolated `robot_state_publisher` TF checks retain the nominal wrist chain and compare camera, registered marker, and TCP across three M3/M4 poses at M5=0°, 35°, and 155°. The 155° cases are algebraic consistency checks, not an extension of the measured 0–40° roll validation.

No robot/camera access, Docker, deployment, or existing live-process changes were performed during offline implementation.

## Parent deployment verification — 2026-10-15

The previous ROS bringup, state publisher and Foxglove process had exited before deployment. Started **only** a local `robot_state_publisher`, Foxglove on `ws://127.0.0.1:8765`, and a clearly named `/z70/model_preview/joint_states` publisher with a **simulated** deployed/open pose (M3=150, M4=45, M5=0, M6=90). This preview does not connect to RoboClaw, query cameras, command the robot, or claim a physical robot pose.

Read back the live latched `/robot_description` and actual TCP TF. Verified expanded open visual envelope **177.8 mm**, live wrist-relative TCP `[-0.04, 0.0011888505920638046, 0.02]` m, and available base-to-TCP transform. Port 8765 is listening on loopback. Independently reran the full build/test command: **103 passed in 23.89 s**. No full camera/control stack was restarted.
