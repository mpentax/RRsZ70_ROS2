# ID68 pincher placement correction

## Adopted visualization transform

`src/z70_description/config/cad_arm.yaml` now places `pincher_marker_68` relative to `arm_wrist_link` at:

- xyz `[0.024, 0, -0.0141]` m (was `[0.014, 0, -0.0138]`).
- RPY `[3.2195592874213355, 0, pi/2]` rad (was `[pi, 0, pi/2]`).

The 10mm distal shift uses Michael's approximate 1cm correction and the retained underside photo's placement toward the camera/finger end, not a newly measured metric sticker position. The small Z adjustment seats the paper outside the CAD underside. The CAD face slopes `dz/dx=-0.078125`; the extra roll is `atan(0.078125)` (~4.467deg). It represents the sticker lying flat on that CAD face, as Michael confirms for the physical sticker. The face inclination relative to the physical roll axis is not independently surveyed. Lateral offset remains zero; perspective does not establish a defensible lateral correction.

## Evidence and limitations

- `../20261015-pincher-underside/IMG_0647_4ec758.png`, decoded copy `../20261015-pincher-underside/operational-model-update/decoded-IMG_0647_4ec758.png`: sticker on underside, distal of collar, proximal to camera recess, aligned lengthwise. OpenCV decoded ID68, with printed marker +Y distal.
- `../20261015-pincher-underside/IMG_0648_87aeb8.png`: anatomical top/distal reference, sticker hidden.
- `src/z70_description/meshes/cad/cad_hand.stl`: face slope and surface seating are CAD-derived, not photo metrology.
- Owner's stated ~1cm discrepancy supplies approximate adjustment magnitude; the photograph supplies direction. Exact xyz is provisional and should be visually checked by the owner.

No lens, shoulder, TCP, historical optical fit, perception implementation or robot joint estimates changed. The old free registration in `mount_calibration.yaml` differs from this CAD-flat registration and remains historical measured evidence. Existing perception's `marker_hand_pose()` still uses that older fit, so this display correction does **not** silently recalibrate manipulation.

## Rotation interpretation for front-camera calibration

Let `R_F_M` be a detected marker orientation in the front-camera optical frame, with OpenCV's decoded corner convention. Let `R_W_M = Rz(pi/2) @ Rx(pi+atan(0.078125))`, the adopted CAD mount.

Then the inferred pincher orientation is:

```
R_F_W = R_F_M @ R_W_M.T
```

Use the full SO(3) rotation, not a difference of image-plane/Euler angles. Translation does not enter this equation. Marker +Y is approximately distal along the pincher's underside; marker +Z is the outward underside normal. A fixed marker mount cancels for camera-frame relative rotations: `R_F_M2 @ R_F_M1.T`. Absolute pincher orientation inherits the provisional physical-to-CAD mounting assumption; this update is not a fresh front-camera optical calibration or a physical roll-zero validation.

## Verification

- New placement regression failed on old X=14mm; corrected model passes.
- 43 description tests pass, including real robot_state_publisher/tf2 across multiple joint poses and flat-mount rotation inversion.
- Installed z70_description rebuilt using normal (non-symlink) colcon build. Existing copied install resources reject symlink overlay; no live processes were restarted.
- Updated only `/robot_state_publisher`'s `robot_description` parameter. Exact parameter readback, latched `/robot_description`, and updated wrist-to-marker `/tf_static` verified. All other joints, links and materials compared unchanged before applying.
- Before/after expanded URDF and hash/TF readback retained alongside this README.
- Full ROS source suite: **299 passed, 3 xfailed** (103.43 seconds). The three expected failures remain expected; no live robot access.
- No robot motion, live camera acquisition, Docker or service restart.
