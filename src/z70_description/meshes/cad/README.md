# CAD Meshes and URDF Stations

Package-installed STL meshes and [`../../config/cad_arm.yaml`](../../config/cad_arm.yaml) define the structural URDF geometry for `z70_description`. Mesh coordinates are in metres (identity scale in `z70.urdf.xacro`).

## Links and Parameters

| Asset / Parameter | URDF Link / Consumer |
| :--- | :--- |
| `cad_body`, `cad_wheels`, `cad_mops`, `cad_spinners`, `cad_lenses` | Chassis and underbody visuals on `base_link`. |
| `cad_bumper_front`, `cad_bumper_rear`, `cad_lid_base`, `cad_lid` | Bumper and hinged top-lid links. |
| `cad_shoulder_mount` | Shoulder turret visual on `shoulder_yaw_link` (under display `shoulder_lift` / `shoulder_yaw`). |
| `cad_link1`, `cad_link2`, `cad_hand` | `arm_link1`, `arm_link2`, and `arm_wrist_link` visuals. |
| `cad_jaw_left`, `cad_jaw_right` | Gripper jaw visuals; wrist-relative hinge origins `[0.0515, 0.0106, 0.0041] m` and `[0.0515, -0.0101, 0.0041] m`. |
| `L1_m = 0.1772`, `L2_m = 0.1157` | CAD `M3 -> M4` and `M4 -> M5` link lengths used by the URDF, `arm_model.hand_pose()`, and `planner`. |
| `shoulder_in_base_m = [0.1429, -0.0117, 0.1266]` | Shoulder pivot in `base_footprint`. |
| `camera_from_wrist_m = [0.0355, 0.0075, -0.0109]`, `camera_rpy = [0, 0, 0]` | Gripper camera pocket in the unrolled wrist frame (`+x` forward, `+z` top cover normal). |
| `marker_from_wrist_m`, `marker_rpy` | Underside `DICT_4X4_1000` `ID68` marker placement on the wrist housing. |

Collision boxes and finger swept-volume bounds come from [`../../config/pincher_geometry.yaml`](../../config/pincher_geometry.yaml) (`15 mm` clearance margin, `opening_ratio = 2/3`). See [`docs/z70-arm-model.md`](../../../../docs/z70-arm-model.md) for joint conventions and kinematics.

## Exported STL SHA-256 Digests

Reproduce from the repository root with `shasum -a 256 src/z70_description/meshes/cad/*.stl`:

```text
e5ffce9fb51ccee3d8140cab6116a69ece36f93d53a1ddbf034faa198439eaa5  cad_body.stl
b0fbf8b3a68f804a37cb7ef14357fcf68d5b8246899bd6d40652ce647eeb110c  cad_bumper_front.stl
a6d5b7b5b5550b39aa4ce4836144b0805e1d4fcd58ba2cc6634dc9c3d7b79b36  cad_bumper_rear.stl
d7c42a93fa26a4a6451257785d2b7a4f0b1f7ba9e8e5f55b2aeedf8e0dbb5332  cad_hand.stl
df5ea816515215dbd1b16e336de3048a733a1807f07a32bafd52fc5d1bf643c3  cad_jaw_left.stl
ef05b689606d36ade761af6cfb37e1d59a4f7c443c010aa518877d25328a1dfb  cad_jaw_right.stl
445744c62a723b5578f0516fe180a2396e90a8ac85debd3cbc2b1e7c9d102a95  cad_lenses.stl
32b991ba0325c0addf424ba60289f78724e2549fc59f3807311b807331fcbb9c  cad_lid.stl
76a2be2b1e5b7083cba4ab3d2ad76fae072e67290d5e7cd4c48d1a218cb4022f  cad_lid_base.stl
c582821b312ffb1ec08e3ac51ddaaeb7db3f6f180eed74dbcd0a8ad3176c8801  cad_link1.stl
5e537d04f87dd7550d5a319567957c0d06c7d948c67093830756ea77dc058e74  cad_link2.stl
83d6b1bd2b949cf7bd228e7a834cae707af9ecc74eca62789099a67e90099056  cad_mops.stl
3584cf24a17f1db20dcfadd75d6407a7da9b10d58330295cc45cbc66de7bdcf5  cad_shoulder_mount.stl
44ffa9a450c70d697e9775b76833884e77d0873ecd826c1151a556072f0f3cd5  cad_spinners.stl
7fbd621f40a93be2bfb6f0de245648e9757fb3ed15413462a5b89539693347ae  cad_wheels.stl
```
