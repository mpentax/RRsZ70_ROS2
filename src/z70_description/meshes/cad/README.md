# Copied CAD assets

These package-owned STL bytes and [`../../config/cad_arm.yaml`](../../config/cad_arm.yaml)
are the runtime CAD representation. `setup.py` installs them in the ament package
share; URDF `package://z70_description/...` references do not import an original
Max/GLB/FBX file. Runtime coordinates are metres (URDF mesh scale is identity).
This establishes the exported coordinate convention, not original authoring units.

## Geometry and ownership

| Assets / values | Runtime frame and consumer |
|---|---|
| `cad_body`, `cad_wheels`, `cad_mops`, `cad_spinners`, `cad_lenses` | Visuals on `base_link`; white body, dark mechanisms. |
| `cad_bumper_front`, `cad_bumper_rear`, `cad_lid_base`, `cad_lid` | Separate visual links; lid has its own hinge. |
| `cad_shoulder_mount` | Visual on `shoulder_yaw_link`, under display `shoulder_lift`/`shoulder_yaw`; not a static body visual or upper-arm mesh. |
| `cad_link1`, `cad_link2`, `cad_hand` | `arm_link1`, `arm_link2`, `arm_wrist_link` visuals. The planning shoulder is separately fixed under `arm_vendor_frame`. |
| `cad_jaw_left`, `cad_jaw_right` | Visible jaws; wrist-relative hinge origins `[0.0515,0.0106,0.0041]` / `[0.0515,-0.0101,0.0041]` m. |
| `L1_m=0.1772`, `L2_m=0.1157` | M3→M4 and M4→M5 CAD stations; URDF, physical hand geometry and planner. The app's 0.205 m endpoint is separate. |
| `shoulder_in_base_m=[0.1429,-0.0117,0.1266]` | CAD shoulder in `base_footprint`. `shoulder_in_vendor_m` re-expresses it through retained arm-origin/yaw registration for the URDF parent frame. |
| `camera_from_wrist_m=[0.0355,0.0075,-0.0109]`, `camera_rpy=[0,0,0]` | CAD camera pocket in the unrolled wrist frame (+x forward, +z cover up); URDF and planner camera projection. |
| `marker_from_wrist_m`, `marker_rpy` | Owner-adjusted, photo-oriented CAD-face ID68 placement, not original CAD marker provenance; [retained placement record](../../../../calibration/physical-measurements/20261018-marker68-placement/README.md). |

Visible jaws and operational collision fingers have different roots/shapes. Collision
meshes/boxes and planner contact bounds remain photo-estimated proxies from
`pincher_geometry.yaml`, with 15 mm allowance and nominal opening ratio 2/3.
M6=0/90 is nominal closed/open; the URDF mimic multiplier is `2*opening_ratio`
against M6/2 drive coordinates. This is articulation, not validated linkage/force physics.

The CAD camera/published marker TF is not the older optical marker-to-hand
reconstruction or legacy terminal-region TCP. The [arm reference](../../../../docs/z70-arm-model.md)
identifies those active consumers and the independent recovered-app APIs.

## Asset provenance

The table above describes the installed geometry; hashes below identify its STL
exports. Re-exporting requires the original asset/version, authoring units,
node/pivot mapping and export transforms, which are not included in this package.

`cad_arm.yaml` also retains
`c3b3e6b4b733e022541b29cdb81c915d0bce597ef74535ed245edb0f9347a127`
with no recorded target; it is not an identified source or combined-export hash.
`resources_provenance.json` covers calibration copies, not CAD lineage.

Original CAD attribution and redistribution terms still need to be recorded;
the project's license alone does not establish rights to third-party assets.

### Exported STL SHA-256

These identify the retained exports, not export fidelity or physical accuracy.
From the repository root, reproduce with
`shasum -a 256 src/z70_description/meshes/cad/*.stl`.

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
