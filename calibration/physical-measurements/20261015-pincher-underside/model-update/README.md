> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [current arm model](../../../../docs/z70-arm-model.md), [camera/calibration guide](../../../../research/cameras/FINDINGS.md). Optical identifiability limits below apply to this experiment; copied CAD now supplies structural anatomy, not independent physical registration or contact validation.

# Pincher housing visual update — offline, provisional

## Result

Replaced separate finger bars, box head/lower camera enclosure and blue external pincher camera block with a coherent rounded dark housing and actual recessed camera pocket. Two opposed tapered, gently hooked blades emerge through short broader root sections on the upper face. The underside adjoining plate supports the existing full-pose marker plane. The narrow proximal wrist transition overlaps the existing link2 cover rather than expanding its whole length. The front RGB camera is unchanged.

The mesh is an illustrative exterior, not surveyed production CAD. Implementation was offline, without robot access, camera acquisition, Docker or control-service changes. The parent subsequently reloaded its visualization-only Foxglove preview and verified the latched description and actual bridge asset delivery, as recorded below.

## Photo → feature mapping

Reference: `../IMG_0647_4ec758.png`, verified SHA-256 `83c593205a76be125f210bf333815f6fc997608f525e68ff43022fdf58e9f6cb`.

The photo was inspected with vision, respecting the owner's sideways orientation. Image-up was never used as robot-up. The established black marker side is 15 mm; paper is approximately 20 mm. No pixel ratio was promoted to a 3D measurement.

| Observed feature | Implemented surface |
|---|---|
| One rounded shared black housing, not a separate camera pod | One connected watertight shell, rounded side transitions and narrower proximal cover junction |
| Camera pocket beside underside flat plate | An actual open recess with annular rim, tapering interior walls and back wall; no filled solid behind a painted dark mask |
| Small optical element inside pocket | 5.6 mm illustrative glass diameter, front surface 0.25 mm behind the unchanged optical origin; shell opening 6.5 mm forward of it |
| Flat marker-bearing region adjoining pocket | Broad underside plane follows full retained marker tilt with approximately 0.8 mm backing clearance; marker TF and paper/black-square visuals unchanged |
| Fingers from opposite face/edges | Existing opposed M6 frame locations with short broad roots attached at the upper plate; slimmer flattened blades taper and bend inward near their tips |
| Blunt rounded finger ends | Rounded cross-sections and tapered terminal stations, not long straight box bars |

The exact hidden upper surface, wall thickness, lens diameter/setback, root linkage construction, physical M5 station, mechanical closing contacts and full 3D blade curvature remain unmeasured. Small metallic hinge details were deliberately not invented. Root sections are simplified solid supports, not a reconstructed internal linkage/window. No optical or contact-surrogate frames were moved to make anatomy appear plausible. The existing nominal closed M6 pose still leaves a visible gap; this update does not claim calibrated physical fingertip contact.

## What changed

- `ros2/src/z70_description/urdf/z70.urdf.xacro`: pincher visual geometry/materials only. All links, joints, joint origins, axes, limits, parent/child topology and calibrated marker/camera/TCP transforms preserved.
- `ros2/src/z70_description/meshes/generate_pincher.py`: deterministic offline numpy/stdlib generator using the existing calibration values read-only.
- Four installed binary STL files: `pincher_shell.stl`, `pincher_lens.stl`, `pincher_finger_left.stl`, `pincher_finger_right.stl`.
- `ros2/src/z70_description/setup.py`: installs `meshes/*.stl` under the package share directory.
- `ros2/src/z70_description/test/test_pincher_mesh.py`: real triangle connectivity/manifold/normals/volume, cavity and optical ray, marker visibility/backing, slender curved fingers, installed package assets and reproducible generation.
- `ros2/src/z70_description/test/test_physical_visuals.py`: evaluates actual mesh vertices as well as boxes for the existing owner-envelope constraint.

No calibration YAML, floor optical TF, planner, bridge or runtime source was changed. L2=205 mm remains the nominal M5 coordinate gauge. Negative link-local shell coordinates are intentional.

## TDD and verification

- `red.txt`: new shape contract failed on the original box model (`arm_wrist_link must use sculpted mesh geometry, not separate box blobs`). Then passed on actual generated shell geometry.
- `asset-red.txt`: installed URI test failed before rebuilding because `share/z70_description/meshes/pincher_lens.stl` did not exist. Setup installation and colcon rebuild made byte-for-byte package lookup pass.
- `finger-red.txt`: thicker first mesh iteration failed the thin-blade geometric requirement (10.748 mm blade z-envelope). Slimmer blades and tapered rounded ends then passed.
- Final `~/.pixi/bin/pixi run test`: **111 passed, 3 xfailed**, 7 packages rebuilt. Existing expected failures retained; complete command output is in `ros-suite.txt`.
- All four meshes are closed edge-manifolds with positive signed volume, non-degenerate triangles and stored normals matching triangle winding. Shell is a single connected component. Generator reproduces the exact binary mesh bytes in an isolated test directory.
- Optical +X forward ray from the retained lens centre crosses **no shell triangles**. A backward ray meets the pocket back wall at **3.000 mm**; radial rays meet actual cavity walls; a ray outside the opening hits the front rim. This is geometry, not an optical/calibration change.
- A grid across the retained marker paper region has clear outward rays and a nearby supporting plate behind it. Marker is neither buried nor floating far from the housing.
- Expanded-URDF before/after comparison: **15 joint definitions exactly equal**, **16 link frames**, **54 closed/open M3/M4/M5 combinations**, maximum all-link homogeneous-transform difference **0.0**. This proves internal kinematic invariance, not new physical metrology.
- Actual vertex-based open longitudinal envelope: **177.800001 mm** at M5=0°,35°,155°, unchanged within STL float precision. This is the retained rough owner assembly constraint, not micrometre-level physical accuracy.

`verification.json` retains poses, numeric checks, mesh counts/volumes/SHA-256 and photo SHA. Shell: 2304 triangles; lens: 384; each finger: 1920.

## Retained visual proof

`pincher-multiview.png` renders actual URDF mesh/box triangles in four orthographic z-buffer views: upper/open, underside/marker, distal/camera pocket and closed articulation. Neutral presentation lighting is used for shape readability; this is not a Foxglove screenshot or a photo reconstruction. Scale bars show generated mesh coordinates, not a photo-derived survey. A first painter-sorted render falsely fragmented the marker; per-pixel depth buffering corrected that rendering artifact. The final render was inspected with vision: it reads as a single rounded-edge housing, clean flat marker square, opposed slender bent blades and genuinely recessed lens. Remaining visible simplifications include faceting, broad root supports and blunt tips.

Reproduce from repository `ros2/`:

```sh
bash -c 'source install/setup.bash && ./.pixi/envs/default/bin/python src/z70_description/meshes/generate_pincher.py'
~/.pixi/bin/pixi run test
bash -c 'source install/setup.bash && ./.pixi/envs/default/bin/python ../calibration/physical-measurements/20261015-pincher-underside/model-update/verify_model.py'
bash -c 'source install/setup.bash && ./.pixi/envs/default/bin/python ../calibration/physical-measurements/20261015-pincher-underside/model-update/render_meshes.py'
```

`before.urdf` is the captured pre-edit source expansion; `after.urdf` is the current expansion. Original owner archive README/image were not modified. PNGs remain locally retained and Git-ignored; this is not remote backup.

## Parent deployment handoff

Installed package URIs tested: `package://z70_description/meshes/{pincher_shell,pincher_lens,pincher_finger_left,pincher_finger_right}.stl`. Installed mesh bytes and offline tests alone do not certify Foxglove browser delivery.

### Parent deployment verification

The parent rebuilt/tested independently: **111 passed, 3 expected optical-accuracy failures**. Only its owned local simulation preview was restarted; no robot or camera connection was opened. The refreshed latched `/robot_description` was read through the actual Foxglove WebSocket at `ws://127.0.0.1:8765`; all 15 joint definitions exactly match the pre-change description. Four `fetchAsset` responses delivered the complete STL files, each byte-for-byte equal to source, with valid binary STL lengths. See `live-description.urdf` and `foxglove-assets-readback.json`.

The bridge requires the `foxglove.sdk.v1` WebSocket subprotocol; legacy-only negotiation failed. The Node test client reported a close-handshake error after successful full asset delivery, recorded separately from geometry/readback assertions. This is not a claim that the Foxglove browser rendering was inspected directly. The retained multi-view image is an offline rendering of the same meshes. The preview remains a simulated deployed/open pose, not live robot telemetry.
