# Compact operational pincher correction — offline only

## Result and operational limitation

The exaggerated draft fingers are replaced by a compact, explicitly **PHOTO-ESTIMATED** shape. Finger size no longer depends on opening gain or an exact open-envelope fit. The broad wrist-aligned upper plate and opposed lateral roots remain; one shared geometry still supplies STL visuals, articulated URDF collisions, planner floor/sweep checks, observed front-servo aim, and the **actual executor's pre-close decision**.

**This is not calibrated CAD or a verified pickup.** The compact geometry exposes a real conservative-model limitation: with the retained15mm anatomy/pose uncertainty, no spherical target can pass the current robust contact-versus-housing rules. The unpatched localhost HTTP/JPEG integration therefore correctly refuses the66mm target before `app_grip`. No anatomy, target label, uncertainty, floor threshold or sweep margin was tuned to restore the old synthetic success. See the proof below.

No robot/camera/MCP, external network, Docker/service or main-preview access; no commit. The parent-owned archive README and unrelated dirty files were not edited. All22 integrated draft source/test/asset files remain part of this task; their shared planner/perception/executor integration was not reverted to the older baseline.

## Both owner photos and uncertainty

`paired-owner-evidence.png` and `photo-model-final-review.png` put both original views beside the final default-open model. Vision inspected the supplied pair, the rejected long-finger draft render, the regenerated open/half/closed renders, full-arm context and the camera close-up.

| Owner evidence | Supported qualitative observation | Not identified |
|---|---|---|
| `IMG_0648_87aeb8.png`, default top | Wide flat longitudinal plate continuing the wrist; compact bent fingers of the same general order as head width, emerging from opposite sides | Exact dimensions, opening angle, hidden linkage/gain, joint stations |
| `IMG_0647_4ec758.png`, rolled underside | Opposed root hardware; local ID68 backing; small underplate cavity | Gravity-up from screen-up, exact pad seam, lens size, concealed anatomy |

Original SHA-256 values, rechecked by the verification script:
- Underside: `83c593205a76be125f210bf333815f6fc997608f525e68ff43022fdf58e9f6cb`
- Top: `5397e7641dee6e197f0d099be780d89c5620bcef5bfa87d32da00eeabbd8e556`

The `.png` originals contain HEIF bytes; only separate `decoded-*` PNGs were used for OpenCV. The15mm black ID68 /20mm paper dimensions remain registration evidence. Neither perspective view was treated as an orthographic ruler. The nominal35–45mm forward-reach /50–60mm curved-shape scale and roughly90–110mm open terminal span are **qualitative design estimates**, not measured photo confidence intervals. Head width60mm and root offsets±26mm also remain photo estimates, not newly surveyed values.

## Root cause and TDD

The previous draft computed finger length by forcing the rough177.8mm open envelope exactly, dividing by `cos(nominal open angle)`. Increasing its assumed M6 gain to0.85 stretched the fingers to78.46mm forward reach /89.39mm full sampled centreline, with192.49mm open terminal separation. That is rejected by both photos. It was also entangled with synthetic66mm pickup success, which is not anatomy evidence.

1. **RED** `compact-proportions-red.txt`: a real retained-STL regression failed on78.4578mm longitudinal reach, against a loose55mm compactness upper bound. A companion65mm centreline bound rejects the89mm exaggeration without pretending to measure the photos precisely.
2. **GREEN** explicit42mm forward reach replaces the inverse-cosine solver; meshes/collision xacro regenerated. A gain-independence regression varies nominal gain through0.5,2/3,0.85,0.999999 and1.0 and verifies the *same* anatomy, with no near90deg length blow-up.
3. **RED** `lens-size-red.txt`: the draft had shrunk the previous illustrative2.8mm-radius lens to1.2mm to fit the aperture. The test catches that; the original nominal2.8mm lens is restored, with its masking honestly retained rather than relabelled calibration.
4. `compact-first-suite.txt` records the resulting real66mm integration refusal before fixture expectations were corrected. No closure-success mock or hidden smaller executor target was introduced.

The earlier `anatomy-red.txt`, `planning-red.txt`, `executor-red.txt` and `before/` remain retained history from the integrated draft; the earlier sibling `model-update/` was not overwritten.

## Final generated dimensions

Numbers below come from `verification.json`; they are model outputs, **not measurements extracted from photos**.

| Quantity | Discarded integrated draft | Final compact model |
|---|---:|---:|
| Finger forward reach |78.4578mm|42.0000mm|
| Full sampled centreline, including proximal root tail |89.3940mm|55.5554mm|
| Pivot-to-tip centreline |—|53.6654mm|
| Nominal angle per finger at appM6=90 |76.5deg|60deg|
| Nominal open terminal-surface gap |192.49mm|98.8461mm|
| Closed terminal station in elbow gauge |212.7578mm|176.3000mm|
| Open longitudinal envelope |177.8mm, forced exactly|177.7301mm, independent cross-check|

The retained owner rough open envelope is177.8mm. Its≈0.07mm numerical agreement with the final shape is **not** a measurement residual or calibration accuracy claim. The envelope regression now uses a loose5mm rough-survey allowance instead of a1micrometre equality requirement.

- Shared source: `ros2/src/z70_description/config/pincher_geometry.yaml` and `z70_description/pincher_geometry.py`; units metres.
- Head remains60mm maximum width with broad upper z9mm, matching the existing wrist-cover top. Lateral roots remain elbow-gauge `(134.3, ±26, 3)mm`; the head was not inflated to disguise long fingers.
- **Nominal articulation only:** bridge appM6/2 drive coordinates are unchanged. Explicit mimic multiplier4/3 yields60deg at appM6=90,30deg at45 and0deg at0. This is a plausible open illustration, **not a calibrated M6 coupling** recovered from two stills. Actual hidden linkage is unknown.
- Closed opposing tips leave a nominal0.2mm surface gap and do not cross at tested M6=0/15/45/75/90. Hidden root-slot overlap, pad shape, forces and deformation remain unvalidated.
- Distal-half contact regions and the10mm-ahead operational aim remain provisional. The aim automatically follows the compact geometry; it is not the historical measured marker-relative terminal surrogate or an exact calibrated TCP.
- The original nominal2.8mm-radius lens remains at the unchanged optical transform behind the smaller illustrative aperture. Its rim is partly masked; this is not a new lens-size or full-FOV calibration. `camera-cavity-close-up.png` shows the actual small central recess/lens at defaultM5=0. The white underside rectangle is the local marker backing, **not a second camera**. Tests prove optical-forward-ray escape only, not the full off-axis cone.

## Shared operational behavior preserved

- `articulated_geometry` returns finger frames, terminal points, contact regions, nominal gap and complete body proxies for four-angle states. Two-angle states meanM5=0,M6=90. Invalid/nonfinite input is rejected.
- One conservative housing box fills the optical recess; nine convex segment boxes per finger retain the open gap. These exact proxies feed URDF collision generation and planner queries. Real STL-vertex containment and actual published TF are tested.
- Firmware bounds, **MIN_TIP10mm**, **15mm provisional uncertainty**, **1.5mm closure inter-sample allowance**, and≤0.5deg joint-edge sampling remain unchanged. Full housing/finger floor checks, intermediate M5/M6 sweeps and optional per-piece world-AABB refusals remain active. This is not complete chassis/self-collision or acquired-scene obstacle avoidance.
- Historical `(250,95)` remains recorded but unsafe: final complete-hand floor bound5.54mm despite the older point-surrogate bound27.24mm. A model-derived open approach/equator goal selects `(250,105)` for the default fixture. It has a valid16-step BFS route from `(150,45)`; this is a **positioning route, not a closure-authorized grasp**.
- Front perception still transports the full observed marker pose into the hand frame; its operational aim and the executor's differential servo both use shared compact anatomy. Full optical/marker rotations and the historical surrogate stay distinct and unchanged.
- Immediately before `app_grip`, `Executor.pick` still reacquires a same-frame observed marker/object sample and calls `observed_closure`. It enforces open-gap exclusion, housing/non-pad clearance, the full floor-safe closing sweep, and both pads reaching at a **common** nominal opening. This remains in the real executor, not an unused helper.

### Why no robust sphere passes the default model

Across all91 nominal openings used by `closure_check`, the maximum distance of **any pad-proxy corner** from the housing box is24.7083mm. Convex distance over each convex box is bounded above by its vertex maximum. For a target sphere of radiusR, robust pad reach requires distance≤R−15mm; housing exclusion requires distance>R+15mm+1.5mm. Triangle inequality therefore requires pad/housing separation>31.5mm, independent ofR or any rigid hand pose. The model's maximum is smaller.

This proves a **current conservative-model/uncertainty blockage**, not physical inability of the real robot to grasp. Merely choosing a smaller target or another nominal hand pose cannot fix it. Resolution needs genuinely better identified anatomy/observed uncertainty or justified collision/contact modelling; it must not be faked by retuning anatomy, shrinking the66mm target label or weakening the retained margins.

### Genuine synthetic integration and positive classifier scope

`offline-http-pick-timeline.jsonl` is the actual **75-row localhost-only** HTTP/JPEG run from the final isolated-artifact suite through target acquisition → ID68 pose reconstruction → safe descent → two fresh observed servo confirmations → final geometry refusal → stow cleanup. No robot is involved. In that scene:
- Model state `(250,105)`; observed operational aim `(429.18, −0.47, 30.70)mm`.
- Target equator `(417.29, −0.088, 33.00)mm`; horizontal error≈11.89mm and vertical error≈2.30mm satisfy unchanged servo tolerances.
- Observed whole-hand floor bound21.21mm passes the retained clearance requirement.
- Closure refuses `target intersects open housing/fingers`; **no `app_grip`, lift or put-down is sent**. No successful default end-to-end pickup is claimed.

The positive contact-classifier unit fixture is explicitly a **known32mm synthetic sphere with zero input geometry/pose error**, hand translationz80mm and target centre from shared aim `(-18.7,0,83)mm` in that synthetic base. Both pads reach it at the same nominalM6=29; full nominal sweep floor bound74.5mm. This exercises actual contact logic rather than mocking success. The exact same example is rejected with the unchanged default15mm uncertainty. Zero error is an analytic test input only—never executor configuration, a relabelled66mm object, or hardware validation.

Ambiguous transport/no replay, two explicit lift commands, held/floor-gone sensing and release/stow settling remain separately exercised by localhost component tests starting from an explicitly synthetic already-held/transport scene. They are **not** represented as a geometry-authorized full pickup. The actual executor's post-close implementation was retained, not replaced with mocks or removed.

## Verification

- **Final:126 passed, 3 unchanged strict optical-accuracy xfails in23.59s**, full `~/.pixi/bin/pixi run test --basetemp build/pytest-compact-final` from `ros2/`; all9 currently present packages built. Log:`ros-suite-final.txt`. Historical12px optical criteria were not weakened.
- The initial pincher suite passed128 tests in24.02s with7 packages. During final verification, unrelated concurrent workspace work added `z70_interfaces`/`z70_driver` and changed the suite count. One shared-directory run timed out and is retained as `ros-suite.txt`, not a passing final result. A separate direct retry passed126 tests in25.21s; the final complete Pixi build/test rerun above passed in its own artifact directory. No concurrent packages/tests were reverted or edited to recover the earlier count.
- Separate real isolated `ROS_DOMAIN_ID=91` robot_state_publisher check: **4 passed in0.83s**, open/intermediate/closed and rolled four-angle finger frames match shared geometry; log `isolated-finger-tf.txt`.
- Deterministic four-STL/collision regeneration, manifoldness/normals/positive volume, optical-axis ray, local marker support, retained-mesh compactness, collision containment, installed package asset bytes, body/floor/closing-sweep/outside-gap/housing tests all pass. The tightened interior-sweep fixture keeps both endpoints above the1.5mm allowance (lower endpoint2mm) yet fails at interiorM6=86; assertions check the specific floor reason, not an unrelated earlier rejection.
- `verification.json`: all13 non-M6 joint definitions compare equal to retained `before/`; `mount_calibration.yaml` is byte-identical toHEAD; source-photo hashes unchanged; fourSTLs, shared config and collision xacro match the installed package share.
- `verification-run.txt`:1000 four-angle pose-query performance probe and16-step BFS result; timings are software measurements, not hardware guarantees.
- `git diff --check` is clean.

## Retained visual evidence

All are **offline expanded-URDF renders**, not Foxglove screenshots:
- `photo-model-final-review.png`: both actual owner views beside final open geometry, defaultM5=0.
- `before-after-open.png`, `before-after-half-open.png`, `before-after-closed.png`: top/underside/side with wrist cover, final AFTER replacing the rejected elongated draft. BEFORE remains the older retained baseline, not the intermediate89mm draft.
- `roll-comparison.png`:M5=0/40/90; roll appears only when commanded in the nominal model.
- `full-model-context.png`: full arm/body context at nominal `(200,95)` and `(250,105)`,M5=0. Body outline/high-level pose and unmeasured motion marks are provisional, not mechanically certified.
- `camera-cavity-close-up.png`: depth-buffered default small lens/recess detail; partial rim masking remains visible, no fabricated extra eye/pod.

Independent vision review of the combined actual-photo/final-model sheet judged finger compactness, broad wrist-aligned plate and side roots qualitatively consistent. Open/half/closed and full-context renders have no obvious missing or frame-clipped parts; closed tips approach without visible crossing. Remaining discrepancies: faceted/angular head and root geometry, schematic fasteners, approximate bend/taper/tips, dark partially obscured cavity and nominal lens rim. No exact photo dimensions or actualM6 dynamics were established.

## Reproduce from `ros2/`

```sh
~/.pixi/bin/pixi run python src/z70_description/meshes/generate_pincher.py
~/.pixi/bin/pixi run test --basetemp build/pytest-compact-final
~/.pixi/bin/pixi run bash -c 'source install/setup.bash; python ../calibration/physical-measurements/20261015-pincher-underside/operational-model-update/verify_offline.py'
~/.pixi/bin/pixi run bash -c 'source install/setup.bash; python ../calibration/physical-measurements/20261015-pincher-underside/operational-model-update/render_comparison.py'
```

Exact22 task source/test/asset paths: `changed-files.json`. Report scripts, logs, final URDF/verification and visual evidence are in this `operational-model-update/` directory. The prior sibling `model-update/`, original attachments and parent-owned archive README were not overwritten. The default closure limitation remains an unresolved modelling/calibration requirement, not a hidden successful-grasp claim.

## Parent deployment and isolated verification

The parent reloaded only its owned visualization-only publisher at `ws://127.0.0.1:8765`. The actual bridge returned the refreshed latched description and all four complete STL assets byte-for-byte equal to source (`foxglove-assets-readback.json`, `live-description.urdf`). All13 non-M6 joint definitions exactly match the pre-correction baseline; the live description contains19 hand collision elements. The pose remains simulated M3=150/M4=45/M5=0/M6=90, not robot telemetry. No robot connection or camera was opened.

Concurrent unrelated ROS-driver migration modified the same working-tree executor/client and fixture. Only hand-specific hunks were staged; the migration's Client/observation/readiness changes and test-provider injection were left untouched and unstaged. An isolated copy of the exact hand-only source index, with three retained non-secret baseline test fixtures copied locally, passed the complete original seven-package suite: **128 passed, 3 unchanged strict xfails in23.54s**. The parent’s combined nine-package working-tree run separately reported **128 passed, 3 xfailed, 1 failure** in the newly added unrelated driver evidence-identity test. That failure was not silently reclassified as a passing whole-workspace result. The earlier snapshot setup/import/fixture failures were resolved before the final isolated passing run.

The Node bridge verifier received and compared all complete assets successfully, then reported a close-handshake error; this is distinct from failed geometry or missing asset delivery. The offline render is not a browser screenshot. Independent review passed for the provisional shared model and fail-closed integration, not for working hardware grasping. The reviewer reproduced the uncertainty blockage and reran11 geometry tests successfully. The source-hash manifest now identifies the reviewed hand-only Git index rather than the concurrent working tree. No validated operational pickup is claimed.
