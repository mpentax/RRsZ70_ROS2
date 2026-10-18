# Foxglove dashboard

Connect Foxglove to **ws://127.0.0.1:8765** after authorized observation bringup,
and import [`z70-dashboard.json`](z70-dashboard.json). The installed copy is
`share/z70_bringup/foxglove/z70-dashboard.json`. Reimport after regeneration;
editing a repository file does not change an already imported layout.

## Panels and evidence

- **Overview** is map-fixed and follows the robot; vendor `/map`, room/zone/carpet
  markers and robot/dock snapshot poses surround the canonical URDF model.
  `map → odom` is a single-pose visual alignment, not current vendor localization.
  Robot status reads `/z70/state`; spatial context is raster-derived, not live
  obstacle clearance.
- **Odometry & calibration** uses odom, the deduplicated `/z70/odom_path`, typed
  quality/calibration scalars and the running-mean `/z70/floor_mosaic`. The mosaic
  is appearance, not occupancy. Command-model fallback has explicit non-VO quality.
- **Cameras** shows front/gripper compressed topics and shared camera state.
  These two selectable streams do not establish physical camera count. Inactive
  lens compressed frames/CameraInfo/metadata remain latched: inspect stamps and
  lens selection before treating visible pixels as current. Both lenses have
  installed calibration. Depth stays empty when disabled/unavailable.
- **Arm** has a base_footprint close-up with joint/uncertainty plots. All eight
  `/joint_states` drive one canonical robot_state_publisher. A retained deployed
  UNKNOWN pose can still be drawn; planning requires ArmEstimate typed validity/
  joints, not displayed TF or fresh visualization stamps.

REP-103 is x-forward/y-left/z-up; optical axes are x-right/y-down/z-forward. The
Overview map/pose visualization headers refresh about every two seconds; source
cache receipt is separate in load_time/status. Image receipt time is not exposure.

## Verification

Offline: `pixi run test` from the repository root includes layout integrity and
serialized typed dashboard/diagnostic checks. Passing it is not proof of a current
rendered Foxglove scene or physical calibration.

`validate_live.py` is a live acquisition harness, not part of offline testing.
It observes for about 65 seconds in domain 96 / Foxglove port 8876, without a lens
switch or manipulation goal. It gives bringup private HOME/XDG/ROS/cache/temp
paths, uses installed calibration, and bounds cleanup to its own process group,
including children whose launch leader has exited. An unexpected bringup exit
fails validation even if topics were received. Successful shutdown removes private
scratch; failed or interrupted validation retains it for inspection. User arm/chassis
mutable state is isolated; this is not a zero-write tool.

It intentionally writes `bringup-validation.log` and `live-validation.json` in
this directory. Preserve any prior receipt separately before an authorized run;
do not overwrite evidence that must be retained. Built ROS packages and optional
depth interpreter/cache are prerequisites, not downloaded by the validator.
The live command (not executed by this documentation update) is:

```sh
pixi run bash -c 'source install/setup.bash && python src/z70_bringup/foxglove/validate_live.py'
```

`test_validate_live.py` exercises child-environment wiring, private write targets
and bounded cleanup with fake ROS inputs and harmless isolated subprocesses; it
does not prove real topic delivery, worker availability or shutdown under hardware
load. Changing only Z70_REPO or DDS domain would not isolate XDG-backed state.

For independent image-rate measurement use reliable depth-10 subscriptions on
native DDS; use RELIABLE + TRANSIENT_LOCAL for late TF-static/description readers.
Only stop processes created by the validation, never unrelated publishers.

## Regenerate with the pinned schema

`generate_dashboard.py` uses Foxglove SDK layout dataclasses (schema 0.59.0),
pinned to commit `0e6c4b4e372854b3e0611268d5a676744f07dc36`. JSON import needs no Python
SDK installation. From the repository root:

```sh
curl -fsSL https://raw.githubusercontent.com/foxglove/foxglove-sdk/0e6c4b4e372854b3e0611268d5a676744f07dc36/python/foxglove-sdk/python/foxglove/layouts/__init__.py -o "${TMPDIR}foxglove_layout_types.py"
PYTHONPATH="$TMPDIR" pixi run python src/z70_bringup/foxglove/generate_dashboard.py
```

The generator writes the checked-in dashboard. Legacy exported panel IDs `3D`,
`DiagnosticSummary`, `DiagnosticStatusPanel` map to ThreeDee/DiagnosticsSummary/
DiagnosticsDetail. Keep frame IDs, CameraInfo links and world-unit line widths
consistent with the generator, not manual edits to one imported copy.

## Historical validation receipt

The original pre-cutover run/browser episode below is evidence only, not a
permanent current login failure or present validator-isolation guarantee.

<details>
<summary>Original validation and UI observation</summary>

## Validation

Current offline command from the repository root: `~/.pixi/bin/pixi run test` (includes layout integrity and typed dashboard/CDR tests). The measurements below and `live-validation.json` are historical pre-cutover evidence, not new live qualification. The retained `src/z70_bringup/foxglove/validate_live.py` requires separate live acquisition authorization.
The bounded validator uses domain 96 / port 8876, reads real service telemetry and front frames, subscribes to every layout topic plus TF/joints, copies calibration into scratch before launching bringup, and stops its own process group. It never dispatches robot commands or edits the live calibration file. It writes `live-validation.json` and `bringup-validation.log`; report `null` scalar samples represent ROS NaN.

Validated: all nonoptional referenced topics delivered messages, including URDF, static/dynamic TF, map, room markers, odom/path, images and diagnostics. The four optional camera feeds and depth were not publishing in this run. Diagnostic CDR round-trip tests cover native Jazzy's byte-valued severity field.

**UI verification remains open:** the browser tool could not use this machine's Safari default profile. A separate clean Chromium session reached Foxglove's sign-in page, not a dashboard. `foxglove-access.png` was vision-checked and confirms the login wall; no imported layout or rendered robot panels were observed. The exported JSON uses real upstream config schemas, but a signed-in Foxglove import/render check is still required.


</details>
