# Z70 · Foxglove dashboard

1. From the repository root, run `~/.pixi/bin/pixi run bringup` only within separately authorized live acquisition (matching backend must already be running).
2. Open Foxglove desktop or <https://app.foxglove.dev>, sign in if prompted, then **Open connection → Foxglove WebSocket** → `ws://127.0.0.1:8765`.
3. **Layouts → Import from file…** → select `src/z70_bringup/foxglove/z70-dashboard.json`, or the installed `share/z70_bringup/foxglove/z70-dashboard.json`.
4. Switch between **Overview**, **Cameras**, **Arm**, **Odometry & calibration**, and **Diagnostics**. The URDF loads from `/robot_description`; TF places it in `odom`.

This is an observation layout, with no teleoperation or command panels. For another machine, use an SSH tunnel to the bridge's localhost port. The web app may require browser permission to reach localhost.

## Topics and semantics

The current typed `/z70/state`, `/z70/arm_estimate`, `/z70/commands`, calibrated camera topics and TF are described in `docs/ros2.md`. Scalar presentation is owned by `z70_driver`, not a second HTTP poller. It consumes typed calibration/odometry/dispatch evidence and publishes once per second:

| Topics | ROS type / meaning |
|---|---|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray`: connection, robot, arm estimate, last RC command, calibration, visual odometry; errors and stale sources have explicit levels |
| `/z70/battery`, `/z70/status_age_s` | `std_msgs/Float32`: percent and seconds since the service's observed status |
| `/z70/state_code`, `/z70/error_code` | `std_msgs/Int32`; missing code is −1 |
| `/z70/state_name`, `/z70/error_name`, `/z70/connection_state` | `std_msgs/String`; unknown enum codes retain their number |
| `/z70/cleaning`, `/z70/returning`, `/z70/charging`, `/z70/connected`, `/z70/status_valid` | `std_msgs/Bool`; cleaning/returning reflect task flags, not motor activity; charging means vendor state 8, not merely docked/full |
| `/z70/last_command/{age_s,method,outcome}` | Float32/String/String; latest typed driver dispatch event, numeric enum values retained. Never verified physical completion |
| `/z70/arm/{state,unknown_reason}` | String; explicit UNKNOWN reason |
| `/z70/arm/{m3_deg,m4_deg,m5_deg,m6_deg,steps_since_anchor,sigma_m3_deg,sigma_m4_deg,sigma_m5_deg,sigma_m6_deg}` | Float32; app-degree dead-reckoned estimates and uncertainty, **not encoder feedback** |
| `/z70/calibration/{yaw_positive_gain,yaw_negative_gain,forward_gain,camera_x_m,camera_x_sigma_m,active_camera_x_m,samples,vo_accepted,vo_rejected}` | Float32; fit values, active VO camera offset, sample count, accepted/rejected frame counts |
| `/z70/calibration/camera_x_ready`, `/z70/calibration/vo_state` | Bool/String |
| `/z70/odom_path` | `nav_msgs/Path`: last 2,000 odometry poses; resets if the odometry frame changes |

Unavailable or stale floats are **NaN**, not invented zeroes. Missing task flags are not published as false. Check `/diagnostics` and `/z70/status_valid` before interpreting retained discrete states. `UNKNOWN` and `IN_ROBOT` joint plots intentionally have no numeric samples: folded/finger placeholders use `/z70/display/joint_states`, not the planning stream. M6 has no measured uncertainty. Active VO offset comes from each typed frame-pair status, not a later calibration estimate, and remains NaN when unknown. Forward gain remains unknown until suitable observations exist.

### Mapping and optional camera topics

Overview includes `/map` (OccupancyGrid), `/z70/map/markers` (room MarkerArray), `/z70/map/robot_pose` (PoseStamped), `/z70/map/dock_pose` (PoseStamped, green dock-reference arrow), and `/camera/front/depth/points` (PointCloud2). The 3D camera starts at an 8 m viewing distance to show the apartment map and surrounding rooms rather than only the robot. Mapping supplies `map → odom`; the display frame stays `odom` so odometry and the model also work without a map. Vendor map poses are snapshot references, not proof of fresh localisation.

The **Spatial context** Raw Messages panel beside Robot status in **Overview** renders `/z70/map/spatial_context`, exposing the mapping spatial-context summary without requiring interpretation of the 3D view.

**Odometry & calibration** places the **Floor mosaic** Image panel below the odometry trail. It renders `/z70/floor_mosaic` as `sensor_msgs/msg/Image` (`bgr8`, frame `odom`): the floor-mosaic node's running-mean floor appearance, not the vendor occupancy map. The panel stays empty when the mosaic publisher is unavailable. The odometry-only 3D view retains its closer 4.5 m camera distance and hides the dock/map overlays.

The **Cameras** tab pairs `/camera/front/image_raw/compressed` (`1600×1200`) and `/camera/gripper/image_raw/compressed` (`1600×896`) with an active-lens indicator (`/z70/camera/state.selected_camera`), `/z70/objects/json` (live 2D/3D object detections), `/z70/camera/state`, and `/camera/front/metadata`. The **Arm** tab includes a close-up 3D URDF arm view (`1.1 m` distance, `base_footprint` display frame) alongside joint estimate and uncertainty plots. Depth is an optional mapping node; the cloud layer stays empty when that node is disabled/unavailable.

## Validation

Current offline command from the repository root: `~/.pixi/bin/pixi run test` (includes layout integrity and typed dashboard/CDR tests). The measurements below and `live-validation.json` are historical pre-cutover evidence, not new live qualification. The retained `src/z70_bringup/foxglove/validate_live.py` requires separate live acquisition authorization.
The bounded validator uses domain 96 / port 8876, reads real service telemetry and front frames, subscribes to every layout topic plus TF/joints, copies calibration into scratch before launching bringup, and stops its own process group. It never dispatches robot commands or edits the live calibration file. It writes `live-validation.json` and `bringup-validation.log`; report `null` scalar samples represent ROS NaN.

Validated: all nonoptional referenced topics delivered messages, including URDF, static/dynamic TF, map, room markers, odom/path, images and diagnostics. The four optional camera feeds and depth were not publishing in this run. Diagnostic CDR round-trip tests cover native Jazzy's byte-valued severity field.

**UI verification remains open:** the browser tool could not use this machine's Safari default profile. A separate clean Chromium session reached Foxglove's sign-in page, not a dashboard. `foxglove-access.png` was vision-checked and confirms the login wall; no imported layout or rendered robot panels were observed. The exported JSON uses real upstream config schemas, but a signed-in Foxglove import/render check is still required.

## Schema sources / regenerating

Panel configs were generated with [Foxglove SDK's layout dataclasses](https://github.com/foxglove/foxglove-sdk/blob/0e6c4b4e372854b3e0611268d5a676744f07dc36/python/foxglove-sdk/python/foxglove/layouts/__init__.py) (schema version 0.59.0), using the established exported-layout envelope (`configById`, mosaic, Tab). Legacy export IDs `3D`, `DiagnosticSummary`, `DiagnosticStatusPanel` correspond to canonical `ThreeDee`, `DiagnosticsSummary`, `DiagnosticsDetail` panel types. [Foxglove layout docs](https://docs.foxglove.dev/docs/visualization/layouts), [3D docs](https://docs.foxglove.dev/docs/visualization/panels/3d), and [an exported tab/diagnostics example](https://github.com/BonsaiRobotics/roscon2024_health_sample/blob/main/roscon2024_health_foxglove.json) establish the format.

The historical SDK did not include `foxglove.layouts`; no SDK install is needed to import JSON. To regenerate with the checked schema, from the repository root:

```sh
curl -fsSL https://raw.githubusercontent.com/foxglove/foxglove-sdk/0e6c4b4e372854b3e0611268d5a676744f07dc36/python/foxglove-sdk/python/foxglove/layouts/__init__.py -o "${TMPDIR}foxglove_layout_types.py"
PYTHONPATH="$TMPDIR" ~/.pixi/bin/pixi run python src/z70_bringup/foxglove/generate_dashboard.py
```
