# Foxglove Studio Dashboard

Connect Foxglove Studio to **`ws://127.0.0.1:8765`** while `pixi run bringup` is running, and import [`z70-dashboard.json`](z70-dashboard.json).

## Tabs and Panels

- **Overview:** Map-fixed 3D view showing `/map`, room/zone/carpet markers (`/z70/map/markers`), session-scoped world object markers (`/z70/world/markers`), garment markers (`/z70/garments`), robot/dock poses, and the full CAD URDF model, alongside `/z70/state`, `/z70/map/spatial_context`, and `/z70/world/objects`.
- **Cameras:** Front and gripper compressed streams (`/camera/front/image_raw/compressed`, `/camera/gripper/image_raw/compressed`), `/z70/camera/state`, `/z70/objects/json`, `/z70/garments/observations`, and `/camera/front/metadata`.
- **Arm:** `base_footprint` close-up of the 5-DOF CAD arm and gripper with live `/z70/arm_estimate` joint and uncertainty plots.
- **Odometry & calibration:** `odom`-frame 3D view with `/z70/odom_path`, online chassis calibration metrics (`/z70/calibration/*`), and the top-down `/z70/floor_mosaic`.
- **Diagnostics:** `/diagnostics` summary and detail panels, battery history (`/z70/battery`), and last RC command age (`/z70/last_command/age_s`).

## Regenerating `z70-dashboard.json`

`generate_dashboard.py` builds `z70-dashboard.json` directly without external dependencies:

```sh
pixi run python src/z70_bringup/foxglove/generate_dashboard.py
```

Layout structure and topic bindings are tested in `src/z70_driver/test/test_dashboard.py` via `pixi run test`.
