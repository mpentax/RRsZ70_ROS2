# Foxglove Studio Dashboard

Connect Foxglove Studio to **`ws://127.0.0.1:8765`** while `pixi run bringup` is running, and import [`z70-dashboard.json`](z70-dashboard.json).

## Tabs and Panels

- **Overview:** Map-fixed 3D view showing `/map`, room/zone/carpet markers (`/z70/map/markers`), robot/dock poses, and the full CAD URDF model, alongside `/z70/state` and `/z70/map/spatial_context`.
- **Odometry & Calibration:** `odom`-frame 3D view with `/z70/odom_path`, online chassis calibration metrics (`/z70/calibration/*`), and the top-down `/z70/floor_mosaic`.
- **Cameras:** Front and gripper compressed streams (`/camera/front/image_raw/compressed`, `/camera/gripper/image_raw/compressed`), `/z70/camera/state`, and optional monocular depth (`/camera/front/depth/points`).
- **Arm:** `base_footprint` close-up of the 5-DOF CAD arm and gripper with live `/z70/arm_estimate` joint and uncertainty plots.

## Regenerating `z70-dashboard.json`

`generate_dashboard.py` builds `z70-dashboard.json` from the Foxglove SDK layout schema (`0.59.0`):

```sh
curl -fsSL https://raw.githubusercontent.com/foxglove/foxglove-sdk/0e6c4b4e372854b3e0611268d5a676744f07dc36/python/foxglove-sdk/python/foxglove/layouts/__init__.py -o "${TMPDIR}foxglove_layout_types.py"
PYTHONPATH="$TMPDIR" pixi run python src/z70_bringup/foxglove/generate_dashboard.py
```

Layout structure and topic bindings are tested in `test_dashboard_layout.py` via `pixi run test`.
