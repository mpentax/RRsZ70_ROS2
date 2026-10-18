# Z70 front camera: lens + floor-distance calibration


## Active provisional floor update — 2026-10-15

`front_camera.json` now keeps the **original lens K/D** below but uses the new-run best front-board registration for its **PROVISIONAL floor extrinsics**, synchronized with ROS optical TF: height **58.233433936mm**, pitch-down **−1.358765985°** (slightly up), roll **−0.415796432°**. The robot-floor heading is optical-forward projected into the observed plane, not arbitrary board yaw. This supports a self-consistent physical preview with shoulder height126.430291524mm; rough owner lens60mm/shoulder127mm are crosschecks, not hard constraints. Print scale/flatness, pose/glare and retained front-lens systematics remain unresolved. **Not precision-certified mapping or manipulation geometry.**

Source: `calibration/arm-calibration/20261015T080103Z-pincher-intrinsics-arm/arm/front-registration.json` (`T_F_W`,1332residuals,3.2425px RMS). Exact derivation and limitations: [`20261015-provisional-model/README.md`](../arm-calibration/20261015-provisional-model/README.md). The original active JSON, including its68.355862410mm floor height, is preserved byte-for-byte in [`prior-front-camera.json`](../arm-calibration/20261015-provisional-model/prior-front-camera.json) with an adjacent SHA-256 note. Front `z70_front_camera_info.yaml` is unchanged; the rejected fcyk1 intrinsics were **not** adopted.

**The remainder is the historical one-frame baseline report. Its floor-height, horizon, metric validation and range-error table are NOT current-projector accuracy claims.** `export.py` reproduces that historical baseline and would overwrite the active provisional floor; run only into an explicitly separate reproduction/archive, not as an active-model update.

From **one** frame (frame-3, two A4 ChArUco boards flat on the floor in front of the dock,
1600×1200 stream), fitted jointly with both boards constrained to the same floor plane.

## Historical one-frame result (original lens retained, floor extrinsics superseded)

| Quantity | Value (bootstrap 1σ) |
|---|---|
| Focal length | **823 ± 3 px** (fx = fy) |
| Horizontal / vertical / diagonal FOV | **88.3° ± 0.2°** / 72.2° / 101° |
| Principal point | (777, 598), centre would be (800, 600) |
| Distortion (OpenCV radial) | k1 = −0.074 ± 0.006, k2 = −0.009 ± 0.003, mild barrel |
| Lens type | Ordinary rectilinear wide-angle; **not fisheye**. Standard pinhole + `plumb_bob` works. |
| Camera height above floor | **68.2 ± 0.4 mm** (check against a tape measure) |
| Pitch | ≈ 2.6–2.9° down (± 0.7°), roll ≈ 0° |
| Floor horizon | image row ≈ 560 at the centre column |

Since this is a standard pinhole + radial model, no custom stack is needed: `cv2.undistort`, ROS
`camera_info`/`image_proc`, or `floor_geometry.py` (numpy only) all consume it directly.

## Validation

- **Physical consistency check (not fitted on purpose):** the solver independently places the second
  board **210.7 mm** beside the first, rotated **0.2°**. Both 200 mm grids are centred on 210 mm-wide A4
  sheets laid side by side, so 210 mm / 0° is what it should be. Without distortion it gets 204 mm / −6°.
- **Held-out metric test:** fit on one board, then measure the *other* board's printed squares on the floor
  with `floor_geometry.pixel_to_floor`. Median pairwise distance error is **1.1–1.6 mm (1.3–2.0 %)**, p95 4–5 mm,
  at 8–32 cm range. With no distortion correction: 7–13 mm (10–16 %).
- Reprojection: 2.05 px RMS, 2.3 px on held-out corners (5-fold), over 92 corners (17 + 75).
  `out/floor_overlay.jpg` shows range rings; `out/undistorted.jpg` shows the original vs the corrected image.

## How far you can trust floor distance

The camera sits 68 mm above the floor, so range per pixel grows ~quadratically with distance:

| Range | Image row | 1 px = | calibration 1σ |
|---|---|---|---|
| 15 cm | 925 | 0.4 mm | 1 mm |
| 30 cm | 746 | 1.6 mm | 1 mm |
| 50 cm | 673 | 4.5 mm | 2 mm |
| 1 m | 617 | 18 mm | 13 mm |
| 2 m | 589 | 69 mm | 65 mm |
| 3 m | 579 | 152 mm | 158 mm |

**Up to ~1 m: good metric distance. 1–2 m: coarse (±5–10 %). Beyond ~2.5 m: direction yes, distance no.**
That limit comes from physics (camera height), not from this calibration.
Points must actually touch the floor: the base of a chair leg, not the seat.

## Limits

- Single image, single plane: intrinsics are solid for this purpose but come from one viewing geometry.
  The boards covered rows 758–1188 only, so the upper image's distortion is extrapolated.
- Pitch is fitted with the robot **docked**. If the body tilts off the dock (carpet, bumps, wheel load),
  the horizon shifts and far ranges shift with it. A cheap per-frame fix is to re-estimate pitch from a visible floor feature.
- The image is a Telegram re-encode; the original was a stream frame, so the geometry is the same.
- Calibrates the delivered stream, which may include firmware processing, not the bare lens.

## Improving it (optional, ~5 min, no robot motion)

The camera is fixed, so the board has to move. Hold the 40 mm board at 3–4 tilts and positions,
including the upper half of the frame, and grab ~10 frames. Run `detect.py <img>` per frame plus a multi-view fit.
This would pin down the principal point and upper-image distortion. Floor distance doesn't need it.

## Use

```python
from floor_geometry import FrontCamera
cam = FrontCamera.load("calibration/front-camera/front_camera.json")
cam.pixel_to_floor(800, 700)   # {'forward_mm': ~401, 'left_mm': -11, 'range_mm': ~401, 'bearing_deg': -1.6}
cam.floor_to_pixel(500, 0)     # where a floor point 50 cm ahead appears
```

Floor frame: origin on the floor directly under the camera, x forward, y left, z up.

## Reproduce

```sh
PY=~/.openclaw/workspace/dimos-v1/native-slam/.module-venv/bin/python   # opencv 5.0, numpy, scipy
cd calibration/front-camera
$PY detect.py raw/frame-3-telegram.jpg   # -> out/detections.json, out/detections.jpg
$PY fit.py                               # model comparison -> out/fit_results.json
$PY export.py                            # -> front_camera.json, out/validation.json, overlays
$PY uncertainty.py                       # bootstrap -> out/uncertainty.json
```

Boards: carlosmccosta/charuco_detector black-and-white A4. One is 5×7 squares, 40/30 mm (modern layout);
the other is 10×14 squares, 20/15 mm (**legacy** layout). Both use DICT_4X4_1000; ids overlap, so each board is detected inside its own image region.
Images (`raw/`, `out/*.jpg`) are git-ignored.
