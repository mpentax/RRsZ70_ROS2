# Front-camera calibration

## Current consumers

Normal ROS bringup and floor projection use the installed `z70_description` resources, not a writable checkout archive:

- [front_camera.json](../../src/z70_description/config/front_camera.json): retained original lens K/D with the later **provisional** floor registration.
- [z70_front_camera_info.yaml](../../src/z70_description/config/z70_front_camera_info.yaml): front ROS intrinsics; the rejected joint front-lens refit was not adopted.
- [mount_calibration.yaml](../../src/z70_description/config/mount_calibration.yaml): coherent optical TF/floor alignment and launch mount offset.
- [Current arm model](../../docs/z70-arm-model.md): CAD shoulder/anatomy and separate optical registration/proxy consumers. The older optical-preview shoulder is not today's structural geometry.

The floor update came from [the pincher-session front registration](../arm-calibration/20261015T080103Z-pincher-intrinsics-arm/arm/front-registration.json). Its [dated preview derivation](../arm-calibration/20261015-provisional-model/README.md) retains the prior active JSON and failure/uncertainty record. Print flatness/scale, lens extrapolation and physical registration remain systematics; coherent TF/projector inputs do **not** certify mapping or manipulation accuracy.

After building/installing and sourcing the workspace, load the owning package resource rather than hard-coding this archive:

```python
from z70_description.resources import config_path
from z70_description.floor_geometry import FrontCamera
cam = FrontCamera.load(config_path("front_camera.json"))
# Floor origin: directly below the camera; x forward, y left, z up.
p = cam.pixel_to_floor(800, 700)  # None for rays not intersecting the forward floor
```

## Historical one-frame baseline — preserved report

**All measurements, range/uncertainty claims and improvement advice in this section describe the original one-frame floor model, not current-projector accuracy.** The historical floor extrinsics were superseded; the original lens was retained. No new physical validation is implied.

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

## Safe baseline reproduction

The baseline scripts use their own `__file__` directory, not the current working directory, for outputs. `export.py` writes `front_camera.json` beside itself plus validation/overlays; merely changing directory or supplying another input image does **not** isolate it. Copy code and evidence first, then execute the copies. The following replays fitting/export from the **retained detected corners**. Raw re-detection is historical: the original OpenCV 5.0 receipt is not reproduced by current Pixi OpenCV 4.13.0 (the dense board returned “no corners detected” in an isolated check). Do not replace retained detections with an empty/changed result. From repository root, with the installed Pixi environment and `~/.pixi/bin` on PATH:

```sh
SCRATCH_ROOT="${TMPDIR:-$HOME/.cache}/rrsz70-replays"
export TMPDIR="$SCRATCH_ROOT"
mkdir -p "$SCRATCH_ROOT"
S=$(mktemp -d "$SCRATCH_ROOT/z70-front-baseline.XXXXXX")
cp -R calibration/front-camera "$S/front-camera"
pixi run python -B "$S/front-camera/fit.py"
pixi run python -B "$S/front-camera/export.py"
pixi run python -B "$S/front-camera/uncertainty.py"
```

The private raw frame and retained `out/detections.json` must be present locally. Historical raw detection used `detect.py raw/frame-3-telegram.jpg` in the old environment; it is not a verified current regeneration command. All new JSON/images remain in scratch; preserve originals by hash, and move desired outputs into a new durable archive, never onto adopted package resources. The old OpenClaw/DimOS `~/.openclaw/workspace/dimos-v1/native-slam/.module-venv/bin/python` invocation was historical environment provenance, not a current dependency or installation command.

Boards: carlosmccosta/charuco_detector black-and-white A4. One is 5×7 squares, 40/30 mm (modern layout);
the other is 10×14 squares, 20/15 mm (**legacy** layout). Both use DICT_4X4_1000; ids overlap, so each board is detected inside its own image region.
Images (`raw/`, `out/*.jpg`) are git-ignored.
