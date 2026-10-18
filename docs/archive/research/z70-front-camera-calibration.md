> **Historical / superseded — retained in place.** Current owners: [ROS guide](../../ros2.md), [command/API contract](../../thin-api.md), [arm model](../../z70-arm-model.md), [shared-camera guidance](../../validation/shared-camera-mode.md), [calibration entrypoint](../../../calibration/front-camera/README.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

The no-calibration finding and private DimOS/venv command describe the original preflight only. Current installed calibration and reproducible-method guidance are linked above; true target scale, pose diversity and held-out validation remain useful.

External HASM/editor inputs below are private-owner provenance, unavailable from the public checkout; full original replay needs the authentic matching inputs. No private files were opened or copied during this curation.

## Historical record

# Z70 fixed-front camera calibration preflight

2026-10-07. Local/public read-only research; no camera/robot connection, motion, credential access or service changes.

## Finding

No trustworthy Z70-specific camera matrix or distortion calibration for the actual 1600×1200 preview was found in the inspected material. This is a bounded negative finding, not proof the manufacturer has no factory calibration. The successful live diagnostic establishes raster shape/cadence, not intrinsics, exposure timing or geometry.

- Connector `src/camera-observation.ts` defines measured calibration contracts; its tests explicitly use synthetic values, not Z70 calibration.
- Existing real datasets remain `calibration_only`/unmeasured. Do not relabel these to bypass dataset validation.
- Retained `deployment/a4-trial-1407/final.jpg` was visually inspected privately: the near portion of the plain page is clipped by the bottom image boundary, so all four paper corners are unavailable. Earlier report documents little pose diversity and unresolved video/snapshot timing differences. These images cannot establish a robust full lens calibration.
- A fully visible rectangle at multiple distinct tilts can theoretically constrain planar calibration; one blank A4 at essentially one orientation with clipped corners cannot identify complete intrinsics/distortion reliably. A blank page is not categorically impossible, but is much weaker than a many-corner target.
- Installed SDK `include/cuvslam/cuvslam2.h` Camera explicitly requires focal lengths in pixels, principal point and distortion parameters. No upstream automatic intrinsic-estimation option was identified in the inspected API. Mono map support does not mean self-calibrating camera support.
- Pinned DimOS already provides `dimos cameracalibrate`: maintained OpenCV checkerboard calibration, folder/webcam/topic input, pinhole and fisheye models. Do not write another custom solver.

## Public-source checks

Managed web search is disabled. DuckDuckGo HTML fallback returned relevant manuals/manufacturer links; no matching calibration numbers. Manufacturer support page was read successfully and exposes manuals/troubleshooting, not lens matrices. Manufacturer product page yielded title-only extraction; no substantive calibration inference is made from it. OpenCV docs fetch encountered HTTP 403; local upstream documentation plus native source describe the maintained solver.

Sources:
- https://help.roborock.com/us/product/roborock-saros-z70
- https://global.roborock.com/pages/roborock-saros-z70 (title-only extraction)
- https://html.duckduckgo.com/html/?q=Roborock%20Saros%20Z70%20camera%20intrinsics%20calibration
- https://github.com/AlexxIT/go2rtc/tree/v1.9.14/pkg/roborock
- https://raw.githubusercontent.com/AlexxIT/go2rtc/v1.9.14/internal/roborock/roborock.go

Actual public go2rtc v1.9.14 `pkg/roborock/{api,client,producer}.go` and registration source were inspected. No calibration/intrinsic/focal/distortion fields or 1600/1200 constants were found there. Transport retrieves video, not a factory camera calibration. This does not rule out another vendor endpoint; no speculative hardware RPC was issued. Marketing FOV, even if found, would not establish principal point, distortion, cropping or per-unit calibration.

## Concrete measurement path using existing native DimOS tool

1. Use A4 landscape paper with **9×7 checker squares**, corresponding to **8×6 inner corners**. Choose nominal **25 mm** squares: checker area 225×175 mm fits 297×210 mm with margins. Mount flat on rigid backing. Print actual size, disable fit-to-page; measure a printed square (prefer several adjacent squares and divide) rather than trusting printer scale. If hand drawing, sharp straight edges and accurately uniform squares matter; a printed target is preferable.
2. Keep robot/camera stationary and unchanged, front view at exactly 1600×1200. Human moves only the board. Acquire 20–30 sharp views with all inner corners visible, varied horizontal/vertical tilts and distances, distributing corners across the image. Retain several held-out poses, not just adjacent duplicate video frames. Avoid extreme blur/glare. No robot or arm motion is required.
3. Capture using existing `integrations.dimos.native_slam.capture --live --calibration-only` with actual fixed-lens evidence and a private new output folder. Existing token reference remains private; do not duplicate it into calibration artifacts. This operation still needs the physical target in place; no capture was started in this research.
4. Solve using upstream folder calibration with measured square size. Example below uses 0.025 only if actual print measures 25.00 mm; substitute measured value.

```sh
SDK="$HOME/.openclaw/workspace/dimos-v1/native-slam"
export PYTHONPATH="$HOME/.openclaw/workspace/dimos-v1/source"
"$SDK/.module-venv/bin/python" -c \
 'from dimos.cli.cameracalibrate.cameracalibrate import main; main()' \
 --source folder --images /private/selected-front-target-frames \
 --cols 8 --rows 6 --square-size-m 0.025 \
 --out /private/front-camera-info.yaml /private/front-camera-preview.png
```

This exact upstream entry was inspected; importing it currently fails on missing `yaml` (PyYAML) in `.module-venv`. OpenCV is installed (5.0.0), but the calibration command has **not** yet been executed or fully dependency-qualified. Install maintained PyYAML/remaining declared CLI dependencies into the isolated environment before claiming it runnable. No installation was performed during this preflight. Upstream documentation: `dimos-v1/source/docs/usage/camera_calibration.md` and implementation `dimos/cli/cameracalibrate/cameracalibrate.py`.

5. Compare supported pinhole and fisheye models using held-out reprojection/residuals and image-edge behavior, not an arbitrary FOV assumption. Record fit RMS, per-view errors, accepted/rejected views and held-out checks; solver success alone is insufficient.
6. Convert reviewed YAML explicitly into existing native JSON contract: exact width/height, `fx=K[0]`, `fy=K[4]`, `cx=K[2]`, `cy=K[5]`; ROS `plumb_bob` → native `brown_conrady` preserving five coefficient order; ROS `equidistant` → native `fisheye` preserving four coefficients. Bind verified physical `lens_id`; `camera_to_base=null` unless independently measured. Retain YAML/images/error provenance privately. Existing live CLI does not automatically consume the ROS YAML, so this conversion is a separate integration step, not a command already provided here.

Only after calibration review should real video enter native sparse SLAM. Translation/parallax and metric scale measurement remain separately authorized later work. Do not use the vendor map as calibration truth or mapper input.
