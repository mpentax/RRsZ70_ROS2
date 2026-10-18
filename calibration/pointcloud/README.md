> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [mapping/depth guide](../../src/z70_mapping/README.md), [current calibration](../front-camera/README.md).

# Historical single-frame depth experiment evidence

The unused standalone Depth Pro / DA2 Base prototype was retired during phase 5.
Its result JSON, observations and local images remain here unchanged; source is
recoverable in Git history. None of these files is a model checkpoint.
The supported optional worker is now `src/z70_mapping/z70_mapping/depth_worker.py`,
which consumes the shared ROS image stream and retains native MPS support.
Its DA2 Small model revision and external-cache setup are documented in
`docs/ros2.md`. No external cache was inspected or deleted by this cleanup.

Venv: `~/.venvs/roboclaw-depth` (torch 2.x + MPS, transformers). It lives outside the repo; `.local/` is the service's data mount.

## Why the floor correction is essential

Indoor depth networks are trained at human eye height; this camera is 68 mm off the floor. Raw metric output is
**+83 % (Depth Pro) to +320 % (Depth Anything V2) too far**. Floor pixels have exact known depth from the calibration,
so each frame is corrected with `1/z = a/z_net + b`, fitted robustly on floor pixels. That form models networks that
compress near range; a pure scale is worse.

## Results on frame-3 (held out: scale fitted on the floor strip *beyond* the boards, tested on the boards)

| Model | Time / frame (M-series GPU) | 40 mm board: corner pos err, pairwise err | 20 mm board (8–32 cm): pos err, pairwise err | Floor flatness RMS |
|---|---|---|---|---|
| Depth Anything V2 Small, inv-affine | **0.05 s** | 5 mm, 3.2 % | 20 mm, 7.9 % | 2.4 mm |
| Depth Anything V2 Base, inv-affine | ~0.1 s | 4 mm, 4.2 % | 28 mm, 17 % | 2.8 mm |
| Depth Pro, inv-affine | 1.2 s | 6 mm, 5.9 % | **11 mm, 7.4 %** | **1.8 mm** |
| Depth Anything V2 Small, scale only | 0.05 s | 23 mm, 5.2 % | 39 mm, 10.6 % | 2.8 mm |

Board corners sit within ±1–7 mm of floor height in the reconstructed cloud.

## What a single frame does and doesn't give you

- **Good:** floor / free space within ~1 m, correct depth ordering of furniture, near-field metric geometry.
- **Not good:** beyond ~1 m, object footprints smear along the viewing ray (classic monocular depth uncertainty). The
  top-down map shows radial streaks rather than compact table-leg footprints. Thin legs, the cat and bags blur together.
- These are limits of one view. Fusing many frames from different poses into an occupancy grid removes the streaks.
  That fusion is the mapping middleware's job.
- Not yet checked: off-floor metric accuracy (e.g. distance to a table leg, table height). A tape-measure check
  of 2–3 objects would close that.
