# Passive chassis calibration

Normal `pixi run bringup` starts `chassis_calibrator`; it never issues robot
commands. During an authorized live session it pairs `/z70/commands` dispatch
attempts with `/odom` and typed `/z70/odom_status`, preserving source session,
sequence, timestamps and UNKNOWN delivery. No calibration-only pulse schedule or
per-step confirmation is required.

Windowed least squares over the last 500 valid samples learns separate positive/
negative yaw gains, forward gain and camera_x, with counts and fit-only sigma.
Interrupted/overlapping moves, rejected/nonvisual odometry, missing poses, event
gaps or changing/unknown camera offsets are skipped, not robot stops. Continuous
motion may provide no isolated settled windows and therefore no new fit samples.

## Odometry versus model fallback

Accepted floor VO is measured image motion; its zero covariance is unmodeled,
not zero uncertainty. Rejection normally holds the last pose/reference. When a
successful correlated RC pulse has an interval not covered by accepted VO, the
odometry node can propagate that uncovered interval using calibrated unicycle
command dead reckoning. It waits for motion/reply readiness, trims covered slices,
and rebases the image reference rather than double counting a whole pulse.

Fallback is `COMMAND_DEAD_RECKONED`, `pair_accepted=false`, planar covariance
`1e6`; it is not measured travel and cannot train the calibrator or enter the
mosaic. Initial returning reference frames retain that uncertainty until a real
VO pair succeeds. `odom -> base_footprint` continues at 5 Hz even without front
images, so TF availability is not evidence of accepted VO.

## Persistence and consumers

Mutable state is `$XDG_DATA_HOME/RRsZ70_ROS2/calibration/chassis/online.json`
(fallback `~/.local/share/RRsZ70_ROS2/...`), atomically saved with seen command IDs.
Installed historical fit seeds initialize new state; archives are never runtime
write targets. An explicit `initial_state_file` import only applies when the
output is absent. Repeated publication retains source update time and calibration
identity rather than pretending to be a fresh observation.

Typed `/z70/chassis_calibration` is RELIABLE + TRANSIENT_LOCAL and is exposed via
the shared gateway. Consumers honor known/valid/stale fields. VO/mosaic accept a
camera offset fit with at least four turns and sigma ≤0.01 m (`min_camera_n`,
`max_camera_sigma_m`). Gain estimates also feed command-model fallback and motion
planning; they do not turn fallback into visual evidence. Fit sigma excludes
floor calibration, lateral-offset and slip bias; inspect residuals and independent
cross-checks before interpreting physical accuracy.

## Offline cross-check (repository root)

```sh
pixi run test
pixi run bash -c 'source install/setup.bash && python -m unittest discover -s calibration/chassis -p "test_live_analysis.py"'
# Set these to an existing manifest and a NEW output directory:
# MANIFEST=calibration/chassis/<session>/pairs.json
# OUT=/absolute/path/to/new-analysis
pixi run bash -c 'source install/setup.bash && python calibration/chassis/analyze.py "$1" --calibration "$(ros2 pkg prefix --share z70_description)/config/front_camera.json" --output "$2/fit.json"' -- "$MANIFEST" "$OUT"
```

The image-based `calibration/chassis/test_synthetic.py` also writes a fixed
`synthetic-results.json` beside its source, so use a copied workspace for that
legacy reproduction rather than running the whole chassis task over retained evidence.

The analyzer writes `fit.json`/`RESULTS.md` (and, if adopted, `chassis_measured.json`)
in the output's parent directory. Always choose a fresh output; omitting `--output`
would overwrite reports beside the input manifest. Sourcing the install resolves
current package imports despite historical source-path hints inside the analyzer.
Keep originals/images unchanged and distinguish live odometry from whole-pair VO
cross-checks. A rejected measurement is a data gap, never automatic motion replay.
