# Yellow pickup: verified supervised lift and hold

## Outcome

Attempt 2 succeeded: the yellow dodecahedron left the floor, remained suspended
at the black gripper in both raised observations, and was held for **10.803933 s**
between `hold_start` and `hold_10s`. Two explicit `app_arm_direction_move
{direction:1}` commands followed one `app_grip`; **no chassis pulses or joint
jogs followed grip**. The final front image shows the target upright on the
floor with no visible gripper. Final status: **48% battery, state 3, error 0,
not cleaning/returning, grab status 0, grab mode 0, arm estimate IN_ROBOT**.

Battery before any motion was 53%; it stayed above 15% throughout. Trials were
stopped after the successful second attempt. No third pickup was attempted.

## Attempts and evidence

1. `../20261015T052857.036635Z-ros/`: no grip sent. Post-deploy bearing error was
   -16.97 degrees; the controller reacquired without the old track gate and
   realigned high. At low pose it correctly raised before each base correction.
   It eventually failed with `target missing from jaw measurement frame`.
   Inspection of `jaw-servo-00-front.jpg`, `jog-250-95-front.jpg`, and the
   timeline showed partial-face segmentation and unstable floor localization,
   not a lifted object. Live HSV measurement found warm/shadow target facets
   H12..21 (median 15); the H16 cutoff discarded much of the dodecahedron.
   Expanded yellow to H10..29 with the metric/aspect/acquisition gates kept.
   Cleanup stowed the arm; status had no error.
2. This directory (`20261015T053609.715453Z-ros`): successful pickup.
   - `1-pregrip-front.jpg`: target on the floor, black open extensions on
     opposite sides of its upper region, gripper centered.
   - `2-lift-step-1-front.jpg`, `2-lift-step-2-front.jpg`: separate observations
     after each hand-up command.
   - `2-lifted-front.jpg` (05:37:58.973 UTC), `3-front-verify-front.jpg`,
     `held-00-front.jpg`: yellow target suspended with clear air gap below.
   - `4-held-10s-front.jpg` (05:38:10.583 UTC), `held-01-front.jpg`: still raised
     in essentially the same configuration. `pickup-sequence.jpg` juxtaposes
     pregrip, lifted and held frames. Copilot vision inspected the saved images
     and the chronological montage: same target, changed support, compliant
     deformation, no other visible support, stable raised configuration.
     Individual load-bearing jaw surfaces are partly obscured; this is not an
     exact pad-contact/anatomy measurement.
   - `6-final-front.jpg`: separate fresh MCP frame after cleanup, showing the
     yellow target resting independently on the floor, with the arm absent.
   - `final-state.json`: sanitized final status/estimate/readback and mode exit.

## Post-deploy offset

The successful run logged `visual_yaw_offset_rad=0.27222651592484975`, or
**15.597430434 degrees left**, from target bearings before/after deploy. The
reacquired target was [0.38899,-0.09756,0] m with -17.6409-degree bearing error.
Odometry was unavailable at that boundary, so `yaw_offset_rad` is explicitly
null: the visual residual is an estimate that also includes localization/
translation uncertainty, not a measured full-turn trajectory. High-arm
realignment followed before descent. The two low-pose confirmation frames had
about 3.5 mm horizontal error and 15.2 mm vertical error.

## What happened around the vendor turn

`hold_start` was 05:37:59.779777 UTC; `hold_10s` was 05:38:10.583710 UTC. Front
images show no substantial background rotation during that hold. Available
VO changed about -0.009 degrees yaw and 0.000053 m (VO is not independent
physical feedback). Release began only afterward, with `app_put_down
{flag:0}` at 05:38:10.583899 UTC. By the `5-put-down-front.jpg` exposure at
05:38:12.910875 UTC the background had swung toward the wooden table and basin,
**while the object was still airborne at the gripper**. Stow was dispatched
at 05:38:12.911137 UTC; the later `6-final-front.jpg` shows the released target
on the floor, facing the table. This localizes the observed rotation to the
put-down/cleanup phase, not the verified hold. Sparse frames/VO loss cannot
establish a complete 360-degree turn or separate every subsequent vendor
motion. The final placement location differs from the original floor spot.

The successful trial exposed that a 2 s put-down delay raced an autonomous
program. The final code now waits **20 s before stow**, logs release progress,
then waits 10 s after stow and captures final evidence. These longer cleanup
waits, snapshot hazard propagation and stale-clearance invalidation were
verified offline after the live success; they were not exercised by a third
pickup. Final release/stow was independently confirmed on this live run.

## Changes and verification

- Post-unstow fresh two-frame acquisition, reset prior, high-arm alignment and
  separate odometric/visual yaw logging.
- Central no-low/unknown-jaw and no-post-grip base-pulse invariant; image-verified
  raise/correct/re-descend sequence; each jog invalidates old high clearance.
- Two explicit lifts, elevated-object/floor-gone observations and >10 s hold.
  When ID68 exits the top of the front FoV, retain the measured pregrip depth
  lift column with no intervening base motion; do not reuse post-grip TF.
- Existing owner-informed 165 mm jaw vector/GRASP=(250,95) kept. Corrected the
  nominal-wrist TCP offset as well as marker registration so TF and planner
  agree; the full suite caught three stale-TCP tests not covered by manipulation.
- Colour-facet regression, spin reacquisition, high-only chassis corrections,
  pulse prohibition, stale-clearance, floor-object negative, cleanup, and real
  hazard tests. No existing reach/floor/path-hazard test was weakened.
- Final `pixi run test`: **96 passed**. Final
  `pixi run test src/z70_manipulation`: **45 passed**. `git diff --check` clean.

JPEG evidence is retained locally in these run directories and intentionally
remains git-ignored under the repository's home-interior-photo policy.
Timelines, this report and sanitized final-state data are committed.
One supervised success is not a repeatability or general-object guarantee.
