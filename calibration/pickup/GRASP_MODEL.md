# Equator grasp model — offline correction, 2026-10-14

## What the images establish (and what they do not)

The owner is authoritative: `app_grip` only closes the jaws. The previous catch was against an upper side of an object jammed between others; it was not autonomous positioning/lifting. A free 66 mm dodecahedron needs opposing pads at its roughly 33 mm-high equator, followed by an explicit hand-up command.

- `20261014T103800.849585Z-ros/1-pregrip-gripper.jpg`: yellow HSV centre `(835,633)`, size 556 px; the bottom of the object is cropped. Close inspection places foreground dark upper-corner portions at left approximately `(0,53)..(151,0)..(0,143)`, right approximately `(1445,0)..(1599,98)`. These are **cropped finger/housing portions, not exposed terminal tips**. The black band farther down is the rug, not a jaw-to-jaw contact line. The former “fingertips at bottom corners” claim is unsupported.
- `2-lifted-gripper.jpg`: the compact empty closed-gripper silhouette has a **candidate** central termination around `(790,110)`, vertical uncertainty roughly ±20 px. Its identity as a contact pad is not resolved. The target is still down near/beyond the bottom edge. This supports the miss, not a held object.
- Open calibration frames `A-00`, `A-02`, `A-07` show floor/board in their lower corners; they do not resolve a pair of contact tips. Their log also contains silently ignored `-1` jogs and mismatches between planned/tracked/estimated angles; do not fit fingers to the planned states.
- Earlier `20261014T082117Z/15-pregrasp-gripper.jpg` and `20-attempt1-hold-gripper.jpg` are out-of-focus close-ups, not measurable opposed-pad geometry. In `17-attempt1-front.jpg`, the visible tips meet the yellow upper silhouette near `(645,421)` and `(734,422)`, while the object bbox centre is around `(678,581)`: an upper-side catch, not equator contact, matching the owner. The wrist marker is visible but neither its camera offset nor the occluded pad contact depth is measured, so it does not determine camera-to-pad length. `RESULTS.md` reports apparent elevation after the up step but explicitly no certified object-between-fingers evidence.

**There is no independently measured camera-to-pad length or demonstrated closing trajectory in these photos. A physically certified grasp criterion cannot be recovered from them alone. The new geometry below is explicitly a provisional trial hypothesis, not a solved calibration.** In particular, extrapolating an open corner silhouette into a closing line is not justified; M5 can change fingers without changing the camera.

## Candidate correction used in code

At the candidate closed midpoint `v=110`, `f=843.0311`, `cy=310.1`, the ray is 13.3525° above the optical axis. Since the camera points 2.4° below link 2, the ray is **10.9525° above link 2**. Consequently:

- `FINGER_ANGLE = radians(-11)` (the code's positive sign means below link 2), replacing +36°.
- `FINGER_LENGTH = 0.10 m` **retained as an unmeasured nominal camera-to-contact radius**. It is not a measured finger-body length. Pixels alone cannot supply this scale.
- Nominal link-local camera-to-closing-centre vector is `(forward, up) = (0.098163, 0.019081) m`; rotation by `M4-M3+90` places it in the vendor sagittal frame.
- Floor checking uses the entire provisional radius/angle envelope: 80–120 mm and 6–16° above link 2. Every sampled edge must keep its lowest candidate endpoint at least 15 mm above the floor, as well as respecting vendor limits and wrist reach `<0.364 m`. This is a bound **within the assumed envelope**, not a physical guarantee until pad geometry is measured.
- Nominal `GRASP=(240,105)` has vendor wrist `(0.346176,0.083793) m`, nominal contact height 36.374 mm and envelope lower endpoint 16.775 mm. Its floor target is base `(0.503057,0.021507,0)` m. The arm origin is base `(0.074,0.026,0.0085)` m, equivalent to front-camera-floor x `-0.201 m` with the 0.275 m camera offset.

The candidate footprint is about 85 mm farther forward than the old `(0.418054,0.022397)` target. Approach therefore changes too; preserving the old chassis standoff would not put this contact point around the object.

## Exact pre-grip condition

For **two consecutive fresh observations of the same tracked target**, without intervening motion:

1. The lowest contour/floor contact must be visible: `4 <= contact_v <= image_height-5`. A clipped bottom is not a floor measurement. Missing target/pose/contact never authorizes closing.
2. Reconstruct the equator as `(contact-derived floor_x, floor_y, object_size/2)` in base coordinates, then project it with the calibrated wrist-camera model. In the 1600×896 reference, require **`90 <= equator_v <= 140`** and **`672 <= equator_u <= 928`**. This is **not the HSV centroid v**. At this upper contact ray a 66 mm object may be top-cropped, biasing its visible centroid downward.
3. Visible bbox width must be 0.6–1.5 times `f * object_size / equator_depth`, with focal scaled to image width. Require nominal contact height within 12 mm of the equator and horizontal contact-point error at most 20 mm.
4. The current pose and each commanded approach edge must satisfy the 15 mm envelope floor bound and firmware checks.

Example: at nominal GRASP and its footprint, predicted equator is `(805.54,133.58)`, floor contact `(804.10,319.95)`, equator depth 99.76 mm. The failed low, large blob no longer meets the criterion. **The 90–140 band depends on the unconfirmed pad annotation; it is not evidence of actual contact by itself.**

## Descent and the extra movement the miss needed

The supplied brief says the last pose was `(230,95)`, but this run's saved `grasp_goal`, `descent_check`, and command sequence end at `(220,85)`; no saved commanded `(230,95)` should be invented. With the candidate model:

| Pose | Nominal contact height | Remaining lowering to 33 mm |
|---|---:|---:|
| (230,95), brief estimate | 61.719 mm | 28.719 mm (~one nominal 40 mm down step) |
| (220,85), saved timeline | 83.597 mm | 50.597 mm (~1–2 nominal down steps) |

**Do not send those counts blindly.** A nominal vendor down from `(230,95)` changes pitch too: FK/IK predicts `(239.708,98.225)`, tip height 12.717 mm and envelope lower endpoint −7.926 mm, failing floor clearance. The implementation instead uses model-checked pitch/lowering jogs. From `(230,95)`, the safe candidate route is M4+ to `(230,105)`, then M3+ to `(240,105)` (net 25.345 mm lowering). From the logged `(220,85)`, it is M4+, M3+, M4+, M3+ (net 47.223 mm lowering). Both need range realignment if retaining the old floor spot around x=0.397 m: approximately 106 mm **backward**, not further forward.

The executor observes after every one-joint action and replans; it does not run a blind descent list. It corrects range while still high (≤12 mm nominal chassis pulses, either sign), then lowers/pitches. It never chooses the largest new blob to replace its tracked target. Non-convergence/unreconciled geometry means no `app_grip`, not a fabricated success. Vendor payloads remain `app_arm_move {m:[0|1|2,...]}` and a one-element list for `app_rc_move`.

Only after the condition holds: snapshot → one `app_grip` → explicit `app_arm_direction_move {direction:1}` lifts → lifted snapshot → held-image check **and** front-camera disappearance at the recorded floor spot → 10 s retention → put-down/stow. Existing explicit lift, front verification, and snapshots are retained.

## Parent's live verification before trusting this hypothesis

Confirm the opposing contact-pad midpoint/trajectory, M5 orientation, actual camera-to-contact radius and pad underside, and that the fingers genuinely straddle the middle rather than merely satisfy the provisional projection. Check the 15 mm floor margin physically; the lowest envelope point has only ~1.8 mm numerical slack at the candidate GRASP. Annotate clear open/closed pads at a known pose/scale to replace the provisional constants. Verify that floor contour projection remains accurate near a top-cropped/close object and is not a reflection; apparent width/centroid cannot prove contact. Reconcile the `(220,85)` versus `(230,95)` pose discrepancy from live observation, not another dead-reckoned anchor.

No live robot RPCs, camera acquisition, MCP tools, service changes or `pixi run pick` were used for this work; only retained photographs and localhost fake-robot tests.
