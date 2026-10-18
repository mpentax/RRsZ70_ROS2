> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [current arm model](../../docs/z70-arm-model.md), [camera/calibration guide](../../research/cameras/FINDINGS.md).

# M4 then M3: front-camera calibration plan

Status: PLANNED ONLY. No robot commands sent for this task.

## Scope

Use the existing flat arm-mounted ID 0 marker: 20 mm paper square, 15 mm printed black side. Test M4 first, then M3, with the other joints held fixed during each measurement block. This is an adaptive front-camera test of joint increments, relative trajectories and return consistency; not an independently calibrated absolute wrist position.

## 1. Prepare a camera-visible baseline with joint travel room

- Fresh supervised setup; inspect animal clearance and marker security before motion. Enter manual mode and deploy. Wait for deployment to finish, rather than interpreting its RPC acknowledgement as physical completion.
- Bring the hand forward/down using model-checked setup moves; observe the resulting framing. Do not blindly replay old counts as proof of a physical pose.
- Seek a baseline where the complete black square and gripping end are visible with margin for the planned joint arc. Keep M5 fixed at its baseline and the jaws unchanged.
- Do not reuse the previous five-forward/five-down wrist baseline: the model rejects M4+ for projection and M3+ for ground collision there.
- Offline candidate: four forward/four down gives a predicted wrist point (316.8, 80.7) mm, and all four single-joint +/-10 degree endpoints pass model checks. However, this setup had marker cropping in previous observations. It is a kinematic candidate, NOT an established camera-visible baseline. Choose an alternative baseline separately for each joint if necessary.
- Model wrist checks do not cover gripper length. Require observed hand/finger clearance as well. Do not add blind motion merely to obtain more samples.

## 2. Record a reference and angular noise

Capture two settled frames after the reply plus 2 seconds. Check the full black square, correct target identity, rigidity, <=1 px marker-centre difference, and separately labelled <=2 px pose-fit RMS screen. Record the orientation difference within the stationary pair too. Preserve rejected images. Obtain an additional stationary pair at the same reference if angular ambiguity is comparable to a nominal 10 degree jog.

## 3. Calibrate M4 first

Commands: positive app_arm_move {m:[0,1,0]}; negative {m:[0,2,0]}.

Hold M3, M5 and gripper state fixed. Start with one nominal 10 degree step in the sign that has both camera margin and clearance, then return. If both signs are observable and safe, sample q0 -> q0+10 -> q0 -> q0-10 -> q0 and repeat. Expand to +/-20 only after single-step observations and model checks support it; no fixed blind three-step block. Re-check both the outward step and return before dispatch.

## 4. Calibrate M3 next

Commands: positive app_arm_move {m:[1,0,0]}; negative {m:[2,0,0]}.

Choose/reacquire a suitable reference; do not assume the M4 reference also permits M3's larger arc. Hold M4, M5 and gripper state fixed. Use the same single-step/return and optional two-sided pattern. The simulator predicts opposite forearm-angle changes for positive M3 and positive M4; compare signed responses rather than only unsigned rotation magnitude.

## 5. Compare observations with simulation honestly

For every accepted pose retain the command, predicted joint state, marker corners, camera-frame marker position/orientation, both frames, fit residual and stationary-pair angular difference.

Report per joint/sign: measured angular increment, variation, cumulative excursion and return difference. Compare the measured gripper-proxy trajectory with the simulated motion, including image-space arrows/overlays and camera-frame marker displacement.

The printed marker centre is not the simulated wrist point. Marker-to-wrist offsets and camera-to-body registration remain unknown. For positional prediction, use a single fixed model-to-camera/marker registration estimated from a designated initial subset, freeze it, and score later held-out poses. Label this a fitted local model-consistency check, not independent extrinsics or absolute gripper calibration. Do not fit each pose separately or assume zero marker offset. If registration is underdetermined, report observed marker motion and rotation without inventing a millimetre wrist error or declaring link lengths calibrated.

## 6. Bounds, stop conditions and outputs

- No motion on unclear delivery, unexpected motion/contact, animal incursion, failed camera capture, or lost target during measurement. No retry of an ambiguous movement.
- Reverse while the target is still observable and approaches an edge; do not expand a direction until it disappears. If a first probe unexpectedly loses the target, stop that acquisition and retain the failure.
- If no safe, camera-visible single-step pair exists for a joint, report that coverage gap; do not substitute a measured zero or claim that joint is calibrated.
- Save everything under calibration/arm-validation/. Produce M4 and M3 tables, annotated contact sheets and model-versus-observed trajectory plots. Keep existing rejected samples distinct.
- Finish with stow/manual-exit and physical stow confirmation. Adopt/correct constants only if measurement uncertainty and held-out evidence support it; otherwise record the discrepancy and its unresolved causes.
