# Encoder-free visual control: offline foundation

Implemented only in new `src/visual-replay.ts`, `src/visual-action-learning.ts` and
`src/visual-control.test.ts`. No service/MCP/ROS writer integration, live acquisition,
model invocation, deployment or authority changes. No new dependencies: existing
Zod supplies strict JSON validation; numeric image-space operations need no library.
Existing camera calibration/projection remains the metric path. Existing command
uncertainty and measured-completion contracts remain mandatory and unchanged.

## Approach and existing solutions

The implementation follows image-based visual servoing (IBVS), but deliberately
uses local empirical discrete-action effects instead of inventing a robot Jacobian,
joint model or encoder feedback. Background: Chaumette & Hutchinson, *Visual
Servo Control, Part I: Basic Approaches*, IEEE RAM 2006,
https://doi.org/10.1109/MRA.2006.250573 . Pixel-space improvement is not collision
avoidance, metric localization, successful grasp or cessation evidence.
For future extraction/calibration, prefer maintained OpenCV tracking and
checkerboard/ChArUco tools rather than building a detector here:
https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html . This task did not download,
install, run or freshly benchmark those tools. Existing stock-camera feasibility
and manual named-step research are the repository baseline; their wire steps are
not calibrated SI motion. The present module accepts externally extracted features
and explicitly supplies neither detector nor camera timestamp implementation.

## Evidence contract

Replay v1 binds device, camera source, stream, connection/acquisition session,
physical lens, exact dimensions, configuration and feature-space identities.
Configuration must change for mode/mount/load/lighting changes outside measured
validity; session must change on restart or source switch. IDs are opaque and
nonsecret. Lens evidence and acquisition evidence refer to externally reviewed
records: passing validation does not authenticate their truth.

Acquisition is either a timestamp with measured clock uncertainty, a measured
min/max exposure age at receipt, or `unknown`. Receipt recency is never substituted
for exposure age. Unknown age is retainable but unusable for training/recommendation.
Age bounds require a transport/producer experiment that excludes cached frames;
a local network round-trip alone cannot bound cached-image age. Stock receipt-only
frames therefore remain blocked. The new replay contract is independent of the
existing camera contract; no adapter upgrades stock evidence automatically.

Features have stable correspondence ID, normalized coordinates, explicit
confidence, occlusion and Euclidean localization-error radius. Confidence is a
quality filter, not a calibrated probability. All included features must pass.
Replay copies/freezes input and rejects repeated receipt IDs, scope changes,
reordered receipt times and overlapping known acquisition intervals. Unique local
receipt IDs alone cannot prove distinct exposures or detect identical cached JPEGs.
A future acquisition adapter must authenticate producer evidence and content history.

## Learning and recommendation

At least three distinct reviewed isolated-one-action trials are required per exact
binding/action/configuration. Frames cannot be reused across trials. Pre-action
acquisition must precede dispatch; post-action acquisition must follow it and be
within the declared observation horizon. Unresolved execution is excluded. Review
is dataset metadata, NOT a controller-terminal certificate or authority settlement.

The model is a mean normalized feature displacement, enveloped by maximum observed
residual, both endpoint localization errors and an explicit extra uncertainty
margin. It is configuration-local with a start-feature neighborhood and expiry.
Three trials is an implementation minimum, not adequate physical calibration.
The envelope is empirical, NOT a certified bound or out-of-distribution guarantee.
No extrapolation, action composition, inverse kinematics or sequence planning.

The stateless controller returns `hold` or one advisory action token with
`dispatchAuthorized: false`. It admits only a strictly robust predicted reduction
in target error using the envelope, checks fresh evidence/locality/expiry, and holds
unconditionally for outstanding action or execution uncertainty. Reaching a pixel
target returns hold, not physical success. Caller lifecycle management is not
implemented: repeated calls can repeat recommendations; never treat this module as
a dispatcher. No automatic uncertainty clearance exists. It cannot exclude delayed
execution, recover command ownership, certify stationary motors or implement the
existing encoder/localization/controller completion contract without those sources.

## Supervised calibration gates (no trial performed)

1. Review existing admission/authority, independent stop paths and current unresolved
   work. Establish owner supervision, clear workspace, stationary robot, safe arm
   configuration and a reviewed recovery plan. Do not clear uncertainty using vision.
2. Privately consent to dataset retention; avoid faces/credentials, record software,
   firmware, device/session/lens/mount/mode/load/resolution and lighting. Verify lens
   physically. Characterize acquisition provenance, clock/latency bounds and caching
   while stationary before any action dataset.
3. Use a fixed visible target and reference features with reviewed correspondence and
   error bounds. Measure occlusion/blur/illumination failure rates on held-out replay.
   No metric geometry claims without the separate calibration contract.
4. Authorize one existing allowlisted discrete action at a time through the existing
   supervised path. Record actual dispatch/ACK separately; never retry ambiguous
   commands. Collect multiple pre/post exposures and continued observation beyond
   delayed execution. Operator review and available independent controller evidence
   remain separate. If pending work cannot be excluded, trial is unresolved/ineligible.
5. Repeat safe isolated trials at multiple nearby starting configurations, with no
   concurrent chassis/arm/workflow action. Separate action IDs/configuration datasets.
   Include no-effect, wrong-sign, variability, load, occlusion and timeout cases;
   do not discard failed/ambiguous records to create optimistic training data.
6. Fit on training trials; validate effect envelopes and recommendation holds on
   held-out sessions and boundary conditions. Select margins/locality/expiry from
   measured failures, not these synthetic fixture constants. Reject unsafe or
   inconsistent effects. Model validity is narrow and must be renewed explicitly.
7. Only after a separate safety/authority design review consider a one-action bridge,
   with lifecycle/replay protection and persistent unresolved-command ownership.
   That bridge is out of scope. Hardware experiments remain supervised; autonomous
   grasping, deployment and unattended operation are not enabled by this foundation.

## Offline checks

`node --test src/visual-control.test.ts` exercises wholly synthetic feature replay,
unknown/bounded age, stale/contradictory captures, lens/session/quality failures,
replay mutation/duplicates/order, isolated-trial rejection, empirical uncertainty,
local expiry and one-action/hold decisions. `npm test` and `npm run typecheck` validate
the merged working tree; other preexisting dirty work was not modified by this task.
