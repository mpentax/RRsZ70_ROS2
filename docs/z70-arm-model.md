# Z70 arm geometry and command model

The ROS model uses copied CAD structure, retained optical registration and separate
photo-estimated contact/collision proxies. The recovered app endpoint is a fourth,
nominal command coordinate—not the CAD wrist.

## Sources and consumers

Paths below are relative to `src/z70_description/` unless otherwise stated.

| Source | Current meaning and consumers |
|---|---|
| [`config/cad_arm.yaml`](../src/z70_description/config/cad_arm.yaml) and [`meshes/cad/`](../src/z70_description/meshes/cad/README.md) | CAD M3→M4 **0.1772 m**, M4→M5 **0.1157 m**; shoulder in `base_footprint` **[0.1429, -0.0117, 0.1266] m**. Supplies URDF structure/visuals, `pincher_geometry.hand_pose()` and planner physical points/camera. |
| [`config/mount_calibration.yaml`](../src/z70_description/config/mount_calibration.yaml) | Retained front-camera floor registration, arm-frame yaw/origin and M3 offset; older optical marker registration and terminal-region surrogate remain active. Its historical geometry/status strings and null physical-L2/M5 fields describe that optical fit, not absence of the current CAD model. |
| [`config/pincher_geometry.yaml`](../src/z70_description/config/pincher_geometry.yaml) | Photo-estimated housing/finger proxies, grasp aim, **15 mm** anatomy allowance and nominal `opening_ratio=2/3`. Used by planner body/floor/contact queries and URDF collision tags—not copied CAD contact anatomy. |
| [`z70_description/arm_model.py`](../src/z70_description/z70_description/arm_model.py) | `vendor_forward()`, `vendor_inverse()`, `vendor_check()` reproduce the app endpoint geometry and checks. Compatibility names `forward/inverse/check` have the same vendor meaning; `L1/L2` remain CAD constants for compatibility, not inputs to these vendor functions. |

The front camera is registered at x **0.275 m**, z **0.058233433935542395 m**;
roll/pitch and the arm-frame alignment are in `mount_calibration.yaml`. The retained
`measured_shoulder_base_m` is an older optical gauge, distinct from the CAD shoulder.

## Frames and endpoint conventions

Physical poses use REP-103 `base_footprint` (+x forward, +y left, +z up).
`arm_wrist_link` is the CAD M5 station, 0.1157 m beyond the elbow. Its +x follows
the forearm, +z is the unrolled cover normal. Positive app M5 rolls the hand/camera
about physical +x. The optical frame uses +z forward, +x right, +y down.

The app's planar `end_pos` uses x forward, y up, metres, shoulder **(0.0475, 0.14)**
and links **0.1775 / 0.205 m**. It is not an anatomical M5 position or a ROS base
point. `vendor_forward(150,45)` gives approximately **(0.156765, 0.240662) m**.
The estimator's legacy JSON key `wrist` and planner compatibility function `wrist()`
mean this app endpoint; new planner code uses `vendor_endpoint()` explicitly.
Physical hand, camera, collision and planning TF remain CAD-backed.

App-to-URDF conversion is in `z70_driver.driver.app_to_urdf()`:

- `m3_joint = rad(M3 - 270)`; link1 app direction is `270 - M3`, with retained M3 registration.
- `m4_joint = rad(180 - M4)`; link2 app direction is `M4 - M3 + 90`.
- `m5_joint = -rad(M5)` about URDF axis -x.
- M6 drive joints use `rad(M6)/2`; separate collision fingers and visible CAD jaws
  mimic them with `2*opening_ratio`. M6=0/90 means nominal closed/open, not feedback.

The shoulder housing visual is on `shoulder_yaw_link` below display lift/yaw joints.
The planning shoulder `arm_shoulder_link` is fixed under `arm_vendor_frame`; it does
not depend on display-only joints.

## Camera, marker and contact registrations

The URDF and `planner.camera_pose()` use CAD camera-from-wrist
**[0.0355, 0.0075, -0.0109] m**, full `camera_rpy` and M5 roll. Camera projection
uses retained calibrated gripper intrinsics/distortion. The published ID68 TF uses
`cad_arm.yaml`'s owner-adjusted CAD-face placement; see the
[placement record](../calibration/physical-measurements/20261018-marker68-placement/README.md).

Front-marker reconstruction (`perception.front_jaw()` and
`pincher_geometry.marker_hand_pose()`) still uses the older optical marker transform.
`gripper_tcp` and `planner.tip_height()` retain the terminal-pad-region surrogate,
not an exact contact-seam centre. Visible CAD jaws have different hinge roots/shapes
from the photo-estimated collision fingers. These are separate active consumers;
CAD display/TF consistency does not validate optical registration, linkage or contact.

## Nominal command and estimate semantics

Joint selectors are 0=no step, 1=+10°, 2=−10°; −1 is not a negative jog.
`app_arm_move {m:[M3,M4,M5]}` applies those selectors. Program `arm_joints` records
use `value:[M3,M4,M5,M6]`, `step:n`, with the first nonzero joint taking priority.
Hand direction codes 1/2/3/4 are up/down/front/back; prediction adds **0.04 m** to
app endpoint y or x and solves vendor IK near the current branch. This is not a
40 mm CAD-wrist displacement guarantee.

`ArmEstimator` owns command/status dead reckoning, not encoder or visual fusion.
It uses vendor endpoint FK/IK/checks and retains the **0.367 m** increasing-reach
refusal prediction. Fault labels are predictions; `lastFault` is not a measured
rejection or itself a planning-validity transition. The planner combines recovered
vendor checks with its retained reach bound and CAD-backed physical/proxy floor and
obstacle checks. Neither an RPC receipt nor a passing model proves motion completed.

`WORKING_POS` is nominal (150,45,0,90). `TRANSFER_POS` (150,75,0,0) and `ArmState`'s
grasp/putdown transitions are historical app simulation, not observed ROS behavior.
Current `app_grip` closes only and invalidates planning pose; lifting is explicit.
Autonomous grasp/putdown program records also invalidate pose. Unknown delivery,
restart reconciliation and unobserved motion remain distinct from usable planning
state. Fresh visualization `/joint_states` can retain an UNKNOWN pose; consumers
must use typed `ArmEstimate` validity, not infer validity from available TF.

## Recovered app provenance

Roborock app **4.74.08**, plugin `targeted-plugin/index.android.hasm` on the retained
`GOLD16` source volume, arm-checker **Function #32735**, lines approximately
1428440–1429890. Exports include `ForwardKinematics`, `InverseKinematics`,
`checkArmInvalidAction`, `RecordRealPos`, `GetMaxStep`; FK/IK are #32739/#32740,
initial shoulder #32741, state transitions #32742/#32743, checks #32744 and steps #32745.
This is recovered app prediction, not a firmware source dump or encoder model.

Recovered checks, in order: M3 **90–270°**, M4 **35–170°**, M5 **0–155°**;
endpoint y>0.37 → height fault; |x|<0.24 with y<0.14 → body collision;
otherwise y<0.01 → ground collision, then |x|>0.37 → projection fault;
finally `M4-M3+90>15°` → A4 angle fault. App program preview checks repetitions
and reports the first failure/valid-step count; that does not prove manual execution.

[Local M3/M4 observations](../calibration/arm-validation/M3_M4_RESULTS.md) and
[direct M5 camera-roll evidence](../calibration/arm-calibration/20261014T144906Z/RESULTS.md)
support local rotation direction/increments. The nominal 40 mm hand displacement
remains unvalidated. CAD supplies represented structural stations; physical-to-CAD
registration, joint zeros/coupling and true pad contact still require independent
calibration. Retained optical results and historical measurements are not rewritten
by adopting that structural source.
