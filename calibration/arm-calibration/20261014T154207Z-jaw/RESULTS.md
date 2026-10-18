# Front jaw measurement and servo — PARTIAL, no verified pickup

## Outcome

**Battery reached15% (initial19%); live work stopped before a pickup attempt.**
Arm is stowed: settled post-stow common_status51330/readiness-bit clear,
final fresh state3/error0/battery15%, exact estimateIN_ROBOT/all joints0,
and99-stowed-front.jpg has no gripper. Yellow object remains on floor beside
board. No ROS bringup, service restart or commit. No object put-down was needed:
the sole app_grip closed an empty hand for measurement, which was reopened
before stow. The task's successful pickup/10s hold is NOT complete.

Counts from durable paths/logs: **23 raw front JPEGs,
21 logged unique captures, 17 joint jogs,
5 closed-jaw fit views,0 pickup attempts**. Initial/final MCP frames were copied
before requesting another frame. Camera acquisitions succeeded; no retry storm.
The chassis only made one short right turn (omega-0.3,duration800ms,velocity0)
to expose the board; no translation was sent during measurement.

## Measured point and uncertainty

This is a **terminal-pad-region surrogate**, not an independently resolved
rubber contact seam. Front vision resolves the bottom closed finger assembly
but cannot distinguish a true pad-to-pad seam. The annotations use the centre
halfway between the visible notch and the bottom edge; frame identity and
uncertainty are retained in jaw-fit.json. Do not call this exact anatomy.

- Wrist observed ID68, DICT_4X4_1000,15mm black side; expected owner label69 is
  preserved in shared calibration. Marker PnP disambiguated by elevation.
- Rigid region vector in marker xyz: **(1.623,81.917,-30.211)mm**.
- Conditional annotation sigma: **(1.813,8.055,9.175)mm**; normalized coordinate
  RMS1.344. Includes neither all intrinsic/floor systematics nor material-point
  ambiguity. A provisional15mm allowance is retained, NOT a certified error bound.
- Effective nominal link2-from-elbow vector: **(212.956,1.189,29.289)mm**;
  per-view scatter **(3.248,0.374,4.352)mm**. With nominalL2=205mm gauge,
  wrist-relative vector is **(7.956,1.189,29.289)mm**; physical L2 staysnull.
- At240/85 closed-empty, direct surrogate base xyz **(424.924,25.144,28.992)mm**.
  This does NOT reproduce the owner's several-cm high/behind miss; that failure
  may include refused-jog dead-reckoning drift, but this run does not prove cause.
- At250/115 closed-empty, direct surrogate base xyz **(486.055,25.025,42.199)mm**.
  Production replay on the independent same-pose open image gives
  **(486.536,24.243,42.050)mm**: gripper opening did not materially move marker.
- Partial ChArUco-plane check (8 retained corners,1.78px RMS) gives250/115 region
  height47.888mm vs frozen-floor42.199mm: about5.7mm local plane discrepancy.
  Only two views had sufficient board corners, so do not infer precise floor
  extrinsics or certify fingertip clearance from this cropped board.

**M4 zero +4.765deg NOT adopted.** Region/marker registration does not add an
independent physical axis/anatomical station to break the existing gauge.

## Controller changes

Provisional GRASP is **(250,115,0)**, an actually observed empty-jaw pose, not an
exercised grasp. Average chain predictsbase(490.829,24.720,46.140)mm; the lower
240/85 pose's15mm uncertainty allowance would undershoot the15mm floor bound.
The240/115 transition was front-observed despite vendorx369.146mm, so the
nominal reachable envelope was expanded to372mm (375mm constant,3mm margin),
with visual refusal checks retained. Parent should review this limited adoption.

Runtime stays on front camera. Route goes through190/65, settles2.5s after each
single joint jog, and checks image/visible marker displacement rather than ACK
or estimate. The former model-only final descent/closure fallback was removed.
At low pose, jaw and target contact+33mm equator are measured from one frame.
Two exposures must align within15mm with a15mm floor allowance; otherwise one
bounded joint correction or small one-element-list chassis pulse, then remeasure.
No unverified hand DOWN/FRONT/BACK codes are used. Direction1 is known UP.
Post-grip verification associates colour with elevated observed jaws and requires
floor disappearance, then repeats after10s. One vendor-nominal40mm lift retains
marker visibility. This servo/held verifier is **offline-tested, NOT live validated**.

Camera-read retries reduced to3 with>=5s backoff; live battery<=15% returns a
charge request without classifying it as an emergency hazard. Cleanup stows on
ordinary failure, but suppresses recovery motion on real hazard stops.

## Evidence and naming correction

Files f240-95-0-open.jpg,f240-105-0-open.jpg,f240-115-0-open.jpg and
f250-115-0-open.jpg actually show **CLOSED empty jaws**; acquisition helper's
name template was misleading. frame-states.json corrects all identities.
f250-115-actual-open.jpg is the independent open view. Other useful evidence:
f240-85-0-open.jpg,f240-85-closed.jpg,99-stowed-front.jpg,jaw-fit.json,
board-check.json,production-replay.json,marker.jsonl,motion.jsonl,log.jsonl.
No frame shows the object in the jaws or lifted because no pickup occurred.

## Files and validation

Changed ros2/src/z70_manipulation/{z70_manipulation/executor.py,perception.py,
planner.py,test/test_executor.py,test/test_planner.py,README.md}, added
 test/test_front_jaw.py, updated z70_description/config/mount_calibration.yaml.
Retained this directory's acquisition/analysis/replay scripts and metadata.
Pre-existing calibration/chassis/online.json was not edited by this task.

- pixi run test src/z70_manipulation: **37 passed**.
- pixi run test: **88 passed**.
- Earlier duplicate-floor/wrist-ID test batch: **5 passed**.
- git diff --check: passed.

Next required work after charging: supervised live servo/pick, review real
pre-grip/lifted/10s-held front frames, and refine the surrogate/uncertainty if the
fingers still miss. Do not commit or claim a successful pickup on these results.
