> **Historical / superseded — retained in place.** Current owners: [ROS guide](../../ros2.md), [command/API contract](../../thin-api.md), [arm model](../../z70-arm-model.md), [calibration entrypoint](../../../calibration/front-camera/README.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

The DimOS supervisor/job API and routine data-gap abort rules are retired. Identifiability, marker-not-TCP, independent holdouts and restart-estimate caveats remain useful methods. Current CAD supplies represented structure; this optical plan does not establish permanent mechanical unknowns.

## Historical record

# Z70 autonomous arm and gripper calibration plan

Status: research and implementation specification only. No robot connection, camera activation, motion, service modification or deployment is authorized by this document.

## Decision

Command-history estimation is a useful direction, but it must be an explicitly estimated state with uncertainty, not fabricated encoder telemetry. Calibrate a state-dependent transition model, not just a constant distance multiplied by a command counter. Deployment is a candidate reference pose; a successful stow request is not proof of a reference reset.

The intended UX is one physical setup and one bounded trial authorization, followed by automatic measurement, termination, fitting and publication without conversational confirmations. This is not yet an executable unattended hardware capability. The prerequisites below must be implemented and qualified first. Failure means an automatic aborted report, not a request for the user to keep confirming the trial.

## Evidence that changes the design

- Current service contract is docs/thin-api.md and src/service.ts, not the historical shared-command-authority or manual-joint-trial policies. Generic vendor RPC is authenticated but unrestricted; orchestration, safety and physical completion belong in the controller.
- src/mcp-server.ts currently exposes get_state, refresh_status, get_camera_frame and robot_rpc. The HTTP stream-hold alias exists but is not currently an MCP tool. Current cameraSession is allocated once at service creation; it does not prove lens or producer continuity.
- docs/research/z70-programming-live-trial-2026-10-06.md records independently operator-observed held M4 displacement with repeated app_get_arm_joints_data responses of {data:[0,0,0]}. This supports treating that query as unusable position feedback for the tested configuration, not identifying the meaning of its three values.
- Programming entry physically undocked the robot in that trial. Exclude programming mode, exhibition mode, preset programs and pickup workflows from this arm-only calibration. No chassis translation, rotation, docking, cleaning or RC startup is in scope.
- Android evidence supports app_change_camera {flag:0|1} separately from arm-mode entry; a mode-2 detour is not needed by the traced app handler. Actual lens identity, pose preservation and stream freshness still need measurement.
- Robot video provides receipt metadata, not verified exposure time. Repeated JPEG IDs or a changed camera-status bit do not establish a new exposure or the physical lens.
- The owned DimOS controller is currently text-only, with fresh history per request and a four-model-turn limit. The ACP facade supports text prompts, reasoning-only by default, and permission per tools-mode prompt. It is not presently a persistent visual calibration runner. The research-time roboclaw_chat invocation failed before a usable receipt with a missing interpreter; no specialist response or physical execution is claimed.

Local evidence: src/vendor-rpc.ts; src/vendor-commands.ts; src/arm-joint-protocol-research.ts; src/service.ts; src/mcp-server.ts; integrations/dimos/src/roboclaw_dimos/controller.py; integrations/dimos/src/roboclaw_dimos/acp_interface.py; docs/camera-observations.md; docs/arm-joint-control-research.md; docs/arm-camera-mode-research.md. These describe inspected source, not verified live deployment parity.

## 1. Physical setup and observability

Before the one authorization, place the robot off dock on a level, clear floor with an empty gripper. Disable scheduled/competing operations and close other control/viewing clients. Keep people, pets, loose cables and fragile objects outside the entire possible arm envelope. Do not mechanically clamp the robot or defeat its sensors.

Use a fixed webcam at an oblique side/front view that sees the base, joints, gripper jaws and full qualified sweep. A second fixed external view is recommended if one view loses joints or has depth ambiguity. A generic webcam pointed at an unmarked robot can record qualitative movement, but cannot by itself guarantee metric 3D position or joint angles.

The metric setup needs measured camera intrinsics and known-size reference geometry. Prefer lightweight fiducials on rigid exterior link surfaces, a base fiducial establishing robot-frame orientation, and a fixed reference rig outside the sweep. Mounting must not obstruct lenses, sensors, hatch, jaws or joint travel; unknown safe attachment sites require a separate setup qualification. Record each marker-to-link transform and its measurement uncertainty. Without a measured gripper-marker-to-tool transform, publish marker pose rather than relabeling it TCP pose. Fiducial correspondences and calibrated projection are supported OpenCV techniques.[1][2]

Use robot base axes x forward, y left, z up; camera optical axes x right, y down, z forward. Declare the robot-base origin physically. Define T_A_B as mapping coordinates in B into A. Define the gripper tool center and jaw aperture independently. Joint zero and signs are calibration conventions, not the zero-array query.

External-camera intrinsics must be established before metric motion interpretation. A stationary front camera viewing one stationary flat board does not provide diverse intrinsic-calibration views. Either pre-calibrate that lens during setup, use a measured multi-plane calibration rig with suitable solver initialization, or later collect views of a moving known target using already-qualified arm movements. Gripper-camera intrinsics can use arm-generated varied views of a fixed target after motion qualification. Lack of coverage must leave camera geometry uncalibrated rather than trigger unsafe exploration.[1]

## 2. Prerequisites for no-intervention execution

Implement a deterministic calibration supervisor inside the existing DimOS ownership boundary, with a local perception worker. ACP starts one durable job and retrieves its status/results; it does not ask an LLM to improvise successive physical actions. Robot actions and robot-camera lifecycle must use the official host MCP adapter, never a direct device HTTP/RPC fallback. Local webcam capture is host sensor acquisition, not another robot controller.

Before enabling unattended motion, qualify: finite behavior of each permitted command; deployment/stow behavior off dock; camera lifecycle effects; observation delay; stopping coverage; and a reachable stopping mechanism independent of model inference. The vendor documents the physical Emergency Stop, collision sensing and anti-pinch protections, but that does not establish that app_stop_grasp is an equivalent remote emergency stop.[3]

A watchdog that merely stops issuing commands cannot halt an in-flight or delayed operation. Without a qualified independent stopping path and bounded actuator behavior, the system must refuse unattended actuation. Establishing unknown emergency-stop coverage is not itself a safe unattended first experiment. This is a hardware qualification prerequisite, not a request for repeated chat assurances.

Give the job an exact device/firmware/calibration identity, allowed command set, prequalified workspace, command/time/energy budgets and stop policy. The controller must enforce these independently of the model. Exclude all other writers during the job. A controller-only allowlist does not restrict another client of the unrestricted service; dedicated trial credentials/routing isolation or an enforced exclusive trial boundary are required. Do not silently restore the historical service workflow architecture.

Proposed new job operations: start_arm_calibration(config_id), get_arm_calibration_status(job_id), abort_arm_calibration(job_id). These names are specifications, not installed tools. Admission stores the job ID and authorization before dispatch. Unknown start outcomes retrieve that same job, never create another. ACP cancellation is not physical STOP.

## 3. Capture and preflight

Before any motion, automatically verify device identity, off-dock stationary base, battery/fault/task state, clear measured envelope, empty gripper, camera coverage, reference geometry, writable private storage, stopping-path readiness and absence of competing writers. A classifier saying clear is not a collision certificate; use measured coverage and conservative geometry. Unknown or occluded required regions fail preflight.

Capture external video continuously and retain one robot viewing producer through an MCP lifecycle adapter that still needs adding. Validate external-frame freshness with a visible timestamped optical event; use a locally driven light/display visible to each relevant lens and outside the sweep. Record clock relationship, decode latency and its measured uncertainty. Host receipt times and media PTS remain distinct from exposure times. Loss of this evidence prevents temporal calibration claims.

Robot front and gripper views should be assumed multiplexed, not simultaneous.[4] Keep the external camera continuous. At stationary checkpoints select one lens with app_change_camera, discard transition frames until scene-based identity and bounded freshness are established, capture, then select the other. Do not restart preview or enter mode 2 as hidden recovery. Lens identity requires recognizable geometry/content, not only a status bit. Unexpected arm/base movement during camera operations aborts the job.

Implement producer/lens epoch tracking; do not use the current service-start cameraSession UUID as proof of continuity. Archive every useful frame immediately: the MCP private-file handoff deletes its previous image when another frame is saved.

## 4. Bounded experimental sequence

Run only within an independently qualified safe region. Do not discover mechanical stops by repeatedly driving into them. Numerical command budgets are assigned from qualification, not guessed from the Android preview model.

A. Record stationary stowed baseline, then separately request manual mode and deployment through MCP. Observe every transition continuously, requiring no base translation or rotation. Mode and deploy remain distinct operations. Measure the deployed reference and gripper aperture. No automatic retry on failure.

B. Characterize camera selection at that reference without moving the arm. Record both lenses, switching latency and any pose change. If either required lens cannot be identified, terminate as incomplete rather than omit it from a claimed complete result.

C. For each permitted control, send one command, observe onset and full swept motion, wait for qualified settling, capture pose/jaw state and both stationary robot views, then choose the next command. One in-flight mutation; no held-button timer, transport backlog or unobserved batch.

The exact existing manual candidates are app_arm_move with {m:[2,0,0]}, {m:[1,0,0]}, {m:[0,1,0]}, {m:[0,2,0]}, {m:[0,0,2]}, {m:[0,0,1]}. They are named M3/M4/M5 left/right selections, not angles. Calibrate direction and physical axis rather than assuming a common polarity. Also characterize app_arm_direction_move {direction:1|2|3|4} as up/down/forward/backward and app_open_gripper {} / app_grip {}. Each family receives a separate qualification and model; program commands are not interchangeable.

D. Use forward/reverse pairs and short cumulative sequences inside the qualified envelope to measure reversal backlash, order dependence and pose-dependent displacement. An inverse command is another measured move, not guaranteed undo. Collect five independent repetitions per accepted control/context as an initial sampling target, subject to hard budgets; count accepted observations, not ACKs. Expand only where measured coverage and remaining margin permit.

E. Advance forward one observed step at a time until the arm becomes visible in the front view or the prequalified boundary is reached. Record first visibility, silhouette/marker location and cross-view agreement. Never keep extending merely to satisfy the visibility goal. If no overlap exists inside the safe region, report that requirement unachieved.

F. Measure empty-gripper open/close endpoints, aperture, timing and repeatability across qualified poses. Do not infer proportional aperture, grip force or payload capacity from two commands; no object pickup is included.

G. Where a safe stow path has already been qualified, observe stow then redeploy to test reference repeatability. Target at least five observed cycles for a reference-repeatability dataset, bounded by the authorized budget. Stow ACK invalidates the deployed estimate immediately; confirmed physical stow marks stowed, and a subsequent confirmed deployment establishes the next measured reference.

## 5. Termination and automatic failure behavior

Abort on intrusion, base motion, unexpected mode/pose, freshness failure, required-marker loss, camera/producer change, transport ambiguity, excessive measured displacement, inadequate margin, fault/battery limit, storage failure or exhausted budget. Record an explicit reason and partial artifacts. Do not prompt the user to certify continuation.

On ambiguous command delivery: retain exact job/command/receipt IDs, invalidate state, inhibit new experimental mutations and use the qualified independent stopping path. Passive visual stillness is not proof that a delayed command cannot execute. No retry, inverse movement, automatic stow, mode detour or reconnect-driven recovery.

Normal completion returns through an already-qualified path, observes stow, releases only owned camera resources and preserves records. Abort cleanup does not blindly stow an unknown pose. Final status distinguishes physical termination verified, unverified cessation, incomplete data and accepted calibration.

## 6. Model fitting and independent acceptance

Fit a transition model state_next = f(state, command, direction/history, mode) with separately estimated onset/duration/settling distributions. Start with measured local lookup/interpolation, adding kinematics only where identifiable. Model saturation, backlash, hysteresis, missed steps and uncertainty growth. Constant increments are an acceptable simplification only if held-out evidence supports them.

Use externally measured link/tool pose as truth; do not feed command-predicted joints into camera calibration and then use that fitted camera to validate the same predictions. Standard hand-eye solving requires independent gripper-to-base pose observations and sufficient rotational diversity. Use measured marker/link poses, not the broken query or Android RealPos.[1]

Split train/validation by entire deployment cycles and command sequences, not neighboring video frames. Preserve separately captured check targets or independently measured geometry. Evaluate spatial/orientation/aperture error, reference drift, timing, failure rate, worst observed errors, bootstrap stability and coverage of prediction intervals. Low training reprojection residual alone is insufficient.[1]

Proposed engineering acceptance targets, not established robot capabilities: held-out position p95 <=10 mm and max <=20 mm; orientation p95 <=3 degrees; aperture p95 <=3 mm; reported 95% empirical prediction intervals cover >=95% of held-out observations. These are usability targets, not safety-certified error bounds. Measurement uncertainty must be reported and sufficiently smaller than the requested accuracy. A larger sample is needed for meaningful tail claims; five repetitions is a pilot, not a safety confidence certificate.

Never relax targets automatically to obtain PASS. Underobserved axes, unavailable lens geometry or insufficient sample size receive unsupported/inconclusive fields. Admit a reduced-scope artifact only as explicitly partial, never full calibration.

## 7. Durable artifacts

Produce a private per-job dataset: manifest.json with identities, versions, units, frame conventions, setup geometry and budgets; append-only commands.jsonl with planned/sent/received/unknown phases and exact non-secret payloads; timestamped external video and retained front/gripper frames; observations.jsonl with marker/link/tool/jaw poses, source frames, covariance and rejection reasons; camera intrinsics/extrinsics and latency evidence; arm-transition-model.json; gripper-calibration.json; validation.json and plots; SHA-256 inventory; final report with accepted scope and abort/completion state.

Generated coefficients are real measurements only. Never write plausible default numbers as calibration. Keep raw images, credentials and tokens out of portable docs/source control. Store calibration in durable private service data rather than a cache or session-only directory.

Generate ARM_CALIBRATION.md, ARM_API_GUIDE.md and an agent runbook from the accepted model: command meanings, units, measured context/range, timing, uncertainty, resets, invalidation rules and examples with explicit estimated provenance. The runbook must instruct consumers to stop using invalid state, not silently fall back to zeros.

## 8. RoboClaw integration without counterfeit telemetry

Preserve robot_rpc response semantics: app_get_arm_joints_data still returns the vendor response unchanged. Add an estimated-arm-state read endpoint/tool and a source-labelled estimated section alongside existing raw telemetry. A legacy get arm location wrapper may explicitly select the estimator, but must return provenance/validity rather than an indistinguishable [x,y,z] array.

Suggested fields: source=command_history_calibrated, measurement=false, calibrationId, deviceId, firmwareId, deploymentEpoch, coordinateFrame, units, estimatedJoints (only identified axes), estimatedToolPose, gripperAperture, uncertainty, lastCommandId, lastObservedAt, state=uninitialized|estimating|stowed|invalid, invalidReason. Unavailable numbers are null. Stored stowed state does not masquerade as zero joint coordinates.

Track every service dispatch surface at src/service.ts dispatch before transport issuance, including generic local/cloud RPC and named aliases. Durably persist intent first; distinguish not_sent, received and unknown. ACK may advance a prediction but never becomes measured completion. Invalidate on ambiguous delivery, unknown arm-affecting RPC/program, external-client control, mode change, unqualified deployment/stow, device/firmware/calibration change, restart or epoch loss. Unsupported commands must not silently preserve a trusted estimate.

A physical stow resets the tracking epoch only after accepted visual evidence; deployment initializes from the measured distribution of deployment poses. Persist history across service restart, but restore validity as invalid until a new qualified reference is observed. Counting service commands cannot track vendor-app commands unless external control is excluded or detected; without that guarantee this is explicitly a best-effort estimator, not authoritative pose.

Import the model through an authenticated validated calibration interface: exact device/firmware binding, finite units/transforms, schema/hash verification, accepted validation scope, atomic durable publication and readback. Model installation is passive; do not turn the thin service into the experiment planner.

## 9. Executable implementation order

1. Implement job schema, durable supervisor, strict command admission and independent watchdog/stop integration within existing DimOS ownership. Add ACP job start/status/abort support with bounded authorization. Repair the broken reasoning-only ACP launch separately and verify receipt retrieval without hardware dispatch.
2. Implement local webcam/fiducial measurement and durable synchronized recording. Extend host MCP with owned robot-camera lifetime, lens epoch and retained frame acquisition. Reuse maintained OpenCV/DimOS calibration solvers rather than invent another generic camera solver.
3. Add network-free fake-MCP/replay tests: chassis/program/pickup rejection, unexpected nested payloads, competing writes, unknown dispatch without replay, stale/repeated frames, wrong lens, base movement, watchdog triggering, stow ACK without reset, restart invalidation and exact artifact counts/hashes. Synthetic models must stay labelled synthetic.
4. Qualify hardware stopping, finite movements, clearances, camera side effects and reference repeatability under a separate safety qualification process. This cannot currently be replaced by chat permissions or stock zero-array feedback.
5. Run one explicitly authorized bounded calibration job only after all qualification gates pass. It completes or automatically aborts without user verification prompts.
6. Fit and validate offline, write durable artifacts, import accepted model into RoboClaw, read back its exact calibration identity, and exercise estimated-state behavior against retained held-out data. No automatic promotion of failed/partial candidates.

No runnable calibration command is supplied here because the supervisor, measurement adapter, camera MCP lifecycle and stopping qualification do not exist as a validated combined capability. Implementation completion is proven by offline execution plus the bounded real dataset, not by a fabricated CLI example.

## Sources

[1] OpenCV Camera Calibration and 3D Reconstruction: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
[2] OpenCV Detection of ArUco Markers: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
[3] Roborock arm safety / Emergency Stop: https://support.roborock.com/hc/en-us/articles/48039636271513-Is-the-robotic-arm-safe-Will-it-pinch-me-when-retracting
[4] Roborock arm capabilities / selectable chassis and gripper views: https://support.roborock.com/hc/en-us/articles/48039609125017-What-can-the-robotic-arm-do
