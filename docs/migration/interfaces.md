# ROS interface contract — current owner reference

The phase-1 draft has been superseded. The living guide is [ROS 2](../ros2.md);
[the command/API contract](../thin-api.md) owns shared web/MCP behavior.
Exact fields, enums and goal/result definitions are authoritative in the
[installed-interface source](../../src/z70_interfaces/CMakeLists.txt),
[messages](../../src/z70_interfaces/msg), [services](../../src/z70_interfaces/srv)
and [actions](../../src/z70_interfaces/action). Do not regenerate IDL from this
historical Markdown entrypoint.

## Preserved contract distinctions

- `z70_driver` owns acquisition, command scheduling and command/status-derived
  estimates. Manipulation, mapping, odometry, web and MCP consume ROS interfaces;
  the private vendor transport is not a second estimator or task executor.
- `EvidenceStamp` distinguishes receipt/source time, clock, stream session,
  sequence, validity and staleness. Unknown exposure time stays unknown. Repeated
  publication of a cached sample is not a fresh observation; a saved map pose is
  not live localisation.
- `CommandOutcome` distinguishes logical command UUID, per-attempt UUID/index,
  dispatch, response and physical completion. SENT is not an observed movement;
  an ACK is not completion or cessation. Call-start time is not confirmed wire
  write time. Unknown motion is not automatically replayed.
- One canonical `/joint_states` drives one `robot_state_publisher`, including
  display poses when planning state is unknown. Usable planning state comes from
  typed `ArmEstimate` validity/joints, not TF availability or fresh display stamps.
  There is no separate display TF tree requirement.
- Driver compressed images, CameraInfo and metadata use reliable transient-local
  publication; raw images use reliable volatile depth 2. Match acquisition stamps,
  frame IDs and lens calibration; do not relabel stale pixels after selection.
- Current actions include BasePulse, ArmStep, HandStep, ArmOperation, Pick, Stack,
  DeployArm, AlignToTarget, MoveArm, GraspAndLift and ReleaseAndStow. Exact goals
  and result meaning belong to IDL and the [manipulation guide](../../src/z70_manipulation/README.md).
- Stop, task exclusion/takeover and control-session enablement are implemented
  scheduler behavior, not retired approval/lease policy. Ordinary writes have no
  pending FIFO. Stops do not wait behind an ordinary RPC or camera wait.
- Calibration is ESTIMATED with explicit known/value fields, not OBSERVED or a
  certified covariance. See [resource and durable-state reference](phase3-resources.md).

The former draft's separate display tree, volatile compressed-camera QoS,
incomplete action list and pending-command wording are obsolete. Its pre-cutover
TypeScript/bridge paths were source citations, not present commands. No measured
joint feedback, continuous trajectory controller or navigation action is implied.
