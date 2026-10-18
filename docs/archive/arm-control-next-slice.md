# Arm control decision after primary Android trace

The supplied Hermes artifact resolves the active manual path: three direction
selections for M3/M4/M5, not absolute angles. Program `arm_joints` uses four
direction selections including the gripper and a repetition count; program
`arm_move` instead uses an end-effector direction plus repetitions. The five-input
live wrapper has no identified caller. The program model's `RealPos` is predictive.

Primary evidence and exact references:
- `arm-joint-control-research.md`
- `arm-camera-mode-research.md`
- `arm-program-research.md`

## Implemented corrections

- `src/arm-joint-protocol-research.ts` preserves primary directional payloads as
  offline fixtures, with regression checks for the program-name discrepancy. It
  is not wired into RobotClient or an action endpoint.
- The mechanical-camera service handler no longer falls back from an unsupported
  `app_change_camera` to setting bit 15. Android's settings serializer does not
  establish that equivalence. Unsupported is an explicit rejection; timeouts and
  missing ACK remain uncertain.
- README's former five-input-manual-UI claim is corrected.

## Recommended hardware sequence

1. Observe fresh robot state, arm mode/action and one camera image through MCP.
2. Characterize selector-only camera switching in manual mode with one established
   retained viewing session. Keep mode, selection feedback and identifiable lens
   content separate. Do not reopen the producer as part of the selector test.
3. Establish a physical reference view for arm movement and stopping. Before
   exposing joint stepping, trace and independently validate the manual stop
   handler's `app_stop_grasp` behavior; its name alone is not generic motor-stop proof.
4. Add one exact named M3/M4/M5 selection at a time, followed by physical
   measurement. Determine displacement/repeatability and compare joint query
   responses. No repeated 100-ms UI loop or command backlog is needed for discovery.
5. Evaluate programs only after individual steps and program cancellation are
   characterized. Runtime result success is a vendor execution report, not an
   independently measured pose or a queue-drained certificate.

The existing service has no calibrated completion source on hardware. Independent
observations and explicit recovery are needed for unresolved experiments; repeated
ACKs or telemetry zeros must not bypass command authority.

## ROS consequences

A visual incremental controller can eventually map task-relative corrections to
bounded named steps and observe each result. This would be a vendor-step action
interface, not `FollowJointTrajectory`. A vendor program action could separately
report sequence progress, with measured completion kept distinct. Both preserve
the autonomy goal without inventing absolute joint feedback.

Real MoveIt execution still requires measured pose and a supported controller.
The Android predictive model can inform a separately labelled model experiment,
but must never be installed as measured joint states. Camera access does not
require an app-side mode-2 transition; direct 1↔2 tab switching exists but firmware
deployment/reset and pose preservation remain unknown.

No hardware movement, mode transition or lens switch was performed for this slice.
Source/test changes require deployment before they affect the running service.
