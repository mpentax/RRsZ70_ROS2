# Manual joint and retained-camera trial interface

Implemented service/MCP operations:

- `arm_joint_step` -> `POST /api/agent/arm/joint-step {control}`. Exact six
  M3/M4/M5 button names, one `app_arm_move {m:[...]}`, fresh state 39,
  manual/waiting feedback and device-bound arm permission. No raw arrays,
  duration, repetition loop, angle, or automatic mode entry.
- `stop_arm_grasp` -> `POST /api/agent/arm/stop-grasp {}`. One
  `app_stop_grasp []` through shared authority's independent stop admission.
  Reserved transport capacity permits issuance alongside occupied telemetry.
  Arm policy and current connected manual/mode-2 feedback remain required.
  Future arm steps are cancelled. Motor cessation is explicitly unverified.

All requests retain unknown execution across missing ACK, transport error and
restart. Stop does not clear earlier command uncertainty or substitute for
chassis stop. The controller's stop permit is independent and bounded while an
earlier action settles, and tests cover both completion orders.

## Trial sequence

1. Read live state and observe the arm through MCP. Establish a stationary
   off-dock trial location before deployment. Mode feedback is not physical stow.
2. Retain one owned stream with `hold_camera_stream {enabled:true}` and capture
   baseline content before arm deployment, avoiding implicit producer restarts.
3. Deploy manual mode using the existing finite action, then independently observe
   its physical result and resolve its outstanding authority record.
4. Use `set_mechanical_camera {enabled:true}` once while keeping manual mode and
   the retained stream. Capture recognizable fresh content and inspect mode/action
   again. A camera flag alone is not lens verification. Unsupported selector
   never falls back to `set_camera_status`.
5. Only after stop coverage and physical workspace are characterized, request one
   named joint operation. Measure result from an independent reference; compare
   raw feedback without interpreting zeros as angles. On uncertain motion use
   independently available stop and physical stop, observe before further work.
6. Restore/release only the viewing session this trial owns, after arm execution
   has ceased. No automatic stow, mode detour, retransmission or recovery is added.

Fake service tests verify selector-only operation in manual mode with a retained
stream, no arm mode transitions or fallback settings, named payload bounds,
shared authority exclusion and stop availability. Transport tests verify priority
admission. These tests do not establish hardware motion limits or stopping.

Deploy source changes and reload the MCP harness before the newly registered
tools appear. No hardware actuation was performed by this implementation slice.
