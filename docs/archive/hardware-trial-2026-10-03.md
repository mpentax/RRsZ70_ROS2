# Deployed undock trial — 2026-10-03

Owner explicitly authorized deployment, supervised undocking and camera feedback.
All robot interactions used configured RoboClaw MCP tools; shell was used only
for Docker deployment and source inspection. No direct HTTP robot-control fallback.

## Sequence and observations

- Preflight: connected L01, battery 100%, charging/state 8, no fault/cleaning/return,
  arm mode off/not-started, no unresolved command authority work.
- Deployed current tested source with `docker compose up -d --build`; health passed.
- Captured/viewed a 1600x1200 baseline. Exposure time and physical lens identity
  remained unknown. Image had substantial overexposure; no metric clearance inferred.
- Requested exactly one MCP `undock {durationMs:300}`. Service returned HTTP 502
  and unknown execution. Diagnostics show fast RPC responses, but did not expose
  the failing command/ACK category. No second undock or motion pulse was requested.
- Fresh telemetry changed to state 39, arm off/not-started, no error. Shared
  authority retained one unresolved action. This is not proof of departure or cessation.
- Requested independent chassis STOP once; service reported its stop paths settled,
  while physical completion remained unverified.
- Subsequent viewed camera image showed a substantially different scene geometry
  and floor perspective. This supports possible physical movement, not a calibrated
  displacement, verified departure or independently measured stationary state.
- A later camera observation failed. Fresh arm observation still reported state 39,
  off/not-started and joint array [0,0,0]. No camera retry loop was used.

## Result and blocker

Deployment succeeded; hardware undock did not yield a valid command ACK outcome.
Possible movement was observed and independently requested STOP was sent.
The durable unresolved record is intentionally retained; no journal deletion,
restart clearance, motion-policy change or fabricated measured-completion source
was used to bypass it. Arm deployment/joint/camera-selector actuation was not attempted.

No trusted hardware controller-terminal or sustained measured cessation source is
implemented. The onboard image does not show the arm or exclude delayed execution.
The existing explicit browser recovery route remains the only installed clearance
path; agent control cannot supply its operator assertion. That limits continuation
of this hardware trial even with general permission to test.

## Software follow-up

Added bounded `ChassisCommandError` categories to identify the command stage and
distinguish RPC failure from a non-ACK response without logging vendor payloads.
The service records only fixed command/category labels for future undock failures.
This diagnostic update is source-only pending deployment and does not retry the trial.

New `arm_joint_step` and `stop_arm_grasp` registrations also require MCP harness
reload before appearing in the running agent's tool list. Service routes are
deployed, but this agent session still has its original discovery list.
