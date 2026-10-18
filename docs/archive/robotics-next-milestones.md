> Historical/superseded architecture. Current contract: [thin-api.md](thin-api.md).
> The service-owned admission, recovery and workflow policies described below have been removed.

# Remaining embodied autonomy milestones

The current working tree is a validated software foundation, not a complete autonomous robot. Active deployment remains unchanged pending final review and supervised deployment readiness.

## Implemented foundation

- ROS 2 Jazzy ARM64 local container, observation driver, record/replay.
- Authenticated versioned read-only API, numeric sanitization, coalesced connected-only reads and conservative admission.
- Typed outcome reporting through service, MCP and browser; no automatic replay after ambiguity.
- Browser-supervised stow/confirm/dock/recovery workflow and persistent quarantine.
- Pure command arbiter with durable serialization contracts (not yet shared production authority).
- Simulation-only ROS bounded actions and executor STOP/cancel saturation tests.

## Next coding slices

1. Production action authority: browser/MCP/ROS robot writes share persistent service authority, genuine writer leases, device/session binding and durable goals. Trusted measured completion can settle work without operator confirmation; manual recovery remains fallback. ROS REAL-mode/service integration is verified against fake hardware in Docker. Next implement calibrated stock-Z70 completion evidence and validate lifecycle behavior on hardware.
2. Reconcile capabilities: preserve current observation envelope while exposing an explicitly versioned optional action capability extension. Do not replace observation capabilities with the incompatible simulation fixture contract. Update both client validators and fixture integration tests together.
3. Local camera publication: explicit session start/stop, media transport, frame timestamps, no images in JSON/MCP data plane, no implicit viewing from observation subscriptions. Native camera can disturb arm mode; test lifecycle conflicts before hardware use.
4. Native inference service: benchmark one local vision model on Apple GPU, with bounded concurrency and memory. Keep semantic outputs untrusted; record model latency separately from geometric tracking/control.
5. Measured state estimation: camera calibration, verified frame selection, base relative motion, visual arm/gripper states. Never emit measured joint angles/odometry from command values or zeros.
6. Deterministic one-object pickup skill: evidence-driven transitions, explicit failure reasons, bounded retries only for provably unexecuted operations, visual result verification and recovery.
7. Navigation: validate map/pose/frame and native navigation cessation before Nav2 integration or production target dispatch.
8. MoveIt: isolated runnable synthetic Jazzy/OMPL planning simulation now passes reachable, unreachable, joint-limit and collision checks (`robotics/moveit_sim/README.md`). Replace synthetic geometry with measured model/feedback before any hardware trajectory execution.

## Deployment and hardware gates

Before service restart inspect current robot/controller state through the approved robot tools and obtain confirmation of physical stationary/stowed condition if telemetry cannot establish it. Preserve private state/configuration, prepare rollback image, and do not change motion permissions. Initial new ROS connection is read-only; it must not implicitly enable camera or actuation.

Hardware tests require nearby operator and physical stop. Establish stow timing, mode-exit readiness, dock cancellation, disconnect behavior and actual cessation independently of RPC ACK. External vendor-app writers must be excluded during supervised trials. No unattended autonomy claim until physical safeguards and closed-loop outcomes are validated.
