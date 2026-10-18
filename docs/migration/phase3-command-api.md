# Phase 3 command API

Interface definitions are compiled and exercised offline. Normative actions
match interfaces.md: BasePulse `/z70/base/pulse`, ArmStep `/z70/arm/step`,
HandStep `/z70/arm/hand_step`, ArmOperation `/z70/arm/operation`. Manipulation
hosts the normative Pick action on `/z70/pick`.

Services: Stop `/z70/stop`, SetArmAnchor `/z70/arm/anchor`,
RawRpc `/z70/driver/raw_rpc`, TaskSession `/z70/tasks/session`.
RawRpc request: method string<=128, params_json string<=16384, timeout_ms
uint32, transport uint8 (LOCAL=0/CLOUD=1), takeover bool, task_id UUID. Response: outcome CommandOutcome,
result_json string<=16384. Empty params_json means [].
TaskSession request: begin bool, task_id UUID, takeover bool. Response:
accepted bool, task_id UUID, reason string<=64. Begin with zero UUID allocates
an ID. Begin with matching active ID is idempotent. End requires matching ID;
it revokes future token writes and publishes UUID on `/z70/tasks/cancel`.
Manipulation subscribes to cancellation and cancels its executor.

One driver dispatcher serializes ordinary writes; no pending FIFO. Manual writes
are BUSY during Pick. Task RawRpc writes require the current token. Takeover
revokes and publishes cancellation, then completes priority ALL stop before new
ordinary writes. Old-token writes remain BUSY. Failed stop outcomes latch
ordinary dispatch disabled; an explicit successful stop/takeover clears it and
failed takeover never dispatches the requested new command. Stop never waits on the ordinary
dispatcher or camera lock, including while an HTTP command reply is stalled.

Driver sends POST `/api/driver/rpc` body {method, params, timeoutMs, transport,
commandId}. It allocates the logical UUID before HTTP; backend preserves it
literally and replies with RpcOutcome plus commandId/attemptId/attemptIndex.
Backend attempt UUIDs are shared with `/api/commands/events`; no duplicate
driver-created wire events. HTTP loss retains logical identity, UNKNOWN dispatch
and zero attempt UUID when no backend attempt evidence was received.
POST `/api/driver/camera/select` body {camera: front|gripper} never deploys or
enters pickup mode. Existing observation routes remain. No GET /api/arm/estimate:
driver owns ArmEstimator; default persistence is
$XDG_DATA_HOME/RRsZ70_ROS2/arm-estimate.json (fallback ~/.local/share), parameter
arm_estimate_path='' disables persistence for tests.

Stop BASE=1 sends app_rc_stop then app_rc_end; ARM=2 app_stop_grasp; ALL=3
all three. Each reply budget <=5s, total <=15s including backend safe retry.
Cancellation revokes future writes immediately and attempts priority ALL stop.
No automatic stow/open/put-down, no unknown motion replay, no cessation claim
from an ACK. Low-level success is received non-rejected response only.

CalibrationState uses the coordinated full per-direction known/value gain,
uncertainty/count and camera-offset schema in phase3-resources.md. The initial
linear_scale/angular_scale draft was replaced before compilation. Odometry and
manipulation consume that generated type; source timestamps/identities survive
unchanged republication. No calibrated covariance or physical accuracy is inferred.

ArmStep direction +1 maps to vendor selection 1, -1 to selection 2 at the
selected M3/M4/M5 input; nominal vendor steps, not measured angle changes.
Action budgets reserve up to 15s for deadline stop (10s for base natural stop),
deliberately reducing ordinary reply waits rather than extending total bounds.

## Offline verification

Pure scheduler suite: 5 passed (Pixi Python, plugin autoload disabled).
Production driver and test Python syntax compilation passes.
`test_command_api.py` runs real generated ROS actions/services over DDS in domain
92 against an isolated loopback ThreadingHTTPServer: action and event UUID joins,
no duplicate events, input rejection, vendor rejection/unknown outcomes, hanging
RPC stop/cancel priority, task token conflicts/revocation/takeover ordering,
base pulse mappings, all-stop 15-second budget, estimator observation once per
RPC, non-moving anchors, and generated message serialization.

The parent completed estimator/backend/manipulation integration and exercised
these DDS tests using a fresh non-symlink install. Added real Pick cancellation
under stalled images, revoked-task cleanup ordering, real display/planning TF,
advanced cloud stop-path coverage and standard ROS CLI inspection. See phase3.md
and phase3-ros-final.txt for the final full-suite result. Example rerun:

```
cd ros2
/Users/mpentax/.pixi/bin/pixi run bash -c 'source install/setup.bash && export ROS_DOMAIN_ID=92 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 && python -m pytest src/z70_driver/test/test_dispatcher.py src/z70_driver/test/test_command_api.py src/z70_driver/test/test_shared_driver.py -q'
```

No hardware completion, camera throughput, motor cessation, or successful pickup
is claimed from these offline tests.