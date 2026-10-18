# Command contract — current owner reference

Use [the current command/API guide](../thin-api.md) and [ROS guide](../ros2.md),
not migration phase ordering. Exact generated interfaces live in
[z70_interfaces](../../src/z70_interfaces/CMakeLists.txt); implemented ownership
is in [commands](../../src/z70_driver/z70_driver/commands.py) and the
[dispatcher](../../src/z70_driver/z70_driver/dispatcher.py).

## Semantics preserved from the cutover

- One dispatcher, no ordinary pending FIFO. Manual commands conflict with an active
  task; task UUIDs identify current execution, not a user permission/lease.
- Takeover revokes/cancels old execution, then completes priority ALL stop before
  dispatching replacement work. Revoked-task cleanup cannot stop its replacement.
  Current stopped-session, busy and unverified-stop states can reject ordinary
  writes; private transport permissiveness does not override ROS scheduling.
- BASE stop sends `app_rc_stop`, then `app_rc_end`; ARM sends `app_stop_grasp`;
  ALL attempts all three. The response budget is at most 5 seconds per command,
  15 seconds total including allowed backend retry, not a physical-stop guarantee.
  Cancellation does not auto-stow, open or put down.
- Driver allocates logical command identity before private HTTP dispatch. Backend
  attempt identities join the event/result stream; do not count both as separate
  physical commands. Lost HTTP response can leave UNKNOWN with no known attempt ID.
- Advanced raw RPC uses this same dispatcher, event stream and estimator. No direct
  client-to-vendor fallback or blind replay of uncertain motion exists.
- `arm_estimate_path` defaults to
  `$XDG_DATA_HOME/RRsZ70_ROS2/arm-estimate.json` (home `.local/share` fallback);
  empty string disables persistence for tests. Restored state needs reconciliation.
- ArmStep ±1 maps to live `app_arm_move` selection 1/2, not a signed wire value.
  Nominal joint/hand steps are command-model semantics, not measured completion.

## Historical offline verification

The phase-3 worker reported five pure scheduler tests and real generated-interface
DDS tests against a fake loopback backend (domain 92), followed by parent installed
integration. These were software checks, not hardware completion, camera throughput,
cessation or pickup evidence. Full scope/counts remain in [phase 3](phase3.md).
Historical output `docs/migration/phase3-ros-final.txt` is unavailable. The retired
`cd ros2` rerun recipe is removed; current verification commands belong to the root
manifests and current ROS guide.
