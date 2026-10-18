# Shared camera selection and validation

Selection is one shared robot mode, not per request. Use MCP `select_camera`,
ROS `/z70/camera/select` (`z70_interfaces/srv/SelectCamera`) or authenticated
`POST /api/ros/camera/select {camera:"front"|"gripper"}`. The old
`/api/camera/select` is retired. Selection does not deploy the arm or enter a mode.

MCP `get_camera_frame({})` follows selection; `{camera:"gripper"}` only filters
shared ROS cache and **does not switch**. No automatic front restoration exists.
The driver alone acquires JPEGs. Supported selectable views are front and gripper;
this is not a claim that the robot contains only two physical cameras.

## Authorized live check

Use the matching sidecars/ROS driver and an authorized supervised setup. If
starting isolated observation, choose a distinct DDS domain, free Foxglove port
and test-owned XDG data; stop only processes you own. Camera switching and arm/
mode setup have live effects, not permission implied by this document.

1. Record shared camera state, source timestamps and current selection. Do not
   reset a valid gripper selection merely to establish a preferred baseline.
2. Read a shared frame and save its original bytes/identity. Front pixels are
   1600×1200; gripper pixels are 1600×896. Check the actual viewpoint, not just tags.
3. For a gripper check, establish pickup/control mode with the arm visibly out
   under the session's authorization. Firmware camera admission cannot be inferred
   from an ACK or a single common_status bit; observe readiness.
4. Explicitly select gripper. Check the generated selector response **and** fresh
   camera status bit 15 (`get_camera_status`) plus gripper pixels. The selector
   often has no vendor reply: bounded timeout followed by a status flip may confirm
   selection while the underlying RPC outcome remains UNKNOWN. Do not replay it
   solely for a missing ACK. No flip/rejection is an actionable failure, not an
   instruction to deploy automatically.
5. Sample shared frames for at least ten warm seconds. Gripper topic stamps/counts
   should advance; front retains its previous sample. The driver target is 3 Hz,
   not a delivered-rate promise. Read CameraInfo/metadata for the same lens/stamp.
6. Explicitly select front and repeat. A camera-filtered frame read must never
   send a selector or restore another lens. Where official-app changes are in
   scope, verify the shared state and topic routing follow them without corrective
   writes from polling.
7. During switching, missing/stale/mismatched frames are data gaps; mislabeled
   pixels are a defect. Correct the first failing boundary. Do not turn ordinary
   gaps into blind selector retries or a general safety abort.

Compressed images, CameraInfo and metadata are transient-local; an inactive lens
can still appear in Foxglove. Raw images are not latched. Compare source receipt
stamps and frame/session identities, not merely panel visibility. Both lenses
have installed calibration; dimensions alone do not establish calibrated validity.

## Evidence and cleanup

Record selector latency/outcome IDs, independent status bit, JPEG dimensions and
viewpoint, metadata calibration identity, and per-topic delivered counts/stamps.
Source exposure time is unknown; receipt age is not sensor age. Retain evidence
under a new session directory, never overwrite calibration originals.

Report final camera/control/physical state and stop only owned validation
processes. Front selection, mode exit and stow are explicit authorized operations,
not frame-request cleanup. Stop control & streams suspends shared acquisition;
inspect `streams_released` and stop outcomes rather than assuming physical cessation.
See [API contract](../thin-api.md), [throughput](camera-throughput.md) and
[preview-rejection incident](camera-preview-start-rejection.md).
