# Docker ROS → TypeScript service end-to-end smoke

Run with Node 24+ and Docker Desktop, using the existing
`roboclaw-ros:jazzy-actions-contract` image:

```sh
node deploy/test-robotics-service.ts
```

The runner starts **actual `createService` production endpoints** on
`0.0.0.0` with an ephemeral port and exact Host allowlist. The image's installed
ROS action adapter runs in **REAL mode**, explicitly opted in, and an actual
`rclpy` action client communicates through ROS/DDS. HTTP crosses Docker Desktop
via `host.docker.internal`; no Python contract server is involved.

Each scenario uses a fresh Node process, private temporary state/config/token,
real service journals, and a connected injected `ServiceRobot`. All robot reads
and supervised commands are fake. Camera, vision and cloud dependencies reject
external I/O. The fake reports manual/waiting arm status and acknowledges only
gripper open/close. No saved project credentials or physical robot are used.

The token is passed into Docker on stdin and written inside the container as a
uid10001-owned `0600` file. The test script is bind-mounted read-only; the
container runs as uid10001 with dropped capabilities. Service state and the
container are removed on completion or failure.

## Assertions

- Authenticated capabilities retain the versioned read-only envelope, empty
  baseline commands and optional non-simulation action extension; state contains
  fresh fake observations without physical-stow claims.
- Missing authentication and untrusted Host are rejected.
- ROS open and close succeed only with injected trusted synthetic measured
  completion evidence. Command ID, operation, device/session and writer lease
  correlate across HTTP admission, polling and terminal ROS results.
- ROS duplicate goals and independent HTTP duplicate submissions reject.
- Stale device-session submit, status and cancel requests reject with HTTP 409.
- A separate ACK-only service process reports ROS `unknown`/aborted,
  `vendorAck=true`, `verifiedSuccess=false`, `cessationVerified=false`.
- The host verifies exact fake dispatch counts (two measured, one ACK-only),
  distinct scopes/command IDs and trusted observer calls.

Individual scenarios can be run with `measured` or `ack-only` as the runner's
argument. Failures print service action responses and fake dispatch/evidence
diagnostics without printing the token. These are synthetic completion tests,
not physical hardware validation.
