> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

The Docker ROS packages, build commands and action policy below are retired. Dated publication/MCAP results remain reported history, not evidence that those runners/images exist today.

Temporary/scratch paths below are historical ephemeral references, not retained portable outputs; their original contents are unavailable from this checkout and were not reconstructed.

## Historical record

# Read-only ROS 2 foundation (Jazzy / Ubuntu 24.04 / ARM64)

This is an **opt-in observation adapter**, not a robot controller. It reads two
authenticated HTTP endpoints; it implements no direct vendor RPCs and cannot start a camera,
move the chassis/arm, create leases, or change the active deployment. No real
robot connection is needed for its tests. Native macOS inference remains outside
the ROS Linux VM; this image includes no inference runtime.

The foundation uses maintained upstream ROS 2 Jazzy, `rclpy`, `std_msgs`,
`diagnostic_msgs` and `rosbag2` instead of implementing a custom middleware or
pretending this consumer robot has a measured joint/odometry interface.

## HTTP contract (aligned to `src/robotics-contract.ts`)

Only `GET /api/robotics/v1/capabilities` and `GET /api/robotics/v1/state` are
permitted. Responses must have JSON content type and a versioned envelope.
The API worker's v1 contract uses `apiVersion: "1.0"` and Unix milliseconds:

```json
{
  "apiVersion": "1.0",
  "kind": "capabilities",
  "generatedAt": 1790726400000,
  "data": {
    "readOnly": true,
    "endpoints": {
      "capabilities": "/api/robotics/v1/capabilities",
      "state": "/api/robotics/v1/state"
    },
    "observations": [
      "status",
      "armJoints"
    ],
    "commands": [],
    "cameraAccess": false,
    "backgroundPolling": false,
    "physicalStowVerification": false,
    "unknowns": [
      "physical-arm-stow",
      "joint-units-and-calibration",
      "robot-pose",
      "dock-contact"
    ]
  }
}
```

```json
{
  "apiVersion": "1.0",
  "kind": "state",
  "generatedAt": 1790726400000,
  "data": {
    "connection": "connected",
    "raw": {
      "status": {
        "value": {
          "battery": 72,
          "state": 8
        },
        "observedAt": 1790726399000,
        "source": "robot-rpc",
        "freshness": "fresh",
        "ageMs": 1000,
        "error": null
      },
      "armJoints": {
        "value": {
          "unverifiedVendorValue": 0
        },
        "observedAt": 1790726399000,
        "source": "robot-rpc",
        "freshness": "fresh",
        "ageMs": 1000,
        "error": null
      }
    },
    "physicalArmStowed": null,
    "unknowns": [
      "physical-arm-stow",
      "joint-units-and-calibration",
      "robot-pose",
      "dock-contact"
    ]
  }
}
```

**Contract semantics:**

- `generatedAt` is response generation time in Unix milliseconds, **not**
  measurement time. The bridge never substitutes it for `observedAt`.
- Each `raw.status` / `raw.armJoints` observation carries `value`, `observedAt`,
  `source`, `freshness`, `ageMs`, `error`. `observedAt` is the service receipt
  timestamp or null, not a independently verified device measurement time.
  `freshness` is `fresh` or `unavailable`; `source` is `robot-rpc`. An unavailable
  observation has null value/time/age; `error` is null or `READ_FAILED`.
- The service performs sequential on-demand vendor reads (`get_status`,
  `app_get_arm_joints_data`) only against an already connected session, with
  shared per-robot coalescing and writer-priority admission checked before each
  read. It never reconnects or queues observation reads behind a backlog.
  **Live ROS polling still causes read RPCs** and requires explicit authorization.
  The service caller deadline is 2.5 seconds; a timed-out/abandoned caller does
  not cancel transport work, which remains globally accounted until settled.
  Each already-admitted transport response has a 5-second bound (two sequential
  reads can therefore occupy accounting for up to about 10 seconds); no new
  poll creates parallel outstanding work. Check these budgets against the final
  service implementation before live integration.
- `raw` is sanitized vendor data, not measured ROS joints. Unknown calibration,
  units, pose and dock contact remain explicit unknowns; `physicalArmStowed`
  remains null. The fixture's `unverifiedVendorValue` is synthetic, not a real
  vendor key or validated joint angle. No ROS JointState, odometry or tf is
  inferred from these fields, commands, targets or acknowledgments.
- The service sanitizes upstream values; the ROS client additionally projects
  envelopes to known v1 fields before publishing. Additive envelope/data/raw/
  observation fields and unknown semantic labels are accepted but dropped;
  they are not implicitly privacy-approved. Status values use closed numeric
  fields (boolean cleaning/return flags permitted) and a narrow numeric firmware
  version grammar. Arm data permits bounded numeric/boolean trees only, with
  suspicious key names filtered. Vendor numeric key names still require human
  privacy review before recording; no arbitrary string telemetry is published.
- Validation requires the connection enum, `physicalArmStowed: null`, empty
  commands, camera/background/physical-stow verification disabled, the known
  observation list/endpoints, closed error codes, and both observations. Fresh
  observations require connected state, non-null time/value/age, null error,
  `observedAt <= generatedAt`, and age matching the timestamp difference within
  0.001 ms. Unavailable observations require null value/time/age. Contradictory
  envelopes are rejected and cannot produce OK diagnostics. A different version
  is rejected. Diagnostics conservatively consider both wall-clock age and
  declared age, and never label unavailable connections OK.
- Errors use HTTP status codes and existing HTTP bearer authentication, not a
  browser cookie flow. Error bodies are never ingested/logged. Redirects are
  rejected; bearer credentials never follow them.

The fixture files define this offline schema. Contract alignment was checked
against the sibling worker's source, not by contacting a live service. Recheck
against its finalized implementation/tests before deployment.

## ROS behavior

The `roboclaw_readonly` ament Python package provides:

- `robotics/state`: `std_msgs/msg/String`, compact versioned JSON, reliable,
  volatile, depth 1. Publishes only on successful HTTP fetches. Stale
  observations retain their original timestamp/flags; never publish a synthetic
  fresh observation on a failed request.
- `robotics/capabilities`: `std_msgs/msg/String`, reliable, transient-local,
  depth 1. Checked at startup and at most every five minutes thereafter.
- `diagnostics`: `diagnostic_msgs/msg/DiagnosticArray`, data-free failure codes,
  age, and HTTP failure count. ROS header time is bridge diagnostic publication
  time, not the vendor observation time.

No `JointState`, `Odometry`, `/tf`, camera topics, command subscriptions, ROS
services or actions exist. Standard ROS diagnostics plus versioned JSON are
intentional initial interfaces; add typed messages only after measurement
semantics and source timestamps are verified.

Default state interval is 10 seconds (minimum 5), request timeout 4 seconds (above the service’s 2.5-second caller deadline),
maximum accepted response 1 MiB. Failures back off 20, 40, 80, 120 seconds at the
default interval, reset after success. A single timer/executor serializes
requests; no parallel polling. Effective cadence includes request duration. No immediate retries or tight polling occur;
keep the 10-second default (minimum 5 seconds) rather than reducing the interval
to compensate for unavailable or deadline-limited reads.
Observations older than 30 seconds, partially unavailable observations, absent observation
time and forward clock skew produce warnings. Diagnostic messages are emitted
on each attempted poll, not continuously during backoff. Monitor diagnostic
receipt age as well. HTTP transport success does not mean fresh robot data.

ROS parameters: `base_url`, `token_file`, `host_header`, `poll_interval`,
`timeout`, `max_observation_age`. URL/token/header configuration is read at
startup; the token file content is re-read for each request to support rotation.
Use wall-clock-synchronized hosts for freshness decisions; replay timestamps
retain historical vendor meaning, while diagnostics use ROS publication time.

## Offline tests (no ROS or robot needed)

```sh
python3 -m unittest discover -s robotics/tests -v
python3 -m compileall -q robotics/ros2/src robotics/fixture_server.py
python3 robotics/fixture_server.py --port 57020
```

The fixture binds only localhost, suppresses access logging, serves static,
historical synthetic data and implements no robot calls. It is not a
production auth service. Tests cover real localhost HTTP, allowed GET endpoints,
URL/header validation, envelope/freshness validation, polling/backoff, token
files, secret-free errors, refused redirects, bad JSON and oversized responses,
malformed capability/connection/physical-stow/error invariants, contradictory
timestamps and ages, unavailable null fields, and additive-data privacy filtering.

## Build / opt-in use

Run from the repository root:

```sh
docker build --platform linux/arm64 -t roboclaw-ros:jazzy-local robotics
# Does not start or restart any service:
ROBOCLAW_ROS_TOKEN_PATH=/absolute/private/http-token \
  docker compose -f robotics/compose.ros.yaml --profile ros config --quiet
```

`ros:jazzy-ros-base-noble` is the upstream Ubuntu 24.04 ARM64 base. The initial
build uses a floating distro tag, not a claimed reproducible digest; production
hardening should lock an independently verified digest and apt package versions.
Colcon runs sequentially. The runtime is non-root (UID 10001), read-only with
bounded tmpfs, dropped capabilities, no published ports, no host network, no
camera devices and no robot state-directory mount. Docker limits the standalone
driver to 512 MiB/one CPU; allow image-build overhead within the host's ~4 GiB
Docker allocation. No GPU/inference dependencies are installed.

**Before any live start:** verify service route/schema/auth implementation and
bounded read-only behavior, then explicitly authorize the read-only connection. Prepare
a file containing only the existing **HTTP service token**, not a robot local key
or credentials JSON. Keep it private on the host, outside this repo and backups;
Docker Compose bind-backed secrets must be readable by UID 10001 in the container.
Check permissions without printing its contents. Docker Desktop file-sharing/
permission behavior is host-dependent. Do not loosen a directory containing
other credentials to make it readable. If token transfer requires a dedicated
copy, manage that copy privately and rotate/delete it when no longer needed.
Never put token values in commands, environment variables, logs or bag metadata.

```sh
# Explicit opt-in; never combine with the active deployment Compose file:
export ROBOCLAW_ROS_TOKEN_PATH=/absolute/private/http-token
export ROBOCLAW_ROS_BASE_URL=http://host.docker.internal:57010
export ROBOCLAW_ROS_HOST_HEADER=127.0.0.1:57010
docker compose -f robotics/compose.ros.yaml --profile ros up -d --no-build driver
# Stop only the opt-in driver:
docker compose -f robotics/compose.ros.yaml --profile ros stop driver
```

The Host header can match the app's existing public-origin host while TCP goes
to `host.docker.internal`. Do not change the application's allowed hosts merely
to make this example work. Docker Desktop reachability of a host-loopback
published port must be verified separately; failure should remain a transport
warning, not justify widening network exposure. A separately authorized private
Docker network and `http://app:57000` are alternatives, but are deliberately not
created/attached here. HTTP is intended for the private local Docker path; use
HTTPS for untrusted networks. Proxies from the environment are disabled.

Default `ROS_DOMAIN_ID=70` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` keep DDS
inside the container/network namespace. A Mac ROS process is not automatically
a DDS peer across Docker Desktop. Initial inspection/recording uses `docker exec`
in the same container; do not expose DDS or use host networking merely to see
these topics. ROS middleware is not an authorization boundary: isolate bag
replay from controllers and untrusted peers.

## Recording and replay

The image includes rosbag2 and MCAP storage. Recording is opt-in; snapshots may
still contain private operational data even after sanitization. Review the
server allowlist first and protect recordings/backups. Never record `/rosout`
or indiscriminately record all topics.

Mount a **separate** private recording directory when creating a driver container
(for example a local Compose override adding `./private-recordings:/bags:rw`,
resolving the path relative to the first Compose file). Ensure UID 10001 can write
there; do not mount the app credential volume. The baseline Compose file has
no recording mount. For a driver started with the reviewed recording mount:

```sh
# Run in a separate terminal, interrupt with Ctrl-C to close the bag cleanly.
docker compose -f robotics/compose.ros.yaml --profile ros exec driver \
  ros2 bag record --storage mcap -o /bags/readonly-session \
  /robotics/state /robotics/capabilities /diagnostics
# Inspect without a running robot/application:
docker run --rm --network none -v /absolute/private-recordings:/bags:ro \
  roboclaw-ros:jazzy-local ros2 bag info /bags/readonly-session
# Replay ONLY in an isolated container; no driver, HTTP token, app network, or controller.
docker run --rm --network none -e ROS_DOMAIN_ID=71 \
  -v /absolute/private-recordings:/bags:ro roboclaw-ros:jazzy-local \
  ros2 bag play /bags/readonly-session --clock
```

An isolated replay container needs its analysis subscriber in the same network
namespace (e.g. via `docker exec`) or an explicitly isolated test network. A
transient-local capability subscriber may need rosbag QoS overrides for a
particular recording; verify with `ros2 topic info --verbose` rather than
assuming replay QoS. JSON fields carry the authoritative vendor timestamps;
`--clock` only controls ROS playback time and cannot make a historical state
fresh. State topics contain observations, never executable commands. Do not
connect replay to any planner/controller or infer calibrated geometry from bags.

## Offline container smoke test and evidence

```sh
docker run --rm --network none --read-only --memory 512m --cpus 1 \
  --cap-drop ALL --security-opt no-new-privileges:true \
  --tmpfs /tmp:rw,nosuid,size=64m,uid=10001,gid=10001,mode=1777 \
  --tmpfs /home/rosdriver/.ros:rw,nosuid,size=16m,uid=10001,gid=10001,mode=0700 \
  -e PYTHONDONTWRITEBYTECODE=1 -v "$PWD/robotics:/offline:ro" \
  roboclaw-ros:jazzy-local python3 /offline/tests/ros_smoke.py
```

This test launches the driver and fixture server inside the same network-disabled
container, subscribes to the three ROS topics, checks stale diagnostics and
unknown physical stow, records an MCAP bag, stops the driver, and replays into an
observation-only subscriber. The temporary bag is discarded with the container.
An initial non-root launch exposed root-only resource permissions inherited
from host files; the Docker build now explicitly makes installed package assets
readable. No app service, camera, credential or real robot is involved.

### Hardened validator verification (2026-09-30)

- Offline Python suite: **10 passed, 0 failed**, including malformed v1 safety
  invariants, additive-field filtering and conservative diagnostic tests.
- Rebuilt `roboclaw-ros:jazzy-local` for `linux/arm64`; non-root UID/GID
  `10001:10001`. Final image ID:
  `sha256:4bad9292b86b39391bb3ca6fcb907f04abe2b3ddc62b035b3620193f4f87cc4d`.
- Repeated the above network-none, read-only, capability-dropped 512 MiB/one-CPU
  smoke on that image: Jazzy launch, all three topics, stale diagnostics,
  physical-stow null, MCAP recording and isolated replay passed. Bag contained
  two state messages, one capability message and two diagnostics (five total).
- Only synthetic loopback HTTP inside the isolated test container was used;
  no real service/robot calls, cameras or active deployment restarts occurred.

## Integration gates still required

1. Finalize and offline-test the service routes with the sibling worker's v1
   envelope above. Verify sanitization, bounded read timeouts and no camera,
   connection creation or motion side effects. Confirm that on-demand status
   and arm-joint read RPCs during polling are acceptable; this is not cache-only.
2. Verify token-file permissions and private Docker-to-host HTTP routing without
   exposing credentials or changing active services.
3. Repeat the fixture smoke test after any contract change, then obtain explicit
   authorization before an actual service connection.
4. Verify observation timestamps and sanitization with a human-reviewed sample.
   Only then consider typed measured interfaces; no physical measurement or
   navigation capability is established by this foundation.

## Final combined validation — 2026-09-30

With the finalized observation hardening and simulation-only action packages:

- Standalone `python3 -m unittest discover -s robotics/tests -v`: **18 passed**.
  Action-test imports now include both source package roots, so no externally
  supplied PYTHONPATH or installed ROS is required for test discovery.
- ARM64 image rebuild succeeded for all three ament packages; image size about
  **1.30 GB**. Normal entrypoint remains the read-only driver.
- Read-only ROS fixture smoke passed under network-none, non-root, read-only,
  512 MiB / one CPU: launch, JSON topics, stale diagnostics, five-message MCAP
  recording and replay after stopping the driver.
- Separate action smoke passed for four simulated operations, verified synthetic
  results and duplicate-ID rejection in the same hardened, isolated environment.
  This does not enable production actions or establish physical verification.
- Both standalone Compose files validate without starting any service.
- Combined repository suite repeated successfully: **522 tests passed**.
  An earlier full-suite run had one independent-STOP fixture failure (HTTP 502
  instead of 200); its isolated reproduction passed, then the full suite passed.
  Retain this as a test-reliability follow-up rather than claiming it was fixed
  or that the passing repeat proves physical STOP behavior.

These are offline software-validation results only. No live API/robot request,
credential inspection, camera startup, motion or active deployment restart was
performed. Host routing/token permissions and supervised physical verification
remain explicit deployment/operation gates, not completed live evidence.

## Authorized live observation validation — 2026-10-01

Owner authorized direct HTTP development observations and container recreation.
Rebuilt/recreated app only; existing go2rtc remained healthy and untouched.
The root Dockerfile preserves host source modes: `observation-admission.ts` was
0600 and initially prevented non-root startup. A local derived application image
normalized `/app/src` and `/app/dist/web` read/traverse permissions, retaining
`USER node`; no credentials or private state permissions changed. **Durable
follow-up:** normalize public code/asset permissions in the root Dockerfile's
runtime stage. The derived-image repair is not preserved by a fresh root build.

Validated authenticated host and container capabilities GET with v1.0. Initial
state correctly reported unavailable before an existing robot connection was
established through MCP. Then ran the normal read-only ROS launch for ~24 seconds
in a temporary non-root/read-only container (512 MiB, one CPU), mounting only the
existing service token read-only and routing via host.docker.internal with the
approved Host header. No token value or raw snapshot was logged.

Result: **three state samples, one capabilities sample, three `ok` diagnostics**.
All three state envelopes had connection connected and fresh status/armJoints.
This verifies authenticated HTTP-to-ROS DDS observation publication, not joint
calibration, physical stow, pose, stop semantics or control actions. Temporary
ROS launch exited cleanly and container was removed; app and go2rtc both healthy.
No live control endpoint, camera-start request or robot motion was issued during
this ROS validation. Persistent ROS polling remains disabled by default.
