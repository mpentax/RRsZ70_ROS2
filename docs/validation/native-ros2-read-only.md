# Native ROS 2 read-only validation

Validated on macOS arm64 with the running Docker RoboClaw service; no robot MCP calls, robot
commands, service POSTs, anchors or secret-file inspection were performed. The bridge alone reads
the service token from its configured path.

## Commands and results

From `ros2/`:

```sh
~/.pixi/bin/pixi run test
# 14 passed (11 bridge/description checks plus 3 concurrently added odometry checks)

# Live run: domain 92 isolates pre-existing ROS processes; 8765 was already occupied.
ROS_DOMAIN_ID=92 ~/.pixi/bin/pixi run bringup camera_hz:=1.0 foxglove_port:=8766

# In the pixi environment, with install/setup.bash sourced and ROS_DOMAIN_ID=92:
ros2 topic list --no-daemon
ros2 topic hz /joint_states
ros2 topic hz /tf
ros2 topic hz /z70/status
ros2 topic hz /z70/arm_estimate
ros2 topic hz /odom
ros2 topic hz /z70/odom_status
```

The 53.7-second bounded harness ran all rate probes concurrently, stopped its launch and probes
with SIGINT, waited for exit, and checked processes afterward. All four launch children exited
cleanly. Pre-existing nodes were not killed. The harness was executed as:

```sh
~/.pixi/bin/pixi run bash -c 'source install/setup.bash && python /Users/mpentax/.hermes/cache/scratch/validate_z70_ros2.py'
```

Camera probes used the same `ros2 topic hz` implementation with **reliable QoS**: the Jazzy CLI
hardcodes best-effort sensor QoS, which failed to receive the large raw images in this native
Fast DDS setup. Exact equivalent invocation (change the topic for camera_info):

```sh
python -c 'import ros2topic.verb.hz as hz; from rclpy.qos import QoSProfile; hz.qos_profile_sensor_data=QoSProfile(depth=10); from ros2cli.cli import main; import sys; sys.argv=["ros2","topic","hz","/camera/front/image_raw"]; main()'
```

| Topic | Measured receiving rate (Hz) |
|---|---:|
| `/joint_states` | 9.861 |
| `/tf` | 9.931 |
| `/z70/status` | 0.999 |
| `/z70/arm_estimate` | 0.999 |
| `/camera/front/image_raw` | 0.137 |
| `/camera/front/camera_info` | 0.149 |
| `/odom` | 0.137 |
| `/z70/odom_status` | 0.137 |

Foxglove returned **HTTP 101 Switching Protocols** on port 8766. This SDK bridge supports the
`foxglove.sdk.v1` WebSocket subprotocol; a legacy-only handshake returned HTTP 400. The probe's
immediate socket closure generated benign broken-pipe logs, not launch failures.

## Findings and limits

- Fresh-shell tasks now build and source the overlay; previously bringup/view did not source it.
- pytest 9 was incompatible with Jazzy's launch_testing plugin. Pinning pytest below 9 produced
  a clean runnable suite, without disabling ROS test plugins.
- Estimate mapping now has behavioral coverage, including UNKNOWN/IN_ROBOT folded visualization,
  closed/open fingers and five actual wrist TF comparisons against vendor FK.
- During this validation images were read via `docker exec` into the go2rtc sidecar to avoid any POST.
  Claude reverted that afterwards: RoboClaw's `POST /api/camera/frame` is a frame read, not a robot command,
  and both paths measured the same ~6.8 s per frame (go2rtc starts the stream per snapshot).
- **The camera was configured at 1 Hz, but did not achieve 1 Hz.** A standalone JPEG GET took
  6.738 seconds and returned 82,604 bytes, decoded as 1600×1200 BGR. Snapshot startup latency
  limited observed image/VO throughput to approximately 0.14 Hz. This is measured, not inferred
  from timer settings. No camera image was visually interpreted in this validation.
- Topic publication and local VO plumbing are validated; this stationary read-only run does not
  calibrate chassis motion, validate VO scale on actual travel or establish arm joint feedback.
- The old `docs/archive/robotics-ros.md` reference to `http://app:57000` is container-internal and
  stays unchanged; the audit's 57000–57100 policy is a port range, not a stale host URL.
