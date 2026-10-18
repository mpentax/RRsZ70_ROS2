# Front-camera throughput: live read-only validation

## Result and design

The default ROS bridge now delivers **3 Hz** at the original **1600 × 1200** resolution.
RoboClaw retains ordinary HTTP MJPEG consumers on `z70_front_jpeg` (1600×1200) and
`z70_gripper_jpeg` (1600×896) for 10 seconds after the final frame request. Concurrent frame
requests and camera switches share the pre-warmed receivers without restarting FFmpeg or
`go2rtc`; service shutdown aborts them. `/api/camera/frame` reads the next
JPEG from the active camera stream (`z70_front_jpeg` or `z70_gripper_jpeg`), not a cached image. Authentication and the external API are unchanged.
There is no permanent preload, exposed sidecar API, new product switch, or robot command.

Keeping only an encoded MP4 consumer warm was insufficient: the original go2rtc JPEG
handler waits for keyframes, yielding 1.27 seconds per warm frame. Continuous decoding
is necessary to reach the requested 2–5 Hz range.

## Measurements

All measurements were performed against the actual configured Z70 using read-only frame
requests. HTTP runs issued sequential POSTs for at least 60 seconds, consuming and validating
each JPEG fully. The ROS run used an isolated DDS domain (92), the real Bridge at its new
3 Hz default, and reliable depth-10 subscriptions to compressed and raw images. It observed
60 seconds after first delivery. The robot was not moved.

| Path | Count / duration | Delivered rate | First frame | Subsequent request mean / max |
|---|---|---|---|---|
| Baseline HTTP snapshots | 9 / 61.392 s | 0.1466 Hz | 5.651 s | 6.967 / 7.436 s |
| Encoded-stream-only hold (rejected) | 44 / 61.189 s | 0.7191 Hz | 6.531 s | 1.271 / 1.284 s |
| Final HTTP, continuous JPEG decoder | 265 / 60.142 s | 4.4063 Hz including cold start | 7.358 s | 0.1995 / 0.2400 s |
| Final ROS, compressed + raw | 181 each / 60.048 s after first | 3.0008 Hz | 7.134 s from node start | 0.0992 / 0.2312 s |

ROS image arrival interval: mean 0.33325 s, maximum 0.46558 s. All 181 image stamps were
unique. First ROS HTTP request took 6.796 s; node timer start adds about 0.333 s.
The final HTTP warm period is approximately 5 Hz; the ROS default intentionally samples at 3 Hz.

Raw timing records and one lifecycle summary are in `calibration/front-camera/throughput-*.json`.
Local, git-ignored JPEGs are alongside those records (`throughput-before.jpg`,
`throughput-after-mjpeg.jpg`, `throughput-ros.jpg`). Vision inspection of the final HTTP image
showed the actual hallway, door and two physical calibration boards; it was not a synthetic
fixture. Images were saved directly from the live HTTP response. Pixels alone do not
establish an exposure timestamp.

## Lifecycle, logs and tests

- Immediately after the final HTTP run, each of `z70` and `z70_jpeg` had one active producer
  and one consumer. After 13 seconds without requests both had zero consumers and zero
  active producers. Following the ROS run they were also idle, with zero FFmpeg processes.
- Both containers remained at `RestartCount=0`; the app was healthy.
- Bounded `docker attach --no-stdin --sig-proxy=false` monitoring throughout both successful
  runs observed zero error-log lines, including zero `wrong login` or CONNACK errors. Raw
  credential-bearing upstream logs were not saved or printed. go2rtc logging is error-level:
  this verifies no observed login failures/crash-loop, not a count of MQTT CONNECT packets.
- Existing unique-MQTT-client-ID patch was preserved.
- TypeScript typecheck passed; Node suite **339/339** passed; ROS suite **15/15** passed.
  Behavioural regressions verify shared/concurrent retention, idle release/reopening, and
  the default ROS camera timer delivering multiple callbacks per second.
- Only RoboClaw app/go2rtc containers were rebuilt/recreated. All validation subprocesses,
  ROS nodes and Docker attach clients were stopped. No commit was made.

## Limits and operational notes

Cold startup still takes about seven seconds. The frame stamp is receipt time, not measured
camera exposure time; no claim about sensor-to-ROS age is made. This fixes delivered cadence,
not floor-VO accuracy, motion blur or the <=15-degree rotation-between-frames requirement.
No motion validation was authorized or performed. Continuous decoding adds sidecar CPU work
while consumers are present. Ordinary WebRTC viewers or explicit manual preloads can keep
the original video stream active independently of the snapshot idle timeout.

Michael should check whether the robot shows a camera indicator or makes a viewing
announcement while the camera stays on; the agent cannot hear it. This is not a new gate.

The `ffmpeg.http` input template belongs in the top-level FFmpeg configuration, not inside
`streams.z70_jpeg`: go2rtc treats any source string containing `{input}` as an uninitialized
source template. Config-only bind-mount edits also require go2rtc recreation/restart; a cached
image build plus ordinary `compose up -d` does not reload them.
