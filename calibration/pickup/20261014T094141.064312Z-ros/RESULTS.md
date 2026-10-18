> **Historical / superseded guidance — 2026-10-18 curation.** Original body below is preserved verbatim. Measurements, failed fits, cleanup uncertainty, test counts and commands describe only the dated experiment/source revision, not current runtime state or reusable authorization. Current guidance: [manipulation guide](../../../src/z70_manipulation/README.md), [latest trial outcome](../20261018T141117Z-gpt-stack/ANALYSIS.md).

# z70_manipulation build and read-only validation

## Outcome

Implemented the ROS packages, calibrated TF/camera model, deterministic colour
perception, safe-grid planner, pickup action/CLI and HTTP executor. No VLM is
used. Physical pickup is pending Claude's authorized supervised trial.

No real robot movement RPC, camera selection, anchor write, service restart or
deployment was sent during this task. Live access consisted only of state and
arm-estimate GETs and three front-frame reads with POST /api/camera/frame {}.
No git commit was made. Test-owned HTTP servers, threads and ROS publisher
processes were shut down/reaped; the process check found no remaining
robot_state_publisher, manipulation executable or Node test processes.

## Tests actually run

- `~/.pixi/bin/pixi run test`: **72 passed**, 7 packages built, 14.19 s test time.
- `npx tsc -p tsconfig.json --noEmit`: passed.
- `npm test`: **370 passed**, no failures.
- `pixi run bringup --show-args`: launch parses, `manipulation` defaults false.
- `git diff --check`: passed.

TF tests launch the real native robot_state_publisher and compare
base_footprint -> gripper_camera_optical_frame to the nested camera_at_wrist
calibration fit at (170,65), (200,95), (220,115), each at M5=0 and M5=155.
All six comparisons pass the 5 mm criterion. Existing wrist TF tests were
updated for the measured mount.

The fake HTTP server exercises complete pickup/release/stow, actual synthetic
front-frame re-observation during chassis pulses, joint-estimate readback,
normal timeout/unknown gripper outcomes, ambiguous grip without replay,
negative code 2, JSONL logging, physical-size/JPEG contact filtering, missing
frames, robot/path/odom hazards, no descent when centering fails and unsuccessful
verification when the floor blob remains. A native ROS action round-trip checks
MarkerArray/JSON publication. Fake-server pickup success is not hardware evidence.

## Final live dry-run

Command: `~/.pixi/bin/pixi run pick yellow --dry-run` from ros2/.

The service returned a tagged 1600x1200 front frame and arm_state IN_ROBOT.
Final report: **objects=[]**, **target=null**, **approach=null**. Exit 1 is the
intentional no-target result. No eligible 2.5–8.5 cm, compact coloured floor
object was found; no yellow pickup approach was fabricated.

Evidence: `front.jpg`, `dry-run.json`, `timeline.jsonl` in this directory.
Offline inspection of raw HSV regions from the saved JPEG found, for example:

| Region (not an accepted object) | Floor-assumed base x,y (m) | Width (m) | Rejection |
|---|---|---|---|
| Blue | 0.7854, -0.3006 | 0.0153 | too small |
| Green | 0.4287, 0.1221 | 0.0125 | too small/elongated |
| Purple | 4.2739, -0.5491 | 0.0807 | beyond 3 m CF projection range |

These floor-assumed region positions do not prove that those regions are actual
objects on the floor. Earlier development captures at 093242 and 093524 used
weaker physical/shape rejection and are superseded by this result. Their raw
colour detections were never used to move the real robot.

## Changed files

Modified:
- `ros2/pixi.toml`
- `ros2/src/z70_bridge/z70_bridge/bridge.py`
- `ros2/src/z70_bridge/test/test_camera.py`
- `ros2/src/z70_bringup/launch/bringup.launch.py`
- `ros2/src/z70_bringup/package.xml`
- `ros2/src/z70_description/urdf/z70.urdf.xacro`
- `ros2/src/z70_description/test/test_wrist_tf.py`
- `src/arm-estimator.ts`
- `src/arm-estimator.test.ts`

Created:
- `ros2/src/z70_description/config/mount_calibration.yaml`
- `ros2/src/z70_description/config/z70_gripper_camera_info.yaml`
- `ros2/src/z70_description/test/test_gripper_tf.py`
- `ros2/src/z70_manipulation/{README.md,package.xml,setup.py,setup.cfg}`
- `ros2/src/z70_manipulation/resource/z70_manipulation`
- `ros2/src/z70_manipulation/config/colors.yaml`
- `ros2/src/z70_manipulation/z70_manipulation/{__init__,perception,planner,executor,node,cli}.py`
- `ros2/src/z70_manipulation/test/test_{perception,planner,executor,ros,cli}.py`
- `ros2/src/z70_manipulation_interfaces/{CMakeLists.txt,package.xml,action/Pick.action}`
- Three `calibration/pickup/*-ros/` directories with read-only frame/dry-run/timeline evidence; this RESULTS.md is the final report.

The pre-existing dirty chassis calibration and other untracked artifacts were
not edited by this implementation. Colcon also regenerated ignored build and
install outputs. The existing control-plane skill gained reusable pickup
calibration, JPEG-contact and no-reply/auto-lift lessons.

## Claude's live handoff

Place a real yellow 4–6 cm target on clear floor first: final dry-run currently
finds no eligible yellow object. The human/pet safety input is external:
`/z70/safety_stop` (std_msgs/Bool); HSV does not recognize people/pets. The
0.12 m fingertip extension and grasp pixel gate remain provisional.

Deploy the TypeScript estimator correction first, only with other controllers
inactive, from the repository root:

```sh
docker compose build app && docker compose up -d app
```

It invalidates the pre-grasp pose after autonomous app_grip auto-lift, instead
of publishing it as if the wrist had stayed still. This source fix was tested
but deliberately not deployed here. The action independently uses pixel-only
held-object evidence against a legacy service too.

Then, if no observation stack is already running, in one terminal:

```sh
cd /Users/mpentax/Code/RoboRockZ70-Connector/ros2
~/.pixi/bin/pixi run bringup manipulation:=true mapping:=false
```

In another:

```sh
cd /Users/mpentax/Code/RoboRockZ70-Connector/ros2
~/.pixi/bin/pixi run pick yellow
```

If observation bringup is already running, start only the action server (do not
add duplicate bridge/TF publishers):

```sh
cd /Users/mpentax/Code/RoboRockZ70-Connector/ros2
~/.pixi/bin/pixi run bash -c 'source install/setup.bash && export Z70_REPO="$(cd .. && pwd)" && exec ros2 run z70_manipulation pick_server'
```

Stop Claude's trial-owned launch/server processes when done. Hazard stops do
not automatically release/stow into a person/pet; supervisor recovery is
required. No success claim for physical pickup, human detection or physical
stow feedback is made by this offline/read-only build.
