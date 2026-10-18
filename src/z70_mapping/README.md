# z70_mapping — vendor LiDAR SLAM + optional local depth

This is a **read-only observation/visualisation layer**, not navigation. RoboClaw remains the only command path. The map is the robot's own `get_map_v1` SLAM raster, **not** the camera floor mosaic and not an invented floor plan.

Phase 2: this package has no HTTP observation client. `MappingNode` consumes the
latched typed `/z70/driver/map_snapshot`; `z70_driver` alone owns cache reads,
refresh and room acquisition. The depth worker consumes matching calibrated ROS
frames from that same acquisition pipeline. This cutover is offline-qualified,
not a new live map/depth/camera validation.

## Run / what Michael sees

From the repository root (live authorization required):

```sh
~/.pixi/bin/pixi run bringup                     # mapping:=true, depth:=true (both enabled by default)
~/.pixi/bin/pixi run bringup depth:=false        # disable local monocular depth cloud
~/.pixi/bin/pixi run bringup mapping:=false      # disable vendor map observer
```

Foxglove Studio (`ws://127.0.0.1:8765`, layout `src/z70_bringup/foxglove/z70-dashboard.json`) displays the home's wall/free/unknown grid, room outlines and labels, vendor robot/dock poses, the 3D robot model, and the local depth fan in a 3D panel with fixed frame `map` (`view_model.launch.py` provides the standalone offline URDF viewer). Room names are joined **only** through the robot's explicit `get_room_mapping` segment-to-account-room relation, never guessed from ordering. Unnamed segments stay `Segment N`. In the live home: Kitchen, Living room, Bedroom, plus three unnamed segments. Depth is an independent approximate 3D overlay, not fused into `/map`.

| Topic | Type / semantics |
|---|---|
| `/map` | `nav_msgs/OccupancyGrid`, `map`, RELIABLE + TRANSIENT_LOCAL, depth 1 |
| `/z70/map/robot_pose` | `geometry_msgs/PoseStamped`, embedded **vendor snapshot** pose, NOT live odometry |
| `/z70/map/dock_pose` | `geometry_msgs/PoseStamped`, embedded dock reference point |
| `/z70/map/markers` | `visualization_msgs/MarkerArray`, room contours/labels, robot/dock, visited/goto paths, virtual walls, forbidden zones and carpet contours; transient-local |
| `/z70/map/spatial_context` | `std_msgs/String` JSON, current room, distance-sorted rooms, dock distance/bearing and eight robot-relative grid ray distances; RELIABLE + TRANSIENT_LOCAL, 5 Hz |
| `/z70/map/status` | JSON source, cache age, map index/sequence, room labels, block/obstacle counts, explicit freshness/alignment caveats |
| `/tf` | periodically broadcasts frozen `map -> odom` visual alignment at 5 Hz |
| `/camera/front/depth/points` | optional `sensor_msgs/PointCloud2`, xyz float32 metres in `camera_optical_frame`, input image stamp, ~0.2 Hz |
| `/z70/depth/status` | floor-fit/inference quality; explicitly says metric accuracy is unverified |

## RoboClaw data available today

- `GET /api/map/cache`: device-bound metadata, cache save/receipt time, map index/sequence; deliberately no geometry/fresh-pose claim.
- `POST /api/map/cache/refresh {}`: existing local-L01 request + correlated MQTT V1 reply. This is a map **read**, not a cleaning/motion/mode command.
- `GET /api/map/cache/image`: existing PNG occupancy/segment raster.
- **New authenticated** `GET /api/map/cache/raw`: exact integrity-verified cached V1 bytes, selected-device-bound, `Cache-Control: no-store`. Headers `X-Map-Saved-At`, `X-Map-Index`, `X-Map-Sequence` describe the same cache source. Raw bytes are not exposed publicly or logged.
- `GET /api/home/rooms` / `POST /api/home/rooms/refresh {}`: account room names, but no polygons. The ROS observer reads the existing room cache; it does not repeatedly refresh account signing material.
- Read-only vendor RPC `get_room_mapping`: explicit segment IDs associated with account room IDs. Polygons are derived from segmented raster contours, retaining disconnected boundaries and holes rather than inventing convex hulls.
- Real snapshot contains robot (block 8), dock (block 1), grid (block 2), paths/other extension blocks. Obstacle block counts 12/15/16 were all **zero**. Occupied grid cells/walls still exist. This does NOT establish an obstacle-free room; no obstacle-object coordinate layout is claimed/decoded. Unknown modern extension blocks are bounded and skipped, not guessed.

The driver first publishes the cached map (works if cloud refresh is down), then refreshes on its `map_refresh_s` interval (default 60 seconds), or via the standard `/z70/map/refresh` ROS service. Acquisition stays off the ROS executor thread. Failed refreshes preserve original cache evidence and expose stale/error status; mapping retains last decoded geometry with `refresh_error` plus age. This package performs no HTTP POSTs. Cache/room observations are the driver's responsibility; no motion, cleaning, arm or mode methods exist here.

## V1 decoding and axes

`decoder.py` follows the private adapter's `adapters/vendor/src/v1-map-summary.ts` and `adapters/vendor/src/private-map-raster.ts` wire conventions:

- `rr` magic, 20-byte header, declared total length, trailing SHA-1 digest block 1024 verified before parsing; maximum 32 MiB, bounded block extents/count and grid size.
- Typed block headers are little-endian `(uint16 type, uint16 header_size, uint32 data_size)`.
- Grid block 2 stores `(top, left, height, width)` at its last 16 header bytes; pixels are raw row-major, **bottom row first**.
- Resolution: **0.05 m/cell**; ROS grid origin `(left*0.05, top*0.05, 0)`, identity orientation. Raw row zero is ROS minimum-y row; **do not vertically flip the OccupancyGrid**. PNG display/rendering alone flips it.
- Pixel `1`/low bits `1`: occupied 100; `255`, `7`, low bits `7`: free 0; all other pixels: unknown -1. Room ID is upper bits of segmented floor pixels (`pixel >> 3`), excluding unsegmented 7/255.
- Robot/dock x/y are signed millimetres, divided by 1000. Optional signed angle is interpreted as degrees counter-clockwise in raw xy. That heading convention is **not physically validated**; missing heading prevents alignment and uses a spherical marker.
- Paths (blocks 3/9), virtual walls (10), forbidden zones (18/19), and extended zones (47/53, two metadata uint16s followed by four corners) use unsigned millimetre coordinates, converted to metres. Malformed optional record lengths are ignored.
- Carpet (17) is a boolean bottom-first raster only when its byte count matches the image dimensions; nonzero bytes indicate carpet.
- Room/carpet boundaries are raster contour **cell centres**, so not survey-grade polygon edges (approximately one-cell discretisation). Labels use segment pixel centroids.

`PoseStamped.header.stamp` and map header are **cache save/receipt time**, not sensor exposure/capture time. Both `active_map_verified` and `current_pose_verified` stay false. A new map sequence proves a new reply, not current localisation. The dock reference can overlap wall/dock occupancy; it is not a navigation docking goal.

## Simple map → odom alignment

At startup, take the first available vendor robot pose `V=(xv,yv,yawv)` and `/odom` pose `O=(xo,yo,yawo)`. Before any `/odom` sample, assume `O=(0,0,0)` so the first valid map snapshot immediately establishes and broadcasts the overlay (provided its robot heading is present). Compute:

```
yaw = wrap(yawv - yawo)
t = [xv,yv] - R(yaw) [xo,yo]
T_map_odom = (t.x, t.y, yaw)
```

Broadcast the offset with current TF timestamps at 5 Hz and keep it frozen between new map sequences. `realign_on_refresh` now defaults to **true**: a different `map_sequence` resets the anchor against the latest odometry; duplicate snapshots of the same sequence do not. Set `realign_on_refresh:=false` to preserve the startup offset indefinitely. Refresh re-anchoring can jump when the vendor snapshot is stale. This is **not sensor fusion**, simultaneous-pose matching, drift correction or relocalisation. Pose snapshot freshness, heading and active floor remain unverified. Diagnostic `alignment_verified` always remains false.

## Spatial context

`/z70/map/spatial_context` uses aligned odometry when an anchor exists, otherwise the vendor snapshot robot pose. `current_room` identifies the segmented pixel under the robot; on unsegmented free floor it falls back to the nearest room centroid within 1.5 m. Out-of-bounds, occupied and unknown cells do not use that fallback. `rooms` includes segment ID/name, centroid, area, distance and wrapped robot-relative bearing; `dock` includes its map pose and relative distance/bearing. Missing robot pose gives null relative quantities; missing yaw gives null bearings.

`clearance_sectors_m` raycasts front, front-left, left, rear-left, rear, rear-right, right and front-right relative to robot yaw (zero when yaw is missing), with half-cell steps up to 5 m. Rays stop on occupied cells or map bounds; **unknown cells are traversed**, not interpreted as observed-free floor. No pose gives an empty sector dictionary. These are distances in a potentially stale vendor raster, not live obstacle clearance, navigation goals or a collision-avoidance guarantee.

## Depth worker / honest limits

ROS subscribes to the bridge's existing compressed JPEG topic; it does not open another camera session. A persistent worker in `~/.venvs/roboclaw-depth/bin/python` uses cached **Depth Anything V2 Metric Indoor Small** weights with MPS. `depth_python:=<venv-python>` selects the interpreter; node parameters `depth_hz` (default 0.2) and `device` (default mps) are available. Missing venv/weights is an optional-layer failure, not a reason to fabricate depth. Pixi stays free of torch/transformers dependencies.

Inference produces 400×300 samples; rays are mapped back to original 1600×1200 calibration pixel centres and undistorted with the measured k1/k2 lens. Per-frame robust **scale-only** correction uses agreement with the calibrated 68.36-mm floor plane over approximately 0.15–1.2 m, excluding the bottom bumper strip. Insufficient floor agreement publishes uncorrected network depth with an explicit `floor_aligned:false` status; it must not be treated as verified metric geometry. Only finite optical-z ranges 0.05–4 m enter the cloud. The model is loaded once, one request is in flight, stale frames are dropped, and the worker is stopped/reaped on node exit.

This intentionally simpler online fit does **not** claim the prototype's inverse-affine accuracy. Existing held-out-board experiment (`calibration/pointcloud/README.md`) reported scale-only errors of 23/39 mm versus inverse-affine 5/20 mm on its two boards; these are historical single-image floor-board results, not online/off-floor guarantees. Intrinsics/height/plane came from one calibration view; floor fit assumes flat visible floor, has no semantic segmentation, and can be fooled by clutter/ramps. The current scene required about 0.14× raw network scale, showing why a network's metre label is not trustworthy by itself. Thin objects/furniture can smear along rays; no object height, obstacle distance or accumulated 3D map is independently validated. The URDF camera longitudinal offset is still nominal; cloud-to-home placement inherits that plus VO and startup alignment uncertainty. This is not collision avoidance, Nav2, a replacement LiDAR, or multi-view depth fusion.

## Executed validation

Offline feature tests cover optional V1 geometry/invalid lengths, ROS marker serialization, room/dock ego geometry, eight-sector raycasting, immediate pre-odometry alignment, sequence-based refresh alignment and spatial-context publication. Node unit tests mock ROS transport and use synthetic snapshots; they do not establish live TF delivery or late-subscriber delivery for the new topic. Run from the repository root:

```sh
# For source-only runs without a sourced colcon install:
export PYTHONPATH="$PWD/src/z70_mapping${PYTHONPATH:+:$PYTHONPATH}"
~/.pixi/bin/pixi run pytest src/z70_mapping/test -q
```

Historical integrated/live results below predate these feature changes:

- Final integrated rerun (including the other agents' concurrent changes): **TypeScript compile clean, 358 service/web tests pass, 44 ROS tests pass, including 11 z70_mapping tests**.
- TypeScript compile succeeded and complete service suite passed **348 tests** immediately after the raw-map route change (other concurrent agents may add tests afterwards).
- Native pixi package build succeeded; package tests include integrity/extent fixtures, row/origin/position conversion, explicit empty obstacle counts, SE(2) alignment/missing heading, room joins, ROS grid/pose/markers, calibrated depth rays/floor correction and invalid depth filtering.
- Read-only isolated-domain full bringup ran **90 seconds**, then **35 seconds** after cleanup/obstacle diagnostics changes. No robot motion/mode/cleaning/arm commands.
- Actual ROS map: **365×296 at 0.05 m**, **18.25×14.8 m canvas**; **12,894 free**, **859 occupied**, **94,287 unknown** cells. Six room segments, three named rooms. Canvas includes unmapped margins; free-pixel area is not the home's surveyed area.
- **17 markers**, **1,944 boundary points**. Robot/dock poses published; actual `map -> odom` TF observed, 5-Hz map diagnostics, odometry around 2.5–2.8 Hz during these runs.
- A subscriber created **15 seconds after startup** received the latched map before the next scheduled refresh: transient-local late join demonstrated, not merely configured.
- Optional cloud: **120,000 finite xyz points**, optical frame, 12 bytes/point; **~0.197 Hz** delivered. Warm inference ~0.07 s (the remaining interval is deliberate rate limiting). In this scene optical-z extent ~0.08–0.95 m: a local view fan, not whole-home depth.
- Copilot vision checked both the original vendor PNG and a render from the actual ROS map/markers: coherent apartment-like regions, robot centre on free floor; dock near occupied boundary. Geographic mirroring/heading cannot be resolved from shape alone. Actual ROS depth render showed a small local fan, not accurate object-shape proof.
- First test's group + launcher double SIGINT interrupted shutdown. Own mapping/depth cleanup was fixed (ignore duplicate interrupt during cleanup, worker exits on EOF); both nodes subsequently exited cleanly, no depth worker/owned validation process survived. Other pre-existing bridge/VO/mosaic/calibrator nodes still print duplicate-interrupt cleanup traces in this harness; not modified here.
- RoboClaw app rebuilt/recreated once for the route and verified **RestartCount=0**. Other agents' ROS processes were not killed.

Local evidence (git-ignored/private): `calibration/mapping/` holds `v1-live.bin`, `vendor-map.png`, `ros-map.png`, `ros-depth-topdown.png`, `cloud-optical.npy`, `live-results.json`, `bringup.log`. No private map fixture is checked in.

The preceding live results are historical, before ownership/directory cutover.
The retained live validator now runs from the repository root; it is NOT an
offline verification command and requires separate acquisition authorization:

```sh
~/.pixi/bin/pixi run build
~/.pixi/bin/pixi run bash -c 'source install/setup.bash && python src/z70_mapping/test/live_check.py'
# Optional shorter check (must exceed 15 s for the late-join assertion): ... live_check.py --duration 35
~/.pixi/bin/pixi run test
```
