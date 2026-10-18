# z70_mapping — vendor SLAM and optional local depth

A read-only ROS interpretation layer, not navigation. `z70_driver` acquires the
vendor map/rooms and camera frames; this package consumes typed
`/z70/driver/map_snapshot` and shared images, not another HTTP/vendor client.
The driver schedules commands; its private sidecar transports them. Neither this
observer nor the depth worker sends cleaning, mode or motion RPCs.

## Run and topics

From the repository root, within authorized live observation:

```sh
pixi run bringup                    # mapping:=true, depth:=true
pixi run bringup depth:=false       # no monocular cloud
pixi run bringup mapping:=false     # no vendor map interpreter
```

Use [Foxglove](../z70_bringup/foxglove/README.md) on ws://127.0.0.1:8765. Import the
installed/repository dashboard; Overview uses map-fixed visualization. Depth is
an independent approximate local overlay, not fused into vendor `/map`.

| Topic | Meaning |
|---|---|
| `/map` | OccupancyGrid in map, RELIABLE + TRANSIENT_LOCAL depth 1 |
| `/z70/map/robot_pose`, `/z70/map/dock_pose` | Vendor snapshot references, not live localization or a docking goal |
| `/z70/map/markers` | Room boundaries/labels, dock, paths, virtual walls, zones and carpet contours; latched |
| `/z70/map/spatial_context` | JSON room/relative bearing/distance and eight-sector raster raycasts; latched, 5 Hz |
| `/z70/map/status` | Cache age/sequence, blocks/counts, freshness/alignment evidence; latched, 5 Hz |
| `/tf` | map → odom visual alignment, 5 Hz |
| `/camera/front/depth/points` | Optional xyz float32 cloud, metres, camera_optical_frame, input image stamp |
| `/z70/depth/status` | Worker availability, floor-fit/inference quality; latched |

Visualization map/markers/poses are stamped with **current ROS time**, republished
on updates and about every two seconds for late viewers' TF buffers. Original
cache receipt is retained in `OccupancyGrid.info.map_load_time` and status
`saved_at_ms`; source capture time is unknown. A new sequence is a new reply, not
proof of active floor or current localization. Driver refresh defaults to 60 s;
`/z70/map/refresh` requests a read. Failed refresh retains last geometry with age/
error evidence rather than manufacturing freshness.

## Acquisition and decoder contract

Private bearer APIs (`/api/map/cache`, `/raw`, `/image`, `/refresh`, room caches
and `get_room_mapping`) are driver acquisition details, not public UI/MCP routes.
Raw bytes are device-bound, integrity-checked and never logged/public fixtures.
The driver joins segment IDs to account room IDs through explicit
`get_room_mapping`, never room ordering. Unnamed segments remain `Segment N`.
Bounded unknown extensions are skipped, not interpreted as absent obstacles.

`z70_mapping/decoder.py` validates:

- `rr` magic, 20-byte header, declared length and trailing SHA-1 digest block 1024;
  maximum 32 MiB, bounded block count/extents and grid dimensions.
- Little-endian block header `(uint16 type,uint16 header_size,uint32 data_size)`.
  Block 2's final header fields are `(top,left,height,width)`.
- Raw raster is row-major **bottom row first** at 0.05 m/cell. ROS origin is
  `(left*0.05,top*0.05,0)` with identity orientation. Do not flip OccupancyGrid;
  only display images are flipped.
- Low pixel bits `1` are occupied 100; `255` or low bits `7` are free 0; others
  unknown -1. Segment ID is `pixel >> 3` for segmented free cells, excluding 7/255.
- Dock/robot blocks 1/8 use signed millimetres; optional signed heading is interpreted
  as degrees CCW in raw xy, not physically validated. Missing heading prevents SE(2)
  alignment. Missing obstacle records do not establish an obstacle-free room.
- Paths 3/9, walls 10 and zones 18/19/47/53 use unsigned mm coordinates. Extended
  zones include two metadata uint16s before corners. Malformed optional records
  are ignored. Carpet block 17 is a same-dimension bottom-first nonzero mask.
- Room boundaries retain disconnected contours/holes and represent cell centres,
  not surveyed edges; labels use segment centroids.

## Alignment and spatial context

Given vendor snapshot V and odometry O, yaw=`wrap(V.yaw-O.yaw)` and
translation=`V.xy-R(yaw)*O.xy`. Before first odometry, O=(0,0,0), so a valid headed
snapshot immediately anchors the overlay. `realign_on_refresh=true` resets on a
new map sequence; duplicate sequence does not. False freezes the startup offset.
This single-pose visual alignment is not synchronized fusion, drift correction
or relocalization and can jump when a stale snapshot reanchors it.

Spatial context uses aligned odometry when available, otherwise snapshot pose.
Room lookup uses the cell under the robot; unsegmented free floor can fall back
to a centroid within 1.5 m. Occupied/unknown/out-of-bounds cells cannot. Missing
pose/yaw leaves relative quantities/bearings null. Eight robot-relative rays use
half-cell steps up to 5 m, stopping on occupancy/bounds; **unknown is traversed**.
These are stale-raster distances, not live obstacle clearance or traversability.

## Depth architecture and geometry

The persistent external worker (`~/.venvs/roboclaw-depth/bin/python`, override
`depth_python`) consumes existing compressed front images. It loads pinned cached
Depth Anything V2 Metric Indoor Small weights once, default device MPS, with
`local_files_only=True`. [Environment/cache setup](../../docs/ros2.md#optional-depth)
is separate from ROS startup; no second camera session or Pixi Torch dependency.
Missing interpreter/weights reports unavailable. One inference is in flight,
stale frames drop, per-frame failures are reported, worker EOF/shutdown is reaped.

Default `depth_hz=0.2`. At 400×300 inference resolution, rays use original
calibration pixel centres and installed lens distortion/floor rotation/height.
The current `calibration` input, not an old fixed floor height, defines geometry.
Robust scale-only floor agreement uses 0.15–1.2 m candidates excluding the bottom
bumper strip; insufficient agreement leaves explicit uncorrected network depth.
Only finite optical-z points 0.05–4 m enter the cloud. `floor_aligned` is not
independent off-floor metric validation. Clutter/ramps can fool the floor fit;
network smear, optical registration and map/odom alignment affect placement.
This is not collision avoidance, Nav2, LiDAR replacement or multiview fusion.

## Verification and retained evidence

```sh
pixi run bash -c 'source install/setup.bash && pytest src/z70_mapping/test -q'
```

Offline tests cover integrity/axes, room joins, optional records, serialization,
alignment, refresh and depth rays/filtering. Fake snapshots cannot establish
physical localization, metric object accuracy or delivered live throughput.
The retained `test/live_check.py` is a live acquisition harness, not an offline
test. It writes under `calibration/mapping/`; use a fresh evidence destination and
test-owned XDG state, and inspect the harness before any authorized run.

Historical measurements/home-specific output remain evidence, not current setup:

<details>
<summary>Original integrated/live observation receipt (pre-cutover)</summary>

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


</details>
