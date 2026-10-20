# z70_mapping — Vendor SLAM Decoder and Optional Monocular Depth

Decodes the robot's native `v1` LiDAR map into ROS 2 `OccupancyGrid` and marker topics, computes eroded safe interior grids and patrol waypoints, and runs an optional Apple Silicon MPS monocular depth worker.

## Run and Topics

From the repository root:

```sh
pixi run bringup                    # mapping:=true, depth:=true
pixi run bringup depth:=false       # disable monocular depth worker
pixi run bringup mapping:=false     # disable vendor map node
```

| Topic | Type / Description |
| :--- | :--- |
| `/map` | `nav_msgs/msg/OccupancyGrid` (`0.05 m/cell`, `RELIABLE + TRANSIENT_LOCAL`) |
| `/z70/map/safe_grid` | `nav_msgs/msg/OccupancyGrid` — reachable eroded allowlisted interior (`0` = free, `100` = excluded) |
| `/z70/map/dynamic_obstacles` | `nav_msgs/msg/OccupancyGrid` — session-scoped dynamic obstacle layer (`100` = obstacle/skipped item, `50` = delivered laundry pile, `0` = free) |
| `/z70/world/objects`, `/z70/world/markers` | `std_msgs/msg/String` (JSON) & `visualization_msgs/msg/MarkerArray` — session-scoped `map`-frame registry of sighted clothing objectives, dynamic obstacles, and delivered laundry piles |
| `/z70/world/update` | `std_msgs/msg/String` (JSON) — updates the session-scoped dynamic obstacle and objective registry (`obstacle`, `delivered`, `failed`, `objective`, `clear`) |
| `/z70/map/robot_pose`, `/z70/map/dock_pose` | `geometry_msgs/msg/PoseStamped` from the vendor map snapshot |
| `/z70/map/markers` | `visualization_msgs/msg/MarkerArray` — room boundaries/labels, dock, paths, virtual walls, zones, carpet, and patrol waypoints |
| `/z70/map/spatial_context` | `std_msgs/msg/String` (JSON) — current room, 8-sector clearance raycasts, and `boundary_contract` |
| `/z70/map/status` | `std_msgs/msg/String` (JSON) — map cache age, sequence, block counts, and alignment metadata |
| `/tf` | `map -> odom` transform (`5 Hz`) |
| `/camera/front/depth/points` | `sensor_msgs/msg/PointCloud2` — metric `xyz` `float32` cloud in `camera_optical_frame` |
| `/z70/depth/status` | `std_msgs/msg/String` (JSON) — depth worker health and floor-scale fit telemetry |

---

## Map Binary Format (`z70_mapping/decoder.py`)

- Validates the `rr` magic header (20 bytes), declared payload size (`<= 32 MiB`), and trailing SHA-1 digest (block `1024`).
- Little-endian block headers `(uint16 type, uint16 header_size, uint32 data_size)`:
  - Block `1`: charger/dock pose (signed mm)
  - Block `2`: occupancy grid `(top, left, height, width)` at `0.05 m/cell`, bottom row first (`pixel & 7 == 1` occupied `100`; `255` or `pixel & 7 == 7` free `0`; `segment_id = pixel >> 3`)
  - Block `3` / `9`: path polylines
  - Block `8`: robot pose `(x_mm, y_mm, heading_deg)`
  - Block `10`: virtual walls
  - Block `17`: carpet mask
  - Blocks `18` / `19` / `47` / `53`: forbidden and cleaning zones
- Segment IDs are joined to named rooms via `get_room_mapping`.

---

## Safe Interior Grid and Patrol Waypoints (`z70_mapping/boundary.py`)

`boundary.compute_safe_interior(m, labels, BoundaryConfig(...), robot_pose_map=None)` computes the reachable interior mask:
1. Unions allowlisted room segments (`boundary.allowed_segments`, default `[1, 2, 3, 4, 5, 6]`) and excludes occupied/unknown cells, forbidden polygons, and supercover virtual-wall lines.
2. Applies `cv2.distanceTransform(DIST_L2, DIST_MASK_PRECISE)` and erodes by `chassis_radius_m + deployed_arm_extension_m + localization_uncertainty_m + stopping_distance_m`.
3. Selects the 4-connected component containing the robot pose and publishes it on `/z70/map/safe_grid`.
4. `generate_patrol_waypoints(result)` seeds each reachable room's highest-clearance cell, adds dispersed observation points (`waypoint_spacing_m = 0.90 m`, `waypoint_clearance_m = 0.25 m`), and orders a nearest-neighbor tour.
5. `check_corridor_containment(safe_grid_or_result, start_xy, end_xy, radius_m)` tests whether the continuous swept capsule along a straight segment stays inside the safe interior.

| Parameter | Default | Meaning |
| :--- | :--- | :--- |
| `boundary.allowed_segments` | `[1, 2, 3, 4, 5, 6]` | Allowlisted room segment IDs |
| `boundary.chassis_radius_m` | `0.199` | Half of the Z70's `39.8 cm` chassis diameter |
| `boundary.deployed_arm_extension_m` | `0.0` | Additional clearance when the arm is deployed (`0.10 m` during loaded garment transit) |
| `boundary.localization_uncertainty_m` | `0.0` | Extra localization margin |
| `boundary.stopping_distance_m` | `0.0` | Extra stopping-distance margin |
| `boundary.waypoint_clearance_m` | `0.25` | Preferred wall clearance for patrol waypoints |
| `boundary.waypoint_spacing_m` | `0.90` | Minimum spacing between patrol waypoints |

---

## Monocular Depth (`z70_mapping/depth_node.py`)

Runs Depth Anything V2 Metric Indoor Small in `~/.venvs/roboclaw-depth` ([setup instructions](../../docs/ros2.md#optional-depth-worker)) at `depth_hz = 0.2` (`400 x 300` inference resolution). Fits a robust floor-plane scale factor over `0.15..1.2 m` floor rays and publishes `0.05..4.0 m` 3D points on `/camera/front/depth/points`.

## Tests

```sh
pixi run bash -c 'source install/setup.bash && pytest src/z70_mapping/test -q'
```
