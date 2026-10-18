> **Historical / superseded — retained in place.** Current owners: [ROS guide](../../ros2.md), [command/API contract](../../thin-api.md), [mapping/depth guide](../../../src/z70_mapping/README.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

Private home-map artifacts remain external/private, not public repository dependencies; their availability was not checked. Retain bottom-first raster, 50-mm grid/mm positions and room-ID joins. Cache receipt and saved pose do not prove source freshness or localisation.

External HASM/editor inputs below are private-owner provenance, unavailable from the public checkout; full original replay needs the authentic matching inputs. No private files were opened or copied during this curation.

## Historical record

# Private Roborock saved-map reference for native DimOS research

Retrieved 2026-10-06 against deployed connector source `efc8cf5` (including prior TCP correction). This is a **saved vendor map retrieval**, not a new scan, mapping run, cleaning run, drive, or native DimOS map. No motion, docking, arm, account/configuration or deployment commands were issued.

## Provenance and receipt

- Authenticated supported service at loopback port 57010; existing private service-token was read in memory only, never printed or copied into artifacts.
- Initial `GET /api/map/cache`: HTTP 200, `cached:false`.
- One `POST /api/map/cache/refresh` with `{}`: HTTP 200, using implemented `get_map_v1` local L01 request and correlated MQTT reply. This endpoint updates only the map cache.
- Saved/cache receipt: `2026-10-06T13:31:56.769Z` (16:31:56.769 Asia/Jerusalem). **This is receipt/cache time, not verified vendor capture time.** Selected-device binding succeeded; private metadata retains device ID, map index/sequence and digest. Active map and current pose are explicitly unverified.
- Subsequent `GET /api/map/cache/image`: HTTP 200, valid privately retained PNG. Raw cache was independently SHA-256 checked and parsed with the connector's SHA-1/header/block validation.
- `GET /api/home/rooms` returned account room labels. One read-only local `get_room_mapping` via `/api/v1/rpc` returned segment/account-room associations. Joining these identifies a segment labeled **Living room**. This is stronger than a free-floating room label, but its relationship to this exact saved-map instance and the physical living room remains independently unverified.

## Private artifacts (do not commit, upload or publish)

Directory: `/Users/mpentax/.openclaw/workspace/dimos-v1/research/maps` (0700; all artifacts 0600).

- `roborock-saved-map.png`: viewable occupancy/segment raster; no position overlay or room text.
- `roborock-saved-map.bin`: verified raw V1 map.
- `private-map-metadata.json`: device binding, map identifiers, digest, receipt time, geometry and embedded positions.
- `cache-status.json`, `cache-status-after.json`, `refresh-receipt.json`: retrieval evidence.
- `rooms.json`, `room-mapping-receipt.json`, `named-segment-summary.json`: private label association and segment extents/counts.

Exact layout, dimensions, room bounds, robot coordinates and account IDs are deliberately omitted from this potentially versioned document. They are available in the private files above. No image or home-layout geometry is embedded here. No git commit or push was performed.

## Format, legend and limitations

The binary is Roborock V1 (`rr`, 20-byte main header), little-endian bounded blocks and trailing SHA-1 digest block. The image is one-byte-per-pixel row-major data; parser resolution is 0.05 m/pixel. Header `left`/`top` give grid offsets. Embedded robot/charger positions are signed integer coordinates; connector interpretation uses `floor(position / 50 - image origin)` for raw raster indexing. These conventions are grounded in the extracted Android plugin, not independently surveyed world coordinates. Angle interpretation, handedness, global reference origin, frame transform and active-map matching require validation.

Raw storage starts with the bottom row; PNG vertically flips rows for display. PNG dark background is unknown/unmapped, gray walls, light gray unsegmented floor, and four repeating pastel colors segmented floor. Repeating palette colors **do not uniquely identify rooms**. Exact pixel masks, not rectangular bounds or colors, define segments. The named segment summary bounds are axis-aligned coarse extents, not polygon boundaries or clear navigable area.

The parser found image, robot and charger position blocks. Embedded robot room-segment association agrees with the account-linked Living room segment; this is only an embedded saved-map position, **not verified current location**. No independent fresh pose or vendor capture timestamp is present in the retrieved evidence. Other V1 blocks are retained but not interpreted as localization or semantic boundary proof.

## Candidate native DimOS adapter outputs (research proposals, not implemented)

1. A private occupancy grid: preserve unknown/wall/floor classification, resolution, width/height and explicit origin/axis metadata. For ROS interoperability, a `nav_msgs/OccupancyGrid`-compatible payload is possible; unknown=-1, obstacle=100, floor=0 are conversion choices, not native vendor probabilities.
2. Separate room-label/segment raster or masks plus account-linked names; maintain original IDs privately and confidence/provenance for saved-map association. Do not infer room identity from PNG colors.
3. Export PGM/PNG plus YAML resolution/origin only after confirming coordinate conversion, or use an equivalent native DimOS grid wrapper identified by the source research worker. PNG alone is insufficient for metric planning.
4. Publish embedded saved pose as a timestamp-unknown historical observation in a vendor-map frame, never as live `map→base_link` localization. A validated frame transform and independent localization are required before navigation.

This reference can support format/adapter research and human comparison. It does not prove native DimOS mapping, current localization, route safety, or suitability for autonomous robot motion.
