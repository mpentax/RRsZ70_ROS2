"""Offline boundary contracts; synthetic grids contain no private map bytes."""
import numpy as np
import pytest
from z70_mapping.decoder import VendorMap
from test_messages import mapping_node, snapshot_message


def floor(shape=(21, 30), resolution=.1):
    pixels = np.full(shape, 15, np.uint8)
    return VendorMap(shape[1], shape[0], (0., 0.), pixels,
                     np.zeros(shape, np.int8), (.55, 1.05, 0.), None,
                     {1: {}, 2: {}}, {}, {}, resolution=resolution)


def config(**kwargs):
    from z70_mapping.boundary import BoundaryConfig
    values = dict(allowed_segments=(1, 2), chassis_radius_m=.2)
    values.update(kwargs)
    return BoundaryConfig(**values)


def test_union_before_erosion_preserves_shared_room_boundary():
    from z70_mapping.boundary import compute_safe_interior
    m = floor()
    m.pixels[:, 15:] = 23
    result = compute_safe_interior(m, {}, config())
    assert result.safe_mask[10, 14:16].all()
    assert result.reachable_mask[10, 14:16].all()
    assert result.room_connectivity[2]['reachable_cells'] > 0
    assert result.lost_rooms == []
    # Control: independent per-room erosion would erase this shared doorway.
    only_first_room = compute_safe_interior(m, {}, config(allowed_segments=(1,)))
    assert not only_first_room.safe_mask[10, 14:16].any()


def test_waypoints_are_deterministic_clear_and_cover_each_reachable_room():
    from z70_mapping.boundary import compute_safe_interior, generate_patrol_waypoints
    m = floor()
    m.pixels[:, 15:] = 23
    r = compute_safe_interior(m, {}, config())
    points = generate_patrol_waypoints(r)
    assert points == generate_patrol_waypoints(r)
    assert {p['segment_id'] for p in points} == {1, 2}
    for p in points:
        x, y = p['grid_xy']
        assert r.reachable_mask[y, x]
        assert p['clearance_m'] >= .25
        assert p['x'] == pytest.approx((x + .5) * m.resolution)
        assert p['y'] == pytest.approx((y + .5) * m.resolution)


def test_corridor_checks_full_swept_disk_and_diagonal_corner():
    from z70_mapping.boundary import check_corridor_containment
    mask = np.ones((10, 10), bool)
    mask[4, 4] = False
    assert check_corridor_containment(mask, (1.5, 2.5), (8.5, 2.5), .4)
    assert not check_corridor_containment(mask, (1.5, 3.7), (8.5, 3.7), .4)
    assert not check_corridor_containment(mask, (1.5, 1.5), (8.5, 8.5), 0.)
    assert not check_corridor_containment(mask, (3.5, 4.5), (4.5, 3.5), 0.)
    assert not check_corridor_containment(mask, (.1, .5), (.1, 8.5), .2)
    assert not check_corridor_containment(mask, (float('nan'), 1.), (2., 2.), 0.)
    assert not check_corridor_containment(mask, (2., 2.), (3., 3.), -1.)


def test_safe_grid_serialization_markers_and_corridor_metric_coordinates():
    from rclpy.serialization import serialize_message, deserialize_message
    from nav_msgs.msg import OccupancyGrid
    from z70_mapping.boundary import compute_safe_interior, check_corridor_containment
    from z70_mapping.messages import safe_grid_message, markers_message
    m = floor()
    m.rooms = {}
    m.origin = (-2., 3.)
    r = compute_safe_interior(m, {}, config())
    msg = deserialize_message(serialize_message(safe_grid_message(r, 12345, load_at=1000)), OccupancyGrid)
    assert msg.header.frame_id == 'map'
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (12, 345000000)
    assert msg.info.map_load_time.sec == 1
    assert (msg.info.width, msg.info.height) == (m.width, m.height)
    assert msg.info.resolution == pytest.approx(m.resolution)
    assert (msg.info.origin.position.x, msg.info.origin.position.y) == m.origin
    assert set(msg.data) == {0, 100}
    assert np.array_equal(np.asarray(msg.data).reshape(m.height, m.width) == 0, r.reachable_mask)
    point = r.waypoints[0]
    xy = (point['x'], point['y'])
    assert check_corridor_containment(r, xy, xy, 0.)
    assert check_corridor_containment(msg, xy, xy, 0.)
    markers = markers_message(m, {}, 12345, boundary_result=r)
    assert sum(x.ns == 'patrol_waypoints' for x in markers.markers) == len(r.waypoints)
    assert any(x.ns == 'boundary_contract' for x in markers.markers)


def test_unsegmented_unknown_occupied_and_nonallowlisted_are_excluded():
    from z70_mapping.boundary import compute_safe_interior, extract_segment_ids
    m = floor((5, 8))
    m.pixels[2, :6] = [7, 255, 15, 15, 16, 31]
    m.occupancy[2, 2] = -1
    m.occupancy[2, 3] = 100
    ids = extract_segment_ids(m)
    assert ids[2, :6].tolist() == [0, 0, 0, 0, 0, 3]
    r = compute_safe_interior(m, {}, config(chassis_radius_m=0.))
    assert not r.allowed_mask[2, :6].any()
    assert not r.reachable_mask[2, :6].any()


def test_zones_and_virtual_walls_are_subtracted_before_clearance():
    from z70_mapping.boundary import compute_safe_interior
    m = floor()
    m.zones = [[(.61, .61), (.89, .61), (.89, .89), (.61, .89)]]
    m.virtual_walls = [(1.55, .05, 1.55, 2.05)]
    r = compute_safe_interior(m, {}, config())
    assert r.forbidden_mask[6:9, 6:9].all()
    assert r.forbidden_mask[:, 15].all()
    assert not r.interior_mask[r.forbidden_mask].any()
    assert not r.safe_mask[10, 14:17].any()
    assert r.room_connectivity[1]['forbidden_cells'] > 0


def test_diagonal_virtual_wall_supercover_has_no_corner_gaps():
    from z70_mapping.boundary import compute_safe_interior
    m = floor((8, 8), resolution=1.)
    m.virtual_walls = [(0., 0., 8., 8.)]
    r = compute_safe_interior(m, {}, config(chassis_radius_m=0.))
    assert all(r.forbidden_mask[i, i] for i in range(8))
    assert r.forbidden_mask[3, 4] and r.forbidden_mask[4, 3]
    assert r.component_labels.max() == 2


def test_map_edges_are_boundaries_and_clearance_is_conservative():
    from z70_mapping.boundary import compute_safe_interior
    m = floor()
    r = compute_safe_interior(m, {}, config())
    assert not r.safe_mask[:2, :].any()
    assert not r.safe_mask[-2:, :].any()
    assert not r.safe_mask[:, :2].any()
    assert not r.safe_mask[:, -2:].any()
    assert r.safe_mask[2, 2]
    assert r.clearance_m[0, 0] < .05
    assert r.clearance_m[2, 2] < .25


def test_four_connectivity_rejects_diagonal_floor_touch():
    from z70_mapping.boundary import compute_safe_interior
    m = floor((3, 3), resolution=1.)
    m.occupancy[:] = 100
    m.occupancy[0, 0] = m.occupancy[1, 1] = 0
    m.pixels[1, 1] = 23
    r = compute_safe_interior(m, {}, config(chassis_radius_m=0.), robot_pose_map=(.5, .5))
    assert r.component_labels.max() == 2
    assert r.reachable_mask[0, 0] and not r.reachable_mask[1, 1]
    assert r.lost_rooms == [2]
    assert r.room_connectivity[2]['safe_cells'] == 1
    assert r.room_connectivity[2]['disconnected']


def test_narrow_doorway_reports_lost_allowed_room():
    from z70_mapping.boundary import compute_safe_interior
    m = floor()
    m.pixels[:, 16:] = 23
    m.occupancy[:, 14:16] = 100
    m.occupancy[10, 14:16] = 0
    r = compute_safe_interior(m, {2: 'Narrow doorway'}, config())
    assert r.room_connectivity[2]['raw_cells'] > 0
    assert r.room_connectivity[2]['safe_cells'] > 0
    assert r.room_connectivity[2]['reachable_cells'] == 0
    assert r.room_connectivity[2]['reachable_fraction'] == 0.
    assert r.lost_rooms == [2]
    assert 'disconnected_after_exclusion_or_clearance' in r.room_connectivity[2]['reasons']
    assert {p['segment_id'] for p in r.waypoints} == {1}


def test_completely_forbidden_room_and_nonallowed_rooms_are_reported():
    from z70_mapping.boundary import compute_safe_interior
    m = floor()
    m.pixels[:, 15:] = 23
    m.rooms[3] = {}
    m.zones = [[(1.5, 0.), (3., 0.), (3., 2.1), (1.5, 2.1)]]
    r = compute_safe_interior(m, {}, config())
    assert r.lost_rooms == [2]
    assert r.room_connectivity[2]['safe_cells'] == 0
    assert r.room_connectivity[2]['forbidden_cells'] == r.room_connectivity[2]['raw_cells']
    assert r.room_connectivity[3]['allowed'] is False
    assert r.room_connectivity[3]['reasons'] == ['not_allowlisted']


def test_component_selection_pose_nearest_and_largest_fallback():
    from z70_mapping.boundary import compute_safe_interior
    m = floor()
    r = compute_safe_interior(m, {}, config(), robot_pose_map=(.55, 1.05))
    assert r.selection_method == 'robot_safe_cell'
    r = compute_safe_interior(m, {}, config(), robot_pose_map=(.1, 1.05))
    assert r.selection_method == 'nearest_safe_cell'
    m.robot = None
    r = compute_safe_interior(m, {}, config())
    assert r.selection_method == 'largest_component_fallback'
    assert r.waypoints
    m.occupancy[:] = -1
    r = compute_safe_interior(m, {}, config())
    assert r.selection_method == 'empty'
    assert not r.reachable_mask.any() and r.waypoints == []


def test_explicit_physical_envelope_parameters_and_invalid_config():
    from z70_mapping.boundary import BoundaryConfig, compute_safe_interior
    c = config(chassis_radius_m=.175, deployed_arm_extension_m=.1,
               localization_uncertainty_m=.025, stopping_distance_m=.05)
    assert c.clearance_m == pytest.approx(.35)
    m = floor()
    stowed = compute_safe_interior(m, {}, config())
    deployed = compute_safe_interior(m, {}, c)
    assert np.all(~deployed.safe_mask | stowed.safe_mask)
    assert deployed.safe_mask.sum() < stowed.safe_mask.sum()
    for kwargs in ({'chassis_radius_m': -1}, {'waypoint_spacing_m': 0},
                   {'stopping_distance_m': float('nan')}, {'allowed_segments': (0,)},
                   {'allowed_segments': (31,)}):
        with pytest.raises(ValueError):
            BoundaryConfig(**kwargs)


def test_waypoint_clearance_preference_fallback_never_leaves_reachable():
    from z70_mapping.boundary import compute_safe_interior
    m = floor((3, 3), resolution=.1)
    r = compute_safe_interior(m, {}, config(allowed_segments=(1,), chassis_radius_m=.05,
                                          waypoint_clearance_m=.25))
    assert len(r.waypoints) == 1
    assert not r.waypoints[0]['clearance_preference_met']
    assert r.waypoints[0]['grid_xy'] == [1, 1]


def test_empty_allowlist_is_fail_closed():
    from z70_mapping.boundary import compute_safe_interior
    r = compute_safe_interior(floor(), {}, config(allowed_segments=()))
    assert not r.allowed_mask.any() and not r.reachable_mask.any()
    assert set(np.unique(r.safe_grid)) == {100}


def test_node_publishes_latched_safe_grid_and_contract_on_both_json_topics(mapping_node):
    import json
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    node, pubs, _ = mapping_node
    node.snapshot(snapshot_message())
    node.publish()
    publisher = pubs['/z70/map/safe_grid']
    assert publisher.qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert publisher.qos.reliability == ReliabilityPolicy.RELIABLE
    message = publisher.publish.call_args.args[0]
    assert message.header.frame_id == 'map'
    assert set(message.data) <= {0, 100}
    contracts = [json.loads(pubs[topic].publish.call_args.args[0].data)['boundary_contract']
                 for topic in ('/z70/map/status', '/z70/map/spatial_context')]
    assert contracts[0] == contracts[1]
    assert contracts[0]['room_connectivity']['2']['raw_cells'] > 0

