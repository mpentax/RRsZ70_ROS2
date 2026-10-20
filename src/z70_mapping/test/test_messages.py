import hashlib
import io
import json
import math
import struct
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import serialize_message, deserialize_message
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from z70_mapping import node as module
from z70_mapping.boundary import (
    BoundaryConfig,
    check_corridor_containment,
    compute_safe_interior,
    extract_segment_ids,
    generate_patrol_waypoints,
)
from z70_mapping.node import (
    VendorMap, alignment, camera_rays, decode_map, grid_message,
    markers_message, metric_cloud, pose_message, room_names,
    safe_grid_message, spatial_context, transform_pose,
)


def block(kind, data, extra=b''):
    return struct.pack('<HHI', kind, 8 + len(extra), len(data)) + extra + data


def fixture(pixels=bytes([1, 23, 0, 39, 255, 7]), width=3, height=2, extra_blocks=b''):
    image = block(2, pixels, struct.pack('<iiII', 100, 200, height, width))
    robot = block(8, struct.pack('<iii', 10075, 5025, 90))
    dock = block(1, struct.pack('<ii', 10000, 5000))
    body = image + robot + dock + extra_blocks + struct.pack('<HHI', 1024, 8, 20)
    header = b'rr' + struct.pack('<HI', 20, len(body) + 20) + bytes(12)
    raw = header + body
    return raw + hashlib.sha1(raw).digest()


def test_overlay_marker_geometry_and_serialization():
    m = decode_map(fixture(pixels=bytes([7] * 16), width=4, height=4))
    m.path = [(1., 2.), (3., 4.)]
    m.goto_path = [(5., 6.), (7., 8.)]
    m.virtual_walls = [(1., 2., 3., 4.)]
    m.zones = [[(1., 1.), (2., 1.), (2., 2.), (1., 2.)]]
    m.carpet = np.zeros((4, 4), dtype=bool)
    m.carpet[1:3, 1:3] = True
    result = deserialize_message(serialize_message(markers_message(m, {}, 12345)), MarkerArray)
    markers = {x.ns: x for x in result.markers if x.action == Marker.ADD}
    for ns, kind, width, color, z in [
        ('path', Marker.LINE_STRIP, .03, (.22, .74, .97, .9), .03),
        ('goto_path', Marker.LINE_STRIP, .03, (.98, .75, .14, .95), .035),
        ('virtual_walls', Marker.LINE_LIST, .04, (.94, .27, .27, .95), .05),
        ('zones', Marker.LINE_LIST, .035, (.98, .45, .09, .9), .04),
        ('carpet', Marker.LINE_LIST, .02, (.65, .55, .98, .85), .015),
    ]:
        x = markers[ns]
        assert x.id == 0 and x.type == kind
        assert x.scale.x == pytest.approx(width)
        assert (x.color.r, x.color.g, x.color.b, x.color.a) == pytest.approx(color)
        assert x.points and all(p.z == pytest.approx(z) for p in x.points)
    assert [(p.x, p.y) for p in markers['path'].points] == m.path
    assert [(p.x, p.y) for p in markers['goto_path'].points] == m.goto_path
    assert [(p.x, p.y) for p in markers['virtual_walls'].points] == [(1., 2.), (3., 4.)]
    assert len(markers['zones'].points) == 8
    assert markers['zones'].points[-1] == markers['zones'].points[0]
    carpet = markers['carpet'].points
    assert len(carpet) == 8 and carpet[0] == carpet[-1]
    assert {(p.x, p.y) for p in carpet} == {
        (10.075, 5.075), (10.075, 5.125), (10.125, 5.125), (10.125, 5.075)}


def test_empty_overlay_markers_are_omitted():
    m = decode_map(fixture())
    m.carpet = np.zeros(m.pixels.shape, dtype=bool)
    assert not {'path', 'goto_path', 'virtual_walls', 'zones', 'carpet'} & {
        x.ns for x in markers_message(m, {}, 0).markers}


@pytest.fixture
def mapping_node(monkeypatch):
    """Exercise callbacks with ROS transport mocked; no DDS or driver access."""
    publishers = {}
    def publisher(self, kind, topic, qos):
        publishers[topic] = Mock(qos=qos)
        return publishers[topic]
    monkeypatch.setattr(Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, name, default: SimpleNamespace(value=default))
    monkeypatch.setattr(Node, 'create_publisher', publisher)
    monkeypatch.setattr(Node, 'create_subscription', Mock())
    monkeypatch.setattr(Node, 'create_timer', Mock())
    monkeypatch.setattr(Node, 'get_clock', lambda self: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123))))
    broadcaster = Mock()
    monkeypatch.setattr(module, 'TransformBroadcaster', lambda node: broadcaster)
    return module.MappingNode(), publishers, broadcaster


def snapshot_message(sequence=1):
    return SimpleNamespace(
        evidence=SimpleNamespace(valid=True, stale=False, received_at=Time(sec=123)),
        raw=fixture(), rooms=[SimpleNamespace(segment_id=2, name='Kitchen')],
        map_index=1, map_sequence=sequence, active_floor_verified=False,
        current_pose_verified=False)


def test_first_snapshot_aligns_before_any_odometry_and_publishes_spatial_context(mapping_node):
    node, pubs, tf = mapping_node
    node.snapshot(snapshot_message())
    node.publish()
    assert node.odom == (0., 0., 0.)
    assert node.anchor == pytest.approx(node.current[0].robot)
    t = tf.sendTransform.call_args.args[0]
    assert (t.header.frame_id, t.child_frame_id) == ('map', 'odom')
    assert (t.transform.translation.x, t.transform.translation.y) == pytest.approx(node.anchor[:2])
    assert t.transform.rotation.z == pytest.approx(2**-.5)
    publisher = pubs['/z70/map/spatial_context']
    assert publisher.qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    payload = json.loads(publisher.publish.call_args.args[0].data)
    assert payload.pop('boundary_contract')['frame_id'] == 'map'
    assert payload == spatial_context(node.current[0], {2: 'Kitchen'})


def test_spatial_context_tracks_transformed_odometry(mapping_node):
    node, pubs, tf = mapping_node
    node.snapshot(snapshot_message())
    node.publish()
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'
    odom.pose.pose.position.x = 1.
    odom.pose.pose.orientation.w = 1.
    node.on_odom(odom)
    anchor = node.anchor
    node.publish()
    assert node.anchor == anchor
    assert tf.sendTransform.call_count == 2
    payload = json.loads(pubs['/z70/map/spatial_context'].publish.call_args.args[0].data)
    assert payload.pop('boundary_contract')['frame_id'] == 'map'
    assert payload == spatial_context(node.current[0], {2: 'Kitchen'}, transform_pose(anchor, node.odom))


def test_alignment_refresh_only_on_new_sequence(mapping_node):
    node, _, _ = mapping_node
    assert node.realign is True
    node.snapshot(snapshot_message())
    node.publish()
    initial = node.anchor
    node.odom = (1., 0., 0.)
    node.snapshot(snapshot_message())
    node.publish()
    assert node.anchor == initial
    node.snapshot(snapshot_message(sequence=2))
    node.publish()
    assert node.last_sequence == 2
    assert node.anchor != initial
    assert transform_pose(node.anchor, node.odom) == pytest.approx(node.current[0].robot)
    anchor = node.anchor
    node.realign = False
    node.odom = (2., 0., 0.)
    node.snapshot(snapshot_message(sequence=3))
    node.publish()
    assert node.anchor == anchor


def test_missing_heading_publishes_context_without_tf(mapping_node):
    node, pubs, tf = mapping_node
    node.snapshot(snapshot_message())
    node.pending[0].robot = (10.075, 5.025, None)
    node.publish()
    assert node.anchor is None
    tf.sendTransform.assert_not_called()
    ctx = json.loads(pubs['/z70/map/spatial_context'].publish.call_args.args[0].data)
    assert ctx['current_room'] == {'segment_id': 2, 'name': 'Kitchen'}
    assert ctx['dock']['bearing_rad'] is None


def test_ros_grid_pose_and_closed_room_boundaries():
    m = decode_map(fixture())
    g = grid_message(m, 12345)
    assert g.header.frame_id == 'map'
    assert (g.info.width, g.info.height, g.info.resolution) == pytest.approx((3, 2, .05))
    assert list(g.data) == [100, 0, -1, 0, 0, 0]
    assert g.info.origin.position.x == 10.
    p = pose_message(m.robot, 12345)
    assert p.header.stamp.sec == 12
    assert p.header.stamp.nanosec == 345000000
    assert p.pose.orientation.z == pytest.approx(2**-.5)
    markers = markers_message(m, {2:'Kitchen'}, 12345)
    assert any(x.text == 'Kitchen' for x in markers.markers)
    assert any(x.ns == 'dock' for x in markers.markers)
    assert all(x.header.frame_id == 'map' for x in markers.markers)


def test_vendor_grid_bottom_row_and_mm_positions():
    m = decode_map(fixture())
    assert (m.width, m.height, m.resolution) == (3, 2, .05)
    assert m.origin == (10., 5.)
    assert m.occupancy.tolist() == [[100, 0, -1], [0, 0, 0]]
    assert m.robot == pytest.approx((10.075, 5.025, 1.5707963267948966))
    assert m.dock == (10., 5., None)
    assert sorted(m.rooms) == [2, 4]
    assert m.rooms[2]['centroid'] == pytest.approx((10.075, 5.025))


def test_zero_vendor_photo_obstacles_are_explicit_not_assumed_from_floor():
    empty = b''.join(struct.pack('<HHIHH', kind, 12, 0, 0, 0) for kind in (12, 15, 16))
    m = decode_map(fixture(extra_blocks=empty))
    assert m.obstacle_counts == {12: 0, 15: 0, 16: 0}


@pytest.mark.parametrize('kind,attribute', [(3, 'path'), (9, 'goto_path')])
def test_path_blocks_in_metres(kind, attribute):
    m = decode_map(fixture(extra_blocks=block(kind, struct.pack('<4H', 100, 200, 1200, 2300))))
    assert getattr(m, attribute) == [(0.1, 0.2), (1.2, 2.3)]


@pytest.mark.parametrize('kind', [18, 19, 47, 53])
def test_zone_blocks_in_metres(kind):
    corners = (1000, 2000, 3000, 2000, 3000, 4000, 1000, 4000)
    payload = struct.pack('<8H', *corners)
    if kind in (47, 53):
        payload = struct.pack('<2H', 123, 456) + payload
    m = decode_map(fixture(extra_blocks=block(kind, payload * 2)))
    assert m.zones == [[(1., 2.), (3., 2.), (3., 4.), (1., 4.)]] * 2


def test_virtual_walls_and_carpet():
    extras = block(10, struct.pack('<8H', 100, 200, 300, 400, 500, 600, 700, 800))
    extras += block(17, bytes([0, 1, 2, 0, 255, 0]))
    m = decode_map(fixture(extra_blocks=extras))
    assert m.virtual_walls == [(0.1, 0.2, 0.3, 0.4), (0.5, 0.6, 0.7, 0.8)]
    assert m.carpet.dtype == bool
    assert m.carpet.tolist() == [[False, True, True], [False, True, False]]


@pytest.mark.parametrize('kind,attribute', [(3, 'path'), (9, 'goto_path'),
    (10, 'virtual_walls'), (18, 'zones'), (19, 'zones'), (47, 'zones'), (53, 'zones')])
def test_malformed_optional_geometry_is_ignored(kind, attribute):
    m = decode_map(fixture(extra_blocks=block(kind, b'\x00')))
    assert getattr(m, attribute) == []


def test_wrong_carpet_size_is_ignored():
    assert decode_map(fixture(extra_blocks=block(17, b'\x00'))).carpet is None


def test_bad_digest_rejected():
    raw = bytearray(fixture()); raw[30] ^= 1
    with pytest.raises(ValueError, match='integrity'):
        decode_map(raw)


def test_bad_grid_extent_rejected():
    with pytest.raises(ValueError, match='image'):
        decode_map(fixture(width=4))


def context_map():
    pixels = np.full((101, 101), 7, dtype=np.uint8)
    occupancy = np.zeros(pixels.shape, dtype=np.int8)
    return VendorMap(101, 101, (-2.525, -2.525), pixels, occupancy,
                     (0., 0., math.pi / 2), (1., 0., None),
                     {2: {'centroid': (0., 2.), 'area_m2': 3.12345},
                      4: {'centroid': (1., 0.), 'area_m2': 1.}}, {}, {})


def test_spatial_context_room_identification_and_egocentric_geometry():
    m = context_map()
    m.pixels[50, 50] = 23
    ctx = spatial_context(m, {2: 'Kitchen'})
    assert ctx['robot_pose_map'] == {'x': 0., 'y': 0., 'yaw': round(math.pi / 2, 3)}
    assert ctx['current_room'] == {'segment_id': 2, 'name': 'Kitchen'}
    assert ctx['rooms'] == [
        {'segment_id': 4, 'name': 'Segment 4', 'centroid': [1., 0.], 'area_m2': 1.,
         'distance_m': 1., 'bearing_rad': round(-math.pi / 2, 3)},
        {'segment_id': 2, 'name': 'Kitchen', 'centroid': [0., 2.], 'area_m2': 3.123,
         'distance_m': 2., 'bearing_rad': 0.}]
    assert ctx['dock'] == {'x': 1., 'y': 0., 'yaw': None, 'distance_m': 1.,
                           'bearing_rad': round(-math.pi / 2, 3)}
    assert spatial_context(m, {}, (1., 0., 0.))['current_room']['segment_id'] == 4


@pytest.mark.parametrize('pose,occupancy,expected', [
    ((0., 0., None), 0, 4), ((0., 0., None), 100, None),
    ((0., 0., None), -1, None), ((-2., -2., 0.), 0, None),
    ((-3., 0., 0.), 0, None), ((3., 0., 0.), 0, None),
])
def test_current_room_fallback_only_on_free_in_bounds_nearby_cell(pose, occupancy, expected):
    m = context_map()
    m.occupancy[:] = occupancy
    current = spatial_context(m, {}, pose)['current_room']
    assert (current['segment_id'] if current else None) == expected


def test_spatial_context_handles_missing_pose_and_heading():
    m = context_map()
    m.robot = None
    ctx = spatial_context(m, {})
    assert ctx['current_room'] is None and ctx['clearance_sectors_m'] == {}
    assert [r['segment_id'] for r in ctx['rooms']] == [2, 4]
    assert all(r['distance_m'] is None and r['bearing_rad'] is None for r in ctx['rooms'])
    assert ctx['dock']['distance_m'] is None
    ctx = spatial_context(m, {}, (0., 0., None))
    assert ctx['dock']['distance_m'] == 1. and ctx['dock']['bearing_rad'] is None
    m.dock = None
    assert spatial_context(m, {})['dock'] is None


def test_clearance_rays_cover_eight_relative_directions():
    m = context_map()
    m.occupancy[:] = -1
    m.occupancy[70, 50] = 100
    ctx = spatial_context(m, {})
    sectors = ctx['clearance_sectors_m']
    assert sectors == pytest.approx({
        'front': .975, 'front_left': 3.575, 'left': 2.525,
        'rear_left': 3.575, 'rear': 2.525, 'rear_right': 3.575,
        'right': 2.525, 'front_right': 3.575}, abs=.025)
    assert spatial_context(m, {}, (3., 0., 0.))['clearance_sectors_m'] == dict.fromkeys(sectors, 0.)
    m.occupancy[50, 50] = 100
    assert spatial_context(m, {})['clearance_sectors_m'] == dict.fromkeys(sectors, 0.)


def test_clearance_rays_stop_at_five_metres():
    m = context_map()
    m.width = m.height = 301
    m.origin = (-7.525, -7.525)
    m.pixels = np.full((301, 301), 7, dtype=np.uint8)
    m.occupancy = np.zeros((301, 301), dtype=np.int8)
    assert set(spatial_context(m, {})['clearance_sectors_m'].values()) == {5.}


def test_alignment_rotates_then_translates_odom_into_vendor_map():
    vendor = (27.4, 24.9, math.pi / 2)
    odom = (1., 2., -.2)
    t = alignment(vendor, odom)
    assert transform_pose(t, odom) == pytest.approx(vendor)
    assert transform_pose(t, (1., 2., 0.))[2] == pytest.approx(math.pi / 2 + .2)


def test_missing_vendor_heading_does_not_invent_alignment():
    assert alignment((1., 2., None), (0., 0., 0.)) is None


def test_room_names_only_join_explicit_vendor_segment_mapping():
    assert room_names([[2, '123', 14], [4, '456', 6]], [{'id': '123', 'name': 'Kitchen'}]) == {2: 'Kitchen'}


CAL = {'width': 4, 'height': 4, 'f_px': 10., 'cx': 1.5, 'cy': 1.5, 'k1': 0., 'k2': 0.,
       'camera_height_mm': 50., 'R_floor_to_cam': [[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]]}
RAYS, FLOOR = camera_rays(CAL, 4, 4)


def test_pinhole_optical_backprojection_and_floor_scale():
    depth = np.where(np.isfinite(FLOOR), FLOOR * 2, 2.)
    points, quality = metric_cloud(depth, RAYS, FLOOR, min_floor_pixels=2)
    assert quality['floor_aligned']
    assert quality['scale'] == pytest.approx(.5)
    assert np.isfinite(points).all()
    assert RAYS[3, 3].tolist() == pytest.approx([.15, .15, 1.])
    assert any(np.allclose(p, RAYS[3, 3] * FLOOR[3, 3]) for p in points)


def test_absent_floor_is_not_reported_as_metric_alignment():
    points, quality = metric_cloud(np.ones((4, 4)), RAYS, np.full((4, 4), np.inf))
    assert not quality['floor_aligned']
    assert quality['scale'] == 1.
    assert len(points) == 16


def test_reject_nonfinite_and_out_of_range_cloud_points():
    depth = np.full((4, 4), np.nan)
    depth[0, 0] = 10.
    depth[0, 1] = -.5
    points, _ = metric_cloud(depth, RAYS, np.full((4, 4), np.inf))
    assert points.shape == (0, 3)


def _mock_depth_node(monkeypatch, python_override=None):
    publishers = {}

    def publisher(self, kind, topic, qos):
        publishers[topic] = Mock(qos=qos)
        return publishers[topic]

    monkeypatch.setattr(Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(Node, 'declare_parameter',
                        lambda self, name, default: SimpleNamespace(
                            value=python_override if (python_override and name == 'depth_python') else default))
    monkeypatch.setattr(Node, 'create_publisher', publisher)
    monkeypatch.setattr(Node, 'create_timer', Mock())
    monkeypatch.setattr(module, 'MatchedFrames', Mock())
    return module, publishers


def test_depth_node_missing_venv_publishes_unavailable_status_and_closes_cleanly(monkeypatch):
    dn_mod, publishers = _mock_depth_node(monkeypatch, '/nonexistent/roboclaw-depth/python')
    node = dn_mod.DepthNode()
    assert node.process is None
    assert node.worker is None
    node.publish()
    assert publishers['/z70/depth/status'].qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    status = json.loads(publishers['/z70/depth/status'].publish.call_args.args[0].data)
    assert status == {'available': False, 'error': 'FileNotFoundError'}
    publishers['/camera/front/depth/points'].publish.assert_not_called()
    node.close()


def test_depth_node_bad_packet_recovers_on_subsequent_frame(monkeypatch):
    dn_mod, publishers = _mock_depth_node(monkeypatch)

    good_packet = io.BytesIO()
    np.savez(good_packet, xyz=np.array([[0.1, 0.2, 0.8]], dtype='<f4'),
             quality=np.array(json.dumps({'floor_aligned': True, 'point_count': 1})))
    good_bytes = good_packet.getvalue()

    responses = [
        struct.pack('<I', 5 * 1024 * 1024),
        struct.pack('<I', len(good_bytes)),
        good_bytes,
    ]
    fake_proc = SimpleNamespace(stdin=io.BytesIO(), stdout=SimpleNamespace(fileno=lambda: 0, close=Mock()),
                                wait=Mock(), terminate=Mock(), kill=Mock())
    monkeypatch.setattr(dn_mod.subprocess, 'Popen', lambda *a, **kw: fake_proc)
    monkeypatch.setattr(dn_mod.threading, 'Thread', lambda **kw: SimpleNamespace(start=Mock(), join=Mock()))

    node = dn_mod.DepthNode()
    node.read_exact = lambda count: responses.pop(0)

    def step_once(sec):
        node.latest = SimpleNamespace(header=SimpleNamespace(frame_id='camera_optical_frame',
                                                             stamp=SimpleNamespace(sec=sec, nanosec=0)),
                                      data=b'jpeg')

        class OneShotEvent(dn_mod.threading.Event):
            def wait(self, timeout=None):
                self.set()
                return True

        node.stop_event = OneShotEvent()
        node.run_worker(0.01)

    step_once(1)
    node.publish()
    err_status = json.loads(publishers['/z70/depth/status'].publish.call_args.args[0].data)
    assert err_status['error'] == 'ValueError'
    publishers['/camera/front/depth/points'].publish.assert_not_called()

    step_once(2)
    node.publish()
    ok_status = json.loads(publishers['/z70/depth/status'].publish.call_args.args[0].data)
    assert ok_status['floor_aligned'] is True
    publishers['/camera/front/depth/points'].publish.assert_called_once()
    node.close()


def floor(shape=(21, 30), resolution=.1):
    pixels = np.full(shape, 15, np.uint8)
    return VendorMap(shape[1], shape[0], (0., 0.), pixels,
                     np.zeros(shape, np.int8), (.55, 1.05, 0.), None,
                     {1: {}, 2: {}}, {}, {}, resolution=resolution)


def config(**kwargs):
    values = dict(allowed_segments=(1, 2), chassis_radius_m=.2)
    values.update(kwargs)
    return BoundaryConfig(**values)


def test_union_before_erosion_preserves_shared_room_boundary():
    m = floor()
    m.pixels[:, 15:] = 23
    result = compute_safe_interior(m, {}, config())
    assert result.safe_mask[10, 14:16].all()
    assert result.reachable_mask[10, 14:16].all()
    assert result.room_connectivity[2]['reachable_cells'] > 0
    assert result.lost_rooms == []
    only_first_room = compute_safe_interior(m, {}, config(allowed_segments=(1,)))
    assert not only_first_room.safe_mask[10, 14:16].any()


def test_waypoints_are_deterministic_clear_and_cover_each_reachable_room():
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
    m = floor((8, 8), resolution=1.)
    m.virtual_walls = [(0., 0., 8., 8.)]
    r = compute_safe_interior(m, {}, config(chassis_radius_m=0.))
    assert all(r.forbidden_mask[i, i] for i in range(8))
    assert r.forbidden_mask[3, 4] and r.forbidden_mask[4, 3]
    assert r.component_labels.max() == 2


def test_map_edges_are_boundaries_and_clearance_is_conservative():
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
    m = floor((3, 3), resolution=.1)
    r = compute_safe_interior(m, {}, config(allowed_segments=(1,), chassis_radius_m=.05,
                                          waypoint_clearance_m=.25))
    assert len(r.waypoints) == 1
    assert not r.waypoints[0]['clearance_preference_met']
    assert r.waypoints[0]['grid_xy'] == [1, 1]


def test_empty_allowlist_is_fail_closed():
    r = compute_safe_interior(floor(), {}, config(allowed_segments=()))
    assert not r.allowed_mask.any() and not r.reachable_mask.any()
    assert set(np.unique(r.safe_grid)) == {100}


def test_node_publishes_latched_safe_grid_and_contract_on_both_json_topics(mapping_node):
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


def test_dynamic_obstacle_layer_subtracts_cells_and_publishes_world_markers(mapping_node):
    node, pubs, _ = mapping_node
    node.snapshot(snapshot_message())
    node.publish()
    m = node.current[0]
    cx = float(m.origin[0] + 1.5 * m.resolution)
    cy = float(m.origin[1] + 0.5 * m.resolution)
    node.on_world_update(String(data=json.dumps({
        'action': 'upsert',
        'x': cx,
        'y': cy,
        'radius_m': 0.08,
        'category': 'blanket',
        'mobility': 'draggable',
        'relevance': 'deferred',
        'detail': 'UNSUPPORTED_GARMENT_SKIP',
    })))
    node.publish()
    dyn_msg = pubs['/z70/map/dynamic_obstacles'].publish.call_args.args[0]
    assert max(dyn_msg.data) == 100
    world_markers = pubs['/z70/world/markers'].publish.call_args.args[0]
    assert any(mk.ns == 'world_obstacles' for mk in world_markers.markers)
    world_json = json.loads(pubs['/z70/world/objects'].publish.call_args.args[0].data)
    assert len(world_json['obstacles']) == 1
    assert world_json['obstacles'][0]['category'] == 'blanket'
    assert world_json['obstacles'][0]['mobility'] == 'draggable'
    assert world_json['obstacles'][0]['relevance'] == 'deferred'
