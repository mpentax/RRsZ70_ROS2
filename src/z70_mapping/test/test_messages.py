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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.serialization import serialize_message, deserialize_message
from visualization_msgs.msg import Marker, MarkerArray
from z70_mapping import depth_node as dn, node as module
from z70_mapping.decoder import VendorMap, alignment, decode_map, room_names, spatial_context, transform_pose
from z70_mapping.depth_worker import camera_rays, metric_cloud
from z70_mapping.messages import grid_message, pose_message, markers_message


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
    monkeypatch.setattr(dn, 'MatchedFrames', Mock())
    return dn, publishers


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
