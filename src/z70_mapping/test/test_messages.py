import numpy as np
import pytest
from visualization_msgs.msg import Marker
from rclpy.serialization import serialize_message, deserialize_message
from visualization_msgs.msg import MarkerArray
from z70_mapping.decoder import decode_map
from z70_mapping.messages import grid_message, pose_message, markers_message
from z70_mapping.geometry import transform_pose
from test_decode import fixture


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
    import importlib.util
    import sys
    from pathlib import Path
    from z70_mapping import decoder
    from types import ModuleType, SimpleNamespace
    from unittest.mock import Mock
    from rclpy.node import Node
    from builtin_interfaces.msg import Time

    # Source-only test runs may not have built the custom interface package.
    try:
        import z70_interfaces.msg
    except ImportError:
        interfaces = ModuleType('z70_interfaces.msg')
        interfaces.MapSnapshot = type('MapSnapshot', (), {})
        monkeypatch.setitem(sys.modules, 'z70_interfaces.msg', interfaces)
    spec = importlib.util.spec_from_file_location(
        'z70_mapping._node_unit', Path(decoder.__file__).with_name('node.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
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
    from types import SimpleNamespace
    from builtin_interfaces.msg import Time
    return SimpleNamespace(
        evidence=SimpleNamespace(valid=True, stale=False, received_at=Time(sec=123)),
        raw=fixture(), rooms=[SimpleNamespace(segment_id=2, name='Kitchen')],
        map_index=1, map_sequence=sequence, active_floor_verified=False,
        current_pose_verified=False)


def test_first_snapshot_aligns_before_any_odometry_and_publishes_spatial_context(mapping_node):
    import json
    from z70_mapping.geometry import spatial_context
    from rclpy.qos import DurabilityPolicy
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
    assert json.loads(publisher.publish.call_args.args[0].data) == spatial_context(
        node.current[0], {2: 'Kitchen'})


def test_spatial_context_tracks_transformed_odometry(mapping_node):
    import json
    from nav_msgs.msg import Odometry
    from z70_mapping.geometry import spatial_context, transform_pose
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
    assert json.loads(pubs['/z70/map/spatial_context'].publish.call_args.args[0].data) == spatial_context(
        node.current[0], {2: 'Kitchen'}, transform_pose(anchor, node.odom))


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
    import json
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
