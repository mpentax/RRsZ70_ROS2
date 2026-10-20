import numpy as np
import pytest
from z70_mapping.depth_geometry import camera_rays, metric_cloud

CAL = {'width':4, 'height':4, 'f_px':10., 'cx':1.5, 'cy':1.5, 'k1':0., 'k2':0.,
       'camera_height_mm':50., 'R_floor_to_cam':[[0.,-1.,0.],[0.,0.,-1.],[1.,0.,0.]]}


def test_pinhole_optical_backprojection_and_floor_scale():
    rays, floor = camera_rays(CAL, 4, 4)
    # The two bottom rows hit a plane; upper rows have no floor intersection.
    depth = np.where(np.isfinite(floor), floor * 2, 2.)
    points, quality = metric_cloud(depth, rays, floor, min_floor_pixels=2)
    assert quality['floor_aligned']
    assert quality['scale'] == pytest.approx(.5)
    assert np.isfinite(points).all()
    assert rays[3,3].tolist() == pytest.approx([.15,.15,1.])
    assert any(np.allclose(p, rays[3,3] * floor[3,3]) for p in points)


def test_absent_floor_is_not_reported_as_metric_alignment():
    rays, floor = camera_rays(CAL, 4, 4)
    points, quality = metric_cloud(np.ones((4,4)), rays, np.full((4,4),np.inf))
    assert not quality['floor_aligned']
    assert quality['scale'] == 1.
    assert len(points) == 16


def test_reject_nonfinite_and_out_of_range_cloud_points():
    rays, floor = camera_rays(CAL, 4, 4)
    depth = np.full((4,4),np.nan); depth[0,0] = 10.; depth[0,1] = -.5
    points, _ = metric_cloud(depth,rays,np.full((4,4),np.inf))
    assert points.shape == (0,3)


def test_depth_node_missing_venv_publishes_unavailable_status_and_closes_cleanly(monkeypatch):
    import json
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy
    import z70_mapping.depth_node as dn

    publishers = {}
    def publisher(self, kind, topic, qos):
        publishers[topic] = Mock(qos=qos)
        return publishers[topic]
    monkeypatch.setattr(Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(Node, 'declare_parameter',
                        lambda self, name, default: SimpleNamespace(
                            value='/nonexistent/roboclaw-depth/python' if name == 'depth_python' else default))
    monkeypatch.setattr(Node, 'create_publisher', publisher)
    monkeypatch.setattr(Node, 'create_timer', Mock())
    monkeypatch.setattr(dn, 'MatchedFrames', Mock())

    node = dn.DepthNode()
    assert node.process is None
    assert node.worker is None
    node.publish()
    assert publishers['/z70/depth/status'].qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    status = json.loads(publishers['/z70/depth/status'].publish.call_args.args[0].data)
    assert status == {'available': False, 'error': 'FileNotFoundError'}
    publishers['/camera/front/depth/points'].publish.assert_not_called()
    node.close()


def test_depth_node_bad_packet_recovers_on_subsequent_frame(monkeypatch):
    import io
    import json
    import struct
    from types import SimpleNamespace
    from unittest.mock import Mock
    from rclpy.node import Node
    import z70_mapping.depth_node as dn

    publishers = {}
    def publisher(self, kind, topic, qos):
        publishers[topic] = Mock(qos=qos)
        return publishers[topic]
    monkeypatch.setattr(Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, name, default: SimpleNamespace(value=default))
    monkeypatch.setattr(Node, 'create_publisher', publisher)
    monkeypatch.setattr(Node, 'create_timer', Mock())
    monkeypatch.setattr(dn, 'MatchedFrames', Mock())

    good_packet = io.BytesIO()
    np.savez(good_packet, xyz=np.array([[0.1, 0.2, 0.8]], dtype='<f4'),
             quality=np.array(json.dumps({'floor_aligned': True, 'point_count': 1})))
    good_bytes = good_packet.getvalue()

    responses = [
        struct.pack('<I', 5 * 1024 * 1024),  # Oversized packet -> ValueError
        struct.pack('<I', len(good_bytes)),
        good_bytes,
    ]
    fake_proc = SimpleNamespace(stdin=io.BytesIO(), stdout=SimpleNamespace(fileno=lambda: 0, close=Mock()),
                                wait=Mock(), terminate=Mock(), kill=Mock())
    monkeypatch.setattr(dn.subprocess, 'Popen', lambda *a, **kw: fake_proc)
    monkeypatch.setattr(dn.threading, 'Thread', lambda **kw: SimpleNamespace(start=Mock(), join=Mock()))

    node = dn.DepthNode()
    node.read_exact = lambda count: responses.pop(0)

    def step_once(sec):
        node.latest = SimpleNamespace(header=SimpleNamespace(frame_id='camera_optical_frame',
                                                             stamp=SimpleNamespace(sec=sec, nanosec=0)),
                                      data=b'jpeg')
        class OneShotEvent(dn.threading.Event):
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
