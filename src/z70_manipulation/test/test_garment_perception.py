"""Garment silhouette projection, tracking, and worker framing."""
import io
import json
from pathlib import Path
import struct
import subprocess
import sys
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]


def geometry():
    from z70_manipulation import garment_geometry
    return garment_geometry


def camera():
    from z70_description.floor_geometry import FrontCamera
    return FrontCamera(800., 800., 400., 0., 0.,
                       [[0, -1, 0], [0, 0, -1], [1, 0, 0]], 60., 1600, 1200)


def test_lower_contact_not_elevated_body():
    mask = np.zeros((1200, 1600), np.uint8)
    mask[420:900, 700:900] = 1
    result = geometry().observe(mask, camera())
    assert result['contact_valid']
    assert result['contact_px'][1] == 899
    assert result['floor_contact_m'] == pytest.approx([.275 + 48/499, 0.00006, 0], abs=.001)
    assert result['grasp_valid'] and result['reachable_for_grasp']
    assert result['footprint_depth_m'] < .01  # upper body never flattened onto floor
    assert result['transport_suitability'] == 0
    assert result['suitability_reason'] == 'UNQUALIFIED'


@pytest.mark.parametrize('rows', [(0, 390), (1130, 1200)])
def test_horizon_and_bumper_rejected(rows):
    mask = np.zeros((1200, 1600), np.uint8)
    mask[rows[0]:rows[1], 700:900] = 1
    assert not geometry().observe(mask, camera())['contact_valid']


def test_clipped_bumper_mask_does_not_invent_a_new_edge():
    mask = np.zeros((1200, 1600), np.uint8)
    mask[800:1200, 700:900] = 1
    assert not geometry().observe(mask, camera())['contact_valid']


def test_uncertainty_increases_with_range():
    g = geometry(); cam = camera()
    assert g.position_uncertainty(cam, (800, 460)) - .005 > (g.position_uncertainty(cam, (800, 900)) - .005) * 10


def test_geofence_time_match_and_fail_closed():
    g = geometry()
    grid = NS(origin=(1., 2.), resolution=.1, safe_grid=np.zeros((10, 10), np.int8))
    assert g.geofence((.35, 0, 0), (1., 2.5, 0), grid, tf_age_s=.1, grid_age_s=1.) == (True, True)
    for kwargs in [dict(tf_age_s=1.), dict(grid_age_s=9.)]:
        assert g.geofence((.35, 0, 0), (1., 2.5, 0), grid, **kwargs) == (False, False)
    assert g.geofence((.35, 0, 0), None, grid) == (False, False)
    grid.safe_grid[5, 3] = -1
    assert g.geofence((.35, 0, 0), (1., 2.5, 0), grid) == (False, True)


def test_tracker_odometry_confirmation_misses_and_unique_association():
    g = geometry(); tracker = g.GarmentTracker(max_misses=2)
    def obs(x): return dict(contact_valid=True, floor_contact_m=[x, 0., 0.], category='garment')
    first = tracker.update([obs(.5), obs(.9)])
    ids = [x['track_id'] for x in first]
    assert len(set(ids)) == 2 and not first[0]['confirmed']
    second = tracker.update([obs(.4), obs(.8)], odom_delta=(.1, 0, 0))
    assert [x['track_id'] for x in second] == ids and all(x['confirmed'] for x in second)
    tracker.update([]); assert all(t['misses'] == 1 for t in tracker.tracks.values())
    tracker.update([]); tracker.update([]); assert not tracker.tracks


def test_tracker_rotation_and_out_of_gate():
    g = geometry()
    assert g.predict((1., 0., 0.), (0., 0., np.pi/2)) == pytest.approx([0., -1., 0.])
    t = g.GarmentTracker()
    a = t.update([dict(contact_valid=True, floor_contact_m=[.4, 0, 0])])[0]
    b = t.update([dict(contact_valid=True, floor_contact_m=[1., 0, 0])])[0]
    assert a['track_id'] != b['track_id']


def test_suitability_requires_explicit_qualification():
    g = geometry()
    assert g.suitability(.05, .05)[0] == 0
    assert g.suitability(.05, .05, qualified_small_lift=True)[0] == 1
    assert g.suitability(.5, .05, qualified_small_lift=True)[0] == 0
    assert g.suitability(.5, .05, qualified_small_lift=True, allow_tow=True)[0] == 2


def test_worker_framing_eof_invalid_image_and_contour_backend():
    from z70_manipulation import garment_worker as worker
    stream = io.BytesIO(struct.pack('<I', 3)+b'abc')
    assert worker.read_packet(stream) == b'abc'
    with pytest.raises(EOFError): worker.read_packet(stream)
    with pytest.raises(EOFError): worker.read_exact(io.BytesIO(b'x'), 2)
    with pytest.raises(ValueError): worker.read_packet(io.BytesIO(struct.pack('<I', 9*1024*1024)))
    image = np.full((120, 160, 3), 180, np.uint8)
    cv2.rectangle(image, (60, 70), (100, 100), (80, 40, 190), -1)
    model = worker.GarmentModel('contour_fallback')
    detections, quality = model.infer(image)
    assert not quality['model_available'] and quality['backend'] == 'contour_fallback'
    assert detections and detections[0]['mask'].shape == image.shape[:2]
    data = worker.encode_result(detections, quality)
    decoded, info = worker.decode_result(data)
    assert np.array_equal(decoded[0]['mask'], detections[0]['mask'])
    assert info == quality


def test_persistent_worker_subprocess_two_frames_and_clean_eof():
    script = ROOT/'src/z70_manipulation/z70_manipulation/garment_worker.py'
    image = np.full((120, 160, 3), 180, np.uint8)
    jpeg = cv2.imencode('.jpg', image)[1].tobytes()
    payload = struct.pack('<I', len(jpeg))+jpeg
    proc = subprocess.run([sys.executable, str(script), '--backend', 'contour_fallback'],
                          input=payload*2, capture_output=True, timeout=15)
    assert proc.returncode == 0, proc.stderr.decode()
    from z70_manipulation import garment_worker as w
    stream = io.BytesIO(proc.stdout)
    for _ in range(2):
        _, quality = w.decode_result(w.read_packet(stream))
        assert not quality['model_available']
    assert not stream.read()


def test_ros_garment_serialization_preserves_provenance_and_unknown_suitability():
    from rclpy.serialization import serialize_message, deserialize_message
    from z70_interfaces.msg import GarmentObservations, FrameMetadata
    from z70_manipulation.garment_node import observations_message
    import uuid
    meta = FrameMetadata(); meta.calibration_id = 'floor-camera-test'
    meta.evidence.valid = True; meta.evidence.sequence = 71
    meta.evidence.received_at.sec = 123; meta.evidence.session_id.uuid = list(uuid.uuid4().bytes)
    item = geometry().observe(np.ones((1200, 1600), np.uint8), camera())
    item['track_id'] = str(uuid.uuid4())
    msg = observations_message([item], meta, dict(model_available=True, model_id='clipseg'), False)
    msg = deserialize_message(serialize_message(msg), GarmentObservations)
    assert msg.header.frame_id == 'base_footprint'
    assert msg.evidence == meta.evidence and msg.garments[0].evidence == meta.evidence
    assert msg.garments[0].transport_suitability == 0
    assert msg.garments[0].track_id.uuid.tolist() == list(uuid.UUID(item['track_id']).bytes)
    assert msg.model_available and not msg.geofence_active


def test_worker_invalid_jpeg_reports_unavailable_not_a_detection():
    script = ROOT/'src/z70_manipulation/z70_manipulation/garment_worker.py'
    proc = subprocess.run([sys.executable, str(script), '--backend', 'contour_fallback'],
                          input=struct.pack('<I', 3)+b'bad', capture_output=True, timeout=15)
    from z70_manipulation import garment_worker as w
    detections, quality = w.decode_result(w.read_packet(io.BytesIO(proc.stdout)))
    assert not detections and not quality['model_available'] and quality['error'] == 'ValueError'


def test_box_only_has_no_pinchable_grasp_and_mask_size_is_checked():
    g = geometry()
    item = g.observe(None, camera(), bbox=[700, 700, 200, 200])
    assert item['contact_valid'] and not item['grasp_valid']
    assert not g.observe(None, camera(), bbox=[-1, 700, 200, 200])['contact_valid']
    with pytest.raises(ValueError): g.observe(np.ones((12,16)), camera())


def test_lower_edge_out_of_reach_is_not_clamped_into_envelope():
    mask = np.zeros((1200,1600), np.uint8); mask[420:500,700:900] = 1
    item = geometry().observe(mask, camera())
    assert item['contact_valid'] and item['floor_contact_m'][0] > .45
    assert not item['grasp_valid'] and not item['reachable_for_grasp']


def test_geofence_occupancy_grid_rotation_unknown_and_map_frame():
    from nav_msgs.msg import OccupancyGrid
    grid = OccupancyGrid(); grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 10; grid.info.resolution = .1
    grid.info.origin.orientation.w = 1.; grid.data = [0]*100
    g = geometry()
    assert g.geofence((.35,0,0), (0,.5,0), grid) == (True, True)
    grid.info.origin.orientation.z = .1
    assert g.geofence((.35,0,0), (0,.5,0), grid) == (False, False)
    grid.info.origin.orientation.z = 0.; grid.header.frame_id = 'odom'
    assert g.geofence((.35,0,0), (0,.5,0), grid) == (False, False)


def test_tracker_reconfirmation_after_gap_invalid_observations_do_not_track():
    g = geometry(); tracker = g.GarmentTracker()
    item = dict(contact_valid=True, floor_contact_m=[.4,0,0], category='garment')
    tracker.update([item]); tracker.update([])
    assert not tracker.update([item])[0]['confirmed']
    assert tracker.update([item])[0]['confirmed']
    result = tracker.update([dict(contact_valid=False)])
    assert result[0]['track_id'] == '00000000-0000-0000-0000-000000000000'


def test_latest_only_backpressure_replaces_pending_frame():
    import threading
    from z70_manipulation.garment_node import GarmentNode
    from sensor_msgs.msg import CompressedImage
    from z70_interfaces.msg import FrameMetadata
    meta = FrameMetadata(); meta.width = 1600; meta.height = 1200
    state = NS(frames=NS(metadata=meta), cam=camera(), lock=threading.Lock(), latest=None)
    a = CompressedImage(); a.header.frame_id = 'camera_optical_frame'; a.data = b'first'
    b = CompressedImage(); b.header.frame_id = 'camera_optical_frame'; b.data = b'latest'
    GarmentNode.on_frame(state, a); GarmentNode.on_frame(state, b)
    assert state.latest[0] is b
    meta.calibration_id = 'changed-after-submission'
    assert state.latest[1].calibration_id == ''  # immutable provenance snapshot


def test_node_publication_missing_tf_fails_closed_and_stale_output_clears():
    import threading
    import time
    import uuid
    from z70_manipulation.garment_node import GarmentNode
    from z70_interfaces.msg import FrameMetadata
    class Publisher:
        def __init__(self): self.messages = []
        def publish(self, msg): self.messages.append(msg)
    meta = FrameMetadata(); meta.calibration_id = 'floor-test'; meta.evidence.valid = True
    now = time.time(); meta.evidence.received_at.sec = int(now)
    meta.evidence.received_at.nanosec = int((now-int(now))*1e9)
    meta.evidence.session_id.uuid = list(uuid.uuid4().bytes)
    mask = np.zeros((1200,1600), np.uint8); mask[700:900,700:900] = 1
    detection = dict(mask=mask, category='garment', confidence=.9)
    statuses = []
    state = NS(lock=threading.Lock(), result=(meta,[detection],dict(model_available=True)), max_age=5.,
               pose_at=lambda stamp:(None,float('inf')), grid=None, cam=camera(), camera_x=.275,
               last_pose=None, last_session=None, tracker=geometry().GarmentTracker(),
               pub=Publisher(), markers=Publisher(), json_pub=Publisher(), set_status=statuses.append)
    GarmentNode.publish(state)
    msg = state.pub.messages[-1]
    assert not msg.geofence_active and len(msg.garments) == 1
    assert not msg.garments[0].geofence_verified and not msg.garments[0].reachable_for_grasp
    assert json.loads(state.json_pub.messages[-1].data)['garments'][0]['confirmed'] is False
    meta.evidence.received_at.sec -= 20
    state.result = (meta,[detection],dict(model_available=True))
    GarmentNode.publish(state)
    assert not state.pub.messages[-1].garments
    assert not state.pub.messages[-1].evidence.valid
    assert state.pub.messages[-1].evidence.stale
    assert len(state.markers.messages[-1].markers) == 1
    assert statuses[-1]['error'] == 'STALE_INFERENCE'


def test_worker_loaders_are_pinned_and_offline(monkeypatch):
    from z70_manipulation import garment_worker as w
    calls = []
    class LocalLoader:
        @classmethod
        def from_pretrained(cls, name, **kwargs):
            calls.append((name, kwargs)); return cls()
        def to(self, device): return self
        def eval(self): return self
    monkeypatch.setitem(sys.modules, 'torch', NS())
    monkeypatch.setitem(sys.modules, 'transformers', NS(AutoProcessor=LocalLoader,
                         AutoModelForZeroShotObjectDetection=LocalLoader, CLIPSegForImageSegmentation=LocalLoader))
    model = w.GarmentModel('grounding_dino_clipseg')
    assert model.error is None and len(calls) == 4
    for name, kwargs in calls:
        assert kwargs['local_files_only'] is True
        assert kwargs['revision'] == (w.DINO_REV if name == w.DINO_ID else w.CLIP_REV)


def test_real_calibration_synthetic_contact_roundtrip():
    from z70_description.floor_geometry import FrontCamera
    cam = FrontCamera.load(ROOT/'src/z70_description/config/front_camera.json')
    for camera_forward_m in (.125, .25, .6):
        u, v = map(round, cam.floor_to_pixel(camera_forward_m*1000, 0.))
        mask = np.zeros((1200,1600), np.uint8)
        mask[v-50:v+1,u-4:u+5] = 1
        item = geometry().observe(mask, cam)
        assert item['contact_valid']
        assert item['floor_contact_m'] == pytest.approx([.275+camera_forward_m,0,0], abs=.005)


def test_node_requires_two_time_matched_geofenced_observations_for_selection():
    import threading
    import time
    import uuid
    from z70_manipulation.garment_node import GarmentNode
    from z70_interfaces.msg import FrameMetadata
    class Publisher:
        def __init__(self): self.messages = []
        def publish(self, msg): self.messages.append(msg)
    meta = FrameMetadata(); meta.calibration_id = 'floor-test'; meta.evidence.valid = True
    now = time.time(); meta.evidence.received_at.sec = int(now)
    meta.evidence.session_id.uuid = list(uuid.uuid4().bytes)
    grid = NS(origin=(0.,-1.), resolution=.1, safe_grid=np.zeros((20,20),np.int8),
              header=NS(stamp=meta.evidence.received_at))
    mask = np.zeros((1200,1600), np.uint8); mask[700:900,700:900] = 1
    d = dict(mask=mask, category='garment', confidence=.9)
    state = NS(lock=threading.Lock(), result=None, max_age=5., pose_at=lambda stamp:((0.,0.,0.),0.),
               grid=grid, grid_received=time.monotonic(), cam=camera(), camera_x=.275,
               last_pose=None, last_session=None, tracker=geometry().GarmentTracker(),
               pub=Publisher(), markers=Publisher(), json_pub=Publisher(), set_status=lambda _:None)
    ids = []
    for expected in (False, True):
        state.result = (meta,[d],dict(model_available=True))
        GarmentNode.publish(state)
        msg = state.pub.messages[-1]
        assert msg.geofence_active and msg.garments[0].inside_geofence
        assert msg.garments[0].reachable_for_grasp is expected
        ids.append(bytes(msg.garments[0].track_id.uuid))
    assert ids[0] == ids[1]
    state.pose_at = lambda stamp:((0.,0.,0.),1.)
    state.result = (meta,[d],dict(model_available=True)); GarmentNode.publish(state)
    assert not state.pub.messages[-1].garments[0].geofence_verified
    assert not state.pub.messages[-1].garments[0].reachable_for_grasp
    state.pose_at = lambda stamp:((0.,0.,0.),0.)
    grid.resolution = -.1
    state.result = (meta,[d],dict(model_available=True)); GarmentNode.publish(state)
    assert not state.pub.messages[-1].geofence_active
