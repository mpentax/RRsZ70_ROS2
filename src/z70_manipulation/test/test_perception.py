import io
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as NS
import uuid
import cv2
from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CompressedImage
import yaml
from z70_description import pincher_geometry as hand
from z70_description.arm_model import config_path
from z70_interfaces.msg import FrameMetadata, GarmentObservations
from z70_manipulation import garment_worker as w, perception, perception as g, perception as p, planner, planner as placement
from z70_manipulation.garment_node import GarmentNode, observations_message
from z70_manipulation.node import Perception
from z70_manipulation.perception import FrontCamera

ROOT = Path(__file__).resolve().parents[3]
CAM = FrontCamera(800., 800., 400., 0., 0., [[0, -1, 0], [0, 0, -1], [1, 0, 0]], 60., 1600, 1200)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_coloured_blobs_reject_tiles_and_noise():
    image=np.full((1200,1600,3),210,np.uint8)
    for x in range(0,1600,200): cv2.line(image,(x,0),(x,1199),(100,100,100),3)
    for y in range(0,1200,200): cv2.line(image,(0,y),(1599,y),(100,100,100),3)
    cv2.circle(image,(780,900),35,(0,150,240),-1)
    cv2.circle(image,(500,950),30,(220,30,220),-1)
    image[100,100]=(0,170,255)
    blobs=p.detect(image)
    assert sorted(b['color'] for b in blobs)==['purple','yellow']
    yellow=next(b for b in blobs if b['color']=='yellow')
    assert yellow['contact_px']==pytest.approx([780,934],abs=3)
    assert yellow['size_px']>=65
    # Metric scene: 4-6 cm objects, independent of the detector-only scene above.
    image=np.full((1200,1600,3),210,np.uint8)
    cv2.circle(image,(780,690),90,(0,150,240),-1)   # ~6.5 cm, the real dodecahedron size
    cv2.circle(image,(500,720),40,(220,30,220),-1)
    objects=p.front_objects(image)
    assert len(objects)==2
    assert all(o['position'][0]>.275 and abs(o['position'][1])<1 for o in objects)

def test_ray_floor_intersection_and_horizon():
    K=np.array([[800.,0,800],[0,800,450],[0,0,1]])
    R=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    assert p.ray_floor((800,1250),K,np.zeros(5),[.4,.02,.2],R)==pytest.approx([.6,.02,0])
    assert p.ray_floor((800,450),K,np.zeros(5),[.4,.02,.2],R) is None

def test_front_contact_roundtrip():
    cam=p.front_camera()
    pixel=cam.floor_to_pixel(400,50)
    image=np.full((1200,1600,3),190,np.uint8)
    u,v=map(round,pixel)
    cv2.rectangle(image,(u-50,v-80),(u+50,v),(0,170,255),-1)
    obj=p.front_objects(image)[0]
    assert obj['position']==pytest.approx([.675,.05,0],abs=.005)

def test_front_rejects_unphysical_sized_regions_and_coloured_labels():
    image=np.full((1200,1600,3),190,np.uint8)
    cv2.rectangle(image,(1240,643),(1247,652),(255,20,20),-1)  # too small in metres
    cv2.rectangle(image,(463,464),(735,672),(0,100,240),-1)  # too large in metres
    cv2.rectangle(image,(561,593),(590,606),(0,170,255),-1)  # squat label, not dodecahedron
    cv2.rectangle(image,(300,528),(345,656),(0,100,240),-1)  # narrow tall region
    assert p.detect(image)
    assert p.front_objects(image)==[]

def test_yellow_facets_include_warm_shadow_hues_without_losing_contact():
    hsv=np.zeros((1200,1600,3),np.uint8);hsv[:,:,2]=190
    cv2.rectangle(hsv,(600,650),(800,850),(19,220,170),-1)
    cv2.rectangle(hsv,(680,650),(800,850),(13,240,150),-1)
    image=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    yellow=next(b for b in p.detect(image) if b['color']=='yellow')
    assert yellow['bbox']==[600,650,201,201]
    assert yellow['contact_px'][0]==pytest.approx(700,abs=2)


def test_purple_pale_facets_are_not_replaced_by_an_offset_saturated_core():
    hsv = np.zeros((1200, 1600, 3), np.uint8)
    hsv[:, :, 2] = 190
    cv2.rectangle(hsv, (600, 650), (800, 850), (145, 95, 170), -1)
    cv2.rectangle(hsv, (600, 650), (710, 780), (145, 220, 170), -1)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    purple = next(b for b in p.front_objects(image) if b['color'] == 'purple')
    assert purple['bbox'] == [600, 650, 201, 201]
    assert purple['contact_px'] == pytest.approx([700, 850], abs=2)


def test_contact_uses_bottom_pixels_not_sparse_contour_vertices():
    image=np.full((1200,1600,3),200,np.uint8)
    cv2.rectangle(image,(599,670),(789,820),(0,160,240),-1)
    jpeg=cv2.imencode('.jpg',image)[1]
    blob=p.detect(cv2.imdecode(jpeg,cv2.IMREAD_COLOR))[0]
    assert blob['contact_px'][0]==pytest.approx(693,abs=3)


def test_default_front_works_without_checkout_or_cwd(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path); monkeypatch.setenv('Z70_REPO', '/missing/archive')
    p.front_camera.cache_clear()
    cam = p.front_camera()
    assert (cam.width, cam.height) == (1600, 1200)
    pixel = cam.floor_to_pixel(400., 50.)
    point = cam.pixel_to_floor(*pixel)
    assert [point['forward_mm'], point['left_mm']] == pytest.approx([400., 50.], abs=1e-7)
    assert p.calibration()['front_camera_x_m'] > 0


def test_fallback_lenses_match_active_resources():
    lens = p.gripper_camera_info()
    assert np.array_equal(planner.GRIPPER_MATRIX, np.array(lens['camera_matrix']['data']).reshape(3, 3))
    assert np.array_equal(planner.GRIPPER_D, np.array(lens['distortion_coefficients']['data']))
    front = json.loads(config_path('front_camera.json').read_text())
    info = yaml.safe_load(config_path('z70_front_camera_info.yaml').read_text())
    expected = [front['f_px'], 0., front['cx'], 0., front['f_px'], front['cy'], 0., 0., 1.]
    assert info['camera_matrix']['data'] == [round(x, 3) for x in expected]
    assert info['distortion_coefficients']['data'] == [round(front['k1'], 6), round(front['k2'], 6), 0., 0., 0.]


def test_node_before_camera_info_uses_shared_pincher_lens():
    rclpy.init(domain_id=91)
    node = None
    try:
        node = Perception(name='pincher_lens_fallback_test', subscribe=False)
        assert node.K == pytest.approx(planner.GRIPPER_MATRIX, abs=1e-12)
        assert node.D == pytest.approx(planner.GRIPPER_D, abs=1e-12)
        assert not np.shares_memory(node.K, planner.GRIPPER_MATRIX)
        assert not np.shares_memory(node.D, planner.GRIPPER_D)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def test_front_jaw_uses_elevated_68_not_floor_duplicate():
    camera = p.front_camera(); K = np.array([[camera.f, 0, camera.cx], [0, camera.f, camera.cy], [0, 0, 1.]])
    R = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0.]])
    pos = np.array([.16, .02, .10]); C = np.array([0, 0, camera.h / 1000])
    obj = np.array([[-.0075, .0075, 0], [.0075, .0075, 0], [.0075, -.0075, 0], [-.0075, -.0075, 0]])
    r = cv2.Rodrigues(camera.R @ R)[0]; t = camera.R @ (pos - C)
    px = cv2.projectPoints(obj, r, t, K, np.array([camera.k1, camera.k2, 0, 0, 0]))[0].reshape(4, 2)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000), 68, 200)
    image = np.full((1200, 1600), 255, np.uint8)
    H = cv2.getPerspectiveTransform(np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32), px.astype(np.float32))
    warped = cv2.warpPerspective(marker, H, (1600, 1200), borderValue=255); image = np.minimum(image, warped)
    floor_px = np.array([camera.floor_to_pixel(x * 1000, y * 1000) for x, y in
                         [(.21, .075), (.21, .060), (.195, .060), (.195, .075)]], np.float32)
    Hfloor = cv2.getPerspectiveTransform(np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32), floor_px)
    image = np.minimum(image, cv2.warpPerspective(marker, Hfloor, (1600, 1200), borderValue=255))
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000))
    _, ids, _ = detector.detectMarkers(image)
    assert list(ids.flatten()).count(68) == 2
    out = p.front_jaw(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
    hand = planner.hand
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = pos + [.275, 0, 0]
    T = hand.marker_hand_pose(M)
    aim = T[:3, :3] @ hand.grasp_reference() + T[:3, 3]
    assert out['position'] == pytest.approx(aim, abs=.004)
    observed = np.array(out['marker_pose_base'])
    assert observed[:3, 3] == pytest.approx(M[:3, 3], abs=.004)
    angle = np.arccos(np.clip((np.trace(observed[:3, :3] @ R.T) - 1) / 2, -1, 1))
    assert angle < np.radians(1.)
    assert out['marker_id'] == 68 and out['marker_position'][2] > .025


def test_lower_contact_not_elevated_body():
    mask = np.zeros((1200, 1600), np.uint8)
    mask[420:900, 700:900] = 1
    result = g.observe(mask, CAM)
    assert result['contact_valid']
    assert result['contact_px'][1] == 899
    assert result['floor_contact_m'] == pytest.approx([.275 + 48 / 499, 0.00006, 0], abs=.001)
    assert result['grasp_valid'] and result['reachable_for_grasp']
    assert result['footprint_depth_m'] < .01
    assert result['transport_suitability'] == 0
    assert result['suitability_reason'] == 'UNQUALIFIED'


@pytest.mark.parametrize('rows', [(0, 390), (1130, 1200)])
def test_horizon_and_bumper_rejected(rows):
    mask = np.zeros((1200, 1600), np.uint8)
    mask[rows[0]:rows[1], 700:900] = 1
    assert not g.observe(mask, CAM)['contact_valid']


def test_clipped_bumper_mask_does_not_invent_a_new_edge():
    mask = np.zeros((1200, 1600), np.uint8)
    mask[800:1200, 700:900] = 1
    assert not g.observe(mask, CAM)['contact_valid']


def test_uncertainty_increases_with_range():
    assert g.position_uncertainty(CAM, (800, 460)) - .005 > (g.position_uncertainty(CAM, (800, 900)) - .005) * 10


def test_geofence_time_match_and_fail_closed():
    grid = NS(origin=(1., 2.), resolution=.1, safe_grid=np.zeros((10, 10), np.int8))
    assert g.geofence((.35, 0, 0), (1., 2.5, 0), grid, tf_age_s=.1, grid_age_s=1.) == (True, True)
    for kwargs in [dict(tf_age_s=1.), dict(grid_age_s=9.)]:
        assert g.geofence((.35, 0, 0), (1., 2.5, 0), grid, **kwargs) == (False, False)
    assert g.geofence((.35, 0, 0), None, grid) == (False, False)
    grid.safe_grid[5, 3] = -1
    assert g.geofence((.35, 0, 0), (1., 2.5, 0), grid) == (False, True)


def test_tracker_odometry_confirmation_misses_and_unique_association():
    tracker = g.GarmentTracker(max_misses=2)
    def obs(x): return dict(contact_valid=True, floor_contact_m=[x, 0., 0.], category='garment')
    first = tracker.update([obs(.5), obs(.9)])
    ids = [x['track_id'] for x in first]
    assert len(set(ids)) == 2 and not first[0]['confirmed']
    second = tracker.update([obs(.4), obs(.8)], odom_delta=(.1, 0, 0))
    assert [x['track_id'] for x in second] == ids and all(x['confirmed'] for x in second)
    tracker.update([])
    assert all(t['misses'] == 1 for t in tracker.tracks.values())
    tracker.update([]); tracker.update([])
    assert not tracker.tracks


def test_tracker_rotation_and_out_of_gate():
    assert g.predict((1., 0., 0.), (0., 0., np.pi / 2)) == pytest.approx([0., -1., 0.])
    t = g.GarmentTracker()
    a = t.update([dict(contact_valid=True, floor_contact_m=[.4, 0, 0])])[0]
    b = t.update([dict(contact_valid=True, floor_contact_m=[1., 0, 0])])[0]
    assert a['track_id'] != b['track_id']


def test_suitability_requires_explicit_qualification():
    assert g.suitability(.05, .05)[0] == 0
    assert g.suitability(.05, .05, qualified_small_lift=True)[0] == 1
    assert g.suitability(.5, .05, qualified_small_lift=True)[0] == 0
    assert g.suitability(.5, .05, qualified_small_lift=True, allow_tow=True)[0] == 2


def test_worker_framing_eof_invalid_image_and_contour_backend():
    stream = io.BytesIO(struct.pack('<I', 3) + b'abc')
    assert w.read_packet(stream) == b'abc'
    with pytest.raises(EOFError): w.read_packet(stream)
    with pytest.raises(EOFError): w.read_exact(io.BytesIO(b'x'), 2)
    with pytest.raises(ValueError): w.read_packet(io.BytesIO(struct.pack('<I', 9 * 1024 * 1024)))
    image = np.full((120, 160, 3), 180, np.uint8)
    cv2.rectangle(image, (60, 70), (100, 100), (80, 40, 190), -1)
    model = w.GarmentModel('contour_fallback')
    detections, quality = model.infer(image)
    assert not quality['model_available'] and quality['backend'] == 'contour_fallback'
    assert detections and detections[0]['mask'].shape == image.shape[:2]
    data = w.encode_result(detections, quality)
    decoded, info = w.decode_result(data)
    assert np.array_equal(decoded[0]['mask'], detections[0]['mask'])
    assert info == quality


def test_persistent_worker_subprocess_two_frames_and_clean_eof():
    script = ROOT / 'src/z70_manipulation/z70_manipulation/garment_worker.py'
    image = np.full((120, 160, 3), 180, np.uint8)
    jpeg = cv2.imencode('.jpg', image)[1].tobytes()
    payload = struct.pack('<I', len(jpeg)) + jpeg
    proc = subprocess.run([sys.executable, str(script), '--backend', 'contour_fallback'],
                          input=payload * 2, capture_output=True, timeout=15)
    assert proc.returncode == 0, proc.stderr.decode()
    stream = io.BytesIO(proc.stdout)
    for _ in range(2):
        _, quality = w.decode_result(w.read_packet(stream))
        assert not quality['model_available']
    assert not stream.read()


def test_ros_garment_serialization_preserves_provenance_and_unknown_suitability():
    meta = FrameMetadata(); meta.calibration_id = 'floor-camera-test'
    meta.evidence.valid = True; meta.evidence.sequence = 71
    meta.evidence.received_at.sec = 123; meta.evidence.session_id.uuid = list(uuid.uuid4().bytes)
    item = g.observe(np.ones((1200, 1600), np.uint8), CAM)
    item['track_id'] = str(uuid.uuid4())
    msg = observations_message([item], meta, dict(model_available=True, model_id='clipseg'), False)
    msg = deserialize_message(serialize_message(msg), GarmentObservations)
    assert msg.header.frame_id == 'base_footprint'
    assert msg.evidence == meta.evidence and msg.garments[0].evidence == meta.evidence
    assert msg.garments[0].transport_suitability == 0
    assert msg.garments[0].track_id.uuid.tolist() == list(uuid.UUID(item['track_id']).bytes)
    assert msg.model_available and not msg.geofence_active


def test_worker_invalid_jpeg_reports_unavailable_not_a_detection():
    script = ROOT / 'src/z70_manipulation/z70_manipulation/garment_worker.py'
    proc = subprocess.run([sys.executable, str(script), '--backend', 'contour_fallback'],
                          input=struct.pack('<I', 3) + b'bad', capture_output=True, timeout=15)
    detections, quality = w.decode_result(w.read_packet(io.BytesIO(proc.stdout)))
    assert not detections and not quality['model_available'] and quality['error'] == 'ValueError'


def test_box_only_has_no_pinchable_grasp_and_mask_size_is_checked():
    item = g.observe(None, CAM, bbox=[700, 700, 200, 200])
    assert item['contact_valid'] and not item['grasp_valid']
    assert not g.observe(None, CAM, bbox=[-1, 700, 200, 200])['contact_valid']
    with pytest.raises(ValueError): g.observe(np.ones((12, 16)), CAM)


def test_lower_edge_out_of_reach_is_not_clamped_into_envelope():
    mask = np.zeros((1200, 1600), np.uint8); mask[420:500, 700:900] = 1
    item = g.observe(mask, CAM)
    assert item['contact_valid'] and item['floor_contact_m'][0] > .45
    assert not item['grasp_valid'] and not item['reachable_for_grasp']


def test_geofence_occupancy_grid_rotation_unknown_and_map_frame():
    grid = OccupancyGrid(); grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 10; grid.info.resolution = .1
    grid.info.origin.orientation.w = 1.; grid.data = [0] * 100
    assert g.geofence((.35, 0, 0), (0, .5, 0), grid) == (True, True)
    grid.info.origin.orientation.z = .1
    assert g.geofence((.35, 0, 0), (0, .5, 0), grid) == (False, False)
    grid.info.origin.orientation.z = 0.; grid.header.frame_id = 'odom'
    assert g.geofence((.35, 0, 0), (0, .5, 0), grid) == (False, False)


def test_tracker_reconfirmation_after_gap_invalid_observations_do_not_track():
    tracker = g.GarmentTracker()
    item = dict(contact_valid=True, floor_contact_m=[.4, 0, 0], category='garment')
    tracker.update([item]); tracker.update([])
    assert not tracker.update([item])[0]['confirmed']
    assert tracker.update([item])[0]['confirmed']
    result = tracker.update([dict(contact_valid=False)])
    assert result[0]['track_id'] == '00000000-0000-0000-0000-000000000000'


def test_latest_only_backpressure_replaces_pending_frame():
    meta = FrameMetadata(); meta.width = 1600; meta.height = 1200
    state = NS(frames=NS(metadata=meta), cam=CAM, lock=threading.Lock(), latest=None)
    a = CompressedImage(); a.header.frame_id = 'camera_optical_frame'; a.data = b'first'
    b = CompressedImage(); b.header.frame_id = 'camera_optical_frame'; b.data = b'latest'
    GarmentNode.on_frame(state, a); GarmentNode.on_frame(state, b)
    assert state.latest[0] is b
    meta.calibration_id = 'changed-after-submission'
    assert state.latest[1].calibration_id == ''


def test_node_publication_missing_tf_fails_closed_and_stale_output_clears():
    meta = FrameMetadata(); meta.calibration_id = 'floor-test'; meta.evidence.valid = True
    now = time.time(); meta.evidence.received_at.sec = int(now)
    meta.evidence.received_at.nanosec = int((now - int(now)) * 1e9)
    meta.evidence.session_id.uuid = list(uuid.uuid4().bytes)
    mask = np.zeros((1200, 1600), np.uint8); mask[700:900, 700:900] = 1
    detection = dict(mask=mask, category='garment', confidence=.9)
    statuses = []
    state = NS(lock=threading.Lock(), result=(meta, [detection], dict(model_available=True)), max_age=5.,
               pose_at=lambda stamp: (None, float('inf')), grid=None, cam=CAM, camera_x=.275,
               last_pose=None, last_session=None, tracker=g.GarmentTracker(),
               pub=Publisher(), markers=Publisher(), json_pub=Publisher(), set_status=statuses.append)
    GarmentNode.publish(state)
    msg = state.pub.messages[-1]
    assert not msg.geofence_active and len(msg.garments) == 1
    assert not msg.garments[0].geofence_verified and not msg.garments[0].reachable_for_grasp
    assert json.loads(state.json_pub.messages[-1].data)['garments'][0]['confirmed'] is False
    meta.evidence.received_at.sec -= 20
    state.result = (meta, [detection], dict(model_available=True))
    GarmentNode.publish(state)
    assert not state.pub.messages[-1].garments
    assert not state.pub.messages[-1].evidence.valid
    assert state.pub.messages[-1].evidence.stale
    assert len(state.markers.messages[-1].markers) == 1
    assert statuses[-1]['error'] == 'STALE_INFERENCE'


def test_worker_loaders_are_pinned_and_offline(monkeypatch):
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
    cam = FrontCamera.load(ROOT / 'src/z70_description/config/front_camera.json')
    for camera_forward_m in (.125, .25, .6):
        u, v = map(round, cam.floor_to_pixel(camera_forward_m * 1000, 0.))
        mask = np.zeros((1200, 1600), np.uint8)
        mask[v - 50:v + 1, u - 4:u + 5] = 1
        item = g.observe(mask, cam)
        assert item['contact_valid']
        assert item['floor_contact_m'] == pytest.approx([.275 + camera_forward_m, 0, 0], abs=.005)


def test_node_requires_two_time_matched_geofenced_observations_for_selection():
    meta = FrameMetadata(); meta.calibration_id = 'floor-test'; meta.evidence.valid = True
    now = time.time(); meta.evidence.received_at.sec = int(now)
    meta.evidence.session_id.uuid = list(uuid.uuid4().bytes)
    grid = NS(origin=(0., -1.), resolution=.1, safe_grid=np.zeros((20, 20), np.int8),
              header=NS(stamp=meta.evidence.received_at))
    mask = np.zeros((1200, 1600), np.uint8); mask[700:900, 700:900] = 1
    d = dict(mask=mask, category='garment', confidence=.9)
    state = NS(lock=threading.Lock(), result=None, max_age=5., pose_at=lambda stamp: ((0., 0., 0.), 0.),
               grid=grid, grid_received=time.monotonic(), cam=CAM, camera_x=.275,
               last_pose=None, last_session=None, tracker=g.GarmentTracker(),
               pub=Publisher(), markers=Publisher(), json_pub=Publisher(), set_status=lambda _: None)
    ids = []
    for expected in (False, True):
        state.result = (meta, [d], dict(model_available=True))
        GarmentNode.publish(state)
        msg = state.pub.messages[-1]
        assert msg.geofence_active and msg.garments[0].inside_geofence
        assert msg.garments[0].reachable_for_grasp is expected
        ids.append(bytes(msg.garments[0].track_id.uuid))
    assert ids[0] == ids[1]
    state.pose_at = lambda stamp: ((0., 0., 0.), 1.)
    state.result = (meta, [d], dict(model_available=True)); GarmentNode.publish(state)
    assert not state.pub.messages[-1].garments[0].geofence_verified
    assert not state.pub.messages[-1].garments[0].reachable_for_grasp
    state.pose_at = lambda stamp: ((0., 0., 0.), 0.)
    grid.resolution = -.1
    state.result = (meta, [d], dict(model_available=True)); GarmentNode.publish(state)
    assert not state.pub.messages[-1].geofence_active


def test_firmware_endpoint_is_separate_from_cad_hand_and_camera():
    a1, a2 = map(math.radians, (120, -15))
    endpoint = (.0475 + .1775*math.cos(a1) + .205*math.cos(a2),
                .14 + .1775*math.sin(a1) + .205*math.sin(a2))
    assert planner.vendor_endpoint((150, 45)) == pytest.approx(endpoint)
    T = hand.hand_pose((150, 45))
    assert not np.allclose(T[[0, 2], 3], endpoint)
    C, R = planner.camera_pose((150, 45))
    optical = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    assert C == pytest.approx(T[:3, 3] + T[:3, :3] @ planner.CAD_MOUNT[0])
    assert R == pytest.approx(T[:3, :3] @ hand.rpy(planner.CAD_MOUNT[1]) @ optical)
    assert planner.vendor_check(240, 130, 0) == 'FAULT_PROJECTION'
    assert not planner.safe((240, 130, 0, 90))


def test_default_grid_grasp_targets_front_measured_region():
    assert planner.GRASP == (250, 95)
    assert planner.tip_height(planner.GRASP) == pytest.approx(.04158389094665366, abs=1e-10)
    assert planner.grasp_point()[:2] == pytest.approx(planner.operational_point((240, 95))[:2])
    assert planner.vendor_check(*planner.GRASP, 0) == 'SUCCESS'
    assert planner.body_floor_bound(planner.GRASP) < planner.MIN_TIP
    assert not planner.safe(planner.GRASP)


def test_pincher_camera_follows_measured_M5_rotation():
    C0, R0 = planner.camera_pose((240, 105, 0))
    C40, R40 = planner.camera_pose((240, 105, 40))
    angle = math.degrees(math.acos(np.clip((np.trace(R40@R0.T)-1)/2, -1, 1)))
    assert angle == pytest.approx(40., abs=1e-6)
    assert np.linalg.norm(C40-C0) == pytest.approx(.009050553550760482, abs=1e-6)
    assert planner.camera_pose((150, 45, 0))[0][2] == pytest.approx(.2337875037399874, abs=1e-9)
    axis = np.array([math.cos(math.radians(-45)), 0, math.sin(math.radians(-45))])
    cross = np.cross(R0[:, 0], R40[:, 0])
    assert np.dot(cross, axis) > 0


def test_metric_closed_jaw_height_uses_owner_registered_closing_point():
    cfg = planner._CALIBRATION
    expected = np.array(cfg['marker_effective_from_elbow_m'])+np.array(cfg['marker_rotation_link2'])@cfg['jaw_region_in_marker_m']
    assert cfg['effective_jaw_from_elbow_m'] == pytest.approx(expected, abs=1e-12)
    assert planner._CALIBRATION['jaw_z_correction_m'] == 0.
    assert planner.tip_height((250, 115)) == pytest.approx(.07953791717046343, abs=1e-10)
    assert planner.tip_height((240, 105)) == pytest.approx(.10802715944384225, abs=1e-10)
    assert planner.tip_height((230, 95)) == pytest.approx(.13401118896525022, abs=1e-10)


def test_approach_places_target_on_arm_plane():
    grasp = planner.grasp_point(); plan = planner.approach([.9, .20, 0], grasp)
    th = plan['yaw']; d = plan['forward']
    x = math.cos(th)*.9+math.sin(th)*.20-d
    y = -math.sin(th)*.9+math.cos(th)*.20
    assert [x, y] == pytest.approx(grasp[:2], abs=1e-8)
    assert .25 <= planner.vendor_endpoint(planner.GRASP)[0] <= planner.REACH_LIMIT-.003
    aligned = planner.approach(grasp, grasp)
    assert abs(aligned['yaw']) < 1e-9 and abs(aligned['forward']) < 1e-9


def test_bfs_safe_grid_and_negative_code_two():
    goal = tuple(planner.grasp_goal(planner.grasp_point(), (190, 65))['state'])
    plan = planner.joint_plan((150, 45), goal)
    state = (150, 45)
    assert plan
    for step in plan:
        assert set(step['m']) <= {0, 1, 2}
        assert 1 <= sum(code != 0 for code in step['m']) <= 2
        for a, b in zip(state, step['state']): assert abs(a-b) in (0, 10)
        assert planner.safe_edge(state, tuple(step['state']))
        state = tuple(step['state'])
    assert any(sum(code != 0 for code in step['m']) == 2 for step in plan)
    assert state == goal
    reverse = planner.joint_plan(goal, (150, 45))
    assert any(2 in step['m'] for step in reverse)
    assert not planner.safe((270, 145))
    assert not planner.safe((240, 35))
    with pytest.raises(ValueError): planner.joint_plan((150, 45), (270, 145))


def test_joint_path_floor_envelope_rejects_low_shortcut():
    assert planner.tip_floor_bound((260, 95)) < planner.MIN_TIP
    assert not planner.safe_edge((250, 95), (260, 95))
    assert planner.tip_floor_bound(planner.GRASP) >= planner.MIN_TIP
    assert planner.body_floor_bound(planner.GRASP) < planner.MIN_TIP
    goal = tuple(planner.grasp_goal(planner.grasp_point(), (190, 65))['state'])
    path = planner.joint_plan((230, 95), goal)
    assert path and all(planner.safe_edge(a, b) for a, b in zip([(230, 95)]+[tuple(s['state']) for s in path], [tuple(s['state']) for s in path]))


def test_stack_tier_plans_cover_three_dodecahedron_tiers():
    tiers = [planner.stack_tier_plan(t) for t in (1, 2, 3)]
    assert [t['tier'] for t in tiers] == [1, 2, 3]
    assert [t['target_equator_z_m'] for t in tiers] == [0.033, 0.099, 0.165]
    assert [t['tower_top_below_z_m'] for t in tiers] == [0.0, 0.066, 0.132]
    assert [t['lift_steps'] for t in tiers] == [2, 2, 3]
    for t in tiers:
        assert planner.safe(tuple(t['placement_state']))
        assert abs(t['operational_point_m'][2] - t['target_equator_z_m']) <= 0.020
    with pytest.raises(ValueError):
        planner.stack_tier_plan(4)


def test_floor_safety_checks_the_whole_open_hand_and_roll():
    candidates = [(a, b, roll, 90) for a in range(180, 271, 5) for b in range(35, 171, 5) for roll in (0, 40, 90)]
    examples = [s for s in candidates if planner.tip_floor_bound(s) >= planner.MIN_TIP and
                hand.lowest(hand.world_bodies(s)) - hand.CONFIG['uncertainty_m'] < planner.MIN_TIP]
    assert examples
    assert not any(planner.safe(s) for s in examples)
    with pytest.raises(ValueError):
        hand.world_bodies([250, 95, float('nan'), 90])
    with pytest.raises(ValueError):
        hand.world_bodies([250, 95, 0, 91])


def test_new_anatomy_routes_to_operational_equator_not_unsafe_historical_preset():
    assert planner.GRASP == (250, 95)
    plan = planner.grasp_goal([.43, 0, 0], (190, 65))
    state = tuple(plan['state'])
    assert planner.safe(state)
    assert abs(planner.operational_point(state)[2] - .033) <= .012
    assert plan['steps']


def test_operational_query_exposes_frames_regions_and_opening_not_legacy_tcp():
    gaps = []
    for m6 in (0, 45, 90):
        detail = hand.articulated_geometry((240, 95, 40, m6))
        gaps.append(detail['terminal_gap_m'])
        assert set(detail['finger_frames']) == {'finger_left', 'finger_right'}
        assert {b.name for b in detail['pad_regions']} == {'finger_left', 'finger_right'}
        assert detail['finger_frames']['finger_left'] == pytest.approx(hand.hand_pose((240, 95, 40, m6)) @ hand.finger_frame(1, m6))
    assert gaps[0] > .01 and gaps[0] < gaps[1] < gaps[2]
    body = hand.world_bodies((190, 65))[0]
    assert not planner.safe((190, 65), [(body.center - .001, body.center + .001)])
    assert not planner.safe_edge((190, 65), (190, 75), [(body.center - .001, body.center + .001)])
    assert planner.safe((190, 65), [([8, 8, 8], [9, 9, 9])])
    for invalid in (float('nan'), -1, 91):
        with pytest.raises(ValueError):
            hand.local_bodies(invalid)
    with pytest.raises(ValueError, match='M3/M4'):
        planner.joint_plan((190, 65, 0, 90), (190, 65, 0, 45))


def test_four_angle_edges_include_m6_and_reject_mismatched_states():
    assert planner.safe_edge((190, 65, 0, 90), (190, 65, 40, 45))
    with pytest.raises(ValueError):
        planner.safe_edge((190, 65), (190, 65, 0, 90))


def _placement_scene(monkeypatch, gap=.020, elevated=False):
    image = np.zeros((400, 600, 3), np.uint8)
    class Camera:
        width, height, h = 600, 400, 350
        def pixel_to_ray(self, u, v):
            return np.array([1., (300 - u) * .002, -v * .002])
    monkeypatch.setattr(perception, 'front_camera', lambda: Camera())
    monkeypatch.setattr(perception, 'calibration', lambda: {'front_camera_x_m': 0.})
    support_top = .132 if elevated else .066
    ytop = round(350 - support_top * 1000)
    cv2.rectangle(image, (275, 284), (325, 350), (0, 160, 240), -1)
    if elevated:
        cv2.rectangle(image, (275, ytop), (325, 283), (200, 0, 200), -1)
    bottom = round(350 - (support_top + gap) * 1000)
    cv2.rectangle(image, (275, bottom - 60), (325, bottom), (0, 200, 0), -1)
    monkeypatch.setattr(perception, 'front_objects', lambda _: [dict(color='yellow', position=[.5, 0., 0.])])
    T = np.eye(4); T[:3, 3] = [.5, 0., support_top + gap + .03]
    jaw = dict(position=T[:3, 3].tolist(), hand_pose_base=T.tolist(), uncertainty_bound_m=.015)
    monkeypatch.setattr(perception, 'front_jaw', lambda _: jaw)
    return image


def test_measures_held_bottom_to_elevated_support_without_floor_projection(monkeypatch):
    image = _placement_scene(monkeypatch, elevated=True)
    sample = placement.measure(image, 'green', 'purple', 'yellow', [.5, 0., 0.], [0., 0., 0.])
    assert sample['gap_m'] == pytest.approx(.020, abs=.001)
    assert sample['held_bottom_m'][2] == pytest.approx(.152, abs=.001)
    assert sample['support_top_m'][2] == pytest.approx(.132, abs=.001)
    assert sample['lateral_error_m'] == pytest.approx([0., 0.], abs=.001)
    assert sample['uncertainty_m'] >= .015


def test_placement_keeps_pale_held_facets_in_contact_geometry(monkeypatch):
    image = _placement_scene(monkeypatch)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv[204:265, 275:326] = (60, 95, 170)
    hsv[204:235, 275:326] = (60, 220, 170)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    sample = placement.measure(image, 'green', 'yellow', 'yellow', [.5, 0., 0.], [0., 0., 0.])
    assert sample['gap_m'] == pytest.approx(.020, abs=.001)


def test_held_depth_uses_retained_object_transform_not_jaw_origin(monkeypatch):
    image = _placement_scene(monkeypatch)
    baseline = placement.measure(image, 'green', 'yellow', 'yellow', [.5, 0., 0.], [0., 0., 0.])
    shifted = placement.measure(image, 'green', 'yellow', 'yellow', [.5, 0., 0.], [.010, 0., 0.])
    assert shifted['held_center_m'][0] - baseline['held_center_m'][0] == pytest.approx(.010)


def test_rejects_same_hue_background_instead_of_assuming_object_depth(monkeypatch):
    image = _placement_scene(monkeypatch)
    cv2.rectangle(image, (220, 180), (380, 283), (0, 200, 0), -1)
    with pytest.raises(LookupError, match='contour'):
        placement.measure(image, 'green', 'yellow', 'yellow', [.5, 0., 0.], [0., 0., 0.])


def test_observation_recovers_local_pose_not_stale_pregrip_angles():
    current = [210., 75., 10.]
    observed = hand.hand_pose(current)
    recovered = placement.observed_joints(observed)
    assert recovered['joints'] == pytest.approx(current)
    bad = observed.copy(); bad[1, 3] += .05
    assert placement.observed_joints(bad) is None


def test_descent_never_substitutes_40mm_for_20mm_gap():
    sample = dict(gap_m=.020, lateral_error_m=[0., 0.])
    assert placement.descent_step(sample, None) is None
    sample['gap_m'] = .060
    assert placement.descent_step(sample, None) == dict(method='app_arm_direction_move', params={'direction': 2}, descent_m=.04)
    sample['gap_m'] = .030
    step = placement.descent_step(sample, [210., 75., 0.])
    assert step is not None and step['method'] == 'app_arm_move'
    assert .002 < step['descent_m'] <= .030
    assert set(step['params']['m']) <= {0, 1, 2} and step['params']['m'][2] == 0 and any(step['params']['m'][:2])
    sample['gap_m'] = .025
    coord_step = placement.descent_step(sample, [210., 75., 0.])
    assert coord_step is not None and coord_step['params']['m'] == [1, 1, 0]
    sample['gap_m'] = .022
    assert placement.descent_step(sample, [210., 75., 0.]) is None
    sample['gap_m'] = .050
    sample['uncertainty_m'] = .015
    assert placement.descent_step(sample, None) is None
