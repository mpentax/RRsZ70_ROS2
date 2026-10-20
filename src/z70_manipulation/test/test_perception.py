import json
import cv2
import numpy as np
import pytest
import rclpy
import yaml
from z70_description.resources import config_path
from z70_manipulation import perception as p, planner
from z70_manipulation.node import Perception


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
