import cv2
import numpy as np
import pytest
import requests
from z70_manipulation import perception, planner
from z70_manipulation.executor import Executor


class FakeClient:
    def __init__(self, status=None, arm=None, fill=0, fail_first=False):
        self._status = status or {}
        self._arm = arm or {'state': 'IN_PROCESS_POS', 'joints': {'m3': 240, 'm4': 105}}
        self.fill = fill
        self.fail_first = fail_first
        self.calls = 0

    def status(self):
        return self._status

    def arm_estimate(self):
        return self._arm

    def rpc(self, *a):
        return {'dispatch': 'sent', 'response': 'received'}

    def frame(self):
        self.calls += 1
        if self.fail_first and self.calls == 1:
            raise requests.RequestException('camera gap')
        return np.full((1200, 1600, 3), self.fill, np.uint8), 'front'


def test_front_jaw_uses_elevated_68_not_floor_duplicate():
    # Render the real registered marker in a synthetic calibrated frame.
    camera=perception.front_camera();K=np.array([[camera.f,0,camera.cx],[0,camera.f,camera.cy],[0,0,1.]])
    R=np.array([[0,0,-1],[1,0,0],[0,-1,0.]])
    pos=np.array([.16,.02,.10]);C=np.array([0,0,camera.h/1000])
    obj=np.array([[-.0075,.0075,0],[.0075,.0075,0],[.0075,-.0075,0],[-.0075,-.0075,0]])
    r=cv2.Rodrigues(camera.R@R)[0];t=camera.R@(pos-C)
    px=cv2.projectPoints(obj,r,t,K,np.array([camera.k1,camera.k2,0,0,0]))[0].reshape(4,2)
    marker=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),68,200)
    image=np.full((1200,1600),255,np.uint8)
    H=cv2.getPerspectiveTransform(np.array([[0,0],[199,0],[199,199],[0,199]],np.float32),px.astype(np.float32))
    warped=cv2.warpPerspective(marker,H,(1600,1200),borderValue=255);image=np.minimum(image,warped)
    # The dense floor board also contains ID68: it must not become the wrist.
    floor_px=np.array([camera.floor_to_pixel(x*1000,y*1000) for x,y in
                       [(.21,.075),(.21,.060),(.195,.060),(.195,.075)]],np.float32)
    Hfloor=cv2.getPerspectiveTransform(np.array([[0,0],[199,0],[199,199],[0,199]],np.float32),floor_px)
    image=np.minimum(image,cv2.warpPerspective(marker,Hfloor,(1600,1200),borderValue=255))
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000))
    _,ids,_=detector.detectMarkers(image)
    assert list(ids.flatten()).count(68)==2
    out=perception.front_jaw(cv2.cvtColor(image,cv2.COLOR_GRAY2BGR))
    hand=planner.hand
    M=np.eye(4);M[:3,:3]=R;M[:3,3]=pos+[.275,0,0]
    T=hand.marker_hand_pose(M)
    aim=T[:3,:3]@hand.grasp_reference()+T[:3,3]
    assert out['position']==pytest.approx(aim,abs=.004)
    observed=np.array(out['marker_pose_base'])
    assert observed[:3,3]==pytest.approx(M[:3,3],abs=.004)
    angle=np.arccos(np.clip((np.trace(observed[:3,:3]@R.T)-1)/2,-1,1))
    assert angle<np.radians(1.)
    assert out['marker_id']==68 and out['marker_position'][2]>.025


def test_frame_recovery_backs_off_without_replaying_motion(tmp_path):
    client = FakeClient(fail_first=True)
    waits = []
    ex = Executor(client, lambda *a: [], tmp_path, sleep=waits.append)
    image, tag = ex.fresh_frame()
    assert tag == 'front' and client.calls == 2 and waits == [5.]


def test_low_battery_returns_charge_request_without_motion(tmp_path):
    ex = Executor(FakeClient(status={'battery': 15}), lambda *a: [], tmp_path, sleep=lambda _: None)
    with pytest.raises(LookupError, match='charge'):
        ex.safety()


def test_jog_executes_on_received_rpc_without_camera_frame(tmp_path):
    ex = Executor(FakeClient(), lambda *a: [], tmp_path, sleep=lambda _: None)
    ex.jog({'m': [1, 0, 0], 'state': [240, 105]})
    assert 'arm_jog' in (tmp_path / 'timeline.jsonl').read_text()


def test_servo_proceeds_when_marker_out_of_frame(tmp_path):
    ex = Executor(FakeClient(fill=255), lambda *a: [], tmp_path, sleep=lambda _: None)
    blob, current, spot = ex.front_servo('yellow', planner.GRASP, [.48, .02, 0])
    assert current == planner.GRASP and spot == [.48, .02, 0]


def test_servo_requires_same_frame_geometric_alignment(tmp_path):
    client = FakeClient()
    ex = Executor(client, lambda *a: [], tmp_path, sleep=lambda _: None)
    state = tuple(planner.grasp_goal(planner.grasp_point(), (190, 65))['state'])
    T = planner.hand.hand_pose(state)
    aim = np.array(planner.operational_point(state))
    sample = dict(
        jaw=dict(position=aim.tolist(), hand_pose_base=T.tolist(), uncertainty_bound_m=.015),
        object=dict(position=aim[:2].tolist() + [0], color='yellow'),
        image=np.zeros((1200, 1600, 3), np.uint8),
    )
    ex.front_measure = lambda color, previous: sample
    blob, current, spot = ex.front_servo('yellow', state, sample['object']['position'])
    assert spot == sample['object']['position']
    rows = (tmp_path / 'timeline.jsonl').read_text()
    assert rows.count('front_jaw_check') == 2
