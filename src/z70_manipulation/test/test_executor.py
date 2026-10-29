import math
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import requests
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from z70_interfaces.action import AlignToTarget, DeployArm, GraspAndLift, MoveArm, Pick, PinAndGo, ReleaseAndStow, Stack
from z70_interfaces.msg import FrameMetadata, RobotState
from z70_manipulation import perception, planner
from z70_manipulation.executor import Executor
from z70_manipulation.goals import ALL_GOALS, BUILTIN_GOALS, GoalContext, GoalSpec, simulate_goal
from z70_manipulation.node import Client, Observations, PickServer, run, run_stack

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def fake(tmp_path, monkeypatch):
    events = []
    state = {'camera': 'front', 'gripped': False, 'error': 0, 'joints': [150, 45, 0], 'arm_state': 'IN_PROCESS_POS', 'floor_point': planner.grasp_point()}

    def handle_get(path):
        events.append(('GET', path))
        if path == '/api/state':
            action = 3 if state.get('deployed') else 0
            return {'telemetry': {'status': {'value': {'error_code': state['error'], 'state': 39, 'common_status': (action << 6) | (state.get('grab_mode', 0) << 10)}}}}
        j = state['joints']
        m5 = j[2] if len(j) > 2 else 0
        return {'state': state['arm_state'], 'joints': {'m3': j[0], 'm4': j[1], 'm5': m5, 'm6': 90}}

    def handle_frame():
        events.append(('POST', '/api/camera/frame', {}))
        image = np.full((1200 if state['camera'] == 'front' else 896, 1600, 3), 200, np.uint8)
        if state['camera'] == 'gripper':
            if state['gripped']:
                u, v, radius = state['held_projection']
            else:
                point = state['floor_point']
                px = planner.gripper_pixel(state['joints'][:2], point[:2] + [.033])
                if px is None:
                    px = (800, 450, .3)
                u, v, depth = px
                radius = max(25, round(843 * .066 / depth / 2))
                state['held_projection'] = (u, v, radius)
            cv2.circle(image, (state.get('gripper_u', round(u)), round(v)), radius, (0, 160, 240), -1)
        elif not state['gripped'] or state.get('missed_grip'):
            g = state['floor_point']
            u, v = map(round, perception.front_camera().floor_to_pixel((g[0] - .275) * 1000, g[1] * 1000))
            half = max(10, round(822 * .066 / (g[0] - .275) / 2))
            height = round(half * 1.6)
            cv2.rectangle(image, (u - half, v - height), (u + half, v), (0, 160, 240), -1)
        if state.get('deployed'):
            cam = perception.front_camera()
            cfg = planner._CALIBRATION
            a = math.radians(state['joints'][1] - state['joints'][0] + 90)
            cy, sy = math.cos(planner.YAW), math.sin(planner.YAW)
            B = np.array([[cy * math.cos(a), -sy, -cy * math.sin(a)], [sy * math.cos(a), cy, -sy * math.sin(a)], [math.sin(a), 0, math.cos(a)]])
            R = B @ np.array(cfg['marker_rotation_link2'])
            P = np.array(planner.measured_point(state['joints'][:2], cfg['marker_effective_from_elbow_m']))
            P[2] += state.get('lift', 0.)
            if not state.get('hide_marker'):
                K = np.array([[cam.f, 0, cam.cx], [0, cam.f, cam.cy], [0, 0, 1.]])
                C = np.array([.275, 0, cam.h / 1000])
                obj = np.array([[-.0075, .0075, 0], [.0075, .0075, 0], [.0075, -.0075, 0], [-.0075, -.0075, 0]])
                r = cv2.Rodrigues(cam.R @ R)[0]
                t = cam.R @ (P - C)
                if t[2] > .03:
                    px = cv2.projectPoints(obj, r, t, K, np.array([cam.k1, cam.k2, 0, 0, 0]))[0].reshape(4, 2)
                    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000), 68, 200)
                    H = cv2.getPerspectiveTransform(np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32), px.astype(np.float32))
                    warped = cv2.warpPerspective(marker, H, (1600, 1200), borderValue=255)
                    image[warped < 128] = 0
            if state['gripped'] and state.get('lift') and not state.get('missed_grip'):
                J = P + B @ np.array(cfg['marker_rotation_link2']) @ np.array(cfg['jaw_region_in_marker_m'])
                uv = cam.floor_to_pixel((J[0] - .275) * 1000, J[1] * 1000, J[2] * 1000)
                if uv:
                    radius = round(cam.f * .033 / (J[0] - .275))
                    cv2.circle(image, tuple(map(round, uv)), radius, (0, 160, 240), -1)
        return image, state['camera']

    def handle_rpc(d):
        events.append(('POST', '/api/v1/rpc', d))
        method = d['method']
        result = ['ok']
        if method == 'get_status':
            action = 3 if state.get('deployed') else 0
            result = [{'common_status': (action << 6) | (state.get('grab_mode', 0) << 10), 'error_code': state['error']}]
        if method == 'app_pick_up_enter_exit':
            state['grab_mode'] = d['params']['flag']
        if method == 'app_arm_move':
            while len(state['joints']) < 3:
                state['joints'].append(0)
            for i, c in enumerate(d['params']['m'][:3]):
                state['joints'][i] += 10 if c == 1 else -10 if c == 2 else 0
            state['arm_state'] = 'IN_PROCESS_POS'
        if method == 'app_rc_move':
            if not (isinstance(d['params'], list) and len(d['params']) == 1):
                return {'dispatch': 'sent', 'response': 'vendor_error', 'vendorCode': -10007}
            p = d['params'][0]
            yaw = p['omega'] * .68 * p['duration'] / 1000
            distance = p['velocity'] * .65 * p['duration'] / 1000
            x, y, _ = state['floor_point']
            state['floor_point'] = [math.cos(yaw) * x + math.sin(yaw) * y - distance, -math.sin(yaw) * x + math.cos(yaw) * y, 0.]
        if method == 'app_grip':
            state['gripped'] = True
            state['arm_state'] = 'UNKNOWN'
        if method == 'app_arm_in_compartment':
            state.update(arm_state='IN_ROBOT', deployed=False)
        if method == 'app_arm_out_compartment':
            state.update(arm_state='WORKING_POS', deployed=True, joints=[150, 45, 0], lift=0.)
            yaw = state.get('deploy_yaw', 0.)
            x, y, _ = state['floor_point']
            state['floor_point'] = [math.cos(yaw) * x + math.sin(yaw) * y, -math.sin(yaw) * x + math.cos(yaw) * y, 0.]
            if 'pose' in state:
                state['pose']['yaw'] += yaw
        if method == 'app_arm_direction_move':
            state['lift'] = state.get('lift', 0.) + .04
        if method == 'app_put_down':
            state['gripped'] = False
        if method in ['app_grip', 'app_open_gripper']:
            reason = state.get('grip_failure', 'timeout') if method == 'app_grip' else 'timeout'
            return {'dispatch': 'unknown', 'response': 'transport_error', 'transportReason': reason}
        return {'dispatch': 'sent', 'response': 'received', 'result': result}

    token = tmp_path / 'token'
    token.write_text('fake-test-token')
    origin = 'http://127.0.0.1:0'

    class FakeObservations:
        def __init__(self, *a, **kw):
            pass

        def get(self, path):
            return handle_get(path)

        def frame(self):
            return handle_frame()

        def select(self, camera):
            events.append(('POST', '/api/camera/select', {'camera': camera}))
            state['camera'] = camera
            return {'camera': camera}

        def close(self):
            pass

    monkeypatch.setattr('z70_manipulation.node.Observations', FakeObservations)

    class FakeCommands:
        def __init__(self, *args):
            from z70_interfaces.msg import CommandOutcome
            self.cancelled = threading.Event()
            self.latest_outcome = CommandOutcome(error_code='NONE', dispatch=1, response=1)

        def rpc(self, method, params, timeout_ms):
            from z70_interfaces.msg import CommandOutcome
            res = handle_rpc(dict(method=method, params=params))
            self.latest_outcome = CommandOutcome(
                error_code='NONE' if res.get('response') == 'received' else 'TIMEOUT',
                dispatch=1 if res.get('dispatch') == 'sent' else 2,
                response=1 if res.get('response') == 'received' else 3,
            )
            return res

        def stop(self):
            for method in ('app_rc_stop', 'app_rc_end', 'app_stop_grasp'):
                self.rpc(method, [], 5000)

        def begin(self):
            pass

        def end(self):
            pass

        def close(self):
            pass

    monkeypatch.setattr('z70_manipulation.node.Commands', FakeCommands)
    yield origin, token, events, state


def observer(client):
    def observe(camera, color):
        image, tag = client.frame()
        assert tag == camera
        blobs = perception.front_objects(image) if camera == 'front' else perception.detect(image)
        for b in blobs:
            if b['color'] == color:
                if 'position' not in b:
                    g = planner.grasp_point()
                    w = b['image_size'][0]
                    b['position'] = [g[0], g[1] - (b['pixel'][0] - w / 2) / (843. * w / 1600) * .3, 0.]
                b.update(camera_distance_m=.12, camera_height_m=.12)
        return [b for b in blobs if b['color'] == color]
    return observe


def test_curated_actions_compose_into_full_verified_pick(fake, tmp_path):
    origin, token, events, state = fake
    state['hide_marker'] = True  # Marker out of frame must not block MoveArm or Pick
    client = Client(origin, token)
    ex = Executor(client, observer(client), tmp_path / 'run', sleep=lambda _: None)
    result = ex.pick('yellow')
    assert result['verified'] is True
    assert state['arm_state'] == 'IN_ROBOT'
    methods = [e[2]['method'] for e in events if e[1] == '/api/v1/rpc']
    assert methods.index('app_pick_up_enter_exit') < methods.index('app_arm_out_compartment')
    assert 'app_open_gripper' not in methods and 'app_arm_move' in methods
    assert 'app_grip' in methods and methods.count('app_arm_direction_move') == 2
    assert any(e[2].get('method') == 'app_grip' and e[2].get('params') == {'flag': 0} for e in events if e[1] == '/api/v1/rpc')
    assert 'app_put_down' in methods and 'app_arm_in_compartment' in methods
    assert methods.index('app_grip') < methods.index('app_arm_direction_move') < methods.index('app_arm_in_compartment')
    rows = [json.loads(line) for line in (tmp_path / 'run/timeline.jsonl').read_text().splitlines()]
    assert all('fake-test-token' not in json.dumps(r) for r in rows)


def test_move_arm_uses_kinematic_model_and_rpc_without_camera_frames(fake, tmp_path):
    origin, token, events, state = fake
    state.update(deployed=True, arm_state='WORKING_POS', joints=[150, 45, 0])
    client = Client(origin, token)
    ex = Executor(client, lambda *_: [], tmp_path / 'move-arm', sleep=lambda _: None)
    res = ex.move_arm((190, 65, 20))
    assert res['state'] == [190, 65, 20]
    assert res['steps_executed'] == 4  # 2 coordinated [1,1,1] + 2 [1,0,0]
    assert state['joints'] == [190, 65, 20]
    # No camera frames were requested during MoveArm
    assert not any(e[1] == '/api/camera/frame' for e in events)
    # Out-of-bounds target is rejected by the kinematic model before any RPC
    with pytest.raises(ValueError):
        ex.move_arm((270, 145, 0))


def test_move_arm_nonzero_m5_dispatches_only_safe_states(tmp_path):
    ex = Executor(None, None, tmp_path / 'rolled-arm', sleep=lambda _: None)
    dispatched = []
    ex.jog = lambda step: dispatched.append(step)
    result = ex.move_arm([250, 115, 30], start_joints=[150, 45, 30])
    assert result['state'] == [250, 115, 30]
    assert dispatched and dispatched[-1]['state'] == result['state']
    assert [250, 105, 30] not in [step['state'] for step in dispatched]
    assert all(planner.safe((*step['state'], 90)) for step in dispatched)
    previous = (150, 45, 30, 90)
    for step in dispatched:
        current = (*step['state'], 90)
        assert planner.safe_edge(previous, current)
        previous = current
    dispatched.clear()
    recovered = ex.move_arm([190, 65, 0], start_joints=[207.4, 71.8, 0.2])
    assert recovered['state'] == [190, 65, 0] and recovered['steps_executed'] == 2
    dispatched.clear()
    ex.current_arm_joints = lambda: [207.4, 71.8, 0.2]
    recovered_auto = ex.move_arm([190, 65, 0])
    assert recovered_auto['state'] == [190, 65, 0] and recovered_auto['steps_executed'] == 2


def test_lift_can_be_verified_when_marker_exits_top_but_not_when_floor_object_remains(fake, tmp_path, monkeypatch):
    origin, token, events, state = fake
    client = Client(origin, token)
    state.update(deployed=True, gripped=True, lift=.08, joints=list(planner.GRASP))
    ex = Executor(client, observer(client), tmp_path / 'held-visible', sleep=lambda _: None)
    ex.pregrip_jaw = planner.measured_point(planner.GRASP, planner._CALIBRATION['effective_jaw_from_elbow_m'])

    def missing(*_):
        raise LookupError('raised marker outside frame')

    monkeypatch.setattr(perception, 'front_jaw', missing)
    assert ex.held_front('yellow', state['floor_point'])
    state['missed_grip'] = True
    assert not ex.held_front('yellow', state['floor_point'])


def test_dry_run_only_reads_and_frames(fake, tmp_path):
    origin, token, events, state = fake
    client = Client(origin, token, read_only=True)
    ex = Executor(client, observer(client), tmp_path / 'dry', sleep=lambda _: None)
    result = ex.pick('yellow', dry_run=True)
    assert result['objects'] and result['target']['color'] == 'yellow'
    assert result['descent']
    far_ex = Executor(
        client,
        lambda _cam, _col: [{'color': 'yellow', 'area_px': 400, 'position': [0.85, 0.15, 0.0]}],
        tmp_path / 'dry-far',
        sleep=lambda _: None,
    )
    far_res = far_ex.pick('yellow', dry_run=True)
    assert far_res['target']['position'] == [0.85, 0.15, 0.0] and far_res['approach']['forward'] > 0.3 and far_res['descent']
    assert all(e[0] == 'GET' or e[1] == '/api/camera/frame' and e[2] == {} for e in events)
    with pytest.raises(PermissionError):
        client.rpc('app_rc_start', [])
    with pytest.raises(PermissionError):
        client.select('front')


def test_robot_error_stops_without_deploy(fake, tmp_path):
    origin, token, events, state = fake
    state['error'] = 8
    client = Client(origin, token)
    ex = Executor(client, observer(client), tmp_path / 'error', sleep=lambda _: None)
    with pytest.raises(RuntimeError, match='robot error'):
        ex.pick('yellow')
    methods = [e[2]['method'] for e in events if e[1] == '/api/v1/rpc']
    assert 'app_rc_stop' in methods and 'app_arm_out_compartment' not in methods


def test_external_path_hazard_stops(fake, tmp_path):
    origin, token, events, state = fake
    client = Client(origin, token)
    ex = Executor(client, observer(client), tmp_path / 'hazard', sleep=lambda _: None, hazard=lambda: 'person in path')
    with pytest.raises(RuntimeError, match='person in path'):
        ex.pick('yellow')
    assert any(e[1] == '/api/v1/rpc' and e[2]['method'] == 'app_rc_stop' for e in events)


def test_approach_reobserves_until_bearing_and_range_converge(fake, tmp_path):
    origin, token, events, state = fake
    state['floor_point'] = [.72, .12, 0.]
    client = Client(origin, token)
    ex = Executor(client, observer(client), tmp_path / 'approach', sleep=lambda _: None)
    blob = ex.align_front('yellow')
    assert abs(blob['bearing_error_deg']) <= 2 and abs(blob['range_error_m']) <= .015
    pulses = [e[2]['params'] for e in events if e[1] == '/api/v1/rpc' and e[2]['method'] == 'app_rc_move']
    assert all(isinstance(p, list) and len(p) == 1 for p in pulses)
    assert [p[0]['seqnum'] for p in pulses] == list(range(1, len(pulses) + 1))


def test_post_deploy_reacquires_after_vendor_spin_before_any_descent(fake, tmp_path):
    origin, token, events, state = fake
    state.update(deploy_yaw=.20, pose=dict(x=0., y=0., yaw=0.))
    client = Client(origin, token)
    ex = Executor(client, observer(client), tmp_path / 'spin', sleep=lambda _: None, odom=lambda: dict(state['pose']))

    def reached_descent(step):
        raise LookupError('descent reached')

    ex.jog = reached_descent
    with pytest.raises(LookupError, match='descent reached'):
        ex.pick('yellow')
    methods = [e[2]['method'] for e in events if e[1] == '/api/v1/rpc']
    assert 'app_rc_move' in methods[methods.index('app_arm_out_compartment') + 1:]
    rows = [json.loads(line) for line in (tmp_path / 'spin/timeline.jsonl').read_text().splitlines()]
    row = next(r for r in rows if r['event'] == 'post_deploy_alignment')
    assert row['yaw_offset_rad'] == pytest.approx(.20)
    assert abs(row['bearing_error_deg']) > 10


def test_target_prediction_follows_the_robots_own_motion():
    e = Executor.__new__(Executor)
    e.last_motion = (0.10, 0.0, 0.0)
    assert e.predict([0.60, 0.05, 0.]) == pytest.approx([0.50, 0.05, 0.], abs=1e-9)
    e.last_motion = (0.0, 0.0, math.radians(-20))
    x, y, _ = e.predict([0.59, -0.20, 0.])
    assert abs(math.degrees(math.atan2(y, x))) < abs(math.degrees(math.atan2(-0.20, 0.59)))


class MemoryClient:
    """No sockets, ROS or robot; records the executor's vendor command boundary."""
    def __init__(self, mode=2, confirm_mode=True):
        self.calls = []
        self.mode = mode
        self.confirm_mode = confirm_mode

    def rpc(self, method, params, timeout_ms=5000):
        self.calls.append((method, params))
        if method == 'app_pick_up_enter_exit' and self.confirm_mode:
            self.mode = params['flag']
        return dict(dispatch='sent', response='received')

    def get(self, path):
        if path == '/api/state':
            return {'telemetry':{'status':{'value':{'common_status':self.mode << 10}}}}
        return {'state':'UNKNOWN', 'joints':{}}


def test_normal_open_requires_observed_manual_mode_before_in_place_release(tmp_path):
    client = MemoryClient()
    ex = Executor(client, lambda *_: [], tmp_path/'open', sleep=lambda _:None)
    assert ex.open_jaws()['release_dispatched']
    assert client.calls == [('app_pick_up_enter_exit', {'flag':1}), ('app_put_down', {'flag':0})]
    assert ex.gripped is False
    client = MemoryClient(confirm_mode=False)
    ex = Executor(client, lambda *_: [], tmp_path/'unconfirmed', sleep=lambda _:None)
    ex.gripped = True
    with pytest.raises(LookupError, match='manual'):
        ex.open_jaws()
    assert ex.gripped is True
    assert all(method != 'app_put_down' for method,_ in client.calls)


def placement_sample(gap, lateral=(0.,0.), separation=.03, jaw_z=None):
    center = [.5, 0., .10]
    z = center[2] + separation if jaw_z is None else float(jaw_z)
    dx = math.sqrt(max(0., separation * separation - (z - center[2]) ** 2))
    jaw_pos = [center[0] - dx, center[1], z]
    pose = np.eye(4)
    pose[:3, 3] = jaw_pos
    return dict(gap_m=gap, lateral_error_m=list(lateral), held_center_m=center,
                held_bottom_m=[.5, 0., .066 + gap], support_top_m=[.5, 0., .066],
                tower_spot=[.5, 0., 0.], jaw_separation_m=float(np.linalg.norm(np.subtract(center, jaw_pos))),
                jaw=dict(position=jaw_pos, hand_pose_base=pose.tolist()), uncertainty_m=.015)


def test_supported_place_descends_observes_contact_then_verifies_detach_stability(tmp_path):
    client = MemoryClient()
    ex = Executor(client, lambda *_:[], tmp_path/'supported', sleep=lambda _:None)
    ex.gripped = True
    # Unknown arm pose permits a real 40mm step ONLY when gap allows it.
    samples = iter([placement_sample(.060), placement_sample(.002), placement_sample(.002)] +
                   [placement_sample(.001, separation=.075) for _ in range(3)])
    reads = []
    def sample(*args, **kwargs):
        reads.append((list(client.calls),kwargs))
        return next(samples)
    ex.placement_sample = sample
    result = ex.supported_place('purple','blue','blue',[.5,0.,0.], tier=2)
    assert result['verified'] and result['detached']
    assert ex.gripped is False
    calls = client.calls
    lower = calls.index(('app_arm_direction_move',{'direction':2}))
    release = calls.index(('app_put_down',{'flag':0}))
    clear = calls.index(('app_arm_direction_move',{'direction':1}))
    assert lower < release < clear
    assert reads[1][1]['after'] > 0  # post-descent image cannot be a buffered pre-step sample
    assert all(kwargs['released'] and kwargs['after'] > 0 for _,kwargs in reads[-3:])
    assert result['stable_observations'] == 3


def test_precision_limited_place_returns_incomplete_without_release_or_stop(tmp_path):
    client = MemoryClient()
    ex = Executor(client, lambda *_:[], tmp_path/'unresolved', sleep=lambda _:None)
    ex.gripped = True
    ex.placement_sample = lambda *a,**kw: placement_sample(.020)
    result = ex.supported_place('purple','blue','blue',[.5,0.,0.], tier=2)
    assert not result['verified']
    assert result['reason'] == 'supported-placement-unresolved'
    assert ex.gripped and not ex.stopped
    assert client.calls == []


def test_failed_detachment_never_marks_object_released(tmp_path):
    client = MemoryClient()
    ex = Executor(client, lambda *_:[], tmp_path/'stuck', sleep=lambda _:None)
    ex.gripped = True
    ex.placement_sample = lambda *a,**kw: placement_sample(.001)
    result = ex.supported_place('purple','blue','blue',[.5,0.,0.], tier=2)
    assert not result['verified'] and ex.gripped


def test_place_alignment_tracks_measured_held_column_not_grasp_point(tmp_path):
    client = MemoryClient()
    ex = Executor(client, lambda *_:[], tmp_path/'column', sleep=lambda _:None)
    observations = iter([placement_sample(.06, lateral=(.03,0.)), placement_sample(.02)])
    ex.placement_sample = lambda *a,**kw: next(observations)
    motions=[]
    ex.pulse = lambda **kw: motions.append(kw)
    result = ex.supported_place('purple','blue','blue',[.5,0.,0.], tier=2)
    assert not result['verified']
    assert motions[0]['velocity'] < 0  # support is behind the measured held column


def stack_executor(tmp_path):
    ex = Executor(MemoryClient(), lambda *_:[], tmp_path/'stack', sleep=lambda _:None)
    ex.deployed = True
    ex.target = lambda *a,**kw: dict(position=[.5,0.,0.])
    ex.align_front = lambda color: dict(position=[.5,0.,0.])
    ex.current_arm_joints = lambda: [150.,45.,0.]
    ex.move_arm = lambda *a,**kw: {}
    ex.front_servo = lambda color,current,spot: ({},current,spot)
    ex.grasp_and_lift = lambda **kw: dict(verified=True)
    ex.snap = lambda _:None
    ex.stow = lambda: dict(stowed=True)
    return ex


def test_stack_keeps_already_observed_front_stream_after_deploy(tmp_path):
    ex = stack_executor(tmp_path)
    ex.deployed = False
    ex.deploy = lambda **kw: setattr(ex, 'deployed', True)
    ex.client.select = lambda *_: (_ for _ in ()).throw(LookupError('BUSY'))
    placements = []
    ex.supported_place = lambda *a, **kw: placements.append(a) or dict(verified=True, detached=True)
    result = ex.stack('blue', 'purple', 'yellow')
    assert len(placements) == 2
    assert result['stack_verified']


def test_stack_reads_start_pose_before_wrist_preserving_manual_open(tmp_path):
    ex = stack_executor(tmp_path)
    ex.supported_place = lambda *a, **kw: dict(verified=False)
    def start_pose():
        assert not any(method == 'app_put_down' for method, _ in ex.client.calls)
        return [150., 45., 0.]
    ex.current_arm_joints = start_pose
    result = ex.stack('blue', 'purple', 'yellow')
    assert result['reason'] == 'supported-placement-unresolved'


def test_stack_counts_only_verified_placement_and_returns_incomplete_not_emergency(tmp_path):
    ex = stack_executor(tmp_path)
    ex.supported_place = lambda *a,**kw: dict(verified=False,reason='supported-placement-unresolved')
    result = ex.stack('blue','purple','yellow')
    assert result['tiers_completed'] == 1 and not result['stack_verified']
    assert result['reason'] == 'supported-placement-unresolved'
    assert not ex.stopped
    assert all(method != 'app_stop_grasp' for method,_ in ex.client.calls)


def test_stack_never_uses_pre_lift_joint_labels_to_start_next_tier(tmp_path):
    ex = stack_executor(tmp_path)
    placements=[]
    def placed(*args,**kwargs):
        placements.append(args)
        return dict(verified=True,detached=True)
    ex.supported_place = placed
    starts=[]
    ex.current_arm_joints = lambda: [210.,75.,0.]
    ex.move_arm = lambda goal,start_joints=None: starts.append(start_joints)
    result = ex.stack('blue','purple','yellow')
    assert result['stack_verified'] and result['tiers_completed'] == 3
    assert len(placements) == 2
    assert placements[0][1:3] == ('blue','blue')
    assert placements[1][1:3] == ('purple','blue')
    assert starts[0] == [210.,75.,0.] and starts[2] == [210.,75.,0.]


def test_pick_uses_normal_in_place_open_instead_of_recovery_rpc(fake,tmp_path):
    origin,token,events,state = fake
    state['hide_marker'] = True
    client = Client(origin,token)
    ex = Executor(client,observer(client),tmp_path/'normal-pick',sleep=lambda _:None)
    ex.pick('yellow')
    methods = [e[2]['method'] for e in events if e[1]=='/api/v1/rpc']
    assert 'app_open_gripper' not in methods


def test_front_servo_retains_observed_object_in_hand_transform(tmp_path):
    ex=Executor(MemoryClient(),lambda *_:[],tmp_path/'transform',sleep=lambda _:None)
    T=np.eye(4);T[:3,3]=[.5,0.,.033]
    ex.front_measure=lambda *_:dict(image=np.zeros((10,10,3),np.uint8),
        jaw=dict(position=[.5,0.,.033],hand_pose_base=T.tolist(),uncertainty_bound_m=.015),
        object=dict(position=[.5,0.,0.]))
    ex.front_servo('purple',(240,95),[.5,0.,0.])
    assert ex.pregrip_object_in_hand == pytest.approx([0.,0.,0.])


def test_snapshot_names_are_unique_per_tier_attempt(tmp_path):
    ex=Executor(MemoryClient(),lambda *_:[],tmp_path/'photos',sleep=lambda _:None)
    ex.fresh_frame=lambda:(np.zeros((10,10,3),np.uint8),'front')
    ex.evidence_prefix='tier-2-attempt-1-';ex.snap('1-pregrip')
    ex.evidence_prefix='tier-3-attempt-1-';ex.snap('1-pregrip')
    assert len(list(ex.directory.glob('*pregrip-front.jpg'))) == 2
    rows = [json.loads(line) for line in (ex.directory / 'timeline.jsonl').read_text().splitlines()]
    assert all(row['source_id'] is None and row['frame_received_at'] is None
               and row['exposure_time_known'] is False for row in rows)


def test_snapshot_provenance_stays_with_consumed_pixels_when_buffer_advances(tmp_path, monkeypatch):
    obs = Observations.__new__(Observations)
    obs.condition = threading.Condition()
    obs.timeout = .1
    obs.cancelled = lambda: False
    obs.used = obs.image = None
    obs.frames = {}; obs.metadata = {}
    now = int(time.time())
    state = RobotState(connected=True)
    state.evidence.valid = True
    state.evidence.received_at.sec = now
    obs.on_state(state)

    def frame(session, sequence, received_ns, value, exposure_known):
        image = CompressedImage(format='png')
        image.header.stamp.sec = now
        image.header.stamp.nanosec = received_ns
        image.data = cv2.imencode('.png', np.full((16, 16, 3), value, np.uint8))[1].tobytes()
        meta = FrameMetadata(width=16, height=16, camera=1, exposure_time_known=exposure_known)
        meta.header = image.header
        meta.evidence.session_id.uuid = [session] * 16
        meta.evidence.sequence = sequence
        meta.evidence.received_at.sec = now - 1
        meta.evidence.received_at.nanosec = received_ns
        meta.evidence.valid = True
        return image, meta

    first, first_meta = frame(1, 7, 100_000_000, 40, False)
    restarted, restarted_meta = frame(2, 1, 200_000_000, 200, True)
    obs.on_image(first, 'front'); obs.on_metadata(first_meta, 'front')
    client = Client(read_only=True, observations=obs)
    ex = Executor(client, lambda *_: [], tmp_path / 'evidence', sleep=lambda _: None)
    write = cv2.imwrite

    def advance_buffer(path, pixels):
        obs.on_metadata(restarted_meta, 'front')
        obs.on_image(restarted, 'front')
        return write(path, pixels)

    monkeypatch.setattr(cv2, 'imwrite', advance_buffer)
    ex.snap('5-put-down')
    monkeypatch.setattr(cv2, 'imwrite', write)
    ex.snap('6-stowed')
    rows = [json.loads(line) for line in (ex.directory / 'timeline.jsonl').read_text().splitlines()]
    snapshots = [row for row in rows if row['event'] == 'snapshot']
    assert len(snapshots) == 2
    for row, name, session, sequence, receipt, value, exposure in (
        (snapshots[0], '5-put-down', 1, 7, now - 1 + .1, 40, False),
        (snapshots[1], '6-stowed', 2, 1, now - 1 + .2, 200, True),
    ):
        assert row['name'] == name and row['camera'] == 'front'
        assert row['source_id'] == [bytes([session] * 16).hex(), sequence]
        assert row['frame_received_at'] == receipt
        assert row['exposure_time_known'] is exposure
        assert np.all(cv2.imread(str(ex.directory / f'{name}-front.jpg')) == value)


def test_snapshot_write_failure_is_recorded_as_gap_without_a_snapshot(tmp_path, monkeypatch):
    ex = Executor(MemoryClient(), lambda *_: [], tmp_path / 'write-failure', sleep=lambda _: None)
    ex.fresh_frame = lambda: (np.zeros((10, 10, 3), np.uint8), 'front')
    monkeypatch.setattr(cv2, 'imwrite', lambda *_: False)
    ex.snap('5-put-down')
    rows = [json.loads(line) for line in (ex.directory / 'timeline.jsonl').read_text().splitlines()]
    assert not any(row['event'] == 'snapshot' for row in rows)
    assert any(row['event'] == 'data_gap' and row['kind'] == 'snapshot' for row in rows)
    assert not list(ex.directory.glob('*.jpg'))


def test_placement_post_command_frames_reject_earlier_receipts(tmp_path,monkeypatch):
    client=MemoryClient()
    ex=Executor(client,lambda *_:[],tmp_path/'timing',sleep=lambda _:None)
    def old_frame():
        client.frame_received_at=9.
        return np.zeros((10,10,3),np.uint8),'front'
    ex.fresh_frame=old_frame
    monkeypatch.setattr(planner,'measure',lambda *a,**kw:pytest.fail('old frame used'))
    with pytest.raises(LookupError,match='after command'):
        ex.placement_sample('purple','blue','blue',[.5,0.,0.],released=True,after=10.)


def test_release_not_verified_when_stack_geometry_moves(tmp_path):
    ex = Executor(MemoryClient(),lambda *_:[],tmp_path/'unstable',sleep=lambda _:None)
    ex.gripped=True
    stable=placement_sample(.001,separation=.075)
    moved=placement_sample(.001,separation=.075);moved['held_center_m']=[.5,.02,.10]
    samples=iter([placement_sample(.001),placement_sample(.001),stable,moved])
    ex.placement_sample=lambda *a,**kw:next(samples)
    result=ex.supported_place('purple','blue','blue',[.5,0.,0.],2)
    assert not result['verified'] and ex.gripped
    assert 'moved' in result['detail']


def test_stack_resets_pregrip_geometry_between_tiers(tmp_path):
    ex=stack_executor(tmp_path)
    seen=[]
    def servo(color,current,spot):
        seen.append((ex.pregrip_jaw,ex.pregrip_object_in_hand,ex.evidence_prefix))
        ex.pregrip_jaw=[.4,0.,.03];ex.pregrip_object_in_hand=[0.,0.,.02]
        return {},current,spot
    ex.front_servo=servo
    ex.supported_place=lambda *a,**kw:dict(verified=True,detached=True)
    ex.stack('blue','purple','yellow')
    assert len(seen)==2
    assert all(jaw is None and transform is None for jaw,transform,_ in seen)
    assert seen[0][2] != seen[1][2]


def test_current_pose_recovery_uses_observation_when_estimator_unknown(tmp_path,monkeypatch):
    ex=Executor(MemoryClient(),lambda *_:[],tmp_path/'recover',sleep=lambda _:None)
    ex.fresh_frame=lambda:(np.zeros((10,10,3),np.uint8),'front')
    monkeypatch.setattr(perception,'front_jaw',lambda _:dict(hand_pose_base=planner.hand.hand_pose([210.,75.,0.]).tolist()))
    assert ex.current_arm_joints() == pytest.approx([210.,75.,0.])


def test_stow_polls_common_status_instead_of_blind_10s_sleep(tmp_path):
    client = MemoryClient()
    statuses = iter([{'common_status': 3 << 6}, {'common_status': 0}])
    client.status = lambda: next(statuses, {'common_status': 0})
    client.arm_estimate = lambda: {'state': 'IN_ROBOT', 'joints': {}}
    sleeps = []
    ex = Executor(client, lambda *_: [], tmp_path / 'stow', sleep=lambda d: sleeps.append(d))
    ex.snap = lambda _: None
    assert ex.stow() == {'stowed': True, 'state': 'IN_ROBOT'}
    assert 10. not in sleeps and sleeps == [.5, 1.]


def test_pickup_client_delegates_observations_without_network_or_token(tmp_path, monkeypatch):
    calls = []

    class Shared:
        def get(self, path):
            calls.append(('get', path))
            return {'shared': True}

        def frame(self):
            calls.append(('frame',))
            return 'image', 'front'

        def select(self, camera):
            calls.append(('select', camera))
            return {'camera': camera}

    def no_network(*args, **kwargs):
        raise AssertionError('independent HTTP observation')

    monkeypatch.setattr('requests.Session.request', no_network)
    client = Client('http://not-contacted.invalid', tmp_path / 'does-not-exist', read_only=True, observations=Shared())
    try:
        assert client.get('/api/state') == {'shared': True}
        assert client.frame() == ('image', 'front')
        with pytest.raises(PermissionError):
            client.select('front')
        with pytest.raises(PermissionError):
            client.rpc('get_status', [])
        assert calls == [('get', '/api/state'), ('frame',)]
    finally:
        client.close()


def test_simulate_goal_exercises_all_builtin_goals():
    assert len(ALL_GOALS) == 15
    cases = [
        ('pick', dict(color='yellow', dry_run=False), lambda r: r.verified_hold),
        ('stack', dict(base_color='blue', middle_color='purple', top_color='yellow', dry_run=False), lambda r: r.stack_verified and r.tiers_completed == 3),
        ('pin_and_go', dict(target_x_m=1., target_y_m=2., max_duration_s=60., dry_run=False), lambda r: r.arrived),
        ('deliver_garment', dict(category='sock', drop_x_m=1., drop_y_m=2., max_duration_s=60., allow_towing=False, dry_run=False), lambda r: r.delivered),
        ('patrol_collect', dict(drop_x_m=1., drop_y_m=2., max_duration_s=60., max_items=3, allow_towing=False, dry_run=False), lambda r: r.completed),
        ('mission', dict(steps_json=json.dumps([
            {'op': 'navigate_room', 'room': 'Kitchen'},
            {'op': 'observe', 'color': 'yellow'},
            {'op': 'pick', 'color': 'yellow'},
        ]), max_duration_s=60., dry_run=False), lambda r: r.completed and r.steps_completed == 3),
        ('deploy_arm', dict(settle=True), lambda r: r.ready),
        ('align_target', dict(color='yellow'), lambda r: r.aligned),
        ('move_arm', dict(m3_deg=160., m4_deg=45., m5_deg=0.), lambda r: r.steps_executed == 1),
        ('grasp_and_lift', dict(lift_steps=2, verify_color='yellow'), lambda r: r.verified_hold),
        ('release_and_stow', dict(put_down=True), lambda r: r.stowed),
        ('base_pulse', dict(linear_mps=0.05, angular_radps=0., duration=dict(sec=0, nanosec=300000000)), lambda r: r.outcome.error_code == 'NONE'),
        ('arm_step', dict(joint=34, direction=1), lambda r: r.outcome.error_code == 'NONE'),
        ('hand_step', dict(direction=1), lambda r: r.outcome.error_code == 'NONE'),
        ('arm_operation', dict(operation=7), lambda r: r.outcome.error_code == 'NONE'),
    ]
    for name, payload, check in cases:
        spec = ALL_GOALS[name]
        assert spec.validate_payload(payload), name
        context = {'objects': [{'color': 'yellow', 'position': [0.35, 0., 0.03]}]} if name == 'mission' else {}
        ok, res = simulate_goal(spec, SimpleNamespace(**payload), **context)
        assert ok and check(res) and isinstance(message_to_ordereddict(res), dict), name

    route_spec = BUILTIN_GOALS['pin_and_go']
    ok, _ = simulate_goal(
         GoalSpec('route_check', route_spec.action_cls, '/z70/route_check', 10., lambda _s, _r: True,
                  lambda ctx, r: (ctx.plan_route(r.target_x_m, r.target_y_m) == [(1., 2.)]
                                  and ctx.corridor_safe(0., 0., r.target_x_m, r.target_y_m) is True
                                  and 'rooms' in ctx.spatial_context()
                                  and ctx.wrist_pose()['marker_id'] == 68
                                  and ctx.fit_wrist_joints()['joints'] == [190., 65., 0.]
                                  and ctx.arm_safe(150., 45., 0., 90.) is True
                                  and 'yaw' in ctx.plan_approach(.5, .05)
                                  and ctx.plan_grasp(.43, 0.)['state'] == [250, 105]
                                  and ctx.plan_grasp(2.0, 0.) is None
                                  and ctx.arm_step(1, 2, 0) == {'m3': 1, 'm4': 2, 'm5': 0}
                                  and ctx.hand_step(1) == {'direction': 1}
                                  and ctx.open_jaws() is None
                                  and ctx.set_arm_anchor('deployed', 'ID68_VISIBLE')['accepted'] is True, route_spec.action_cls.Result())),
        SimpleNamespace(target_x_m=1., target_y_m=2.),
    )
    assert ok

    rpc_calls = []
    fake_client = SimpleNamespace(commands=None, rpc=lambda m, p, timeout_ms=5000: rpc_calls.append((m, p, timeout_ms)) or {'result': ['ok']})
    fake_server = SimpleNamespace(observations=None, gains={}, odom=lambda: None)
    real_ctx = GoalContext(fake_server, SimpleNamespace(request=SimpleNamespace(), is_cancel_requested=False),
                           route_spec.action_cls, fake_client, Path('.'), create_executor=False)
    real_ctx.arm_step(1, 2, 0)
    real_ctx.hand_step(3)
    assert rpc_calls == [
        ('app_arm_move', {'m': [1, 2, 0]}, 3000),
        ('app_arm_direction_move', {'direction': 3}, 3000),
    ]
    assert [t['primitive'] for t in real_ctx.trace] == ['arm_step', 'rpc', 'hand_step', 'rpc']
    real_ctx.arm = lambda: {'state': 'WORKING_POS', 'joints': {'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.}}
    with pytest.raises(ValueError, match='unsafe arm_step'):
        real_ctx.arm = lambda: {'state': 'WORKING_POS', 'joints': {'m3': 150., 'm4': 35., 'm5': 0., 'm6': 90.}}
        real_ctx.arm_step(0, 2, 0)
    with pytest.raises(ValueError, match='unreachable hand_step'):
        real_ctx.arm = lambda: {'state': 'WORKING_POS', 'joints': {'m3': 250., 'm4': 105., 'm5': 0., 'm6': 90.}}
        real_ctx.hand_step(2)
    real_ctx.arm = lambda: {'state': 'WORKING_POS', 'joints': {'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.}}
    assert real_ctx.plan_grasp(0.43, 0.0)['state'] == [250, 105]
    assert real_ctx.plan_stack_tier(1)['tier'] == 1
    assert len(real_ctx.plan_trajectory(170., 45., 0.)) == 2
    real_ctx.arm = lambda: {'state': 'UNKNOWN', 'joints': None}
    real_ctx.fit_wrist_joints = lambda: {'joints': [188.2, 64.1, 0.3]}
    assert real_ctx._grid_joints() == (190, 65, 0)
    real_ctx.arm = lambda: {'state': 'IN_ROBOT', 'joints': None}
    assert real_ctx._grid_joints() == (150, 45, 0)
    real_ctx.safe_interior = lambda refresh=False, deployed_arm_extension_m=0.: (
        SimpleNamespace(labels={2: 'Kitchen'}),
        SimpleNamespace(waypoints=[
            {'segment_id': 2, 'x': 2.5, 'y': 1.5, 'clearance_m': 0.6},
            {'segment_id': 2, 'x': 3.0, 'y': 2.0, 'clearance_m': 0.9},
        ]),
    )
    assert real_ctx.room_waypoint('kitchen') == {'segment_id': 2, 'name': 'Kitchen', 'x': 3.0, 'y': 2.0, 'clearance_m': 0.9}
    assert real_ctx.room_waypoint('Segment 2')['x'] == 3.0
    assert real_ctx.room_waypoint(2)['x'] == 3.0
    assert real_ctx.room_waypoint('attic') is None
    real_ctx.navigate = lambda x, y, max_duration_s=60., dry_run=False: {'arrived': True, 'dry_run_valid': False, 'status_code': 'ARRIVED'}
    assert real_ctx.navigate_room('kitchen') == {'arrived': True, 'dry_run_valid': False, 'status_code': 'ARRIVED', 'room': 'Kitchen', 'segment_id': 2, 'target_x_m': 3.0, 'target_y_m': 2.0}
    assert real_ctx.navigate_room('attic') == {'arrived': False, 'dry_run_valid': False, 'status_code': 'ROOM_UNREACHABLE', 'room': 'attic', 'detail': 'ROOM_UNREACHABLE'}

    from z70_interfaces.msg import ArmEstimate
    from z70_interfaces.srv import SetArmAnchor
    from z70_manipulation.node import Commands
    cmd = Commands.__new__(Commands)
    est = ArmEstimate(); est.validity.value = 1
    cmd.call = lambda kind, req, timeout: SetArmAnchor.Response(estimate=est)
    assert cmd.set_arm_anchor('deployed', 'ID68_VISIBLE') == {'accepted': True, 'anchor': 'deployed'}
    assert cmd.set_arm_anchor('stowed', 'STOW_DONE') == {'accepted': False, 'anchor': 'stowed'}


class _FakeJawClient:
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


def test_frame_recovery_backs_off_without_replaying_motion(tmp_path):
    client = _FakeJawClient(fail_first=True)
    waits = []
    ex = Executor(client, lambda *a: [], tmp_path, sleep=waits.append)
    image, tag = ex.fresh_frame()
    assert tag == 'front' and client.calls == 2 and waits == [5.]


def test_low_battery_returns_charge_request_without_motion(tmp_path):
    ex = Executor(_FakeJawClient(status={'battery': 15}), lambda *a: [], tmp_path, sleep=lambda _: None)
    with pytest.raises(LookupError, match='charge'):
        ex.safety()


def test_jog_executes_on_received_rpc_without_camera_frame(tmp_path):
    ex = Executor(_FakeJawClient(), lambda *a: [], tmp_path, sleep=lambda _: None)
    ex.jog({'m': [1, 0, 0], 'state': [240, 105]})
    assert 'arm_jog' in (tmp_path / 'timeline.jsonl').read_text()


def test_servo_proceeds_when_marker_out_of_frame(tmp_path):
    ex = Executor(_FakeJawClient(fill=255), lambda *a: [], tmp_path, sleep=lambda _: None)
    blob, current, spot = ex.front_servo('yellow', planner.GRASP, [.48, .02, 0])
    assert current == planner.GRASP and spot == [.48, .02, 0]


def test_servo_requires_same_frame_geometric_alignment(tmp_path):
    client = _FakeJawClient()
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


def _send_and_wait(action, goal, timeout=10.):
    assert action.wait_for_server(timeout_sec=5)
    future = action.send_goal_async(goal)
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.02)
    handle = future.result()
    assert handle.accepted
    outcome = handle.get_result_async()
    while not outcome.done() and time.monotonic() < deadline:
        time.sleep(.02)
    assert outcome.done()
    return outcome.result().result


def test_ros_pick_and_curated_actions(fake, tmp_path):
    origin, token, _, _ = fake
    rclpy.init(domain_id=92)
    server = PickServer(parameter_overrides=[
        rclpy.parameter.Parameter('repo', value=str(ROOT)),
        rclpy.parameter.Parameter('origin', value=origin),
        rclpy.parameter.Parameter('token_file', value=str(token)),
        rclpy.parameter.Parameter('log_root', value=str(tmp_path)),
    ])
    client_node = rclpy.create_node('pick_test_client')
    received, markers = [], []
    client_node.create_subscription(String, '/z70/objects/json', lambda m: received.append(m.data), 10)
    client_node.create_subscription(MarkerArray, '/z70/objects', lambda m: markers.append(m), 10)
    clients = {
        cls: ActionClient(client_node, cls, topic)
        for cls, topic in [
            (Pick, '/z70/pick'),
            (Stack, '/z70/stack'),
            (DeployArm, '/z70/arm/deploy'),
            (AlignToTarget, '/z70/align_target'),
            (MoveArm, '/z70/arm/move'),
            (GraspAndLift, '/z70/arm/grasp_and_lift'),
            (ReleaseAndStow, '/z70/arm/release_and_stow'),
        ]
    }
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        result = _send_and_wait(clients[Pick], Pick.Goal(color='yellow', dry_run=True))
        assert result.dry_run_valid and not result.verified_hold and result.outcome.error_code == 'NONE'
        assert received and markers

        assert _send_and_wait(clients[DeployArm], DeployArm.Goal(settle=False)).ready
        assert _send_and_wait(clients[AlignToTarget], AlignToTarget.Goal(color='yellow')).aligned
        assert _send_and_wait(clients[MoveArm], MoveArm.Goal(m3_deg=170., m4_deg=45., m5_deg=0.)).steps_executed == 2
        assert _send_and_wait(clients[GraspAndLift], GraspAndLift.Goal(lift_steps=1, verify_color='')).outcome.error_code == 'NONE'
        assert _send_and_wait(clients[ReleaseAndStow], ReleaseAndStow.Goal(put_down=False), timeout=20.).stowed

        stack_res = _send_and_wait(clients[Stack], Stack.Goal(base_color='yellow', middle_color='blue', top_color='purple', dry_run=True))
        assert not stack_res.stack_verified and stack_res.evidence_uri and stack_res.outcome.error_code == 'OBSERVATION_GAP'
    finally:
        for c in clients.values():
            c.destroy()
        executor.shutdown()
        thread.join()
        server.destroy_node()
        client_node.destroy_node()
        rclpy.try_shutdown()


def test_cli_dry_run_is_read_only(fake, tmp_path):
    origin, token, events, _ = fake
    result = run(['yellow', '--dry-run', '--origin', origin, '--token-file', str(token), '--repo', str(ROOT), '--log-dir', str(tmp_path / 'cli')])
    assert result['target']['color'] == 'yellow' and result['objects']
    assert (tmp_path / 'cli/front.jpg').exists()
    assert all(e[0] == 'GET' or e[1] == '/api/camera/frame' and e[2] == {} for e in events)
    stack_res = run_stack(['yellow', 'blue', 'purple', '--dry-run', '--origin', origin, '--token-file', str(token), '--repo', str(ROOT), '--log-dir', str(tmp_path / 'stack-cli')])
    assert stack_res['dry_run'] and len(stack_res['tiers']) == 3
    assert stack_res['targets']['yellow'] is not None
    assert (tmp_path / 'stack-cli/front.jpg').exists()


def test_simulate_goal_filters_list_overrides_and_plans_stack_tier():
    custom = GoalSpec(
        'custom_check', Pick, '/z70/custom_check', 10., lambda _s, _r: True,
        lambda ctx, req: (
            bool(ctx.objects('front', req.color)) and ctx.plan_stack_tier(1)['tier'] == 1 and ctx.arm_kinematics(150, 45)['safe'],
            Pick.Result(outcome=ctx.outcome(True), verified_hold=bool(ctx.objects('front', req.color)), dry_run_valid=req.dry_run, evidence_uri='sim'),
        ),
        fields=('color', 'dry_run'),
    )
    objs = [{'color': 'yellow', 'position': [0.35, 0., 0.]}, {'color': 'purple', 'position': [0.40, 0., 0.]}]
    ok_yellow, res_yellow = simulate_goal(custom, SimpleNamespace(color='yellow', dry_run=True), objects=objs)
    ok_blue, _ = simulate_goal(custom, SimpleNamespace(color='blue', dry_run=True), objects=objs)
    assert ok_yellow is True and res_yellow.verified_hold is True and ok_blue is False


def test_simulate_goal_tracks_stateful_arm_world_and_trace():
    stateful = GoalSpec(
        'stateful_check', MoveArm, '/z70/stateful_check', 10., lambda _s, _r: True,
        lambda ctx, req: (
            (ctx.deploy_arm(settle=False),
             ctx.update_world_object(1.0, 2.0, category='sock', mobility='pickupable', relevance='objective'),
             ctx.move_arm(req.m3_deg, req.m4_deg, req.m5_deg))[-1]['steps_executed'] == 2
            and ctx.arm()['joints']['m3'] == 170.0
            and len(ctx.world_objects()['objectives']) == 1,
            MoveArm.Result(outcome=ctx.outcome(True), steps_executed=len(ctx.trace)),
        ),
        fields=('m3_deg', 'm4_deg', 'm5_deg'),
    )
    trace = []
    ok, res = simulate_goal(stateful, SimpleNamespace(m3_deg=170., m4_deg=45., m5_deg=0.), _trace=trace)
    assert ok is True and res.steps_executed == 3
    assert [t['primitive'] for t in trace] == ['deploy_arm', 'update_world_object', 'move_arm']
    ok_move, move_res = simulate_goal(ALL_GOALS['move_arm'], SimpleNamespace(m3_deg=170., m4_deg=45., m5_deg=0.))
    assert ok_move is True and move_res.estimate.validity.value == 1 and list(move_res.estimate.joints.name) == ['m3_joint', 'm4_joint', 'm5_joint']
    ok_hand, hand_res = simulate_goal(ALL_GOALS['hand_step'], SimpleNamespace(direction=3))
    assert ok_hand is True and hand_res.estimate.validity.value == 1 and hand_res.estimate.joints.position[0] != math.radians(150. - 270.)
    ok_pulse, _ = simulate_goal(
        GoalSpec('pulse_rpc_check', MoveArm, '/z70/pulse_rpc_check', 10., lambda _s, _r: True,
                 lambda ctx, _r: (
                     ctx.pulse(velocity=0.1, omega=0.0, duration=500)
                     and abs(ctx.odom()['x'] - 0.05) < 1e-6
                     and ctx.select_camera('gripper')['camera'] == 'gripper'
                     and ctx.rpc('app_arm_out_compartment')
                     and ctx.rpc('app_grip', {'flag': 0})
                     and ctx.arm()['joints']['m6'] == 0.0
                     and ctx.rpc('app_arm_recover')
                     and ctx.arm_msg().validity.value == 3,
                     MoveArm.Result(),
                 )),
        SimpleNamespace(),
    )
    assert ok_pulse is True
    ok_ctx, _ = simulate_goal(
        GoalSpec('spatial_world_check', MoveArm, '/z70/spatial_world_check', 10., lambda _s, _r: True,
                 lambda ctx, _r: (
                     ctx.update_world_object(1.0, 2.0, category='sock', mobility='pickupable', relevance='objective')
                     and len(ctx.spatial_context()['world_objects']['objectives']) == 1
                     and len(ctx.garments('sock')) == 1
                     and ctx.mark_obstacle(1.0, 0.0, radius_m=0.2)
                     and ctx.corridor_safe(0.0, 0.0, 2.0, 0.0) is False
                     and ctx.plan_route(1.0, 0.0) is None
                     and len(ctx.plan_route(2.0, 0.0)) == 2
                     and ctx.deliver_garment('sock', 1.0, 0.0)['status_code'] == 'LOADED_ROUTE_BLOCKED'
                     and ctx.patrol_collect(1.0, 0.0)['status_code'] == 'TARGET_OUTSIDE_INTERIOR'
                     and ctx.navigate(1.0, 0.0)['status_code'] == 'TARGET_OUTSIDE_INTERIOR'
                     and ctx.patrol_collect(2.0, 2.0)['items_delivered'] == 1
                     and len(ctx.garments('sock')) == 0
                     and ctx.clear_world()['cleared'] is True
                     and ctx.corridor_safe(0.0, 0.0, 2.0, 0.0) is True,
                     MoveArm.Result(),
                 )),
        SimpleNamespace(),
    )
    assert ok_ctx is True
    ok_room, _ = simulate_goal(
        GoalSpec('room_nav_check', MoveArm, '/z70/room_nav_check', 10., lambda _s, _r: True,
                 lambda ctx, _r: (
                     ctx.navigate_room('Kitchen')['target_x_m'] == 4.2
                     and ctx.map_pose()[:2] == (4.2, 1.8)
                     and ctx.spatial_context()['robot_pose_map']['x'] == 4.2
                     and ctx.navigate_room('Balcony')['status_code'] == 'ROOM_UNREACHABLE'
                     and (ctx.deploy_arm(settle=False), ctx.navigate_room('Kitchen'))[-1]['status_code'] == 'ARM_NOT_STOWED',
                     MoveArm.Result(),
                 )),
        SimpleNamespace(),
        spatial_context={
            'rooms': [
                {'segment_id': 2, 'name': 'Kitchen', 'centroid': [4.0, 2.0], 'safe_waypoint': [4.2, 1.8], 'reachable': True},
                {'segment_id': 3, 'name': 'Balcony', 'centroid': [6.0, 2.0], 'safe_waypoint': None, 'reachable': False},
            ],
        },
    )
    assert ok_room is True
    ok_deliv, _ = simulate_goal(
        GoalSpec('deliver_sim_check', MoveArm, '/z70/deliver_sim_check', 10., lambda _s, _r: True,
                 lambda ctx, _r: (
                     ctx.plan_approach(0.0, 0.0) is None
                     and ctx.deliver_garment('sock', 2.5, 3.5)['status_code'] == 'DELIVERED'
                     and ctx.arm()['state'] == 'IN_ROBOT'
                     and ctx.map_pose()[:2] == (2.5, 3.5)
                     and ctx.pick('yellow', dry_run=False)['verified'] is True
                     and len(ctx.world_objects()['completed']) == 2,
                     MoveArm.Result(),
                 )),
        SimpleNamespace(),
    )
    assert ok_deliv is True
    ok_nav_extra, nav_res = simulate_goal(
        ALL_GOALS['pin_and_go'],
        SimpleNamespace(target_x_m=1.0, target_y_m=2.0, max_duration_s=30.0, dry_run=False),
        navigate={'arrived': True, 'status_code': 'ARRIVED', 'extra_field': 'ignored'},
    )
    assert ok_nav_extra is True and nav_res.arrived is True
    ok_nav_deployed, nav_dep_res = simulate_goal(
        ALL_GOALS['pin_and_go'],
        SimpleNamespace(target_x_m=1.0, target_y_m=2.0, max_duration_s=30.0, dry_run=False),
        arm={'state': 'WORKING_POS', 'joints': {'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.}},
    )
    assert ok_nav_deployed is False and nav_dep_res.status_code == 'ARM_NOT_STOWED' and nav_dep_res.outcome.error_code == 'ARM_NOT_STOWED'
    scene = [{'color': 'yellow', 'position': [0.35, 0.0, 0.03]}, {'color': 'red', 'position': [0.38, 0.04, 0.03]}]
    assert simulate_goal(ALL_GOALS['pick'], SimpleNamespace(color='yellow', dry_run=True), objects=scene)[0] is True
    ok_consume, _ = simulate_goal(
        GoalSpec('consume_pick_check', MoveArm, '/z70/consume_pick_check', 10., lambda _s, _r: True,
                 lambda ctx, _r: (
                     bool(ctx.update_world_object(0.35, 0.0, category='blue', mobility='pickupable', relevance='objective'))
                     and ctx.pick('blue', dry_run=False)['verified'] is False
                     and ctx.pick('blue', dry_run=False)['verified'] is False
                     and len(ctx.world_objects()['obstacles']) == 1
                     and ctx.pick('yellow', dry_run=False)['verified'] is True
                     and ctx.pick('yellow', dry_run=False)['verified'] is False
                     and len(ctx.objects('front', 'red')) == 1,
                     MoveArm.Result(),
                 )),
        SimpleNamespace(),
        objects=scene,
    )
    assert ok_consume is True
    assert simulate_goal(ALL_GOALS['pick'], SimpleNamespace(color='yellow', dry_run=True), objects=[{'color': 'yellow', 'position': [0.0, 0.0, 0.03]}])[0] is False
    assert simulate_goal(ALL_GOALS['pick'], SimpleNamespace(color='blue', dry_run=True), objects=scene)[0] is False
    assert simulate_goal(ALL_GOALS['stack'], SimpleNamespace(base_color='yellow', middle_color='red', top_color='blue', dry_run=True), objects=scene)[0] is False
    assert simulate_goal(ALL_GOALS['stack'], SimpleNamespace(base_color='yellow', middle_color='red', top_color='yellow', dry_run=True), objects=scene)[0] is False
    assert simulate_goal(ALL_GOALS['align_target'], SimpleNamespace(color='blue'), objects=scene)[0] is False
    assert simulate_goal(ALL_GOALS['align_target'], SimpleNamespace(color='yellow'), objects=[{'color': 'yellow', 'position': [0.55, 0.05, 0.03]}])[0] is True
    assert simulate_goal(ALL_GOALS['align_target'], SimpleNamespace(color='yellow'), objects=[{'color': 'yellow', 'position': [0.0, 0.0, 0.03]}])[1].outcome.error_code == 'INVALID_INPUT'
    assert simulate_goal(ALL_GOALS['grasp_and_lift'], SimpleNamespace(lift_steps=2, verify_color='yellow'), objects=scene)[1].verified_hold is True
    assert simulate_goal(ALL_GOALS['grasp_and_lift'], SimpleNamespace(lift_steps=2, verify_color='blue'), objects=scene)[1].verified_hold is False
    g_scene = [{'category': 'sock', 'transport_suitability': 1}, {'category': 'jeans', 'transport_suitability': 2}]
    assert simulate_goal(ALL_GOALS['deliver_garment'], SimpleNamespace(category='sock', drop_x_m=1.0, drop_y_m=1.0, max_duration_s=60.0, allow_towing=False, dry_run=True), garments=g_scene)[0] is True
    assert simulate_goal(ALL_GOALS['deliver_garment'], SimpleNamespace(category='towel', drop_x_m=1.0, drop_y_m=1.0, max_duration_s=60.0, allow_towing=False, dry_run=True), garments=g_scene)[1].status_code == 'NO_CONFIRMED_GARMENT'
    assert simulate_goal(ALL_GOALS['deliver_garment'], SimpleNamespace(category='jeans', drop_x_m=1.0, drop_y_m=1.0, max_duration_s=60.0, allow_towing=False, dry_run=True), garments=g_scene)[1].status_code == 'UNSUPPORTED_GARMENT_SKIP'
    assert simulate_goal(ALL_GOALS['deliver_garment'], SimpleNamespace(category='jeans', drop_x_m=1.0, drop_y_m=1.0, max_duration_s=60.0, allow_towing=True, dry_run=True), garments=g_scene)[1].status_code == 'TOWING_UNQUALIFIED_SKIP'
    assert simulate_goal(ALL_GOALS['deliver_garment'], SimpleNamespace(category='jeans', drop_x_m=1.0, drop_y_m=1.0, max_duration_s=60.0, allow_towing=True, dry_run=True), garments=g_scene, towing_qualified=True)[0] is True
    ok_skip_defer, _ = simulate_goal(
        GoalSpec('skip_defer_check', MoveArm, '/z70/skip_defer_check', 10., lambda _s, _r: True,
                 lambda ctx, _r: (
                     ctx.deliver_garment('jeans', 1.0, 1.0, dry_run=False)['status_code'] == 'UNSUPPORTED_GARMENT_SKIP'
                     and len(ctx.world_objects()['obstacles']) == 1,
                     MoveArm.Result(),
                 )),
        SimpleNamespace(),
        garments=[{'track_id': {'uuid': [1] * 16}, 'category': 'jeans', 'transport_suitability': 3, 'floor_contact_m': {'x': 0.5, 'y': -0.2}}],
    )
    assert ok_skip_defer is True
    assert simulate_goal(ALL_GOALS['deliver_garment'], SimpleNamespace(category='sock', drop_x_m=1.0, drop_y_m=1.0, max_duration_s=60.0, allow_towing=False, dry_run=True), plan_route=None)[1].status_code == 'LOADED_ROUTE_BLOCKED'
    assert simulate_goal(ALL_GOALS['pin_and_go'], SimpleNamespace(target_x_m=1.0, target_y_m=1.0, max_duration_s=60.0, dry_run=True), corridor_safe=False)[1].status_code == 'TARGET_OUTSIDE_INTERIOR'
    assert simulate_goal(ALL_GOALS['pin_and_go'], SimpleNamespace(target_x_m=1.0, target_y_m=1.0, max_duration_s=60.0, dry_run=False), status={'state': 3, 'error_code': 0, 'battery': 10.0, 'common_status': 51202})[1].status_code == 'ARM_NOT_STOWED'
    assert simulate_goal(ALL_GOALS['deliver_garment'], SimpleNamespace(category='sock', drop_x_m=1.0, drop_y_m=1.0, max_duration_s=60.0, allow_towing=False, dry_run=False), arm={'state': 'WORKING_POS', 'joints': {'m3': 150., 'm4': 45., 'm5': 0., 'm6': 90.}})[1].status_code == 'ARM_NOT_STOWED'
    assert simulate_goal(ALL_GOALS['patrol_collect'], SimpleNamespace(drop_x_m=1.0, drop_y_m=1.0, max_duration_s=60.0, max_items=5, allow_towing=False, dry_run=False), status={'state': 3, 'error_code': 0, 'battery': 10.0, 'common_status': 51202})[1].status_code == 'ARM_NOT_STOWED'
    assert simulate_goal(ALL_GOALS['pick'], SimpleNamespace(color='yellow', dry_run=False), status={'state': 3, 'error_code': 0, 'battery': 10.0, 'common_status': 51202})[0] is False
    ok_patrol, patrol_res = simulate_goal(ALL_GOALS['patrol_collect'], SimpleNamespace(drop_x_m=1.0, drop_y_m=1.0, max_duration_s=60.0, max_items=5, allow_towing=False, dry_run=False), garments=g_scene)
    assert ok_patrol is True and patrol_res.completed is False and patrol_res.partial_completion is True and patrol_res.items_delivered == 1 and patrol_res.items_skipped == 1 and patrol_res.status_code == 'PARTIAL_COMPLETION_SMALL_ITEMS_ONLY'
    ok_unsafe, unsafe_res = simulate_goal(
        ALL_GOALS['arm_step'],
        SimpleNamespace(joint=4, direction=-1),
        arm={'state': 'WORKING_POS', 'joints': {'m3': 150., 'm4': 35., 'm5': 0., 'm6': 90.}},
    )
    assert ok_unsafe is False and unsafe_res.outcome.error_code == 'INVALID_INPUT'
    with pytest.raises(ValueError, match='unsafe arm_step'):
        simulate_goal(
            GoalSpec('unsafe_step', MoveArm, '/z70/unsafe_step', 10., lambda _s, _r: True,
                     lambda ctx, _r: (bool((ctx.arm_step(0, 2, 0), ctx.arm_step(0, 2, 0))), MoveArm.Result())),
            SimpleNamespace(),
        )
    with pytest.raises(ValueError, match='hand_step direction'):
        simulate_goal(
            GoalSpec('bad_hand', MoveArm, '/z70/bad_hand', 10., lambda _s, _r: True,
                     lambda ctx, _r: (bool(ctx.hand_step(9)), MoveArm.Result())),
            SimpleNamespace(),
        )
    ok_rpc, _ = simulate_goal(
        GoalSpec('rpc_sim_check', MoveArm, '/z70/rpc_sim_check', 10., lambda _s, _r: True,
                 lambda ctx, _r: (
                     bool(ctx.rpc('app_start_program', {'cmd_ids': [
                         {'cmd_id': 'arm_out'},
                         {'cmd_id': 'arm_joints', 'value': [1, 0, 1, 2], 'step': 2},
                         {'cmd_id': 'arm_move', 'direction': 1, 'step': 1},
                     ]}))
                     and ctx.arm()['joints']['m5'] == 20.
                     and ctx.arm()['joints']['m6'] == 0.
                     and bool(ctx.rpc('app_arm_move', {'m': [0, 1, 0, 1, 0]}))
                     and ctx.arm()['joints']['m6'] == 90.
                     and (ctx.fit_wrist_joints() or {}).get('joints') == [ctx.arm()['joints']['m3'], ctx.arm()['joints']['m4'], ctx.arm()['joints']['m5']]
                     and 'hand_pose_base' in (ctx.wrist_pose() or {})
                     and bool(ctx.rpc('app_arm_direction_move', {'direction': 2}))
                     and bool(ctx.rpc('app_goto_target', [1500, 0]))
                     and ctx.map_pose()[:2] == (1.5, 0.0)
                     and bool(ctx.rpc('app_rc_move', [{'velocity': 0.5, 'omega': 0.0, 'duration': 1000}]))
                     and ctx.map_pose()[:2] == pytest.approx((2.0, 0.0))
                     and bool(ctx.rpc('app_start_program', {'cmd_ids': [{'cmd_id': 'arm_in'}]}))
                     and ctx.arm()['state'] == 'IN_ROBOT',
                     MoveArm.Result(),
                 )),
        SimpleNamespace(),
    )
    assert ok_rpc is True


def test_held_front_rejects_displaced_floor_target_bridged_contact_and_ambiguous_blobs(fake, tmp_path, monkeypatch):
    origin, token, _, state = fake
    state.update(hide_marker=True, deployed=True, gripped=True, lift=.08, joints=list(planner.GRASP))
    client = Client(origin, token)
    ex = Executor(client, observer(client), tmp_path / 'held-rejections', sleep=lambda _: None)
    ex.pregrip_jaw = planner.measured_point(planner.GRASP, planner._CALIBRATION['effective_jaw_from_elbow_m'])

    # 1. Target nudged on floor beyond 5.5 cm (within TRACK_GATE_M = 12 cm) is still floor-supported.
    displaced_spot = [state['floor_point'][0], state['floor_point'][1] + .08, 0.]
    orig_front_objects = perception.front_objects
    monkeypatch.setattr(perception, 'front_objects', lambda _img, *a, **kw: [dict(color='yellow', position=displaced_spot)])
    assert not ex.held_front('yellow', state['floor_point'])
    assert ex.last_floor_gone is False

    # 2. Bridged/tall contour with elevated centroid but floor-supported bottom contact (bottom_z <= 0.02 m) is rejected.
    cam = perception.front_camera()
    jaw_x = ex.pregrip_jaw[0]
    top_uv = cam.floor_to_pixel((jaw_x - .275) * 1000, ex.pregrip_jaw[1] * 1000, 120.)
    bot_uv = cam.floor_to_pixel((jaw_x - .275) * 1000, ex.pregrip_jaw[1] * 1000, 5.)
    mid_uv = [top_uv[0], (top_uv[1] + bot_uv[1]) / 2]
    monkeypatch.setattr(perception, 'detect', lambda *_a, **_kw: [
        dict(color='yellow', pixel=mid_uv, contact_px=list(bot_uv), bbox=[750, 450, 30, 220], size_px=220., area_px=4000., image_size=[1600, 1200]),
    ])
    monkeypatch.setattr(perception, 'front_objects', lambda *_a, **_kw: [])
    assert not ex.held_front('yellow', state['floor_point'])

    # 3. Multiple same-color candidates in the jaw association gate are rejected as ambiguous.
    lifted_uv = cam.floor_to_pixel((jaw_x - .275) * 1000, ex.pregrip_jaw[1] * 1000, 110.)
    lifted_bot = cam.floor_to_pixel((jaw_x - .275) * 1000, ex.pregrip_jaw[1] * 1000, 80.)
    monkeypatch.setattr(perception, 'detect', lambda *_a, **_kw: [
        dict(color='yellow', pixel=list(lifted_uv), contact_px=list(lifted_bot), bbox=[760, 460, 40, 40], size_px=40., area_px=1200., image_size=[1600, 1200]),
        dict(color='yellow', pixel=[lifted_uv[0] + 8., lifted_uv[1] + 4.], contact_px=list(lifted_bot), bbox=[770, 464, 40, 40], size_px=40., area_px=1100., image_size=[1600, 1200]),
    ])
    assert not ex.held_front('yellow', state['floor_point'])
    monkeypatch.setattr(perception, 'front_objects', orig_front_objects)


def test_pick_loss_of_hold_after_initial_verify_returns_unverified_and_stows(fake, tmp_path):
    origin, token, _, state = fake
    state['hide_marker'] = True
    client = Client(origin, token)
    ex = Executor(client, observer(client), tmp_path / 'slipped-pick', sleep=lambda _: None)
    real_held_front = ex.held_front
    checks = []

    def slip_on_second_check(color, spot):
        if len(checks) == 1:
            state['missed_grip'] = True
        ok = real_held_front(color, spot)
        checks.append(ok)
        return ok

    ex.held_front = slip_on_second_check
    pick_res = ex.pick('yellow')
    assert checks == [True, False]
    assert pick_res['verified'] is False
    assert state['arm_state'] == 'IN_ROBOT' and ex.deployed is False and ex.gripped is False
    timeline = [json.loads(line) for line in (tmp_path / 'slipped-pick/timeline.jsonl').read_text().splitlines()]
    assert any(r['event'] == 'hold_start' and r['initial_held'] is True for r in timeline)
    assert any(r['event'] == 'hold_10s' and r['held'] is False and r['final_held'] is False for r in timeline)


def test_goal_outcome_distinguishes_rpc_ack_from_unverified_physical_hold(fake, tmp_path):
    origin, token, _, state = fake
    state.update(hide_marker=True, missed_grip=True)
    client = Client(origin, token)
    server = SimpleNamespace(
        observations=client.observations, gains={}, odom=lambda: None,
        thresholds={'yellow': {}}, hazard=lambda: None,
        _current_arm_msg=lambda: SimpleNamespace(),
    )
    from z70_interfaces.msg import ArmEstimate
    server._current_arm_msg = lambda: ArmEstimate()
    handle = SimpleNamespace(request=SimpleNamespace(color='yellow', dry_run=False), is_cancel_requested=False, publish_feedback=lambda _: None)
    ctx = GoalContext(server, handle, Pick, client, tmp_path / 'ctx-pick')
    ctx.executor.sleep = lambda _: None
    ok_pick, res_pick = BUILTIN_GOALS['pick'].run(ctx, handle.request)
    assert client.commands.latest_outcome.response == 1
    assert ok_pick is False and res_pick.verified_hold is False and res_pick.outcome.error_code == 'OBSERVATION_GAP'

    state.update(deployed=True, arm_state='WORKING_POS', joints=list(planner.GRASP))
    lift_req = SimpleNamespace(lift_steps=1, verify_color='yellow')
    ctx_lift = GoalContext(server, SimpleNamespace(request=lift_req, is_cancel_requested=False, publish_feedback=lambda _: None),
                           GraspAndLift, client, tmp_path / 'ctx-lift')
    ctx_lift.executor.sleep = lambda _: None
    ok_verify, res_verify = BUILTIN_GOALS['grasp_and_lift'].run(ctx_lift, lift_req)
    assert ok_verify is False and res_verify.verified_hold is False and res_verify.outcome.error_code == 'OBSERVATION_GAP'
    unverified_req = SimpleNamespace(lift_steps=1, verify_color='')
    ok_unverified, res_unverified = BUILTIN_GOALS['grasp_and_lift'].run(ctx_lift, unverified_req)
    assert ok_unverified is True and res_unverified.outcome.error_code == 'NONE'


def test_supported_place_rejects_missing_or_unwithdrawn_jaw_and_unsent_release(tmp_path):
    # 1. Missing jaw.position geometry cannot silently satisfy withdrawal.
    ex_missing = Executor(MemoryClient(), lambda *_: [], tmp_path / 'missing-jaw', sleep=lambda _: None)
    ex_missing.gripped = True
    no_pos = placement_sample(.001, separation=.075)
    no_pos['jaw'] = dict(hand_pose_base=np.eye(4).tolist())
    samples_missing = iter([placement_sample(.002), placement_sample(.002), no_pos])
    ex_missing.placement_sample = lambda *a, **kw: next(samples_missing)
    res_missing = ex_missing.supported_place('purple', 'blue', 'blue', [.5, 0., 0.], tier=2)
    assert not res_missing['verified'] and not res_missing['detached'] and ex_missing.gripped is True

    # 2. Contact appearing valid before hand withdrawal (jaw Z not raised >= 15 mm) is rejected even if 3D separation is large.
    ex_unwithdrawn = Executor(MemoryClient(), lambda *_: [], tmp_path / 'pre-withdrawal', sleep=lambda _: None)
    ex_unwithdrawn.gripped = True
    samples_unwithdrawn = iter([
        placement_sample(.002, separation=.02, jaw_z=.120),
        placement_sample(.002, separation=.02, jaw_z=.120),
        placement_sample(.001, separation=.060, jaw_z=.125),
    ])
    ex_unwithdrawn.placement_sample = lambda *a, **kw: next(samples_unwithdrawn)
    res_unwithdrawn = ex_unwithdrawn.supported_place('purple', 'blue', 'blue', [.5, 0., 0.], tier=2)
    assert not res_unwithdrawn['verified'] and not res_unwithdrawn['detached'] and ex_unwithdrawn.gripped is True

    # 3. Definite release non-dispatch (RequestException) aborts before withdrawing the closed hand.
    client_fail = MemoryClient()
    ex_fail = Executor(client_fail, lambda *_: [], tmp_path / 'release-not-sent', sleep=lambda _: None)
    ex_fail.gripped = True
    ex_fail.placement_sample = lambda *a, **kw: placement_sample(.002)
    ex_fail.open_jaws = lambda: (_ for _ in ()).throw(requests.RequestException('put_down not sent'))
    res_fail = ex_fail.supported_place('purple', 'blue', 'blue', [.5, 0., 0.], tier=2)
    assert not res_fail['verified'] and ex_fail.gripped is True
    assert all(m != 'app_arm_direction_move' for m, _ in client_fail.calls)


def test_grasp_and_lift_rejects_not_sent_grip_and_lift_while_preserving_unknown_timeout_observation(fake, tmp_path, monkeypatch):
    origin, token, events, state = fake
    state.update(deployed=True, arm_state='WORKING_POS', joints=list(planner.GRASP), hide_marker=True)
    client = Client(origin, token)

    # 1. app_grip dispatch=not_sent fails grasp_and_lift, leaves gripped=False, never lifts, and fails GraspAndLift goal.
    orig_rpc = client.commands.rpc

    def not_sent_grip(method, params, timeout_ms):
        events.append(('POST', '/api/v1/rpc', dict(method=method, params=params)))
        if method == 'app_grip':
            return {'dispatch': 'not_sent', 'response': 'transport_error', 'transportReason': 'link_unavailable'}
        return orig_rpc(method, params, timeout_ms)

    monkeypatch.setattr(client.commands, 'rpc', not_sent_grip)
    ex = Executor(client, observer(client), tmp_path / 'grip-not-sent', sleep=lambda _: None)
    with pytest.raises(requests.RequestException):
        ex.grasp_and_lift(lift_steps=2, verify_color='')
    assert ex.gripped is False
    assert not any(e[2].get('method') == 'app_arm_direction_move' for e in events if e[1] == '/api/v1/rpc')

    from z70_interfaces.msg import ArmEstimate
    server = SimpleNamespace(
        observations=client.observations, gains={}, odom=lambda: None,
        thresholds={'yellow': {}}, hazard=lambda: None,
        _current_arm_msg=lambda: ArmEstimate(),
    )
    req0 = SimpleNamespace(lift_steps=0, verify_color='')
    ctx0 = GoalContext(server, SimpleNamespace(request=req0, is_cancel_requested=False, publish_feedback=lambda _: None),
                       GraspAndLift, client, tmp_path / 'goal-grip-not-sent')
    ctx0.executor.sleep = lambda _: None
    ok0, res0 = BUILTIN_GOALS['grasp_and_lift'].run(ctx0, req0)
    assert ok0 is False and res0.verified_hold is False and res0.outcome.error_code != 'NONE'
    assert ctx0.executor.gripped is False

    # 2. app_arm_direction_move dispatch=not_sent fails grasp_and_lift on the unsent step without continuing.
    events.clear()

    def not_sent_lift(method, params, timeout_ms):
        if method == 'app_arm_direction_move':
            events.append(('POST', '/api/v1/rpc', dict(method=method, params=params)))
            return {'dispatch': 'not_sent', 'response': 'transport_error', 'transportReason': 'link_unavailable'}
        return orig_rpc(method, params, timeout_ms)

    monkeypatch.setattr(client.commands, 'rpc', not_sent_lift)
    ex_lift = Executor(client, observer(client), tmp_path / 'lift-not-sent', sleep=lambda _: None)
    with pytest.raises(requests.RequestException):
        ex_lift.grasp_and_lift(lift_steps=2, verify_color='')
    assert ex_lift.gripped is True
    assert sum(1 for e in events if e[1] == '/api/v1/rpc' and e[2].get('method') == 'app_arm_direction_move') == 1

    # 3. Unknown/sent-timeout dispatch still observes without blind replay.
    events.clear()

    def sent_timeout_lift(method, params, timeout_ms):
        res = orig_rpc(method, params, timeout_ms)
        if method == 'app_arm_direction_move':
            return {'dispatch': 'sent', 'response': 'transport_error', 'transportReason': 'timeout'}
        return res

    monkeypatch.setattr(client.commands, 'rpc', sent_timeout_lift)
    ex_obs = Executor(client, observer(client), tmp_path / 'lift-unknown-observe', sleep=lambda _: None)
    res_obs = ex_obs.grasp_and_lift(lift_steps=2, verify_color='yellow', floor_spot=state['floor_point'])
    assert res_obs['gripped'] is True and res_obs['verified'] is True
    assert sum(1 for e in events if e[1] == '/api/v1/rpc' and e[2].get('method') == 'app_arm_direction_move') == 2


def test_put_down_and_stow_require_observed_common_status_and_preserve_cleanup_state(fake, tmp_path, monkeypatch):
    origin, token, events, state = fake
    state.update(deployed=True, gripped=True, arm_state='IN_PROCESS_POS', grab_mode=1)
    client = Client(origin, token)
    orig_rpc = client.commands.rpc

    # 1. Unknown app_put_down with unchanged deployed status (3 << 6) fails closed, keeps gripped=True, and never stows.
    def unknown_put_down(method, params, timeout_ms):
        events.append(('POST', '/api/v1/rpc', dict(method=method, params=params)))
        if method == 'app_put_down':
            return {'dispatch': 'unknown', 'response': 'transport_error', 'transportReason': 'timeout'}
        return orig_rpc(method, params, timeout_ms)

    monkeypatch.setattr(client.commands, 'rpc', unknown_put_down)
    ex_unk = Executor(client, observer(client), tmp_path / 'put-down-unknown', sleep=lambda _: None)
    ex_unk.deployed = True
    ex_unk.gripped = True
    with pytest.raises(LookupError, match='put_down outcome unknown'):
        ex_unk.release_and_stow(put_down=True)
    assert ex_unk.gripped is True and ex_unk.deployed is True
    assert sum(1 for e in events if e[1] == '/api/v1/rpc' and e[2].get('method') == 'app_put_down') == 1
    assert not any(e[2].get('method') == 'app_arm_in_compartment' for e in events if e[1] == '/api/v1/rpc')

    # Acknowledged app_put_down with steady status (3 << 6) succeeds normally and clears gripped=False.
    monkeypatch.setattr(client.commands, 'rpc', orig_rpc)
    ex_ack = Executor(client, observer(client), tmp_path / 'put-down-ack', sleep=lambda _: None)
    ex_ack.deployed = True
    ex_ack.gripped = True
    ex_ack.put_down()
    assert ex_ack.gripped is False

    # 2. put_down() with missing common_status fails closed and preserves gripped=True.
    client_pd = MemoryClient()
    client_pd.status = lambda: {}
    ex_pd = Executor(client_pd, lambda *_: [], tmp_path / 'put-down-missing', sleep=lambda _: None)
    ex_pd.gripped = True
    with pytest.raises(LookupError, match='put_down readiness not observed'):
        ex_pd.put_down()
    assert ex_pd.gripped is True

    # 3. stow() with missing common_status ({}) fails closed without replaying app_arm_in_compartment and preserves deployed=True.
    client_missing = MemoryClient()
    client_missing.status = lambda: {}
    client_missing.arm_estimate = lambda: {'state': 'IN_ROBOT', 'joints': {}}
    ex_missing = Executor(client_missing, lambda *_: [], tmp_path / 'stow-missing', sleep=lambda _: None)
    ex_missing.deployed = True
    with pytest.raises(LookupError, match='stow dispatched but estimate not reconciled'):
        ex_missing.stow()
    assert ex_missing.deployed is True
    assert client_missing.calls == [('app_arm_in_compartment', {})]

    # 4. stow() with busy common_status (3 << 6) fails closed without replaying and dispatches app_stop_grasp on emergency stop.
    client_busy = MemoryClient()
    client_busy.status = lambda: {'common_status': 3 << 6}
    client_busy.arm_estimate = lambda: {'state': 'IN_ROBOT', 'joints': {}}
    ex_busy = Executor(client_busy, lambda *_: [], tmp_path / 'stow-busy', sleep=lambda _: None)
    ex_busy.deployed = True
    with pytest.raises(LookupError, match='stow dispatched but estimate not reconciled'):
        ex_busy.stow()
    assert ex_busy.deployed is True
    assert client_busy.calls == [('app_arm_in_compartment', {})]
    ex_busy._emergency_stop(LookupError('stow timeout'))
    assert ('app_stop_grasp', []) in client_busy.calls

    # 5. stow() interrupted by cancellation/hazard raises SafetyStop while preserving deployed=True for emergency stop.
    client_cancel = MemoryClient()
    client_cancel.status = lambda: {'common_status': 3 << 6}
    client_cancel.arm_estimate = lambda: {'state': 'IN_ROBOT', 'joints': {}}
    ex_cancel = Executor(client_cancel, lambda *_: [], tmp_path / 'stow-cancel', sleep=lambda _: None, hazard=lambda: 'operator cancelled task')
    ex_cancel.deployed = True
    with pytest.raises(RuntimeError, match='operator cancelled task'):
        ex_cancel.stow()
    assert ex_cancel.deployed is True
    ex_cancel._emergency_stop('operator cancelled task')
    assert ('app_stop_grasp', []) in client_cancel.calls


def _nav_room(pose=(1., 1., 0.)):
    pixels = np.full((80, 80), 15, np.uint8)
    pixels[[0, -1], :] = 1
    pixels[:, [0, -1]] = 1
    occupancy = np.where(pixels == 1, 100, 0).astype(np.int8)
    from z70_mapping.node import VendorMap
    return VendorMap(80, 80, (0., 0.), pixels, occupancy, pose, None, {1: {}}, {}, {})


class _NavObs:
    def __init__(self):
        self.labels = {1: 'Kitchen'}
        self.refreshes = []
        self.reset()

    def reset(self, poses=None, states=None):
        self.poses = iter(poses or [(1., 1., 0.), (1.3, 1., 0.), (2., 1., 0.), (2., 1., 0.), (2., 1., 0.)])
        self.states = iter(states or [3, 3, 16, 3])
        self.sequence = 0
        self.refreshes = []

    def get(self, path):
        if path == '/api/arm/estimate':
            return {'state': 'IN_ROBOT'}
        return {'telemetry': {'status': {'value': {
            'common_status': 0, 'error_code': 0, 'battery': 80,
            'in_cleaning': False, 'in_returning': False, 'state': next(self.states, 3),
        }}}}

    def navigation_snapshot(self, refresh=False, max_age_s=5.):
        self.refreshes.append(bool(refresh))
        self.sequence += 1
        pose = next(self.poses, (2., 1., 0.))
        return SimpleNamespace(
            map=_nav_room(pose), labels=dict(self.labels), sequence=self.sequence,
            map_index=1, map_sequence=1, active_floor_verified=True, current_pose_verified=True,
            cache_age_s=0., evidence=None,
        )


class _NavCmd:
    def __init__(self):
        self.calls = []
        self.stops = 0
        self.latest_outcome = None
        self.cancelled = threading.Event()

    def rpc(self, method, params, timeout_ms=5000):
        self.calls.append((method, list(params)))
        return {'dispatch': 'sent', 'response': 'received'}

    def stop(self):
        self.stops += 1


def test_navigate_honors_parent_lifecycle_and_room_preview_is_read_only(tmp_path, monkeypatch):
    from z70_manipulation.garment_delivery import PinAndGoNavigator
    from z70_manipulation.goals import GoalContext as GoalContextCls
    from z70_mapping.boundary import BoundaryConfig

    seen = []

    class RecordingNavigator(PinAndGoNavigator):
        def execute(self, goal):
            seen.append((goal.max_duration_s, self.hazard))
            return super().execute(goal)

    monkeypatch.setattr('z70_manipulation.goals.PinAndGoNavigator', RecordingNavigator)
    obs, commands, feedback = _NavObs(), _NavCmd(), []
    server = SimpleNamespace(
        observations=obs, navigation_config=BoundaryConfig(), gains={},
        odom=lambda: {'x': 1., 'y': 1., 'yaw': 0.}, safety_stop=False, deadline=None,
        hazard=lambda: None,
    )
    handle = SimpleNamespace(request=SimpleNamespace(), is_cancel_requested=False, publish_feedback=feedback.append)
    ctx = GoalContext(server, handle, PinAndGo, SimpleNamespace(commands=commands), tmp_path, create_executor=False)

    arrived = ctx.navigate(2., 1., max_duration_s=4.)
    assert arrived['status_code'] == 'ARRIVED' and arrived['arrived'] is True
    assert commands.calls == [('app_goto_target', [2000, 1000])]
    assert seen[-1][0] == 4. and seen[-1][1].__func__ is GoalContextCls.hazard and seen[-1][1].__self__ is ctx
    assert feedback and feedback[-1].reason == 'MONITORING'
    assert ctx.commands is commands

    seen.clear(); commands.calls.clear(); feedback.clear(); obs.reset()
    rejected = ctx.navigate(2., 1., max_duration_s=0.4)
    assert rejected['status_code'] == 'INVALID_INPUT' and commands.calls == [] and seen[-1][0] == 0.4

    seen.clear(); commands.calls.clear(); obs.reset()
    server.deadline = time.monotonic() + 4.
    capped = ctx.navigate(2., 1., max_duration_s=40.)
    assert capped['status_code'] == 'ARRIVED' and commands.calls == [('app_goto_target', [2000, 1000])]
    assert 1. <= seen[-1][0] < 4.05 and seen[-1][0] < 40.

    seen.clear(); commands.calls.clear(); obs.reset()
    server.deadline = time.monotonic() + 0.4
    short = ctx.navigate(2., 1., max_duration_s=40.)
    assert short['status_code'] == 'TIMED_OUT' and short['status_code'] != 'INVALID_INPUT'
    assert short['arrived'] is False and commands.calls == [] and seen == []

    server.deadline = time.monotonic() - 1.
    expired = ctx.navigate(2., 1., max_duration_s=40.)
    assert expired['status_code'] == 'TIMED_OUT' and commands.calls == [] and seen == []

    seen.clear(); commands.calls.clear(); obs.reset()
    server.deadline = None
    handle.is_cancel_requested = True
    cancelled = ctx.navigate(2., 1., max_duration_s=4.)
    assert cancelled['status_code'] == 'CANCELLED' and commands.calls == []

    seen.clear(); commands.calls.clear(); obs.reset()
    handle.is_cancel_requested = False
    server.safety_stop = False
    server.hazard = lambda: 'task revoked'
    aborted = ctx.navigate(2., 1., max_duration_s=4.)
    assert aborted['status_code'] == 'BOUNDARY_ABORT' and commands.calls == [] and server.safety_stop is False

    commands.calls.clear(); obs.reset(); obs.labels = {1: 'Kitchen'}
    server.hazard = lambda: None
    preview = ctx.navigate_room('kitchen', dry_run=True)
    assert preview['dry_run_valid'] is True and preview['room'] == 'Kitchen'
    assert obs.refreshes and all(flag is False for flag in obs.refreshes)
    assert commands.calls == []

    obs.reset(); obs.labels = {}
    missing = ctx.navigate_room('kitchen')
    assert missing['status_code'] == 'ROOM_UNREACHABLE' and obs.refreshes == [True] and commands.calls == []


@pytest.mark.parametrize('ending,expected', [
    ('deadline', 'TIMED_OUT'), ('cancel', 'CANCELLED'), ('safety_stop', 'SAFETY_STOP'),
    ('step_deadline', 'TIMED_OUT'), ('boundary', 'BOUNDARY_ABORT'), ('unknown_dispatch', 'UNKNOWN_DISPATCH'),
])
def test_mission_navigation_preserves_parent_terminal_cause(tmp_path, monkeypatch, ending, expected):
    from z70_interfaces.action import RunMission
    from z70_manipulation.garment_delivery import PinAndGoNavigator
    from z70_mapping.boundary import BoundaryConfig

    now = [100.]
    class NavCommands(_NavCmd):
        def rpc(self, method, params, timeout_ms=5000):
            reply = super().rpc(method, params, timeout_ms)
            return {'dispatch': 'unknown', 'response': 'transport_error'} if ending == 'unknown_dispatch' and method == 'app_goto_target' else reply

        def stop(self):
            super().stop()
            self.cancelled.set()  # A child stop also revokes the shared command client.

    commands, feedback, child_codes = NavCommands(), [], []
    navigation = {'op': 'navigate', 'target_x_m': 2., 'target_y_m': 1.}
    if ending == 'step_deadline':
        navigation['max_duration_s'] = 1.
    request = RunMission.Goal(
        steps_json=json.dumps([navigation, {'op': 'pick', 'color': 'yellow'}]),
        max_duration_s=5., dry_run=False,
    )
    handle = SimpleNamespace(request=request, is_cancel_requested=False, publish_feedback=feedback.append)
    client = SimpleNamespace(commands=commands)

    class SlowPreflight(_NavObs):
        def navigation_snapshot(self, *args, **kwargs):
            if self.sequence == 0:
                # The native navigation duration starts after map preflight; the parent budget starts before it.
                now[0] += 2.
            return super().navigation_snapshot(*args, **kwargs)

    obs = SlowPreflight()
    poses = [(1., 1., 0.), (50., 50., 0.)] if ending == 'boundary' else [(1., 1., 0.)] * 20
    obs.reset(poses=poses, states=[3, 3] + [16] * 20)
    server = SimpleNamespace(
        observations=obs, navigation_config=BoundaryConfig(), gains={}, thresholds={'yellow': {}},
        odom=lambda: {'x': 1., 'y': 1., 'yaw': 0.}, safety_stop=False, deadline=105.,
        active_handle=handle, client=client,
    )
    server.hazard = lambda: PickServer.hazard(server)

    def advance(_duration):
        now[0] += 1.
        if ending == 'deadline' and now[0] >= server.deadline:
            commands.cancelled.set()  # Same flag the parent watchdog sets on expiry.
        elif ending == 'cancel':
            handle.is_cancel_requested = True
            commands.cancelled.set()
        elif ending == 'safety_stop':
            server.safety_stop = True
            commands.cancelled.set()

    class TimedNavigator(PinAndGoNavigator):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs, clock=lambda: now[0], sleep=advance)

        def execute(self, goal):
            result = super().execute(goal)
            child_codes.append(result['status_code'])
            return result

    monkeypatch.setattr('z70_manipulation.goals.time.monotonic', lambda: now[0])
    monkeypatch.setattr('z70_manipulation.goals.PinAndGoNavigator', TimedNavigator)
    ctx = GoalContext(server, handle, RunMission, client, tmp_path, create_executor=False)
    ctx.clock = lambda: now[0]
    ctx.pick = lambda *a, **kw: pytest.fail('later pick must not run after parent termination')
    original_hazard = ctx.hazard
    ok, result = ALL_GOALS['mission'].run(ctx, request)
    assert not ok and result.status_code == expected, child_codes
    assert result.outcome.error_code == expected
    assert result.failed_step == 'navigate' and result.steps_completed == 0 and not result.partial_completion
    assert json.loads(result.step_results_json)[0]['status_code'] == expected
    assert [method for method, _ in commands.calls] == ['app_goto_target', 'stop_goto_target']
    assert commands.stops == 1 and ctx.commands is commands
    assert ctx.hazard == original_hazard and server.deadline == 105.
