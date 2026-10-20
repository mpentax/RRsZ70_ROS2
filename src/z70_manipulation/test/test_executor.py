import math
import json
from pathlib import Path
import threading
import cv2
import numpy as np
import pytest
from z70_manipulation import planner, perception
from z70_manipulation.executor import Client, Executor

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

    monkeypatch.setattr('z70_manipulation.observations.Observations', FakeObservations)
    monkeypatch.setattr('z70_manipulation.node.Observations', FakeObservations)

    class FakeCommands:
        def __init__(self, *args):
            self.cancelled = threading.Event()
            self.latest_outcome = None

        def rpc(self, method, params, timeout_ms):
            return handle_rpc(dict(method=method, params=params))

        def stop(self):
            for method in ('app_rc_stop', 'app_rc_end', 'app_stop_grasp'):
                self.rpc(method, [], 5000)

        def begin(self):
            pass

        def end(self):
            pass

        def close(self):
            pass

    monkeypatch.setattr('z70_manipulation.commands.Commands', FakeCommands)
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
                    b['position'] = [g[0], planner.MOUNT[1] - (b['pixel'][0] - w / 2) / (843. * w / 1600) * .3, 0.]
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


def placement_sample(gap, lateral=(0.,0.), separation=.03):
    return dict(gap_m=gap, lateral_error_m=list(lateral), held_center_m=[.5,0.,.10],
                held_bottom_m=[.5,0.,.066+gap], support_top_m=[.5,0.,.066],
                tower_spot=[.5,0.,0.], jaw_separation_m=separation,
                jaw=dict(hand_pose_base=np.eye(4).tolist()), uncertainty_m=.015)


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


def test_placement_post_command_frames_reject_earlier_receipts(tmp_path,monkeypatch):
    from z70_manipulation import placement
    client=MemoryClient()
    ex=Executor(client,lambda *_:[],tmp_path/'timing',sleep=lambda _:None)
    def old_frame():
        client.frame_received_at=9.
        return np.zeros((10,10,3),np.uint8),'front'
    ex.fresh_frame=old_frame
    monkeypatch.setattr(placement,'measure',lambda *a,**kw:pytest.fail('old frame used'))
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
    from pathlib import Path
    from types import SimpleNamespace
    from z70_manipulation.goals import ALL_GOALS, BUILTIN_GOALS, GoalContext, GoalSpec, simulate_goal

    assert len(ALL_GOALS) == 14
    cases = [
        ('pick', dict(color='yellow', dry_run=False), lambda r: r.verified_hold),
        ('stack', dict(base_color='blue', middle_color='purple', top_color='yellow', dry_run=False), lambda r: r.stack_verified and r.tiers_completed == 3),
        ('pin_and_go', dict(target_x_m=1., target_y_m=2., max_duration_s=60., dry_run=False), lambda r: r.arrived),
        ('deliver_garment', dict(category='sock', drop_x_m=1., drop_y_m=2., max_duration_s=60., allow_towing=False, dry_run=False), lambda r: r.delivered),
        ('patrol_collect', dict(drop_x_m=1., drop_y_m=2., max_duration_s=60., max_items=3, allow_towing=False, dry_run=False), lambda r: r.completed),
        ('deploy_arm', dict(settle=True), lambda r: r.ready),
        ('align_target', dict(color='yellow'), lambda r: r.aligned),
        ('move_arm', dict(m3_deg=150., m4_deg=45., m5_deg=0.), lambda r: r.steps_executed == 1),
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
        ok, res = simulate_goal(spec, SimpleNamespace(**payload))
        assert ok and check(res), name

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
                                  and ctx.plan_grasp(.43, 0.)['state'] == [250, 95]
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



