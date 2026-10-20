"""Offline real DDS + fake wire backend. No access to selected robot or private files."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
import uuid

import pytest
import requests
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.action import ActionServer, CancelResponse
from z70_driver.driver import Driver
from z70_interfaces.action import Pick
from z70_interfaces.srv import RawRpc
from z70_interfaces.msg import FrameMetadata
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from z70_manipulation.observations import LATCHED
from z70_web_gateway.gateway import Gateway, HttpServer, ApiError, goal_message, read_token

TOKEN = 'f'*43  # Synthetic fixture token, not a credential.


def wait(predicate, seconds=5):
    deadline = time.monotonic() + seconds
    while not (result := predicate()):
        assert time.monotonic() < deadline, 'condition deadline'
        time.sleep(.02)
    return result


@pytest.fixture
def system():
    state = dict(calls=[], hang=False, unknown=False, reject=False, release=threading.Event(), shutdown=threading.Event())
    class Backend(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def reply(self, body, status=200):
            content = json.dumps(body).encode()
            try:
                self.send_response(status); self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(content))); self.end_headers(); self.wfile.write(content)
            except OSError: pass
        def do_GET(self):
            if self.path == '/api/commands/events':
                self.send_response(200); self.end_headers()
                while not state['shutdown'].wait(.1):
                    try: self.wfile.write(b'\n'); self.wfile.flush()
                    except OSError: return
                return
            if self.path == '/api/state':
                return self.reply(dict(observationSession='fake-phase4', connection=dict(state='connected'),
                    telemetry=dict(status=dict(value=dict(state=39, error_code=0, battery=75), observedAt=int(time.time()*1000)))))
            self.reply({}, 404)
        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            if self.path == '/api/camera/frame':
                state['frames_requested'] = state.get('frames_requested', 0) + 1
                if state.get('hang_frame'): state['release'].wait(15)
                return self.reply({}, 404)
            if self.path == '/api/driver/control/session':
                return self.reply(dict(enabled=body['enabled'], streams_released=not body['enabled']))
            if self.path != '/api/driver/rpc': return self.reply({}, 404)
            method = body['method']; state['calls'].append(method)
            if method == 'app_arm_move' and state['hang']: state['release'].wait(15)
            unknown = state['unknown'] and method == 'app_arm_move'
            reject = state['reject'] and method == 'app_arm_move'
            if method == 'stop_camera_preview':
                assert body['params'] == {'client_id':'676f32727463'}
                state['preview_active'] = False
            value = (dict(status=1 if state.get('preview_active', True) else 0,
                          client_id=state.get('preview_owner', '676f32727463') if state.get('preview_active', True) else 'none')
                     if method == 'get_homesec_connect_status' else ['ok'])
            self.reply(dict(commandId=body['commandId'], attemptId=str(uuid.uuid4()), attemptIndex=0,
                dispatch='unknown' if unknown else 'sent', response='transport_error' if unknown else 'vendor_error' if reject else 'received',
                result=value, transportReason='timeout' if unknown else None, startedAt=int(time.time()*1000), finishedAt=int(time.time()*1000)),
                502 if unknown or reject else 200)
    backend = ThreadingHTTPServer(('127.0.0.1', 0), Backend); backend.daemon_threads = True
    bt = threading.Thread(target=backend.serve_forever); bt.start()
    rclpy.init(domain_id=94)
    driver = Driver([Parameter('origin', value=f'http://127.0.0.1:{backend.server_port}'),
        Parameter('camera_hz', value=0.), Parameter('map_refresh_s', value=0.),
        Parameter('state_hz', value=5.), Parameter('arm_estimate_path', value='')])
    gateway = Gateway(); peer = rclpy.create_node('phase4_fake_observations')
    executor = MultiThreadedExecutor(num_threads=10)
    for node in (driver, gateway, peer): executor.add_node(node)
    spin = threading.Thread(target=executor.spin); spin.start()
    http = HttpServer(('127.0.0.1', 0), gateway, TOKEN)
    ht = threading.Thread(target=http.serve_forever); ht.start()
    origin = f'http://127.0.0.1:{http.server_port}'
    def request(method, path, body=None, token=TOKEN):
        return requests.request(method, origin+path, json=body, headers={'Authorization':'Bearer '+token}, timeout=20)
    state.update(gateway=gateway, driver=driver, peer=peer, request=request, origin=origin)
    wait(lambda: gateway.ros_clients['stop'].service_is_ready())
    wait(lambda: gateway.actions['arm_step'].server_is_ready())
    try: yield state
    finally:
        state['release'].set(); state['shutdown'].set()
        http.shutdown(); http.server_close(); ht.join(3)
        driver.close(); executor.shutdown(timeout_sec=10); spin.join(10)
        for node in (peer, gateway, driver): node.destroy_node()
        rclpy.shutdown(); backend.shutdown(); backend.server_close(); bt.join(3)
        assert not spin.is_alive() and not ht.is_alive()


@pytest.mark.parametrize('field,value', [('target_x_m', True), ('target_y_m', float('nan')), ('max_duration_s', 0), ('dry_run', 1)])
def test_pin_and_go_validation(field, value):
    payload=dict(target_x_m=24.5,target_y_m=26.,max_duration_s=60.,dry_run=True)
    message=goal_message('pin_and_go',payload)
    assert message.target_x_m==24.5
    with pytest.raises(ApiError): goal_message('pin_and_go',dict(payload,**{field:value}))
    with pytest.raises(ApiError): goal_message('pin_and_go',dict(payload,extra=True))
    with pytest.raises(ApiError): goal_message('pin_and_go',{})


@pytest.mark.parametrize('kind', ['deliver_garment','patrol_collect'])
def test_garment_goal_validation(kind):
    goal=dict(drop_x_m=1.,drop_y_m=2.,max_duration_s=60.,allow_towing=False,dry_run=True)
    goal.update(category='sock') if kind=='deliver_garment' else goal.update(max_items=10)
    assert goal_message(kind,goal).dry_run
    for field,value in [('drop_x_m',True),('drop_y_m',float('nan')),('allow_towing',1),('dry_run',0),('max_duration_s',0)]:
        with pytest.raises(ApiError): goal_message(kind,dict(goal,**{field:value}))
    with pytest.raises(ApiError): goal_message(kind,dict(goal,extra=True))
    with pytest.raises(ApiError): goal_message(kind,{})


def start(system, kind='arm_step', goal=None):
    r = system['request']('POST', '/api/ros/tasks', {'kind':kind,'goal':goal or {'joint':3,'direction':1}})
    assert r.status_code == 202, r.text
    return r.json()


def task(system, ident):
    return system['request']('GET', '/api/ros/tasks/'+ident).json()


def test_native_gateway_enforces_exact_loopback_host_and_auth():
    class FakeGateway:
        def route(self, method, path, body):
            return 200, {'offline_fixture': True}, {}

    server = HttpServer(('127.0.0.1', 0), FakeGateway(), TOKEN)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    port = server.server_port
    url = f'http://127.0.0.1:{port}/api/ros/state'
    try:
        response = requests.get(url, headers={'Host': f'127.0.0.1:{port}', 'Authorization': 'Bearer '+TOKEN}, timeout=3)
        assert response.status_code == 200, response.text
        assert response.json() == {'offline_fixture': True}
        for host in ('evil.example', f'host.docker.internal:{port}', '127.0.0.1:1'):
            response = requests.get(url, headers={'Host': host, 'Authorization': 'Bearer '+TOKEN}, timeout=3)
            assert response.status_code == 403
        headers = {'Host': f'127.0.0.1:{port}'}
        assert requests.get(url, headers=headers, timeout=3).status_code == 401
        headers.update({'Authorization': 'Bearer '+TOKEN, 'Origin': 'http://localhost'})
        assert requests.get(url, headers=headers, timeout=3).status_code == 403
    finally:
        server.shutdown()
        server.server_close()
        thread.join(3)


def test_validation_token_and_closed_error_contract(system, tmp_path):
    request = system['request']
    catalog = request('GET', '/api/ros/goals').json()
    goals = catalog['goals']
    assert any(g['kind'] == 'pick' and g['topic'] == '/z70/pick' and g['action'] == 'Pick' and g['fields'] == ['color', 'dry_run'] for g in goals)
    primitives = {p['name'] for p in catalog['context_primitives']}
    assert {'status', 'arm', 'map_pose', 'safe_interior', 'objects', 'garments', 'world_objects', 'update_world_object', 'mark_obstacle', 'clear_world', 'navigate', 'deploy_arm', 'align_target', 'move_arm', 'grasp_and_lift', 'release_and_stow', 'pulse', 'rpc', 'feedback', 'outcome'} <= primitives
    world_pub = system['peer'].create_publisher(String, '/z70/world/objects', LATCHED)
    dyn_pub = system['peer'].create_publisher(OccupancyGrid, '/z70/map/dynamic_obstacles', LATCHED)
    try:
        world_pub.publish(String(data=json.dumps({'session_bound': True, 'objectives': [{'id': 'sock_1'}], 'obstacles': [], 'delivered_piles': [], 'entries': [{'id': 'sock_1'}]})))
        dyn_grid = OccupancyGrid(); dyn_grid.info.width = 4; dyn_grid.info.height = 4; dyn_grid.data = [0] * 16
        dyn_pub.publish(dyn_grid)
        wait(lambda: 'world_objects' in request('GET', '/api/ros/map').json() and 'dynamic_obstacles' in request('GET', '/api/ros/map').json())
        map_res = request('GET', '/api/ros/map').json()
        assert map_res['world_objects']['objectives'] == [{'id': 'sock_1'}]
        assert map_res['dynamic_obstacles']['info']['width'] == 4
    finally:
        system['peer'].destroy_publisher(world_pub)
        system['peer'].destroy_publisher(dyn_pub)
    valid = request('POST', '/api/ros/goals/validate', {'kind': 'pick', 'goal': {'color': 'yellow', 'dry_run': True}})
    assert valid.status_code == 200 and valid.json() == {'valid': True, 'kind': 'pick', 'topic': '/z70/pick', 'action': 'Pick'}
    simulated = request('POST', '/api/ros/goals/validate', {'kind': 'pick', 'goal': {'color': 'yellow', 'dry_run': True}, 'simulate': True})
    assert simulated.status_code == 200 and simulated.json()['simulated']['ok'] is True and simulated.json()['simulated']['result']['dry_run_valid'] is True
    sim_override = request('POST', '/api/ros/goals/validate', {
        'kind': 'pick', 'goal': {'color': 'yellow', 'dry_run': True},
        'simulate': True, 'context': {'pick': {'verified': False, 'dry_run': True, 'target': None}},
    })
    assert sim_override.status_code == 200 and sim_override.json()['simulated']['ok'] is False
    bad_ctx = request('POST', '/api/ros/goals/validate', {
        'kind': 'pick', 'goal': {'color': 'yellow', 'dry_run': True},
        'simulate': True, 'context': {'unknown_primitive': 1},
    })
    assert bad_ctx.status_code == 400 and bad_ctx.json()['error'] == 'INVALID_INPUT'
    invalid = request('POST', '/api/ros/goals/validate', {'kind': 'pick', 'goal': {'color': 'yellow', 'dry_run': 1}})
    assert invalid.status_code == 400 and invalid.json()['valid'] is False and invalid.json()['error'] == 'INVALID_INPUT'
    assert request('GET','/api/ros/state',token='wrong').status_code == 401
    assert request('POST','/api/ros/tasks',{'kind':'arm_step','goal':{'joint':3,'direction':True}}).status_code == 400
    assert request('POST','/api/ros/tasks',{'kind':'base_pulse','goal':{'linear_mps':.1,'angular_radps':.2,'duration':{'sec':0,'nanosec':100}}}).status_code == 400
    assert not system['calls']
    bad = request('GET', '/api/ros/tasks/'+str(uuid.uuid4()))
    assert bad.status_code == 404 and bad.json()['error'] == 'TASK_NOT_FOUND'
    assert request('POST','/api/ros/rpc',{'method':'app_grip','params':{},'token':'never DDS'}).status_code == 400
    path = tmp_path/'fixture-token'; path.write_text(TOKEN);path.chmod(0o600)
    assert read_token(path) == TOKEN
    path.chmod(0o644)
    with pytest.raises(ValueError): read_token(path)
    link = tmp_path/'symlink'; link.symlink_to(path)
    with pytest.raises(OSError): read_token(link)


def test_end_control_session_stops_motion_and_blocks_reacquisition_until_resume(system):
    request = system['request']
    system['hang'] = True
    first = start(system); wait(lambda: 'app_arm_move' in system['calls'])
    before = time.monotonic()
    stopped = request('POST', '/api/ros/control/session', {'enabled': False})
    assert stopped.status_code == 200, stopped.text
    assert time.monotonic()-before < 2
    result = stopped.json()
    assert result['enabled'] is False and result['streams_released'] is True
    assert result['cessation_observed'] is False
    assert 'stop_camera_preview' in system['calls'], 'owned robot preview must be explicitly released'
    assert system.get('preview_active', True) is False
    assert all(m in system['calls'] for m in ('app_rc_stop', 'app_rc_end', 'app_stop_grasp', 'app_pick_up_enter_exit'))
    wait(lambda: task(system, first['id'])['status'] in ('failed', 'cancelled'))
    assert request('POST', '/api/ros/tasks', {'kind':'arm_step','goal':{'joint':3,'direction':1}}).status_code == 409
    rpc = request('POST', '/api/ros/rpc', {'method':'app_rc_start','params':[]})
    assert rpc.json()['outcome']['error_code'] == 'SESSION_STOPPED'
    assert rpc.json()['outcome']['dispatch'] == 0
    assert request('POST', '/api/ros/camera/select', {'camera':'front'}).json()['outcome']['dispatch'] == 0
    assert request('POST', '/api/ros/frame', {}).status_code == 409
    # Cached state reads must not reopen acquisition, and a normal stop must not resume it.
    assert request('GET', '/api/ros/state').json()['session_enabled'] is False
    assert request('POST', '/api/ros/stop', {'scope':3}).status_code == 200
    assert request('GET', '/api/ros/state').json()['session_enabled'] is False
    system['release'].set(); system['hang'] = False
    resumed = request('POST', '/api/ros/control/session', {'enabled':True})
    assert resumed.status_code == 200 and resumed.json()['enabled'] is True
    last = start(system)
    wait(lambda: task(system, last['id'])['status'] == 'succeeded')


def test_session_stop_does_not_wait_for_stalled_image_or_allow_background_reopen(system):
    driver = system['driver']; driver.camera_hz = 10.
    system['hang_frame'] = True
    driver.start_worker('camera_test', driver.camera_loop)
    wait(lambda: system.get('frames_requested', 0) == 1)
    before = time.monotonic()
    stopped = system['request']('POST', '/api/ros/control/session', {'enabled':False})
    assert stopped.status_code == 200 and time.monotonic()-before < 2
    assert stopped.json()['streams_released']
    system['release'].set()
    time.sleep(.35)
    assert system['frames_requested'] == 1
    assert all(method in system['calls'] for method in ('app_rc_stop','app_rc_end','app_stop_grasp'))
    assert system['request']('POST', '/api/ros/control/session', {'enabled':True}).json()['enabled']
    wait(lambda: system['frames_requested'] > 1)


@pytest.mark.parametrize('preview_owner', ['official-app', None])
def test_session_cleanup_never_stops_another_owner_or_claims_unknown_release(system, preview_owner):
    system['preview_owner'] = preview_owner
    stopped = system['request']('POST', '/api/ros/control/session', {'enabled':False})
    assert stopped.status_code == 200
    assert stopped.json()['streams_released'] is False
    assert 'stop_camera_preview' not in system['calls']
    assert stopped.json()['enabled'] is False
    assert stopped.json()['outcomes'][-1]['error_code'] == 'STREAM_RELEASE_UNVERIFIED'


def test_native_navigation_stop_is_added_only_after_goto(system):
    request=system['request']
    reply=request('POST','/api/ros/rpc',{'method':'app_goto_target','params':[2000,1000]})
    assert reply.status_code==200 and reply.json()['outcome']['response']==1
    stopped=request('POST','/api/ros/stop',{'scope':3})
    assert stopped.status_code==200
    assert system['calls']==['app_goto_target','stop_goto_target','app_rc_stop','app_rc_end','app_stop_grasp']
    system['calls'].clear()
    request('POST','/api/ros/stop',{'scope':3})
    assert system['calls']==['app_rc_stop','app_rc_end','app_stop_grasp']


def test_shared_goal_uuid_feedback_and_unknown_failed_results(system):
    first = start(system); ident = first['id']
    result = wait(lambda: (r := task(system, ident))['status'] == 'succeeded' and r)
    assert result['phase'] == 5
    assert str(uuid.UUID(bytes=bytes(system['gateway'].handles[ident].goal_id.uuid))) == ident
    assert any(t['id'] == ident for t in system['request']('GET','/api/ros/tasks').json()['tasks'])
    system['unknown'] = True
    second = start(system)
    unknown = wait(lambda: (r := task(system, second['id']))['status'] == 'failed' and r)
    assert unknown['result']['outcome']['dispatch'] == 2
    assert system['calls'].count('app_arm_move') == 2
    system['unknown'] = False; system['reject'] = True
    third = start(system)
    rejected = wait(lambda: (r := task(system, third['id']))['status'] == 'failed' and r)
    assert rejected['result']['outcome']['response'] == 2


@pytest.mark.parametrize('mode', ['stop', 'cancel'])
def test_stop_and_cancel_do_not_wait_on_long_task(system, mode):
    system['hang'] = True
    first = start(system); wait(lambda:'app_arm_move' in system['calls'])
    before = time.monotonic()
    if mode == 'stop':
        stop = system['request']('POST','/api/ros/stop', {'scope':3})
        assert stop.status_code == 200 and time.monotonic()-before < 2
        assert stop.json()['cessation_observed'] is False
        # Priority stop already revokes the running action. A subsequent cancel
        # may correctly find a terminal task; do not race it against stop.
        result = wait(lambda:(r := task(system,first['id']))['status'] in ('failed','cancelled') and r)
        cancel = system['request']('POST',f"/api/ros/tasks/{first['id']}/cancel", {})
        assert cancel.status_code == 409 and cancel.json()['error'] == 'TASK_FINISHED'
    else:
        cancel = system['request']('POST',f"/api/ros/tasks/{first['id']}/cancel", {})
        assert cancel.status_code == 200 and cancel.json()['cancel_requested']
        result = wait(lambda:(r := task(system,first['id']))['status'] == 'cancelled' and r)
        assert time.monotonic()-before < 2
    assert not any(m in system['calls'] for m in ('app_arm_in_compartment','app_open_gripper','app_put_down'))


def test_latest_shared_frames_preserve_identity_and_do_not_acquire(system):
    peer = system['peer']; gateway = system['gateway']; request=system['request']
    pub = peer.create_publisher(CompressedImage,'/camera/front/image_raw/compressed',2)
    metadata = peer.create_publisher(FrameMetadata,'/camera/front/metadata',2)
    wait(lambda: pub.get_subscription_count() > 0 and metadata.get_subscription_count() > 0)
    image = CompressedImage();image.header.stamp=peer.get_clock().now().to_msg();image.header.frame_id='front_optical'
    image.data=[255,216,255,217]; image.format='jpeg'
    meta=FrameMetadata();meta.header=image.header;meta.camera=1;meta.evidence.valid=True
    meta.evidence.received_at=image.header.stamp;meta.evidence.session_id.uuid=list(uuid.uuid4().bytes);meta.evidence.sequence=42
    # No calibration ID needed for viewing; metric consumers remain separate.
    pub.publish(image); metadata.publish(meta)
    wait(lambda:'front' in gateway.frames)
    first=request('POST','/api/ros/frame',{'camera':'front'}); second=request('POST','/api/ros/frame',{'camera':'front'})
    assert first.content == bytes(image.data) == second.content
    assert first.headers['X-Roboclaw-Frame-Id'] == second.headers['X-Roboclaw-Frame-Id'] == '42'
    assert not system['calls']
    with gateway.lock: gateway.frames['front'][1].evidence.received_at.sec -= 10
    assert request('POST','/api/ros/frame',{'camera':'front'}).json()['error']=='OBSERVATION_GAP'


def test_ros_unavailable_reconnect_does_not_replay(system):
    gateway=system['gateway']; request=system['request']
    missing = gateway.create_client(RawRpc,'/phase4/missing')
    original = gateway.ros_clients['rpc']; gateway.ros_clients['rpc']=missing
    reply=request('POST','/api/ros/rpc',{'method':'app_grip','params':{}})
    assert reply.status_code == 503 and reply.json()['dispatch']=='not_sent'
    assert not system['calls']
    gateway.ros_clients['rpc']=original
    reply=request('POST','/api/ros/rpc',{'method':'app_open_gripper','params':{}})
    assert reply.status_code == 200 and system['calls']==['app_open_gripper']
    gateway.destroy_client(missing)


def test_pick_feedback_and_cancel_have_same_action_identity(system):
    peer=system['peer']
    def execute(goal):
        feedback=Pick.Feedback(phase=3,reason='OBSERVING');goal.publish_feedback(feedback)
        until=time.monotonic()+5
        while time.monotonic()<until:
            if goal.is_cancel_requested:
                goal.canceled();return Pick.Result()
            time.sleep(.02)
        goal.abort();return Pick.Result()
    server=ActionServer(peer,Pick,'/z70/pick',execute_callback=execute,cancel_callback=lambda _:CancelResponse.ACCEPT,
        callback_group=system['gateway'].group)
    try:
        wait(lambda:system['gateway'].actions['pick'].server_is_ready())
        first=start(system,'pick',{'color':'yellow','dry_run':True})
        ident=first['id']; wait(lambda: task(system,ident)['phase']==3)
        assert system['request']('POST',f'/api/ros/tasks/{ident}/cancel',{}).status_code == 200
        wait(lambda:task(system,ident)['status']=='cancelled')
    finally: server.destroy()
