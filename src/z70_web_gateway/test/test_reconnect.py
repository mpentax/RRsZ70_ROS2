"""Web gateway ROS service, action timeout, reconnect, and launch control-session handling."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import signal
import socket
import subprocess
import threading
import time
import uuid

import pytest
import rclpy
import requests
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from z70_interfaces.action import Pick
from z70_interfaces.msg import CalibrationState
from z70_interfaces.srv import RawRpc
from z70_web_gateway.gateway import ApiError, Gateway
from test_gateway import wait


@pytest.fixture
def graph():
    rclpy.init(domain_id=95)
    gateway = Gateway()
    peer = rclpy.create_node('phase4_disconnected_peer')
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(gateway)
    executor.add_node(peer)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        yield gateway, peer
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(5)
        gateway.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()
        assert not thread.is_alive()


def test_actual_service_discovery_loss_and_reconnect_never_replays(graph):
    gateway, peer = graph
    calls = []

    def rpc(request, response):
        calls.append(request.method)
        response.outcome.dispatch = 2
        response.outcome.error_code = 'TIMEOUT'
        return response

    with pytest.raises(ApiError) as error:
        gateway.route('POST', '/api/ros/rpc', {'method': 'app_grip', 'params': {}})
    assert error.value.body['dispatch'] == 'not_sent'
    server = peer.create_service(RawRpc, '/z70/driver/raw_rpc', rpc)
    wait(gateway.ros_clients['rpc'].service_is_ready)
    response = gateway.route('POST', '/api/ros/rpc', {'method': 'app_grip', 'params': {}})[1]
    assert response['outcome']['dispatch'] == 2 and calls == ['app_grip']
    peer.destroy_service(server)
    wait(lambda: not gateway.ros_clients['rpc'].service_is_ready())
    with pytest.raises(ApiError):
        gateway.route('POST', '/api/ros/rpc', {'method': 'app_arm_move', 'params': {}})
    assert calls == ['app_grip']
    server = peer.create_service(RawRpc, '/z70/driver/raw_rpc', rpc)
    wait(gateway.ros_clients['rpc'].service_is_ready)
    # Discovery resumes reads; it does not resend either uncertain or unavailable writes.
    assert calls == ['app_grip']
    gateway.route('POST', '/api/ros/rpc', {'method': 'app_open_gripper', 'params': {}})
    assert calls == ['app_grip', 'app_open_gripper']
    peer.destroy_service(server)


def test_acceptance_timeout_retains_uuid_and_reconciles_late_result_once(graph):
    gateway, peer = graph
    goals = []

    def accept(request):
        time.sleep(3.2)
        return GoalResponse.ACCEPT

    def execute(handle):
        goals.append(bytes(handle.goal_id.uuid))
        handle.succeed()
        return Pick.Result(dry_run_valid=True)

    server = ActionServer(peer, Pick, '/z70/pick', goal_callback=accept, execute_callback=execute, callback_group=gateway.group)
    try:
        wait(gateway.actions['pick'].server_is_ready)
        with pytest.raises(ApiError) as error:
            gateway.start({'kind': 'pick', 'goal': {'color': 'yellow', 'dry_run': True}})
        assert error.value.status == 504 and error.value.body['dispatch'] == 'unknown'
        ident = error.value.body['id']
        assert gateway.task(ident)['status'] == 'unknown'
        wait(lambda: gateway.task(ident)['status'] == 'succeeded')
        assert len(goals) == 1 and len(gateway.tasks) == 1
        assert gateway.task(ident)['result']['dry_run_valid'] is True
    finally:
        server.destroy()


def test_standard_map_and_calibration_keep_source_fields_and_latched_identity(graph):
    gateway, peer = graph
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    pub = peer.create_publisher(OccupancyGrid, '/map', qos)
    cal_pub = peer.create_publisher(CalibrationState, '/z70/chassis_calibration', qos)
    wait(lambda: pub.get_subscription_count() > 0 and cal_pub.get_subscription_count() > 0)
    grid = OccupancyGrid()
    grid.info.width = 2
    grid.info.height = 2
    grid.info.resolution = .05
    grid.data = [-1, 0, 50, 100]
    grid.header.stamp.sec = 123
    grid.header.frame_id = 'map'
    pub.publish(grid)
    cal = CalibrationState()
    cal.evidence.sequence = 19
    cal.evidence.stale = True
    cal.evidence.received_at.sec = 456
    cal_pub.publish(cal)
    wait(lambda: 'map' in gateway.samples and 'calibration' in gateway.samples)
    first = gateway.route('GET', '/api/ros/map', {})[1]['map']
    second = gateway.route('GET', '/api/ros/map', {})[1]['map']
    assert first == second and first['data'] == [-1, 0, 50, 100] and first['header']['stamp']['sec'] == 123
    state = gateway.route('GET', '/api/ros/state', {})[1]
    assert state['calibration']['evidence']['sequence'] == 19 and state['calibration']['evidence']['stale']
    assert state['calibration']['evidence']['received_at']['sec'] == 456


def test_invalid_startup_configuration_is_bounded_and_nonzero(tmp_path):
    result = subprocess.run(
        ['ros2', 'run', 'z70_web_gateway', 'gateway', '--ros-args', '-p',
         'token_file:=' + str(tmp_path / 'missing-fixture-token')],
        env=dict(os.environ, ROS_DOMAIN_ID='97'), text=True, capture_output=True, timeout=15)
    assert result.returncode != 0
    assert 'Gateway unavailable: check port and server-side token_file parameter.' in result.stderr
    assert 'Traceback' not in result.stderr and 'missing-fixture-token' not in result.stderr


def test_standard_launch_session_stop_resume_with_fake_backend(tmp_path):
    token = 's'*43
    token_file = tmp_path/'fixture-token'; token_file.write_text(token); token_file.chmod(0o600)
    calls = []; shutdown = threading.Event(); release = threading.Event()
    camera_entered = threading.Event()
    class Backend(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def reply(self, value, status=200):
            body = json.dumps(value).encode()
            try:
                self.send_response(status); self.send_header('Content-Length', str(len(body)))
                self.send_header('Content-Type', 'application/json'); self.end_headers(); self.wfile.write(body)
            except OSError: pass
        def do_GET(self):
            if self.path == '/api/commands/events':
                self.send_response(200); self.end_headers()
                while not shutdown.wait(.1):
                    try: self.wfile.write(b'\n'); self.wfile.flush()
                    except OSError: return
                return
            if self.path == '/api/state':
                return self.reply(dict(observationSession='offline-session-launch',connection=dict(state='connected'),
                    telemetry=dict(status=dict(value=dict(state=39,error_code=0),observedAt=int(time.time()*1000)))))
            self.reply({},404)
        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            calls.append((self.path,body))
            if self.path == '/api/camera/frame':
                camera_entered.set(); release.wait(15); return self.reply({},404)
            if self.path == '/api/driver/control/session':
                return self.reply(dict(enabled=body['enabled'],streams_released=not body['enabled']))
            if self.path == '/api/driver/rpc':
                now = int(time.time()*1000)
                return self.reply(dict(commandId=body['commandId'],attemptId=str(uuid.uuid4()),attemptIndex=0,
                    startedAt=now,finishedAt=now,dispatch='sent',response='received',
                    result=dict(status=0,client_id='none') if body['method']=='get_homesec_connect_status' else ['ok']))
            self.reply({},404)
    backend = ThreadingHTTPServer(('127.0.0.1',0),Backend); backend.daemon_threads = True
    bt = threading.Thread(target=backend.serve_forever); bt.start()
    def port():
        with socket.socket() as s: s.bind(('127.0.0.1',0)); return s.getsockname()[1]
    fox_port = port()
    web_port = None
    for candidate in range(57070, 57100):
        with socket.socket() as s:
            try: s.bind(('127.0.0.1', candidate))
            except OSError: continue
            web_port = candidate; break
    assert web_port is not None, 'no isolated gateway test port available'
    launch_tmp = tmp_path/'tmp'; launch_tmp.mkdir()
    env = dict(os.environ,TMPDIR=str(launch_tmp),ROS_DOMAIN_ID='97',XDG_DATA_HOME=str(tmp_path/'data'),ROS_LOG_DIR=str(tmp_path/'ros-logs'),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    log = tmp_path/'launch.log'
    with log.open('w') as output:
        process = subprocess.Popen(['ros2','launch','z70_bringup','bringup.launch.py',
            f'origin:=http://127.0.0.1:{backend.server_port}',f'token_file:={token_file}',
            'camera_hz:=5.0','mapping:=false','depth:=false','manipulation:=false','web_gateway:=true',
            f'web_gateway_port:={web_port}',f'foxglove_port:={fox_port}'],
            env=env,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        headers = {'Authorization':'Bearer '+token}
        def request(method,path,body=None):
            return requests.request(method,f'http://127.0.0.1:{web_port}'+path,json=body,headers=headers,timeout=35)
        try:
            deadline = time.monotonic()+25
            while time.monotonic()<deadline:
                assert process.poll() is None, log.read_text()
                try:
                    response=request('GET','/api/ros/state')
                    if response.status_code==200 and response.json()['available']: break
                except requests.RequestException: pass
                time.sleep(.1)
            else: raise AssertionError(log.read_text())
            assert camera_entered.wait(5),log.read_text()
            before=time.monotonic()
            stopped=request('POST','/api/ros/control/session',{'enabled':False})
            assert stopped.status_code==200,stopped.text
            assert time.monotonic()-before<3,'session stop must not wait for the stalled image'
            assert stopped.json()['streams_released'] and not stopped.json()['enabled']
            assert not stopped.json()['cessation_observed']
            methods=[body['method'] for path,body in calls if path=='/api/driver/rpc']
            assert all(method in methods for method in ('app_rc_stop','app_rc_end','app_stop_grasp','app_pick_up_enter_exit'))
            release.set();time.sleep(.5)
            frame_count=sum(path=='/api/camera/frame' for path,_ in calls)
            time.sleep(.5)
            assert sum(path=='/api/camera/frame' for path,_ in calls)==frame_count==1
            resumed=request('POST','/api/ros/control/session',{'enabled':True})
            assert resumed.status_code==200 and resumed.json()['enabled']
            deadline=time.monotonic()+3
            while sum(path=='/api/camera/frame' for path,_ in calls)==1 and time.monotonic()<deadline:time.sleep(.05)
            assert sum(path=='/api/camera/frame' for path,_ in calls)>1
            assert request('POST','/api/ros/control/session',{'enabled':False}).json()['enabled'] is False
        finally:
            release.set();shutdown.set()
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
                raise AssertionError('isolated launch failed to shut down: '+log.read_text())
            backend.shutdown();backend.server_close();bt.join(3)
    assert process.returncode==0,log.read_text()
