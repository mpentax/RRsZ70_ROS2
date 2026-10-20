"""ROS action and service command routing over loopback HTTP transport."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import queue
import socket
import threading
import time
import uuid

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message, deserialize_message
from unique_identifier_msgs.msg import UUID
from z70_interfaces.action import BasePulse, ArmStep, HandStep, ArmOperation, Pick
from z70_interfaces.msg import CommandEvent, CalibrationState
from z70_interfaces.srv import RawRpc, Stop, TaskSession, SetArmAnchor
from z70_driver.commands import action_plan
from z70_driver.driver import Driver


def wait_for(predicate, seconds=5):
    until = time.monotonic()+seconds
    while time.monotonic() < until:
        if predicate(): return
        time.sleep(.01)
    assert predicate(), 'condition deadline'


def resolved(future, seconds=5):
    wait_for(future.done, seconds)
    return future.result()


@pytest.fixture
def system():
    data = dict(calls=[], hang=set(), reject=set(), unknown=set(), disconnect=set(), release=threading.Event(),
                shutdown=threading.Event(), events=queue.Queue(), sequence=0, session=str(uuid.uuid4()))
    lock = threading.Lock()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def reply(self, value, status=200):
            content = json.dumps(value).encode()
            try:
                self.send_response(status); self.send_header('Content-Length', str(len(content)))
                self.end_headers(); self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError): pass
        def do_GET(self):
            if self.path == '/api/commands/events':
                self.send_response(200); self.end_headers()
                while not data['shutdown'].is_set():
                    try: row = data['events'].get(timeout=.1); line = json.dumps(row).encode()+b'\n'
                    except queue.Empty: line = b'\n'
                    try: self.wfile.write(line); self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError): return
                return
            if self.path == '/api/state':
                now = int(time.time()*1000)
                return self.reply(dict(observationSession=data['session'], connection=dict(state='connected'),
                    telemetry=dict(status=dict(value=dict(state=39, error_code=0), observedAt=now))))
            data['calls'].append(('GET', self.path))
            self.reply({}, 404)
        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            if self.path != '/api/driver/rpc':
                data['calls'].append(('POST', self.path)); return self.reply({}, 404)
            method = body['method']; now = int(time.time()*1000)
            command, attempt = body.get('commandId',str(uuid.uuid4())), str(uuid.uuid4())
            with lock: data['calls'].append((method, body, command, attempt))
            def event(phase, dispatch='sent', response='none', error='NONE'):
                with lock:
                    data['sequence'] += 1
                    data['events'].put(dict(sessionId=data['session'], sequence=data['sequence'],
                        phase=phase, operation=method, commandId=command, attemptId=attempt,
                        attemptIndex=0, requestedAt=now, attemptedAt=now, dispatch=dispatch,
                        response=response, errorCode=error, finishedAt=int(time.time()*1000)))
            event('accepted', 'not_sent'); event('dispatched')
            if method in data['disconnect']:
                self.connection.shutdown(socket.SHUT_RDWR);self.connection.close();return
            if method in data['hang']: data['release'].wait(20)
            response = 'vendor_error' if method in data['reject'] else 'transport_error' if method in data['unknown'] else 'received'
            dispatch = 'unknown' if method in data['unknown'] else 'sent'
            error = 'VENDOR_REJECTED' if response == 'vendor_error' else 'TIMEOUT' if response == 'transport_error' else 'NONE'
            event('replied', dispatch, response, error); event('finished', dispatch, response, error)
            self.reply(dict(commandId=command, attemptId=attempt, attemptIndex=0, requestId='not-a-ros-uuid',
                dispatch=dispatch, response=response, startedAt=now, finishedAt=int(time.time()*1000),
                result=['ok'], transportReason='timeout' if dispatch == 'unknown' else None),
                status=200 if response == 'received' else 502)
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    server.daemon_threads = True
    server_thread = threading.Thread(target=server.serve_forever); server_thread.start()
    rclpy.init(domain_id=92)
    driver = Driver([Parameter('origin', value=f'http://127.0.0.1:{server.server_port}'),
        Parameter('camera_hz', value=0.), Parameter('map_refresh_s', value=0.),
        Parameter('state_hz', value=5.), Parameter('arm_estimate_path', value='')])
    node = rclpy.create_node('command_test_client')
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(driver); executor.add_node(node)
    spin = threading.Thread(target=executor.spin); spin.start()
    data.update(driver=driver, node=node, executor=executor, observed=[])
    node.create_subscription(CommandEvent, '/z70/commands', data['observed'].append, 256)
    # Wait discovery before emitting events (the stream intentionally has no replay).
    wait_for(lambda: driver.pub_events.get_subscription_count() > 0)
    time.sleep(.1)
    try: yield data
    finally:
        data['release'].set(); data['shutdown'].set()
        driver.close(); executor.shutdown(timeout_sec=10); spin.join(10)
        node.destroy_node(); driver.destroy_node(); rclpy.shutdown()
        server.shutdown(); server.server_close(); server_thread.join(5)
        assert not spin.is_alive() and not any(worker.is_alive() for worker in driver.workers)


def service(system, kind, name, request):
    client = system['node'].create_client(kind, name)
    assert client.wait_for_service(timeout_sec=3)
    return client.call_async(request)


def action(system, kind, name, goal):
    client = ActionClient(system['node'], kind, name)
    assert client.wait_for_server(timeout_sec=3)
    handle = resolved(client.send_goal_async(goal))
    return client, handle


def methods(system):
    return [row[0] for row in system['calls']]


def test_generated_contract_and_input_validation():
    for kind in (BasePulse, ArmStep, HandStep, ArmOperation, Pick):
        for part in (kind.Goal, kind.Result, kind.Feedback):
            msg = part(); assert deserialize_message(serialize_message(msg), part) == msg
    for kind in (RawRpc, TaskSession, Stop, SetArmAnchor):
        for part in (kind.Request, kind.Response):
            msg = part(); assert deserialize_message(serialize_message(msg), part) == msg
    msg = CalibrationState(); assert deserialize_message(serialize_message(msg), type(msg)) == msg
    invalid = [BasePulse.Goal(), BasePulse.Goal(linear_mps=float('nan'))]
    for goal in invalid:
        with pytest.raises(ValueError): action_plan(BasePulse, goal)
    for kind, goal in [(ArmStep, ArmStep.Goal(joint=2, direction=1)),
                       (HandStep, HandStep.Goal(direction=5)),
                       (ArmOperation, ArmOperation.Goal(operation=0)),
                       (ArmOperation, ArmOperation.Goal(operation=10))]:
        with pytest.raises(ValueError): action_plan(kind, goal)
    assert action_plan(ArmStep, ArmStep.Goal(joint=4, direction=-1))[0] == [('app_arm_move', {'m':[0,2,0]})]
    assert action_plan(ArmOperation, ArmOperation.Goal(operation=6))[0] == [('app_pick_up_enter_exit', {'flag':1})]
    assert action_plan(ArmOperation, ArmOperation.Goal(operation=7))[0] == [('app_pick_up_enter_exit', {'flag':2})]
    assert action_plan(ArmOperation, ArmOperation.Goal(operation=8))[0] == [('app_pick_up_enter_exit', {'flag':0})]
    assert action_plan(ArmOperation, ArmOperation.Goal(operation=9))[0] == [('app_arm_recover', {})]


def test_action_result_backend_ids_and_no_duplicate_events(system):
    client, handle = action(system, ArmStep, '/z70/arm/step', ArmStep.Goal(joint=3, direction=1))
    assert handle.accepted
    result = resolved(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    outcome = result.result.outcome
    row = next(row for row in system['calls'] if row[0] == 'app_arm_move')
    assert bytes(outcome.command_id.uuid) == uuid.UUID(row[2]).bytes
    assert bytes(outcome.attempt_id.uuid) == uuid.UUID(row[3]).bytes
    assert outcome.physical_completion == 0 and not outcome.write_time_known
    wait_for(lambda: len(system['observed']) == 4)
    assert all(event.outcome.command_id == outcome.command_id for event in system['observed'])
    assert [event.phase for event in system['observed']] == [0, 1, 2, 4]
    assert 'GET' not in methods(system)  # In particular, no TS arm estimate read.
    client.destroy()


def test_hanging_action_cancel_priority_conflicts_and_no_cleanup_motion(system):
    system['hang'].add('app_arm_move')
    client, handle = action(system, ArmStep, '/z70/arm/step', ArmStep.Goal(joint=3, direction=1))
    assert handle.accepted
    wait_for(lambda: 'app_arm_move' in methods(system))
    other, rejected = action(system, HandStep, '/z70/arm/hand_step', HandStep.Goal(direction=1))
    assert not rejected.accepted
    raw = resolved(service(system, RawRpc, '/z70/driver/raw_rpc', RawRpc.Request(
        method='app_grip', params_json='{}', timeout_ms=1000)))
    assert raw.outcome.error_code == 'BUSY'
    # Camera/inference lock cannot hold cancellation or stop.
    system['driver'].camera_lock.acquire()
    start = time.monotonic()
    try:
        cancel = resolved(handle.cancel_goal_async(), 2)
        assert cancel.goals_canceling
        result = resolved(handle.get_result_async(), 3)
    finally: system['driver'].camera_lock.release()
    assert time.monotonic()-start < 3
    assert result.status == GoalStatus.STATUS_CANCELED and result.result.outcome.physical_completion == 0
    assert methods(system) == ['app_arm_move', 'app_rc_stop', 'app_rc_end', 'app_stop_grasp']
    assert all(row[1]['timeoutMs'] <= 5000 for row in system['calls'][1:])
    system['release'].set()
    client.destroy(); other.destroy()


def test_task_manual_conflict_takeover_stops_before_new_rpc_and_revokes_token(system):
    cancelled = []; system['node'].create_subscription(UUID, '/z70/tasks/cancel', cancelled.append, 10)
    begin = resolved(service(system, TaskSession, '/z70/tasks/session', TaskSession.Request(begin=True)))
    assert begin.accepted and any(begin.task_id.uuid)
    manual = resolved(service(system, RawRpc, '/z70/driver/raw_rpc', RawRpc.Request(method='app_grip', timeout_ms=1000)))
    assert manual.outcome.error_code == 'BUSY'
    system['hang'].add('app_arm_move')
    pending = service(system, RawRpc, '/z70/driver/raw_rpc', RawRpc.Request(
        method='app_arm_move', params_json='{"m":[1,0,0]}', timeout_ms=10000, task_id=begin.task_id))
    wait_for(lambda: 'app_arm_move' in methods(system))
    takeover = resolved(service(system, RawRpc, '/z70/driver/raw_rpc', RawRpc.Request(
        method='app_open_gripper', timeout_ms=1000, takeover=True)), 3)
    assert takeover.outcome.response == 1 and takeover.outcome.physical_completion == 0
    assert methods(system) == ['app_arm_move', 'app_rc_stop', 'app_rc_end', 'app_stop_grasp', 'app_open_gripper']
    wait_for(lambda: cancelled)
    assert cancelled[0] == begin.task_id
    old = resolved(service(system, RawRpc, '/z70/driver/raw_rpc', RawRpc.Request(
        method='app_grip', timeout_ms=1000, task_id=begin.task_id)))
    assert old.outcome.error_code == 'BUSY'
    assert not resolved(service(system, TaskSession, '/z70/tasks/session', TaskSession.Request(task_id=begin.task_id))).accepted
    system['release'].set(); resolved(pending)


def test_stop_independent_hanging_raw_and_unknown_motion_not_replayed(system):
    system['hang'].add('app_arm_move')
    pending = service(system, RawRpc, '/z70/driver/raw_rpc', RawRpc.Request(
        method='app_arm_move', params_json='{"m":[1,0,0]}', timeout_ms=10000))
    wait_for(lambda: 'app_arm_move' in methods(system))
    start = time.monotonic()
    stopped = resolved(service(system, Stop, '/z70/stop', Stop.Request(scope=3)), 3)
    assert time.monotonic()-start < 3 and len(stopped.outcomes) == 3 and not stopped.cessation_observed
    assert all(outcome.physical_completion == 0 for outcome in stopped.outcomes)
    system['release'].set(); resolved(pending)
    system['unknown'].add('app_grip')
    client, handle = action(system, ArmOperation, '/z70/arm/operation', ArmOperation.Goal(operation=4))
    result = resolved(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.outcome.dispatch == 2 and result.result.outcome.error_code == 'TIMEOUT'
    assert methods(system).count('app_grip') == 1
    client.destroy()


def test_vendor_rejection_and_invalid_service_do_not_succeed(system):
    system['reject'].add('app_arm_direction_move')
    client, handle = action(system, HandStep, '/z70/arm/hand_step', HandStep.Goal(direction=2))
    result = resolved(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.outcome.response == 2 and result.result.outcome.error_code == 'VENDOR_REJECTED'
    response = resolved(service(system, Stop, '/z70/stop', Stop.Request(scope=0)))
    assert response.outcomes[0].error_code == 'INVALID_INPUT'
    response = resolved(service(system, RawRpc, '/z70/driver/raw_rpc', RawRpc.Request(method='bad method', timeout_ms=1000)))
    assert response.outcome.error_code == 'INVALID_INPUT'
    assert methods(system) == ['app_arm_direction_move']
    client.destroy()


def test_base_pulse_bounded_wire_mapping_and_unverified_odom(system):
    from builtin_interfaces.msg import Duration
    client, invalid = action(system, BasePulse, '/z70/base/pulse', BasePulse.Goal(
        linear_mps=.1, angular_radps=.1, duration=Duration(nanosec=100000000)))
    assert not invalid.accepted and not methods(system)
    client.destroy()
    client, handle = action(system, BasePulse, '/z70/base/pulse', BasePulse.Goal(
        linear_mps=.05, duration=Duration(nanosec=100000000)))
    assert handle.accepted
    result = resolved(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert not result.result.odom_delta_valid and result.result.outcome.physical_completion == 0
    assert methods(system) == ['app_rc_start', 'app_rc_move', 'app_rc_stop', 'app_rc_end']
    move = system['calls'][1][1]['params']
    assert move == [dict(velocity=.05, omega=0., duration=100, seqnum=1)]
    assert all(row[1]['timeoutMs'] <= 5000 for row in system['calls'])
    client.destroy()


def test_stop_budget_with_all_three_http_replies_hanging(system):
    system['hang'].update(('app_rc_stop', 'app_rc_end', 'app_stop_grasp'))
    start = time.monotonic()
    response = resolved(service(system, Stop, '/z70/stop', Stop.Request(scope=3)), 16)
    elapsed = time.monotonic()-start
    assert elapsed <= 15.5  # Includes DDS scheduling, not an extended transport budget.
    assert len(response.outcomes) == 3 and not response.cessation_observed
    assert all(o.dispatch == 2 and o.error_code == 'STOP_UNVERIFIED' for o in response.outcomes)
    assert methods(system) == ['app_rc_stop', 'app_rc_end', 'app_stop_grasp']
    assert all(row[1]['timeoutMs'] <= 5000 for row in system['calls'])
    system['release'].set()


def test_anchor_without_motion_and_each_rpc_observed_exactly_once(system):
    driver = system['driver']; observed = []
    original = driver.estimator.observe
    def track(method, params, outcome, command_id=None):
        observed.append((method, outcome, command_id))
        return original(method, params, outcome, command_id=command_id)
    driver.estimator.observe = track
    anchor = resolved(service(system, SetArmAnchor, '/z70/arm/anchor',
        SetArmAnchor.Request(anchor=2, observation_reason='TEST_OPERATOR_ANCHOR')))
    assert anchor.estimate.source == 3 and anchor.estimate.validity.value == 1
    assert not methods(system)
    response = resolved(service(system, RawRpc, '/z70/driver/raw_rpc', RawRpc.Request(
        method='app_arm_move', params_json='{"m":[1,0,0]}', timeout_ms=1000)))
    assert response.outcome.response == 1
    stop = resolved(service(system, Stop, '/z70/stop', Stop.Request(scope=3)))
    assert len(stop.outcomes) == 3
    assert [row[0] for row in observed] == methods(system)
    assert len(observed) == 4 and len({row[2] for row in observed}) == 4


def test_advanced_cloud_uses_same_scheduler_estimator_and_stop_path(system):
    response=resolved(service(system,RawRpc,'/z70/driver/raw_rpc',RawRpc.Request(
        method='app_arm_move',params_json='{"m":[0,0,1]}',timeout_ms=1000,transport=1)))
    assert response.outcome.response==1 and system['calls'][0][1]['transport']=='cloud'
    stopped=resolved(service(system,Stop,'/z70/stop',Stop.Request(scope=3)))
    assert len(stopped.outcomes)==3
    assert all(row[1]['transport']=='cloud' for row in system['calls'])
    invalid=resolved(service(system,RawRpc,'/z70/driver/raw_rpc',RawRpc.Request(
        method='app_grip',timeout_ms=1000,transport=2)))
    assert invalid.outcome.error_code=='INVALID_INPUT' and len(system['calls'])==4


def test_actual_http_disconnect_preserves_logical_id_and_never_replays_motion(system):
    system['disconnect'].add('app_arm_move')
    client,handle=action(system,ArmStep,'/z70/arm/step',ArmStep.Goal(joint=3,direction=1))
    result=resolved(handle.get_result_async(),4)
    assert result.status==GoalStatus.STATUS_ABORTED and result.result.outcome.dispatch==2
    assert result.result.outcome.error_code=='TRANSPORT'
    row=next(r for r in system['calls'] if r[0]=='app_arm_move')
    assert bytes(result.result.outcome.command_id.uuid)==uuid.UUID(row[1]['commandId']).bytes
    assert bytes(result.result.outcome.attempt_id.uuid)==bytes(16) # no fabricated vendor attempt evidence
    assert methods(system).count('app_arm_move')==1
    client.destroy()
