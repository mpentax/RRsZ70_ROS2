"""PickServer cancellation, manual takeover, and deadline handling."""
import os
import subprocess
import time
import pytest
from action_msgs.msg import GoalStatus
from rclpy.action import get_action_names_and_types
from rclpy.parameter import Parameter
from z70_interfaces.action import Pick
from z70_interfaces.srv import RawRpc
from z70_manipulation.node import PickServer
from test_command_api import system, resolved, wait_for, action, service, methods


@pytest.fixture
def pick_server(system, tmp_path):
    server = PickServer([Parameter('log_root', value=str(tmp_path))])
    system['executor'].add_node(server)
    clients = []
    try:
        yield server, clients
    finally:
        system['executor'].shutdown(timeout_sec=10)
        for client in clients:
            client.destroy()
        system['executor'].remove_node(server)
        server.destroy_node()


@pytest.mark.parametrize('mode', ['cancel', 'takeover', 'deadline'])
def test_pick_image_stall_cancellation_and_manual_takeover(system, pick_server, mode):
    server, clients = pick_server
    client, handle = action(system, Pick, '/z70/pick', Pick.Goal(color='yellow'))
    clients.append(client)
    assert handle.accepted
    wait_for(lambda: any(system['driver'].dispatcher.task_id))
    # The Pick executor is now waiting for a shared image; not doing HTTP.
    manual = resolved(service(system, RawRpc, '/z70/driver/raw_rpc', RawRpc.Request(method='app_grip', timeout_ms=1000)))
    assert manual.outcome.error_code == 'BUSY' and not methods(system)
    start = time.monotonic()
    if mode == 'takeover':
        reply = resolved(service(system, RawRpc, '/z70/driver/raw_rpc', RawRpc.Request(method='app_open_gripper', timeout_ms=1000, takeover=True)), 3)
        assert reply.outcome.response == 1
    elif mode == 'cancel':
        assert resolved(handle.cancel_goal_async(), 2).goals_canceling
    else:
        server.deadline = time.monotonic()  # Exercise the independent deterministic watchdog.
    result = resolved(handle.get_result_async(), 4)
    assert time.monotonic() - start < 4
    assert result.status == (GoalStatus.STATUS_CANCELED if mode == 'cancel' else GoalStatus.STATUS_ABORTED)
    assert not result.result.verified_hold
    calls = methods(system)
    assert calls[:3] == ['app_rc_stop', 'app_rc_end', 'app_stop_grasp']
    assert calls[3:] == (['app_open_gripper'] if mode == 'takeover' else [])
    time.sleep(.15)
    assert methods(system) == calls  # revoked task cannot stop a new controller
    assert all(m not in calls for m in ('app_put_down', 'app_arm_in_compartment', 'app_arm_move'))


def test_repeated_pick_takeover_keeps_shared_executor_and_replacement_alive(system, pick_server):
    _, clients = pick_server
    for _ in range(2):
        client, handle = action(system, Pick, '/z70/pick', Pick.Goal(color='yellow'))
        clients.append(client)
        assert handle.accepted
        wait_for(lambda: any(system['driver'].dispatcher.task_id))
        reply = resolved(service(system, RawRpc, '/z70/driver/raw_rpc', RawRpc.Request(method='app_open_gripper', timeout_ms=1000, takeover=True)), 3)
        assert reply.outcome.response == 1
        result = resolved(handle.get_result_async(), 4)
        assert result.status == GoalStatus.STATUS_ABORTED and not result.result.verified_hold
    before = methods(system)
    time.sleep(.15)
    assert methods(system) == before
    assert before == ['app_rc_stop', 'app_rc_end', 'app_stop_grasp', 'app_open_gripper'] * 2


def test_standard_ros_cli_inspects_fake_graph(system):
    # Jazzy's action-list verb does not expose --no-daemon, and the environment
    # variable previously used here is ignored by NodeStrategy. Inspect actions
    # with the same standard rclpy graph API, not another session's stale daemon.
    wait_for(lambda: ('/z70/arm/step', ['z70_interfaces/action/ArmStep']) in get_action_names_and_types(system['node']))
    env = dict(os.environ, ROS_DOMAIN_ID='92')
    commands = [
        ['ros2', 'topic', 'list', '-t', '--no-daemon', '--spin-time', '1'],
        ['ros2', 'param', 'get', '--no-daemon', '--spin-time', '1', '/z70_driver', 'arm_estimate_path'],
    ]
    outputs = []
    for command in commands:
        result = subprocess.run(command, env=env, text=True, capture_output=True, timeout=15)
        assert result.returncode == 0, result.stderr
        outputs.append(result.stdout)
    assert '/z70/commands [z70_interfaces/msg/CommandEvent]' in outputs[0]
    assert 'String value is:' in outputs[1]
