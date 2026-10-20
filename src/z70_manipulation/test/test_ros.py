import threading
import time
from test_executor import fake, ROOT
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from z70_interfaces.action import AlignToTarget, DeployArm, GraspAndLift, MoveArm, Pick, ReleaseAndStow, Stack
from z70_manipulation.cli import run, run_stack
from z70_manipulation.node import PickServer


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
        assert not stack_res.stack_verified and stack_res.evidence_uri
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
