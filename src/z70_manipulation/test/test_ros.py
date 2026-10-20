import threading
import time
from test_executor import fake, ROOT
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor


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
    from z70_manipulation.node import PickServer
    from z70_interfaces.action import Pick, Stack, DeployArm, AlignToTarget, MoveArm, GraspAndLift, ReleaseAndStow
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray
    origin, token, events, state = fake
    rclpy.init(domain_id=92)
    server = PickServer(parameter_overrides=[
        rclpy.parameter.Parameter('repo', value=str(ROOT)),
        rclpy.parameter.Parameter('origin', value=origin),
        rclpy.parameter.Parameter('token_file', value=str(token)),
        rclpy.parameter.Parameter('log_root', value=str(tmp_path)),
    ])
    # Speed up tests by shortening sleep in server runs
    client_node = rclpy.create_node('pick_test_client')
    received = []
    markers = []
    client_node.create_subscription(String, '/z70/objects/json', lambda m: received.append(m.data), 10)
    client_node.create_subscription(MarkerArray, '/z70/objects', lambda m: markers.append(m), 10)
    pick_client = ActionClient(client_node, Pick, '/z70/pick')
    stack_client = ActionClient(client_node, Stack, '/z70/stack')
    deploy_client = ActionClient(client_node, DeployArm, '/z70/arm/deploy')
    align_client = ActionClient(client_node, AlignToTarget, '/z70/align_target')
    move_client = ActionClient(client_node, MoveArm, '/z70/arm/move')
    grasp_client = ActionClient(client_node, GraspAndLift, '/z70/arm/grasp_and_lift')
    release_client = ActionClient(client_node, ReleaseAndStow, '/z70/arm/release_and_stow')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        result = _send_and_wait(pick_client, Pick.Goal(color='yellow', dry_run=True))
        assert result.dry_run_valid and not result.verified_hold and result.outcome.error_code == 'NONE'
        assert received and markers

        deploy_res = _send_and_wait(deploy_client, DeployArm.Goal(settle=False))
        assert deploy_res.ready

        align_res = _send_and_wait(align_client, AlignToTarget.Goal(color='yellow'))
        assert align_res.aligned

        move_res = _send_and_wait(move_client, MoveArm.Goal(m3_deg=170., m4_deg=45., m5_deg=0.))
        assert move_res.steps_executed == 2

        grasp_res = _send_and_wait(grasp_client, GraspAndLift.Goal(lift_steps=1, verify_color=''))
        assert grasp_res.outcome.error_code == 'NONE'

        release_res = _send_and_wait(release_client, ReleaseAndStow.Goal(put_down=False), timeout=20.)
        assert release_res.stowed

        stack_res = _send_and_wait(stack_client, Stack.Goal(base_color='yellow', middle_color='blue', top_color='purple', dry_run=True))
        assert not stack_res.stack_verified and stack_res.evidence_uri
    finally:
        for c in (pick_client, stack_client, deploy_client, align_client, move_client, grasp_client, release_client):
            c.destroy()
        executor.shutdown()
        thread.join()
        server.destroy_node()
        client_node.destroy_node()
        rclpy.try_shutdown()
