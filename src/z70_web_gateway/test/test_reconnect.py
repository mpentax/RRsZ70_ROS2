"""Additional ROS disconnect and lost-response paths, no real device access."""
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from z70_interfaces.action import Pick
from z70_interfaces.srv import RawRpc, Stop
from z70_web_gateway.gateway import Gateway, ApiError


def wait(predicate, seconds=5):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        result=predicate()
        if result:return result
        time.sleep(.02)
    assert predicate()


@pytest.fixture
def graph():
    rclpy.init(domain_id=95)
    gateway=Gateway();peer=rclpy.create_node('phase4_disconnected_peer')
    executor=MultiThreadedExecutor(num_threads=6)
    executor.add_node(gateway);executor.add_node(peer)
    thread=threading.Thread(target=executor.spin);thread.start()
    try:yield gateway,peer
    finally:
        executor.shutdown(timeout_sec=5);thread.join(5)
        gateway.destroy_node();peer.destroy_node();rclpy.shutdown()
        assert not thread.is_alive()


def test_actual_service_discovery_loss_and_reconnect_never_replays(graph):
    gateway,peer=graph;calls=[]
    def rpc(request,response):
        calls.append(request.method);response.outcome.dispatch=2;response.outcome.error_code='TIMEOUT';return response
    with pytest.raises(ApiError) as error:gateway.route('POST','/api/ros/rpc',{'method':'app_grip','params':{}})
    assert error.value.body['dispatch']=='not_sent'
    server=peer.create_service(RawRpc,'/z70/driver/raw_rpc',rpc)
    wait(gateway.ros_clients['rpc'].service_is_ready)
    response=gateway.route('POST','/api/ros/rpc',{'method':'app_grip','params':{}})[1]
    assert response['outcome']['dispatch']==2 and calls==['app_grip']
    peer.destroy_service(server);wait(lambda:not gateway.ros_clients['rpc'].service_is_ready())
    with pytest.raises(ApiError):gateway.route('POST','/api/ros/rpc',{'method':'app_arm_move','params':{}})
    assert calls==['app_grip']
    server=peer.create_service(RawRpc,'/z70/driver/raw_rpc',rpc);wait(gateway.ros_clients['rpc'].service_is_ready)
    # Discovery resumes reads; it does not resend either uncertain or unavailable writes.
    assert calls==['app_grip']
    gateway.route('POST','/api/ros/rpc',{'method':'app_open_gripper','params':{}})
    assert calls==['app_grip','app_open_gripper']
    peer.destroy_service(server)


def test_acceptance_timeout_retains_uuid_and_reconciles_late_result_once(graph):
    gateway,peer=graph;goals=[]
    def accept(request):time.sleep(3.2);return GoalResponse.ACCEPT
    def execute(handle):
        goals.append(bytes(handle.goal_id.uuid));handle.succeed();return Pick.Result(dry_run_valid=True)
    server=ActionServer(peer,Pick,'/z70/pick',goal_callback=accept,execute_callback=execute,callback_group=gateway.group)
    try:
        wait(gateway.actions['pick'].server_is_ready)
        with pytest.raises(ApiError) as error:
            gateway.start({'kind':'pick','goal':{'color':'yellow','dry_run':True}})
        assert error.value.status==504 and error.value.body['dispatch']=='unknown'
        ident=error.value.body['id']
        assert gateway.task(ident)['status']=='unknown'
        wait(lambda:gateway.task(ident)['status']=='succeeded')
        assert len(goals)==1 and len(gateway.tasks)==1
        assert gateway.task(ident)['result']['dry_run_valid'] is True
    finally:server.destroy()


def test_standard_map_and_calibration_keep_source_fields_and_latched_identity(graph):
    from nav_msgs.msg import OccupancyGrid
    from z70_interfaces.msg import CalibrationState
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    gateway,peer=graph
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    pub=peer.create_publisher(OccupancyGrid,'/map',qos)
    cal_pub=peer.create_publisher(CalibrationState,'/z70/chassis_calibration',qos)
    wait(lambda:pub.get_subscription_count()>0 and cal_pub.get_subscription_count()>0)
    grid=OccupancyGrid();grid.info.width=2;grid.info.height=2;grid.info.resolution=.05;grid.data=[-1,0,50,100]
    grid.header.stamp.sec=123;grid.header.frame_id='map';pub.publish(grid)
    cal=CalibrationState();cal.evidence.sequence=19;cal.evidence.stale=True;cal.evidence.received_at.sec=456;cal_pub.publish(cal)
    wait(lambda:'map' in gateway.samples and 'calibration' in gateway.samples)
    first=gateway.route('GET','/api/ros/map',{})[1]['map']
    second=gateway.route('GET','/api/ros/map',{})[1]['map']
    assert first==second and first['data']==[-1,0,50,100] and first['header']['stamp']['sec']==123
    state=gateway.route('GET','/api/ros/state',{})[1]
    assert state['calibration']['evidence']['sequence']==19 and state['calibration']['evidence']['stale']
    assert state['calibration']['evidence']['received_at']['sec']==456



def test_invalid_startup_configuration_is_bounded_and_nonzero(tmp_path):
    import os
    import subprocess
    result=subprocess.run(['ros2','run','z70_web_gateway','gateway','--ros-args','-p',
        'token_file:='+str(tmp_path/'missing-fixture-token')],
        env=dict(os.environ,ROS_DOMAIN_ID='97'),text=True,capture_output=True,timeout=15)
    assert result.returncode != 0
    assert 'Gateway unavailable: check port and server-side token_file parameter.' in result.stderr
    assert 'Traceback' not in result.stderr and 'missing-fixture-token' not in result.stderr
