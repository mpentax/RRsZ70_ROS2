"""Online chassis calibration node stream consumption and persistence."""
import json
import math
import os
import sys
import subprocess
import time


def test_ros_calibrator_consumes_stream_and_persists(tmp_path):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from nav_msgs.msg import Odometry
    from z70_interfaces.msg import CommandEvent, CalibrationState, OdometryStatus
    from rclpy.qos import QoSProfile, DurabilityPolicy
    context=Context(); rclpy.init(args=[],context=context,domain_id=95)
    node=Node('synthetic_calibration_stream',context=context)
    executor=SingleThreadedExecutor(context=context)
    odom=node.create_publisher(Odometry,'/odom',100)
    status=node.create_publisher(OdometryStatus,'/z70/odom_status',100)
    commands=node.create_publisher(CommandEvent,'/z70/commands',256)
    results=[]
    node.create_subscription(CalibrationState,'/z70/chassis_calibration',results.append,
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    path=tmp_path/'online.json'; log=(tmp_path/'calibrator.log').open('w')
    process=subprocess.Popen([sys.executable,'-m','z70_odometry.chassis_calibrator','--ros-args','-p',f'output_file:={path}','-p','seed_directory:=""'],env=dict(os.environ,ROS_DOMAIN_ID='95'),stdout=log,stderr=log)
    try:
        deadline=time.monotonic()+8
        while (odom.get_subscription_count()==0 or status.get_subscription_count()==0 or commands.get_subscription_count()==0) and time.monotonic()<deadline:
            rclpy.spin_once(node,executor=executor,timeout_sec=.1)
        assert commands.get_subscription_count()>0
        base=time.time_ns()-6_000_000_000; start=base+1_000_000_000
        for i in range(17):
            stamp=base+i*250_000_000; fraction=max(0,min(1,(stamp-start)/600_000_000)); yaw=.5*.6*.65*fraction
            msg=Odometry(); msg.header.stamp.sec=stamp//10**9; msg.header.stamp.nanosec=stamp%10**9
            msg.pose.pose.position.x=(.28-.16)*(math.cos(yaw)-1)
            msg.pose.pose.position.y=(.28-.16)*math.sin(yaw)
            msg.pose.pose.orientation.z=math.sin(yaw/2); msg.pose.pose.orientation.w=math.cos(yaw/2)
            quality=OdometryStatus(pair_accepted=True,active_camera_x_known=True,active_camera_x_m=.16)
            quality.header=msg.header;quality.evidence.valid=True;quality.validity.value=1
            odom.publish(msg); status.publish(quality)
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline: rclpy.spin_once(node,executor=executor,timeout_sec=.05)
        command=CommandEvent(operation='app_rc_move',phase=1)
        command.evidence.session_id.uuid=[1]*16;command.evidence.sequence=1
        command.evidence.source_time_known=True;command.evidence.source_clock=1
        command.evidence.source_at.sec=start//10**9;command.evidence.source_at.nanosec=start%10**9
        command.outcome.attempt_id.uuid=[2]*16
        command.requested_twist.angular.z=.5;command.requested_duration.nanosec=600_000_000
        commands.publish(command)
        command.phase=2;command.evidence.sequence=2
        command.outcome.dispatch=1;command.outcome.response=1
        commands.publish(command)
        deadline=time.monotonic()+5
        while not any(r.sample_count==1 for r in results) and time.monotonic()<deadline: rclpy.spin_once(node,executor=executor,timeout_sec=.1)
        measured=next(r for r in results if r.sample_count==1)
        assert measured.yaw_positive_known and abs(measured.yaw_positive_gain-.65)<1e-6
        assert measured.camera_x_known and abs(measured.camera_x_m-.28)<1e-6
        assert json.loads(path.read_text())['n']==1
    finally:
        process.terminate()
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill(); process.wait()
        executor.shutdown(); node.destroy_node(); context.shutdown(); log.close()
    assert process.poll() is not None
