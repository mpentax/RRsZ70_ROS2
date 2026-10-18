"""Actual RSP trees: display-only folded poses cannot populate planning joints."""
import os
from pathlib import Path
import subprocess
import time
import yaml
import xacro
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer,TransformListener
from z70_interfaces.srv import SetArmAnchor
from test_command_api import system,service,resolved,wait_for


def test_real_rsp_tf_remains_complete_while_typed_arm_estimate_tracks_planning_validity(system,tmp_path):
    description=xacro.process_file(str(Path(get_package_share_directory('z70_description'))/'urdf/z70.urdf.xacro')).toxml()
    path=tmp_path/'robot.yaml';path.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':description}}}))
    executable=Path(get_package_prefix('robot_state_publisher'))/'lib/robot_state_publisher/robot_state_publisher'
    output=(tmp_path/'rsp.log').open('w');processes=[]
    buffer=Buffer();listener=TransformListener(buffer,system['node']);joints=[]
    sub=system['node'].create_subscription(JointState,'/joint_states',joints.append,10)
    try:
        processes.append(subprocess.Popen([str(executable),'--ros-args','--params-file',str(path)],
            env=dict(os.environ,ROS_DOMAIN_ID='92'),stdout=output,stderr=output))
        wait_for(lambda:system['driver'].pub_js.get_subscription_count()>0)
        resolved(service(system,SetArmAnchor,'/z70/arm/anchor',SetArmAnchor.Request(anchor=0)))
        wait_for(lambda:buffer.can_transform('base_footprint','arm_wrist_link',Time()) and
                        buffer.can_transform('base_footprint','jaw_left',Time()) and
                        buffer.can_transform('base_footprint','arm_compartment_lid',Time()) and bool(joints))
        assert set(joints[-1].name)=={'m3_joint','m4_joint','m5_joint','m6_left_joint','m6_right_joint','shoulder_lift','shoulder_yaw','arm_lid'}
        reply=resolved(service(system,SetArmAnchor,'/z70/arm/anchor',SetArmAnchor.Request(anchor=2)))
        assert reply.estimate.validity.value==1 and reply.estimate.source==3
        assert reply.estimate.joints.name==['m3_joint','m4_joint','m5_joint']
        reply=resolved(service(system,SetArmAnchor,'/z70/arm/anchor',SetArmAnchor.Request(anchor=0)))
        assert reply.estimate.validity.value==0 and not reply.estimate.joints.name
    finally:
        system['node'].destroy_subscription(sub)
        listener.unregister()
        for process in processes:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        output.close()
