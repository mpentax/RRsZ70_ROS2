"""Gripper camera, TCP, and marker TF verification through robot_state_publisher."""
import time
import numpy as np
import pytest
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from test_wrist_tf import tf_robot
from z70_driver.driver import app_to_urdf
from z70_manipulation import planner

@pytest.mark.parametrize('m3,m4', [(170,65),(200,95),(240,105)])
@pytest.mark.parametrize('m5',[0,35,155])
def test_measured_gripper_tf(tf_robot,m3,m4,m5):
    node,buffer,publisher,process=tf_robot
    angles=app_to_urdf(m3,m4,m5,90)
    msg=JointState(name=list(angles),position=list(angles.values()))
    started=node.get_clock().now(); deadline=time.monotonic()+8
    tr=None
    while time.monotonic()<deadline:
        assert process.poll() is None
        msg.header.stamp=node.get_clock().now().to_msg(); publisher.publish(msg)
        rclpy.spin_once(node,timeout_sec=.1)
        try:
            candidate=buffer.lookup_transform('base_footprint','gripper_camera_optical_frame',Time())
            if Time.from_msg(candidate.header.stamp)>=started: tr=candidate; break
        except TransformException: pass
    assert tr is not None, 'measured gripper camera TF missing'
    expected,R=planner.camera_pose((m3,m4,m5))
    p=tr.transform.translation;q=tr.transform.rotation
    assert (p.x,p.y,p.z)==pytest.approx(expected,abs=1e-8)
    x,y,z,w=q.x,q.y,q.z,q.w
    actual=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    assert actual==pytest.approx(R,abs=1e-8)
    # CAD wrist/camera geometry and the retained optical contact surrogate
    # remain separate; the legacy 0.205m offset below records its old gauge.
    cfg=planner._CALIBRATION
    camera=np.array([planner.CAD_L2,0,0])+np.array(planner._CAD['camera_from_wrist_m'],float)
    marker=np.array([planner.CAD_L2,0,0])+np.array(planner._CAD['marker_from_wrist_m'],float)
    jaw=np.array(cfg['effective_jaw_from_elbow_m'])
    assert jaw==pytest.approx(np.array(cfg['marker_effective_from_elbow_m'])+np.array(cfg['marker_rotation_link2'])@cfg['jaw_region_in_marker_m'],abs=1e-12)
    assert cfg['jaw_nominal_wrist_offset_m']==pytest.approx(jaw-np.array([.205,0,0]),abs=1e-12)
    roll=np.deg2rad(m5);c,s=np.cos(roll),np.sin(roll)
    Q=np.array([[1,0,0],[0,c,-s],[0,s,c]])
    tcp=buffer.lookup_transform('base_footprint','gripper_tcp',Time()).transform.translation
    expected_jaw=planner.measured_point((m3,m4),Q@jaw)
    assert (tcp.x,tcp.y,tcp.z)==pytest.approx(expected_jaw,abs=1e-8)
    # Recover full registration through real optical TF, then compare the actual
    # published marker frame, including orientation, through the same M5 chain.
    optical=planner.hand.rpy(planner._CAD['camera_rpy'])@np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    marker_rotation=planner.hand.rpy(planner._CAD['marker_rpy'])
    marker_in_optical=optical.T@(marker-camera)
    actual_marker=np.array([p.x,p.y,p.z])+actual@marker_in_optical
    expected_marker=planner.measured_point((m3,m4),Q@marker)
    assert actual_marker==pytest.approx(expected_marker,abs=1e-8)
    marker_tf=buffer.lookup_transform('gripper_camera_optical_frame','pincher_marker_68',Time()).transform
    assert (marker_tf.translation.x,marker_tf.translation.y,marker_tf.translation.z)==pytest.approx(marker_in_optical,abs=1e-8)
    q=marker_tf.rotation;x,y,z,w=q.x,q.y,q.z,q.w
    marker_actual_R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                             [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                             [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    assert marker_actual_R==pytest.approx(optical.T@marker_rotation,abs=1e-8)
    a=np.deg2rad(m4-m3+90);ca,sa=np.cos(a),np.sin(a)
    cy,sy=np.cos(planner.YAW),np.sin(planner.YAW)
    link2_rotation=np.array([[cy*ca,-sy,-cy*sa],[sy*ca,cy,-sy*sa],[sa,0,ca]])
    assert actual@optical.T@marker_rotation==pytest.approx(link2_rotation@Q@marker_rotation,abs=1e-8)
