"""Gripper camera, TCP, and marker TF verification through robot_state_publisher."""
import numpy as np
import pytest
from rclpy.time import Time
from test_wrist_tf import publish_and_lookup, quat_matrix, tf_robot
from z70_driver.driver import app_to_urdf
from z70_manipulation import planner

@pytest.mark.parametrize('m3,m4', [(170,65),(200,95),(240,105)])
@pytest.mark.parametrize('m5',[0,35,155])
def test_measured_gripper_tf(tf_robot,m3,m4,m5):
    _,buffer,_,_=tf_robot
    tr=publish_and_lookup(tf_robot, app_to_urdf(m3,m4,m5,90), 'gripper_camera_optical_frame')
    expected,R=planner.camera_pose((m3,m4,m5))
    p=tr.transform.translation; actual=quat_matrix(tr.transform.rotation)
    assert (p.x,p.y,p.z)==pytest.approx(expected,abs=1e-8)
    assert actual==pytest.approx(R,abs=1e-8)
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
    optical=planner.hand.rpy(planner._CAD['camera_rpy'])@np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    marker_rotation=planner.hand.rpy(planner._CAD['marker_rpy'])
    marker_in_optical=optical.T@(marker-camera)
    actual_marker=np.array([p.x,p.y,p.z])+actual@marker_in_optical
    expected_marker=planner.measured_point((m3,m4),Q@marker)
    assert actual_marker==pytest.approx(expected_marker,abs=1e-8)
    marker_tf=buffer.lookup_transform('gripper_camera_optical_frame','pincher_marker_68',Time()).transform
    assert (marker_tf.translation.x,marker_tf.translation.y,marker_tf.translation.z)==pytest.approx(marker_in_optical,abs=1e-8)
    assert quat_matrix(marker_tf.rotation)==pytest.approx(optical.T@marker_rotation,abs=1e-8)
    a=np.deg2rad(m4-m3+90);ca,sa=np.cos(a),np.sin(a)
    cy,sy=np.cos(planner.YAW),np.sin(planner.YAW)
    link2_rotation=np.array([[cy*ca,-sy,-cy*sa],[sy*ca,cy,-sy*sa],[sa,0,ca]])
    assert actual@optical.T@marker_rotation==pytest.approx(link2_rotation@Q@marker_rotation,abs=1e-8)
