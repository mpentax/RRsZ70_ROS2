"""Standard ros2 launch against a fake backend, isolated DDS and test-owned data."""
import os
import signal
import subprocess
import time
import cv2
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid
from z70_interfaces.msg import RobotState, CalibrationState, OdometryStatus
from z70_driver.driver import LATCHED
from test_shared_driver import backend


def test_standard_launch_acquires_fake_images_maps_state_and_shuts_down(backend,tmp_path):
    backend['image_shape']=(1200,1600)  # Match unchanged installed camera resources.
    token=tmp_path/'synthetic-token';token.write_text('offline-launch-fixture');token.chmod(0o600)
    env=dict(os.environ,ROS_DOMAIN_ID='94',XDG_DATA_HOME=str(tmp_path/'application-data'),
             ROS_LOG_DIR=str(tmp_path/'ros-log'))
    env.pop('Z70_REPO',None)
    context=Context();rclpy.init(context=context,domain_id=94)
    node=rclpy.create_node('offline_standard_launch_probe',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    states=[];images=[];maps=[];calibrations=[];odometry=[]
    node.create_subscription(RobotState,'/z70/state',states.append,LATCHED)
    node.create_subscription(CompressedImage,'/camera/front/image_raw/compressed',images.append,2)
    node.create_subscription(OccupancyGrid,'/map',maps.append,LATCHED)
    node.create_subscription(CalibrationState,'/z70/chassis_calibration',calibrations.append,LATCHED)
    node.create_subscription(OdometryStatus,'/z70/odom_status',odometry.append,10)
    log=(tmp_path/'standard-launch.log').open('w')
    process=subprocess.Popen(['ros2','launch','z70_bringup','bringup.launch.py',
        'origin:='+backend['origin'],'token_file:='+str(token),'foxglove_port:=0'],
        cwd=tmp_path,env=env,stdout=log,stderr=log,start_new_session=True)
    forced=False
    try:
        deadline=time.monotonic()+25
        while time.monotonic()<deadline and not (states and images and maps and calibrations and odometry):
            assert process.poll() is None,'Standard launch exited during fake acquisition'
            executor.spin_once(timeout_sec=.1)
        assert states and images and maps and calibrations and odometry, 'Required launched ROS observation topics missing'
        assert odometry[-1].evidence.valid and odometry[-1].active_camera_x_known
        assert not odometry[-1].covariance_calibrated
        assert cv2.imdecode(np.frombuffer(bytes(images[-1].data),np.uint8),cv2.IMREAD_COLOR).shape[:2]==(1200,1600)
        assert backend['frames']>0
        # This entire graph points only at the fixture, never the live backend.
        assert all(path!='/api/driver/rpc' or body['method']=='get_room_mapping'
                   for method,path,body in backend['calls'])
    finally:
        if process.poll() is None:process.send_signal(signal.SIGINT)
        try:process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            forced=True;os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close();executor.shutdown();node.destroy_node();context.shutdown()
    assert not forced,'Owned offline launch did not finish bounded standard shutdown'
    assert process.returncode==0
