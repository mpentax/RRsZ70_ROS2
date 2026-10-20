"""Stamp-exact /camera/front/image_raw + /odom -> appearance-only floor mosaic."""
from collections import OrderedDict
import json
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from z70_odometry.chassis_calibrator import quat_yaw
from z70_odometry.floor_vo import FloorCamera
from z70_odometry.mosaic import FloorMosaic
from z70_odometry.calibration_state import calibrated_camera_x
from z70_interfaces.msg import CalibrationState
from z70_odometry.resources import front_calibration_path, initial_camera_x_m
from z70_driver.frame_subscription import MatchedFrames


def stamp_ns(msg):
    return msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec


class FloorMosaicNode(Node):
    def __init__(self):
        super().__init__('z70_floor_mosaic')
        cam=FloorCamera.load(self.declare_parameter('calibration',str(front_calibration_path())).value)
        params={name:float(self.declare_parameter(name,value).value) for name,value in
                [('resolution',.005),('size_m',4.),('max_range',1.2),('camera_x_m',initial_camera_x_m())]}
        self.mosaic=FloorMosaic(cam,**params)
        self.images=OrderedDict(); self.odoms=OrderedDict()
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(Image,'/z70/floor_mosaic',qos)
        self.mask_pub=self.create_publisher(Image,'/z70/floor_mosaic_mask',qos)
        self.info_pub=self.create_publisher(CameraInfo,'/z70/camera_info',qos)
        self.status_pub=self.create_publisher(String,'/z70/floor_mosaic_status',10)
        self.frames=MatchedFrames(self,'front',lambda m:self.add(self.images,m))
        self.create_subscription(Odometry,'/odom',lambda m:self.add(self.odoms,m),10)
        self.create_subscription(CalibrationState,'/z70/chassis_calibration',self.on_calibration,qos)
        self._publish_outputs(self.get_clock().now().to_msg())

    def _camera_info(self, stamp):
        fx = fy = 0.01 / self.mosaic.resolution
        cx = cy = self.mosaic.n / 2.0
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'odom'
        info.width = info.height = self.mosaic.n
        info.distortion_model = 'plumb_bob'
        info.d = [0., 0., 0., 0., 0.]
        info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.]
        return info

    def _publish_outputs(self, stamp):
        self.info_pub.publish(self._camera_info(stamp))
        for data,encoding,pub in [(self.mosaic.image(),'bgr8',self.pub),
                                 ((self.mosaic.count>0).astype(np.uint8)*255,'mono8',self.mask_pub)]:
            out=Image(); out.header.stamp=stamp; out.header.frame_id='odom'
            out.height,out.width=data.shape[:2]; out.encoding=encoding
            out.step=out.width*(3 if encoding=='bgr8' else 1); out.data=data.tobytes(); pub.publish(out)

    def on_calibration(self,msg):
        value=calibrated_camera_x(msg)
        if value is not None: self.mosaic.cam_x=value

    def add(self,cache,msg):
        key=stamp_ns(msg); cache[key]=msg
        while len(cache)>20: cache.popitem(last=False)
        if key not in self.images or key not in self.odoms: return
        image=self.images.pop(key); odom=self.odoms.pop(key)
        # A rejected VO frame publishes held pose with large covariance. Do not
        # project this moving view at the previous pose and smear the map.
        if odom.header.frame_id!='odom' or odom.pose.covariance[0]>1000: return
        if image.encoding!='bgr8': return
        if image.width!=self.mosaic.cam.W or image.height!=self.mosaic.cam.H: return
        if image.step<image.width*3 or len(image.data)!=image.height*image.step: return
        view=np.frombuffer(image.data,np.uint8).reshape(image.height,image.step)[:,:image.width*3].reshape(image.height,image.width,3)
        pose=odom.pose.pose
        stats=self.mosaic.update(view,(pose.position.x,pose.position.y,quat_yaw(pose.orientation)))
        self._publish_outputs(image.header.stamp)
        stats.update(stamp_ns=key,resolution=self.mosaic.resolution,origin_xy=list(self.mosaic.origin),
                     frame_id='odom',width=self.mosaic.n,height=self.mosaic.n)
        self.status_pub.publish(String(data=json.dumps(stats)))


def main():
    rclpy.init(); node=FloorMosaicNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally: node.destroy_node(); rclpy.try_shutdown()
