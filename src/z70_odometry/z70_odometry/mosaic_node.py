"""Stamp-exact /camera/front/image_raw + /odom -> appearance-only floor mosaic."""
from collections import OrderedDict
import json
import math
import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from z70_odometry.chassis_calibrator import calibrated_camera_x, front_calibration_path, initial_camera_x_m, quat_yaw, stamp_ns
from z70_odometry.floor_vo import FloorCamera, decode_bgr8
from z70_interfaces.msg import CalibrationState
from z70_driver.frame_subscription import MatchedFrames


class FloorMosaic:
    def __init__(self, cam, resolution=.005, size_m=4., max_range=1.2, camera_x_m=None):
        if camera_x_m is None: camera_x_m=initial_camera_x_m()
        if resolution <= 0 or size_m <= 0 or max_range <= .06:
            raise ValueError('positive map size/resolution and range > 0.06 required')
        self.cam, self.resolution, self.max_range, self.cam_x = cam, resolution, max_range, camera_x_m
        self.n = int(math.ceil(size_m / resolution))
        self.origin = (-self.n * resolution / 2, -self.n * resolution / 2)
        self.mean = np.zeros((self.n, self.n, 3), np.float32)
        self.count = np.zeros((self.n, self.n), np.uint32)
        yy, xx = np.meshgrid(np.arange(self.n), np.arange(self.n), indexing='ij')
        self.world = np.stack([self.origin[0]+(xx.ravel()+.5)*resolution,
                               self.origin[1]+(yy.ravel()+.5)*resolution],1)

    def update(self, image, pose):
        if image.shape != (self.cam.H,self.cam.W,3):
            raise ValueError('image dimensions must match camera calibration')
        x,y,th = pose
        c,s = math.cos(th), math.sin(th)
        R=np.array([[c,-s],[s,c]])
        camera_origin=np.array([x,y])+R@np.array([self.cam_x,0])
        local=(self.world-camera_origin)@R
        r=np.linalg.norm(local,axis=1)
        candidate=np.flatnonzero((r>.06)&(r<self.max_range)&(local[:,0]>0))
        uv,ok=self.cam.floor_to_pixels(local[candidate])
        ok &= (uv[:,0]>=1)&(uv[:,0]<self.cam.W-2)&(uv[:,1]>=1)&(uv[:,1]<self.cam.H-2)
        ids=candidate[ok]; uv=uv[ok].astype(np.float32)
        sample=np.empty((len(ids),3),np.float32)
        for start in range(0,len(ids),30000):
            part=uv[start:start+30000]
            sample[start:start+len(part)]=cv2.remap(
                image,part[:,0].reshape(-1,1),part[:,1].reshape(-1,1),cv2.INTER_LINEAR).reshape(-1,3)
        supported=np.any(sample!=0,axis=1)
        ids,sample=ids[supported],sample[supported]
        count=self.count.ravel(); mean=self.mean.reshape(-1,3)
        overlap=count[ids]>0
        ncc=None
        if overlap.sum()>20:
            a=mean[ids[overlap]].mean(1); b=sample[overlap].mean(1)
            a=a-a.mean(); b=b-b.mean(); denom=np.linalg.norm(a)*np.linalg.norm(b)
            if denom>1e-6: ncc=float(a@b/denom)
        mean[ids]+=(sample-mean[ids])/(count[ids,None]+1)
        count[ids]+=1
        return {'observed_cells':int(len(ids)), 'overlap_cells':int(overlap.sum()),
                'overlap_ncc':ncc,'total_cells':int(np.count_nonzero(count))}

    def image(self):
        return np.clip(np.rint(self.mean),0,255).astype(np.uint8)


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
        view=decode_bgr8(image,self.mosaic.cam)
        if view is None: return
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
