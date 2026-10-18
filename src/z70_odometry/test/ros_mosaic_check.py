"""Isolated-domain offline ROS integration: real image transport -> VO -> mosaic.
Starts only owned ROS nodes; never bridge, HTTP or robot services.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from z70_odometry.floor_vo import FloorCamera,render_floor_view
REPO=Path(__file__).resolve().parents[3]

def main():
    os.environ['ROS_DOMAIN_ID']='87'
    cal=REPO/'calibration/front-camera/front_camera.json'
    cam=FloorCamera.load(cal)
    image=cv2.imread(str(REPO/'calibration/front-camera/raw/frame-3-telegram.jpg'))
    processes=[]
    rclpy.init(); node=Node('offline_mosaic_check')
    received={}; mosaics={}; odoms={}; mask=[]
    def status(msg):
        data=json.loads(msg.data); received[data['stamp_ns']]=data
    node.create_subscription(String,'/z70/floor_mosaic_status',status,10)
    node.create_subscription(Image,'/z70/floor_mosaic',lambda m:mosaics.__setitem__(m.header.stamp.sec,m),10)
    node.create_subscription(Image,'/z70/floor_mosaic_mask',lambda m:mask.append(m),10)
    node.create_subscription(Odometry,'/odom',lambda m:odoms.__setitem__(m.header.stamp.sec,m),10)
    pub=node.create_publisher(Image,'/camera/front/image_raw',10)
    def until(predicate,seconds=15):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if predicate(): return
        raise AssertionError('ROS timeout; child exits '+str([p.poll() for p in processes]))
    try:
        prefix=subprocess.check_output(['ros2','pkg','prefix','z70_odometry'],text=True).strip()
        for executable in ['floor_odometry','floor_mosaic']:
            processes.append(subprocess.Popen([str(Path(prefix)/'lib/z70_odometry'/executable),
                '--ros-args','-p',f'calibration:={cal}']))
        until(lambda:pub.get_subscription_count()==2)
        poses=[(0,0,0),(.02,0,math.radians(5)),(.04,.003,math.radians(10)),
               (.06,.003,math.radians(5)),(.04,.003,0)]
        stats=[]; translation_errors=[]; yaw_errors=[]
        for i,(x,y,th) in enumerate(poses,1):
            tc=(x+.16*(math.cos(th)-1),y+.16*math.sin(th),th)
            view=image if i==1 else render_floor_view(image,cam,tc)
            msg=Image(); msg.header.stamp.sec=i; msg.header.frame_id='front_camera_optical'
            msg.height,msg.width=view.shape[:2]; msg.encoding='bgr8'; msg.step=msg.width*3; msg.data=view.tobytes()
            if i==2:
                msg.step+=16
                padded=np.zeros((msg.height,msg.step),np.uint8)
                padded[:,:msg.width*3]=view.reshape(msg.height,msg.width*3)
                msg.data=padded.tobytes()
            pub.publish(msg)
            until(lambda:i*10**9 in received and i in mosaics and i in odoms)
            st=received[i*10**9]; stats.append(st)
            if i>1:
                assert st['overlap_ncc']>.93,st
                print('frame overlap', i, st['overlap_ncc'],flush=True)
            o=odoms[i].pose.pose
            yaw=2*math.atan2(o.orientation.z,o.orientation.w)
            translation_errors.append(math.hypot(o.position.x-x,o.position.y-y))
            yaw_errors.append(abs(yaw-th))
            assert translation_errors[-1]<.002,(i,o,x,y)
            assert yaw_errors[-1]<math.radians(.3),(i,yaw,th)
        until(lambda:len(mask)>=len(poses))
        last=mosaics[len(poses)]
        assert last.header.frame_id=='odom' and last.encoding=='bgr8' and last.width==800
        assert np.count_nonzero(np.frombuffer(mask[-1].data,np.uint8))>50000
        preview=np.frombuffer(last.data,np.uint8).reshape(last.height,last.width,3)
        cv2.imwrite(str(Path(__file__).with_name('ros-mosaic-preview.png')),preview)
        print(json.dumps({'frames':len(poses),'stats':stats,'final_pose':{'x':o.position.x,'y':o.position.y,'yaw':yaw}},indent=2))
        # Out-of-envelope movement: odom must reject it and the mosaic must not fuse a held pose.
        th=math.radians(40); view=render_floor_view(image,cam,(.04+.16*(math.cos(th)-1),.003+.16*math.sin(th),th))
        msg.header.stamp.sec=6; msg.data=view.tobytes(); pub.publish(msg)
        until(lambda:6 in odoms)
        assert odoms[6].pose.covariance[0]>1000
        deadline=time.monotonic()+1
        while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
        assert 6 not in mosaics, 'rejected held pose smeared into mosaic'
        # Return to a valid view: VO retains last accepted reference and recovers full displacement.
        msg.header.stamp.sec=7; msg.data=image.tobytes(); pub.publish(msg)
        until(lambda:7 in mosaics and 7 in odoms)
        o=odoms[7].pose.pose
        assert math.hypot(o.position.x,o.position.y)<.002
        print('rejection skipped; accepted-reference recovery passed')
        result={'frames':len(poses),'stats':stats,
                'max_translation_error_mm':max(translation_errors)*1000,
                'max_yaw_error_deg':math.degrees(max(yaw_errors)),
                'min_overlap_ncc':min(s['overlap_ncc'] for s in stats if s['overlap_ncc'] is not None),
                'rejected_frame_skipped':True,'accepted_reference_recovered':True}
        Path(__file__).with_name('ros-mosaic-results.json').write_text(json.dumps(result,indent=2)+'\n')
        print('ROS measured summary:',json.dumps({k:v for k,v in result.items() if k!='stats'}))
    finally:
        for p in processes: p.terminate()
        for p in processes:
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired: p.kill(); p.wait()
        node.destroy_node(); rclpy.try_shutdown()

if __name__=='__main__': main()
