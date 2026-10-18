"""Read-only full-bringup validation, isolated ROS domain; always cleans owned processes.
Run from the repository root after build: pixi run bash -c 'source install/setup.bash; python src/z70_mapping/test/live_check.py'
Private renders/clouds remain in ignored calibration/mapping/. No robot motion calls.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

REPO = Path(__file__).resolve().parents[3]
OUT = REPO/'calibration/mapping'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=90.)
    args = parser.parse_args()
    OUT.mkdir(exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = '94'
    os.environ['Z70_REPO'] = str(REPO)
    log = open(OUT/'bringup.log','w')
    process = subprocess.Popen(['ros2','launch','z70_bringup','bringup.launch.py',
                                'depth:=true','foxglove_port:=18768'],
                               stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    node = None
    try:
        rclpy.init(); node = Node('mapping_live_validation')
        durable = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        latest, times, transforms = {}, {}, {}
        def receive(topic):
            def f(msg):
                latest[topic] = msg
                if topic == '/tf':
                    for t in msg.transforms:
                        transforms[t.header.frame_id + '->' + t.child_frame_id] = t
                times.setdefault(topic,[]).append(time.monotonic())
            return f
        for topic,typ in [('/map',OccupancyGrid),('/z70/map/markers',MarkerArray),
                          ('/z70/map/robot_pose',PoseStamped),('/z70/map/dock_pose',PoseStamped),
                          ('/z70/map/status',String)]:
            node.create_subscription(typ,topic,receive(topic),durable)
        for topic,typ in [('/odom',Odometry),('/camera/front/depth/points',PointCloud2),
                          ('/z70/depth/status',String),('/tf',TFMessage)]:
            node.create_subscription(typ,topic,receive(topic),10)
        start = time.monotonic()
        late = {}
        while time.monotonic()-start < args.duration:
            rclpy.spin_once(node,timeout_sec=.2)
            if time.monotonic()-start > 15 and not late:
                late['subscription'] = node.create_subscription(OccupancyGrid,'/map',
                    lambda msg: late.update(received=True,width=msg.info.width),durable)
        results = {'duration_s':time.monotonic()-start, 'topics':{},
                   'late_join_map_received':late.get('received',False),
                   'map_publisher_qos':[{'durability':str(x.qos_profile.durability),
                                         'reliability':str(x.qos_profile.reliability)}
                                        for x in node.get_publishers_info_by_topic('/map')]}
        if 'map->odom' in transforms:
            t = transforms['map->odom'].transform
            results['map_to_odom_tf'] = [t.translation.x,t.translation.y,
                                         2*math.atan2(t.rotation.z,t.rotation.w)]
        for topic,ts in times.items():
            results['topics'][topic] = {'messages':len(ts),
                'hz':(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else None}
        if '/map' in latest:
            g = latest['/map']; occ = np.array(g.data).reshape(g.info.height,g.info.width)
            results['map'] = {'width':g.info.width,'height':g.info.height,'resolution':g.info.resolution,
                              'origin':[g.info.origin.position.x,g.info.origin.position.y],
                              'free_cells':int((occ==0).sum()),'wall_cells':int((occ==100).sum()),
                              'unknown_cells':int((occ==-1).sum())}
            render(g,latest)
        for topic,key in [('/z70/map/status','map_status'),('/z70/depth/status','depth_status')]:
            if topic in latest:
                results[key] = json.loads(latest[topic].data)
        if '/z70/map/markers' in latest:
            markers = latest['/z70/map/markers'].markers
            results['markers'] = {'count':len(markers),'labels':[m.text for m in markers if m.text],
                                  'boundary_points':sum(len(m.points) for m in markers)}
        for topic,key in [('/z70/map/robot_pose','robot_pose'),('/z70/map/dock_pose','dock_pose')]:
            if topic in latest:
                p = latest[topic].pose
                results[key] = [p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w)]
        if '/camera/front/depth/points' in latest:
            msg = latest['/camera/front/depth/points']
            xyz = np.frombuffer(bytes(msg.data),'<f4').reshape(-1,3)
            results['depth_cloud'] = {'points':msg.width,'frame':msg.header.frame_id,
                                     'point_step':msg.point_step,'finite':bool(np.isfinite(xyz).all()),
                                     'min_xyz':xyz.min(axis=0).tolist(),'max_xyz':xyz.max(axis=0).tolist()}
            np.save(OUT/'cloud-optical.npy',xyz)
            render_cloud(xyz)
        (OUT/'live-results.json').write_text(json.dumps(results,indent=2))
        print(json.dumps(results,indent=2))
        required = ['/map','/z70/map/markers','/z70/map/robot_pose','/z70/map/dock_pose','/odom',
                    '/camera/front/depth/points']
        missing = [x for x in required if x not in latest]
        if missing:
            raise RuntimeError('Missing required live topics: '+','.join(missing))
        if not results['late_join_map_received'] or 'map_to_odom_tf' not in results:
            raise RuntimeError('Late-join map/actual map->odom TF missing')
        if results.get('map_status',{}).get('map_to_odom') is None:
            raise RuntimeError('map->odom alignment missing')
    finally:
        if node:
            node.destroy_node(); rclpy.try_shutdown()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
        try:
            process.wait(timeout=50)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL); process.wait()
        # Catch any worker left behind by a prematurely exited ROS launcher.
        try:
            os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:
            pass
        log.close()


def render(g,latest):
    occ = np.array(g.data).reshape(g.info.height,g.info.width)
    image = np.full((*occ.shape,3),35,np.uint8)
    image[occ==0] = (232,237,239); image[occ==100] = (85,85,85)
    image = cv2.resize(np.flipud(image),None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)
    def px(x,y):
        return (int((x-g.info.origin.position.x)/g.info.resolution*3),
                int((g.info.height-(y-g.info.origin.position.y)/g.info.resolution)*3))
    for m in latest.get('/z70/map/markers',MarkerArray()).markers:
        color = tuple(int(c*255) for c in (m.color.b,m.color.g,m.color.r))
        if m.ns=='room_polygons':
            for a,b in zip(m.points[::2],m.points[1::2]):
                cv2.line(image,px(a.x,a.y),px(b.x,b.y),color,1)
        if m.text:
            p = m.pose.position
            cv2.putText(image,m.text,px(p.x,p.y),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1,cv2.LINE_AA)
    for topic,color in [('/z70/map/robot_pose',(0,100,255)),('/z70/map/dock_pose',(100,255,0))]:
        if topic in latest:
            p = latest[topic].pose; yaw=2*math.atan2(p.orientation.z,p.orientation.w)
            cv2.circle(image,px(p.position.x,p.position.y),6,color,-1)
            cv2.arrowedLine(image,px(p.position.x,p.position.y),
                           px(p.position.x+.45*math.cos(yaw),p.position.y+.45*math.sin(yaw)),color,2)
    cv2.putText(image,'Actual ROS /map: vendor LiDAR snapshot, 5 cm/cell',
                (15,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(240,240,240),1,cv2.LINE_AA)
    cv2.imwrite(str(OUT/'ros-map.png'),image)


def render_cloud(xyz):
    cal = json.loads((REPO/'calibration/front-camera/front_camera.json').read_text())
    p = xyz @ np.array(cal['R_floor_to_cam']) + [0.,0.,cal['camera_height_mm']/1000.]
    image = np.full((700,900,3),30,np.uint8)
    keep = (p[:,0]>0)&(p[:,0]<3)&(abs(p[:,1])<2)&(p[:,2]>-.1)&(p[:,2]<2.)
    p = p[keep]
    x = ((2-p[:,1])*200).astype(int)+50
    y = (650-p[:,0]*200).astype(int)
    colors = cv2.applyColorMap((np.clip(p[:,2]/1.,0,1)*255).astype(np.uint8).reshape(-1,1),cv2.COLORMAP_TURBO).reshape(-1,3)
    image[y,x] = colors
    cv2.putText(image,'Actual ROS depth cloud: top down, height color (0-1 m)',
                (15,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(240,240,240),1,cv2.LINE_AA)
    for d in (.5,1.,1.5,2.,2.5,3.):
        cv2.putText(image,f'{d:.1f}m',(5,int(650-d*200)),cv2.FONT_HERSHEY_SIMPLEX,.4,(220,220,220),1)
    cv2.circle(image,(450,650),5,(0,0,255),-1)
    cv2.imwrite(str(OUT/'ros-depth-topdown.png'),image)


if __name__=='__main__':
    main()
