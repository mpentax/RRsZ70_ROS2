#!/usr/bin/env python3
"""Passive ROS recording; no robot commands or secret access."""
import json, time, signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rosidl_runtime_py.convert import message_to_ordereddict
from cv_bridge import CvBridge
import cv2

ROOT=Path(__file__).resolve().parent
rclpy.init()
node=Node('chassis_trial_recorder')
f=(ROOT/'ros-odom.jsonl').open('a',buffering=1)
latest={}
bridge=CvBridge()

def record(topic,msg):
    data=message_to_ordereddict(msg)
    if topic=='/z70/odom_status':
        data=json.loads(msg.data)
    row={'received_at_ns':time.time_ns(),'topic':topic,'data':data}
    f.write(json.dumps(row)+'\n')
    latest[topic]=row
    (ROOT/'ros-latest.json').write_text(json.dumps(latest,indent=2))

def mosaic(msg):
    im=bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    cv2.imwrite(str(ROOT/'floor-mosaic.png'),im)
    (ROOT/'mosaic-metadata.json').write_text(json.dumps({'received_at_ns':time.time_ns(),'header':message_to_ordereddict(msg.header),'width':msg.width,'height':msg.height,'encoding':msg.encoding}))

node.create_subscription(Odometry,'/odom',lambda m:record('/odom',m),10)
node.create_subscription(String,'/z70/odom_status',lambda m:record('/z70/odom_status',m),10)
q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(Image,'/z70/floor_mosaic',mosaic,q)
running=True

def stop(*_):
    global running
    running=False
signal.signal(signal.SIGINT,stop)
signal.signal(signal.SIGTERM,stop)
try:
    while running and rclpy.ok():
        rclpy.spin_once(node,timeout_sec=.2)
finally:
    f.close()
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
