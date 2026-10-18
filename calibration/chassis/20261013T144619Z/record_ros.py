#!/usr/bin/env python3
"""Read-only ROS topic recorder for this supervised trial; no service access."""
import json
import math
from pathlib import Path
import signal
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

ROOT = Path(__file__).resolve().parent
running = True

def stop(*_):
    global running
    running = False

signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
rclpy.init()
node = Node('chassis_trial_recorder')
stream = (ROOT / 'ros-odom.jsonl').open('a', buffering=1)
latest = {}
mosaic = None

def emit(topic, payload):
    row = {'received_ns': time.time_ns(), 'topic': topic, **payload}
    stream.write(json.dumps(row, allow_nan=False) + '\n')
    latest[topic] = row
    (ROOT / 'ros-latest.json').write_text(json.dumps(latest, indent=2))

def odom(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    emit('/odom', {'stamp_ns': m.header.stamp.sec*10**9+m.header.stamp.nanosec,
        'x_m': p.x, 'y_m': p.y, 'yaw_rad': yaw, 'covariance': list(m.pose.covariance)})

def status(m):
    emit('/z70/odom_status', {'status': json.loads(m.data)})

def image(m):
    global mosaic
    if m.encoding not in ('bgr8', 'rgb8'):
        return
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width*3].reshape(m.height,m.width,3)
    mosaic = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else img.copy()
    cv2.imwrite(str(ROOT / 'floor-mosaic.png'), mosaic)

node.create_subscription(Odometry, '/odom', odom, 100)
node.create_subscription(String, '/z70/odom_status', status, 100)
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(Image, '/z70/floor_mosaic', image, qos)
print('recorder ready', flush=True)
try:
    while running and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
finally:
    if mosaic is not None:
        cv2.imwrite(str(ROOT / 'floor-mosaic.png'), mosaic)
    stream.close()
    node.destroy_node()
    rclpy.try_shutdown()
    print('recorder stopped', flush=True)
