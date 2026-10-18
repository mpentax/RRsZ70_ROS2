"""Read-only ROS odometry sample, bounded wait, no robot command path."""
import json
import time
import rclpy
from nav_msgs.msg import Odometry
rclpy.init()
node = rclpy.create_node('pickup_odom_sample')
samples = []
def callback(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    samples.append({'stamp_s': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9, 'x':p.x, 'y':p.y, 'qz':q.z, 'qw':q.w, 'vx': msg.twist.twist.linear.x, 'wz':msg.twist.twist.angular.z})
sub = node.create_subscription(Odometry, '/odom', callback, 10)
end = time.monotonic() + 5
while time.monotonic() < end and len(samples)<3:
    rclpy.spin_once(node, timeout_sec=0.5)
print(json.dumps({'odom_samples':samples}))
node.destroy_node()
rclpy.shutdown()
