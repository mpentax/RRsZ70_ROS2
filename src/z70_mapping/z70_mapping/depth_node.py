"""ROS front-camera JPEG -> out-of-process monocular cloud; latest-only, 0.2 Hz.
No independent camera session/robot calls. Inference worker dies with this node.
"""
import io
import json
import signal
import os
import select
import struct
import subprocess
import threading
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import String
from z70_driver.frame_subscription import MatchedFrames


class DepthNode(Node):
    def __init__(self):
        super().__init__('z70_depth_cloud')
        python = self.declare_parameter('depth_python', str(Path.home()/'.venvs/roboclaw-depth/bin/python')).value
        cal = self.declare_parameter('calibration', '').value
        device = self.declare_parameter('device','mps').value
        hz = float(self.declare_parameter('depth_hz', .2).value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(PointCloud2, '/camera/front/depth/points', 1)
        self.status = self.create_publisher(String, '/z70/depth/status', qos)
        self.latest = None; self.result = None; self.last_stamp = None
        self.lock = threading.Lock(); self.stop_event = threading.Event()
        # Separate env is deliberately not allowed to inherit pixi Python paths/libraries.
        env = dict(os.environ)
        for key in ('PYTHONPATH','PYTHONHOME','DYLD_LIBRARY_PATH'):
            env.pop(key,None)
        self.process = None
        try:
            self.process = subprocess.Popen([python, '-u', str(Path(__file__).with_name('depth_worker.py')),
                                             '--calibration',cal,'--device',device],
                                            stdin=subprocess.PIPE,stdout=subprocess.PIPE,env=env,bufsize=-1)
        except OSError as e:
            self.result = (None, np.empty((0,3),dtype='<f4'),
                           {'available':False,'error':type(e).__name__})
        self.frames=MatchedFrames(self,'front',self.on_frame,compressed=True)
        self.worker = None
        if self.process is not None:
            self.worker = threading.Thread(target=self.run_worker,daemon=True,args=(max(1.,1/max(.01,hz)),))
            self.worker.start()
        self.create_timer(.2,self.publish)

    def on_frame(self,msg):
        if msg.header.frame_id != 'camera_optical_frame' or len(msg.data) > 8*1024*1024:
            return
        with self.lock:
            self.latest = msg

    def read_exact(self,count):
        proc = self.process
        if proc is None or proc.stdout is None:
            raise EOFError('Inference worker unavailable')
        data = bytearray(); deadline = time.monotonic()+120
        while len(data)<count and not self.stop_event.is_set():
            if time.monotonic()>deadline:
                raise TimeoutError('Inference deadline')
            ready,_,_ = select.select([proc.stdout],[],[],.2)
            if not ready:
                continue
            chunk = os.read(proc.stdout.fileno(),count-len(data))
            if not chunk:
                raise EOFError('Inference worker ended')
            data.extend(chunk)
        if len(data)!=count:
            raise EOFError('Inference interrupted')
        return bytes(data)

    def run_worker(self,period):
        proc = self.process
        if proc is None or proc.stdin is None:
            return
        while not self.stop_event.is_set():
            with self.lock:
                msg = self.latest
            if msg is not None and (msg.header.stamp.sec,msg.header.stamp.nanosec)!=self.last_stamp:
                stamp = (msg.header.stamp.sec,msg.header.stamp.nanosec)
                try:
                    data = bytes(msg.data)
                    proc.stdin.write(struct.pack('<I',len(data))+data)
                    proc.stdin.flush()
                    size = struct.unpack('<I',self.read_exact(4))[0]
                    if size>4*1024*1024:
                        raise ValueError('Oversized worker output')
                    packet = np.load(io.BytesIO(self.read_exact(size)),allow_pickle=False)
                    xyz = np.asarray(packet['xyz'],dtype='<f4')
                    quality = json.loads(str(packet['quality']))
                    with self.lock:
                        self.result = (msg.header,xyz,quality)
                    self.last_stamp = stamp
                except (ValueError,OSError,EOFError,TimeoutError) as e:
                    with self.lock:
                        self.result = (msg.header,np.empty((0,3),dtype='<f4'),
                                       {'error':type(e).__name__})
                    if isinstance(e,ValueError):
                        self.last_stamp = stamp
                    else:
                        break
            self.stop_event.wait(period)

    def publish(self):
        with self.lock:
            result,self.result = self.result,None
        if result is None:
            return
        header,points,quality = result
        self.status.publish(String(data=json.dumps(quality)))
        if header is None or not len(points):
            return
        msg = PointCloud2(); msg.header = header; msg.height = 1; msg.width = len(points)
        msg.fields = [PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                      for i,name in enumerate(('x','y','z'))]
        msg.point_step = 12; msg.row_step = len(points)*12
        msg.is_bigendian = False; msg.is_dense = True; msg.data = points.tobytes()
        self.pub.publish(msg)

    def close(self):
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=5)
        proc = self.process
        if proc is None:
            return
        if proc.stdin is not None:
            proc.stdin.close()  # EOF lets the model worker release resources normally.
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait()
        if proc.stdout is not None:
            proc.stdout.close()


def main():
    rclpy.init(); node = DepthNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.close(); node.destroy_node(); rclpy.try_shutdown()
