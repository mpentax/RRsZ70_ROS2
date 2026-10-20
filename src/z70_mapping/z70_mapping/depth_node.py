"""ROS front-camera JPEG -> out-of-process monocular cloud; latest-only, 0.2 Hz.
No independent camera session/robot calls. Inference worker dies with this node.
"""
import argparse
import contextlib
import io
import json
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time
import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
    from std_msgs.msg import String
    from z70_driver.driver import MatchedFrames, read_pipe_exact, stop_worker_process, worker_env
except ImportError:
    Node = object


def camera_rays(cal, width, height):
    u, v = np.meshgrid((np.arange(width) + .5) * cal['width'] / width - .5,
                       (np.arange(height) + .5) * cal['height'] / height - .5)
    xd, yd = (u - cal['cx']) / cal['f_px'], (v - cal['cy']) / cal['f_px']
    x, y = xd.copy(), yd.copy()
    for _ in range(30):
        r2 = x * x + y * y
        factor = 1 + cal['k1'] * r2 + cal['k2'] * r2 * r2
        x, y = xd / factor, yd / factor
    rays = np.stack([x, y, np.ones_like(x)], -1)
    down = -(rays @ np.array(cal['R_floor_to_cam']))[..., 2]
    floor = np.full(down.shape, np.inf)
    valid = down > 1e-4
    floor[valid] = cal['camera_height_mm'] / 1000. / down[valid]
    return rays, floor


def metric_cloud(depth, rays, floor, min_floor_pixels=200, max_range=4.):
    valid = np.isfinite(depth) & (depth > .05)
    candidate = valid & np.isfinite(floor) & (floor >= .15) & (floor < 1.2)
    candidate[int(depth.shape[0] * .92):] = False
    ratios = np.log(floor[candidate] / depth[candidate])
    scale, aligned, count = 1., False, 0
    if len(ratios) >= min_floor_pixels:
        median = np.median(ratios)
        inlier = np.abs(ratios - median) < .15
        count = int(inlier.sum())
        if count >= min_floor_pixels and count >= .35 * len(ratios):
            scale = float(np.exp(np.median(ratios[inlier])))
            aligned = .05 < scale < 20.
            if not aligned:
                scale = 1.
    corrected = depth * scale
    keep = valid & (corrected > .05) & (corrected < max_range)
    points = (rays * corrected[..., None])[keep].astype('<f4')
    return points, {'floor_aligned': bool(aligned), 'scale': scale, 'floor_inliers': count,
                    'floor_candidates': int(candidate.sum()), 'point_count': len(points),
                    'source': 'monocular_network_plus_floor_scale' if aligned else 'uncorrected_monocular_network'}


def read_exact(stream, count):
    data = bytearray()
    while len(data) < count:
        chunk = stream.read(count - len(data))
        if not chunk:
            raise EOFError('Worker input ended')
        data.extend(chunk)
    return bytes(data)


def worker_main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--device', default='mps')
    parser.add_argument('--model', default='depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf')
    parser.add_argument('--revision', default='8078d68a9c75a972131914f6afd0c1723be0da7f')
    args = parser.parse_args()
    cal = json.loads(Path(args.calibration).read_text())
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    output = sys.stdout.buffer
    with contextlib.redirect_stdout(sys.stderr):
        import torch
        from PIL import Image
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        proc = AutoImageProcessor.from_pretrained(args.model, revision=args.revision, local_files_only=True)
        model = AutoModelForDepthEstimation.from_pretrained(args.model, revision=args.revision, local_files_only=True).to(args.device).eval()
    rays, floor = camera_rays(cal, 400, 300)
    while True:
        try:
            size = struct.unpack('<I', read_exact(sys.stdin.buffer, 4))[0]
            if size > 8 * 1024 * 1024:
                raise ValueError('Oversized worker frame')
            jpeg = read_exact(sys.stdin.buffer, size)
        except EOFError:
            break
        t0 = time.monotonic()
        try:
            image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if image is None or image.shape[:2] != (cal['height'], cal['width']):
                raise ValueError('Frame dimensions do not match calibration')
            rgb = cv2.cvtColor(cv2.resize(image, (400, 300)), cv2.COLOR_BGR2RGB)
            with contextlib.redirect_stdout(sys.stderr), torch.no_grad():
                inputs = proc(images=Image.fromarray(rgb), return_tensors='pt').to(args.device)
                prediction = model(**inputs)
                depth = proc.post_process_depth_estimation(prediction, target_sizes=[(300, 400)])[0]['predicted_depth'].float().cpu().numpy()
            points, quality = metric_cloud(depth, rays, floor)
            quality['inference_s'] = time.monotonic() - t0
        except Exception as error:
            points = np.empty((0, 3), dtype='<f4')
            quality = {'error': type(error).__name__, 'point_count': 0}
        packet = io.BytesIO()
        np.savez(packet, xyz=points, quality=np.array(json.dumps(quality)))
        data = packet.getvalue()
        output.write(struct.pack('<I', len(data)) + data); output.flush()


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
        env = worker_env()
        self.process = None
        try:
            self.process = subprocess.Popen([python, '-u', str(Path(__file__).resolve()),
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
        return read_pipe_exact(self.process, count, self.stop_event)

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
        stop_worker_process(self.process, self.worker, self.stop_event)


def main():
    rclpy.init(); node = DepthNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.close(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    worker_main()
