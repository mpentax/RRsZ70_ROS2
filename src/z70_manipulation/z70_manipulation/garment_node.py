"""Front-camera garment perception ROS 2 node and Apple Silicon MPS segmentation worker."""
import argparse
import contextlib
from copy import deepcopy
import io
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time
import uuid

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import Point
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener, TransformException
    from z70_interfaces.msg import GarmentObservation, GarmentObservations
    from z70_driver.driver import MatchedFrames, read_pipe_exact, stop_worker_process, worker_env
    from z70_mapping.decoder import quat_yaw
    from .perception import front_camera, calibration, observe, geofence, GarmentTracker
    LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
except ImportError:
    Node = object
    LATCHED = None

DINO_ID = 'IDEA-Research/grounding-dino-tiny'
DINO_REV = 'a2bb814dd30d776dcf7e30523b00659f4f141c71'
CLIP_ID = 'CIDAS/clipseg-rd64-refined'
CLIP_REV = '999e0328d9e10b484360c477313983f9afdd7050'
BACKENDS = ('grounding_dino_clipseg', 'grounding_dino', 'clipseg', 'sim_hsv', 'contour_fallback')
SIM_GARMENT_HSV = (
    ('sock', (5, 110, 65), (17, 255, 255)),
    ('towel', (39, 75, 45), (85, 255, 255)),
    ('shirt', (127, 65, 45), (168, 255, 255)),
)
PROMPTS = ['sock or shirt or towel or pants on floor', 'bare floor tile wood carpet',
           'furniture wall cabinet door']
MAX_PACKET = 8 * 1024 * 1024


def read_exact(stream, count):
    data = bytearray()
    while len(data) < count:
        chunk = stream.read(count - len(data))
        if not chunk:
            raise EOFError('Worker input ended')
        data.extend(chunk)
    return bytes(data)


def read_packet(stream):
    size = struct.unpack('<I', read_exact(stream, 4))[0]
    if not 0 < size <= MAX_PACKET:
        raise ValueError('Oversized or empty worker frame')
    return read_exact(stream, size)


def encode_result(detections, quality):
    packet = io.BytesIO()
    metadata = [{k: v for k, v in d.items() if k != 'mask'} for d in detections]
    masks = {f'mask_{i}': np.asarray(d['mask'], np.uint8) for i, d in enumerate(detections)}
    np.savez_compressed(packet, quality=np.array(json.dumps(quality)),
                        detections=np.array(json.dumps(metadata)), **masks)
    return packet.getvalue()


def decode_result(data):
    with np.load(io.BytesIO(data), allow_pickle=False) as packet:
        detections = json.loads(str(packet['detections']))
        if len(detections) > 32:
            raise ValueError('Too many detections')
        for i, d in enumerate(detections):
            mask = np.asarray(packet[f'mask_{i}'], np.uint8)
            if mask.ndim != 2 or mask.size > 1600 * 1200:
                raise ValueError('Invalid mask')
            d['mask'] = mask
        return detections, json.loads(str(packet['quality']))


def components(mask, confidence=.5, category='garment', min_area=150):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(np.asarray(mask, np.uint8), 8)
    result = []
    for i in sorted(range(1, n), key=lambda i: -stats[i, cv2.CC_STAT_AREA])[:32]:
        x, y, w, h, area = map(int, stats[i])
        if area < min_area or area > mask.size * .6:
            continue
        result.append(dict(category=category[:32], confidence=float(confidence), bbox=[x, y, w, h],
                           mask=(labels == i).astype(np.uint8)))
    return result


def refine_box(image, box):
    x0, y0, x1, y1 = box
    h, w = image.shape[:2]
    x0, y0 = max(0, int(x0)), max(0, int(y0))
    x1, y1 = min(w - 1, int(x1)), min(h - 1, int(y1))
    mask = np.zeros((h, w), np.uint8)
    if x1 - x0 < 3 or y1 - y0 < 3:
        return mask
    try:
        cv2.grabCut(image, mask, (x0, y0, x1 - x0, y1 - y0), np.zeros((1, 65)), np.zeros((1, 65)), 2, cv2.GC_INIT_WITH_RECT)
    except cv2.error:
        return np.zeros((h, w), np.uint8)
    mask = ((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD)).astype(np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


class GarmentModel:
    def __init__(self, backend='grounding_dino_clipseg', device='mps'):
        if backend not in BACKENDS:
            raise ValueError('Unknown garment backend')
        self.backend, self.device = backend, device
        self.dino = self.clip = None
        self.error = None
        os.environ['HF_HUB_OFFLINE'] = '1'; os.environ['TRANSFORMERS_OFFLINE'] = '1'
        os.environ['HF_HUB_DISABLE_TELEMETRY'] = '1'
        if backend in ('contour_fallback', 'sim_hsv'):
            return
        try:
            with contextlib.redirect_stdout(sys.stderr):
                import torch
                from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection, CLIPSegForImageSegmentation
                self.torch = torch
                if backend in ('grounding_dino', 'grounding_dino_clipseg'):
                    self.dino_proc = AutoProcessor.from_pretrained(DINO_ID, revision=DINO_REV, local_files_only=True)
                    self.dino = AutoModelForZeroShotObjectDetection.from_pretrained(DINO_ID, revision=DINO_REV, local_files_only=True).to(device).eval()
                if backend in ('clipseg', 'grounding_dino_clipseg'):
                    self.clip_proc = AutoProcessor.from_pretrained(CLIP_ID, revision=CLIP_REV, local_files_only=True)
                    self.clip = CLIPSegForImageSegmentation.from_pretrained(CLIP_ID, revision=CLIP_REV, local_files_only=True).to(device).eval()
        except Exception as error:
            self.error = type(error).__name__ + ': ' + str(error)[:240]
            self.dino = self.clip = None

    def sync(self):
        if self.device == 'mps' and hasattr(self, 'torch'):
            self.torch.mps.synchronize()

    def infer(self, image):
        self.sync(); start = time.monotonic()
        quality = dict(backend=self.backend, model_id=self.backend, model_available=self.error is None and self.backend != 'contour_fallback')
        if self.error:
            quality.update(error=self.error, inference_ms=0.)
            return [], quality
        detections = []
        if self.backend == 'contour_fallback':
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = ((hsv[:, :, 1] > 65) & (np.indices(image.shape[:2])[0] > image.shape[0] * .45)).astype(np.uint8)
            detections = components(mask, confidence=.1, category='unqualified_contour', min_area=50)
        elif self.backend == 'sim_hsv':
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            floor_band = (np.indices(image.shape[:2])[0] > image.shape[0] * .38).astype(np.uint8)
            kernel = np.ones((3, 3), np.uint8)
            min_area = max(50, image.shape[0] * image.shape[1] // 10000)
            for category, lo, hi in SIM_GARMENT_HSV:
                raw = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8)) > 0
                cleaned = cv2.morphologyEx((raw & (floor_band > 0)).astype(np.uint8), cv2.MORPH_OPEN, kernel)
                cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
                detections.extend(components(cleaned, confidence=.95, category=category, min_area=min_area))
        else:
            from PIL import Image
            rgb = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            with contextlib.redirect_stdout(sys.stderr), self.torch.inference_mode():
                clip_mask = None
                if self.clip is not None:
                    inputs = self.clip_proc(text=PROMPTS, images=[rgb] * 3, padding=True, return_tensors='pt').to(self.device)
                    logits = self.clip(**inputs).logits.float()
                    probabilities = self.torch.sigmoid(logits).cpu().numpy()
                    contrast = probabilities[0] - np.maximum(probabilities[1], probabilities[2])
                    clip_mask = ((cv2.resize(contrast, (image.shape[1], image.shape[0])) > .12) &
                                 (cv2.resize(probabilities[0], (image.shape[1], image.shape[0])) > .45)).astype(np.uint8)
                min_area = max(50, image.shape[0] * image.shape[1] // 10000)
                if self.dino is not None:
                    inputs = self.dino_proc(images=rgb, text='sock. shirt. towel. pants. jeans. hoodie.', return_tensors='pt').to(self.device)
                    prediction = self.dino(**inputs)
                    import inspect
                    post = self.dino_proc.post_process_grounded_object_detection
                    threshold_key = 'threshold' if 'threshold' in inspect.signature(post).parameters else 'box_threshold'
                    result = post(prediction, inputs.input_ids, **{threshold_key: .3}, text_threshold=.25,
                                  target_sizes=[image.shape[:2]])[0]
                    for box, score in zip(result['boxes'][:16], result['scores'][:16]):
                        mask = refine_box(image, box.cpu().numpy())
                        if clip_mask is not None:
                            mask &= clip_mask
                        detections.extend(components(mask, float(score.cpu()), min_area=min_area))
                elif clip_mask is not None:
                    detections = components(clip_mask, min_area=min_area)
        self.sync()
        quality['inference_ms'] = (time.monotonic() - start) * 1000
        if self.device == 'mps' and hasattr(self, 'torch'):
            quality['mps_allocated_bytes'] = self.torch.mps.current_allocated_memory()
            quality['mps_driver_bytes'] = self.torch.mps.driver_allocated_memory()
        return detections[:32], quality


def worker_main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=BACKENDS, default='grounding_dino_clipseg')
    parser.add_argument('--device', default='mps')
    args = parser.parse_args()
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    output = sys.stdout.buffer
    model = GarmentModel(args.backend, args.device)
    try:
        while True:
            try:
                jpeg = read_packet(sys.stdin.buffer)
            except EOFError:
                break
            try:
                image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if image is None or image.shape[0] * image.shape[1] > 1600 * 1200:
                    raise ValueError('Invalid JPEG or oversized image')
                detections, quality = model.infer(image)
            except Exception as error:
                detections, quality = [], dict(model_available=False, backend=args.backend, error=type(error).__name__)
            data = encode_result(detections, quality)
            if len(data) > MAX_PACKET:
                data = encode_result([], dict(model_available=False, error='OversizedResult'))
            output.write(struct.pack('<I', len(data)) + data); output.flush()
    finally:
        del model


def seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


def observations_message(items, meta, quality, geofence_active):
    msg = GarmentObservations()
    msg.header = deepcopy(meta.header); msg.header.frame_id = 'base_footprint'
    msg.evidence = deepcopy(meta.evidence)
    msg.model_id = str(quality.get('model_id', quality.get('backend', 'unavailable')))[:64]
    msg.model_available = bool(quality.get('model_available', False))
    msg.geofence_active = bool(geofence_active)
    scalar_fields = ('category', 'confidence', 'bbox_px', 'contact_px', 'grasp_px', 'contact_valid',
                     'grasp_valid', 'footprint_width_m', 'footprint_depth_m', 'position_uncertainty_m',
                     'inside_geofence', 'geofence_verified', 'reachable_for_grasp', 'transport_suitability',
                     'suitability_reason', 'calibration_id')
    for item in items:
        obs = GarmentObservation(); obs.header = deepcopy(msg.header); obs.evidence = deepcopy(msg.evidence)
        obs.track_id.uuid = list(uuid.UUID(item['track_id']).bytes)
        for field in scalar_fields:
            setattr(obs, field, item[field])
        for field in ('floor_contact_m', 'grasp_point_m'):
            x, y, z = map(float, item[field])
            setattr(obs, field, Point(x=x, y=y, z=z))
        msg.garments.append(obs)
    return msg


class GarmentNode(Node):
    def __init__(self):
        super().__init__('z70_garment_perception')
        python = self.declare_parameter('garment_python', str(Path.home()/'.venvs/roboclaw-depth/bin/python')).value
        backend = self.declare_parameter('backend', 'grounding_dino_clipseg').value
        device = self.declare_parameter('device', 'mps').value
        hz = float(self.declare_parameter('garment_hz', .5).value)
        self.max_age = float(self.declare_parameter('max_frame_age_s', 5.).value)
        qual_small = bool(self.declare_parameter('qualified_small_lift', backend == 'sim_hsv').value)
        allow_tow = bool(self.declare_parameter('allow_tow', backend == 'sim_hsv').value)
        self.qualified_policy = dict(qualified_small_lift=qual_small, allow_tow=allow_tow) if (qual_small or allow_tow) else None
        self.cam = front_camera(); self.camera_x = calibration()['front_camera_x_m']
        self.pub = self.create_publisher(GarmentObservations, '/z70/garments/observations', LATCHED)
        self.markers = self.create_publisher(MarkerArray, '/z70/garments', 1)
        self.json_pub = self.create_publisher(String, '/z70/garments/json', 1)
        self.status = self.create_publisher(String, '/z70/garments/status', LATCHED)
        self.grid = None; self.grid_received = None
        self.create_subscription(OccupancyGrid, '/z70/map/safe_grid', self.on_grid, LATCHED)
        self.tf = Buffer(cache_time=Duration(seconds=30.)); self.tf_listener = TransformListener(self.tf, self)
        self.tracker = GarmentTracker(); self.last_pose = None; self.last_session = None
        self.lock = threading.Lock(); self.stop_event = threading.Event()
        self.latest = None; self.result = None; self.process = None; self.worker = None
        self.frames = MatchedFrames(self, 'front', self.on_frame, compressed=True)
        self.set_status(dict(model_available=False, backend=backend, state='STARTING'))
        if backend not in BACKENDS:
            self.set_status(dict(model_available=False, error='UNKNOWN_BACKEND')); return
        env = worker_env(HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1', HF_HUB_DISABLE_TELEMETRY='1')
        try:
            self.process = subprocess.Popen([python, '-u', str(Path(__file__).resolve()),
                                             '--backend', backend, '--device', device], stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, env=env, bufsize=0)
            self.worker = threading.Thread(target=self.run_worker, args=(1/max(.01, hz),), daemon=True)
            self.worker.start()
        except OSError as error:
            self.set_status(dict(model_available=False, error=type(error).__name__))
        self.create_timer(.1, self.publish)

    def set_status(self, quality):
        self.status.publish(String(data=json.dumps(quality, allow_nan=False)))

    def on_grid(self, msg):
        self.grid = msg; self.grid_received = time.monotonic()

    def on_frame(self, msg):
        meta = self.frames.metadata
        if (msg.header.frame_id != 'camera_optical_frame' or meta is None
                or (meta.width, meta.height) != (self.cam.width, self.cam.height)
                or not 0 < len(msg.data) <= MAX_PACKET):
            return
        with self.lock:
            self.latest = (msg, deepcopy(meta))

    def read_exact(self, count):
        return read_pipe_exact(self.process, count, self.stop_event)

    def run_worker(self, period):
        proc = self.process
        while not self.stop_event.is_set():
            with self.lock:
                frame, self.latest = self.latest, None
            if frame is None:
                self.stop_event.wait(.05); continue
            msg, meta = frame
            if not 0 <= time.time()-seconds(meta.evidence.received_at) <= self.max_age:
                continue
            try:
                data = bytes(msg.data)
                # Raw pipes can short-write; use write-all, not one optimistic write.
                payload = memoryview(struct.pack('<I', len(data))+data)
                while payload and not self.stop_event.is_set():
                    written = proc.stdin.write(payload)
                    if not written:
                        raise EOFError('Garment stdin closed')
                    payload = payload[written:]
                size = struct.unpack('<I', self.read_exact(4))[0]
                if not 0 < size <= MAX_PACKET:
                    raise ValueError('Oversized worker response')
                detections, quality = decode_result(self.read_exact(size))
                with self.lock:
                    self.result = (meta, detections, quality)
            except (OSError, EOFError, TimeoutError, ValueError, KeyError) as error:
                with self.lock:
                    self.result = (meta, [], dict(model_available=False, error=type(error).__name__))
                break
            self.stop_event.wait(period)

    def pose_at(self, stamp):
        try:
            tf = self.tf.lookup_transform('map', 'base_footprint', Time.from_msg(stamp), timeout=Duration(seconds=0.))
            t = tf.transform.translation
            age = abs(seconds(tf.header.stamp)-seconds(stamp))
            return (t.x, t.y, quat_yaw(tf.transform.rotation)), age
        except TransformException:
            return None, float('inf')

    def publish(self):
        with self.lock:
            result, self.result = self.result, None
        if result is None:
            if self.process is not None and self.process.poll() is not None:
                self.set_status(dict(model_available=False, error='WORKER_EXITED'))
            return
        meta, detections, quality = result
        age = time.time()-seconds(meta.evidence.received_at)
        if not 0 <= age <= self.max_age:
            quality = dict(quality, model_available=False, error='STALE_INFERENCE', frame_age_s=age)
            detections = []
            meta = deepcopy(meta); meta.evidence.stale = True; meta.evidence.valid = False
            meta.evidence.reason = 'STALE_INFERENCE'
        pose, tf_age = self.pose_at(meta.header.stamp)
        grid = self.grid
        grid_age = (max(time.monotonic()-self.grid_received, time.time()-seconds(grid.header.stamp))
                    if grid is not None else float('inf'))
        session = bytes(meta.evidence.session_id.uuid)
        # Missing ego pose cannot be interpreted as zero motion.
        if session != self.last_session or pose is None or self.last_pose is None:
            self.tracker = GarmentTracker(); delta = (0., 0., 0.)
        else:
            old_x, old_y, old_yaw = self.last_pose
            dx, dy = pose[0]-old_x, pose[1]-old_y
            delta = (math.cos(old_yaw)*dx+math.sin(old_yaw)*dy,
                     -math.sin(old_yaw)*dx+math.cos(old_yaw)*dy, pose[2]-old_yaw)
        self.last_pose, self.last_session = pose, session
        items = []
        for detection in detections:
            try:
                item = observe(detection['mask'], self.cam, category=detection['category'],
                               confidence=detection['confidence'], camera_x_m=self.camera_x,
                               calibration_id=meta.calibration_id,
                               qualified_policy=getattr(self, 'qualified_policy', None))
            except (ValueError, KeyError):
                continue
            if item['contact_valid']:
                item['inside_geofence'], item['geofence_verified'] = geofence(
                    item['floor_contact_m'], pose, grid, tf_age_s=tf_age, grid_age_s=grid_age)
            # Contour diagnostic output is not a garment selection candidate.
            if not quality.get('model_available', False):
                item['grasp_valid'] = False; item['reachable_for_grasp'] = False
            items.append(item)
        items = self.tracker.update(items, delta)
        for item in items:
            item['reachable_for_grasp'] = bool(item['reachable_for_grasp'] and item['confirmed']
                                               and item['inside_geofence'] and item['geofence_verified'])
        _, active = geofence((0., 0., 0.), pose, grid, tf_age_s=tf_age, grid_age_s=grid_age)
        msg = observations_message(items, meta, quality, active)
        self.pub.publish(msg)
        self.json_pub.publish(String(data=json.dumps(dict(model=quality, geofence_active=active,
                                                          frame_age_s=age, garments=items), allow_nan=False)))
        markers = MarkerArray()
        clear = Marker(); clear.header = deepcopy(msg.header); clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, item in enumerate(items):
            if not item['contact_valid']:
                continue
            marker = Marker(); marker.header = deepcopy(msg.header); marker.ns = 'garments'; marker.id = i
            marker.type = Marker.SPHERE; marker.action = Marker.ADD
            marker.pose.position = Point(**dict(zip(('x', 'y', 'z'), item['floor_contact_m'])))
            marker.pose.orientation.w = 1.; marker.scale.x = marker.scale.y = marker.scale.z = .025
            marker.color.a = 1.; marker.color.g = .8 if item['inside_geofence'] else 0.; marker.color.r = .8
            marker.lifetime = Duration(seconds=5.).to_msg(); markers.markers.append(marker)
        self.markers.publish(markers)
        self.set_status(dict(quality, geofence_active=active, frame_age_s=age, tracks=len(self.tracker.tracks),
                             confirmed=sum(x['confirmed'] for x in items)))

    def close(self):
        stop_worker_process(self.process, self.worker, self.stop_event)


def main():
    rclpy.init(); node = GarmentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.close(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    worker_main()
