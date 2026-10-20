"""Persistent inference worker, run ONLY with the depth venv Python (no ROS imports).
Protocol: uint32 LE length + JPEG in; uint32 LE length + NPZ (xyz + JSON) out.
"""
import argparse
import contextlib
import io
import json
from pathlib import Path
import struct
import sys
import signal
import time
import cv2
import numpy as np


def camera_rays(cal, width, height):
    u, v = np.meshgrid((np.arange(width)+.5)*cal['width']/width-.5,
                       (np.arange(height)+.5)*cal['height']/height-.5)
    xd, yd = (u-cal['cx'])/cal['f_px'], (v-cal['cy'])/cal['f_px']
    x, y = xd.copy(), yd.copy()
    for _ in range(30):
        r2 = x*x+y*y
        factor = 1+cal['k1']*r2+cal['k2']*r2*r2
        x, y = xd/factor, yd/factor
    rays = np.stack([x,y,np.ones_like(x)],-1)
    down = -(rays @ np.array(cal['R_floor_to_cam']))[...,2]
    floor = np.full(down.shape,np.inf)
    valid = down > 1e-4
    floor[valid] = cal['camera_height_mm']/1000./down[valid]
    return rays, floor


def metric_cloud(depth, rays, floor, min_floor_pixels=200, max_range=4.):
    valid = np.isfinite(depth) & (depth > .05)
    candidate = valid & np.isfinite(floor) & (floor >= .15) & (floor < 1.2)
    candidate[int(depth.shape[0]*.92):] = False
    ratios = np.log(floor[candidate]/depth[candidate])
    scale, aligned, count = 1., False, 0
    if len(ratios) >= min_floor_pixels:
        median = np.median(ratios)
        inlier = np.abs(ratios-median) < .15
        count = int(inlier.sum())
        if count >= min_floor_pixels and count >= .35*len(ratios):
            scale = float(np.exp(np.median(ratios[inlier])))
            aligned = .05 < scale < 20.
            if not aligned:
                scale = 1.
    corrected = depth*scale
    keep = valid & (corrected > .05) & (corrected < max_range)
    points = (rays*corrected[...,None])[keep].astype('<f4')
    return points, {'floor_aligned':bool(aligned),'scale':scale,'floor_inliers':count,
                    'floor_candidates':int(candidate.sum()),'point_count':len(points),
                    'source':'monocular_network_plus_floor_scale' if aligned else 'uncorrected_monocular_network'}


def read_exact(stream, count):
    data = bytearray()
    while len(data) < count:
        chunk = stream.read(count-len(data))
        if not chunk:
            raise EOFError('Worker input ended')
        data.extend(chunk)
    return bytes(data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--device', default='mps')
    parser.add_argument('--model', default='depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf')
    parser.add_argument('--revision', default='8078d68a9c75a972131914f6afd0c1723be0da7f')
    args = parser.parse_args()
    cal = json.loads(Path(args.calibration).read_text())
    # The ROS parent owns shutdown; avoid launch + process-group double SIGINT.
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
            size = struct.unpack('<I', read_exact(sys.stdin.buffer,4))[0]
            if size > 8*1024*1024:
                raise ValueError('Oversized worker frame')
            jpeg = read_exact(sys.stdin.buffer,size)
        except EOFError:
            break
        t0 = time.monotonic()
        try:
            image = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
            if image is None or image.shape[:2] != (cal['height'],cal['width']):
                raise ValueError('Frame dimensions do not match calibration')
            rgb = cv2.cvtColor(cv2.resize(image,(400,300)),cv2.COLOR_BGR2RGB)
            with contextlib.redirect_stdout(sys.stderr), torch.no_grad():
                inputs = proc(images=Image.fromarray(rgb),return_tensors='pt').to(args.device)
                prediction = model(**inputs)
                depth = proc.post_process_depth_estimation(prediction,target_sizes=[(300,400)])[0]['predicted_depth'].float().cpu().numpy()
            points, quality = metric_cloud(depth,rays,floor)
            quality['inference_s'] = time.monotonic()-t0
        except Exception as error:
            points = np.empty((0,3),dtype='<f4')
            quality = {'error':type(error).__name__,'point_count':0}
        packet = io.BytesIO()
        np.savez(packet, xyz=points, quality=np.array(json.dumps(quality)))
        data = packet.getvalue()
        output.write(struct.pack('<I',len(data))+data); output.flush()


if __name__ == '__main__':
    main()
