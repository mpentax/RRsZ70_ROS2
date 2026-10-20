"""Persistent Apple Silicon MPS garment segmentation worker."""
import argparse
import contextlib
import io
import json
import os
import signal
import struct
import sys
import time
import cv2
import numpy as np

DINO_ID = 'IDEA-Research/grounding-dino-tiny'
DINO_REV = 'a2bb814dd30d776dcf7e30523b00659f4f141c71'
CLIP_ID = 'CIDAS/clipseg-rd64-refined'
CLIP_REV = '999e0328d9e10b484360c477313983f9afdd7050'
BACKENDS = ('grounding_dino_clipseg', 'grounding_dino', 'clipseg', 'contour_fallback')
PROMPTS = ['sock or shirt or towel or pants on floor', 'bare floor tile wood carpet',
           'furniture wall cabinet door']
MAX_PACKET = 8*1024*1024


def read_exact(stream, count):
    data = bytearray()
    while len(data) < count:
        chunk = stream.read(count-len(data))
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
            if mask.ndim != 2 or mask.size > 1600*1200:
                raise ValueError('Invalid mask')
            d['mask'] = mask
        return detections, json.loads(str(packet['quality']))


def components(mask, confidence=.5, category='garment', min_area=150):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(np.asarray(mask, np.uint8), 8)
    result = []
    for i in sorted(range(1, n), key=lambda i: -stats[i, cv2.CC_STAT_AREA])[:32]:
        x, y, w, h, area = map(int, stats[i])
        if area < min_area or area > mask.size*.6:
            continue
        result.append(dict(category=category[:32], confidence=float(confidence), bbox=[x, y, w, h],
                           mask=(labels == i).astype(np.uint8)))
    return result


def refine_box(image, box):
    x0, y0, x1, y1 = box
    h, w = image.shape[:2]
    x0, y0 = max(0, int(x0)), max(0, int(y0))
    x1, y1 = min(w-1, int(x1)), min(h-1, int(y1))
    mask = np.zeros((h, w), np.uint8)
    if x1-x0 < 3 or y1-y0 < 3:
        return mask
    try:
        cv2.grabCut(image, mask, (x0, y0, x1-x0, y1-y0), np.zeros((1, 65)), np.zeros((1, 65)), 2, cv2.GC_INIT_WITH_RECT)
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
        # No hub/telemetry network even if a future upstream API ignores a flag.
        os.environ['HF_HUB_OFFLINE'] = '1'; os.environ['TRANSFORMERS_OFFLINE'] = '1'
        os.environ['HF_HUB_DISABLE_TELEMETRY'] = '1'
        if backend == 'contour_fallback':
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
            self.error = type(error).__name__+': '+str(error)[:240]
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
            mask = ((hsv[:, :, 1] > 65) & (np.indices(image.shape[:2])[0] > image.shape[0]*.45)).astype(np.uint8)
            detections = components(mask, confidence=.1, category='unqualified_contour', min_area=50)
        else:
            from PIL import Image
            rgb = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            with contextlib.redirect_stdout(sys.stderr), self.torch.inference_mode():
                clip_mask = None
                if self.clip is not None:
                    inputs = self.clip_proc(text=PROMPTS, images=[rgb]*3, padding=True, return_tensors='pt').to(self.device)
                    logits = self.clip(**inputs).logits.float()
                    probabilities = self.torch.sigmoid(logits).cpu().numpy()
                    contrast = probabilities[0]-np.maximum(probabilities[1], probabilities[2])
                    clip_mask = ((cv2.resize(contrast, (image.shape[1], image.shape[0])) > .12) &
                                 (cv2.resize(probabilities[0], (image.shape[1], image.shape[0])) > .45)).astype(np.uint8)
                if self.dino is not None:
                    inputs = self.dino_proc(images=rgb, text='sock. shirt. towel. pants. jeans. hoodie.', return_tensors='pt').to(self.device)
                    prediction = self.dino(**inputs)
                    # transformers renamed box_threshold to threshold in newer releases.
                    import inspect
                    post = self.dino_proc.post_process_grounded_object_detection
                    threshold_key = 'threshold' if 'threshold' in inspect.signature(post).parameters else 'box_threshold'
                    result = post(prediction, inputs.input_ids, **{threshold_key: .3}, text_threshold=.25,
                                  target_sizes=[image.shape[:2]])[0]
                    for box, score in zip(result['boxes'][:16], result['scores'][:16]):
                        mask = refine_box(image, box.cpu().numpy())
                        if clip_mask is not None:
                            mask &= clip_mask
                        detections.extend(components(mask, float(score.cpu()), min_area=max(50, image.shape[0]*image.shape[1]//10000)))
                elif clip_mask is not None:
                    detections = components(clip_mask, min_area=max(50, image.shape[0]*image.shape[1]//10000))
        self.sync()
        quality['inference_ms'] = (time.monotonic()-start)*1000
        if self.device == 'mps' and hasattr(self, 'torch'):
            quality['mps_allocated_bytes'] = self.torch.mps.current_allocated_memory()
            quality['mps_driver_bytes'] = self.torch.mps.driver_allocated_memory()
        return detections[:32], quality


def main():
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
                if image is None or image.shape[0]*image.shape[1] > 1600*1200:
                    raise ValueError('Invalid JPEG or oversized image')
                detections, quality = model.infer(image)
            except Exception as error:
                detections, quality = [], dict(model_available=False, backend=args.backend, error=type(error).__name__)
            data = encode_result(detections, quality)
            if len(data) > MAX_PACKET:
                data = encode_result([], dict(model_available=False, error='OversizedResult'))
            output.write(struct.pack('<I', len(data))+data); output.flush()
    finally:
        del model


if __name__ == '__main__':
    main()
