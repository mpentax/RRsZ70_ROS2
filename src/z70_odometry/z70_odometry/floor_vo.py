"""Floor visual odometry for the Z70 front camera.

Two frames -> robot planar motion (dx, dy, dtheta) in metres/radians, metric with no scale ambiguity:
the camera is calibrated and sits 68 mm above a flat floor, so every floor pixel maps to a floor point.
Steps: ORB features on the floor region -> match -> map both sides to floor coordinates ->
RANSAC rigid 2-D fit (2-point minimal sets) -> camera-floor motion -> robot base motion via the camera offset.

Floor frame of a camera pose: origin on the floor under the camera, x forward, y left (see floor_geometry.py).
Static floor points obey p1 = R^T (p0 - t) when the camera moves by (t, R) from frame 0 to frame 1.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass

import cv2
import numpy as np


class FloorCamera:
    """Vectorised pixel <-> floor mapping (same model as calibration/front-camera/floor_geometry.py)."""

    def __init__(self, cal: dict):
        self.f, self.cx, self.cy = cal["f_px"], cal["cx"], cal["cy"]
        self.k1, self.k2 = cal["k1"], cal["k2"]
        self.R = np.asarray(cal["R_floor_to_cam"], float)
        self.h = cal["camera_height_mm"] / 1000.0
        self.W, self.H = cal["width"], cal["height"]

    @classmethod
    def load(cls, path=None):
        from .resources import front_calibration_path
        with open(path or front_calibration_path()) as stream:
            return cls(json.load(stream))

    def pixels_to_floor(self, uv: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(N,2) pixels -> (N,2) floor xy metres, plus mask of pixels whose ray hits the floor."""
        xd = (uv[:, 0] - self.cx) / self.f
        yd = (uv[:, 1] - self.cy) / self.f
        x, y = xd.copy(), yd.copy()
        for _ in range(20):
            r2 = x * x + y * y
            s = 1 + self.k1 * r2 + self.k2 * r2 * r2
            x, y = xd / s, yd / s
        d = np.stack([x, y, np.ones_like(x)], 1) @ self.R          # floor-frame ray directions
        ok = d[:, 2] < -1e-4
        t = np.where(ok, self.h / -np.where(ok, d[:, 2], -1), 0)
        return np.stack([d[:, 0] * t, d[:, 1] * t], 1), ok

    def floor_to_pixels(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        p = np.c_[xy, np.zeros(len(xy))] - [0, 0, self.h]
        pc = p @ self.R.T
        ok = pc[:, 2] > 1e-6
        z = np.where(ok, pc[:, 2], 1)
        x, y = pc[:, 0] / z, pc[:, 1] / z
        r2 = x * x + y * y
        s = 1 + self.k1 * r2 + self.k2 * r2 * r2
        return np.stack([self.f * x * s + self.cx, self.f * y * s + self.cy], 1), ok

    def floor_mask(self, max_range=1.5, min_range=0.06) -> np.ndarray:
        """Image mask of pixels that map to the floor within a usable range (accuracy falls with D^2)."""
        u, v = np.meshgrid(np.arange(self.W), np.arange(self.H))
        xy, ok = self.pixels_to_floor(np.stack([u.ravel(), v.ravel()], 1).astype(float))
        r = np.hypot(xy[:, 0], xy[:, 1])
        return (ok & (r < max_range) & (r > min_range)).reshape(self.H, self.W)


@dataclass
class Motion:
    dx: float          # forward, metres (robot base frame of pose 0)
    dy: float          # left, metres
    dtheta: float      # radians, CCW
    inliers: int
    matches: int
    rms_m: float


def rigid2d(a: np.ndarray, b: np.ndarray):
    """Least-squares R, t with b ~ R a + t (2-D Kabsch)."""
    ca, cb = a.mean(0), b.mean(0)
    H = (a - ca).T @ (b - cb)
    th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    return R, cb - R @ ca, th


class FloorOdometry:
    def __init__(self, cam: FloorCamera, camera_x_m: float | None = None, max_range=1.5, n_features=3000,
                 ransac_thresh_m=0.008, seed=0, min_inliers=80, max_rms_m=0.003):
        from .resources import initial_camera_x_m
        self.cam, self.cam_x = cam, initial_camera_x_m() if camera_x_m is None else camera_x_m
        self.min_inliers, self.max_rms = min_inliers, max_rms_m
        self.mask = (cam.floor_mask(max_range) * 255).astype(np.uint8)
        self.orb = cv2.ORB_create(n_features, scaleFactor=1.2, nlevels=8, fastThreshold=10)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.th = ransac_thresh_m
        self.rng = np.random.default_rng(seed)

    def features(self, gray):
        return self.orb.detectAndCompute(gray, self.mask)

    def estimate(self, img0, img1) -> Motion | None:
        g0 = img0 if img0.ndim == 2 else cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
        g1 = img1 if img1.ndim == 2 else cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        k0, d0 = self.features(g0); k1, d1 = self.features(g1)
        if d0 is None or d1 is None or len(k0) < 10 or len(k1) < 10:
            return None
        pairs = self.bf.knnMatch(d0, d1, k=2)
        good = [m for m, n in (p for p in pairs if len(p) == 2) if m.distance < 0.8 * n.distance]
        if len(good) < 8:
            return None
        u0 = np.float64([k0[m.queryIdx].pt for m in good]); u1 = np.float64([k1[m.trainIdx].pt for m in good])
        f0, ok0 = self.cam.pixels_to_floor(u0); f1, ok1 = self.cam.pixels_to_floor(u1)
        ok = ok0 & ok1
        f0, f1 = f0[ok], f1[ok]
        n = len(f0)
        if n < 8:
            return None
        # RANSAC on p0 = R_c p1 + t_c (frame-1 floor points expressed in frame 0 = camera motion)
        best = None
        for _ in range(400):
            i, j = self.rng.choice(n, 2, replace=False)
            if np.linalg.norm(f1[i] - f1[j]) < 0.02:
                continue
            R, t, _ = rigid2d(f1[[i, j]], f0[[i, j]])
            err = np.linalg.norm(f1 @ R.T + t - f0, axis=1)
            inl = err < self.th
            if best is None or inl.sum() > best.sum():
                best = inl
        if best is None or best.sum() < 6:
            return None
        # The broad RANSAC radius finds the consensus; refine against a tighter
        # metric radius so distant noisy matches do not dominate a near-floor fit.
        for _ in range(3):
            R, t, th = rigid2d(f1[best], f0[best])
            residual = np.linalg.norm(f1 @ R.T + t - f0, axis=1)
            refined = residual < min(self.th, self.max_rms)
            if refined.sum() < self.min_inliers:
                break
            best = refined
        R, t, th = rigid2d(f1[best], f0[best])
        rms = float(np.sqrt(np.mean(np.sum((f1[best] @ R.T + t - f0[best]) ** 2, 1))))
        # Retain the quality limits after consensus refinement; lack of overlap
        # and non-floor matches must not turn into an invented chassis motion.
        if best.sum() < self.min_inliers or rms > self.max_rms:
            return None
        # camera floor-frame motion (t, th) -> robot base motion. Camera sits camera_x ahead of the base origin:
        # T_base = C T_cam C^-1 with C = translate(camera_x, 0)
        c = np.array([self.cam_x, 0.0])
        tb = t + c - R @ c
        return Motion(float(tb[0]), float(tb[1]), th, int(best.sum()), n, rms)


def render_floor_view(img0, cam: FloorCamera, motion_cam: tuple[float, float, float]):
    """Synthesise the image seen after the camera moves by (dx, dy, dtheta) over a perfectly flat world.
    Every pixel is treated as floor: exact for floor pixels, a synthetic 'painted floor' elsewhere."""
    dx, dy, th = motion_cam
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    u, v = np.meshgrid(np.arange(cam.W, dtype=float), np.arange(cam.H, dtype=float))
    uv1 = np.stack([u.ravel(), v.ravel()], 1)
    f1, ok = cam.pixels_to_floor(uv1)
    f0 = f1 @ R.T + [dx, dy]                     # where that floor point was in frame 0
    uv0, ok0 = cam.floor_to_pixels(f0)
    mx = np.where(ok & ok0, uv0[:, 0], -1).reshape(cam.H, cam.W).astype(np.float32)
    my = np.where(ok & ok0, uv0[:, 1], -1).reshape(cam.H, cam.W).astype(np.float32)
    return cv2.remap(img0, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
