"""Pixel <-> floor plane projection for the Z70 front camera.

Floor frame: origin on the floor directly below the camera centre, x forward
(optical axis projected onto the floor), y to the left, z up.
"""
import json
import math
from pathlib import Path

import numpy as np


class FrontCamera:
    def __init__(self, f, cx, cy, k1, k2, R_floor_to_cam, height_mm, width, height):
        self.f, self.cx, self.cy, self.k1, self.k2 = f, cx, cy, k1, k2
        self.R = np.asarray(R_floor_to_cam, float)   # x_cam = R @ (x_floor - C)
        self.h = height_mm
        self.C = np.array([0.0, 0.0, height_mm])
        self.width, self.height = width, height

    @classmethod
    def load(cls, path):
        d = json.loads(Path(path).read_text())
        return cls(d["f_px"], d["cx"], d["cy"], d["k1"], d["k2"], d["R_floor_to_cam"],
                   d["camera_height_mm"], d["width"], d["height"])

    # --- lens model (OpenCV radial k1, k2) ---------------------------------
    def _distort(self, x, y):
        r2 = x * x + y * y
        s = 1 + self.k1 * r2 + self.k2 * r2 * r2
        return x * s, y * s

    def _undistort(self, xd, yd):
        x, y = xd, yd
        for _ in range(30):                       # fixed-point inversion
            r2 = x * x + y * y
            s = 1 + self.k1 * r2 + self.k2 * r2 * r2
            x, y = xd / s, yd / s
        return x, y

    # --- public API ---------------------------------------------------------
    def pixel_to_ray(self, u, v):
        """Unit ray in the floor frame for pixel (u, v)."""
        x, y = self._undistort((u - self.cx) / self.f, (v - self.cy) / self.f)
        d = self.R.T @ np.array([x, y, 1.0])
        return d / np.linalg.norm(d)

    def pixel_to_floor(self, u, v):
        d = self.pixel_to_ray(u, v)
        if d[2] >= -1e-9:
            return None                           # at/above the horizon: never hits the floor
        t = self.h / -d[2]
        p = self.C + t * d
        return dict(forward_mm=float(p[0]), left_mm=float(p[1]),
                    range_mm=float(math.hypot(p[0], p[1])),
                    bearing_deg=float(math.degrees(math.atan2(p[1], p[0]))))

    def floor_to_pixel(self, forward_mm, left_mm, z_mm=0.0):
        pc = self.R @ (np.array([forward_mm, left_mm, z_mm]) - self.C)
        if pc[2] <= 0:
            return None
        x, y = self._distort(pc[0] / pc[2], pc[1] / pc[2])
        return float(self.f * x + self.cx), float(self.f * y + self.cy)

    def horizon_v(self, u=None):
        """Image row of the floor horizon at column u (default: centre)."""
        u = self.cx if u is None else u
        lo, hi = 0.0, float(self.height)
        for _ in range(40):                       # bisection on ray elevation
            mid = (lo + hi) / 2
            if self.pixel_to_ray(u, mid)[2] < 0:
                hi = mid
            else:
                lo = mid
        return hi
