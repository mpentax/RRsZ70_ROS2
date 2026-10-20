"""Calibrated optical rays and floor-plane scale correction for monocular depth clouds."""
import numpy as np


def camera_rays(cal, width, height):
    # Sample pixel centres at original calibration resolution, including resize offsets.
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
    # Bottom ~8% may contain the bumper and near-plane calibration is weakest.
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
