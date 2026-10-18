"""Single-image floor-plane calibration of the Z70 front camera.

Both ChArUco boards lie flat on the same floor. We fit, jointly:
  intrinsics : f (fx = fy), optional principal point, radial k1, k2 (OpenCV convention)
  extrinsics : camera pose w.r.t. board A's floor frame (rvec, tvec)
  coplanarity: board B pose inside the same floor plane (theta, dx, dy)

From the fit we report focal length / FOV, camera height above floor, pitch/roll,
residuals, and cross-validated (held-out corner) error. Output: out/front_camera.json.
"""
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).parent
det = json.loads((HERE / "out/detections.json").read_text())
W, H = det["width"], det["height"]
A, B = det["boards"]["coarse_40mm"], det["boards"]["dense_20mm"]
pxA, objA = np.array(A["px"]), np.array(A["board_mm"])
pxB, objB = np.array(B["px"]), np.array(B["board_mm"])


def K_of(f, cx, cy):
    return np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]])


def unpack(p, cfg):
    i = 0
    f = p[i]; i += 1
    if cfg["free_pp"]:
        cx, cy = p[i:i + 2]; i += 2
    else:
        cx, cy = W / 2, H / 2
    k = np.zeros(5)
    nk = cfg["nk"]
    k[:nk] = p[i:i + nk]; i += nk
    rvec, tvec = p[i:i + 3], p[i + 3:i + 6]; i += 6
    th, dx, dy = p[i:i + 3]
    return f, cx, cy, k, rvec, tvec, (th, dx, dy)


def board_B_in_A(objB_, th, dx, dy):
    c, s = np.cos(th), np.sin(th)
    xy = objB_ @ np.array([[c, s], [-s, c]]) + [dx, dy]
    return np.c_[xy, np.zeros(len(xy))]


def project(p, cfg, objA_, objB_):
    f, cx, cy, k, rvec, tvec, (th, dx, dy) = unpack(p, cfg)
    K = K_of(f, cx, cy)

    def proj(pts3):
        if len(pts3) == 0:
            return np.zeros((0, 2))
        q, _ = cv2.projectPoints(pts3, rvec, tvec, K, k)
        return q.reshape(-1, 2)

    return proj(np.c_[objA_, np.zeros(len(objA_))]), proj(board_B_in_A(objB_, th, dx, dy))


def init_params(cfg):
    # Initial guess: OpenCV calibrateCamera on board A (pp fixed, no distortion),
    # then board B's in-plane pose from a separate solvePnP.
    f0 = 900.0
    K0 = K_of(f0, W / 2, H / 2)
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO
             | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
    o = [np.c_[objA, np.zeros(len(objA))].astype(np.float32), np.c_[objB, np.zeros(len(objB))].astype(np.float32)]
    im = [pxA.astype(np.float32), pxB.astype(np.float32)]
    _, K, _, rv, tv = cv2.calibrateCamera(o, im, (W, H), K0, np.zeros(5), flags=flags)
    RA, _ = cv2.Rodrigues(rv[0]); RB, _ = cv2.Rodrigues(rv[1])
    # B -> A transform (should be in-plane): T_A^-1 * T_B
    R = RA.T @ RB; t = RA.T @ (tv[1].ravel() - tv[0].ravel())
    th = np.arctan2(R[1, 0], R[0, 0])
    p = [K[0, 0]] + ([W / 2, H / 2] if cfg["free_pp"] else []) + [0.0] * cfg["nk"]
    return np.array(p + list(rv[0].ravel()) + list(tv[0].ravel()) + [th, t[0], t[1]])


def fit(cfg, maskA=None, maskB=None):
    mA = np.ones(len(objA), bool) if maskA is None else maskA
    mB = np.ones(len(objB), bool) if maskB is None else maskB

    def res(p):
        a, b = project(p, cfg, objA[mA], objB[mB])
        return np.r_[(a - pxA[mA]).ravel(), (b - pxB[mB]).ravel()]

    p0 = init_params(cfg)
    if mA.all() and mB.all() or (mA.sum() > 0 and mB.sum() > 0):
        return least_squares(res, p0, method="lm", x_scale="jac")
    # single-board fit: the other board's in-plane pose is unconstrained; hold it fixed
    free = np.ones(len(p0), bool); free[-3:] = False

    def res_sub(q):
        p = p0.copy(); p[free] = q
        return res(p)

    r = least_squares(res_sub, p0[free], method="lm", x_scale="jac")
    p = p0.copy(); p[free] = r.x
    r.x = p
    return r


def rms(e):
    return float(np.sqrt(np.mean(np.sum(e.reshape(-1, 2) ** 2, 1))))


def describe(p, cfg):
    f, cx, cy, k, rvec, tvec, (th, dx, dy) = unpack(p, cfg)
    R, _ = cv2.Rodrigues(rvec)
    cam_in_floor = -R.T @ tvec            # camera centre in board-A floor frame (mm)
    n_cam = R[:, 2]                       # floor normal in camera frame
    height = abs(cam_in_floor[2])
    # pitch: angle of optical axis below the floor-parallel plane
    z = np.array([0, 0, 1.0])
    n = n_cam if n_cam @ (R @ np.zeros(3) + tvec) > 0 else -n_cam  # normal pointing from floor to camera side
    pitch_down = np.degrees(np.arcsin(abs(z @ n_cam)))
    # roll: tilt of image x-axis relative to the floor plane
    roll = np.degrees(np.arcsin(np.clip(np.array([1.0, 0, 0]) @ n_cam, -1, 1)))
    return dict(
        f_px=float(f), cx=float(cx), cy=float(cy), dist_k1k2p1p2k3=[float(x) for x in k],
        hfov_deg=float(np.degrees(2 * np.arctan(W / 2 / f))),
        vfov_deg=float(np.degrees(2 * np.arctan(H / 2 / f))),
        dfov_deg=float(np.degrees(2 * np.arctan(np.hypot(W, H) / 2 / f))),
        camera_height_mm=float(height), pitch_down_deg=float(pitch_down), roll_deg=float(roll),
        boardB_in_A=dict(theta_deg=float(np.degrees(th)), dx_mm=float(dx), dy_mm=float(dy)),
        rvec=rvec.tolist(), tvec=tvec.tolist())


def cross_val(cfg, folds=5, seed=0):
    rng = np.random.default_rng(seed)
    fa = rng.integers(0, folds, len(objA)); fb = rng.integers(0, folds, len(objB))
    errs = []
    for k in range(folds):
        r = fit(cfg, fa != k, fb != k)
        a, b = project(r.x, cfg, objA, objB)
        errs += list(np.hypot(*(a - pxA)[fa == k].T)) + list(np.hypot(*(b - pxB)[fb == k].T))
    return float(np.sqrt(np.mean(np.square(errs))))


CONFIGS = {
    "pinhole_no_distortion": dict(free_pp=False, nk=0),
    "k1": dict(free_pp=False, nk=1),
    "k1k2": dict(free_pp=False, nk=2),
    "k1k2_free_pp": dict(free_pp=True, nk=2),
}


def main():
    results = {}
    for name, cfg in CONFIGS.items():
        r = fit(cfg)
        a, b = project(r.x, cfg, objA, objB)
        d = describe(r.x, cfg)
        # 1-sigma parameter uncertainty from the Jacobian
        J = r.jac; dof = max(1, len(r.fun) - len(r.x)); s2 = np.sum(r.fun ** 2) / dof
        cov = np.linalg.pinv(J.T @ J) * s2
        d["f_px_sigma"] = float(np.sqrt(cov[0, 0]))
        d.update(rms_px=rms(np.r_[a - pxA, b - pxB]), rms_A_px=rms(a - pxA), rms_B_px=rms(b - pxB),
                 max_px=float(np.max(np.hypot(*np.r_[a - pxA, b - pxB].T))),
                 heldout_rms_px=cross_val(cfg), n_points=len(objA) + len(objB), params=r.x.tolist())
        results[name] = d
        print(f"{name:24s} f={d['f_px']:7.1f}±{d['f_px_sigma']:5.1f}px HFOV={d['hfov_deg']:5.1f} "
              f"k={np.round(d['dist_k1k2p1p2k3'][:2], 3)} pp=({d['cx']:.0f},{d['cy']:.0f}) "
              f"h={d['camera_height_mm']:5.1f}mm pitch={d['pitch_down_deg']:5.1f} roll={d['roll_deg']:5.2f} "
              f"rms={d['rms_px']:.2f} (A {d['rms_A_px']:.2f}, B {d['rms_B_px']:.2f}) "
              f"heldout={d['heldout_rms_px']:.2f} Bth={d['boardB_in_A']['theta_deg']:.1f}")
    (HERE / "out/fit_results.json").write_text(json.dumps(results, indent=1))
    return results


if __name__ == "__main__":
    main()
