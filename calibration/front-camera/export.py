"""Export the floor calibration, validate metric distances, draw visual checks.

  1. Choose model (default k1k2_free_pp) and write front_camera.json in a robot-centric
     floor frame (origin under the camera, x forward, y left, z up).
  2. Held-out metric test: fit using ONE board only, map the OTHER board's detected
     corners to the floor with floor_geometry, and compare every pairwise distance
     against the printed grid (known square size).
  3. Overlays: floor range rings / bearing lines on the image, and an undistorted image.
"""
import itertools
import json
import sys
from pathlib import Path

import cv2
import numpy as np

import fit as F
from floor_geometry import FrontCamera

HERE = Path(__file__).parent
MODEL = sys.argv[1] if len(sys.argv) > 1 else "k1k2_free_pp"


def to_calibration(p, cfg):
    f, cx, cy, k, rvec, tvec, _ = F.unpack(p, cfg)
    RA, _ = cv2.Rodrigues(np.asarray(rvec))          # board-A frame -> camera
    C = -RA.T @ np.asarray(tvec)                      # camera centre in board-A frame
    up = np.array([0, 0, 1.0]) * np.sign(C[2])        # floor normal pointing at camera
    opt = RA.T @ np.array([0, 0, 1.0])                # optical axis in board-A frame
    fwd = opt - (opt @ up) * up; fwd /= np.linalg.norm(fwd)
    left = np.cross(up, fwd)
    M = np.vstack([fwd, left, up])                    # board-A -> floor axes
    R_floor_to_cam = RA @ M.T
    height = float(abs(C[2]))
    pitch = float(np.degrees(np.arcsin(-(opt @ up))))
    return dict(width=F.W, height=F.H, model=MODEL, f_px=float(f), cx=float(cx), cy=float(cy),
                k1=float(k[0]), k2=float(k[1]), camera_height_mm=height, pitch_down_deg=pitch,
                R_floor_to_cam=R_floor_to_cam.tolist(),
                hfov_deg=float(np.degrees(2 * np.arctan(F.W / 2 / f))),
                vfov_deg=float(np.degrees(2 * np.arctan(F.H / 2 / f))),
                note="Fitted from ONE image (frame-3, Telegram re-encode), two coplanar A4 ChArUco "
                     "boards on the floor. Floor-plane use only. See README.md for accuracy.")


def heldout_metric(cfg, train):
    """Fit on one board, measure the other board's grid in mm."""
    nA, nB = len(F.objA), len(F.objB)
    mA = np.full(nA, train == "A"); mB = np.full(nB, train == "B")
    r = F.fit(cfg, mA, mB)
    cam = calib_cam(to_calibration(r.x, cfg))
    px, obj = (F.pxB, F.objB) if train == "A" else (F.pxA, F.objA)
    floor = np.array([[cam.pixel_to_floor(u, v)[k] for k in ("forward_mm", "left_mm")] for u, v in px])
    errs, rel = [], []
    for i, j in itertools.combinations(range(len(obj)), 2):
        true = np.linalg.norm(obj[i] - obj[j]); est = np.linalg.norm(floor[i] - floor[j])
        errs.append(est - true); rel.append((est - true) / true)
    errs, rel = np.array(errs), np.array(rel)
    rng = np.hypot(floor[:, 0], floor[:, 1])
    return dict(train=train, test="B" if train == "A" else "A", pairs=len(errs),
                median_abs_err_mm=float(np.median(np.abs(errs))), p95_abs_err_mm=float(np.percentile(np.abs(errs), 95)),
                median_rel_err_pct=float(100 * np.median(np.abs(rel))), mean_scale_bias_pct=float(100 * np.mean(rel)),
                test_range_mm=[float(rng.min()), float(rng.max())], f_px=float(r.x[0]))


def calib_cam(d):
    return FrontCamera(d["f_px"], d["cx"], d["cy"], d["k1"], d["k2"], d["R_floor_to_cam"],
                       d["camera_height_mm"], d["width"], d["height"])


def main():
    cfg = F.CONFIGS[MODEL]
    r = F.fit(cfg)
    calib = to_calibration(r.x, cfg)
    (HERE / "front_camera.json").write_text(json.dumps(calib, indent=1))
    cam = FrontCamera.load(HERE / "front_camera.json")

    # self-consistency: floor_geometry reproduces OpenCV projection of the fit
    a, b = F.project(r.x, cfg, F.objA, F.objB)
    print(f"model {MODEL}: f={calib['f_px']:.1f}px HFOV={calib['hfov_deg']:.1f} VFOV={calib['vfov_deg']:.1f} "
          f"h={calib['camera_height_mm']:.1f}mm pitch={calib['pitch_down_deg']:.2f} deg")
    print("horizon row at centre:", round(cam.horizon_v(), 1))

    val = {"model": MODEL, "heldout": []}
    for cfg_name in ("k1k2", MODEL, "pinhole_no_distortion"):
        for train in ("A", "B"):
            if train == "A" and F.CONFIGS[cfg_name]["free_pp"]:
                continue  # 17 corners cannot also pin the principal point
            h = heldout_metric(F.CONFIGS[cfg_name], train); h["config"] = cfg_name
            val["heldout"].append(h)
            print(f"[{cfg_name:22s}] train {h['train']} -> test {h['test']}: median |err| {h['median_abs_err_mm']:.1f}mm "
                  f"p95 {h['p95_abs_err_mm']:.1f}mm, median rel {h['median_rel_err_pct']:.1f}%, "
                  f"scale bias {h['mean_scale_bias_pct']:+.1f}%, test range {h['test_range_mm'][0]:.0f}-{h['test_range_mm'][1]:.0f}mm")

    # a few reference distances
    table = []
    for v in (1199, 1100, 1000, 900, 850, 800, 760, 720, 700, 680):
        p = cam.pixel_to_floor(cam.cx, v)
        table.append((v, None if p is None else round(p["forward_mm"])))
    val["centre_column_row_to_forward_mm"] = table
    print("centre column row -> forward mm:", table)
    (HERE / "out/validation.json").write_text(json.dumps(val, indent=1))

    # overlay: range rings every 100 mm, bearing rays every 15 deg
    img = cv2.imread(str(HERE / "raw/frame-3-telegram.jpg"))
    ov = img.copy()
    for R in list(range(100, 1001, 100)) + [1500, 2000, 3000]:
        pts = [cam.floor_to_pixel(R * np.cos(t), R * np.sin(t)) for t in np.radians(np.linspace(-80, 80, 321))]
        pts = np.array([q for q in pts if q and -50 < q[0] < F.W + 50 and -50 < q[1] < F.H + 50], np.int32)
        if len(pts) > 1:
            cv2.polylines(ov, [pts], False, (0, 255, 255), 2)
            m = pts[len(pts) // 2]
            cv2.putText(ov, f"{R / 10:.0f}cm", (int(m[0]) + 5, int(m[1]) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    for bdeg in range(-45, 46, 15):
        t = np.radians(bdeg)
        pts = [cam.floor_to_pixel(R * np.cos(t), R * np.sin(t)) for R in np.linspace(50, 5000, 200)]
        pts = np.array([q for q in pts if q and 0 <= q[0] < F.W and 0 <= q[1] < F.H], np.int32)
        if len(pts) > 1:
            cv2.polylines(ov, [pts], False, (255, 0, 255), 1)
    hv = cam.horizon_v()
    cv2.line(ov, (0, int(hv)), (F.W, int(hv)), (0, 128, 255), 1)
    cv2.putText(ov, "floor horizon", (10, int(hv) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 2)
    for (u, v), (pu, pv) in zip(np.r_[F.pxA, F.pxB], np.r_[a, b]):
        cv2.circle(ov, (int(u), int(v)), 4, (0, 0, 255), 1)
        cv2.circle(ov, (int(pu), int(pv)), 1, (0, 255, 0), -1)
    cv2.imwrite(str(HERE / "out/floor_overlay.jpg"), ov)

    K = np.array([[cam.f, 0, cam.cx], [0, cam.f, cam.cy], [0, 0, 1]])
    D = np.array([cam.k1, cam.k2, 0, 0, 0])
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (F.W, F.H), 0.0)
    und = cv2.undistort(img, K, D, None, newK)
    cv2.imwrite(str(HERE / "out/undistorted.jpg"), np.hstack([img, und])[::2, ::2])


if __name__ == "__main__":
    main()
