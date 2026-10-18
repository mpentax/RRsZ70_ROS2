"""Arm calibration fit (Claude, 2026-10-14).

Frames
  CF  camera-floor frame of the front camera: origin on the floor below the front camera, x forward, y left, z up (metres).
  B   the dense 20 mm ChArUco board lying on the floor (fixed during the sweep).
  G   gripper (pincher) camera optical frame.

Chain
  1. Gripper camera intrinsics from all pass-A board views (cv2.calibrateCamera on ChArUco corners).
  2. T_CF_B from pass-B front frames (front calibration is known); board assumed rigid and static.
  3. Arm model in CF: shoulder S=(sx,sy,sz), arm-plane yaw psi, vendor links L1/L2, joint offsets o3/o4/o5,
     wrist-pitch sign/scale k5, and a 6-DoF wrist->gripper-camera mount. Residual = board-corner reprojection
     error in every gripper frame. Joints come from the RoboClaw estimator (vendor step logic) or the commanded
     count, whichever fits better (both are reported).
  4. Arm marker (ArUco ID0, 15 mm) in pass-B front frames: fit its mount on the wrist and report its residual
     as an independent check.
Usage: python fit.py <sweep_dir>
"""
import glob, json, math, sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
D = Path(sys.argv[1])
FRONT = json.loads((ROOT / "front-camera/front_camera.json").read_text())
L1, L2 = 0.1775, 0.205
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
BOARD = cv2.aruco.CharucoBoard((10, 14), 0.020, 0.015, DICT); BOARD.setLegacyPattern(True)
BOARD_XYZ = BOARD.getChessboardCorners()          # metres, z=0


def charuco(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    p = cv2.aruco.DetectorParameters(); p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    det = cv2.aruco.CharucoDetector(BOARD, cv2.aruco.CharucoParameters(), p)
    cc, ci, mc, mi = det.detectBoard(g)
    if ci is None or len(ci) < 8: return None
    return ci.flatten(), cc.reshape(-1, 2)


def arm_marker(img):
    p = cv2.aruco.DetectorParameters(); p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    c, ids, _ = cv2.aruco.ArucoDetector(DICT, p).detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    if ids is None: return None
    # the board also carries id 0; the arm marker is the one far from the board's other markers (largest, highest in view)
    cand = [cc.reshape(4, 2) for cc, i in zip(c, ids.flatten()) if i == 0]
    if not cand: return None
    others = np.concatenate([cc.reshape(4, 2) for cc, i in zip(c, ids.flatten()) if i != 0]) if len(ids) > 1 else None
    def isolation(q):
        return np.inf if others is None else np.min(np.linalg.norm(others - q.mean(0), axis=1))
    best = max(cand, key=isolation)
    return best if isolation(best) > 80 else None


def rt_to_T(r, t):
    T = np.eye(4); T[:3, :3] = cv2.Rodrigues(np.asarray(r, float))[0]; T[:3, 3] = np.ravel(t); return T


def front_K():
    return np.array([[FRONT["f_px"], 0, FRONT["cx"]], [0, FRONT["f_px"], FRONT["cy"]], [0, 0, 1.]]), np.array([FRONT["k1"], FRONT["k2"], 0, 0, 0.])


def T_CF_frontcam():
    """front camera optical frame -> CF (x_cam = R (x_floor - C))."""
    R = np.asarray(FRONT["R_floor_to_cam"]); C = np.array([0, 0, FRONT["camera_height_mm"] / 1000])
    T = np.eye(4); T[:3, :3] = R.T; T[:3, 3] = C; return T


def load():
    caps = [json.loads(l) for l in open(D / "log.jsonl") if '"capture"' in l]
    A = [c for c in caps if c["pass"] == "A" and (D / c["file"]).exists()]
    B = [c for c in caps if c["pass"] == "B" and (D / c["file"]).exists()]
    return A, B


def main():
    A, B = load()
    print(f"{len(A)} gripper frames, {len(B)} front frames")
    # 1. gripper intrinsics
    obs = []
    for c in A:
        r = charuco(cv2.imread(str(D / c["file"])))
        if r is not None: obs.append((c, *r))
    objp = [BOARD_XYZ[ids].astype(np.float32) for _, ids, _ in obs]
    imgp = [px.astype(np.float32) for _, _, px in obs]
    flags = cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
    K0 = np.array([[900, 0, 800], [0, 900, 448], [0, 0, 1.]])
    rms, Kg, dg, rv, tv = cv2.calibrateCamera(objp, imgp, (1600, 896), K0, None, flags=flags | cv2.CALIB_USE_INTRINSIC_GUESS)
    print(f"gripper intrinsics: rms {rms:.2f}px f={Kg[0,0]:.1f} c=({Kg[0,2]:.1f},{Kg[1,2]:.1f}) k1={dg[0,0]:.3f} k2={dg[0,1]:.3f} from {len(obs)} views")

    # 2. board in CF from front frames
    Kf, df = front_K(); Tcf_fc = T_CF_frontcam(); boards = []; markers = []
    for c in B:
        img = cv2.imread(str(D / c["file"]))
        r = charuco(img)
        if r is not None and len(r[0]) >= 12:
            ok, rvec, tvec = cv2.solvePnP(BOARD_XYZ[r[0]], r[1], Kf, df)
            if ok: boards.append(Tcf_fc @ rt_to_T(rvec, tvec))
        m = arm_marker(img)
        if m is not None:
            s = 0.015 / 2; obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
            ok, rvec, tvec = cv2.solvePnP(obj, m, Kf, df, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok: markers.append((c, Tcf_fc @ rt_to_T(rvec, tvec)))
    if not boards: sys.exit("board not seen by the front camera")
    P = np.array([b[:3, 3] for b in boards])
    Tcf_b = boards[int(np.argmin(np.linalg.norm(P - np.median(P, 0), axis=1)))]
    print(f"board in CF from {len(boards)} front frames: origin {Tcf_b[:3,3].round(4)} (spread {np.ptp(P,0).round(4)} m)")
    print(f"arm marker seen in {len(markers)} front frames")

    # 3. arm model fit
    corners_cf = [(c, ids, px, (Tcf_b @ np.c_[BOARD_XYZ[ids], np.ones(len(ids))].T).T[:, :3]) for c, ids, px in obs]

    def wrist_T(p, j):
        sx, sy, sz, psi, o3, o4, o5, k5 = p[:8]
        m3, m4, m5 = j
        a1 = math.radians(270 - m3 + o3); a2 = math.radians(m4 - m3 + 90 + o4); a3 = a2 - math.radians(k5 * m5 + o5)
        ex, ez = L1 * math.cos(a1), L1 * math.sin(a1)
        wx, wz = ex + L2 * math.cos(a2), ez + L2 * math.sin(a2)
        cp, sp = math.cos(psi), math.sin(psi)
        T = np.eye(4)
        # arm plane = x-z plane rotated by psi about z; wrist link points along a3 within the plane
        T[:3, 3] = [sx + cp * wx, sy + sp * wx, sz + wz]
        xa = np.array([cp * math.cos(a3), sp * math.cos(a3), math.sin(a3)])
        ya = np.array([-sp, cp, 0.]); za = np.cross(xa, ya)
        T[:3, :3] = np.c_[xa, ya, za]; return T

    def cam_T(p, j):
        return wrist_T(p, j) @ rt_to_T(p[8:11], p[11:14])

    def residuals(p, key):
        res = []
        for c, ids, px, Xcf in corners_cf:
            T = np.linalg.inv(cam_T(p, c[key]))
            Xc = (T @ np.c_[Xcf, np.ones(len(Xcf))].T).T[:, :3]
            uv, _ = cv2.projectPoints(Xc, np.zeros(3), np.zeros(3), Kg, dg)
            res.append((uv.reshape(-1, 2) - px).ravel())
        return np.concatenate(res)

    # initial mount: camera looks along the wrist link (optical z = link x), image x = -link y
    R0 = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float).T     # columns: cam axes in wrist frame
    r0 = cv2.Rodrigues(R0)[0].ravel()
    results = {}
    for key in ("estimateJ", "tracked"):
        for c, *_ in corners_cf:
            e = c["estimate"]["joints"] or {}
            c["estimateJ"] = [e.get("m3", c["tracked"][0]), e.get("m4", c["tracked"][1]), e.get("m5", c["tracked"][2])]
        best = None
        for k5 in (1, -1):
            for mount in (r0, cv2.Rodrigues(R0 @ cv2.Rodrigues(np.array([0, 0, math.pi]))[0])[0].ravel()):
                p0 = np.r_[0.0, 0, 0.14, 0, 0, 0, 0, k5, mount, 0.03, 0, 0]
                try:
                    sol = least_squares(residuals, p0, args=(key,), loss="soft_l1", f_scale=5.0, max_nfev=4000)
                except Exception as e:
                    continue
                rmsp = float(np.sqrt(np.mean(sol.fun ** 2)))
                if best is None or rmsp < best[0]: best = (rmsp, sol)
        results[key] = best
        print(f"[{key}] reprojection RMS {best[0]:.2f}px")
    key = min(results, key=lambda k: results[k][0]); rmsp, sol = results[key]
    p = sol.x
    J = sol.jac; cov = np.linalg.pinv(J.T @ J) * (rmsp ** 2); sig = np.sqrt(np.diag(cov))
    names = ["sx", "sy", "sz", "psi", "o3", "o4", "o5", "k5"]
    print(f"best joints source: {key}")
    for n, v, s in zip(names, p[:8], sig[:8]):
        unit = "rad" if n == "psi" else ("deg" if n.startswith("o") else ("" if n == "k5" else "m"))
        print(f"  {n:4s} = {v: .4f} ± {s:.4f} {unit}")
    print(f"  camera mount t = {p[11:14].round(4)} m, r = {np.degrees(p[8:11]).round(1)} deg")

    # per-frame residuals
    per = []
    for c, ids, px, Xcf in corners_cf:
        T = np.linalg.inv(cam_T(p, c[key])); Xc = (T @ np.c_[Xcf, np.ones(len(Xcf))].T).T[:, :3]
        uv, _ = cv2.projectPoints(Xc, np.zeros(3), np.zeros(3), Kg, dg)
        per.append((c["n"], c[key], float(np.sqrt(np.mean(np.sum((uv.reshape(-1, 2) - px) ** 2, 1))))))
    print("per-frame RMS px:", ", ".join(f"{n}:{e:.1f}" for n, _, e in per))

    # 4. marker mount + check
    marker_fit = None
    if len(markers) >= 3:
        def mres(q):
            out = []
            for c, Tm in markers:
                j = [c["estimate"]["joints"].get(k, t) for k, t in zip(("m3", "m4", "m5"), c["tracked"])] if key == "estimateJ" else c["tracked"]
                pred = wrist_T(p, j) @ rt_to_T(q[:3], q[3:])
                out.append(pred[:3, 3] - Tm[:3, 3])
            return np.concatenate(out)
        ms = least_squares(mres, np.r_[0, 0, 0, 0.05, 0, 0])
        err = np.linalg.norm(ms.fun.reshape(-1, 3), axis=1)
        marker_fit = dict(t=ms.x[3:].tolist(), r=ms.x[:3].tolist(), err_mm=(err * 1000).round(1).tolist())
        print(f"arm marker mount on wrist t={np.round(ms.x[3:],4)} m; position residuals mm: {marker_fit['err_mm']}")

    out = dict(joints_source=key, reprojection_rms_px=rmsp, params=dict(zip(names, p[:8].tolist())), sigma=dict(zip(names, sig[:8].tolist())),
               camera_mount=dict(r=p[8:11].tolist(), t=p[11:14].tolist()),
               gripper_intrinsics=dict(K=Kg.tolist(), dist=dg.ravel().tolist(), rms_px=rms, size=[1600, 896]),
               board_in_CF=Tcf_b.tolist(), per_frame_rms_px=per, marker=marker_fit,
               frame="CF: floor below front camera, x forward, y left, z up (m)")
    (D / "fit.json").write_text(json.dumps(out, indent=1))
    print("wrote", D / "fit.json")


if __name__ == "__main__":
    main()
