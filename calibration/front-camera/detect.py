"""Detect both A4 ChArUco boards in a single front-camera frame.

Boards (carlosmccosta/charuco_detector, black_and_white vector A4, DICT_4X4_1000):
  coarse_40mm: 5x7 squares, 40 mm squares, 30 mm markers (ids 0..16), modern layout
  dense_20mm : 10x14 squares, 20 mm squares, 15 mm markers (ids 0..69), legacy layout
Marker ids overlap, so each board is detected inside its own image region.
Writes out/detections.json (pixel coords + board-frame mm coords) and an overlay.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent
img_path = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "raw/frame-3-telegram.jpg"
img = cv2.imread(str(img_path))
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
H, W = gray.shape
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

# Image regions split along the visible seam between the boards
# (~(775,742) -> (700,1199)). Settings chosen by a small parameter sweep.
BOARDS = {
    "coarse_40mm": dict(size=(5, 7), sq=40.0, mk=30.0, legacy=False, scale=1.0, win=23, clahe=False,
                        region=[(0, 650), (800, 650), (725, H), (0, H)]),
    "dense_20mm": dict(size=(10, 14), sq=20.0, mk=15.0, legacy=True, scale=2.0, win=53, clahe=True,
                       region=[(760, 650), (W, 650), (W, H), (685, H)]),
}


def detect(spec):
    mask = np.zeros_like(gray)
    cv2.fillPoly(mask, [np.array(spec["region"], np.int32)], 255)
    g = cv2.createCLAHE(3.0, (8, 8)).apply(gray) if spec["clahe"] else gray.copy()
    g = np.where(mask > 0, g, 255).astype(np.uint8)
    s = spec["scale"]
    if s != 1:
        g = cv2.resize(g, None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC)
    p = cv2.aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMax = spec["win"]
    p.adaptiveThreshWinSizeStep = 6
    p.polygonalApproxAccuracyRate = 0.03
    p.errorCorrectionRate = 0.8
    p.minMarkerPerimeterRate = 0.01
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    board = cv2.aruco.CharucoBoard(spec["size"], spec["sq"], spec["mk"], DICT)
    board.setLegacyPattern(spec["legacy"])
    cp = cv2.aruco.CharucoParameters()
    cp.tryRefineMarkers = True
    cp.minMarkers = 1
    cc, ci, _, mi = cv2.aruco.CharucoDetector(board, cp, p).detectBoard(g)
    if ci is None:
        raise SystemExit("no corners detected")
    ids = ci.flatten()
    px = cc.reshape(-1, 2) / s
    obj = board.getChessboardCorners()[ids][:, :2]
    return ids, px, obj, 0 if mi is None else len(mi)


out = {"image": img_path.name, "width": W, "height": H, "boards": {}}
for name, spec in BOARDS.items():
    ids, px, obj, nm = detect(spec)
    out["boards"][name] = dict(ids=ids.tolist(), px=px.tolist(), board_mm=obj.tolist(),
                               squares=list(spec["size"]), square_mm=spec["sq"], legacy=spec["legacy"])
    print(f"{name}: markers={nm} corners={len(ids)}")

(HERE / "out").mkdir(exist_ok=True)
(HERE / "out/detections.json").write_text(json.dumps(out, indent=1))

vis = img.copy()
colors = {"coarse_40mm": (0, 0, 255), "dense_20mm": (255, 0, 0)}
for name, d in out["boards"].items():
    for (x, y), i in zip(d["px"], d["ids"]):
        cv2.circle(vis, (int(x), int(y)), 5, colors[name], 2)
        cv2.putText(vis, str(i), (int(x) + 4, int(y) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, colors[name], 1)
cv2.imwrite(str(HERE / "out/detections.jpg"), vis)
