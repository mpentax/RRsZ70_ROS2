"""Bootstrap the fit (resample corners with replacement) to get parameter spread,
then turn pitch/f/height spread into floor-range uncertainty vs distance."""
import json
from pathlib import Path

import numpy as np

import export as E
import fit as F
from floor_geometry import FrontCamera

HERE = Path(__file__).parent
cfg = F.CONFIGS[E.MODEL]
rng = np.random.default_rng(1)
pxA0, objA0, pxB0, objB0 = F.pxA, F.objA, F.pxB, F.objB
cams, rows = [], []
for _ in range(150):
    ia = rng.integers(0, len(objA0), len(objA0)); ib = rng.integers(0, len(objB0), len(objB0))
    F.pxA, F.objA, F.pxB, F.objB = pxA0[ia], objA0[ia], pxB0[ib], objB0[ib]
    r = F.fit(cfg)
    c = E.to_calibration(r.x, cfg)
    rows.append((c["f_px"], c["hfov_deg"], c["camera_height_mm"], c["pitch_down_deg"], c["k1"], c["k2"]))
    cams.append(E.calib_cam(c))
F.pxA, F.objA, F.pxB, F.objB = pxA0, objA0, pxB0, objB0
rows = np.array(rows)
names = ["f_px", "hfov_deg", "camera_height_mm", "pitch_down_deg", "k1", "k2"]
spread = {n: dict(mean=float(rows[:, i].mean()), std=float(rows[:, i].std())) for i, n in enumerate(names)}
for n, s in spread.items():
    print(f"{n:18s} {s['mean']:9.4f} ± {s['std']:.4f}")

ref = FrontCamera.load(HERE / "front_camera.json")
table = []
for D in (150, 300, 500, 750, 1000, 1500, 2000, 3000):
    u, v = ref.floor_to_pixel(D, 0)
    ds = [c.pixel_to_floor(u, v) for c in cams]
    ds = np.array([d["forward_mm"] if d else np.nan for d in ds])
    per_px = ref.pixel_to_floor(u, v + 1)["forward_mm"] - ref.pixel_to_floor(u, v)["forward_mm"]
    table.append(dict(range_mm=D, image_row=round(v, 1), calib_std_mm=float(np.nanstd(ds)),
                      mm_per_px=float(abs(per_px))))
    print(f"range {D:5d}mm row {v:6.1f}  calib 1σ ±{np.nanstd(ds):6.1f}mm   1px = {abs(per_px):6.1f}mm")
(HERE / "out/uncertainty.json").write_text(json.dumps(dict(bootstrap=spread, range=table), indent=1))
