"""Find coloured objects in one front frame: python find.py <jpg> -> JSON (z70_manipulation perception, no VLM)."""
import json, sys
from pathlib import Path
import cv2, yaml
REPO = Path("/Users/mpentax/Code/RoboRockZ70-Connector")
sys.path.insert(0, str(REPO / "ros2/src/z70_manipulation"))
from z70_manipulation import perception as p
th = yaml.safe_load((REPO / "ros2/src/z70_manipulation/config/colors.yaml").read_text())
im = cv2.imread(sys.argv[1])
raw = p.detect(im, thresholds=th)
try:
    objs = p.front_objects(im, str(REPO), thresholds=th)
except Exception as e:
    objs = {"error": repr(e)}
def clean(o):
    if isinstance(o, dict): return {k: clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)): return [clean(v) for v in o]
    return o.tolist() if hasattr(o, "tolist") else o
print(json.dumps({"raw": clean(raw)[:10], "objects": clean(objs)}, default=str))
