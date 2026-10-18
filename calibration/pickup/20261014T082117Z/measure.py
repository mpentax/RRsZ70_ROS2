"""Floor-contact geometry; coordinates must come from an actual frame."""
import json
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'front-camera'))
from floor_geometry import FrontCamera
cam = FrontCamera.load(pathlib.Path(__file__).resolve().parents[2] / 'front-camera/front_camera.json')
for name, u, v in json.loads(sys.argv[1]):
    print(json.dumps({'target': name, 'pixel': [u,v], 'floor': cam.pixel_to_floor(u,v)}))
