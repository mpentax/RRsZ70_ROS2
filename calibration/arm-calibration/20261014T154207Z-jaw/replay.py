"""Replay real front exposures through production perception without robot calls."""
import json
from pathlib import Path
import cv2
from z70_manipulation import perception,planner
D=Path(__file__).resolve().parent;ROOT=D.parents[2]
rows=[]
for file in ['f240-85-0-open.jpg','f240-85-closed.jpg','f240-95-0-open.jpg','f240-105-0-open.jpg','f240-115-0-open.jpg','f250-115-0-open.jpg','f250-115-actual-open.jpg','99-stowed-front.jpg']:
 im=cv2.imread(str(D/file));row={'file':file}
 try:row['jaw']=perception.front_jaw(im,ROOT)
 except LookupError as e:row['jaw_error']=str(e)
 row['objects']=perception.front_objects(im,ROOT)
 rows.append(row)
(D/'production-replay.json').write_text(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2))
