"""Offline candidates under the RETAINED provisional jaw; no robot access."""
import json,math
from pathlib import Path
import numpy as np
from z70_manipulation import planner as p
D=Path(__file__).resolve().parent
rows=[]
for m3 in range(90,271,10):
 for m4 in range(35,171,10):
  st=(m3,m4);J=p.measured_point(st,p._CALIBRATION['effective_jaw_from_elbow_m'])
  if not p.safe(st) or not (.35<=J[0]<=.50):continue
  rows.append(dict(joints=[m3,m4,0],jaw_base_m=J,equator_error_m=J[2]-.033,nominal_height_ge_3cm=J[2]>=.03,lower_bound_ge_3cm=p.tip_floor_bound(st)>=.03))
rows.sort(key=lambda q:abs(q['equator_error_m']))
poses={}
for st in [(250,105),(250,115),(240,95),(240,105)]:
 poses[str(st)]=dict(jaw_base_m=p.measured_point(st,p._CALIBRATION['effective_jaw_from_elbow_m']),firmware_wrist_x_m=p.wrist(st)[0],safe=p.safe(st),lower_contact_bound_m=p.tip_floor_bound(st))
hover=p.hover_plan();near_hover=(230,85);target=p.grasp_point();near=dict(state=list(near_hover),jaw_base_m=p.measured_point(near_hover,p._CALIBRATION['effective_jaw_from_elbow_m']),target_equator_pixel=p.gripper_pixel(near_hover,target[:2]+[.033]),descent_steps=p.descent_plan(near_hover))
fit=json.loads((D/'fit.json').read_text());approach=p.approach(fit['object_final']['rotation_center_m'])
out=dict(status='provisional predictions ONLY; new run did not resolve exact contact',jaw_vector_from_elbow_m=p._CALIBRATION['effective_jaw_from_elbow_m'],jaw_uncertainty_bound_m=p._CALIBRATION['jaw_uncertainty_bound_m'],GRASP=list(p.GRASP)+[0],GRASP_base_m=p.measured_point(p.GRASP,p._CALIBRATION['effective_jaw_from_elbow_m']),grid_candidates=rows,poses=poses,observation_hover=hover,near_hover=near,nominal_approach_from_final_visible_contact=approach)
(D/'recommendations.json').write_text(json.dumps(out,indent=1));print(json.dumps(out,indent=1))
