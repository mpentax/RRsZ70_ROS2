import sys,json,hashlib
from pathlib import Path
import numpy as np,cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
BASE=HERE.parent
sys.path.insert(0,str(BASE/'physical_base'))
import observations as ob
sys.path.insert(0,str(BASE/'front_pincher_joint'))
import joint as j
read=lambda p:json.loads(p.read_text())
def save(name,v):(HERE/name).write_text(json.dumps(v,indent=2))
models=read(HERE/'frozen-models/evaluation-models.json')
g=read(HERE/'frozen-models/proposed-camera-info.json')
# Standard camera-info artifact uses K / D (inspect schema explicitly).
Kg=np.array(g.get('K',g.get('camera_matrix')),float).reshape(3,3)
dg=np.array(g.get('D',g.get('dist',g.get('distortion_coefficients'))),float)
orig=ob.intrinsics
ob.intrinsics=lambda camera,perturb=None:(Kg,dg) if camera!='front' else (np.array(models['fixed']['K']),np.array(models['fixed']['dist']))
def detect(file,camera):
 o=ob.detect(HERE/'raw'/file,camera,floor_min_y=650)
 save(file.replace('.jpg','-obs.json'),o)
 print(json.dumps({'file':file,'camera':camera,'board':None if not o['board'] else {k:o['board'][k] for k in ['kind','accepted_corners','quality_accepted','full_rms_px','T_cam_board']},'marker':None if not o['marker'] else {k:o['marker'][k] for k in ['px','absolute_height_over_board_m']}}))
 return o
if __name__=='__main__':
 if sys.argv[1]=='detect':detect(sys.argv[2],sys.argv[3])
 elif sys.argv[1]=='freeze':
  fs=list((HERE/'frozen-models').glob('*.json'))+[Path(__file__),BASE/'front_pincher_joint/joint.py',BASE/'physical_base/observations.py',BASE/'live.mjs']
  save('frozen-hashes.json',{str(p.relative_to(BASE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in fs})
  save('prospective-protocol.json',dict(block='owner-repositioned-new-W-block',models=['fixed','fcyk1'],registration='each model fresh T_F_W from preflight F FLOOR ONLY; no marker input',frozen=['Kf','Df','Kg','Dg','T_G_M'],target='archived achieved 065-g250-45-40-nominal.jpg; label actual departure, not assumed absolute angle',comparator='nominal25045roll0',brackets='F/G/G/F, 2-3 each pose if feasible',limits={'route_jogs':24,'roll_jogs':12,'minutes':15},stationarity={'marker_translation_m':.003,'marker_orientation_deg':12.779567131728307,'board_translation_m':.003,'board_orientation_deg':3},adoption='NO production adoption; fixed-model discriminator only',marker_fitting=False,exposure_times='unknown',scale_systematics='unsurveyed print/flatness/lens height; carry unknown'))
 elif sys.argv[1]=='register':
  oo=[read(HERE/n) for n in sys.argv[2:]];reg={}
  for name,m in models.items():
   K=np.array(m['K']);d=np.array(m['dist']);b=oo[0]['board'];ok,r,t=cv2.solvePnP(np.array(b['obj']),np.array(b['px']),K,d)
   T=ob.transform(r,t)
   def res(v):return np.concatenate([(j.project(o['board']['obj'],j.mat(v),K,d)-o['board']['px']).ravel() for o in oo])
   s=least_squares(res,j.vec(T),loss='soft_l1',f_scale=.5,max_nfev=300)
   T=j.mat(s.x);positive=all(np.all((np.array(o['board']['obj'])@T[:3,:3].T+T[:3,3])[:,2]>0) for o in oo)
   if not s.success or not positive:raise ValueError('registration invalid')
   reg[name]={'T_F_W':T.tolist(),'success':bool(s.success),'positive_depth':positive,'floor_errors':j.stats([res(s.x).reshape(-1,2)]),'training_floor_files':[o['file'] for o in oo]}
  save('new-block-floor-registration.json',reg);print(json.dumps(reg))
