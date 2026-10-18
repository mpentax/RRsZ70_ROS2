"""Offline physical-base reconstruction from durable MCP captures; no robot IO.

Usage: ../../arm-validation/.venv/bin/python analyze.py SESSION
A model-free single-joint line plus robust corner-space latent-angle fit is
identifiable without guessing wrist registration or treating commands as angles.
"""
import argparse,json,hashlib,math
from pathlib import Path
import cv2,numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from observations import detect,intrinsics,ROOT
from geometry import fit_axis,fit_pixel_axis,predict_pose


def summarize(errors):
    e=np.asarray(errors,dtype=float)
    return {'n':len(e),'rms':float(np.sqrt(np.mean(e**2))),'median':float(np.median(e)),'p95':float(np.percentile(e,95)),'max':float(e.max())} if len(e) else None


def front_registration(records,K=None,d=None):
    good=[o for o in records if o['camera']=='front' and o['board'] and o['board']['quality_accepted']]
    Ts=[np.array(o['board']['T_cam_board']) for o in good]
    R=Rotation.from_matrix([x[:3,:3] for x in Ts]).mean().as_rotvec();t=np.median([x[:3,3] for x in Ts],axis=0)
    if K is None:K,d=intrinsics('front')
    def res(v):
        out=[]
        for o in good:
            b=o['board'];X=np.array(b['obj']);px=np.array(b['px']);uv=cv2.projectPoints(X,v[:3],v[3:],K,d)[0].reshape(-1,2)
            out.extend(((uv-px)/np.sqrt(len(X))).ravel())
        return out
    sol=least_squares(res,np.r_[R,t],loss='soft_l1',f_scale=.5,max_nfev=250)
    T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(sol.x[:3]).as_matrix();T[:3,3]=sol.x[3:]
    errs=[]
    for o in good:
        b=o['board'];uv=cv2.projectPoints(np.array(b['obj']),sol.x[:3],sol.x[3:],K,d)[0].reshape(-1,2);errs.extend(np.linalg.norm(uv-b['px'],axis=1))
    return T,{'frames':[o['file'] for o in good],'point_errors_px':summarize(errs),'camera_height_above_board_m':float(abs(np.linalg.inv(T)[2,3]))}


def in_front(axis,T):
    n=T[:3,:3]@np.array(axis['direction']);p=T[:3,:3]@np.array(axis['closest_point'])+T[:3,3]
    n/=np.linalg.norm(n)
    if n[0]<0:n=-n
    p-=n*(p@n)
    Wcam=np.linalg.inv(T)[:3,3];sign=1 if Wcam[2]>0 else -1
    up=sign*T[:3,2];forward=np.array([0.,0,1.])-up*up[2];forward/=np.linalg.norm(forward)
    mid=p-n*p[0]/n[0] if abs(n[0])>.5 else p
    return {'direction_F':n.tolist(),'closest_point_to_front_optical_origin_m':p.tolist(),'point_at_front_optical_x0_m':mid.tolist(),'setback_from_optical_origin_m':float(-mid@forward),'axis_height_above_camera_m':float(mid@up),'axis_height_above_board_m':float(mid@up+abs(Wcam[2])),'floor_up_F':up.tolist(),'floor_forward_F':forward.tolist(),'station_definition':'intersection with front optical x=0 plane, NOT an anatomical shoulder lateral centre'}


def record(o):
    b=o['board'];return {'T':np.linalg.inv(np.array(b['T_cam_board'])),'obj':np.array(b['obj']),'px':np.array(b['px'])}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);ap.add_argument('--quick',action='store_true');ap.add_argument('--bootstrap',type=int,default=200);args=ap.parse_args();D=args.session.resolve()
    logs=[json.loads(x) for x in (D/'log.jsonl').read_text().splitlines()];caps={x['file']:x for x in logs if x.get('event')=='capture'}
    raw=sorted((D/'raw').glob('*.jpg'));version=hashlib.sha256(Path(__file__).with_name('observations.py').read_bytes()).hexdigest()
    cache=D/'observations.json';observed=[]
    if cache.exists():
        prev=json.loads(cache.read_text())
        if prev.get('detector_sha256')==version and prev.get('raw_count')==len(raw):observed=prev['frames']
    if not observed:
        for p in raw:
            file=str(p.relative_to(D));im=cv2.imread(str(p));camera='front' if im.shape[0]==1200 else 'gripper'
            o=detect(p,camera);o['file']=file;o['nominal_tracked']=caps.get(file,{}).get('tracked');o['capture_log']=caps.get(file);observed.append(o)
        cache.write_text(json.dumps({'detector_sha256':version,'raw_count':len(raw),'frames':observed},indent=1))
    byfile={o['file']:o for o in observed};groups=json.loads((D/'groups.json').read_text());fronts={}
    for block,names in {'A':['raw/003-deployed-front.jpg','raw/pilot-front-current.jpg','raw/front-A-reference.jpg','raw/A-stowed-front.jpg'], 'B':['raw/B-deployed-front.jpg','raw/B-front-reference-before.jpg','raw/B-front-reference-before2.jpg','raw/B-reference-Fafter.jpg'], 'D':['raw/D-deployed-front.jpg','raw/D-front230-85-a.jpg','raw/D-front230-85-b.jpg','raw/D-front240-95-before.jpg','raw/D-front240-95-before2.jpg','raw/D-reference240-95-Fafter.jpg','raw/D-front240-105.jpg','raw/D-front240-105-repeat.jpg','raw/final-idle-front.jpg']}.items():
        T,diag=front_registration([byfile[f] for f in names]);fronts[block]={'T_F_W':T.tolist(),**diag}
    K,d=intrinsics('gripper');sweeps={};holdouts={};rng=np.random.default_rng(8713);unc={};models={}
    for name,files in groups.items():
        available=[byfile[f] for f in files if byfile[f]['board']]
        if len(available)<3:continue
        rs=[record(o) for o in available];model_free=fit_axis([r['T'] for r in rs]);pixel=fit_pixel_axis(rs,K,d)
        T=np.array(fronts[name[0]]['T_F_W']);F=in_front(pixel,T)
        # The final complete pose is held out. No corners, translation or angle
        # from that pose enter training. Its second exposure supplies orientation
        # only for conditional geometry prediction, never a command-only claim.
        if len(available)>=4:
            train=rs[:-1];fit=fit_pixel_axis(train,K,d)
            target=available[-1];alt=byfile.get(target['file'].replace('.jpg','-settled.jpg'),target)
            angle_source=alt if alt.get('board') else target
            Rangle=record(angle_source)['T'][:3,:3]
            angle=float(np.array(fit['direction'])@Rotation.from_matrix(Rangle@np.array(fit['R0']).T).as_rotvec())
            predicted=predict_pose(fit,angle);X=rs[-1]['obj'];uv=cv2.projectPoints((X-predicted[:3,3])@predicted[:3,:3],np.zeros(3),np.zeros(3),K,d)[0].reshape(-1,2)
            e=np.linalg.norm(uv-rs[-1]['px'],axis=1)
            holdouts[name]={'training_files':[o['file'] for o in available[:-1]],'whole_pose_holdout':target['file'],'actual_angle_orientation_source':angle_source['file'],'uses_nominal_angle':False,'independent_sensor':False,'conditional_only':'angle measured from separate stationary exposure when available; correlated same-lens/print systematics remain','point_errors_px':summarize(e),'camera_center_error_m':float(np.linalg.norm(predicted[:3,3]-rs[-1]['T'][:3,3])),'observed_angle_deg':math.degrees(angle),'train_point_rms_px':fit['point_rms_px']}
        sweeps[name]={'files':[o['file'] for o in available],'quality_accepted_poses':sum(o['board']['quality_accepted'] for o in available),'model_free':model_free,'pixel_fit':pixel,'camera_relative_axis':F,'observed_span_deg':float(np.ptp(np.degrees(pixel['latent_angles_rad']))),'all_input_corners_used':True};models[name]=rs
        print(name,'span',round(sweeps[name]['observed_span_deg'],2),'rms',round(pixel['point_rms_px'],2),'setback/height mm',round(F['setback_from_optical_origin_m']*1000,2),round(F['axis_height_above_board_m']*1000,2),flush=True)
        if name.startswith('D_M3') and not args.quick:
            samples=[];fails=0
            for _ in range(args.bootstrap):
                ix=rng.integers(0,len(rs),len(rs))
                try:
                    a=fit_axis([rs[i]['T'] for i in ix]);f=in_front(a,T);samples.append([f['setback_from_optical_origin_m'],f['axis_height_above_camera_m']])
                except (ValueError,np.linalg.LinAlgError):fails+=1
            arr=np.array(samples)
            unc[name]={'method':'whole-pose resampling of model-free PnP SE3 axes, not pixel-bundle covariance','attempted':args.bootstrap,'successful':len(samples),'failures':fails,'conditional_1sigma_m':np.std(arr,axis=0,ddof=1).tolist(),'conditional_percentile95_m':np.percentile(arr,[2.5,97.5],axis=0).tolist(),'components':['setback','height_above_camera'],'excludes':['intrinsics extrapolation','board planarity','print scale','physical survey','remount variation']}
    # Independent whole redeployment axis reconstructions in their own freshly
    # registered front frames, not a third-session command prediction.
    replication={}
    for name in ['A_M3_45','B_M3_45','B_M3_75']:
        if name in sweeps and 'D_M3_85' in sweeps:
            a=sweeps[name]['camera_relative_axis'];b=sweeps['D_M3_85']['camera_relative_axis'];n=np.array(b['direction_F']);v=np.array(a['closest_point_to_front_optical_origin_m'])-b['closest_point_to_front_optical_origin_m']
            replication[name]={'axis_perpendicular_separation_m':float(np.linalg.norm(v-n*(v@n))),'axis_direction_difference_deg':float(np.degrees(np.arccos(np.clip(abs(np.array(a['direction_F'])@n),-1,1)))),'setback_difference_m':a['setback_from_optical_origin_m']-b['setback_from_optical_origin_m'],'height_difference_m':a['axis_height_above_camera_m']-b['axis_height_above_camera_m']}
    # Frozen mounts stay unknown anatomy; a local axis fit never resolves them.
    unknown={'physical_L2_m':None,'physical_M4_zero_deg':None,'shoulder_lateral_centre_m':None,'pedestal_landmarks_m':None,'bumper_setback_m':None,'physical_M5_axial_station_m':None,'print_scale_tolerance':None,'independently_verified_absolute_95percent_uncertainty_m':None}
    quality=[o for o in observed if o['board'] and o['board']['quality_accepted']];distinct={tuple(o['nominal_tracked'][:2]) for o in quality if o['nominal_tracked'] and o['nominal_tracked'][:2]!=[150,45] and o['nominal_tracked'][:2]!=[0,0]}
    counts={'retained_raw_frames':len(raw),'logged_capture_files':len(caps),'front_frames':sum(o['camera']=='front' for o in observed),'gripper_frames':sum(o['camera']=='gripper' for o in observed),'board_pose_frames':sum(o['board'] is not None for o in observed),'board_quality_accepted':len(quality),'board_quality_rejected_or_absent':len(observed)-len(quality),'elevated_ID68_frames':sum(o['marker'] is not None for o in observed),'distinct_nominal_M3M4':len({tuple(x['tracked'][:2]) for x in caps.values() if x.get('tracked')}),'distinct_quality_board_M3M4_labels':len(distinct),'arm_jog_commands':sum(x.get('method')=='app_arm_move' for x in logs),'chassis_pulses':sum(x.get('method')=='app_rc_move' for x in logs),'deploy_commands':sum(x.get('method')=='app_arm_out_compartment' for x in logs)}
    baseline_text=(ROOT/'ros2/src/z70_description/config/mount_calibration.yaml').read_text();baseline=json.loads(baseline_text[baseline_text.index('{'):]);old={'setback_m':baseline['front_camera_x_m']-baseline['measured_shoulder_base_m'][0],'axis_height_above_camera_m':baseline['measured_shoulder_base_m'][2]-baseline['front_camera_z_m'],'axis_height_above_floor_m':baseline['measured_shoulder_base_m'][2],'L1_m':baseline['measured_L1_m']}
    out={'status':'camera_relative_axes_reconstructed_absolute_anatomical_adoption_inconclusive','adoption':'none','counts':counts,'frames':'T_A_B maps B into A; W=OpenCV dense printed board coordinates, +z chosen by positive-depth PnP; F optical x right/y down/z forward; inferred floor-up sign points from board toward camera','front_registration':fronts,'sweeps':sweeps,'whole_redeployment_reconstruction_checks':replication,'unidentifiable':unknown,'baseline':old,'primary_conditional_axis':'D_M3_85','limitations':['No independent pedestal/bumper/shoulder survey or physical print tolerance','Existing one-plane front intrinsics extrapolated to elevated marker; no new independent multi-tilt lens calibration','No marker remount session','F/G/F brackets acquired only at visible reference poses, not every joint-sweep pose','No full global two-joint joint-bundle/planner/real-TF adoption package; pure-joint corner-space fits only','Later centred D reconstructions are independent checks against A/B; D reconstruction values are not claimed third-session frozen-model predictions','Pose labels nominal; latent measured rotations used for axis fits; refused/partial jogs retained','No independent proof of 5mm/1deg absolute placement; sensitivity scenarios are assumptions, not surveyed bounds']}
    if args.quick and (D/'uncertainty.json').exists():
        unc=json.loads((D/'uncertainty.json').read_text())
    (D/'fit.json').write_text(json.dumps(out,indent=2));(D/'holdout.json').write_text(json.dumps({'whole_pose_conditional_geometry':holdouts,'whole_redeployment_axis_checks':replication},indent=2));(D/'uncertainty.json').write_text(json.dumps(unc,indent=2))
    # Detection overlays are inspection evidence, not measurement annotations.
    (D/'overlays').mkdir(exist_ok=True)
    for name in ['raw/D-front240-95-before2.jpg','raw/D-front240-105.jpg','raw/066-D-M3-85-forward-g210-85-0.jpg','raw/079-D-M5-third-forward-g200-65-10.jpg']:
        o=byfile[name];im=cv2.imread(str(D/name))
        if o['board']:
            for x,y in o['board']['px']:cv2.circle(im,(round(x),round(y)),3,(0,255,0),1)
        if o['marker']:
            cv2.polylines(im,[np.array(o['marker']['px'],np.int32)],True,(0,0,255),2);cv2.putText(im,'elevated ID68 / 15mm black',(20,40),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,255),2)
        cv2.imwrite(str(D/'overlays'/Path(name).name),im)
    print(json.dumps({'counts':counts,'replication':replication,'holdouts':holdouts},indent=1))

if __name__=='__main__':main()
