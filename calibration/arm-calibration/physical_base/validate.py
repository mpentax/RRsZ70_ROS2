"""Independent offline reconstruction checks and systematic sensitivity.

All sensor evidence comes from retained captures. Artificial values below are
explicit calibration sensitivity scenarios, NEVER fabricated robot observations.
"""
import argparse,json,math,copy
from pathlib import Path
import numpy as np,cv2
from scipy.spatial.transform import Rotation
from geometry import fit_axis,fit_circle_axis,fit_pixel_axis
from observations import intrinsics
from analyze import record,in_front,front_registration,summarize,ROOT


def angle(A,B):return float(np.degrees(np.linalg.norm(Rotation.from_matrix(A[:3,:3]@B[:3,:3].T).as_rotvec())))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);ap.add_argument('--quick',action='store_true');a=ap.parse_args();D=a.session
    fit=json.loads((D/'fit.json').read_text());obs=json.loads((D/'observations.json').read_text())['frames'];by={o['file']:o for o in obs};groups=json.loads((D/'groups.json').read_text());TF=np.array(fit['front_registration']['D']['T_F_W']);Kg,dg=intrinsics('gripper');Kf,df=intrinsics('front')
    circles={}
    for key in ['D_M3_45','D_M3_85','D_M3_95']:
        records=[record(by[p]) for p in groups[key]];c=fit_circle_axis([r['T'][:3,3] for r in records]);F=in_front(c,TF);ref=fit['sweeps'][key]['camera_relative_axis'];n=np.array(ref['direction_F']);delta=np.array(F['closest_point_to_front_optical_origin_m'])-ref['closest_point_to_front_optical_origin_m']
        circles[key]={'position_only_circle':c,'axis_F':F,'perpendicular_distance_from_pixel_screw_line_m':float(np.linalg.norm(delta-n*(delta@n))),'axis_direction_difference_deg':float(np.degrees(np.arccos(np.clip(abs(np.array(F['direction_F'])@n),-1,1)))),'independent_metrology':False,'method':'camera-center positions only; no camera orientations or nominal angles in circle fit'}
    # Register unknown rigid camera->marker mount at one complete pose, freeze,
    # predict *other front pixels* from pincher floor-board pose at held-out pose.
    f1='raw/D-front240-95-before2.jpg';g1='raw/D-reference240-95-G.jpg';f2='raw/D-front240-105.jpg';g2='raw/D-reference240-105-G.jpg'
    T_F_M=np.array(by[f1]['marker']['T_cam_marker']);T_W_G=record(by[g1])['T'];mount=np.linalg.inv(TF@T_W_G)@T_F_M
    pred=TF@record(by[g2])['T']@mount;h=.0075;X=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]])
    uv=cv2.projectPoints(X,Rotation.from_matrix(pred[:3,:3]).as_rotvec(),pred[:3,3],Kf,df)[0].reshape(4,2);e=np.linalg.norm(uv-by[f2]['marker']['px'],axis=1);actual=np.array(by[f2]['marker']['T_cam_marker'])
    cross={'training_front_file':f1,'training_pincher_file':g1,'heldout_front_file':f2,'heldout_pincher_file':g2,'heldout_predicted_from_pincher_board':True,'validates_physical_shoulder_or_L2':False,'mount_T_G_M':mount.tolist(),'point_errors_px':summarize(e),'marker_translation_error_m':float(np.linalg.norm(pred[:3,3]-actual[:3,3])),'marker_orientation_error_deg':angle(pred,actual),'predicted_marker_px':uv.tolist(),'scope':'fixed relative marker/pincher optical mount check only; free mount cancels shoulder/L2, so NOT shoulder placement proof'}
    # Front-only ID68 M5 trajectory corroborates rolling attachment independently
    # of floor-board camera trajectory. Keep planar-pose branch ambiguity visible.
    marker_files=['raw/B-reference-Fafter.jpg']+[f'raw/B-M5front-240-105-{k}.jpg' for k in [10,20,30,40]]
    MT=[np.array(by[p]['marker']['T_cam_marker']) for p in marker_files];ma=fit_axis(MT)
    front_m5={'files':marker_files,'observed_rotation_deg_from_zero':[angle(T,MT[0]) for T in MT],'axis_F':ma,'planar_branch_policy':'lowest positive-depth corner reprojection candidate; all alternatives retained in observations.json; no independent target orientation fixture','physical_axial_station_m':None}
    roll={}
    for key,forward,reverse,zero in [('B19055',range(48,52),range(52,56),'raw/047-B-return-reference-g190-55-0.jpg'),('D20065',range(79,83),range(83,87),'raw/078-D-offdiagonal-validation-g200-65-0.jpg')]:
        R0=record(by[zero])['T'][:3,:3];rows=[]
        for direction,numbers in [('forward',forward),('reverse',reverse)]:
            for i in numbers:
                p=next(p for p in by if p.startswith(f'raw/{i:03d}-') and not p.endswith('-settled.jpg'));o=by[p]
                if o['board']:
                    T=record(o)['T'];rv=Rotation.from_matrix(T[:3,:3]@R0.T).as_rotvec();rows.append({'file':p,'nominal_M5':o['nominal_tracked'][2],'direction':direction,'observed_rotation_deg':float(np.degrees(np.linalg.norm(rv)))})
        roll[key]={'trace':rows,'direction_hysteresis_deg':{str(k):next(q['observed_rotation_deg'] for q in rows if q['nominal_M5']==k and q['direction']=='reverse')-next(q['observed_rotation_deg'] for q in rows if q['nominal_M5']==k and q['direction']=='forward') for k in [10,20,30]}}
    s=fit['sweeps']['D_M3_85']['camera_relative_axis'];ns=np.array(s['direction_F']);cs=np.array(s['closest_point_to_front_optical_origin_m']);links={}
    for key in ['D_M4_190','D_M4_210']:
        E=fit['sweeps'][key]['camera_relative_axis'];v=np.array(E['closest_point_to_front_optical_origin_m'])-cs;links[key]={'parallel_axis_direction_difference_deg':float(np.degrees(np.arccos(np.clip(abs(ns@E['direction_F']),-1,1)))),'effective_pitch_axis_perpendicular_distance_m':float(np.linalg.norm(v-ns*(v@ns))),'convention':'conditional approximately-parallel pitch-axis separation; not independently surveyed anatomy'}
    stationary={}
    for key,f0,fafter in [('B','raw/B-front-reference-before2.jpg','raw/B-reference-Fafter.jpg'),('D','raw/D-front240-95-before2.jpg','raw/D-reference240-95-Fafter.jpg')]:
        o0=by[f0];o1=by[fafter];T0=np.array(o0['board']['T_cam_board']);T1=np.array(o1['board']['T_cam_board']);C0=np.linalg.inv(T0)[:3,3];C1=np.linalg.inv(T1)[:3,3];m0=np.array(o0['marker']['T_cam_marker']);m1=np.array(o1['marker']['T_cam_marker']);shift=float(np.linalg.norm(C1-C0));ang=angle(T0,T1)
        stationary[key]={'front_before':f0,'front_after':fafter,'board_camera_position_change_m':shift,'board_orientation_change_deg':ang,'marker_position_change_m':float(np.linalg.norm(m0[:3,3]-m1[:3,3])),'marker_orientation_change_deg':angle(m0,m1),'board_registration_stationarity_pass':shift<=.003 and ang<=3,'limits':'image-derived apparent body drift includes planar PnP noise; not synchronized exposure proof'}
    for v in stationary.values():
        v['marker_stationarity_pass']=v['marker_position_change_m']<=.003 and v['marker_orientation_change_deg']<=3
        v['joint_pair_accepted']=v['board_registration_stationarity_pass'] and v['marker_stationarity_pass']
    cross['eligible_for_paired_calibration']=stationary['D']['joint_pair_accepted']
    cross['rejection_reason']=None if cross['eligible_for_paired_calibration'] else 'front-before/after marker orientation variation exceeds frozen 3deg stationarity threshold; diagnostic only'
    multi=[];sensitivity=[]
    if not a.quick:
        records=[record(by[p]) for p in groups['D_M3_85']]
        for shift in [[.03,.03],[-.03,-.03],[.04,-.04]]:
            f=fit_pixel_axis(records,Kg,dg,initial_shift=shift);multi.append({'initial_axis_perpendicular_shift_m':shift,'axis_F':in_front(f,TF),'point_rms_px':f['point_rms_px'],'success':f['success'],'rank':f['jacobian_rank'],'parameter_count':f['parameter_count']})
        front_records=[by[p] for p in fit['front_registration']['D']['frames']]
        scenarios=[('G_f_+1pct','gripper',{'fscale':1.01}),('G_f_-1pct','gripper',{'fscale':.99}),('G_cx_+10px','gripper',{'cx':10}),('G_cy_+10px','gripper',{'cy':10}),('G_k1_+0.01','gripper',{'k1':.01}),('F_f_+1pct','front',{'fscale':1.01}),('F_cx_+20px','front',{'cx':20}),('F_cy_+20px','front',{'cy':20}),('F_k1_+0.012','front',{'k1':.012})]
        for label,camera,perturb in scenarios:
            K,d=intrinsics(camera,perturb);T=TF;axis=fit['sweeps']['D_M3_85']['pixel_fit']
            if camera=='front':T,_=front_registration(front_records,K,d)
            else:
                rr=[]
                for old in records:
                    X=old['obj'];px=old['px'];oldcam=np.linalg.inv(old['T']);r0=Rotation.from_matrix(oldcam[:3,:3]).as_rotvec();t0=oldcam[:3,3].copy();ok,r,t=cv2.solvePnP(X,px,K,d,r0,t0,useExtrinsicGuess=True,flags=cv2.SOLVEPNP_ITERATIVE);cam=np.eye(4);cam[:3,:3]=Rotation.from_rotvec(r.ravel()).as_matrix();cam[:3,3]=t.ravel();rr.append({'T':np.linalg.inv(cam),'obj':X,'px':px})
                axis=fit_pixel_axis(rr,K,d)
            result=in_front(axis,T);sensitivity.append({'scenario':label,'assumed_not_measured_bound':perturb,'axis_F':result,'delta_setback_m':result['setback_from_optical_origin_m']-s['setback_from_optical_origin_m'],'delta_height_above_camera_m':result['axis_height_above_camera_m']-s['axis_height_above_camera_m']})
        for scale in [.99,1.01]:sensitivity.append({'scenario':'uniform_print_scale_'+str(scale),'assumed_not_measured_bound':{'uniform_print_scale':scale},'delta_setback_m':s['setback_from_optical_origin_m']*(scale-1),'delta_height_above_camera_m':s['axis_height_above_camera_m']*(scale-1)})
    elif (D/'validation.json').exists():
        old=json.loads((D/'validation.json').read_text());multi=old.get('multistart',[]);sensitivity=old.get('intrinsics_print_scale_sensitivity',[])
    bt=(ROOT/'ros2/src/z70_description/config/mount_calibration.yaml').read_text();baseline=json.loads(bt[bt.index('{'):]);vec=np.array(baseline['effective_camera_from_elbow_m']);gauges=[{'physical_L2_hypothesis_m':L,'camera_nominal_wrist_x_m':float(vec[0]-L),'sum_elbow_to_camera_x_m':float(L+vec[0]-L),'same_observations_by_construction':True,'is_measurement':False} for L in [.17,.205,.24]]
    output={'cross_view_rigid_mount_holdout':cross,'front_ID68_M5':front_m5,'M5_board_traces':roll,'position_only_circle_crosschecks':circles,'effective_upper_link_checks':links,'stationarity_brackets':stationary,'multistart':multi,'intrinsics_print_scale_sensitivity':sensitivity,'L2_mount_equivalence_examples':gauges,'absolute_5mm_1deg_verdict':{'verdict':'INCONCLUSIVE','certified_95percent_bound_m':None,'reason':'No independent physical survey, target-scale tolerance, full-image lens validation or remount session; A/B/D reconstruction disagreement exceeds 5mm/1deg goals'},'whole_redeployment_5mm_1deg_goal':{'verdict':'FAIL','reason':'A/B versus centred D lines differ 28-38mm and >2deg; cannot identify motion versus lens/registration bias from present metrology'}}
    (D/'validation.json').write_text(json.dumps(output,indent=2));print(json.dumps({'cross_view':cross,'circle_crosschecks':{k:{'line_separation_m':v['perpendicular_distance_from_pixel_screw_line_m'],'axis_diff_deg':v['axis_direction_difference_deg']} for k,v in circles.items()},'effective_upper_link':links,'M5_front_angles':front_m5['observed_rotation_deg_from_zero'],'M5_hysteresis':{k:v['direction_hysteresis_deg'] for k,v in roll.items()},'sensitivity':sensitivity,'multistart':multi},indent=1))
if __name__=='__main__':main()
