"""Offline arm/marker reconstruction from actual archived pixels. No robot IO.

Uses tested physical_base solvers; does NOT run their archive-writing CLIs.
"""
import argparse,json,sys,copy,hashlib
from pathlib import Path
import numpy as np,cv2
from scipy.spatial.transform import Rotation
import calibrate as lens
from arm_analysis import station_separation,fit_mount,project_marker,as_transform,MARKER
BASE=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(BASE/'physical_base'))
from observations import detect,intrinsics,transform
from geometry import fit_axis,fit_circle_axis,fit_pixel_axis,predict_pose
from analyze import front_registration,in_front,summarize

NEW_GROUPS={'N_M3_55':[85,87,88,89,90],'N_M3_45_reverse':[50,54,58],'N_M3_45_forward':[6,11,12],'N_M3_45_roll40':[63,64,65],'N_M4_200_forward':[12,13,17],'N_M4_220_reverse':[35,36,37],'N_M4_230_reverse':[42,46,50],'N_M5_20065_forward':[17,18,19,20,21],'N_M5_20065_reverse':[21,22,23,24,25],'N_M5_21045':[58,60,61,62,63],'N_M5_23055_reverse':[69,73,77,81,85]}
PAIR_SEQS=[(39,41,42,43),(44,45,46,47),(48,49,50,51),(52,53,54,55),(56,57,58,59),(67,68,69,70),(71,72,73,74),(75,76,77,78),(79,80,81,82),(83,84,85,86)]
MOUNT_HOLDOUT={(250,45,0),(250,55,20),(250,55,40)}

def pnp_record(o,K,d):
    X=np.asarray(o['obj'],float);uv=np.asarray(o['px'],float)
    ok,r,t=cv2.solvePnP(X,uv,K,d,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:raise ValueError('PnP failed')
    err=np.linalg.norm(cv2.projectPoints(X,r,t,K,d)[0].reshape(-1,2)-uv,axis=1);keep=err<4
    if keep.sum()>=12:r,t=cv2.solvePnPRefineLM(X[keep],uv[keep],K,d,r,t)
    T=transform(r,t);err=np.linalg.norm(cv2.projectPoints(X,r,t,K,d)[0].reshape(-1,2)-uv,axis=1)
    _,rs,ts,_=cv2.solvePnPGeneric(X,uv,K,d,flags=cv2.SOLVEPNP_IPPE)
    branches=[]
    for ri,ti in zip(rs,ts):
        Ti=transform(ri,ti);e=np.linalg.norm(cv2.projectPoints(X,ri,ti,K,d)[0].reshape(-1,2)-uv,axis=1)
        branches.append({'T_cam_board':Ti.tolist(),'positive_depth':bool(np.all((X@Ti[:3,:3].T+Ti[:3,3])[:,2]>0)),'full_rms_px':float(np.sqrt(np.mean(e**2)))})
    return {'T':np.linalg.inv(T),'obj':X,'px':uv,'file':o['file'],'full_pnp_rms_px':float(np.sqrt(np.mean(err**2))),'trimmed_initialization_corners':int(keep.sum()),'positive_depth':bool(np.all((X@T[:3,:3].T+T[:3,3])[:,2]>0)),'branches':branches}

def plain(r):return {k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in r.items()}
def angle(A,B):return float(np.degrees(Rotation.from_matrix(A[:3,:3]@B[:3,:3].T).magnitude()))
def sep(a,b):
    return station_separation(a,b,[.1,.14,0])

def main():
    ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);ap.add_argument('--bootstrap',type=int,default=200);ap.add_argument('--skip-old',action='store_true');args=ap.parse_args();D=args.session.resolve();out=D/'arm';out.mkdir(exist_ok=True)
    models=json.loads((D/'lens/models.json').read_text());newfit=models[models['frozen_selected_model']]['fit'];Knew=np.array(newfit['K']);dnew=np.array(newfit['dist']);Kold,dold=intrinsics('gripper');Kf,df=intrinsics('front')
    caps=[json.loads(l) for l in (D/'log.jsonl').read_text().splitlines()];caps={r['file']:r for r in caps if r.get('event')=='capture'}
    obs=json.loads((D/'intrinsics-observations.json').read_text());byseq={int(Path(o['file']).name[:3]):o for o in obs}
    fronts={};rawobs=[]
    for file,c in caps.items():
        if c.get('camera')!='front':continue
        seq=int(Path(file).name[:3]);p=D/file;cache=out/(p.stem+'-front.json')
        o=detect(p,'front');o['file']=file;o['nominal']=c.get('tracked');cache.write_text(json.dumps(o));fronts[seq]=o;rawobs.append(o)
    TF,fdiag=front_registration([o for seq,o in fronts.items() if 39<=seq<=86]);(out/'front-registration.json').write_text(json.dumps({'T_F_W':TF.tolist(),**fdiag},indent=2))
    groups={k:[byseq[i] for i in v if i in byseq] for k,v in NEW_GROUPS.items()}
    oldsession=BASE/'20261015T061415Z-physical-base';oldobs=json.loads((oldsession/'observations.json').read_text())['frames'];oldby={o['file']:o for o in oldobs};oldgroups=json.loads((oldsession/'groups.json').read_text());oldfront=json.loads((oldsession/'fit.json').read_text())['front_registration']
    if not args.skip_old:
        for name,files in oldgroups.items():
            groups['O_'+name]=[{'file':str(oldsession/f),'obj':oldby[f]['board']['obj'],'px':oldby[f]['board']['px']} for f in files if oldby[f].get('board')]
    results={};recordsets={};holdouts={}
    for scenario,K,d in [('new',Knew,dnew),('old',Kold,dold)]:
        results[scenario]={};recordsets[scenario]={}
        for name,observations in groups.items():
            if len(observations)<3:continue
            rr=[pnp_record(o,K,d) for o in observations];recordsets[scenario][name]=rr
            screw=fit_axis([r['T'] for r in rr]);circle=fit_circle_axis([r['T'][:3,3] for r in rr]);pixel=fit_pixel_axis(rr,K,d)
            trans=TF if name.startswith('N_') else np.array(oldfront[name[2]]['T_F_W'])
            result={'files':[r['file'] for r in rr],'model_free':screw,'position_circle':circle,'pixel':pixel,'front_axis':in_front(pixel,trans),'observed_span_deg':float(np.ptp(np.degrees(pixel['latent_angles_rad']))),'observed_pose_records':[plain(r) for r in rr]}
            result['circle_pixel_at_station']=sep(circle,pixel);results[scenario][name]=result
            if len(rr)>=4:
                # Middle whole pose held out, conditional angle from independent
                # PnP orientation only; no translation/pixels from it enter axis.
                hi=len(rr)//2;training=[r for i,r in enumerate(rr) if i!=hi];f=fit_pixel_axis(training,K,d);target=rr[hi];a=float(np.array(f['direction'])@Rotation.from_matrix(target['T'][:3,:3]@np.array(f['R0']).T).as_rotvec());pred=predict_pose(f,a)
                uv=cv2.projectPoints((target['obj']-pred[:3,3])@pred[:3,:3],np.zeros(3),np.zeros(3),K,d)[0].reshape(-1,2)
                holdouts[scenario+'_'+name]={'heldout_file':target['file'],'training_files':[r['file'] for r in training],'angle_source':'heldout observed camera orientation, conditional geometry only, not independent sensor','center_error_m':float(np.linalg.norm(pred[:3,3]-target['T'][:3,3])),'point_errors_px':summarize(np.linalg.norm(uv-target['px'],axis=1))}
            print(scenario,name,'span',round(result['observed_span_deg'],2),'rms',round(pixel['point_rms_px'],3),flush=True)
    (out/'axis-fits.json').write_text(json.dumps(results,indent=2));(out/'axis-holdouts.json').write_text(json.dumps(holdouts,indent=2))
    links={}
    for scenario,r in results.items():
        links[scenario]={}
        for shoulder in ['N_M3_55','N_M3_45_forward','N_M3_45_reverse','O_D_M3_85','O_D_M3_45']:
            if shoulder not in r:continue
            es=['N_M4_200_forward','N_M4_220_reverse','N_M4_230_reverse'] if shoulder.startswith('N_') else ['O_D_M4_190','O_D_M4_210']
            for elbow in es:
                if elbow not in r:continue
                links[scenario][shoulder+'__'+elbow]={method:sep(r[shoulder][method],r[elbow][method]) for method in ['pixel','model_free','position_circle']}
    (out/'link-geometry.json').write_text(json.dumps(links,indent=2))
    policy=json.loads((D/'stationarity-policy.json').read_text());pairs=[]
    for f0,g1,g2,f1 in PAIR_SEQS:
        a=fronts[f0];b=fronts[f1];p={'seqs':[f0,g1,g2,f1],'nominal':a.get('nominal'),'exposure_simultaneous':False,'accepted':False}
        if not a.get('marker') or not b.get('marker') or g1 not in byseq or g2 not in byseq:
            p['reject']='missing elevated ID68 or board';pairs.append(p);continue
        A=np.array(a['marker']['T_cam_marker']);B=np.array(b['marker']['T_cam_marker']);p['marker_translation_change_m']=float(np.linalg.norm(A[:3,3]-B[:3,3]));p['marker_orientation_change_deg']=angle(A,B)
        BF0=np.array(a['board']['T_cam_board']);BF1=np.array(b['board']['T_cam_board']);p['front_board_translation_change_m']=float(np.linalg.norm(np.linalg.inv(BF0)[:3,3]-np.linalg.inv(BF1)[:3,3]));p['front_board_orientation_change_deg']=angle(BF0,BF1)
        ga=pnp_record(byseq[g1],Knew,dnew);gb=pnp_record(byseq[g2],Knew,dnew);p['gripper_translation_change_m']=float(np.linalg.norm(ga['T'][:3,3]-gb['T'][:3,3]));p['gripper_orientation_change_deg']=angle(ga['T'],gb['T'])
        p['accepted']=bool(p['marker_translation_change_m']<=policy['max_translation_m'] and p['marker_orientation_change_deg']<=policy['max_orientation_deg'] and p['front_board_translation_change_m']<=.003 and p['front_board_orientation_change_deg']<=3 and p['gripper_translation_change_m']<=.003 and p['gripper_orientation_change_deg']<=3)
        p['reject']=None if p['accepted'] else 'translation/orientation stationarity failed; retained diagnostic only';p['split']='holdout' if tuple(p['nominal']) in MOUNT_HOLDOUT else 'train';pairs.append(p)
    (out/'pairing.json').write_text(json.dumps({'policy':policy,'gripper_and_board_limits':{'translation_m':.003,'orientation_deg':3},'heldout_marker_poses':sorted(MOUNT_HOLDOUT),'pairs':pairs},indent=2))
    registration={}
    for scenario,K,d in [('new',Knew,dnew),('old',Kold,dold)]:
        good=[p for p in pairs if p['accepted']];train=[p for p in good if p['split']=='train'];test=[p for p in good if p['split']=='holdout'];poses=[];pixels=[]
        for p in train:
            f0,g1,g2,f1=p['seqs'];G=pnp_record(byseq[g2],K,d)['T'];poses.append(TF@G);pixels.append(np.array(fronts[f0]['marker']['px']))
        if len(train)<3 or not test:
            registration[scenario]={'verdict':'INCONCLUSIVE','accepted_train':len(train),'accepted_holdout':len(test),'reason':'insufficient stationary paired poses'};continue
        first=train[0];initial=np.linalg.inv(poses[0])@np.array(fronts[first['seqs'][0]]['marker']['T_cam_marker']);mount=fit_mount(poses,pixels,Kf,df,initial);M=np.array(mount['T_G_M']);predictions=[]
        for p in test:
            f0,g1,g2,f1=p['seqs'];G=pnp_record(byseq[g2],K,d)['T'];P=TF@G@M;uv=project_marker(P,Kf,df);actual=np.array(fronts[f0]['marker']['T_cam_marker']);e=np.linalg.norm(uv-fronts[f0]['marker']['px'],axis=1)
            predictions.append({'nominal':p['nominal'],'front_file':fronts[f0]['file'],'G_file':byseq[g2]['file'],'pixel_errors':summarize(e),'marker_translation_disagreement_m':float(np.linalg.norm(P[:3,3]-actual[:3,3])),'marker_orientation_disagreement_deg':angle(P,actual),'predicted_px':uv.tolist(),'independent_marker_pixels_heldout':True})
        starts=[]
        for delta in [[.02,-.02,.01],[-.02,.02,-.01]]:
            init=initial.copy();init[:3,3]+=delta;starts.append(fit_mount(poses,pixels,Kf,df,init))
        registration[scenario]={'fit':mount,'train_pairs':[p['seqs'] for p in train],'heldout':predictions,'multistart':starts,'scope':'fixed optical-camera to pincher-ID68 registration; NOT wrist registration, front intrinsic calibration, or independent L1 validation'}
    (out/'marker-registration.json').write_text(json.dumps(registration,indent=2))
    # Whole-pose model-free uncertainty; distinguish from pixel-bundle bootstrap.
    rng=np.random.default_rng(912);boots=[];failures=0;attempts=0;ss=recordsets['new']['N_M3_55'];ee=recordsets['new']['N_M4_200_forward']
    while len(boots)<args.bootstrap and attempts<args.bootstrap*15:
        attempts+=1
        try:
            si=rng.integers(0,len(ss),len(ss));ei=rng.integers(0,len(ee),len(ee))
            if len(set(si))<3 or len(set(ei))<3:raise ValueError('insufficient unique excitation')
            S=fit_axis([ss[i]['T'] for i in si]);E=fit_axis([ee[i]['T'] for i in ei]);v=sep(S,E)['separation_m']
            if not np.isfinite(v) or v>1:raise ValueError('invalid line')
            boots.append(v)
        except (ValueError,np.linalg.LinAlgError):failures+=1
    arr=np.array(boots);unc={'method':'whole-pose PnP SE3 axis resampling, NOT pixel-bundle/lens joint covariance','attempted':attempts,'successful':len(boots),'failed':failures,'L1_station_sigma_m':float(np.std(arr,ddof=1)) if len(arr)>1 else None,'L1_station_percentile95_m':np.percentile(arr,[2.5,97.5]).tolist() if len(arr)>1 else None,'samples_m':boots,'limitations':['three-pose elbow bootstrap has only one distinct fully-excited support; cannot estimate full elbow systematic/random error','print scale, floor flatness, lens systematic, hysteresis and deployment variation excluded']}
    # Pixel-axis multistart, intrinsic confidence/model and physical-print scenarios.
    multi=[]
    for shift in [[.03,.03],[-.03,-.03],[.04,-.04]]:
        f=fit_pixel_axis(ss,Knew,dnew,initial_shift=shift);multi.append({'initial_shift_m':shift,'fit':f,'L1_station_m':sep(f,results['new']['N_M4_200_forward']['pixel'])['separation_m']})
    sensitivities=[]
    scenarios=[('new_k1',Knew,dnew),('alternative_k1k2',np.array(models['k1k2']['fit']['K']),np.array(models['k1k2']['fit']['dist'])),('f_plus1pct_assumed',Knew.copy(),dnew.copy()),('cy_plus10px_assumed',Knew.copy(),dnew.copy())]
    scenarios[2][1][0,0]*=1.01;scenarios[2][1][1,1]*=1.01;scenarios[3][1][1,2]+=10
    for label,K,d in scenarios:
        sr=[pnp_record(o,K,d) for o in groups['N_M3_55']];er=[pnp_record(o,K,d) for o in groups['N_M4_200_forward']];S=fit_pixel_axis(sr,K,d);E=fit_pixel_axis(er,K,d);sensitivities.append({'scenario':label,'L1_station_m':sep(S,E)['separation_m'],'is_measured_bound':False,'success':S['success'] and E['success']})
    primary=links['new']['N_M3_55__N_M4_200_forward']['pixel']['separation_m']
    for scale in [.99,1.01]:sensitivities.append({'scenario':'print_scale_'+str(scale)+'_assumed','L1_station_m':primary*scale,'is_measured_bound':False})
    unc['multistart']=multi;unc['sensitivity_scenarios']=sensitivities;(out/'uncertainty.json').write_text(json.dumps(unc,indent=2))
    # Observable effective second-assembly vectors, with explicit axial gauge.
    E=results['new']['N_M4_230_reverse']['pixel'];G=recordsets['new']['N_M4_230_reverse'][1]['T'];ne=np.array(E['direction']);ce=np.array(E['closest_point']);camera=G[:3,3];elbow_station=ce+ne*((camera-ce)@ne);v=camera-elbow_station
    effective={'elbow_axis_to_pincher_optical_center_perpendicular_vector_W_m':v.tolist(),'lever_radius_m':float(np.linalg.norm(v)),'convention':'elbow line point at optical-center axial station; not wrist-centre L2','fixed_optical_to_marker_mount_new':registration.get('new',{}).get('fit'),'physical_L2_m':None,'physical_M4_zero_deg':None,'physical_M5_axial_station_m':None,'exact_contact_pad_vector_m':None,'gauge_examples':[{'assumed_L2_m':L,'compensating_free_mount_along_link_m':float(np.linalg.norm(v)-L),'same_effective_scalar_sum_m':float(np.linalg.norm(v)),'is_measurement':False} for L in [.17,.205,.24]],'gauge_jacobian':{'columns':['L2','mount_along_link'],'matrix':[[1.,1.]],'singular_values':[float(np.sqrt(2)),0.],'rank':1,'null_direction':[float(1/np.sqrt(2)),float(-1/np.sqrt(2))]},'M4_zero_gauge':'Rotating the free mount by inverse zero offset leaves all optical/marker poses identical; physical zero unidentifiable','M5_axial_gauge':'Translation along the recovered roll line commutes with roll; no physical axial station from this trajectory'}
    (out/'effective-second-assembly.json').write_text(json.dumps(effective,indent=2))
    # Front ID68 roll trajectory, independent optical observation but weak planar pose.
    ids=[67,71,75,79,83];visible=[fronts[i] for i in ids if fronts[i].get('marker')]
    if len(visible)>=3:
        Ts=[np.array(o['marker']['T_cam_marker']) for o in visible];frontroll={'files':[o['file'] for o in visible],'model_free':fit_axis(Ts),'position_circle':fit_circle_axis([T[:3,3] for T in Ts]),'orientation_span_deg':max(angle(T,Ts[0]) for T in Ts),'physical_wrist_station_m':None,'warning':'15mm target planar branch/orientation noise retained; front intrinsics frozen and extrapolated'};(out/'front-ID68-roll.json').write_text(json.dumps(frontroll,indent=2))
    print(json.dumps({'primary_L1_station_m':primary,'new_link_checks':links['new'],'pair_accepted':sum(p['accepted'] for p in pairs),'pair_attempted':len(pairs),'marker_registration':registration,'bootstrap':{k:v for k,v in unc.items() if k in ['attempted','successful','failed','L1_station_sigma_m','L1_station_percentile95_m']},'effective_radius_m':effective['lever_radius_m']},indent=2))

if __name__=='__main__':main()
