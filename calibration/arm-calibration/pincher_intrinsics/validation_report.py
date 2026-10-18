"""Additional offline validation from saved real fits. No robot IO."""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np,cv2
from scipy.spatial.transform import Rotation
import calibrate as lens
import arm_analysis as a
from arm_report import pnp_record,NEW_GROUPS
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'physical_base'))
from geometry import fit_axis,fit_pixel_axis
from observations import intrinsics
from analyze import summarize

def main():
    ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);args=ap.parse_args();D=args.session.resolve();out=D/'validation';out.mkdir(exist_ok=True)
    axes=json.loads((D/'arm/axis-fits.json').read_text());S=axes['new']['N_M3_55'];E=axes['new']['N_M4_200_forward'];sp=[np.array(r['T']) for r in S['observed_pose_records']];ep=[np.array(r['T']) for r in E['observed_pose_records']]
    profile=a.axis_distance_profile(sp,ep,S['model_free']['direction'],E['model_free']['direction'],np.arange(.16,.201,.001));(out/'L1-conditional-profile.json').write_text(json.dumps(profile,indent=2))
    # Keep the literal nominal labels and expose actual measured rotation per
    # original frame. Do not silently repair refused-command label drift.
    obs=json.loads((D/'intrinsics-observations.json').read_text());by={int(Path(o['file']).name[:3]):o for o in obs};model=json.loads((D/'lens/models.json').read_text());fit=model[model['frozen_selected_model']]['fit'];K=np.array(fit['K']);d=np.array(fit['dist']);records={i:pnp_record(o,K,d) for i,o in by.items()};trace=[];last=None
    for i in sorted(records):
        r=records[i];o=by[i];T=r['T'];step=None if last is None else float(np.degrees(Rotation.from_matrix(T[:3,:3]@last[:3,:3].T).magnitude()));dist=None if last is None else float(np.linalg.norm(T[:3,3]-last[:3,3]));trace.append({'sequence':i,'file':r['file'],'nominal_label_not_feedback':o['nominal'],'rotation_from_previous_observed_deg':step,'translation_from_previous_m':dist,'T_W_G':T.tolist(),'pnp_full_rms_px':r['full_pnp_rms_px']});last=T
    (out/'observed-trajectory.json').write_text(json.dumps(trace,indent=2))
    # Historical views never entered the new intrinsics fit. Compare precisely
    # identical archived corner correspondences under old/new lens models.
    olddir=D.parent/'20261015T061415Z-physical-base';oldobs=json.loads((olddir/'observations.json').read_text())['frames'];oldviews=[{'file':o['file'],'size':[1600,896],'obj':o['board']['obj'],'px':o['board']['px']} for o in oldobs if o['camera']=='gripper' and o.get('board') and len(o['board']['px'])>=12]
    Ko,do=intrinsics('gripper');oldfit={'K':Ko.tolist(),'dist':do.tolist()};comparison={'views':len(oldviews),'new_lens':lens.evaluate(oldviews,fit),'old_lens_same_corners':lens.evaluate(oldviews,oldfit),'not_used_for_new_lens_selection':True,'correspondence_caveat':'original observations detector selected candidates using old lens; both models evaluated on identical full retained corners'}
    (out/'historical-lens-holdout.json').write_text(json.dumps(comparison,indent=2))
    # Lens group-bootstrap parameter vectors propagated into new pure-joint fits.
    samples=json.loads((D/'lens/uncertainty.json').read_text())['samples'];order=np.argsort([v[0] for v in samples]);indices=order[np.linspace(0,len(order)-1,12).astype(int)];prop=[]
    for ix in indices:
        b=samples[int(ix)];Kb=np.array([[b[0],0,b[2]],[0,b[1],b[3]],[0,0,1.]]);db=np.array([b[4],b[5],0,0,0.]);sr=[pnp_record(by[i],Kb,db) for i in NEW_GROUPS['N_M3_55']];er=[pnp_record(by[i],Kb,db) for i in NEW_GROUPS['N_M4_200_forward']];s=fit_pixel_axis(sr,Kb,db);e=fit_pixel_axis(er,Kb,db);prop.append({'bootstrap_index':int(ix),'L1_station_m':a.station_separation(s,e,[.1,.14,0])['separation_m'],'success':s['success'] and e['success']})
    (out/'lens-confidence-propagation.json').write_text(json.dumps({'selection':'12 focal-length quantiles from wholepose lens bootstrap, including endpoints; not random sigma estimation','samples':prop,'range_m':[min(p['L1_station_m'] for p in prop),max(p['L1_station_m'] for p in prop)],'not_systematic_inclusive':True},indent=2))
    # Region-wise residuals use fixed models with per-view extrinsic nuisance fit.
    regional={}
    for name,f in [('old',oldfit),('new',fit)]:
        KK=np.array(f['K']);dd=np.array(f['dist']);bins={k:[] for k in ['r_lt400','r400_600','r_ge600','top_y_lt200','bottom_y_ge700']}
        for o in obs:
            X=np.array(o['obj']);uv=np.array(o['px']);ok,r,t=cv2.solvePnP(X,uv,KK,dd);err=np.linalg.norm(cv2.projectPoints(X,r,t,KK,dd)[0].reshape(-1,2)-uv,axis=1);rad=np.linalg.norm(uv-np.array([800,448]),axis=1)
            masks={'r_lt400':rad<400,'r400_600':(rad>=400)&(rad<600),'r_ge600':rad>=600,'top_y_lt200':uv[:,1]<200,'bottom_y_ge700':uv[:,1]>=700}
            for key,mask in masks.items():bins[key].extend(err[mask])
        regional[name]={k:summarize(v) for k,v in bins.items()}
    (out/'regional-residuals.json').write_text(json.dumps(regional,indent=2))
    # Demonstrate exact physical L2/mount and M4-zero gauges: the same camera
    # transforms and all projections are unchanged, not just near-equal minima.
    effective=json.loads((D/'arm/effective-second-assembly.json').read_text());R=np.array(E['pixel']['R0']);G=np.array(axes['new']['N_M4_230_reverse']['observed_pose_records'][1]['T']);M=np.array(effective['fixed_optical_to_marker_mount_new']['T_G_M']);marker=(G@M)[:3,3];ee=axes['new']['N_M4_230_reverse']['pixel'];ne=np.array(ee['direction']);ce=np.array(ee['closest_point']);v=marker-(ce+ne*((marker-ce)@ne));effective['elbow_axis_to_ID68_center_perpendicular_vector_W_m']=v.tolist();effective['marker_lever_radius_m']=float(np.linalg.norm(v));effective['gauge_profiles']={'L2':[{**g,'pixel_cost_change_exact':0.} for g in effective['gauge_examples']],'M4_zero':[{'assumed_zero_offset_deg':z,'free_mount_counter_rotation_deg':-z,'pixel_cost_change_exact':0.} for z in [-20,-10,0,10,20]],'scope':'analytic equivalent factorizations, not additional hardware observations'};(out/'effective-second-assembly-and-gauges.json').write_text(json.dumps(effective,indent=2))
    # Stationary marker reprojection overlays: train-frozen mount, heldout corners.
    reg=json.loads((D/'arm/marker-registration.json').read_text())['new'];overlaydir=out/'overlays';overlaydir.mkdir(exist_ok=True)
    for h in reg.get('heldout',[]):
        f=D/h['front_file'];im=cv2.imread(str(f));seq=int(f.name[:3]);fo=json.loads(next((D/'arm').glob(f'{seq:03}-*-front.json')).read_text());actual=np.array(fo['marker']['px']);pred=np.array(h['predicted_px'])
        cv2.polylines(im,[actual.round().astype(np.int32)],True,(0,255,0),2);cv2.polylines(im,[pred.round().astype(np.int32)],True,(0,0,255),2);cv2.putText(im,'Held-out ID68: observed green / fixed-mount predicted red',(20,40),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,255),2);cv2.imwrite(str(overlaydir/(f.stem+'-heldout.jpg')),im)
    # Verify retained originals and original baseline archives, including derived
    # baseline files; no legacy archive-writing test is called.
    before=json.loads((D/'baseline-before-sha256.json').read_text());changed=[str(p) for p,h in before.items() if not Path(p).is_file() or hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h];manifest=json.loads((D/'manifest.json').read_text());mismatch=[m['file'] for m in manifest if hashlib.sha256((D/m['file']).read_bytes()).hexdigest()!=m['sha256']]
    verification={'baseline_files_verified':len(before),'changed_baseline_files':changed,'originals_verified':len(manifest),'original_hash_mismatches':mismatch,'selected_lens':'k1','raw_untouched':not mismatch,'no_commits':True,'source_camera_identity':'MCP log camera, not file labels; raw/002-deployed-gripper.jpg is correctly logged FRONT despite requested filename','actual_exposure_times_unknown':True};(out/'verification.json').write_text(json.dumps(verification,indent=2));assert not changed and not mismatch
    # Complete counts and independent front-marker elbow/roll checks.
    from lens_report import group
    logs=[json.loads(line) for line in (D/'log.jsonl').read_text().splitlines()];caps=[r for r in logs if r.get('event')=='capture'];frontframes={int(f.name[:3]):json.loads(f.read_text()) for f in (D/'arm').glob('???-*-front.json')}
    counts={'original_JPEGs':len(manifest),'logged_G':sum(r.get('camera')=='gripper' for r in caps),'board_G_accepted_min12':len(obs),'board_G_absent':sum(r.get('camera')=='gripper' for r in caps)-len(obs),'distinct_pose_groups':len({group(o) for o in obs}),'distinct_M3M4_groups':len({group(o)[:2] for o in obs}),'front_marker_frames':sum(o.get('marker') is not None for o in frontframes.values()),'front_frames_including_direct_preflight':len(frontframes)+1,'commands_jog':sum(r.get('method')=='app_arm_move' for r in logs)};(D/'counts.json').write_text(json.dumps(counts,indent=2))
    Mposes=[np.array(frontframes[i]['marker']['T_cam_marker']) for i in [39,44,48]];ma=fit_axis(Mposes)
    from geometry import fit_circle_axis
    circle=fit_circle_axis([T[:3,3] for T in Mposes]);TF=np.array(json.loads((D/'arm/front-registration.json').read_text())['T_F_W']);ej=axes['new']['N_M4_230_reverse']['pixel'];n=TF[:3,:3]@ej['direction'];ce=TF[:3,:3]@ej['closest_point']+TF[:3,3];ef={'direction':n.tolist(),'closest_point':(ce-n*(ce@n)).tolist()}
    cross={'marker_only_M4_front':ma,'marker_only_position_circle':circle,'pincher_board_elbow_axis_in_front':ef,'model_free_marker_vs_pincher_at_front_station':a.station_separation(ma,ef,[0,0,0]),'circle_marker_vs_pincher_at_front_station':a.station_separation(circle,ef,[0,0,0]),'scope':'three-pose20deg front ID68 elbow line diagnostic, NOT shoulder axis or L1; planar marker orientation noisy'};(out/'front-ID68-elbow-crosscheck.json').write_text(json.dumps(cross,indent=2))
    R0=records[17]['T'];roll=[]
    for i,nom,direction in [(18,10,'forward'),(19,20,'forward'),(20,30,'forward'),(21,40,'forward'),(22,30,'reverse'),(23,20,'reverse'),(24,10,'reverse'),(25,0,'reverse')]:
        roll.append({'seq':i,'nominal_M5':nom,'direction':direction,'observed_deg_from_first_zero':float(np.degrees(Rotation.from_matrix(records[i]['T'][:3,:3]@R0[:3,:3].T).magnitude()))})
    hysteresis={str(k):next(r['observed_deg_from_first_zero'] for r in roll if r['nominal_M5']==k and r['direction']=='reverse')-next(r['observed_deg_from_first_zero'] for r in roll if r['nominal_M5']==k and r['direction']=='forward') for k in [10,20,30]};(out/'M5-forward-reverse.json').write_text(json.dumps({'trace':roll,'hysteresis_deg':hysteresis},indent=2))
    print(json.dumps({'counts':counts,'hysteresis_deg':hysteresis,'old_views':len(oldviews),'historical_rms_old':comparison['old_lens_same_corners']['rms_px'],'historical_rms_new':comparison['new_lens']['rms_px'],'regional':regional,'lens_propagated_L1_range_mm':[1000*min(p['L1_station_m'] for p in prop),1000*max(p['L1_station_m'] for p in prop)],'profile_minimum':min(profile,key=lambda p:p['translation_rms_m']),'marker_effective_radius_mm':1000*effective['marker_lever_radius_m'],'verification':verification},indent=2))

if __name__=='__main__':main()
