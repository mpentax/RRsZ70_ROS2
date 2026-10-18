"""Reproducible offline report; all fitting primitives tested in test_calibrate."""
import argparse,json,hashlib
from pathlib import Path
import cv2,numpy as np
import calibrate as c

HOLDOUTS={(180,45,0),(200,65,20),(200,65,40),(240,45,0),(240,45,40),(230,55,0),(210,55,0)}

def group(o):
    q=tuple(o['nominal'])
    # Equivalence established by observed near-zero rotations at refused jogs,
    # not a claim that actual joint angles equal these corrected labels.
    if q in [(230,85,0),(240,85,0)]:return (220,85,0)
    if q==(220,55,0):return (200,55,0)
    return q

def main():
    ap=argparse.ArgumentParser();ap.add_argument('session',type=Path);ap.add_argument('--bootstrap',type=int,default=200);args=ap.parse_args();D=args.session.resolve();out=D/'lens';out.mkdir(exist_ok=True)
    obs=json.loads((D/'intrinsics-observations.json').read_text());heldgroups={group({'nominal':q}) for q in HOLDOUTS};train=[o for o in obs if group(o) not in heldgroups];held=[o for o in obs if group(o) in heldgroups]
    assert train and held and not ({group(o) for o in train}&{group(o) for o in held})
    (out/'splits.json').write_text(json.dumps({'train':[o['file'] for o in train],'heldout':[o['file'] for o in held],'heldout_tuples':sorted(HOLDOUTS),'refused_pose_equivalences':'230/240-85 nominal grouped with220-85; 220-55 grouped with200-55 based near-identical observed board poses; labels not measured angles','frozen_before_final_holdout_evaluation':True},indent=2))
    models={};groups=sorted({group(o) for o in train});fold={q:i%5 for i,q in enumerate(groups)}
    for model in ['none','k1','k1k2']:
        cv=[]
        for k in range(5):
            tr=[o for o in train if fold[group(o)]!=k];te=[o for o in train if fold[group(o)]==k]
            fit=c.fit_model(tr,model);cv.append(c.evaluate(te,fit))
        fit=c.fit_model(train,model);models[model]={'fit':fit,'training_wholepose_5fold_cv':cv,'cv_mean_rms_px':float(np.mean([x['rms_px'] for x in cv])),'conditioning':c.conditioning(train,fit)}
    best=min(m['cv_mean_rms_px'] for m in models.values());selected=next(k for k,v in models.items() if v['cv_mean_rms_px']<=best*1.1);fit=models[selected]['fit']
    # Final holdout is opened only after model selection above.
    for model,m in models.items():m['final_holdout_diagnostic']=c.evaluate(held,m['fit'])
    models['frozen_selected_model']=selected
    oldpath=D.parent/'20261014T090217Z/arm_calibration.json';old=json.loads(oldpath.read_text())['gripper_intrinsics'];baseline={'model':'frozen_historical','K':old['K'],'dist':old['dist']}
    evaluation={'new':c.evaluate(held,fit),'old_on_identical_corners':c.evaluate(held,baseline),'heldout_metric_checks_new':[dict(file=o['file'],**c.metric_check(o,fit)) for o in held],'heldout_metric_checks_old':[dict(file=o['file'],**c.metric_check(o,baseline)) for o in held]}
    rng=np.random.default_rng(812);boot=[];fails=[]
    for i in range(args.bootstrap):
        sampled=[groups[j] for j in rng.integers(0,len(groups),len(groups))];draw=[o for q in sampled for o in train if group(o)==q]
        try:
            f=c.fit_model(draw,selected)
            if not f['success']:raise ValueError('invalid calibration')
            K=np.array(f['K']);boot.append([K[0,0],K[1,1],K[0,2],K[1,2],*f['dist'][:2]])
        except (ValueError,cv2.error) as e:fails.append({'draw':i,'error':type(e).__name__})
    arr=np.array(boot);unc={'unit':'whole observed pose group, repeats stay together','attempted':args.bootstrap,'successful':len(boot),'failed':fails,'parameter_order':['fx','fy','cx','cy','k1','k2'],'samples':boot,'conditional_sigma':np.std(arr,axis=0,ddof=1).tolist() if len(arr)>1 else None,'conditional_95percent':np.percentile(arr,[2.5,97.5],axis=0).tolist() if len(arr)>1 else None,'excludes':['unknown print tolerance','board nonplanarity','pixel-detection correlated bias','systematic lens model misspecification']}
    px=np.concatenate([o['px'] for o in obs]);hist=np.histogram2d(px[:,0],px[:,1],bins=[np.linspace(0,1600,11),np.linspace(0,896,7)])[0]
    cov={'corners':len(px),'bounds_xy':[px.min(axis=0).tolist(),px.max(axis=0).tolist()],'grid_10x6_counts':hist.T.tolist(),'occupied_cells':int(np.sum(hist>0)),'total_cells':60,'training_normal_depth':models[selected]['conditioning'],'all_views_normal_depth':c.conditioning(obs,fit)}
    canvas=np.zeros((896,1600,3),np.uint8)
    for x,y in px:cv2.circle(canvas,(round(x),round(y)),2,(0,180,255),-1)
    cv2.imwrite(str(out/'coverage.png'),canvas)
    for o in [max(obs,key=lambda x:len(x['px'])),held[0],obs[-1]]:
        im=cv2.imread(o['file']);K=np.array(fit['K']);d=np.array(fit['dist']);X=np.array(o['obj']);uv=np.array(o['px']);ok,r,t=cv2.solvePnP(X,uv,K,d);pred=cv2.projectPoints(X,r,t,K,d)[0].reshape(-1,2)
        for p,q in zip(uv,pred):cv2.circle(im,tuple(np.round(p).astype(int)),3,(0,255,0),1);cv2.line(im,tuple(np.round(p).astype(int)),tuple(np.round(q).astype(int)),(0,0,255),1)
        cv2.imwrite(str(out/(Path(o['file']).stem+'-overlay.jpg')),im)
    for name,data in [('models.json',models),('holdout.json',evaluation),('uncertainty.json',unc),('coverage.json',cov),('proposed-camera-info.json',{'camera':'pincher delivered1600x896','image_width':1600,'image_height':896,'distortion_model':'plumb_bob','K':fit['K'],'D':fit['dist'],'adoption':'proposed offline only; no production edit','selected_model':selected,'pose_count_train':len(train),'pose_count_heldout':len(held)})]:(out/name).write_text(json.dumps(data,indent=2))
    print(json.dumps({'selected':selected,'train_frames':len(train),'heldout_frames':len(held),'train_pose_groups':len(groups),'heldout_pose_groups':len({group(o) for o in held}),'cv':{k:v['cv_mean_rms_px'] for k,v in models.items() if isinstance(v,dict)},'heldout':{k:v for k,v in evaluation.items() if k in ['new','old_on_identical_corners']},'coverage_bounds':cov['bounds_xy'],'normal_span_deg':cov['all_views_normal_depth']['normal_span_deg'],'bootstrap_success':len(boot)},indent=2))

if __name__=='__main__':main()
