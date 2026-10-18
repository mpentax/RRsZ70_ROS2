"""Offline reproduction. Writes only explicit guarded --out, never source archives."""
import argparse,json,hashlib,sys,copy,os
from contextlib import contextmanager
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import joint as j
ROOT=j.BASE.parent.parent
OLD=j.BASE/'20261015T061415Z-physical-base'
sys.path.insert(0,str(j.BASE/'pincher_intrinsics'))
from arm_analysis import station_separation
from geometry import fit_axis,fit_pixel_axis
from analyze import in_front

@contextmanager
def output_writer(out,sources):
    out=j.output_guard(out,sources)
    flags=os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW
    identity=lambda st:(st.st_dev,st.st_ino)
    protected={identity(Path(source).stat()) for source in sources}
    # Walk the resolved path from a stable root, never re-resolve ancestors.
    # Check each opened component against its no-follow stat and source roots.
    parent=os.open(out.anchor,flags)
    try:
        for component in out.parts[1:-1]:
            try:os.mkdir(component,dir_fd=parent)
            except FileExistsError:pass
            expected=os.stat(component,dir_fd=parent,follow_symlinks=False)
            child=os.open(component,flags,dir_fd=parent)
            actual=os.fstat(child)
            if identity(actual)!=identity(expected) or identity(actual) in protected:
                os.close(child);raise ValueError('Output parent changed or overlaps preserved source tree')
            os.close(parent);parent=child
        os.mkdir(out.name,dir_fd=parent) # existing output is always refused
        expected=os.stat(out.name,dir_fd=parent,follow_symlinks=False)
        directory=os.open(out.name,flags,dir_fd=parent)
        if identity(os.fstat(directory))!=identity(expected) or identity(os.fstat(directory)) in protected or os.listdir(directory):
            os.close(directory);raise ValueError('Output directory changed or is not empty')
    finally:os.close(parent)
    try:
        def save(name,obj):
            if Path(name).name!=name:raise ValueError('Output filename must be a leaf')
            payload=json.dumps(obj,indent=2,allow_nan=False)
            # Never truncate a destination inode: links are replaced, not followed.
            # Anchor every operation to this directory even if its path is swapped.
            temporary='.joint-output.tmp'
            fd=os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600,dir_fd=directory)
            try:
                with os.fdopen(fd,'w') as stream:stream.write(payload)
                os.replace(temporary,name,src_dir_fd=directory,dst_dir_fd=directory)
            finally:
                try:os.unlink(temporary,dir_fd=directory)
                except FileNotFoundError:pass
        yield save
    finally:os.close(directory)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--bootstrap',type=int,default=50);args=ap.parse_args()
    sources=[d for d in j.BASE.iterdir() if d.is_dir() and d.name not in ['front_pincher_joint','20261015-front-pincher-joint']]+[ROOT/'calibration/front-camera',ROOT/'ros2/src/z70_description/config']
    with output_writer(args.out,sources+[Path(__file__).resolve().parent,j.BASE/'20261015-front-pincher-joint']) as save:
        reproduce(args,sources,save)

def reproduce(args,sources,save):
    snapshot=lambda:{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for d in sources for p in d.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
    before=snapshot();save('archive-before-sha256.json',before);data=j.load_data()
    import scipy,platform
    save('software-and-code.json',dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,opencv=j.cv2.__version__,code_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')},retained_inputs_sha256=before))
    groups=sorted({tuple(p['nominal']) for p in data['train']})
    protocol=dict(version=1,fit_scope='single documented stationary deployment block; fixed F optical mount; marker after M5; 15mm black square; no FK/nominal angle objective',counts=dict(brackets_attempted=len(data['policy']['pairs']),accepted=len(data['pairs']),rejected=sum(not p['accepted'] for p in data['policy']['pairs']),training_brackets=len(data['train']),training_pose_groups=len(groups),heldout_brackets=len(data['test'])),training_groups=[list(g) for g in groups],test_ids=[p['id'] for p in data['test']],training_ids=[p['id'] for p in data['train']],models=j.MODELS,selection='training-only leave-whole-nominal-pose-group-out; score sqrt(mean(marker_RMS^2+floor_RMS^2)/2); simplest within10% best; no floor RMS >110% baseline; full and every CV fit converged, no bound hits, all F-floor/F-marker/G-floor corners positive depth; CV evaluation positive depth',holdout_disclosure='Prior analysis already inspected the retained final pitch/roll poses. These are exploratory held-out pixels under a newly declared protocol, NOT independent prospective validation.',weighting='each front floor/marker/G board image divided by sqrt(number of points); robust soft_l1 f_scale=.5; duplicate exposures remain correlated, not independent poses',pair_policy=data['policy'],coordinate_convention='T_A_B maps B into A; W=printed board frame; F/G=OpenCV optical frames; M=ID68 square frame; metre units')
    save('protocol.json',protocol)
    models={}
    for model in j.MODELS:
        fit=j.fit_bundle(data,data['train'],model);cv=[]
        for group in groups:
            train=[p for p in data['train'] if tuple(p['nominal'])!=group];test=[p for p in data['train'] if tuple(p['nominal'])==group]
            f=j.fit_bundle(data,train,model);e=j.evaluate(data,test,f);cv.append(dict(group=list(group),fit=f,evaluation=e))
        marker=float(np.sqrt(np.mean([v['evaluation']['metrics']['marker']['rms_px']**2 for v in cv])));floor=float(np.sqrt(np.mean([v['evaluation']['metrics']['floor']['rms_px']**2 for v in cv])))
        invalid=[]
        for scope,f in [('full',fit)]+[(f'fold {i} group {v["group"]}',v['fit']) for i,v in enumerate(cv)]:
            if not f['success']:invalid.append(f'{scope}: success=False (nonconverged)')
            if f['bound_hits']:invalid.append(f'{scope}: bound_hits={f["bound_hits"]}')
            if not f['positive_depth']:invalid.append(f'{scope}: positive_depth=False')
        for i,v in enumerate(cv):
            if not v['evaluation']['positive_depth']:invalid.append(f'fold {i} evaluation: positive_depth=False')
        models[model]=dict(fit=fit,cv=cv,cv_marker_rms_px=marker,cv_floor_rms_px=floor,cv_score=float(np.sqrt((marker*marker+floor*floor)/2)),all_converged=fit['success'] and all(v['fit']['success'] for v in cv),invalid_reasons=invalid)
        save('models-progress.json',models);print(model,'CV marker/floor',marker,floor,'K',fit['K'],'condition',fit['conditioning']['condition'],flush=True)
    eligible=[n for n in models if not models[n]['invalid_reasons'] and models[n]['cv_floor_rms_px']<=models['fixed']['cv_floor_rms_px']*1.1]
    if not eligible:raise ValueError('No valid model eligible for selection; see models-progress.json invalid_reasons')
    best=min(models[n]['cv_score'] for n in eligible);selected=next(n for n in j.MODELS if n in eligible and models[n]['cv_score']<=best*1.1);fit=models[selected]['fit']
    save('selection-frozen.json',dict(selected=selected,eligible=eligible,best_score=best,rule=protocol['selection']))
    final=j.evaluate(data,data['test'],fit);fixed=j.evaluate(data,data['test'],models['fixed']['fit'])
    save('models.json',dict(selected=selected,models=models,final=final,fixed_lens_final=fixed,disclosure=protocol['holdout_disclosure']))
    # Diagnostics only, never feed back into model selection.
    full=models['full']['fit'];profiles=[]
    for key,deltas in [('fx',[-100,-50,0,50,100]),('cy',[-80,-40,0,40,80])]:
        for delta in deltas:
            val=(full['K'][0][0]-data['Kf'][0,0] if key=='fx' else full['K'][1][2]-data['Kf'][1,2])+delta
            f=j.fit_bundle(data,data['train'],'full',initial=full,fixed_values={key:val});profiles.append(dict(key=key,delta_from_optimum=delta,fit=f))
    starts=[]
    for sign in [-1,1]:
        init=copy.deepcopy(fit);init['T_G_M']=np.array(init['T_G_M']);init['T_G_M'][:3,3]+=sign*np.array([.02,-.02,.01]);init['T_G_M']=init['T_G_M'].tolist();starts.append(j.fit_bundle(data,data['train'],selected,initial=init))
    save('identifiability.json',dict(profiles=profiles,multistarts=starts,full_model=full['conditioning'],scope='unrobust pixel Jacobian with latent G nuisances eliminated; numeric full rank does not prove calibrated anatomy or systematic accuracy'))
    # Compare full latent-G fit against explicitly frozen G-only conditional fit.
    conditional={}
    for label,model in [('old','fixed'),('candidate',selected)]:
        cf=j.fit_bundle(data,data['train'],model,fixed_gripper=True);conditional[label]=dict(fit=cf,heldout=j.evaluate(data,data['test'],cf))
    save('conditional-versus-joint.json',conditional)
    # Pixel/normal/depth support and local acquisition-information design.
    marker_uv=np.concatenate([np.array(o['marker']['px']) for p in data['train'] for o in p['front']]);floor_uv=np.concatenate([np.array(o['board']['px']) for p in data['train'] for o in p['front']]);normals=[];depths=[];adjustments=[]
    for p,Tg in zip(data['train'],fit['T_W_G']):
        W=j.gripper_pose(data,p);P=np.array(fit['T_F_W'])@W@np.array(fit['T_G_M']);normals.append(P[:3,2]);depths.append(float(P[2,3]));adjustments.append(dict(id=p['id'],translation_mm=float(np.linalg.norm(W[:3,3]-np.array(Tg)[:3,3])*1000),orientation_deg=float(np.degrees(Rotation.from_matrix(W[:3,:3]@np.array(Tg)[:3,:3].T).magnitude()))))
    candidates=[]
    for x in [400,800,1200]:
        for y in [100,300,500]:
            for z in [.12,.18,.24]:
                for tilt in [-20,0,20]:
                    v=j.design_view(data,full,[x,y],z,tilt,0)
                    if v['front_marker_fully_visible'] and v['G_floor_corners']>=20:candidates.append(v)
    H=np.array(full['conditioning']['information']);baseeig=float(np.linalg.eigvalsh(H)[0]);design=[]
    for _ in range(2):
        bestv=max(candidates,key=lambda c:np.linalg.eigvalsh(H+np.array(c['information']))[0]);H+=np.array(bestv['information']);bestv=dict(bestv,cumulative_min_eigenvalue=float(np.linalg.eigvalsh(H)[0]),cumulative_condition=float(np.sqrt(np.linalg.eigvalsh(H)[-1]/np.linalg.eigvalsh(H)[0])));design.append(bestv);candidates=[c for c in candidates if c['center_px']!=bestv['center_px'] or c['depth_m']!=bestv['depth_m']]
    normals=np.array(normals)
    save('coverage-and-acquisition.json',dict(marker_training_bounds_px=[marker_uv.min(0).tolist(),marker_uv.max(0).tolist()],floor_training_bounds_px=[floor_uv.min(0).tolist(),floor_uv.max(0).tolist()],marker_depths_m=depths,marker_normal_span_deg=float(np.degrees(np.arccos(np.clip(normals@normals.T,-1,1))).max()),G_pose_adjustments=adjustments,baseline_min_eigenvalue=baseeig,greedy_design_views=design,remaining_candidate_count=len(candidates),scope='Local linear information design only, not acquired pixels, not reachability/occlusion assurance or precision certification; two views selected greedily from a declared81-view grid after current fitting; duplicates treated ideal independent detector samples; actual correlated noise reduces gains. Needs unchanged stationary block or add/register a new TF nuisance. Do not use these design values as measurements.',grid=dict(x_px=[400,800,1200],y_px=[100,300,500],depth_m=[.12,.18,.24],tilt_x_deg=[-20,0,20],tilt_y_deg=[0])))
    # Rank next measurements only among arm poses actually achieved in retained G views.
    observed_design=[];observed_rejected=[];paired_G=[j.gripper_pose(data,p) for p in data['pairs']]
    for o in json.loads((j.SESSION/'intrinsics-observations.json').read_text()):
        seq=int(Path(o['file']).name[:3])
        if seq in [53,54,57,58]:observed_rejected.append(dict(file=o['file'],reason='paired front ID68 absent; preserve rejection'));continue
        WG=j.camera_pose(o,data['Kg'],data['dg'])
        near=any(np.linalg.norm(WG[:3,3]-W[:3,3])<.002 and np.degrees(Rotation.from_matrix(WG[:3,:3]@W[:3,:3].T).magnitude())<1 for W in paired_G)
        if near:continue
        v=j.design_view(data,full,[0,0],1,0,0,observed_WG=WG);v['file']=o['file']
        if v['front_marker_fully_visible'] and v['G_floor_corners']>=20:observed_design.append(v)
        else:observed_rejected.append(dict(file=o['file'],reason='predicted F marker visibility or G floor coverage inadequate',predicted_center_px=v['center_px']))
    Ho=np.array(full['conditioning']['information']);observed_best=[]
    for _ in range(min(2,len(observed_design))):
        if not observed_design:break
        v=max(observed_design,key=lambda c:np.linalg.eigvalsh(Ho+np.array(c['information']))[0]);Ho+=np.array(v['information']);observed_best.append(dict(v,cumulative_min_eigenvalue=float(np.linalg.eigvalsh(Ho)[0]),cumulative_condition=float(np.sqrt(np.linalg.eigvalsh(Ho)[-1]/np.linalg.eigvalsh(Ho)[0]))));Wv=np.array(v['T_W_G']);observed_design=[c for c in observed_design if np.linalg.norm(np.array(c['T_W_G'])[:3,3]-Wv[:3,3])>.002 or np.degrees(Rotation.from_matrix(np.array(c['T_W_G'])[:3,:3]@Wv[:3,:3].T).magnitude())>1]
    save('achieved-pose-acquisition-design.json',dict(best_two=observed_best,rejected=observed_rejected,remaining_candidates=observed_design,scope='Planning only: G pose actually achieved, front ID68 visibility/information only predicted. Excludes previously paired equivalent poses and explicitly rejected missing-ID68 brackets. Chassis/marker remount changes the nuisance model. No nominal angles used to rank poses.'))
    # Whole-bracket bootstrap, plus propagated frozen G lens bootstrap draws.
    rng=np.random.default_rng(6891);samples=[];failed=[];attempts=0
    lens_samples=json.loads((j.SESSION/'lens/uncertainty.json').read_text())['samples']
    while len(samples)<args.bootstrap and attempts<args.bootstrap*5:
        attempts+=1;ix=rng.integers(0,len(data['train']),len(data['train']));pp=[data['train'][i] for i in ix]
        if len({tuple(p['nominal']) for p in pp})<3:failed.append('insufficient pose excitation');continue
        dg=copy.copy(data);ls=lens_samples[(attempts*7)%len(lens_samples)];dg['Kg']=np.array([[ls[0],0,ls[2]],[0,ls[1],ls[3]],[0,0,1]]);dg['dg']=np.array([ls[4],ls[5],0,0,0]);f=j.fit_bundle(dg,pp,selected,initial=fit)
        if not f['success'] or not f['positive_depth'] or f['bound_hits']:failed.append('nonconverged/invalid/bound');continue
        samples.append(dict(lens_draw=ls,sampled_ids=[p['id'] for p in pp],fit=f,heldout=j.evaluate(dg,data['test'],f)))
    save('uncertainty.json',dict(attempted=attempts,successful=len(samples),failed=len(failed),failure_labels=failed,samples=samples,scope='whole bracket resampling plus existing whole-pose G lens bootstrap draws; G poses are latent pixel-fit nuisances; excludes common printer-scale, marker/floor warping, exposure lag, detector bias, remount and optical-origin anatomy'))
    # Historical floor lens checks: frozen chosen K, new per-block TF only.
    old=json.loads((OLD/'observations.json').read_text())['frames'];by={o['file']:o for o in old};oldfit=json.loads((OLD/'fit.json').read_text());floorchecks={}
    def floor_pose(records,K,d):
        TF=np.array(records[0]['board']['T_cam_board'])
        def res(v):return np.concatenate([((j.project(o['board']['obj'],j.mat(v),K,d)-np.array(o['board']['px']))/np.sqrt(len(o['board']['px']))).ravel() for o in records])
        sol=least_squares(res,j.vec(TF),loss='soft_l1',f_scale=.5);return j.mat(sol.x),bool(sol.success)
    for block,b in oldfit['front_registration'].items():
        rr=[by[f] for f in b['frames']];train=rr[:-2];test=rr[-2:];cc={}
        for name,K,d in [('old',data['Kf'],data['df']),('candidate',np.array(fit['K']),np.array(fit['dist']))]:
            TF,success=floor_pose(train,K,d);cc[name]=dict(T_F_W=TF.tolist(),success=success,training_files=[o['file'] for o in train],test_files=[o['file'] for o in test],training=j.stats([j.project(o['board']['obj'],TF,K,d)-np.array(o['board']['px']) for o in train]),heldout=j.stats([j.project(o['board']['obj'],TF,K,d)-np.array(o['board']['px']) for o in test]),height_m=float(abs(np.linalg.inv(TF)[2,3])))
        floorchecks[block]=cc
    save('historical-floor-validation.json',dict(blocks=floorchecks,scope='previous session floor frames not used in current optical fit/selection; previously seen in legacy floor analyses; TF newly estimated per stationary block from training floor frames; no old marker mount assumed shared'))
    # Refit identifiable axes from real G pixels under validated G lens.
    gs={int(Path(o['file']).name[:3]):o for o in json.loads((j.SESSION/'intrinsics-observations.json').read_text())};groupids={'shoulder':[85,87,88,89,90],'elbow200':[12,13,17],'elbow220':[35,36,37],'elbow230':[42,46,50]};axes={};recordsets={}
    for name,ids in groupids.items():
        rr=[dict(T=j.camera_pose(gs[i],data['Kg'],data['dg']),obj=np.array(gs[i]['obj']),px=np.array(gs[i]['px'])) for i in ids];recordsets[name]=rr;axes[name]=fit_pixel_axis(rr,data['Kg'],data['dg'])
    links={e:station_separation(axes['shoulder'],axes[e],[.1,.14,0]) for e in ['elbow200','elbow220','elbow230']}
    TF=np.array(fit['T_F_W']);M=np.array(fit['T_G_M']);E=axes['elbow230'];WG=recordsets['elbow230'][1]['T'];n=np.array(E['direction']);c=np.array(E['closest_point']);cg=WG[:3,3];cm=(WG@M)[:3,3];vg=cg-c-n*((cg-c)@n);vm=cm-c-n*((cm-c)@n)
    frontaxes={}
    for name,ids in [('elbow',[39,44,48]),('roll',[67,71,75,79,83])]:
        pp=[p for p in data['pairs'] if p['id'] in ids];poses=[]
        for p in pp:
            o=p['front'][0];K=np.array(fit['K']);d=np.array(fit['dist']);ok,r,t=j.cv2.solvePnP(j.MARKER,np.array(o['marker']['px'],float),K,d,flags=j.cv2.SOLVEPNP_ITERATIVE);poses.append(j.mat(np.r_[r.ravel(),t.ravel()/.1]))
        fa=fit_axis(poses);frontaxes[name]=dict(fit=fa,ids=ids,scope='exploratory front-only ID68 PnP rigid trajectory, including prior-seen holdouts; not independent L1')
        if name=='elbow':
            ng=TF[:3,:3]@n;nf=np.array(fa['direction']);frontaxes[name]['G_direction_disagreement_deg']=float(np.degrees(np.arccos(np.clip(abs(ng@nf),-1,1))))
    geometry=dict(axes=axes,links=links,shoulder_in_front=in_front(axes['shoulder'],TF),camera_height_m=float(abs(np.linalg.inv(TF)[2,3])),front_ID68_axes=frontaxes,effective=dict(elbow_to_G_perpendicular_W_m=vg.tolist(),elbow_to_M_perpendicular_W_m=vm.tolist(),G_radius_m=float(np.linalg.norm(vg)),M_radius_m=float(np.linalg.norm(vm)),physical_L2_m=None,physical_M4_zero_deg=None,physical_M5_station_m=None,pad_vector_m=None,gauge='L2 + mount_along_link is invariant; δL2=-δmount preserves optical/marker chain; M4 zero absorbed in free mount rotation; M5 translation along axis commutes with roll',gauge_jacobian=[[1,1]],null_direction=[float(1/np.sqrt(2)),float(-1/np.sqrt(2))]),scope='front K/TF changes camera-relative line registration, not intrinsic W-axis separation; no FK imposed; independent front shoulder missing, so no new independent L1 certification')
    save('geometry.json',geometry)
    save('candidate-proposal.json',dict(status='DO_NOT_ADOPT_PRODUCTION: retained pitch holdout and front/G reconstructed axis disagreement fail',front_camera=dict(K=fit['K'],dist=fit['dist'],size=[1600,1200]),pincher_camera=dict(K=data['Kg'].tolist(),dist=data['dg'].tolist(),size=[1600,896]),paired_registration=dict(T_F_W=fit['T_F_W'],T_G_M=fit['T_G_M'],session=str(j.SESSION.relative_to(ROOT))),effective_geometry=geometry['effective'],L1_common_station_m=links['elbow200']['separation_m'],production_patch=None,reason='Do not change K without jointly refitting optical mounts and effective geometry; no anatomical L2/wrist/TCP proposal'))
    after=snapshot();legacy=json.loads((j.SESSION/'baseline-before-sha256.json').read_text());changed=[p for p,h in before.items() if after.get(p)!=h];badlegacy=[p for p,h in legacy.items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
    manifest=json.loads((j.SESSION/'manifest.json').read_text());badraw=[r['file'] for r in manifest if hashlib.sha256((j.SESSION/r['file']).read_bytes()).hexdigest()!=r['sha256']]
    save('archive-verification.json',dict(snapshot_files=len(before),changed=changed,legacy_baseline_verified=len(legacy)-len(badlegacy),legacy_mismatches=badlegacy,originals_verified=len(manifest)-len(badraw),original_manifest_mismatches=badraw,scope='SHA256 before/after every preserved source file; legacy410 comparison; all92 originals verified against original manifest'))
    save('run-summary.json',dict(selected=selected,heldout=final['metrics'],fixed_heldout=fixed['metrics'],L1_m=links['elbow200']['separation_m'],front_height_m=geometry['camera_height_m'],hash_changed=changed,legacy_bad=badlegacy))
    print('SELECTED',selected,'FINAL',final['metrics'],'L1',links['elbow200']['separation_m'],flush=True)
    if changed or badlegacy or badraw:raise RuntimeError('Preserved source hash mismatch')

if __name__=='__main__':main()
