import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
import joint

class JointTests(unittest.TestCase):
    def test_projection_respects_transform_chain(self):
        K=np.array([[800.,0,800],[0,800,600],[0,0,1]])
        TF=np.eye(4);TF[2,3]=1
        WG=np.eye(4);WG[0,3]=.1
        GM=np.eye(4);GM[1,3]=.2
        uv=joint.project(joint.MARKER,TF@WG@GM,K,np.zeros(5))
        np.testing.assert_allclose(uv.mean(axis=0),[880,760],atol=1e-9)

    def test_output_guard_rejects_sources_and_symlink_alias(self):
        import tempfile
        with tempfile.TemporaryDirectory(dir=str(Path.home()/'.hermes/cache/scratch')) as td:
            root=Path(td);src=root/'source';src.mkdir();alias=root/'alias';alias.symlink_to(src,target_is_directory=True)
            for p in [src,src/'new',root,alias/'new']:
                with self.assertRaises(ValueError):joint.output_guard(p,[src])
            self.assertEqual(joint.output_guard(root/'result',[src]),(root/'result').resolve())

    def test_cli_refuses_preexisting_linked_output_without_changing_evidence(self):
        import os,tempfile
        from unittest.mock import patch
        import run
        with tempfile.TemporaryDirectory(dir=str(Path.home()/'.hermes/cache/scratch')) as td:
            root=Path(td)
            for kind in ['symlink','hardlink']:
                with self.subTest(kind=kind):
                    protected=root/(kind+'-protected');protected.write_bytes(b'preserved evidence')
                    out=root/kind;out.mkdir();leaf=out/'archive-before-sha256.json'
                    if kind=='symlink':leaf.symlink_to(protected)
                    else:os.link(protected,leaf)
                    # Stop before any fitting; only harmless scratch evidence is linked.
                    failure=None
                    with patch.object(sys,'argv',['run.py','--out',str(out)]),patch.object(joint,'load_data',side_effect=RuntimeError('stop after first write')):
                        try:run.main()
                        except (FileExistsError,RuntimeError) as exc:failure=exc
                    self.assertEqual(protected.read_bytes(),b'preserved evidence')
                    self.assertIsInstance(failure,FileExistsError)

    def test_cli_output_races_cannot_redirect_writes_into_evidence(self):
        import os,tempfile
        from unittest.mock import patch
        import run
        with tempfile.TemporaryDirectory(dir=str(Path.home()/'.hermes/cache/scratch')) as td:
            root=Path(td)
            for kind in ['symlink','hardlink','directory-swap']:
                with self.subTest(kind=kind):
                    source=root/(kind+'-source');source.mkdir();protected=source/'software-and-code.json';protected.write_bytes(b'preserved evidence')
                    out=root/kind
                    def attack():
                        if kind=='directory-swap':
                            out.rename(root/(kind+'-moved'));out.symlink_to(source,target_is_directory=True)
                        elif kind=='symlink':(out/protected.name).symlink_to(protected)
                        else:os.link(protected,out/protected.name)
                        return {'train':[]} # next metadata save runs, then stop before any fitting
                    with patch.object(sys,'argv',['run.py','--out',str(out)]),patch.object(joint,'load_data',side_effect=attack):
                        with self.assertRaises(KeyError):run.main()
                    self.assertEqual(protected.read_bytes(),b'preserved evidence')

    def test_preopen_ancestor_swap_cannot_overwrite_evidence(self):
        import os,tempfile
        from unittest.mock import patch
        import run
        with tempfile.TemporaryDirectory(dir=str(Path.home()/'.hermes/cache/scratch')) as td:
            root=Path(td)
            for stage in ['parent-open','output-open']:
                with self.subTest(stage=stage):
                    base=root/stage;base.mkdir()
                    protected=base/'protected';(protected/'nested/result').mkdir(parents=True)
                    evidence=protected/'nested/result/evidence.json';evidence.write_bytes(b'preserved evidence')
                    ancestor=base/'workspace';(ancestor/'nested').mkdir(parents=True)
                    out=ancestor/'nested/result';moved=base/'moved';opened=os.open;attacked=False
                    def swap_before_open(path,flags,*args,**kwargs):
                        nonlocal attacked
                        component='workspace' if stage=='parent-open' else 'result'
                        if not attacked and (Path(path)==out or (kwargs.get('dir_fd') is not None and str(path)==component)):
                            ancestor.rename(moved);ancestor.symlink_to(protected,target_is_directory=True);attacked=True
                        return opened(path,flags,*args,**kwargs)
                    with patch.object(os,'open',side_effect=swap_before_open):
                        try:
                            with run.output_writer(out,[protected]) as save:save('evidence.json',dict(generated=True))
                        except (OSError,ValueError):pass # rejecting a changed path is also safe
                    self.assertTrue(attacked,'swap must occur before the relevant descriptor opens')
                    self.assertEqual(evidence.read_bytes(),b'preserved evidence')

    def test_cv_selection_rejects_invalid_full_fits_and_folds(self):
        import copy,io
        from types import SimpleNamespace
        from unittest.mock import patch
        import run
        pairs=[dict(id=i,nominal=[i,0,0]) for i in range(2)]
        data=dict(train=pairs,test=[],pairs=pairs,policy=dict(pairs=[]))
        for scope in ['full','fold','evaluation','all']:
            cases=[('positive_depth',False)] if scope=='evaluation' else [('success',False)] if scope=='all' else [('success',False),('bound_hits',['f']),('positive_depth',False)]
            for field,value in cases:
                with self.subTest(scope=scope,field=field):
                    calls={};saved={}
                    def fit_bundle(data,train,model):
                        call=calls.get(model,0);calls[model]=call+1
                        fit=dict(model=model,success=True,bound_hits=[],positive_depth=True,K=[],conditioning=dict(condition=1))
                        if scope=='all' or (scope!='evaluation' and model=='f' and call==(0 if scope=='full' else 1)):fit[field]=copy.deepcopy(value)
                        return fit
                    def evaluate(data,test,fit):
                        return dict(positive_depth=not(scope=='evaluation' and fit['model']=='f'),metrics=dict(marker=dict(rms_px=1 if fit['model']=='f' else 10),floor=dict(rms_px=1)))
                    def save(name,obj):
                        saved[name]=copy.deepcopy(obj)
                        if name=='selection-frozen.json':raise RuntimeError('selection captured')
                    with patch.object(joint,'load_data',return_value=data),patch.object(joint,'fit_bundle',side_effect=fit_bundle),patch.object(joint,'evaluate',side_effect=evaluate),patch.object(sys,'stdout',io.StringIO()):
                        if scope=='all':
                            with self.assertRaisesRegex(ValueError,'No valid model eligible'):run.reproduce(SimpleNamespace(),[],save)
                            self.assertNotIn('selection-frozen.json',saved)
                            continue
                        with self.assertRaisesRegex(RuntimeError,'selection captured'):run.reproduce(SimpleNamespace(),[],save)
                    self.assertNotIn('f',saved['selection-frozen.json']['eligible'])
                    reasons=saved['models-progress.json']['f'].get('invalid_reasons',[])
                    self.assertTrue(any(scope in reason and field in reason for reason in reasons),reasons)

    @staticmethod
    def synthetic_data() -> tuple[dict,dict,np.ndarray,np.ndarray,np.ndarray]:
        K=np.array([[800.,0,800],[0,800,600],[0,0,1]]);d=np.zeros(5)
        X=np.array([[x,y,0] for x in [-.1,-.05,0,.05,.1] for y in [-.1,-.05,0,.05,.1]])
        TF=np.eye(4);TF[2,3]=1
        WG=np.eye(4);WG[2,3]=-1
        M=np.eye(4);M[2,3]=.5
        uv=joint.project(X,np.linalg.inv(WG),K,d)
        offset=np.column_stack([np.linspace(-.4,.4,len(X)),np.sin(np.arange(len(X)))*.3])
        g=[dict(obj=X.tolist(),px=(uv+sign*offset).tolist()) for sign in [-1,1]]
        front=dict(file='synthetic',board=dict(obj=X.tolist(),px=joint.project(X,TF,K,d).tolist(),T_cam_board=TF.tolist()),marker=dict(px=joint.project(joint.MARKER,TF@WG@M,K,d).tolist(),T_cam_marker=(TF@WG@M).tolist()))
        p=dict(id=1,nominal=[0,0,0],gripper=g,front=[front])
        data=dict(Kg=K,dg=d,Kf=K,df=d)
        return data,p,TF,M,WG

    def test_nonconverged_G_only_fit_is_rejected_by_all_callers(self):
        from unittest.mock import patch
        data,p,TF,M,WG=self.synthetic_data()
        fit=dict(K=data['Kf'].tolist(),dist=data['df'].tolist(),T_F_W=TF.tolist(),T_G_M=M.tolist())
        solver=joint.least_squares
        def limited_solver(*args,**kwargs):
            g_only=kwargs['max_nfev']==150
            kwargs['max_nfev']=1
            result=solver(*args,**kwargs)
            if g_only:self.assertFalse(result.success) # actual iteration limit, not fabricated status
            return result
        for caller in [lambda:joint.gripper_pose(data,p),lambda:joint.evaluate(data,[p],fit),lambda:joint.fit_bundle(data,[p],fixed_gripper=True)]:
            with self.subTest(caller=caller),patch.object(joint,'least_squares',side_effect=limited_solver):
                with self.assertRaisesRegex(ValueError,'G-only.*nonconverged'):caller()

    def test_positive_depth_covers_all_three_fitted_targets(self):
        data,p,TF,M,WG=self.synthetic_data()
        self.assertTrue(joint.errors(data,[p],TF,M,data['Kf'],data['df'],[WG])[1])
        for target in ['F-floor','F-marker','G-floor']:
            with self.subTest(target=target):
                F=TF.copy();GM=M.copy();W=WG.copy()
                if target=='F-floor':F[2,3]=-1;GM[2,3]=3
                elif target=='F-marker':GM[2,3]=-.5
                else:W[2,3]=1
                _,positive=joint.errors(data,[p],F,GM,data['Kf'],data['df'],[W])
                self.assertFalse(positive)

    def test_G_only_pose_rejects_converged_behind_camera_branch(self):
        from unittest.mock import patch
        data,p,TF,M,WG=self.synthetic_data()
        # Planar projection has an exact behind-camera mirror branch.
        W=WG.copy();W[:3,:3]=np.diag([-1.,-1.,1.]);W[2,3]=1
        for o in p['gripper']:o['px']=joint.project(o['obj'],np.linalg.inv(W),data['Kg'],data['dg']).tolist()
        solver=joint.least_squares
        def converged_solver(*args,**kwargs):
            sol=solver(*args,**kwargs);self.assertTrue(sol.success);return sol
        with patch.object(joint,'camera_pose',return_value=W),patch.object(joint,'least_squares',side_effect=converged_solver):
            with self.assertRaisesRegex(ValueError,'G-only.*positive depth'):joint.gripper_pose(data,p)

    def test_real_data_joint_fit_uses_training_pixels_only(self):
        data=joint.load_data()
        self.assertEqual(len(data['pairs']),8)
        self.assertEqual(len(data['train']),5)
        fit=joint.fit_bundle(data,data['train'],'fixed')
        self.assertTrue(fit['success'])
        self.assertLess(fit['metrics']['marker']['rms_px'],5)
        self.assertEqual(fit['fit_ids'],[p['id'] for p in data['train']])
        self.assertTrue(fit['positive_depth'])
        changed=dict(data);changed['pairs']=[dict(p) for p in data['pairs']]
        for p in changed['pairs']:
            if p['split']=='holdout':p['front']=[]
        again=joint.fit_bundle(changed,data['train'],'fixed')
        np.testing.assert_allclose(fit['T_G_M'],again['T_G_M'],atol=1e-10)

    def test_whole_front_holdout_does_not_change_gripper_pose(self):
        import copy
        data=joint.load_data();fit=joint.fit_bundle(data,data['train'],'f')
        p=data['test'][0];a=joint.evaluate(data,[p],fit)
        q=copy.deepcopy(p)
        for f in q['front']:f['marker']['px']=(np.array(f['marker']['px'])+10).tolist()
        b=joint.evaluate(data,[q],fit)
        np.testing.assert_allclose(a['poses'][0]['T_W_G'],b['poses'][0]['T_W_G'],atol=1e-12)
        self.assertGreater(b['metrics']['marker']['rms_px'],a['metrics']['marker']['rms_px'])
        self.assertTrue(a['poses'][0]['independent_front_pixels'])

    def test_cli_reproduces_real_trace_without_archive_writes(self):
        import subprocess,tempfile,json
        script=Path(__file__).with_name('run.py')
        with tempfile.TemporaryDirectory(dir=str(Path.home()/'.hermes/cache/scratch')) as td:
            result=subprocess.run([sys.executable,str(script),'--out',td+'/joint','--bootstrap','0'],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            trace=json.loads((Path(td)/'joint/models.json').read_text())
            self.assertEqual(len(trace['models']),5)
            self.assertEqual(trace['selected'],'fcyk1')
            self.assertAlmostEqual(trace['final']['metrics']['marker']['rms_px'],5.497539220183687,places=5)
            pitch=next(p for p in trace['final']['poses'] if p['id']==48)
            pitch_rms=float(np.sqrt(np.mean([f['marker_errors_px']['rms_px']**2 for f in pitch['front']])))
            self.assertAlmostEqual(pitch_rms,9.130255,places=5)
            for model in trace['models'].values():
                self.assertEqual(model['invalid_reasons'],[])
                for fit in [model['fit']]+[fold['fit'] for fold in model['cv']]:
                    self.assertTrue(fit['success']);self.assertTrue(fit['positive_depth']);self.assertEqual(fit['bound_hits'],[])
            self.assertEqual(len(trace['final']['poses']),3)
            hashes=json.loads((Path(td)/'joint/archive-verification.json').read_text())
            self.assertEqual(hashes['changed'],[])
            self.assertEqual(hashes['legacy_baseline_verified'],410)
            coverage=json.loads((Path(td)/'joint/coverage-and-acquisition.json').read_text())
            self.assertEqual(len(coverage['greedy_design_views']),2)
            self.assertGreater(coverage['greedy_design_views'][1]['cumulative_min_eigenvalue'],coverage['baseline_min_eigenvalue'])
            conditional=json.loads((Path(td)/'joint/conditional-versus-joint.json').read_text())
            self.assertEqual(conditional['candidate']['fit']['gripper_scope'],'fixed G-only estimate; conditional')

    def test_fixed_gripper_conditional_fit_preserves_measured_poses(self):
        data=joint.load_data();f=joint.fit_bundle(data,data['train'],'fcyk1',fixed_gripper=True)
        for p,T in zip(data['train'],f['T_W_G']):
            np.testing.assert_allclose(T,joint.gripper_pose(data,p),atol=1e-10)
        self.assertEqual(f['gripper_scope'],'fixed G-only estimate; conditional')

    def test_acquisition_information_is_psd_not_hardware_evidence(self):
        data=joint.load_data();fit=joint.fit_bundle(data,data['train'],'full')
        p=joint.design_view(data,fit,[800,300],.18,0,0)
        self.assertEqual(p['scope'],'design-only geometric prediction; reach/occlusion/rigidity unverified')
        H=np.array(p['information']);self.assertEqual(H.shape,(18,18))
        self.assertGreater(np.linalg.eigvalsh(H).min(),-1e-5)

    def test_design_can_use_an_actually_observed_gripper_pose(self):
        data=joint.load_data();fit=joint.fit_bundle(data,data['train'],'full');WG=joint.gripper_pose(data,data['train'][0])
        v=joint.design_view(data,fit,[0,0],1,0,0,observed_WG=WG)
        np.testing.assert_allclose(v['T_W_G'],WG,atol=1e-12)
        self.assertEqual(v['pose_source'],'retained observed G board pose; F counterpart design prediction only')

if __name__=='__main__':unittest.main()
