import unittest,copy,hashlib
from analyze import *
class ValidationChecks(unittest.TestCase):
 def test_frozen_inputs_unchanged(self):
  for path,h in read(HERE/'frozen-hashes.json').items():self.assertEqual(hashlib.sha256((BASE/path).read_bytes()).hexdigest(),h)
 def test_G_pose_excludes_front_pixels(self):
  p={'id':23,'front':[read(HERE/'022-comparator1-Fbefore-obs.json')],'gripper':[read(HERE/(s+'-obs.json'))['board'] for s in ['023-comparator1-G1','024-comparator1-G2']]}
  a=j.gripper_pose({'Kg':Kg,'dg':dg},p);q=copy.deepcopy(p);q['front'][0]['marker']['px']=(np.array(q['front'][0]['marker']['px'])+1000).tolist();q['front'][0]['board']['px']=(np.array(q['front'][0]['board']['px'])-500).tolist()
  np.testing.assert_allclose(a,j.gripper_pose({'Kg':Kg,'dg':dg},q),atol=1e-12)
 def test_stationarity_rejections_retained(self):
  primary=read(HERE/'prospective-evaluation.json')['brackets'];self.assertTrue(primary[1]['missing']);self.assertFalse(primary[2]['models']['fixed']['accepted_stationarity'])
  recover=read(HERE/'detector-recovery-prospective-evaluation.json')['brackets'];self.assertFalse(recover[3]['models']['fixed']['accepted_stationarity'])
 def test_metrics_from_raw_all_corners(self):
  for fn in ['prospective-evaluation.json','detector-recovery-prospective-evaluation.json']:
   for p in read(HERE/fn)['brackets']:
    for m in p['models'].values():
     es=[np.array(o['errors_xy_px']) for o in m['front']];self.assertEqual(j.stats(es),m['metrics']);self.assertEqual(m['metrics']['n'],8);self.assertTrue(m['positive_depth'])
 def test_final_idle_and_no_consumers(self):
  s=read(HERE/'final-state.json');self.assertEqual(s['telemetry']['status']['value'],{'battery':63,'state':3,'error_code':0,'in_cleaning':0,'in_returning':0});self.assertEqual(s['telemetry']['camera']['value'],[11685])
  for stream in ['z70','z70_jpeg']:self.assertEqual(s['camera']['streamDiagnostics'][stream]['consumers'],[])
if __name__=='__main__':unittest.main()
