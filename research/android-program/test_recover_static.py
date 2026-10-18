import os
import unittest
from recover_static import literal, recover

class RecoveryTests(unittest.TestCase):
    def test_literals_are_not_executed(self):
        with self.assertRaises((ValueError,SyntaxError)):
            literal("__import__('os').system('false')")
    def test_tokens_in_strings_are_preserved(self):
        self.assertEqual(literal("{'x': null, 'y': false, 's':'null true false'}"),{'x':None,'y':False,'s':'null true false'})
    @unittest.skipUnless(os.environ.get('ANDROID_HASM'),'Set ANDROID_HASM for original artifact verification')
    def test_original_artifact(self):
        r=recover(os.environ['ANDROID_HASM'])
        self.assertEqual(r['hasm_sha256'],'5fa398d080e801a31b7d55598ce8c696535808fbbbba6b5bac13c413d7ba4a2c')
        self.assertFalse(r['requested_M4_up_step1_generated'])
        up=next(x['record'] for x in r['records'] if x['lines'][0]==1430379)
        self.assertEqual(up,{'value':[0,1,0,0],'id':15,'cmd_id':'arm_joints','step':2})
        block=next(x['record'] for x in r['records'] if x['lines'][0]==1430426)
        self.assertEqual(block['fields'],{'dir':'1','step':'2'})
        self.assertEqual(block['type'],'arm_m4')
        self.assertEqual(len(r['records']),5)

if __name__=='__main__':unittest.main()
