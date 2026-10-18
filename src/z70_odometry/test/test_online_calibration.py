import math
from pathlib import Path
import tempfile
import unittest
from z70_odometry.chassis_calibrator import OnlineCalibration

class OnlineTests(unittest.TestCase):
    def motion(self,c,index,velocity=0,omega=.5,rejected=False,silent=False,dispatch='sent'):
        start=index*6+1; duration=.6; th=omega*duration*(.65 if omega>=0 else .67); x=.28; old=.16
        for j in range(17):
            t=start-1+j*.25
            if silent and 3<j<14: continue
            fraction=max(0,min(1,(t-start)/duration)); angle=th*fraction
            distance=velocity*duration*.55*fraction
            dx=distance*(math.sin(angle)/angle if angle else 1)+(x-old)*(math.cos(angle)-1)
            dy=distance*((1-math.cos(angle))/angle if angle else 0)+(x-old)*math.sin(angle)
            c.add_pose(dict(t=t,x=dx,y=dy,yaw=angle,camera_x_m=old,rejected=rejected and j==6))
        command=dict(id=str(index),method='app_rc_move',params=[dict(velocity=velocity,omega=omega,duration=600,seqnum=index+1)],dispatch=dispatch,response='received',sentAt=start*1000,replyAt=(start+.02)*1000)
        c.add_command(command); c.process(start+4)
        return command
    def test_stream_convergence_dedup_and_persistence(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'online.json'; c=OnlineCalibration(path)
            for i in range(24):
                command=self.motion(c,i,velocity=.1 if i%3==2 else 0,omega=0 if i%3==2 else (.5 if i%3==0 else -.5))
                c.add_command(command); c.process(i*6+5)
            result=c.summary()
            self.assertEqual(result['n'],24)
            self.assertAlmostEqual(result['yaw_positive']['gain'],.65)
            self.assertAlmostEqual(result['yaw_negative']['gain'],.67)
            self.assertAlmostEqual(result['forward']['gain'],.55)
            self.assertAlmostEqual(result['camera_x']['value_m'],.28)
            self.assertTrue(result['camera_x']['ready'])
            restored=OnlineCalibration(path)
            self.assertEqual(restored.summary(),result)
            restored.add_command(command); restored.process(200)
            self.assertEqual(restored.summary()['n'],24)
    def test_rejected_silent_and_uncertain_samples_are_skipped(self):
        with tempfile.TemporaryDirectory() as folder:
            c=OnlineCalibration(Path(folder)/'online.json')
            self.motion(c,0,rejected=True); self.motion(c,1,silent=True); self.motion(c,2,dispatch='unknown')
            self.motion(c,3)
            self.assertEqual(c.summary()['n'],1)
            self.assertEqual(c.summary()['skipped'],3)
    def test_mixed_turn_translation_converges_and_offset_is_consumed(self):
        with tempfile.TemporaryDirectory() as folder:
            c=OnlineCalibration(Path(folder)/'online.json')
            for i in range(12): self.motion(c,i,velocity=.1,omega=.5 if i%2==0 else -.5)
            summary=c.summary()
            self.assertAlmostEqual(summary['camera_x']['value_m'],.28)
            self.assertAlmostEqual(summary['forward']['gain'],.55)
            from types import SimpleNamespace
            from z70_odometry.calibration_state import fill_state, calibrated_camera_x
            def message():
                return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace()),
                                       evidence=SimpleNamespace(stale=False),validity=SimpleNamespace())
            not_ready=dict(summary,camera_x=dict(summary['camera_x'],ready=False))
            self.assertIsNone(calibrated_camera_x(fill_state(message(),not_ready,'test',1.)))
            self.assertAlmostEqual(calibrated_camera_x(fill_state(message(),summary,'test',1.)),.28)
    def test_overlapping_commands_are_not_misassigned(self):
        with tempfile.TemporaryDirectory() as folder:
            c=OnlineCalibration(Path(folder)/'online.json')
            c.add_command(dict(id='a',method='app_rc_move',params=[dict(velocity=0,omega=.5,duration=600)],dispatch='sent',response='received',sentAt=1000,replyAt=1020))
            c.add_command(dict(id='b',method='app_rc_move',params=[dict(velocity=0,omega=-.5,duration=600)],dispatch='sent',response='received',sentAt=2000,replyAt=2020))
            c.process(10)
            self.assertEqual(c.summary()['n'],0)
            self.assertEqual(c.summary()['skipped'],2)
