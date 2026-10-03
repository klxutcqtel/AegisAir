"""异步遥测对齐回归：不放宽门限，不改变原测量对象。"""
import unittest
import numpy as np
from marllib.phase5_runner import _admission_snapshots_at_epoch
from swarm.safety import DroneSnapshot
from swarm.recovery import RecoverabilityAdmissionConfig, RecoverabilityAdmissionCoordinator

class AdmissionCommonEpochTests(unittest.TestCase):
    def test_advances_each_vehicle_by_its_own_age(self):
        raw={2:DroneSnapshot(2,(0.,1.,2.),velocity=(2.,0.,-1.),timestamp_ms=940),3:DroneSnapshot(3,(3.,0.,2.),velocity=(1.,0.,0.),timestamp_ms=990)}
        aligned,ages=_admission_snapshots_at_epoch(raw,1000)
        self.assertEqual(ages,{2:.06,3:.01})
        np.testing.assert_allclose(aligned[2].position,(.12,1.,1.94))
        np.testing.assert_allclose(aligned[3].position,(3.01,0.,2.))
        self.assertEqual(raw[2].position,(0.,1.,2.));self.assertEqual(aligned[2].timestamp_ms,940)
    def test_future_timestamp_is_not_extrapolated_backwards(self):
        raw={2:DroneSnapshot(2,(0.,0.,2.),timestamp_ms=1010,velocity=(1.,0.,0.))}
        aligned,ages=_admission_snapshots_at_epoch(raw,1000)
        self.assertEqual(ages[2],0.);self.assertEqual(aligned[2].position,raw[2].position)
    def test_asynchronous_bottleneck_reproduces_spurious_refusal(self):
        raw={2:DroneSnapshot(2,(-.22873306274414062,-.014669991098344326,2.501828670501709),velocity=(1.424722671508789,-.03839806839823723,-.011546485126018524),timestamp_ms=941),3:DroneSnapshot(3,(-2.519902229309082,-1.1950844526290894,2.496899366378784),velocity=(1.4841244220733643,-.00864629540592432,-.0058258227072656155),timestamp_ms=989)}
        config=RecoverabilityAdmissionConfig(arc_extra_m=(1.4,),stabilization_wait_s=2.)
        def decide(states):
            c=RecoverabilityAdmissionCoordinator(config)
            c.step(step=30,snapshots=states,base_goals={2:(4.,0.,2.5),3:(4.,-1.2,2.5)},mission_change={'kind':'fail_drone','drone':2})
            return c
        original=decide(raw);aligned,_=_admission_snapshots_at_epoch(raw,1000);corrected=decide(aligned)
        self.assertEqual(original.state,'rejected_hold');self.assertEqual(original.geometric_rejection_count,101)
        self.assertEqual(corrected.state,'admitted');self.assertEqual(corrected.geometric_rejection_count,98)
        self.assertGreaterEqual(corrected.predicted_min_clearance_m,2.6)
        self.assertEqual(config.effective_clearance_m,2.6)
if __name__=='__main__':unittest.main()
