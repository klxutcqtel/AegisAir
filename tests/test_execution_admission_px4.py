"""真实飞控入口前的固定路线与失败权限检查，不启动PX4。"""
import json
from pathlib import Path
import unittest
import numpy as np
from marllib.execution_admission import FixedRouteAdmissionCoordinator, FailedVehicleLoiterPilot
from marllib.run_c_recoverability_admission_gazebo import _admission_config
from swarm.safety import DroneSnapshot


class FixedRouteAdmissionTest(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[1]
        self.m = json.loads((root/'configs/admission_execution_px4_development_v1.json').read_text())
        self.config = _admission_config(self.m['recoverability_admission'],rate_hz=20,execution_tau_s=.7,command_feedforward_tau_s=.7)

    def test_methods_share_waypoints_and_dont_mutate_library(self):
        routes = self.m['geometries']['turning']['waypoint_routes']
        a = FixedRouteAdmissionCoordinator(self.config,waypoint_routes=routes,geometry_only=True)
        b = FixedRouteAdmissionCoordinator(self.config,waypoint_routes=routes)
        start, goal = np.array([-5.,0.]), np.array([4.,0.])
        aa = a._candidate_routes(start,goal,[])
        bb = b._candidate_routes(start,goal,[])
        for x,y in zip(aa,bb):
            np.testing.assert_array_equal(x,y)
        aa[0][1][0] = 100
        self.assertEqual(a.waypoint_routes[0][0][0],routes[0][0][0])

    def test_occupied_goal_rejects_without_commit(self):
        g = self.m['geometries']['occupied']
        for geometry_only in (False,True):
            c = FixedRouteAdmissionCoordinator(self.config,waypoint_routes=g['waypoint_routes'],geometry_only=geometry_only)
            snaps = {int(k):DroneSnapshot(int(k),tuple(v),velocity=(0.,0.,0.)) for k,v in g['reset_starts'].items()}
            c.step(step=30,snapshots=snaps,base_goals={int(k):tuple(v) for k,v in g['base_goals'].items()},
                   mission_change={'kind':'fail_drone','drone':2})
            s = c.summary()
            self.assertEqual(s['plans_committed'],0)
            self.assertEqual(s['state'],'rejected_hold')
            self.assertIsNotNone(s['decision_state'])

    def test_loiter_only_changes_nominal_failed_intent(self):
        actions = FailedVehicleLoiterPilot().actions(positions={2:np.zeros(3),3:np.zeros(3)},
            velocities={2:np.zeros(3),3:np.zeros(3)},goals={2:np.array([4.,0.]),3:np.array([0.,-4.])})
        np.testing.assert_array_equal(actions[2],np.zeros(2))
        np.testing.assert_array_equal(actions[3],[0.,-1.5])


if __name__=='__main__':
    unittest.main()
