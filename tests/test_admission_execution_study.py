"""新开发实验的独立物理积分、权限与比较口径检查。"""
import importlib.util
import json
from pathlib import Path
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('admission_execution_study', ROOT/'scripts/admission_execution_study.py')
study = importlib.util.module_from_spec(spec)
spec.loader.exec_module(study)


class AdmissionExecutionStudyTest(unittest.TestCase):
    def setUp(self):
        self.manifest = json.loads((ROOT/'configs/admission_execution_development_v1.json').read_text())

    def test_exact_free_response(self):
        p, v = study.advance(np.zeros(2), np.zeros(2), np.array([1., 0.]),
                             {'tau_s': .7, 'accel_mps2': 10}, .2)
        alpha = 1-np.exp(-.2/.7)
        self.assertAlmostEqual(v[0], alpha)
        self.assertAlmostEqual(p[0], .2-.7*alpha)

    def test_saturation_transition_matches_substeps(self):
        plant = {'tau_s': .7, 'accel_mps2': 1.}
        p, v = study.advance(np.zeros(2), np.zeros(2), np.array([1., -1.]), plant, .8)
        q, w = np.zeros(2), np.zeros(2)
        for _ in range(80):
            q, w = study.advance(q, w, np.array([1., -1.]), plant, .01)
        np.testing.assert_allclose(p, q, atol=1e-12)
        np.testing.assert_allclose(v, w, atol=1e-12)
        self.assertAlmostEqual(p[0], -p[1])

    def test_occupied_goal_has_no_candidates(self):
        self.assertEqual(study.candidates(self.manifest, self.manifest['geometries'][-1]), [])

    def test_scenario_acceptance_implies_nominal_acceptance(self):
        geometry = self.manifest['geometries'][0]
        routes = study.candidates(self.manifest, geometry)
        self.assertGreater(len(routes), 0)
        n = study.select(routes, [0., 0.], self.manifest, geometry, 'nominal_dynamic')
        s = study.select(routes, [0., 0.], self.manifest, geometry, 'scenario_dynamic')
        if s['route_index'] is not None:
            self.assertIsNotNone(n['route_index'])

    def test_referee_revokes_failed_vehicle_command(self):
        m = dict(self.manifest, horizon_s=.15)
        geo = self.manifest['geometries'][0]
        outcome = study.referee(study.candidates(m, geo)[0], [0., 0.], m, geo,
                                self.manifest['nominal'], capture=True)
        for row in outcome['trace']:
            self.assertEqual(row['published_commands']['2'], [0., 0.])

    def test_internal_output_rejected(self):
        with self.assertRaises(ValueError):
            study.external_root(ROOT/'output')


if __name__ == '__main__':
    unittest.main()
