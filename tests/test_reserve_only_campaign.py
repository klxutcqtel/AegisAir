"""复跑只改变预测开关，保留旧配置默认行为及本次前置门身份。"""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from run_reserve_only_campaign import derive, check_parent, digest
from marllib.run_c_recoverability_admission_gazebo import _ra_kwargs
from marllib.run_c3_gazebo import _c3_ra_kwargs
from marllib.run_c1_sota_cbf_gazebo import _method_kwargs


class ReserveOnlyCampaignTests(unittest.TestCase):
    def test_driver_flag_preserves_all_other_parameters(self):
        parent = json.loads((ROOT/'configs/c3_closed_loop_v3_hocbf_v4_validation_v1.json').read_text())
        config = parent['ra_config']
        for factory in (_ra_kwargs, _c3_ra_kwargs):
            legacy = factory(config)
            selected = factory({**config, 'predictive_recovery': False})
            self.assertTrue(legacy['hocbf_predictive_recovery'])
            self.assertFalse(selected['hocbf_predictive_recovery'])
            selected['hocbf_predictive_recovery'] = True
            self.assertEqual(legacy, selected)

    def test_derive_does_not_change_trials_or_modify_parent(self):
        parent = json.loads((ROOT/'configs/c3_closed_loop_v3_hocbf_v4_validation_v1.json').read_text())
        original = copy.deepcopy(parent)
        selected = derive(parent, 'reference-hash')
        self.assertEqual(parent, original)
        self.assertEqual(parent['trials'], selected['trials'])
        config = selected['ra_config'].copy()
        self.assertFalse(config.pop('predictive_recovery'))
        self.assertEqual(config, parent['ra_config'])

    def test_group_driver_respects_disabled_prediction(self):
        profile = json.loads((ROOT/'configs/reserve_only_selected_method_v1.json').read_text())
        actual = _method_kwargs(profile['method'], profile['config'], .7)
        self.assertFalse(actual['hocbf_predictive_recovery'])
        self.assertTrue(actual['hocbf_pb_recovery'])

    def test_rejects_historical_or_incomplete_go(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            stage = root/'calibration'
            stage.mkdir()
            (stage/'run_settings.json').write_text('{}')
            (stage/'status.json').write_text('{"state":"RUNNING"}')
            (stage/'audit.json').write_text('{"decision":"GO"}')
            plan = {'settings_sha256': {'calibration': digest(stage/'run_settings.json')}}
            self.assertIsNone(check_parent(root, 'calibration', plan))
            (stage/'status.json').write_text('{"state":"COMPLETED"}')
            self.assertEqual(check_parent(root, 'calibration', plan), stage/'audit.json')
            (stage/'run_settings.json').write_text('{"old":true}')
            with self.assertRaises(ValueError):
                check_parent(root, 'calibration', plan)
