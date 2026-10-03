"""资格判定必须捕获任务间距、发布失败和错误接纳，不混同于碰撞。"""
import copy
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_execution_admission_qualification import condition_failures


class QualificationOutcomeTest(unittest.TestCase):
    def setUp(self):
        self.row={'collision':False,'min_rho':.5,'min_distance_m':2.5,
            'audit':{'selected_qp_infeasible_steps':0,'ra_bypass_count':0},'safety_bypass_count':0,
            'published_command_constraint_failure_count':0,'published_command_constraint_unknown_count':0,
            'admission':{'plans_committed':1},'post_failure_critical_reached':True}

    def test_positive_rho_and_completion_do_not_hide_task_spacing_failure(self):
        self.row['min_distance_m']=2.399
        self.assertEqual(condition_failures(self.row,'admit'),['task_separation'])

    def test_feasible_qp_does_not_hide_published_constraint_failure(self):
        self.row['published_command_constraint_failure_count']=1
        self.assertEqual(condition_failures(self.row,'admit'),['published_constraint_failure'])

    def test_rejection_requires_zero_commit_not_goal_completion(self):
        self.row['admission']['plans_committed']=0;self.row['post_failure_critical_reached']=False
        self.assertEqual(condition_failures(self.row,'reject'),[])
        self.assertIn('commit_count',condition_failures(self.row,'admit'))

    def test_unknown_and_bypass_fail_closed(self):
        self.row['audit']['ra_bypass_count']=1;self.row['published_command_constraint_unknown_count']=1
        self.assertEqual(condition_failures(self.row,'admit'),['authority_bypass','published_constraint_unknown'])
