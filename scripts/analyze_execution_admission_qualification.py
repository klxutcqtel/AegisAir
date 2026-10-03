"""冻结限定验证的任务判定；只从核验完毕的全组结果生成报告。"""
import argparse
import json
from pathlib import Path
from admission_execution_study import digest


def condition_failures(row, expected):
    reasons=[]
    if row['collision']: reasons.append('collision')
    if row['min_rho']<=0: reasons.append('nonpositive_rho')
    if row['min_distance_m']<2.4: reasons.append('task_separation')
    if row['audit']['selected_qp_infeasible_steps']: reasons.append('selected_qp_infeasible')
    if row['audit']['ra_bypass_count'] or row['safety_bypass_count']: reasons.append('authority_bypass')
    if row['published_command_constraint_failure_count']: reasons.append('published_constraint_failure')
    if row['published_command_constraint_unknown_count']: reasons.append('published_constraint_unknown')
    commits=row['admission']['plans_committed']
    if expected=='admit':
        if commits!=1: reasons.append('commit_count')
        if not row['post_failure_critical_reached']: reasons.append('mission_not_completed')
    elif expected=='reject':
        if commits!=0: reasons.append('unexpected_commit')
    else: raise ValueError('未冻结的接纳标签')
    return reasons


def analyze(root):
    receipt=root/'verified_analysis_v2.json'
    verified=json.loads(receipt.read_text());m=json.loads((root/'manifest.json').read_text())
    if not verified['verified'] or verified['state']!='COMPLETED': raise ValueError('全组未完成并核验')
    rows=json.loads((root/'results.json').read_text());lookup={(r['trial_id'],r['condition']):r for r in rows}
    expected={(t['trial_id'],c) for t in m['trials'] for c in t['condition_order']}
    if set(lookup)!=expected or len(rows)!=len(expected): raise ValueError('分母不完整或重复')
    pairs=[]
    for t in m['trials']:
        by={c:lookup[t['trial_id'],c] for c in m['conditions']}
        failures={c:condition_failures(r,t['expected_dynamic_admission'] if c!='HOLD' else 'reject') for c,r in by.items()}
        replays=[r for r in verified['same_state_replays'] if r['trial_id']==t['trial_id']]
        pairs.append({'trial_id':t['trial_id'],'geometry_id':t['geometry_id'],'seed':t['seed'],
            'expected':t['expected_dynamic_admission'],'failures':failures,
            'separation_m':{c:r['min_distance_m'] for c,r in by.items()},
            'goal_reached':{c:r['post_failure_critical_reached'] for c,r in by.items()},
            'same_state_route_differences':sum(r['different_route_or_decision'] for r in replays),
            'same_state_replay_count':len(replays),
            'geometry_task_failure_resolved':t['expected_dynamic_admission']=='admit' and 'task_separation' in failures['GEOMETRY'] and not failures['NOMINAL_DYNAMIC']})
    benefit_geometries=sorted({p['geometry_id'] for p in pairs if p['geometry_task_failure_resolved'] and p['same_state_route_differences'] and p['geometry_id'] not in ('aligned_control','occupied_control')})
    counts={c:{'conditions':len(pairs),'qualified_outcomes':sum(not p['failures'][c] for p in pairs),
        'goal_reached':sum(p['goal_reached'][c] for p in pairs),'task_separation_failures':sum('task_separation' in p['failures'][c] for p in pairs),
        'published_constraint_failures_conditions':sum('published_constraint_failure' in p['failures'][c] for p in pairs)} for c in m['conditions']}
    passed=all(not p['failures']['NOMINAL_DYNAMIC'] for p in pairs) and len(benefit_geometries)>=2
    result={'protocol_id':m['protocol_id'],'manifest_sha256':digest(root/'manifest.json'),'verification_sha256':digest(receipt),
        'decision':'GO_IN_FROZEN_FAMILY' if passed else 'NO_GO_IN_FROZEN_FAMILY','pairs':pairs,'method_counts':counts,
        'benefit_geometries':benefit_geometries,'dynamic_failure_trials':[p['trial_id'] for p in pairs if p['failures']['NOMINAL_DYNAMIC']],
        'scope':'预声明变换家族、同向控制和占据拒绝控制；种子嵌套于几何，不作36个独立样本统计推断；全程任务间距2.4m，非碰撞阈值。'}
    with (root/'qualification_decision.json').open('x') as f: json.dump(result,f,ensure_ascii=False,indent=2)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);args=p.parse_args()
    r=analyze(args.root);print(json.dumps({k:v for k,v in r.items() if k!='pairs'},ensure_ascii=False,indent=2))
