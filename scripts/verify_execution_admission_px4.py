"""只读核验独立PX4接纳开发组及每个实测状态的两方法重放。"""
import argparse
import json
from pathlib import Path
from run_execution_admission_px4 import validate_row
from admission_execution_study import ROOT,digest
from marllib.execution_admission import FixedRouteAdmissionCoordinator
from marllib.run_c_recoverability_admission_gazebo import _admission_config
from swarm.safety import DroneSnapshot


def verify(root):
    m=json.loads((root/'manifest.json').read_text())
    state=json.loads((root/'status.json').read_text())
    if state['state'] not in ('COMPLETED','STOP_VALID_NEGATIVE'):
        raise ValueError('实验未结束，不能作为最终结果核验')
    for name,value in json.loads((root/'source_hashes.json').read_text()).items():
        source=ROOT/name
        frozen=root/'source_snapshot'/name
        if digest(source)!=value and (not frozen.exists() or digest(frozen)!=value):
            raise ValueError('原源码变化且无哈希一致快照: '+name)
    for name,value in json.loads((root/'runtime_hashes.json').read_text()).items():
        if digest(Path(name))!=value:
            raise ValueError('实际PX4运行时变化')
    rows=json.loads((root/'results.json').read_text())
    expected=[(t['trial_id'],c,t['seed']) for t in m['trials'] for c in t['condition_order']]
    identities=[(r['trial_id'],r['condition'],r['seed']) for r in rows]
    if identities != (expected if state['state']=='COMPLETED' else expected[:len(rows)]):
        raise ValueError('有效条件身份不完整或非原定前缀')
    replays=[]
    for row in rows:
        t=next(t for t in m['trials'] if t['trial_id']==row['trial_id'])
        attempt=[a for a in state['attempts'] if a['state']=='VALID' and a['label'].startswith(row['trial_id']+'_'+row['condition']+'_attempt')]
        if len(attempt)!=1 or validate_row(Path(attempt[0]['summary']).parent,root/'manifest.json',m,t,row['condition'])!=row:
            raise ValueError('原始条件归属或摘要不一致')
        if not row['infrastructure_valid']:
            raise ValueError('有效分母含基础设施无效')
        if row['condition']=='HOLD':
            continue
        g=m['geometries'][t['geometry_id']]
        s=row['admission']['decision_state']
        decisions=[]
        for geometry_only in (True,False):
            c=FixedRouteAdmissionCoordinator(_admission_config(m['recoverability_admission'],rate_hz=20,
                 execution_tau_s=.7,command_feedforward_tau_s=.7),waypoint_routes=g['waypoint_routes'],geometry_only=geometry_only)
            c.step(step=t['change_step'],snapshots={int(i):DroneSnapshot(int(i),tuple(v['position']),velocity=tuple(v['velocity'])) for i,v in s.items()},
                   base_goals={int(k):tuple(v) for k,v in g['base_goals'].items()},mission_change={'kind':'fail_drone','drone':2})
            decisions.append({'geometry_only':geometry_only,'state':c.state,'route':c.route,
                              'rejection_reason':c.rejection_reason,'dynamic_rejection_reasons':c.dynamic_rejection_reasons})
        replays.append({'trial_id':row['trial_id'],'original_condition':row['condition'],'decisions':decisions,
                        'different_route_or_decision':decisions[0]['state']!=decisions[1]['state'] or decisions[0]['route']!=decisions[1]['route']})
    result={'verified':True,'protocol_id':m['protocol_id'],'state':state['state'],'valid_conditions':len(rows),
            'infrastructure_invalid_attempts':sum(a['state']=='INFRASTRUCTURE_INVALID' for a in state['attempts']),
            'collisions':sum(r['collision'] for r in rows),'global_min_rho':min(r['min_rho'] for r in rows),
            'selected_qp_infeasible_steps':sum(r['audit']['selected_qp_infeasible_steps'] for r in rows),
            'authority_bypass_count':sum(r['audit']['ra_bypass_count'] for r in rows),
            'same_state_replays':replays,
            'outcomes':[{'trial_id':r['trial_id'],'condition':r['condition'],'min_distance_m':r['min_distance_m'],
                         'min_rho':r['min_rho'],'goal_reached':r['post_failure_critical_reached'],
                         'commits':r['admission']['plans_committed'],'published_constraint_failures':r['published_command_constraint_failure_count']}
                        for r in rows],
            'scope':m['scope']+'；不是统计显著/连续时间证明；2.4m为任务间距要求，不是碰撞阈值。'}
    with (root/'verified_analysis_v2.json').open('x') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path)
    args=p.parse_args()
    result=verify(args.root)
    print(json.dumps({k:v for k,v in result.items() if k not in ('same_state_replays','outcomes')},ensure_ascii=False))
