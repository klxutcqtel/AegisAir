#!/usr/bin/env python3
"""只读诊断：同状态接管逻辑与瓶颈候选重建；不覆盖原始实验。"""
import argparse, json, hashlib, sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from marllib.run_c_recoverability_admission_gazebo import _admission_config
from swarm.recovery import RecoverabilityAdmissionCoordinator
from swarm.safety import DroneSnapshot

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def trajectory(root,row):
    paths=[p for p in root.rglob(row['trajectory']) if not p.name.startswith('._') and sha(p)==row['trajectory_sha256']]
    if len(paths)!=1:raise ValueError('轨迹身份不唯一')
    return paths[0],[json.loads(x) for x in paths[0].read_text().splitlines()]
def switching(records,config):
    threshold=config['recovery_reserve_threshold'];clear=config['recovery_clear_steps'];active=False;clean=0;counts=dict(cycles=0,independent_prediction_triggers=0,latch_mismatches=0,prediction_reason_cycles=0,forecast_alerts=0,alerts_before_fault=0)
    for r in records:
        ds=list(r['drones'].values());d=ds[0]
        for other in ds:
            for k in ['primary_feasible','predictive_feasible','feasibility_reserve','recovery_active']:
                assert other[k]==d[k]
        low=d['feasibility_reserve']<threshold;bad=d['primary_feasible'] is False;alert=d['primary_feasible'] is True and d['predictive_feasible'] is False
        if bad or low:active=True;clean=0
        elif active:
            clean+=1
            if clean>=clear:active=False;clean=0
        counts['cycles']+=1;counts['independent_prediction_triggers']+=int(alert and not low)
        counts['latch_mismatches']+=int(active!=d['recovery_active']);counts['prediction_reason_cycles']+=int(d['recovery_reason']=='predictive_infeasible')
        counts['forecast_alerts']+=int(alert);counts['alerts_before_fault']+=int(alert and r['step']<30)
    return counts

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(exist_ok=False)
    roots={'historical':Path('/Volumes/Expansion/Aegis/drones_corrected_admission_calibration_v6_20260923'),'reserve_only':Path('/Volumes/Expansion/Aegis/reserve_only_px4_rerun_v1_20261003/admission_calibration')}
    plan={'类型':'事后只读诊断；不是新增闭环独立验证','状态':'仅使用已核验有效原轨迹，旧/新瓶颈故障第30周期，及固定的交叉状态组合','比较':'原几何候选和动态接纳重建；同状态删除预测分支重建锁存；不调种子或参数','边界':'锁存相同不证明实时闭环轨迹相同；计算耗时影响需要专门时序对照。'}
    (a.out/'plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2))
    evidence={};states={};configs={}
    for name,root in roots.items():
        settings=json.loads((root/'run_settings.json').read_text());configs[name]=settings
        episodes=[]
        for row in json.loads((root/'results.json').read_text()):
            assert row['infrastructure_valid'];path,recs=trajectory(root,row);assert len(recs)==settings['max_steps']
            episodes.append({'trial':row['trial_id'],'condition':row['condition'],'trajectory_sha256':sha(path),'switching':switching(recs,settings['ra_config'])})
            if row['seed']==13303 and row['condition']=='RECOVERABILITY_ADMISSION_RA':
                fault=next(r for r in recs if r['step']==settings['change_step']);states[name]=fault['drones']
        evidence[name]={'settings_sha256':sha(root/'run_settings.json'),'episodes':episodes}
    config=_admission_config(configs['historical']['recoverability_admission'],rate_hz=configs['historical']['rate_hz'],execution_tau_s=configs['historical']['ra_config']['execution_tau_s'],command_feedforward_tau_s=configs['historical']['tau_command_s'])
    assert configs['historical']['recoverability_admission']==configs['reserve_only']['recoverability_admission']
    reconstructions=[]
    for failed_origin in states:
        for healthy_origin in states:
            ds={2:states[failed_origin]['2'],3:states[healthy_origin]['3']};snap={i:DroneSnapshot(i,tuple(d['pos']),velocity=tuple(d['v_actual'])) for i,d in ds.items()}
            c=RecoverabilityAdmissionCoordinator(config);failed=c._failed_trajectory(snap[2]);start=np.array(snap[3].position[:2]);goal=np.array([4.,0.]);candidates=c._candidate_routes(start,goal,failed)
            clearances=[c._route_clearance(points,failed) for points in candidates]
            c._admit(start=start,initial_velocity=np.array(snap[3].velocity[:2]),goal=goal,failed_trajectory=failed,altitude=snap[3].position[2])
            reconstructions.append({'failed_state':failed_origin,'healthy_state':healthy_origin,'candidate_count':len(candidates),'max_geometric_clearance_m':max(clearances),'threshold_m':config.effective_clearance_m,'geometrically_clear_candidates':sum(v+1e-9>=config.effective_clearance_m for v in clearances),'decision':c.state,'rollout_candidates':c.rollout_candidate_count,'geometric_rejections':c.geometric_rejection_count,'route':c.route})
    for result in reconstructions:
        if result['failed_state']==result['healthy_state']:
            rows=json.loads((roots[result['failed_state']]/'results.json').read_text())
            baseline=next(r['recoverability_admission'] for r in rows if r['seed']==13303 and r['condition']=='RECOVERABILITY_ADMISSION_RA')
            assert result['candidate_count']==baseline['candidate_count']
            assert result['geometric_rejections']==baseline['geometric_rejection_count']
            assert result['rollout_candidates']==baseline['rollout_candidate_count']
            assert result['route']==baseline['route']
    report={'plan':plan,'evidence':evidence,'candidate_reconstructions':reconstructions,'source_hashes':{str(p.relative_to(ROOT)):sha(p) for p in [ROOT/'swarm/ra/runtime_assurance.py',ROOT/'swarm/recovery/recoverability_admission.py',Path(__file__)]}}
    (a.out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(reconstructions,ensure_ascii=False));print({name:{k:sum(e['switching'][k] for e in d['episodes']) for k in d['episodes'][0]['switching']} for name,d in evidence.items()})
if __name__=='__main__':main()
