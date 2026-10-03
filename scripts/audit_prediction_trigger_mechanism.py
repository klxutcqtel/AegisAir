"""只读核验既有有效轨迹，区分真实前瞻预警、当前不可行占位和独立切换贡献。"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def analyze(root, methods):
    settings = json.loads((root / 'run_settings.json').read_text())
    rows = json.loads((root / 'results.json').read_text())
    status = json.loads((root / 'status.json').read_text())
    if status['state'] != 'COMPLETED':
        raise ValueError('仅核验完整完成组')
    selected = [r for r in rows if r.get('method', 'GROUP') in methods]
    trials = []
    for row in selected:
        if not row['infrastructure_valid']:
            raise ValueError('不允许混入基础设施无效条件')
        # 以已冻结结果的轨迹身份查找，不选择结果较好的尝试。
        paths = [f for f in root.rglob(row['trajectory']) if not f.name.startswith('._') and sha(f) == row['trajectory_sha256']]
        if len(paths) != 1:
            raise ValueError('轨迹身份不唯一或哈希变化')
        path = paths[0]
        records = [json.loads(line) for line in path.read_text().splitlines()]
        if len(records) != settings['max_steps'] or len(set(x['step'] for x in records)) != len(records):
            raise ValueError('轨迹周期不完整')
        if min(x['min_rho'] for x in records) != row['min_rho']:
            raise ValueError('最低裕度与既有摘要不一致')
        counts = Counter()
        alerts = []
        entries = []
        no_prediction_active = False
        clean_steps = 0
        previous_active = False
        ds = [next(iter(x['drones'].values())) for x in records]
        for index, (x, d) in enumerate(zip(records, ds)):
            # 联合QP和切换状态按周期计，不能按UAV重复计数。
            for other in x['drones'].values():
                for key in ('primary_feasible', 'predictive_feasible', 'feasibility_reserve', 'recovery_active', 'recovery_reason'):
                    if other[key] != d[key]:
                        raise ValueError('联合状态在不同UAV中不一致：' + key)
            primary = d['primary_feasible']
            pred = d['predictive_feasible']
            reserve_low = d['feasibility_reserve'] < 1.0
            active = bool(d['recovery_active'])
            counts['cycles'] += 1
            counts['primary_infeasible'] += primary is False
            counts['prediction_true'] += pred is True
            counts['prediction_false'] += pred is False
            counts['false_with_current_infeasible'] += pred is False and primary is False
            actual_forecast_alert = pred is False and primary is True
            counts['forecast_alert_when_primary_feasible'] += actual_forecast_alert
            counts['forecast_alert_with_reserve_low'] += actual_forecast_alert and reserve_low
            independent = actual_forecast_alert and not reserve_low
            counts['independent_prediction_trigger'] += independent
            counts['prediction_reason_cycles'] += d['recovery_reason'] == 'predictive_infeasible'
            counts['recovery_cycles'] += active
            counts['primary_feasible_cycles'] += primary is True
            if actual_forecast_alert:
                future = ds[index + 1:index + 4]
                future_bad = next((j + 1 for j, t in enumerate(future) if t['primary_feasible'] is False), None)
                last_entry = entries[-1]['step'] if entries else None
                alerts.append(dict(step=x['step'], reserve=d['feasibility_reserve'], previous_recovery_active=previous_active,
                    current_recovery_active=active, recovery_reason=d['recovery_reason'],
                    cycles_since_latest_entry=None if last_entry is None else x['step'] - last_entry,
                    next_three_observed_primary_infeasible_at=future_bad,
                    independent_prediction_trigger=independent))
            if active and not previous_active:
                entries.append(dict(step=x['step'], reason=d['recovery_reason'],
                    reserve=d['feasibility_reserve'], predictive_feasible=pred, primary_feasible=primary))
            # 在同一已记录状态序列上删除预测分支，重建五周期解除锁存。
            # 不宣称删除计算后实际PX4轨迹、时序或闭环结果完全相同。
            if primary is False or reserve_low:
                no_prediction_active = True
                clean_steps = 0
            elif no_prediction_active:
                clean_steps += 1
                if clean_steps >= 5:
                    no_prediction_active = False
                    clean_steps = 0
            counts['same_state_no_prediction_latch_mismatch'] += no_prediction_active != active
            previous_active = active
        if 'hocbf_predictive_infeasible_steps' in row and counts['prediction_false'] != row['hocbf_predictive_infeasible_steps']:
            raise ValueError('预测汇总字段与轨迹不一致')
        counts['recovery_entries'] = len(entries)
        trials.append(dict(trial_id=row.get('trial_id', 'seed_' + str(row['seed'])), seed=row['seed'],
            method=row.get('method', 'GROUP'), trajectory=str(path), trajectory_sha256=row['trajectory_sha256'],
            counts=dict(counts), forecast_alerts=alerts, recovery_entries=entries,
            source_hashes_sha256=sha(path.parent.parent / 'source_hashes.json') if (path.parent.parent / 'source_hashes.json').exists() else None))
    totals = Counter()
    for trial in trials:
        totals.update(trial['counts'])
    source_file = root / 'source_hashes.json'
    return dict(root=str(root), results_sha256=sha(root / 'results.json'), settings_sha256=sha(root / 'run_settings.json'),
        source_ra_sha256=json.loads(source_file.read_text()).get('swarm/ra/runtime_assurance.py') if source_file.exists() else None,
        episodes_verified=len(trials), totals=dict(totals), trials=trials)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--storage', type=Path, default=Path('/Volumes/Expansion/Aegis'))
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    configs = [
        ('main', 'drones_corrected_c1_v2_20260921', ['AEGIS_HOCBF_V4']),
        ('ablation', 'drones_corrected_ablation_v2_20260921', ['AEGIS_HOCBF_V4']),
        ('tau_nominal', 'drones_corrected_c2_v2_20260922', ['AEGIS_HOCBF_V4']),
        ('tau_mismatch', 'drones_corrected_c2_v2_20260922', ['AEGIS_HOCBF_V4_BAD_TAU']),
        ('external_matched', 'drones_corrected_external_matched_v2_20260922', ['AEGIS_HOCBF_V4']),
        ('ood_wide', 'drones_corrected_ood_v2_20260922/wide_head_on', ['AEGIS_HOCBF_V4']),
        ('ood_diagonal', 'drones_corrected_ood_v2_20260922/diagonal_crossing', ['AEGIS_HOCBF_V4']),
        ('ood_hold', 'drones_corrected_ood_v2_20260922/command_hold_jitter', ['AEGIS_HOCBF_V4']),
        ('four_uav', 'drones_corrected_group_sealed_v2_20260922', ['GROUP']),
    ]
    report = dict(analysis_type='既有数据事后机制诊断，不调参、不重跑、不改原始数据',
        counting_unit='联合控制周期；不同实验组分别报告，不作为独立样本合池',
        limits=['当前主约束不可行时的预测false不能算前瞻计算失败',
            '未来实测状态属于已经接管的实际轨迹，不是无接管反事实，不据此估计预测器误报率或闭环因果收益',
            '删除预测的锁存重建只比较同一实测状态上的切换逻辑，实际删除计算可能改变调度和轨迹',
            '未保存完整预测状态、每层QP残差及预测失败层，无法完成位置预测误差、失败步精度或数值失败分解'], campaigns={})
    for name, directory, methods in configs:
        result = analyze(args.storage / directory, methods)
        report['campaigns'][name] = result
        print(name, result['episodes_verified'], result['totals'], flush=True)
    with args.out.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)


if __name__ == '__main__':
    main()
