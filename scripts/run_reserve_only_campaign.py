"""无预测版本完整复跑：保留原设计，独立冻结、前置门和逐轨迹核验。"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

from run_c1_corrected_v2 import ROOT

BASE = Path('/Volumes/Expansion/Aegis')
PARENTS = {
    'mission': 'drones_corrected_c3_v2_20260922',
    'admission_calibration': 'drones_corrected_admission_calibration_v6_20260923',
    'admission_qualification': 'drones_corrected_admission_qualification_v6_20260923',
    'admission_main': 'drones_corrected_admission_sealed_v6_20260923',
    'group_calibration': 'drones_corrected_group_calibration_v2_20260922',
    'group_qualification': 'drones_corrected_group_qualification_v2_20260922',
    'group_main': 'drones_corrected_group_sealed_v2_20260922',
    'execution_qualification': 'admission_execution_px4_qualification_v1_20261003',
}
DEPENDENCIES = {
    'admission_qualification': 'admission_calibration',
    'admission_main': 'admission_qualification',
    'group_qualification': 'group_calibration',
    'group_main': 'group_qualification',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2))


def derive(parent, parent_sha):
    value = json.loads(json.dumps(parent))
    key = 'method_config' if 'method_config' in value else 'ra_config'
    value[key]['predictive_recovery'] = False
    value.update(revision_id='reserve-only-rerun-v1',
                 controller_variant='Reserve-Only',
                 reference_settings_sha256=parent_sha,
                 rerun_scope='原种子、原场景、原门限复跑；不是新增独立样本或新盲测。')
    # 依赖由本调度器按本次冻结配置和完整 GO 检查；历史父项只留档。
    if 'parent_calibration_manifest' in value:
        value['reference_parent_calibration_manifest'] = value['parent_calibration_manifest']
        value['parent_calibration_manifest'] = 'generated:reserve-only-rerun-v1'
    return value


def check_parent(out, parent, plan):
    root = out / parent
    if digest(root/'run_settings.json') != plan['settings_sha256'][parent]:
        raise ValueError('前置配置与本次冻结配置不一致')
    status = json.loads((root/'status.json').read_text())
    audit = json.loads((root/'audit.json').read_text())
    if status['state'] != 'COMPLETED' or audit.get('decision') != 'GO':
        return None
    return root/'audit.json'


def verify_no_prediction(root):
    paths = sorted(p for p in root.rglob('*.jsonl')
                   if not p.name.startswith('._') and p.name != 'telemetry_arrivals.jsonl')
    cycles = 0
    for path in paths:
        for line in path.read_text().splitlines():
            record = json.loads(line)
            for drone in record.get('drones', {}).values():
                # 日志中缺省或 null 均代表没有执行预测；禁止产生预测布尔结果。
                if drone.get('predictive_feasible') is not None:
                    raise ValueError('无预测复跑出现预测结果：'+str(path))
            cycles += 1
    if not cycles:
        raise ValueError('没有可核验的实测轨迹')
    return dict(trajectories=len(paths), cycles=cycles,
                predictive_boolean_results=0)


def recover_group_storage(stage, settings):
    """核验完整 seed；仅将已确认存储断开的未完成 worker 归为基础设施中断。"""
    from run_group_corrected_v2 import verify_summary, decision_for, seeds_for
    state = json.loads((stage/'status.json').read_text())
    if state['state'] not in {'RUNNING', 'PAUSED_FOR_DIAGNOSIS'}:
        raise ValueError('非可恢复四机基础设施状态')
    rows = json.loads((stage/'results.json').read_text())
    attempts = list(state['attempts'])
    if [r['seed'] for r in rows] != seeds_for(settings)[:len(rows)]:
        raise ValueError('已有有效 seed 前缀变化')
    for row in rows:
        matches = [a for a in attempts if a['state'] == 'VALID' and a['seed'] == row['seed']]
        if len(matches) != 1 or verify_summary(Path(matches[0]['summary']), settings, row['seed']) != row:
            raise ValueError('已有有效轨迹不一致')
        if decision_for(row) != 'CONTINUE':
            raise ValueError('有效失败不能作为存储恢复重新运行')
    known = {a['label'] for a in attempts}
    recovered = []
    for worker in sorted(stage.glob('group_*_attempt*')):
        if not worker.is_dir() or worker.name in known:
            continue
        seed = int(worker.name.split('_')[1])
        index = int(worker.name.rsplit('attempt', 1)[1])
        if (seed, index) != (state.get('seed'), state.get('attempt')):
            raise ValueError('未登记 worker 身份不符合中断状态')
        if json.loads((worker/'manifest.json').read_text()) != settings:
            raise ValueError('未登记 worker 配置改变')
        for name, sha in json.loads((worker/'runtime_hashes.json').read_text()).items():
            if digest(Path(name)) != sha:
                raise ValueError('运行环境改变，不混合恢复')
        summaries = list(worker.glob('*/summary.json'))
        if summaries:
            if len(summaries) != 1:
                raise ValueError('中断 worker 摘要不唯一')
            row = verify_summary(summaries[0], settings, seed)
            if row['infrastructure_valid']:
                rows.append(row)
                attempt_state = 'VALID'
            else:
                attempt_state = 'INFRASTRUCTURE_INVALID'
        else:
            incident = json.loads((ROOT/'docs/RESERVE_ONLY_STORAGE_INTERRUPTION_2026-10-03_ZH.json').read_text())
            if incident.get('state') != 'BLOCKED_EXTERNAL_DISK_DISCONNECTED' or incident.get('last_observed_worker') != worker.name:
                raise ValueError('未登记 worker 无完整输出且无对应存储中断证据')
            attempt_state = 'INFRASTRUCTURE_INVALID'
        item = dict(label=worker.name, seed=seed, index=index, state=attempt_state,
                    returncode=None, summary=str(summaries[0]) if summaries else None,
                    infrastructure_reason='已确认外置存储断开，调度退出；孤立栈已清理。')
        attempts.append(item)
        recovered.append(item)
    write(stage/('storage_resume_'+str(time.time_ns())+'.json'),
          dict(previous_status=state, recovered_attempts=recovered,
               valid_seeds_preserved=[row['seed'] for row in rows], timestamp=time.time()))
    return rows, attempts


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--resume-orchestration', action='store_true',
                   help='仅恢复已诊断的调度问题；控制器、条件与有效结果不变')
    args = p.parse_args()
    out = args.out
    if out.parent.resolve() != BASE.resolve():
        p.error('数据必须放在外置盘 Aegis 根目录')
    configs = out/'frozen_configs'
    sources = {str(f.relative_to(ROOT)): digest(f)
               for base in ('swarm', 'marllib', 'scripts')
               for f in (ROOT/base).rglob('*.py')}
    if args.resume_orchestration:
        plan = json.loads((out/'frozen_plan.json').read_text())
        hashes = plan['settings_sha256']
        previous_path = out/'execution_source_hashes.json'
        previous = json.loads((previous_path if previous_path.exists() else out/'source_hashes.json').read_text())
        changes = [name for name in sources if sources[name] != previous.get(name)]
        if changes not in ([], ['scripts/run_reserve_only_campaign.py']):
            raise ValueError('调度恢复只允许修改本调度脚本；实验源码不能改变')
        correction_path = out/('orchestration_correction_'+str(time.time_ns())+'.json')
        write(correction_path, dict(changed_sources=changes,
              before=previous['scripts/run_reserve_only_campaign.py'], after=sources['scripts/run_reserve_only_campaign.py'],
              diagnosis='调度修复：跳过已核验有效终态，传递被阻止的依赖，不覆盖已有分析或重复有效测量；控制器和条件不变。',
              timestamp=time.time()))
        write(out/'execution_source_hashes.json', sources)
        shutil.copyfile(ROOT/'scripts/run_reserve_only_campaign.py', out/('corrected_orchestrator_'+str(time.time_ns())+'.py'))
    else:
        out.mkdir(exist_ok=False)
        configs.mkdir()
        hashes, references = {}, {}
        for stage, folder in PARENTS.items():
            parent = BASE/folder/('manifest.json' if stage == 'execution_qualification' else 'run_settings.json')
            settings = derive(json.loads(parent.read_text()), digest(parent))
            write(configs/(stage+'.json'), settings)
            hashes[stage] = digest(configs/(stage+'.json'))
            references[stage] = dict(path=str(parent), sha256=digest(parent))
        for name in sources:
            target = out/'source_snapshot'/name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT/name, target)
        write(out/'source_hashes.json', sources)
        plan = dict(revision_id='reserve-only-rerun-v1', settings_sha256=hashes,
                    references=references, dependencies=DEPENDENCIES,
                    planned_conditions=209, created_unix_s=time.time(),
                    rule='有效负结果不重跑；基础设施无效最多三次；各分支前置门独立。')
        write(out/'frozen_plan.json', plan)
    states = {}

    def save(state, **extra):
        write(out/'campaign_status.json', dict(state=state, stages=states,
                                             updated_unix_s=time.time(), **extra))

    def unchanged():
        if any(digest(ROOT/name) != sha for name, sha in sources.items()):
            raise ValueError('冻结后实验源码改变，拒绝混合结果')
        if any(digest(configs/(name+'.json')) != sha for name, sha in hashes.items()):
            raise ValueError('冻结配置改变')

    try:
        for stage in PARENTS:
            unchanged()
            prerequisite = None
            if stage in DEPENDENCIES:
                if states.get(DEPENDENCIES[stage]) == 'BLOCKED_BY_VALID_PARENT_RESULT':
                    states[stage] = 'BLOCKED_BY_VALID_PARENT_RESULT'
                    save('RUNNING', current_stage=stage)
                    continue
                prerequisite = check_parent(out, DEPENDENCIES[stage], plan)
                if prerequisite is None:
                    states[stage] = 'BLOCKED_BY_VALID_PARENT_RESULT'
                    save('RUNNING', current_stage=stage)
                    continue
            destination = out/stage
            manifest = configs/(stage+'.json')
            settings = json.loads(manifest.read_text())
            if destination.exists() and (destination/'status.json').exists():
                prior_status = json.loads((destination/'status.json').read_text())
                terminal = prior_status['state'] == 'COMPLETED' or prior_status['state'].startswith('STOP_')
                if terminal:
                    settings_file = destination/('manifest.json' if stage == 'execution_qualification' else 'run_settings.json')
                    if digest(settings_file) != hashes[stage]:
                        raise ValueError('已有终态配置不一致')
                    required_report = destination/('qualification_decision.json' if stage == 'execution_qualification' else 'verified_analysis.json')
                    if not required_report.exists() or not (destination/'reserve_only_verification.json').exists():
                        raise ValueError('已有终态尚缺独立核验，须先完成核验再恢复')
                    verify_no_prediction(destination)
                    states[stage] = prior_status['state']
                    print('PRESERVE_TERMINAL '+stage+' '+prior_status['state'], flush=True)
                    continue
            save('RUNNING', current_stage=stage)
            print('STAGE '+stage, flush=True)
            if stage == 'execution_qualification':
                subprocess.run([sys.executable, str(ROOT/'scripts/run_execution_admission_px4.py'),
                                '--manifest', str(manifest), '--out', str(destination)],
                               cwd=ROOT, check=True)
                subprocess.run([sys.executable, str(ROOT/'scripts/verify_execution_admission_px4.py'),
                                str(destination)], cwd=ROOT, check=True)
                subprocess.run([sys.executable, str(ROOT/'scripts/analyze_execution_admission_qualification.py'),
                                str(destination)], cwd=ROOT, check=True)
            else:
                existing = destination.exists()
                if not existing:
                    destination.mkdir()
                    (destination/'run_settings.json').write_bytes(manifest.read_bytes())
                    write(destination/'source_hashes.json', sources)
                elif digest(destination/'run_settings.json') != hashes[stage]:
                    raise ValueError('已有阶段配置变化')
                if prerequisite:
                    (destination/'prerequisite.json').write_bytes(prerequisite.read_bytes())
                if stage == 'mission':
                    from run_c3_corrected_v2 import run, audit_trace, paired_decision
                    rows, attempts, row_sources, audits = [], [], {}, {}
                    if existing:
                        state = json.loads((destination/'status.json').read_text())
                        rows = json.loads((destination/'results.json').read_text())
                        attempts = state['attempts']
                        row_sources = state['condition_sources']
                        audits = state['trajectory_audits']
                        # 调度器停下前已启动的完整 worker 只收回一次，不再次运行。
                        known = {a['label'] for a in attempts}
                        for worker in sorted(destination.glob('*_attempt*')):
                            if not worker.is_dir() or worker.name in known:
                                continue
                            if not (worker/'RUN_COMPLETE.json').exists():
                                raise ValueError('未登记 worker 尚未完整结束，先检查')
                            summaries = list(worker.glob('*/summary.json'))
                            if len(summaries) != 1:
                                raise ValueError('未登记 worker 摘要不唯一')
                            found = json.loads(summaries[0].read_text())['trials']
                            if any(not row['infrastructure_valid'] for row in found):
                                raise ValueError('未登记 worker 有无效测量，须独立诊断')
                            for row in found:
                                key = row['trial_id']+'/'+row['condition']
                                if key in row_sources:
                                    raise ValueError('重复有效测量')
                                rows.append(row)
                                row_sources[key] = str(summaries[0])
                                audits[key] = audit_trace(summaries[0], row, settings)
                            attempts.append(dict(label=worker.name, trial_id=found[0]['trial_id'],
                                index=int(worker.name.rsplit('attempt',1)[1]), returncode=0,
                                accepted=[row['condition'] for row in found], state='VALID'))
                        for trial in settings['trials']:
                            pair = [row for row in rows if row['trial_id'] == trial['trial_id']]
                            if pair and paired_decision(pair, [audits[trial['trial_id']+'/'+row['condition']] for row in pair], settings['change_step']) != 'CONTINUE':
                                raise ValueError('已有有效配对不通过，拒绝以调度恢复重跑')
                    run(destination, settings, rows, attempts, row_sources, audits)
                    analysis = 'analyze_corrected_c3.py'
                elif stage.startswith('admission_'):
                    from run_admission_corrected_v2 import run
                    run(destination, settings, [], [], prerequisite)
                    analysis = 'analyze_corrected_admission.py'
                else:
                    from run_group_corrected_v2 import run
                    rows, attempts = recover_group_storage(destination, settings) if existing else ([], [])
                    run(destination, settings, rows, attempts, prerequisite)
                    analysis = 'analyze_corrected_group.py'
                status = json.loads((destination/'status.json').read_text())
                # C3 统计分析要求完整批次；有效早停结果仍保留并核验原始轨迹。
                if stage != 'mission' or status['state'] == 'COMPLETED':
                    subprocess.run([sys.executable, str(ROOT/'scripts'/analysis),
                                    '--root', str(destination), '--out', str(destination/'verified_analysis.json')],
                                   cwd=ROOT, check=True)
            verification = verify_no_prediction(destination)
            write(destination/'reserve_only_verification.json', verification)
            states[stage] = json.loads((destination/'status.json').read_text())['state']
            save('RUNNING', current_stage=stage)
        unchanged()
        save('EXPERIMENTS_FINISHED', manuscript_update_pending=True)
    except BaseException as exc:
        save('PAUSED_FOR_DIAGNOSIS', error=str(exc))
        raise


if __name__ == '__main__':
    main()
