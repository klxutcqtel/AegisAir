"""新栈运行冻结清单；保留无效启动及每个协议的停止门。"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from run_paper_step_response_rerun import ROOT, GATE, DOCKER, IMAGE, run


def prepare_worker_manifest(source, out, manifest, family, single_method):
    worker_manifest = source.resolve()
    if family == 'c3' and single_method:
        # C3 worker 从文件读取 condition_order，不能只过滤调度器的内存对象。
        worker_manifest = out.resolve()/'selected_conditions.json'
        with worker_manifest.open('x') as stream:
            json.dump(manifest, stream, ensure_ascii=False, indent=2)
    return worker_manifest


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--family', choices=['c1', 'c3', 'admission', 'group'], default='c1')
    p.add_argument('--prerequisite', type=Path, help='接纳 qualification/新对照所依赖的本次 GO audit')
    p.add_argument('--single-trial', help='仅供修订版比较调度器使用')
    p.add_argument('--single-method', help='仅供修订版比较调度器使用')
    p.add_argument('--single-group-seed', type=int, help='仅供修正版 GroupSlot 调度器补跑缺失 seed')
    args = p.parse_args()
    manifest = json.loads(args.manifest.read_bytes())
    if args.single_group_seed is not None and (args.family!='group' or manifest.get('revision_id') not in {'group-corrected-v2','reserve-only-rerun-v1'}):
        p.error('单 seed 仅用于修正版 GroupSlot')
    if args.single_trial or args.single_method:
        c1_selection = (manifest['protocol_id'] in {
                'aegisair-c1-corrected-comparison-v2',
                'aegisair-c1-corrected-ablation-v2',
                'aegisair-c1-corrected-external-matched-v2',
                'aegisair-corrected-c1-family-v1',
                } and args.family == 'c1' and args.single_trial and args.single_method)
        c3_selection = (args.family == 'c3' and args.single_trial and
                        manifest['protocol_id']=='aegisair-c3-closed-loop-v3-hocbf-v4-validation-v1' and
                        manifest.get('revision_id') in {'c3-corrected-v2','reserve-only-rerun-v1'})
        admission_selection = (args.family == 'admission' and args.single_trial and args.single_method and
            manifest.get('revision_id') in {'admission-corrected-v2','admission-corrected-v5','admission-arc-probe-v6','admission-corrected-v6','reserve-only-rerun-v1','admission-common-epoch-v1'} and manifest['protocol_id'] in {
                'aegisair-c-recoverability-admission-calibration-v4',
                'aegisair-c-recoverability-admission-qualification-v4',
                'aegisair-admission-comparison-development-v1',
                'aegisair-c-recoverability-admission-calibration-v5',
                'aegisair-c-recoverability-admission-qualification-v5',
                'aegisair-c-recoverability-admission-sealed-corrected-v5',
                'aegisair-admission-comparison-development-v5',
                'aegisair-c-recoverability-admission-arc-probe-v6',
                'aegisair-c-recoverability-admission-calibration-v6',
                'aegisair-c-recoverability-admission-qualification-v6',
                'aegisair-c-recoverability-admission-sealed-corrected-v6',
                'aegisair-admission-comparison-development-v6'})
        if not (c1_selection or c3_selection or admission_selection):
            p.error('单条件/单配对调度仅允许明确修订版本')
        chosen = next((t for t in manifest['trials'] if t['trial_id']==args.single_trial), None)
        if chosen is None or (args.single_method and args.single_method not in chosen['condition_order']):
            p.error('trial/method 不在运行记录中')
        manifest['trials'] = [{**chosen, 'condition_order':[args.single_method] if args.single_method else chosen['condition_order']}]
    if args.family != 'group' and len(manifest['drone_ids']) != 2:
        p.error('此入口只支持两机')
    if args.family == 'group':
        if 'c3-group-slot-4uav-' not in manifest['protocol_id']:
            p.error('四机入口仅允许 GroupSlot，不切换 SEQUENTIAL_PASS')
        seeds = manifest.get('sealed_seeds', manifest.get('qualification_seeds', [manifest.get('gazebo_seed')]))
        if args.single_group_seed is not None:
            if args.single_group_seed not in seeds:
                p.error('seed 不在原定 GroupSlot 集合')
            seeds = [args.single_group_seed]
        manifest['trials'] = [dict(trial_id=f'group_{seed}', seed=seed, condition_order=['GROUP_SLOT']) for seed in seeds]
        if 'qualification_seeds' in manifest or 'sealed_seeds' in manifest:
            prior = json.loads(args.prerequisite.read_text()) if args.prerequisite else {}
            required = 'qualification' if 'sealed_seeds' in manifest else 'calibration'
            if prior.get('decision') != 'GO' or required not in prior.get('protocol_id', ''):
                p.error('GroupSlot 本次前置门未通过')
    if args.family == 'admission' and ('qualification' in manifest['protocol_id'] or 'comparison' in manifest['protocol_id']):
        if args.prerequisite is None or json.loads(args.prerequisite.read_text()).get('decision') != 'GO':
            p.error('接纳前置 GO 未满足')
        parent = manifest.get('parent_calibration_manifest')
        if parent and not parent.startswith('generated:') and hashlib.sha256((ROOT/'configs'/parent).read_bytes()).hexdigest() != manifest['parent_calibration_manifest_sha256']:
            p.error('冻结父配置哈希不匹配')
    args.out.mkdir(parents=True, exist_ok=False)
    (args.out/'manifest.json').write_bytes(args.manifest.read_bytes())
    worker_manifest = prepare_worker_manifest(args.manifest, args.out, manifest,
                                               args.family, args.single_method)
    if args.prerequisite:
        (args.out/'current_prerequisite.json').write_bytes(args.prerequisite.read_bytes())
    px4_root = GATE.parent/'PX4-Autopilot'
    runtime_files = [px4_root/'build/px4_sitl_default/bin/px4', GATE/'worlds/s1_single_obstacle.sdf', GATE/'scripts/launch_multi_sitl_pose.sh']
    (args.out/'runtime_hashes.json').write_text(json.dumps({str(f): hashlib.sha256(f.read_bytes()).hexdigest() for f in runtime_files}, indent=2))
    (args.out/'source_hashes.json').write_text(json.dumps({str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest() for base in ('swarm','marllib','scripts') for f in (ROOT/base).rglob('*.py')}, indent=2))
    broker, adapter = 'aegisair-paper-c1-broker', 'aegisair-paper-c1-adapter'
    broker_created, adapter_created = False, False
    procs, streams = [], []
    active_logs = None
    def cleanup_trial():
        nonlocal adapter_created
        if adapter_created:
            if active_logs is not None:
                with (active_logs/'adapter_cleanup_capture.log').open('a') as f:
                    subprocess.run([DOCKER,'logs',adapter],stdout=f,stderr=subprocess.STDOUT)
                with (active_logs/'docker_stats_cleanup.log').open('a') as f:
                    subprocess.run([DOCKER,'stats','--no-stream',adapter,broker],stdout=f,stderr=subprocess.STDOUT)
            with (args.out/'last_adapter_cleanup.log').open('a') as f:
                subprocess.run([DOCKER, 'rm', '-f', adapter], stdout=f, stderr=subprocess.STDOUT)
            adapter_created = False
        for proc in reversed(procs):
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            if proc.poll() is None:
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)
            # launcher 退出不代表 gz 子进程退出；清除该独立 session 的残留。
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        procs.clear()
        for f in streams:
            f.close()
        streams.clear()
    try:
        if subprocess.run(['pgrep', '-x', 'px4'], stdout=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError('已有 PX4，拒绝混用')
        run([DOCKER, 'image', 'inspect', IMAGE], args.out/'image.json')
        run([sys.executable, '-m', 'pip', 'freeze'], args.out/'pip-freeze.txt')
        run(['git', '-C', str(px4_root), 'rev-parse', 'HEAD'], args.out/'px4_commit.txt')
        run(['/opt/homebrew/bin/gz', 'sim', '--versions'], args.out/'gazebo_version.txt')
        run([DOCKER, 'run', '-d', '--name', broker, '-p', '127.0.0.1:1883:1883',
             'eclipse-mosquitto:2', 'mosquitto', '-c', '/mosquitto-no-auth.conf'], args.out/'broker.log')
        broker_created = True
        stopped_scenarios = set()
        all_rows = []
        protocol = manifest['protocol_id']
        for trial in manifest['trials']:
            scenario = trial.get('scenario_id', 'base')
            if scenario in stopped_scenarios:
                continue
            paired = []
            # 原 C3 runner 按冻结顺序在同一新栈内 reset 后跑完整配对。
            for method in (['PAIRED'] if args.family == 'c3' else trial['condition_order']):
                label = trial['trial_id']+'_'+method
                print('START '+label, flush=True)
                logs = args.out/(label+'_logs')
                logs.mkdir()
                active_logs = logs
                f = (logs/'timing_observer.log').open('x'); streams.append(f)
                procs.append(subprocess.Popen([sys.executable,str(ROOT/'scripts/observe_telemetry_timing.py'),
                    '--out',str(logs/'telemetry_arrivals.jsonl')],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
                for drone in manifest['drone_ids']:
                    f = (logs/f'heartbeat{drone}.log').open('w'); streams.append(f)
                    procs.append(subprocess.Popen([sys.executable, str(GATE/'scripts/gcs_heartbeat.py'), '--port', str(18570+drone), '--duration-s', '600'], stdout=f, stderr=subprocess.STDOUT, start_new_session=True))
                f = (logs/'launch.log').open('w'); streams.append(f)
                poses = '2=0,-3,0.5;3=0,3,0.5' if args.family != 'group' else '2=-0.8,-3,0.5;3=0.8,3,0.5;4=0.8,-3,0.5;5=-0.8,3,0.5'
                instances = ','.join(str(i) for i in manifest['drone_ids'])
                env = {**os.environ, 'C3_GAZEBO_SEED': str(trial['seed']), 'POSES': poses}
                launcher = subprocess.Popen(['bash', str(GATE/'scripts/launch_multi_sitl_pose.sh'), 'S1', instances], cwd=ROOT, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
                procs.append(launcher)
                deadline = time.monotonic()+120
                while time.monotonic()<deadline:
                    if launcher.poll() is not None:
                        raise RuntimeError(f'{label} SITL 提前退出')
                    if 'state_dir=' in (logs/'launch.log').read_text():
                        break
                    time.sleep(1)
                else:
                    raise RuntimeError(f'{label} SITL 超时')
                offsets = {2:'-3,0,0',3:'3,0,0'} if args.family != 'group' else {2:'-3,-0.8,0',3:'3,0.8,0',4:'-3,0.8,0',5:'3,-0.8,0'}
                origin_args = [item for i, value in offsets.items() for item in ('-e',f'ORIGIN_OFFSET_{i}={value}')]
                run([DOCKER, 'run', '-d', '--name', adapter, '--link', broker+':mqtt',
                     '-p', '127.0.0.1:8889:8889/udp', '-e', 'XRCE_UDP_PORT=8889',
                     '-e', 'ADAPTER_INSTANCES='+instances.replace(',', ' '), *origin_args,
                     '-e', 'ADAPTER_ARGS=--no-read-only --mqtt-host mqtt --mqtt-port 1883 --control-rate-hz 20 --telemetry-rate-hz 20', IMAGE], logs/'adapter_start.log')
                adapter_created = True
                run([sys.executable, str(ROOT/'scripts/wait_for_px4_telemetry.py'), '--drone-ids', *instances.split(','),'--timeout-s','90'], logs/'telemetry.log')
                command = [sys.executable, str(ROOT/'marllib/run_c1_sota_cbf_gazebo.py'), '--manifest',str(args.manifest.resolve()), '--out-dir',str(args.out/label), '--trial-id',trial['trial_id'], '--method',method]
                if args.family == 'c3':
                    command = [sys.executable, str(ROOT/'marllib/run_c3_gazebo.py'), '--manifest',str(worker_manifest), '--out-dir',str(args.out/label), '--trial-id',trial['trial_id'], '--velocity-command-mode','feedforward_tau','--tau-command-s','.7']
                elif args.family == 'admission':
                    command = [sys.executable, str(ROOT/'marllib/run_c_recoverability_admission_gazebo.py'), '--manifest',str(args.manifest.resolve()), '--out-dir',str(args.out/label), '--trial-id',trial['trial_id'], '--condition',method]
                elif args.family == 'group':
                    command = [sys.executable,str(ROOT/'marllib/run_c3_group_slot_gazebo.py'),'--manifest',str(args.manifest.resolve()),'--output',str(args.out/label)]
                    if 'qualification_seeds' in manifest or 'sealed_seeds' in manifest:
                        command += ['--seed',str(trial['seed'])]
                (logs/'command.json').write_text(json.dumps(command))
                try:
                    run(command, logs/'runner.log', timeout=420)
                finally:
                    run([DOCKER,'logs',adapter],logs/'adapter.log')
                result = json.loads((args.out/label/'summary.json').read_text())
                if result['manifest_sha256'] != hashlib.sha256(worker_manifest.read_bytes()).hexdigest():
                    raise RuntimeError('manifest 哈希不一致')
                if args.family == 'group':
                    result['trials'] = [{**result['trial'], 'seed':trial['seed'], 'm4_go':result['m4_go']}]
                paired.extend(result['trials']); all_rows.extend(result['trials'])
                cleanup_trial()
                (args.out/'progress.json').write_text(json.dumps(all_rows,ensure_ascii=False,indent=2))
                print('DONE '+label, flush=True)
                if any(not row.get('infrastructure_valid', False) for row in result['trials']):
                    raise RuntimeError(f'{label} infrastructure_invalid，保留现场并暂停，不自动计为有效结果')
            if args.single_trial:
                # 单条件仅收集结果；由外层在完整配对后使用 V4/基线分层规则。
                failed = False
            elif args.family == 'group':
                failed = any(not r['m4_go'] for r in paired)
            elif args.family == 'admission':
                sys.path.insert(0, str(ROOT))
                from marllib.analyze_c_recoverability_admission_calibration import analyze
                audit = analyze(manifest, all_rows)
                (args.out/'audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2))
                failed = any(not check['passed'] for key in ('admission_checks','hold_checks') for check in audit[key])
            elif args.family == 'c3':
                r0 = next(r for r in paired if r['condition']=='R0')
                r1 = next(r for r in paired if r['condition']=='R1')
                failed = (r0['critical_reached'] or not r1['critical_reached'] or
                          (r1.get('counters') or {}).get('mission_changes') != 1 or
                          any(r['collision'] or r['min_rho']<=0 for r in paired))
            elif 'postfreeze-ood' in protocol:
                selected = [r for r in paired if r['method']=='AEGIS_HOCBF_V4']
                failed = any(r['collision'] or not r['mission_complete'] or r['min_rho']<=0 for r in selected)
            elif 'execution-bridge' in protocol or 'trigger-comparison' in protocol:
                failed = any(r['collision'] for r in paired)
            else:
                failed = any(r['collision'] or not r['mission_complete'] or r['min_rho']<=0 for r in paired)
            if failed:
                stopped_scenarios.add(scenario)
                (args.out/'STOP_RULE_NO_GO.json').write_text(json.dumps(sorted(stopped_scenarios)))
                if 'postfreeze-ood' not in protocol and 'trigger-comparison' not in protocol:
                    break
        complete = len(all_rows)==sum(len(t['condition_order']) for t in manifest['trials'])
        decision = 'NO_GO' if stopped_scenarios else ('GO' if args.family == 'group' and complete else 'COMPLETED_PENDING_ANALYSIS')
        (args.out/'RUN_COMPLETE.json').write_text(json.dumps({'protocol_id':protocol,'decision':decision,'rows':len(all_rows),'stopped_scenarios':sorted(stopped_scenarios),'all_planned_conditions_run':complete},ensure_ascii=False,indent=2))
    except Exception as exc:
        (args.out/'BLOCKED.txt').write_text(str(exc)+'\n')
        raise
    finally:
        cleanup_trial()
        if broker_created:
            subprocess.run([DOCKER,'rm','-f',broker],stdout=subprocess.DEVNULL)


if __name__ == '__main__':
    main()
