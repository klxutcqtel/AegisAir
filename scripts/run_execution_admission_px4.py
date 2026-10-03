"""持续监督新接纳分支PX4开发配对，自动清理自身栈并核验结果。"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
import shutil
from run_paper_step_response_rerun import ROOT, GATE, DOCKER, IMAGE, run
from admission_execution_study import external_root, digest


def validate_row(root, manifest_path, m, trial, condition):
    summary = json.loads((root/'summary.json').read_text())
    if summary['manifest_sha256'] != digest(manifest_path):
        raise ValueError('运行配置哈希不一致')
    row = summary['trials'][0]
    if (row['trial_id'],row['condition'],row['seed']) != (trial['trial_id'],condition,trial['seed']):
        raise ValueError('条件身份变化')
    if digest(root/'trajectory.jsonl') != row['trajectory_sha256']:
        raise ValueError('轨迹哈希不一致')
    trace = [json.loads(line) for line in (root/'trajectory.jsonl').read_text().splitlines()]
    if len(trace) != m['max_steps'] or min(r['min_rho'] for r in trace) != row['min_rho']:
        raise ValueError('轨迹horizon/最低裕度不一致')
    from marllib.run_c_recoverability_admission_gazebo import _trajectory_audit
    g = m['geometries'][trial['geometry_id']]
    audit = _trajectory_audit(root/'trajectory.jsonl', failed_drone=2, healthy_drone=3,
                              change_step=trial['change_step'], critical_goal=tuple(g['critical_goal']), goal_epsilon=.55)
    if audit != row['audit']:
        raise ValueError('逐步权限与任务审计不一致')
    return row


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, default=ROOT/'configs/admission_execution_px4_development_v1.json')
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    external_root(args.out)
    m = json.loads(args.manifest.read_text())
    args.out.mkdir(parents=True,exist_ok=False)
    manifest = args.out/'manifest.json'
    manifest.write_bytes(args.manifest.read_bytes())
    sources = [f for base in ('swarm','marllib','scripts') for f in (ROOT/base).rglob('*.py')]
    source_hashes = {str(f.relative_to(ROOT)):digest(f) for f in sources}
    for name in source_hashes:
        target=args.out/'source_snapshot'/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,target)
    (args.out/'source_hashes.json').write_text(json.dumps(source_hashes,indent=2))
    px4_root = GATE.parent/'PX4-Autopilot'
    runtime = [px4_root/'build/px4_sitl_default/bin/px4', GATE/'worlds/s1_single_obstacle.sdf', GATE/'scripts/launch_multi_sitl_pose.sh']
    (args.out/'runtime_hashes.json').write_text(json.dumps({str(f):digest(f) for f in runtime},indent=2))
    rows, attempts = [], []
    def save(state, **extra):
        (args.out/'results.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
        (args.out/'status.json').write_text(json.dumps({'state':state,'valid_conditions':len(rows),
            'attempts':attempts,'updated_unix_s':time.time(),**extra},ensure_ascii=False,indent=2))
    broker, adapter = 'aegisair-exec-admission-broker', 'aegisair-exec-admission-adapter'
    owned_broker = owned_adapter = False
    procs, streams = [], []
    def cleanup_trial(logs):
        nonlocal owned_adapter
        if owned_adapter:
            with (logs/'adapter_cleanup.log').open('a') as stream:
                subprocess.run([DOCKER,'logs',adapter],stdout=stream,stderr=subprocess.STDOUT,timeout=30)
                subprocess.run([DOCKER,'rm','-f',adapter],stdout=stream,stderr=subprocess.STDOUT,timeout=30)
            owned_adapter=False
        for proc in reversed(procs):
            try:
                os.killpg(proc.pid,signal.SIGTERM)
                proc.wait(timeout=10)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL)
                proc.wait(timeout=5)
            try:
                os.killpg(proc.pid,signal.SIGKILL)
            except ProcessLookupError:
                pass
        procs.clear()
        for stream in streams:
            stream.close()
        streams.clear()
    active_logs = args.out
    try:
        save('PREFLIGHT')
        if subprocess.run(['pgrep','-x','px4'],stdout=subprocess.DEVNULL).returncode==0:
            raise RuntimeError('已有PX4进程，不能混用')
        run([DOCKER,'image','inspect',IMAGE],args.out/'image.json')
        run([sys.executable,'-m','pip','freeze'],args.out/'pip-freeze.txt')
        run([DOCKER,'run','-d','--name',broker,'-p','127.0.0.1:1883:1883',
             'eclipse-mosquitto:2','mosquitto','-c','/mosquitto-no-auth.conf'],args.out/'broker_start.log')
        owned_broker=True
        for trial in m['trials']:
            for condition in trial['condition_order']:
                accepted=False
                for attempt in range(1,4):
                    label=f"{trial['trial_id']}_{condition}_attempt{attempt}"
                    logs=args.out/(label+'_logs')
                    logs.mkdir()
                    active_logs=logs
                    destination=args.out/label
                    save('RUNNING',label=label)
                    print('START '+label,flush=True)
                    try:
                        for drone in (2,3):
                            stream=(logs/f'heartbeat{drone}.log').open('x'); streams.append(stream)
                            procs.append(subprocess.Popen([sys.executable,str(GATE/'scripts/gcs_heartbeat.py'),
                                '--port',str(18570+drone),'--duration-s','600'],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True))
                        stream=(logs/'launch.log').open('x'); streams.append(stream)
                        launcher=subprocess.Popen(['bash',str(GATE/'scripts/launch_multi_sitl_pose.sh'),'S1','2,3'],cwd=ROOT,
                            env={**os.environ,'C3_GAZEBO_SEED':str(trial['seed']),'POSES':'2=0,-3,0.5;3=0,3,0.5'},
                            stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
                        procs.append(launcher)
                        deadline=time.monotonic()+120
                        while time.monotonic()<deadline:
                            if launcher.poll() is not None:
                                raise RuntimeError('SITL提前退出')
                            if 'state_dir=' in (logs/'launch.log').read_text():
                                break
                            time.sleep(1)
                        else:
                            raise RuntimeError('SITL启动超时')
                        run([DOCKER,'run','-d','--name',adapter,'--link',broker+':mqtt',
                            '-p','127.0.0.1:8889:8889/udp','-e','XRCE_UDP_PORT=8889','-e','ADAPTER_INSTANCES=2 3',
                            '-e','ORIGIN_OFFSET_2=-3,0,0','-e','ORIGIN_OFFSET_3=3,0,0',
                            '-e','ADAPTER_ARGS=--no-read-only --mqtt-host mqtt --mqtt-port 1883 --control-rate-hz 20 --telemetry-rate-hz 20',IMAGE],logs/'adapter_start.log')
                        owned_adapter=True
                        run([sys.executable,str(ROOT/'scripts/wait_for_px4_telemetry.py'),'--drone-ids','2','3','--timeout-s','90'],logs/'telemetry.log')
                        command=[sys.executable,str(ROOT/'marllib/run_execution_admission_px4.py'),
                            '--manifest',str(manifest),'--out-dir',str(destination),'--trial-id',trial['trial_id'],'--condition',condition]
                        (logs/'command.json').write_text(json.dumps(command))
                        run(command,logs/'runner.log',timeout=420)
                        row=validate_row(destination,manifest,m,trial,condition)
                        attempts.append({'label':label,'state':'VALID' if row['infrastructure_valid'] else 'INFRASTRUCTURE_INVALID',
                                         'summary':str(destination/'summary.json')})
                        if row['infrastructure_valid']:
                            rows.append(row)
                            accepted=True
                        else:
                            print('INVALID '+label+' '+str(row['infrastructure_invalid_reasons']),flush=True)
                    except Exception as exc:
                        # 仅无轨迹的预运行reset超时/遥测等待超时允许同条件重试。
                        text=(logs/'runner.log').read_text(errors='replace') if (logs/'runner.log').exists() else ''
                        pre_reset='PX4 velocity reset failed to settle within 45s' in text and not (destination/'trajectory.jsonl').exists()
                        telemetry_timeout='wait_for_px4_telemetry.py' in str(exc) and not destination.exists()
                        if pre_reset or telemetry_timeout:
                            attempts.append({'label':label,'state':'INFRASTRUCTURE_INVALID','reason':str(exc)})
                        else:
                            attempts.append({'label':label,'state':'NEEDS_DIAGNOSIS','reason':str(exc)})
                            raise
                    finally:
                        cleanup_trial(logs)
                        save('RUNNING',label=label)
                    if accepted:
                        print('DONE '+label,flush=True)
                        break
                if not accepted:
                    raise RuntimeError('连续三次基础设施无效；需诊断')
            pair=[r for r in rows if r['trial_id']==trial['trial_id']]
            dynamic=next(r for r in pair if r['condition']=='NOMINAL_DYNAMIC')
            counters=dynamic['counters'] or {}
            failures=[k for k,v in counters.items() if 'infeasible' in k and ('output' in k or 'selected' in k) and v]
            mission_failed = m.get('require_dynamic_turn_completion',False) and trial['geometry_id']=='turning' and (
                not dynamic['post_failure_critical_reached'] or dynamic['admission']['plans_committed']!=1 or dynamic['min_distance_m']<2.4)
            if dynamic['collision'] or dynamic['min_rho']<=0 or dynamic['safety_bypass_count'] or dynamic['published_command_constraint_unknown_count'] or failures or dynamic['audit']['selected_qp_infeasible_steps'] or mission_failed:
                save('STOP_VALID_NEGATIVE',trial_id=trial['trial_id'],counter_failures=failures)
                return
        if any(digest(ROOT/name)!=value for name,value in source_hashes.items()):
            raise RuntimeError('运行期间源码变化')
        save('COMPLETED')
    except BaseException as exc:
        save('PAUSED_FOR_DIAGNOSIS',error=str(exc))
        raise
    finally:
        cleanup_trial(active_logs)
        if owned_broker:
            subprocess.run([DOCKER,'rm','-f',broker],stdout=subprocess.DEVNULL,timeout=30)


if __name__=='__main__':
    main()
