"""新执行接纳分支单条件PX4/Gazebo开发验证；保持同一RA。"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from marllib.execution_admission import FixedRouteAdmissionCoordinator, FailedVehicleLoiterPilot
from marllib.phase5_runner import run_mqtt_loop
from marllib.run_c_recoverability_admission_gazebo import (
    _admission_config, _ra_kwargs, _sha256, _trajectory_audit, _published_command_audit_summary,
)
from swarm.recovery import RAOnlySafeHoldCoordinator


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--trial-id', required=True)
    p.add_argument('--condition', choices=['GEOMETRY','NOMINAL_DYNAMIC','HOLD'], required=True)
    args = p.parse_args()
    m = json.loads(args.manifest.read_text())
    if m['protocol_id'] not in ('aegisair-execution-admission-px4-development-v1','aegisair-execution-admission-px4-development-v2','aegisair-execution-admission-px4-qualification-v1'):
        p.error('只接受冻结的开发v1/v2或限定验证v1协议')
    if m.get('failed_vehicle_loiter') and m['protocol_id'] not in ('aegisair-execution-admission-px4-development-v2','aegisair-execution-admission-px4-qualification-v1'):
        p.error('悬停控制变量仅允许开发v2和限定验证v1协议')
    trial = next(t for t in m['trials'] if t['trial_id']==args.trial_id)
    if args.condition not in trial['condition_order']:
        p.error('条件不在冻结顺序中')
    g = m['geometries'][trial['geometry_id']]
    starts = {int(k):tuple(v) for k,v in g['reset_starts'].items()}
    goals = {int(k):tuple(v) for k,v in g['base_goals'].items()}
    config = _admission_config(m['recoverability_admission'], rate_hz=20,
                              execution_tau_s=.7, command_feedforward_tau_s=.7)
    coordinator = RAOnlySafeHoldCoordinator() if args.condition=='HOLD' else FixedRouteAdmissionCoordinator(
        config, waypoint_routes=g['waypoint_routes'], geometry_only=args.condition=='GEOMETRY')
    args.out_dir.mkdir(parents=True, exist_ok=False)
    trajectory = args.out_dir/'trajectory.jsonl'
    run = run_mqtt_loop(
        drone_ids=[2,3], base_goals=goals, reset_starts=starts, mode='CBF_ONLY',
        llm_client=None, llm_fallback=None, host='127.0.0.1', port=1883,
        max_steps=m['max_steps'], rate_hz=20, trajectory=trajectory,
        mission_change={'kind':'fail_drone','drone':2}, change_step=trial['change_step'],
        failed_drone=2, critical_goal=tuple(g['critical_goal']), goal_epsilon=.55,
        velocity_command_mode='feedforward_tau', tau_command_s=.7,
        recoverability_admission_coordinator=coordinator,
        pilot=FailedVehicleLoiterPilot() if m.get('failed_vehicle_loiter') else None,
        **_ra_kwargs(m['ra_config']))
    audit = _trajectory_audit(trajectory, failed_drone=2, healthy_drone=3,
                              change_step=trial['change_step'], critical_goal=tuple(g['critical_goal']), goal_epsilon=.55)
    row = {'trial_id':trial['trial_id'], 'condition':args.condition, 'seed':trial['seed'],
           'geometry_id':trial['geometry_id'], 'collision':run['collision'], 'min_rho':run['min_rho'],
           'min_distance_m':run['min_distance_m'], 'post_failure_critical_reached':audit['post_failure_critical_reached_by_healthy'],
           'infrastructure_valid':run['infrastructure_valid'],
           'infrastructure_invalid_reasons':run['infrastructure_invalid_reasons'],
           'admission':run.get('recoverability_admission'), 'audit':audit,
           'counters':run.get('counters'), 'safety_bypass_count':run['safety_bypass_count'],
           'ra_solve_latency_summary_ms':run['ra_solve_latency_summary_ms'],
           **_published_command_audit_summary(run),
           'trajectory_sha256':_sha256(trajectory)}
    summary = {'manifest_sha256':_sha256(args.manifest), 'protocol_id':m['protocol_id'], 'trials':[row]}
    (args.out_dir/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    (args.out_dir/'manifest.json').write_bytes(args.manifest.read_bytes())
    print(json.dumps(row,ensure_ascii=False),flush=True)


if __name__=='__main__':
    main()
