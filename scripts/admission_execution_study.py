"""固定路线的执行约束开发实验：筛查与RA闭环裁判分离，输出只允许真实外置盘。"""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm.recovery.recoverability_admission import (
    RecoverabilityAdmissionConfig, RecoverabilityAdmissionCoordinator, _segment_distance,
)
from swarm.ra.runtime_assurance import RuntimeAssurance
from swarm.ra.margins import RuntimeAssuranceParams
from swarm.safety import DroneSnapshot


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def config_for(manifest, plant):
    return RecoverabilityAdmissionConfig(
        clearance_m=manifest['physical_clearance_m'],
        tracking_error_buffer_m=manifest['screen_clearance_m']-manifest['physical_clearance_m'],
        arc_extra_m=(1.4,), arc_segments=(5, 8), arena=tuple(manifest['arena']),
        rollout_horizon_s=manifest['horizon_s'], max_route_length_m=manifest['route_length_limit_m'],
        execution_tau_s=plant['tau_s'], reaction_delay_s=plant['delay_s'],
        healthy_acceleration_limit_mps2=plant['accel_mps2'],
        braking_accel_mps2=plant['accel_mps2'], braking_accel_uncertainty_mps2=0.0,
        dt_s=manifest['control_dt_s'], command_feedforward_tau_s=0.7,
    )


def failed_snapshot(geometry):
    return DroneSnapshot(2, (*geometry['failed_position'], 2.5),
                         velocity=(*geometry['failed_velocity'], 0.0))


def candidates(manifest, geometry):
    c = RecoverabilityAdmissionCoordinator(config_for(manifest, manifest['nominal']))
    obstacle = c._failed_trajectory(failed_snapshot(geometry))
    ranked = []
    for route in c._candidate_routes(np.array(geometry['start']), np.array(geometry['goal']), obstacle):
        clearance = c._route_clearance(route, obstacle)
        length = c._route_length(route)
        if clearance+1e-9 >= c.config.effective_clearance_m and length <= c.config.max_route_length_m:
            ranked.append((length, -clearance, route))
    ranked.sort(key=lambda r: (r[0], r[1]))
    # 同一位置的所有速度与所有方法复用此固定库，不按结局重新生成路线。
    return [route for _, _, route in ranked[:manifest['candidate_limit']]]


def screen(route, velocity, manifest, geometry, plant):
    c = RecoverabilityAdmissionCoordinator(config_for(manifest, plant))
    failed = c._failed_trajectory(failed_snapshot(geometry))
    return asdict(c._rollout_route(points=route, initial_velocity=np.array(velocity), failed_trajectory=failed))


def select(routes, velocity, manifest, geometry, method):
    started = time.perf_counter()
    details = []
    chosen = None
    if method == 'geometry':
        chosen = 0 if routes else None
    elif method != 'hold':
        models = [manifest['nominal']]
        if method == 'scenario_dynamic':
            box = manifest['screen_box']
            models += [dict(zip(('tau_s', 'delay_s', 'accel_mps2'), values))
                       for values in itertools.product(box['tau_s'], box['delay_s'], box['accel_mps2'])]
        for index, route in enumerate(routes):
            checks = [screen(route, velocity, manifest, geometry, plant) for plant in models]
            details.append({'route_index': index, 'checks': checks})
            if all(check['feasible'] for check in checks):
                chosen = index
                break
    return {'route_index': chosen, 'latency_ms': (time.perf_counter()-started)*1000, 'checks': details}


def advance(position, velocity, command, plant, dt):
    """每分量饱和的一阶响应，恒定命令下精确积分至饱和解除时刻。"""
    delta = command-velocity
    accel = plant['accel_mps2']
    tau = plant['tau_s']
    new_position = position.copy()
    new_velocity = velocity.copy()
    for axis in range(2):
        error = float(delta[axis])
        sign = 1.0 if error >= 0 else -1.0
        linear_time = max(0.0, (abs(error)-accel*tau)/accel)
        linear_dt = min(dt, linear_time)
        v_mid = velocity[axis]+sign*accel*linear_dt
        new_position[axis] += velocity[axis]*linear_dt+0.5*sign*accel*linear_dt**2
        remaining = dt-linear_dt
        alpha = -math.expm1(-remaining/tau)
        new_position[axis] += command[axis]*remaining+(v_mid-command[axis])*tau*alpha
        new_velocity[axis] = v_mid+(command[axis]-v_mid)*alpha
    return new_position, new_velocity


def referee(route, initial_velocity, manifest, geometry, plant, *, capture=False):
    """独立实际命令闭环；无调用接纳rollout，不是PX4或实机。"""
    ra = RuntimeAssurance(
        params=RuntimeAssuranceParams(tau_ctrl=.2, degradation_dt=.05),
        v_max=1.5, a_max=2.0, use_hocbf=True, hocbf_k1=4, hocbf_k2=4,
        tau_px4=.7, command_feedforward_tau_s=.7,
        hocbf_boundary_guard=1, hocbf_boundary_buffer_m=.3,
        hocbf_infeasible_fallback='max_brake', hocbf_pb_recovery=True,
        hocbf_predictive_recovery=True, hocbf_prediction_execution_fraction=0,
        hocbf_prediction_steps=3, hocbf_recovery_reserve_threshold=1,
        hocbf_recovery_alpha=.5, hocbf_recovery_braking_accel=2,
        hocbf_recovery_boundary_buffer_m=.3, hocbf_recovery_clear_steps=5,
    )
    positions = {2: np.array(geometry['failed_position'], dtype=float), 3: np.array(geometry['start'], dtype=float)}
    velocities = {2: np.array(geometry['failed_velocity'], dtype=float), 3: np.array(initial_velocity, dtype=float)}
    initial = {i: velocities[i].copy() for i in positions}
    # 故障前发布命令统一设为初速度；故障时零命令也受同一传输延迟。
    history = []
    target_index = 1
    minimum = float(np.linalg.norm(positions[3]-positions[2]))
    min_rho = math.inf
    infeasible = published_failed = 0
    completed = False
    completion_time = None
    arena_ok = True
    effort = 0.0
    rows = []
    dt = manifest['control_dt_s']
    sub_dt = manifest['plant_dt_s']
    nsub = round(dt/sub_dt)
    if not math.isclose(nsub*sub_dt, dt, abs_tol=1e-12):
        raise ValueError('积分步长必须整除控制周期')
    delay_cycles = round(plant['delay_s']/dt)
    if not math.isclose(delay_cycles*dt, plant['delay_s'], abs_tol=1e-12):
        raise ValueError('本版延迟必须是控制周期整数倍')
    for step in range(math.ceil(manifest['horizon_s']/dt)):
        if route is not None:
            while target_index < len(route)-1 and np.linalg.norm(route[target_index]-positions[3]) <= .55:
                target_index += 1
            target = route[target_index]
            nominal = np.clip(1.5*(target-positions[3]), -1.5, 1.5)
        else:
            nominal = np.zeros(2)
        snapshots = {i: DroneSnapshot(i, (*positions[i], 2.5), velocity=(*velocities[i], 0.0)) for i in positions}
        result = ra.filter(snapshots, {2: np.zeros(2), 3: nominal}, t=step*dt,
                           fixed_actions={2: np.zeros(2)})
        command = {i: np.array(result[i].safe_action) for i in positions}
        history.append(command)
        applied = history[step-delay_cycles] if step >= delay_cycles else initial
        min_rho = min(min_rho, result[3].safety_margin)
        infeasible += int(result[3].feasible is False)
        audit, _ = ra.audit_published_accelerations({i: (command[i]-velocities[i])/.7 for i in positions})
        published_failed += int(audit is False)
        effort += np.linalg.norm(command[3]-nominal)*dt
        for _ in range(nsub):
            relative_start = positions[3]-positions[2]
            for i in positions:
                positions[i], velocities[i] = advance(positions[i], velocities[i], applied[i], plant, sub_dt)
            relative_end = positions[3]-positions[2]
            minimum = min(minimum, _segment_distance(np.zeros(2), relative_start, relative_end))
            x0, x1, y0, y1 = manifest['arena']
            arena_ok &= bool(x0 <= positions[3][0] <= x1 and y0 <= positions[3][1] <= y1)
        if capture:
            rows.append({'step': step, 'positions': {str(i): p.tolist() for i, p in positions.items()},
                         'velocities': {str(i): v.tolist() for i, v in velocities.items()},
                         'published_commands': {str(i): c.tolist() for i, c in command.items()},
                         'min_distance_m': minimum, 'min_rho': min_rho})
        if route is not None and target_index == len(route)-1 and np.linalg.norm(route[-1]-positions[3]) <= .55 and np.linalg.norm(velocities[3]) <= .15:
            completed = True
            if completion_time is None:
                completion_time = (step+1)*dt
            # 已完成仍继续监督到共同horizon，防止停止瞬间隐藏后续失效。
    safe = bool(minimum >= manifest['physical_clearance_m'] and min_rho > 0 and infeasible == 0 and arena_ok)
    return {'completed': completed, 'safe': safe, 'executable': completed and safe,
            'min_distance_m': minimum, 'min_rho': min_rho, 'selected_infeasible_steps': infeasible,
            'published_constraint_failure_steps': published_failed, 'arena_ok': arena_ok,
            'completion_time_s': completion_time, 'command_correction_integral': float(effort), 'trace': rows}


def external_root(path):
    volume = Path('/Volumes/Expansion')
    if not volume.is_mount() or not path.resolve().is_relative_to(volume.resolve()):
        raise ValueError('仅允许向已真实挂载的 /Volumes/Expansion 写入数据')


def summarize(rows):
    report = {}
    for method in ('geometry', 'nominal_dynamic', 'scenario_dynamic', 'hold'):
        subset = [r for r in rows if r['method']==method]
        admitted = [r for r in subset if r['route_index'] is not None]
        rejected = [r for r in subset if r['route_index'] is None]
        report[method] = {
            'conditions': len(subset), 'admitted': len(admitted),
            'successful_admissions': sum(r['outcome']['executable'] for r in admitted),
            'false_admissions': sum(not r['outcome']['executable'] for r in admitted),
            'false_rejections_within_library': sum(r['library_executable'] for r in rejected),
            'correct_rejections_within_library': sum(not r['library_executable'] for r in rejected),
            'unsafe_hold_conditions': sum(not r['outcome']['safe'] for r in rejected),
            'latency_p99_ms': float(np.percentile([r['latency_ms'] for r in subset], 99)) if subset else None,
        }
    # 开发GO只是值得后续验证，不是方法已获独立确认。
    g, n, s = (report[k] for k in ('geometry', 'nominal_dynamic', 'scenario_dynamic'))
    report['development_gate'] = bool(
        (n['successful_admissions'] or s['successful_admissions']) and
        (n['correct_rejections_within_library'] or s['correct_rejections_within_library']) and
        min(n['false_admissions'], s['false_admissions']) < g['false_admissions'])
    return report


def run(manifest_path, out):
    external_root(out)
    manifest = json.loads(manifest_path.read_text())
    out.mkdir(parents=True, exist_ok=False)
    (out/'manifest.json').write_bytes(manifest_path.read_bytes())
    source_paths = [Path(__file__), ROOT/'swarm/recovery/recoverability_admission.py']+list((ROOT/'swarm/ra').glob('*.py'))
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in source_paths}
    (out/'source_hashes.json').write_text(json.dumps(hashes, indent=2))
    rows = []
    total_cases = len(manifest['geometries'])*(1+(len(manifest['speeds_mps'])-1)*len(manifest['headings_deg']))
    def save(state, **extra):
        (out/'status.json').write_text(json.dumps({'state': state, 'conditions': len(rows), 'expected_cases': total_cases,
                                                'updated_unix_s': time.time(), **extra}, ensure_ascii=False, indent=2))
    try:
        case_number = 0
        trace_dir = out/'trajectories'
        trace_dir.mkdir()
        def evaluate(route, velocity, geometry, plant, label):
            outcome = referee(route, velocity, manifest, geometry, plant, capture=True)
            trace = outcome.pop('trace')
            path = trace_dir/(label+'.jsonl')
            with path.open('x') as log:
                for record in trace:
                    log.write(json.dumps(record, allow_nan=False)+'\n')
            outcome['trajectory'] = str(path.relative_to(out))
            outcome['trajectory_sha256'] = digest(path)
            return outcome
        with (out/'conditions.jsonl').open('x') as stream:
            for geometry in manifest['geometries']:
                routes = candidates(manifest, geometry)
                (out/(geometry['name']+'_routes.json')).write_text(json.dumps([[p.tolist() for p in r] for r in routes]))
                for speed in manifest['speeds_mps']:
                    for heading in ([0] if speed==0 else manifest['headings_deg']):
                        case_number += 1
                        case_id = f"{geometry['name']}_v{speed}_h{heading}"
                        angle = math.radians(heading)
                        velocity = [speed*math.cos(angle), speed*math.sin(angle)]
                        save('RUNNING', case_id=case_id, case_number=case_number)
                        selections = {m: select(routes, velocity, manifest, geometry, m) for m in manifest['methods']}
                        for plant in manifest['evaluation_plants']:
                            outcomes = [evaluate(r, velocity, geometry, plant, case_id+'_'+plant['name']+f'_route{i}') for i, r in enumerate(routes)]
                            hold = evaluate(None, velocity, geometry, plant, case_id+'_'+plant['name']+'_hold')
                            oracle = any(o['executable'] for o in outcomes)
                            for method, selection in selections.items():
                                index = selection['route_index']
                                row = {'case_id': case_id, 'geometry': geometry['name'], 'velocity': velocity,
                                       'plant': plant, 'method': method, **selection,
                                       'library_executable': oracle, 'library_route_count': len(routes),
                                       'library_outcomes': outcomes,
                                       'outcome': hold if index is None else outcomes[index]}
                                rows.append(row)
                                stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
                            stream.flush()
                        print(f'DONE {case_number}/{total_cases} {case_id}', flush=True)
        if any(digest(ROOT/name)!=value for name, value in hashes.items()):
            raise ValueError('运行中源码变化，不允许混合结论')
        analysis = summarize(rows)
        (out/'analysis.json').write_text(json.dumps(analysis, ensure_ascii=False, indent=2))
        (out/'integrity.json').write_text(json.dumps({p.name:digest(p) for p in out.iterdir() if p.is_file() and p.name not in ('status.json','integrity.json')}, indent=2))
        save('COMPLETED', development_gate=analysis['development_gate'])
        print(json.dumps(analysis, ensure_ascii=False), flush=True)
    except BaseException as exc:
        save('FAILED', error=str(exc))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=ROOT/'configs/admission_execution_development_v1.json')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    run(args.manifest, args.out)
