"""只读复核开发/后续验证的完整配对、轨迹与结论；不重跑或删负结果。"""
import argparse
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np

from admission_execution_study import ROOT, digest, select, summarize, _segment_distance


def verify(root):
    manifest = json.loads((root/'manifest.json').read_text())
    status = json.loads((root/'status.json').read_text())
    if status['state'] != 'COMPLETED':
        raise ValueError('进程未完成')
    for name, value in json.loads((root/'integrity.json').read_text()).items():
        if digest(root/name) != value:
            raise ValueError('输出哈希变化: '+name)
    for name, value in json.loads((root/'source_hashes.json').read_text()).items():
        if digest(ROOT/name) != value:
            raise ValueError('原实现已变化: '+name)
    rows = [json.loads(line) for line in (root/'conditions.jsonl').read_text().splitlines()]
    expected = []
    geometries = {g['name']:g for g in manifest['geometries']}
    for geometry in manifest['geometries']:
        for speed in manifest['speeds_mps']:
            for heading in ([0] if speed == 0 else manifest['headings_deg']):
                case_id = f"{geometry['name']}_v{speed}_h{heading}"
                for plant in manifest['evaluation_plants']:
                    expected += [(case_id, plant['name'], m) for m in manifest['methods']]
    if [(r['case_id'],r['plant']['name'],r['method']) for r in rows] != expected:
        raise ValueError('条件顺序/身份/分母不完整')
    checked = {}
    selections = {}
    for row in rows:
        geometry = geometries[row['geometry']]
        routes = [[np.array(p) for p in route] for route in json.loads((root/(row['geometry']+'_routes.json')).read_text())]
        key = row['case_id'], row['method']
        if key not in selections:
            selections[key] = select(routes, row['velocity'], manifest, geometry, row['method'])
        if selections[key]['route_index'] != row['route_index'] or selections[key]['checks'] != row['checks']:
            raise ValueError('接纳决定不能由原模型重现')
        if len(row['library_outcomes']) != len(routes) or row['library_route_count'] != len(routes):
            raise ValueError('路线库裁判缺失')
        if row['library_executable'] != any(o['executable'] for o in row['library_outcomes']):
            raise ValueError('有限库标签不一致')
        for outcome in [*row['library_outcomes'],row['outcome']]:
            name = outcome['trajectory']
            if name in checked:
                if checked[name] != outcome:
                    raise ValueError('同轨迹摘要冲突')
                continue
            if digest(root/name) != outcome['trajectory_sha256']:
                raise ValueError('轨迹哈希变化')
            trace = [json.loads(line) for line in (root/name).read_text().splitlines()]
            if len(trace) != math.ceil(manifest['horizon_s']/manifest['control_dt_s']) or [r['step'] for r in trace] != list(range(len(trace))):
                raise ValueError('轨迹horizon/步号不完整')
            if min(r['min_distance_m'] for r in trace) != outcome['min_distance_m'] or min(r['min_rho'] for r in trace) != outcome['min_rho']:
                raise ValueError('最低裕度摘要不一致')
            if any(r['published_commands']['2'] != [0.,0.] for r in trace):
                raise ValueError('失效机权限未撤销')
            if any(abs(c)>1.5+1e-9 for r in trace for c in r['published_commands']['3']):
                raise ValueError('速度发布超限')
            if outcome['executable'] != (outcome['safe'] and outcome['completed']):
                raise ValueError('可执行标签矛盾')
            safe = outcome['min_distance_m'] >= manifest['physical_clearance_m'] and outcome['min_rho'] > 0 and outcome['selected_infeasible_steps']==0 and outcome['arena_ok']
            if outcome['safe'] != safe:
                raise ValueError('安全判据矛盾')
            checked[name] = outcome
    analysis = summarize(rows)
    if analysis != json.loads((root/'analysis.json').read_text()):
        raise ValueError('结论重算不一致')
    report = {'verified': True, 'protocol_id': manifest['protocol_id'], 'conditions':len(rows),
              'unique_trajectories':len(checked), 'manifest_sha256':digest(root/'manifest.json'),
              'analysis':analysis, 'scope':'简化模型有限候选库；不代表PX4、全局可恢复性或区间鲁棒证明'}
    path = root/'verified_analysis_v1.json'
    with path.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('root', type=Path)
    args = p.parse_args()
    result = verify(args.root)
    print(json.dumps({k:v for k,v in result.items() if k!='analysis'},ensure_ascii=False))
