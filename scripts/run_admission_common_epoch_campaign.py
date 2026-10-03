#!/usr/bin/env python3
"""共同时间戳接纳修复的新版本验证；保留历史有效拒绝，不调原门限。"""
import argparse,json,hashlib,shutil,sys,time,fcntl,os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from run_admission_corrected_v2 import run,verify_trace
from analyze_corrected_admission import analyze_root
from run_reserve_only_campaign import verify_no_prediction
BASE=Path('/Volumes/Expansion/Aegis')
REF=BASE/'reserve_only_px4_rerun_v1_20261003'
STAGES=['admission_calibration','admission_qualification','admission_main']
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def sources():return {str(p.relative_to(ROOT)):digest(p) for base in ['swarm','marllib','scripts'] for p in (ROOT/base).rglob('*.py')}
def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--resume',action='store_true');a=p.parse_args();out=a.out
 if out.parent.resolve()!=BASE.resolve():raise ValueError('实验数据必须在外置盘')
 if not a.resume:
  out.mkdir(exist_ok=False)
 lock=(out/'campaign.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 write(out/'process.json',{'pid':os.getpid(),'started_unix':time.time()})
 if not a.resume:
  configs={}
  for stage in STAGES:
   settings=json.loads((REF/'frozen_configs'/f'{stage}.json').read_text())
   settings.update(revision_id='admission-common-epoch-v1',implementation_version='dynamic_admission_v6_common_epoch_v1',recoverability_admission_align_states=True,rerun_scope='输入时间戳对齐修复的新版本；原种子原几何原门限，不是新盲测。')
   configs[stage]=settings;(out/stage).mkdir();write(out/stage/'run_settings.json',settings)
  hashes=sources();write(out/'source_hashes.json',hashes)
  for name in hashes:
   dest=out/'source_snapshot'/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,dest)
  plan={'revision_id':'admission-common-epoch-v1','frozen_settings_sha256':{stage:digest(out/stage/'run_settings.json') for stage in STAGES},'planned_conditions':87,'source_snapshot':'source_snapshot','fix':'仅接纳输入从异步测量时刻对齐到当前控制周期，预测关闭，候选库和所有门限保持不变。','stopping_rule':'有效负结果完成当前配对后停止依赖；只重试基础设施无效条件。','historical_negative':str(REF/'admission_calibration'),'prerequisites':'资格仅依赖本次完整开发GO；主实验仅依赖本次完整资格GO。'}
  write(out/'frozen_plan.json',plan)
 plan=json.loads((out/'frozen_plan.json').read_text());frozen=json.loads((out/'source_hashes.json').read_text())
 if sources()!=frozen:raise ValueError('冻结后源码变化，禁止混合实现')
 decisions={};prerequisite=None
 for stage in STAGES:
  root=out/stage
  if digest(root/'run_settings.json')!=plan['frozen_settings_sha256'][stage]:raise ValueError('冻结配置变化')
  settings=json.loads((root/'run_settings.json').read_text())
  if prerequisite:
   previous=prerequisite.parent;status=json.loads((previous/'status.json').read_text());audit=json.loads(prerequisite.read_text())
   if status['state']!='COMPLETED' or audit['decision']!='GO':
    decisions[stage]='BLOCKED_BY_VALID_PARENT_RESULT';continue
   write(root/'prerequisite_identity.json',{'path':str(prerequisite),'sha256':digest(prerequisite),'parent_settings_sha256':digest(previous/'run_settings.json')})
  elif stage!=STAGES[0]:
   decisions[stage]='BLOCKED_BY_VALID_PARENT_RESULT';continue
  status_path=root/'status.json';rows=[];attempts=[]
  if status_path.exists():
   status=json.loads(status_path.read_text());rows=json.loads((root/'results.json').read_text());attempts=status['attempts']
   if status['state'] not in ['COMPLETED','STOP_ADMISSION_DIAGNOSIS','STOP_INTEGRITY_DIAGNOSIS','PAUSED_FOR_DIAGNOSIS']:raise ValueError('非终态必须先诊断，禁止自动重跑')
   for row in rows:
    matches=[x for x in attempts if x['state']=='VALID' and x['label'].startswith(row['trial_id']+'_'+row['condition']+'_attempt')]
    assert len(matches)==1;verify_trace(Path(matches[0]['summary']),row,settings)
   if status['state']=='PAUSED_FOR_DIAGNOSIS' and any(x['state']=='NEEDS_DIAGNOSIS' for x in attempts):raise ValueError('存在未分类条件，必须诊断')
  else:write(root/'source_hashes.json',frozen)
  if not status_path.exists() or json.loads(status_path.read_text())['state']=='PAUSED_FOR_DIAGNOSIS':
   write(out/'campaign_status.json',{'state':'RUNNING','stage':stage,'decisions':decisions,'updated_unix':time.time()})
   run(root,settings,rows,attempts,prerequisite)
  if sources()!=frozen:raise ValueError('阶段运行中源码变化')
  status=json.loads(status_path.read_text());decisions[stage]=status['state']
  report=analyze_root(root);write(root/'verified_analysis.json',report);write(root/'prediction_absence_audit.json',verify_no_prediction(root))
  # 每个有效周期必须记录对齐输入；预测开关保持关闭。
  for row in json.loads((root/'results.json').read_text()):
   match=next(x for x in json.loads(status_path.read_text())['attempts'] if x['state']=='VALID' and x['label'].startswith(row['trial_id']+'_'+row['condition']+'_attempt'))
   path=Path(match['summary']).parent/row['trajectory']
   for line in path.read_text().splitlines():
    rec=json.loads(line)
    if row['condition']=='RECOVERABILITY_ADMISSION_RA':
     alignment=rec.get('recoverability_admission_alignment');assert alignment and alignment['method']=='constant_velocity_to_control_epoch'
     for key,d in rec['drones'].items():
      expected=[v+u*alignment['ages_s'][key] for v,u in zip(d['pos'],d['v_actual'])]
      assert max(abs(x-y) for x,y in zip(expected,alignment['positions'][key]))<1e-9
  prerequisite=root/'audit.json'
  if status['state']!='COMPLETED':prerequisite=None
 write(out/'campaign_status.json',{'state':'EXPERIMENTS_FINISHED','decisions':decisions,'manuscript_update_pending':True,'updated_unix':time.time()})
 print('FINISHED',decisions,flush=True)
if __name__=='__main__':main()
