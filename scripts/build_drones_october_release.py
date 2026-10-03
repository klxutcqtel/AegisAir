"""构建十月增补证据包；保留历史版本、负结果和逐文件哈希，不改实验原件。"""
from __future__ import annotations
import argparse
import json
import shutil
from pathlib import Path
from build_drones_public_release import selected_data, selected_source, write_zip, sha256, SOURCE_SUFFIXES

CAMPAIGNS = {
    'reserve-only': ('reserve_only_px4_rerun_v1_20261003',),
    'common-epoch': ('admission_common_epoch_v1_20261003',),
    'execution-admission': (
        'admission_execution_development_v1_20261003',
        'admission_execution_qualification_v1_20261003',
        'admission_execution_px4_development_v1_20261003',
        'admission_execution_px4_development_v2_20261003',
        'admission_execution_px4_qualification_v1_20261003',
    ),
    'prediction-audit': ('prediction_mechanism_audit_20261003', 'prediction_same_state_diagnosis_20261003_v1'),
}

def evidence(root, campaign):
    seen=set()
    for path, member in selected_data(root,campaign):
        if any(x in path.name for x in ('heartbeat_verification_', 'manuscript')):
            continue
        seen.add(path)
        yield path,member
    # 新实验的冻结源码与记录哈希一并公开；不靠发布时源码冒充历史实现。
    for path in sorted((root/campaign).rglob('*')):
        if not path.is_file() or path in seen or path.name.startswith('._'): continue
        rel=path.relative_to(root/campaign)
        if any('source_snapshot' in part for part in rel.parts) and path.suffix in SOURCE_SUFFIXES:
            yield path,Path('evidence')/campaign/rel

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root',type=Path,default=Path('/Volumes/Expansion/Aegis'))
    p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    args.out.mkdir(exist_ok=False,parents=True)
    entries=[];findings=[]
    for category,campaigns in CAMPAIGNS.items():
        write_zip(args.out/f'aegisair-{category}-evidence.zip',
                  (item for campaign in campaigns for item in evidence(args.data_root,campaign)),category,entries,findings)
    write_zip(args.out/'aegisair-source-snapshot.zip',selected_source(),'source',entries,findings)
    (args.out/'file_manifest.json').write_text(json.dumps({
        'schema':'aegisair-drones-october-evidence-v1',
        'license_for_evidence':'CC-BY-4.0',
        'previous_release':'https://github.com/klxutcqtel/AegisAir/releases/tag/drones-evidence-2026-09-23',
        'source_snapshot_note':'发布时源码和实验内冻结源码分别保留；每次运行哈希识别历史实现。',
        'files':entries},ensure_ascii=False,indent=2)+'\n')
    (args.out/'security_scan_findings.json').write_text(json.dumps(findings,indent=2)+'\n')
    if findings: raise RuntimeError(f'存在 {len(findings)} 个潜在敏感项，停止发布')
    (args.out/'README.md').write_text('''# AegisAir 十月增补证据包

本包补充论文 *Control-Authority Reserve and Selective Mission Recovery for Multi-UAV Runtime Assurance* 的无辅助预测实验及接纳输入对齐实验。
九月控制对照、消融、外部 PCBF、触发器、跟踪尺度和误差审计证据继续使用不可覆盖的历史发布：https://github.com/klxutcqtel/AegisAir/releases/tag/drones-evidence-2026-09-23 。两次发布共同构成论文证据。

## 内容

- reserve-only：30 对任务恢复、四机校准/资格/20 种子主实验、36 条件固定三路线试验，以及未对齐多航点接纳的有效开发拒绝及阻止依赖记录。
- common-epoch：输入对齐后的多航点接纳，开发 12、资格 15、主实验 60 条件；每阶段只依赖本次完整前置验收。
- execution-admission：固定候选库的离线和 PX4 开发/资格证据，保留其预测开关、不同协议与独立分母，不与无辅助预测结果混算。
- prediction-audit：105 条轨迹的触发机制审计、同状态接纳诊断；被引用的九月原始轨迹见历史发布。
- source-snapshot：发布时源码、配置、分析器与测试；实验根内的冻结源码另行保留。

## 校验

下载本发布全部附件到同一目录，运行 `python verify_release.py .`。SHA256SUMS 校验附件，file_manifest.json 校验每个 ZIP 成员。归档名和绝对路径只是历史来源标识，不要求核验机器具有作者路径。

## 解释边界

原种子和原场景复跑不是新的独立盲测。输入对齐采用匀速模型，不是连续真值。固定库只覆盖预定义转弯及控制场景。算法有效负结果完整保留；启动、遥测或存储异常按原证据单独记录，不作为算法失败或成功。摘要、轨迹、配置、哈希、诊断、冻结源码与独立核验收据保留。大体积容器/PX4/MQTT 文本日志和多数运行侧文件不收录。部分历史分析器包含本机绝对路径，重算须重定位；本包不承诺逐位复原历史运行环境。

数据采用 CC BY 4.0，引用 Jiajun Li、论文标题及此发布标签。源码沿用仓库既有许可状态，此声明不另行授予软件许可。论文正文、投稿包、密钥与模型 checkpoint 不在此发布。
''')
    (args.out/'LICENSE_DATA.md').write_text('证据归档、文件清单和发布元数据采用 CC BY 4.0：https://creativecommons.org/licenses/by/4.0/ 。署名 Jiajun Li、论文标题及发布标签。本声明不重新许可第三方组件或源码。\n')
    shutil.copyfile(Path(__file__).with_name('verify_drones_public_release.py'),args.out/'verify_release.py')
    names=['README.md','LICENSE_DATA.md','file_manifest.json','verify_release.py']+[x.name for x in sorted(args.out.glob('*.zip'))]
    (args.out/'SHA256SUMS').write_text(''.join(f'{sha256(args.out/name)}  {name}\n' for name in names))
    print(f'完成：{len(entries)} 文件；潜在敏感项 0',flush=True)
if __name__=='__main__':main()
