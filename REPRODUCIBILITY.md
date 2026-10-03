# AegisAir 可复现性说明

## 源码与公开证据

Git 仓库提供 RA、恢复、接口、配置、离线测试和分析脚本。论文、投稿材料、checkpoint 和实验产物不进入 Git。九月与十月证据包见 [DATA_RELEASE.md](DATA_RELEASE.md)，提供入选轨迹、摘要、运行设置、有效性记录、哈希及源码快照。

发布时源码不能替代每次实验记录的执行哈希；十月归档还保留实验内冻结源码。代码检查、历史数据复算与重新运行闭环是不同层次的复现。部分分析器含历史绝对路径，解压后需重定位；归档校验本身只使用相对路径。

## 离线检查

```bash
conda env create -f environment.yml
conda activate eai-swarm
pip install -r requirements.txt
python -m unittest discover -s tests
python scripts/validate_pcbf_baseline.py
python scripts/validate_dynamic_admission.py
```

单元测试不依赖硬件。PCBF 数值验证需 CasADi/IPOPT；动态接纳验证只覆盖核心 rollout，不验证遥测、执行器或飞行性能。

## 当前配置

- `configs/reserve_only_selected_method_v1.json`：关闭辅助三步预测，保留 PB-CBF、余量触发与恢复锁存。
- `configs/admission_common_epoch_revision_v1.json`：多航点接纳输入按匀速模型对齐共同控制时刻；对应协议 `dynamic_admission_v6_common_epoch_v1`。归档内各阶段 run_settings.json 才是本次实际完整运行设置。
- 固定候选库执行接纳使用独立 runner 和冻结协议；不能将其三条路线与多航点 101 候选混为一个实验。
- 历史 v4/v5/v6 协议继续用于识别其原始数据；不能替换协议标识来冒充另一实现。

资格只依赖同次完整开发通过，主实验只依赖同次完整资格通过。已完成、有效的失败不重跑；基础设施异常须保留原尝试并依据日志诊断。沿用原种子和场景的复跑不是新增独立验证。

## 闭环运行与边界

环境与单条件入口见 [docs/PX4_GAZEBO_REPRODUCTION.md](docs/PX4_GAZEBO_REPRODUCTION.md)。部分批调度器固定了作者的外置盘路径、容器名与历史父目录，公开用于检查实验组织逻辑；不保证在任意机器直接运行。跨机器复跑应使用对应完整设置和全新输出根，记录本机环境，保留每个有效结果及无效尝试。

归档省略大体积容器/PX4/MQTT 文本日志和多数运行侧文件，因此不完整重建历史环境。正采样裕量、零模拟碰撞和输入存在性命题均不能替代真实飞行验证或每条发布命令的约束证明。
