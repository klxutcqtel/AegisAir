# AegisAir

AegisAir 面向多无人机系统的运行时安全保障、任务恢复与选择性接纳。上游学习或规则组件只提出任务与动作意图，独立 RA 层负责最终模型约束过滤；仿真中观察到的正裕量不等同于真实飞行或连续时间安全证明。

当前论文选用 Reserve-Only：基于控制权限余量触发并锁存 PB-CBF 恢复，关闭辅助三步主约束预测。PB 停止距离项与接纳动态 rollout 仍保留。多航点接纳将遥测位置按匀速模型对齐共同控制时刻；固定三路线对照单独评估执行状态对路线选择的作用。

## 最小检查

```bash
conda env create -f environment.yml
conda activate eai-swarm
pip install -r requirements.txt
python -m unittest discover -s tests
python scripts/validate_pcbf_baseline.py
python scripts/validate_dynamic_admission.py
```

离线测试检查 barrier/QP、失败回退、权限与消息契约、接纳输入对齐及实验调度规则，不需要历史轨迹、GPU 或 PX4。PCBF 验证需要 CasADi/IPOPT。动态接纳验证只检验核心模型，不能替代 PX4/Gazebo 闭环验收。

## 目录

- `swarm/`：安全几何、RA、恢复与接口。
- `marllib/`：环境、控制与实验 runner。
- `px4_adapter/`：PX4、Gazebo、ROS 2、MQTT 适配。
- `configs/`：冻结设计和参数；`reserve_only_selected_method_v1.json` 说明选定配置与证据归属。
- `schemas/`、`examples/`：接口定义与示例。
- `tests/`：离线单元与协议测试。
- `scripts/`：分析、校验与受控调度；部分历史调度器包含作者机器路径，不能直接当作通用启动器。
- `docs/`：协议、复现说明与诊断记录。

## 数据和闭环复现

Git 源码不包含论文、投稿文件、checkpoint 或实验产物。可公开的轨迹、摘要、冻结配置及哈希通过 [数据 Release](DATA_RELEASE.md) 单独提供；历史与新增证据保持各自版本和分母。

PX4/Gazebo 闭环需要另行准备仿真、ROS 2、MQTT 与适配桥。见 [闭环复现说明](docs/PX4_GAZEBO_REPRODUCTION.md) 和 [可复现性说明](REPRODUCIBILITY.md)。复跑使用新输出目录；有效负结果保留，基础设施无效尝试单独诊断，不能改变门限或种子救结果。
