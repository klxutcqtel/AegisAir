# PX4/Gazebo 闭环复现

本说明给出基础环境与单条件运行入口。批调度器中仍有机器专用路径；跨机器运行需按实际环境重定位，不能将启动成功视为完整复现。

## 前提

在 Linux 或能运行 PX4 SITL/Gazebo 的主机上，准备 Docker Compose v2、PX4-Autopilot **v1.16.0** 源码及其 Gazebo SITL 依赖，以及 Python 3.11 和本仓库的 `requirements.txt`。

```bash
git clone --recursive --branch v1.16.0 https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash Tools/setup/ubuntu.sh
make px4_sitl
export PX4_DIR="$PWD"
```

macOS 可用于离线测试；闭环建议使用 Linux，因为 PX4/Gazebo 的图形和 UDP/DDS 网络链路依赖主机环境。

## 启动 bridge

```bash
docker compose up --build -d
docker compose logs -f bridge
```

Compose 会启动 MQTT broker 与 ROS 2 bridge。bridge 镜像从公开固定提交构建 Micro-XRCE-DDS-Agent 和 `px4_msgs`，并把 UDP 8889 暴露给主机 PX4 SITL。

## 运行一个冻结条件

```bash
export PX4_DIR=/absolute/path/to/PX4-Autopilot
bash scripts/run_c1_paper_trial.sh \
  configs/c1_hocbf_v4_px4_validation_v1.json c1v4_01 AEGIS_HOCBF_V4 \
  /absolute/path/to/new-output
```

该脚本启动两机 S1 世界、发送 GCS heartbeat、等待 MQTT 遥测，再运行对应 runner；结束时只停止它启动的 PID。完整试验需要对冻结 manifest 内每个 `trial_id` 与 `condition_order` 重复调用，并保留每次输出。

以下为保留的 calibration-v4 历史协议示例（不是当前论文选定配置）：

```bash
export PX4_DIR=/absolute/path/to/PX4-Autopilot
bash scripts/run_recoverability_admission_trial.sh \
  configs/c_recoverability_admission_calibration_v4.json \
  radm4_lateral_dev RECOVERABILITY_ADMISSION_RA \
  /absolute/path/to/new-output
```

当前多航点接纳使用 `dynamic_admission_v6_common_epoch_v1`，完整阶段设置见十月 Release 中 `admission_common_epoch_v1_20261003/*/run_settings.json`。同一单条件入口可读取其完整 manifest；必须使用其中实际 trial_id/condition 和新的输出目录。先完成本次开发与分析，再按其 GO 启动资格和主实验，不借用历史 GO。`scripts/run_admission_common_epoch_campaign.py` 保留作者的 `/Volumes/Expansion/Aegis` 父目录和历史配置依赖，属于实验调度记录，不能不作路径适配直接运行。固定三路线分支使用 `marllib/run_execution_admission_px4.py`，与多航点 gate 分开。

## 边界

Git 源码不含轨迹、日志或 checkpoint；入选历史证据通过 DATA_RELEASE.md 指向的 Release 提供。重新运行得到的是新的复现实验，不能替代论文中封存的数值。
