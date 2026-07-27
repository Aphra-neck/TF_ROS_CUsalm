# YOPO 实机集成版本清单

基线标识：`yopo_platform_baseline_20260723_v2`

状态：`PASSED`。localization-runtime、YOPO runtime 和宿主 MAVROS/PX4 三层
版本证据已采集并核对。本结论只通过 `YP-020` 版本门禁，不授权控制输出、
OFFBOARD、解锁或飞行。

历史证据来源：`agent_merged_20260721.md`，SHA-256
`b265514e9e92993cee81b32189778a74cfd7dac5587bb52886a891ce79752fa5`。
该文件包含 2026-07-16 至 2026-07-21 的设备与联合运行记录。本文只继承其中有
明确命令输出或验收记录的事实；旧任务状态和被后续提交替代的设计不继续生效。

## 1. 仓库基线

| 组件 | 分支或版本 | Commit | 来源 | 状态 |
| --- | --- | --- | --- | --- |
| `TF_ROS_CUsalm` | `feat/yopo-integration-v1` | 首次实机采集 `ac69b196e1c098a5fbcb8e0076ce2d2966359496`；分支起点 `5f03815d840513799908559eb427077155079c6e` | `https://github.com/u5-4/TF_ROS_CUsalm.git` | Jetson 已验证 clean |
| `YOPO_ROS2` | Jetson `ros2-humble`；Windows 审计分支 `jetson-passive-deployment` | `80c0569e4d1ed8ed8c885bc7df200b18b9881088` | `https://github.com/u5-4/YOPO_ROS2.git` | Jetson 已验证 clean，ROS packages 已安装 |
| cuVSLAM 定制与 bringup | `u5-4/fcu-imu-cuvslam-integration` | 平台基线 `04d7b9cd1a4f6ebe324538a3892d32ccc794e650`；YP-100 Depth `2584606` | `https://github.com/u5-4/cuVSLAM.git` | `2584606` 已在 Jetson 构建；73/73 package tests 通过 |
| `isaac_ros_visual_slam` | package `3.2.6` | `e31f4cc1d41a329a01946e5fe63669f8b15da677` | NVIDIA Isaac ROS 3.2.15 审计副本 | Jetson commit 已验证；预期 wrapper patch 已应用，待 patch verifier 复核 |
| `vrpn_client_ros` | package `0.2.2` | `1b9731c055c08d8496897108580534a80da0b158` | `https://github.com/u5-4/vrpn_client_ros2.git` | Jetson 已验证 clean |

`isaac_ros_yopo_bringup` package 版本为 `0.2.0`，位于 cuVSLAM 定制仓库的
`integrations/isaac_ros_3_2_yopo/` 下。表中的本地审计 commit 不能替代 Jetson
实际工作区 commit；两者必须完全一致或明确记录差异。

YOPO 本体已经部署和构建。当前尚未解决的是历史 SO3 controller 到 ROS 2 Humble、
实机状态反馈和可替换 MAVROS backend 的迁移；该工作属于 `YP-400/410`，不是
`YP-020` 的版本采集缺口，也不是重新部署 YOPO 的理由。

## 2. 固定文件与模型

| 文件 | SHA-256 |
| --- | --- |
| `YOPO/saved/YOPO_1/epoch50.pth` | `09ec31094ed83e09702efc1facf18076564ebbe1afc164960c41036e16ba229f` |
| `YOPO/config/traj_opt.yaml` | `c8dae0ea9b21c60de9abce85e3ceecfbf0e3cf0ef56a5a8afab9743e788dc21f` |
| `YOPO/requirements.txt` | `e5e8c547e3a038f74e9f182fc9518319bf46ba4b9fb7463b633cff3cf131f07f` |
| `isaac_ros_yopo_bringup/launch/d435i_fcu_imu_cuvslam.launch.py` | `e272ff88df99668de3ddf8f0c597767be0331b5cafe9207a9166f8bc9afaa39d` |
| `isaac_ros_yopo_bringup/config/d435i_243622070369_fcu_imu.yaml` | `934388fe191b53b1b71b96bc3a5600a9524357fc0c0a8cc7c0da3ffaeb2f4f92` |
| `isaac_ros_yopo_bringup/config/px4_imu_noise_unvalidated.yaml` | `73328ada53105daf96af06dcc3e80edcfc9f9184ad67bca4e68e50d617062b9e` |
| `patches/isaac_ros_visual_slam_v3_2_15_imu_timestamp.patch` | `913b82b16e144a640139e14c24d01881b183c29643401e176df481bdabb0de50` |

YOPO 权重约 43 MiB，当前由 Git 跟踪。后续如果权重文件发生变化，即使文件名不变，
也必须建立新的基线，不允许覆盖本表 hash。

YOPO 文本文件使用 Git commit 内容计算规范化 SHA-256，不使用平台相关的工作树字节。
对应 Git blob 分别为：模型 `17c1ea8da45c512a2472ec776805331866d01e02`、配置
`6893372123e3f3b57a26c555e34be87169928837`、requirements
`0c3f2ecb3ead0c771737dd567d7656b97d17a8fb`。此前 Windows 工作树 hash 因 CRLF/LF
差异与 Jetson 不同，现已废止。

## 3. Jetson 与分层运行时基线

部署不是单容器。localization-runtime、宿主 YOPO system-Python runtime 和宿主 MAVROS/PX4
分别采集，详见 [`DEPLOYMENT_ARCHITECTURE.md`](DEPLOYMENT_ARCHITECTURE.md)。

| 项目 | 已验证值 | 证据状态 |
| --- | --- | --- |
| Jetson | Orin NX 16GB | 2026-07-21 历史证据已确认，待本轮复核 |
| JetPack / L4T | JetPack 6.2 / L4T 36.4.3 | 2026-07-21 历史证据已确认，待本轮复核 |
| Ubuntu | 容器 22.04.4 LTS；历史宿主记录 22.04.5 LTS | 容器已于本轮复核；宿主待复核 |
| ROS | ROS 2 Humble | 本轮已复核 |
| CUDA / cuDNN / TensorRT | CUDA 12.6.68；历史 cuDNN 9.3 / TensorRT 10.3 | CUDA 已于本轮复核；其余待复核 |
| localization 容器 PyTorch | `2.5.0a0+872d972e41.nv24.08`，CUDA 可用 | 仅为环境事实，不代表 YOPO runtime |
| YOPO runtime | `/usr/bin/python3` (Python 3.10)；PyTorch `2.11.0`；NumPy `1.21.5`；CUDA `12.6` | 入口 shebang、全部 import、CUDA Orin 和模型 warm-up 已通过 |
| 历史 Conda `yopo` | Python 3.8.20，缺少 `torch`/`numpy` | 不在当前 `yopo_node` 执行路径，不作为生产运行时 |
| YOPO source | `/home/nvidia/catkin_ws/src/YOPO_ROS2` | `ros2-humble@80c0569`，clean |
| YOPO ROS install | `/home/nvidia/catkin_ws/install/yopo_planner`、`quadrotor_msgs` | 已构建安装 |
| Docker | 29.4.3 | 本轮宿主复核 |
| NVIDIA Container Toolkit | 1.16.2 | 2026-07-21 历史证据已确认，待本轮复核 |
| RMW implementation | `rmw_fastrtps_cpp` | 本轮已复核 |
| ROS Domain | `42` | 本轮已复核 |
| cuVSLAM | Isaac ROS Visual SLAM `3.2.6` | Jetson package/commit 待复核 |
| MAVROS | `2.14.0-1jammy.20260608.191037` | 本轮宿主复核 |
| `mavros_msgs` | `2.14.0-1jammy.20260605.140341` | 本轮宿主复核 |

第一次采集所在 localization 容器没有 `mavros`、`mavros_msgs` 或 YOPO 源码，符合
部署隔离设计，不构成缺包。MAVROS/PX4 版本和 vehicle identity 必须在宿主环境采集；
YOPO commit、权重和 Python 版本必须在宿主实际 `yopo_node` 执行环境采集。禁止为了完成
清单而把这些依赖安装进 localization-runtime。

历史容器恢复锚点：

| 镜像 | Image ID | 状态 |
| --- | --- | --- |
| `isaac_ros_dev-aarch64:isaac3.2-rs2.55.1-jp6.2` | `sha256:bceda07bd17756dc582693e00fc388fb5b2589c94d8267e6a253546640f8a933` | 2026-07-17 已记录 |
| `isaac_ros_dev-aarch64:isaac3.2-rs2.55.1-vslam3.2-dev-20260717` | `sha256:ddd233211dc6cd29aaa6eaf6b317931e912f0ed057a762496e8df390904474dc` | 2026-07-21 已记录 |

镜像存在不等于其中已包含当前 `04d7b9c`、定位 adapter 或 YOPO 实现。必须重新执行
镜像 inspect 和工作区 commit 采集，禁止根据标签推断内容。

## 4. D435I 基线与证据边界

| 项目 | 已验证值 | 状态 |
| --- | --- | --- |
| USB ID | `8086:0b3a` | 已识别 |
| USB 链路 | 全部接口 `5000M`，descriptor `3.2` | 已通过 `lsusb -t` 与 librealsense 实测 |
| 序列号 | `243622070369` | 已记录 |
| 固件 | `5.15.1.55` | 已运行，不属于 Isaac ROS 3.2 官方推荐的 `5.13.0.50` |
| librealsense | `2.55.1` | 容器内 `pkg-config` 已验证 |
| RealSense ROS | `4.51.1-0jammy` | camera、msgs、description 已安装 |
| emitter | 关闭 | 已用于 90 Hz VIO 基线；Depth 联合运行仍待验证 |

必须区分以下三种配置：

| 配置 | 已有结果 | 能否批准 `YP-110` |
| --- | --- | --- |
| IR1 + IR2 + Depth + D435I IMU，约 30 Hz | 短时并发通过；图像约 29.99 Hz，合并 IMU 约 199.8 Hz，时间戳零值和回退均为 0 | 否；不是当前 FCU IMU/90 Hz 配置 |
| IR1 + IR2 @90 Hz + PX4 FCU IMU + cuVSLAM，Depth 关闭 | 双目约 89.9--90.4 Hz，aligned FCU IMU 约 171 Hz，连续约 17 分 48 秒，无持续失跟、USB 断连或 CUDA OOM | 否；没有并行 Depth |
| IR1 + IR2 @90 Hz + 原生 Depth + PX4 FCU IMU + cuVSLAM | Depth/IR 约 89.4--89.9 Hz；Depth full/centre p05 `0.873587`/`0.854368`；emitter=0；cuVSLAM `vo_state=1`；GPU/CPU 最高约 54.6/57.5 C | `YP-100/110` 和 Gate G2 通过；`YP-120=N/A_DECISION` |

因此，已有记录证明硬件能够分别运行 Depth 和 90 Hz cuVSLAM，但不证明两者在当前
launch 中并行时仍满足频率、同步、深度有效率、USB 和资源门禁。

2026-07-23 第一次本轮枚举返回 `RS2_USB_STATUS_BUSY`。这表示 D435I 被 RealSense
节点或其他进程占用，不是“未插入设备”的证据。复测前必须停止所有 RealSense launch；
设备保持 USB3 连接，但不启动相机 ROS 节点。

## 5. PX4 基线

| 项目 | 值 | 状态 |
| --- | --- | --- |
| 飞控硬件 | MAVLink vendor `7052`、product `54`、board version `54` | 已采集 |
| PX4 flight software version | packed `17761280` / `0x010F0400`，版本字段为 1.15.4 | 已采集 |
| PX4 custom version | `99c40407ff000000` | 已采集，保持 MAVROS 原始表达 |
| MAVLink vehicle identity | sysid `1`、compid `1`、autopilot `12`、type `2` | 已采集 |
| `MPC_THR_HOVER` | `0.60` | 用户确认；实际读取延后至 SO3/控制台架门禁 |
| 外部视觉 EKF 参数 | 不属于 `YP-020` | 在 `YP-200` 按实际接口读取必要子集 |
| OFFBOARD loss 参数 | 不属于 `YP-020` | 在 `YP-420` 读取必要子集 |

当前清单不批准任何 PX4 external-vision topic，也不批准飞行。接口选择仍由 `YP-200`
的 MAVROS 2.14/PX4 源码和台架证据决定。

## 6. 实机采集方法

三份采集必须分别在实际运行环境执行。

localization-runtime 容器：

```bash
cd /workspaces/isaac_ros-dev/src/TF_ROS_CUsalm
bash tools/collect_yopo_platform_baseline.sh \
  | tee /workspaces/isaac_ros-dev/localization_runtime_baseline_20260723_v2.txt
```

Jetson 宿主机的 YOPO system-Python runtime：

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /home/nvidia/catkin_ws/install/setup.bash

bash /home/nvidia/workspaces/isaac_ros_3_2/src/TF_ROS_CUsalm/tools/collect_yopo_runtime_baseline.sh \
  | tee /home/nvidia/yopo_runtime_baseline_20260723_v2.txt
```

Jetson 宿主机、MAVROS 已启动时：

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

bash /home/nvidia/workspaces/isaac_ros_3_2/src/TF_ROS_CUsalm/tools/collect_host_flight_stack_baseline.sh \
  | tee /home/nvidia/host_flight_stack_baseline_20260723.txt
```

三个脚本都只读。输出不得直接提交为“通过”；必须人工核对以下条件：

1. Jetson 实际仓库 commit 与第 1 节一致；
2. 各运行环境中的相关工作区均为 clean，或每个差异都有固定 patch；
3. 宿主 `yopo_node` shebang 为 `/usr/bin/python3`，权重和配置 hash 与第 2 节一致；
4. D435I 序列号与固件一致；
5. 宿主 MAVROS、`mavros_msgs` 和 PX4 固件身份完整；
6. 三份采集输出分别生成 SHA-256，并在本表记录路径和 hash。

## 7. 完成记录

| 字段 | 值 |
| --- | --- |
| localization-runtime v2 输出与 SHA-256 | `/workspaces/isaac_ros-dev/localization_runtime_baseline_20260723_v2.txt`；`aef99888985e09cd81d616aac869a76a05f46d6967917e18a3f5949efe04b999` |
| YOPO runtime inventory v1 | `/home/nvidia/yopo_runtime_baseline_20260723.txt`；`3c5940b7487ae55ffeaef1962d10096d169f8bbbc634080b91fb5963e1dfc3ed`；Conda 采集上下文，保留为历史证据 |
| YOPO runtime inventory v2 | `/home/nvidia/yopo_runtime_baseline_20260723_v2.txt`；`6a6974aa7faa8b74fa045f0e8734563a09ee4bad9c0d91a224a4278663dcc570` |
| YOPO 模型冒烟输出与 SHA-256 | `/home/nvidia/yopo_model_smoke_20260723.txt`；`909f0d839ddbdb64be7a1a9d44495139beaf86154f257d3a8fc615d3b8615608` |
| host flight stack 输出与 SHA-256 | `/home/nvidia/host_flight_stack_baseline_20260723.txt`；`fe9c7c5694d146879d8834c8c67467952d79a8e9c115b070480c3886e424231d` |
| 核对人 | 用户采集；Codex 按固定条件核对 |
| 核对结果 | `PASS_VERSION_INVENTORY` |
| `YP-020` 最终状态 | `PASSED` |

### 7.1 第一次采集结果

| 字段 | 值 |
| --- | --- |
| 采集时间 | `2026-07-23T08:06:25+00:00` |
| 输出路径 | `/workspaces/isaac_ros-dev/yopo_platform_baseline_20260723.txt` |
| 输出 SHA-256 | `1658891f87510a6e032eaf9053990719957d78a6636510ab308ab73896873081` |
| 结果 | `PARTIAL` |
| 已完成 | localization 容器的 Jetson/L4T、Ubuntu、CUDA、ROS/RMW、主要仓库 commit、librealsense |
| 符合设计的隔离项 | localization 容器内没有 YOPO、MAVROS 和 `mavros_msgs` |
| 缺口 | D435I busy；Visual SLAM applied patch 待复核；YOPO runtime 与 host flight stack 尚未独立采集 |

### 7.2 分层采集结果

| Scope | 结果 | 结论 |
| --- | --- | --- |
| localization-runtime | `PASS_VERSION_INVENTORY` | `4789863`，工作树 clean；D435I `243622070369`/`5.15.1.55` 可直接枚举且无 busy；wrapper patch reverse-check 和 source marker 通过 |
| YOPO runtime | `PASS_MODEL_LOAD` | `/usr/bin/python3`；PyTorch 2.11.0/CUDA 12.6 on Orin；官方 epoch-50 权重被动加载和 warm-up 通过；控制输出关闭 |
| host flight stack | `PASS_VERSION_INVENTORY` | USB3 5000M、Docker 镜像、MAVROS packages、PX4 connected/未解锁和 firmware identity 已固定 |

localization 容器未安装 `lsusb`，因此 v2 输出中记录
`lsusb=UNAVAILABLE_IN_THIS_ENVIRONMENT`。这不是 USB 证据缺口：宿主基线已记录同一
D435I `8086:0b3a` 全部接口为 `5000M`，容器内 librealsense 又独立确认了序列号和固件。
