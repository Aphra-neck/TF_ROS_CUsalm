# YOPO 实机集成项目需求

## 1. 项目目标

最终目标是在 Jetson、ROS 2 Humble、MAVROS 2.14 和 PX4 上完成室内低速
YOPO 自主避障飞行。完成标准不是节点启动，而是经过阶段门禁的实机闭环：

```text
定位源 -> PX4 EKF 融合状态 -> YOPO -> SO3 -> MAVROS -> PX4 -> 电机
```

cuVSLAM 是最终主定位源。动捕是临时主定位源、备份验证模式和 cuVSLAM 的
影子真值来源。首版禁止飞行中自动切换定位源。

## 2. 已确认的物理定义

### 2.1 参考点和安装外参

本项目明确采用：

```text
base_link = fcu_imu = mocap_rigid_body

T[base_link,camera_link]:
  translation_m = [0.05, 0.0, 0.0]
  rotation_xyzw = [0.0, 0.0, 0.0, 1.0]
```

相机位于飞控中心前方 50 mm，轴向与 `base_link` 平行。后续实现不得再次把
这两个安装关系当作未知占位值。

### 2.2 飞行器参数

| 参数 | 已确认值 |
| --- | --- |
| 平面布局 | X 布局 |
| 外轮廓 | 两条包含机圈的对角跨度均为 248 mm |
| 参考中心 | 两条对角线交点，即飞控和 `base_link` 中心 |
| 完整起飞重量 | 0.689 kg，包含电池和全部载荷 |
| PX4 悬停推力参数 | `MPC_THR_HOVER = 0.60` |

`MPC_THR_HOVER` 必须在阶段证据中从实际 PX4 参数导出复核，不能只依赖本文抄录。

## 3. 功能需求

### REQ-F-001：ROS 版本

Jetson 上所有新增运行代码必须使用 ROS 2 Humble。禁止为本阶段引入
`ros1_bridge`。历史 ROS 1 Controller 只能作为算法参考。

### REQ-F-002：定位模式

系统必须提供两个启动时互斥选择的模式：

- `cuvslam_primary`：cuVSLAM 是 PX4 外部定位源，动捕只做影子对比；
- `mocap_primary`：动捕是 PX4 外部定位源，cuVSLAM 继续运行但只做影子对比。

模式必须由独立 launch 明确选择。启动后参数只读，不提供运行时切换 service。
selector 只允许订阅当前 mode 的定位 pose，禁止同时订阅两个来源后在 callback 中
选择，也禁止因当前来源 stale 或故障自动创建另一来源 subscription。

selector 的输入 interface 固定为
`localization_adapter_interfaces/msg/LocalizationSourceCandidate`，输出 interface
固定为 `localization_adapter_interfaces/msg/SelectedPoseCandidate`。输出表达当前
epoch 的 `map -> base_link`，保留来源采样时间且不包含 twist/covariance；来源由
字符串 `mode` 和 `source_contract_id` 固定，不使用 source enum。完整双模式系统
launch 由 `YP-250` 交付，不能用单独 selector 节点启动代替。

### REQ-F-003：模式固定的地图语义

`cuvslam_primary` 每次 localization epoch 建立一个起飞局部 `map`：

- 起飞位置为原点；
- 初始机头为 `+x`；
- 左侧为 `+y`；
- 上方为 `+z`；
- 对齐只计算一次并在整个 epoch 内锁定。

`mocap_primary` 令 `map == mocap_world`，保留动捕系统定义的绝对 XYZ 和 yaw，
`T[map,mocap_world]` 固定为 identity。首版不宣称任一模式的 `map +x` 是真实地理东向。

### REQ-F-004：定位职责

定位 adapter 负责来源特有的读取、校验和固定安装变换；selector 负责启动时来源
互斥、模式固定的 `map` 变换和 pose-only selected seam；gateway 只负责把获批的
selected pose 门禁到 PX4 pose-only 输入。三者不得通过 Pose 差分建立新的飞行速度
估计器。

selector 不得发布 `/localization/odometry`、`/state/odom`、TF、MAVROS 或控制 topic。
选择成功不等于 gateway、Gate G3、OFFBOARD、解锁或飞行授权。

首版不发布 `/localization/odometry`。`SelectedPoseCandidate` 没有 twist 或 covariance，
将它包装为 `nav_msgs/Odometry` 会扩大未获批准的速度、covariance 和 child-frame
语义。完整状态由 PX4 EKF 提供，并由 `yopo_state_bridge` 转成 YOPO 私有接口。

### REQ-F-005：最终状态来源

PX4 EKF 是飞行时完整状态估计 authority。cuVSLAM 或动捕向 PX4 提供经过
审核的外部位姿，PX4 与 IMU 融合后输出位置、姿态和速度。YOPO 飞行时只消费
该融合状态的兼容表示，不直接消费原始定位 Pose。

### REQ-F-006：YOPO 兼容接口

首版必须提供 C++ `yopo_state_bridge`，将 PX4 已估计的融合状态转换为当前 YOPO
需要的 `/state/odom`：Pose 表达于本次 `map`，`twist.linear` 表达于 `map`。
该非标准接口只允许 YOPO 消费，不允许 SO3、MAVROS 或其他节点复用。

### REQ-F-007：深度输入

D435 必须同时提供：

- cuVSLAM 红外双目 `640x360@90 Hz`；
- 原生深度流；
- YOPO 只处理最新深度帧，推理目标为 15--30 Hz。

第一轮保持红外发射器关闭。只有深度有效率不满足验收时才执行 emitter A/B。

### REQ-F-008：YOPO 输出

YOPO 输出期望位置、速度、加速度和偏航轨迹。被动阶段必须保持
`output_enabled=false`，不得产生有效飞控命令。

### REQ-F-009：SO3 控制

历史 SO3 Controller 的必要生产功能必须移植为 ROS 2 C++ package。SO3 根据
YOPO 参考轨迹和 PX4 融合状态计算期望姿态、角速度和推力。模拟器、历史 MAVROS
消息副本和无关 ROS 1 工具不进入生产移植范围。

### REQ-F-010：控制后端

SO3 核心与 MAVROS backend 必须解耦。首版 backend 优先输出
`mavros_msgs/msg/AttitudeTarget`，但实际字段和 type mask 允许根据台架验证调整，
且不得因此修改 SO3 核心算法接口。

### REQ-F-011：人工权限

任何软件不得自动解锁飞行器。解锁和进入 OFFBOARD 必须由飞手通过遥控器或
明确的人工命令完成。任一节点重启后不得自动恢复飞行。

## 4. 非功能需求

### REQ-NF-001：语言和模块

定位 selector、output gateway、YOPO state bridge 和 SO3 新运行节点优先使用
C++17。YOPO 神经网络和轨迹代码保留现有 Python/PyTorch 实现。

### REQ-NF-002：发布权

每个规范 topic 必须只有一个 publisher authority。重复 publisher、错误 GID、
未知 mode 或冲突配置必须 fail closed。

### REQ-NF-003：时间

转换节点必须保留来源采样时间。当前 VRPN `use_server_time=false` 的 Jetson 回调
时间足以用于首版低速影子验证，但不得据此输出光学延迟或高动态 ATE/RPE 结论。

### REQ-NF-004：诊断

所有 gateway 和 selector 必须持续发布结构化 diagnostics，包括 mode、authority、
输入新鲜度、frame、拒绝计数、输出授权和故障原因。selector 还必须报告 source 与
selector contract ID、publisher GID、一次对齐状态和 localization epoch ID。

### REQ-NF-005：证据

每次阶段验收必须固定仓库 revision、PX4 固件和参数、MAVROS 版本、配置 hash、
rosbag hash、分析报告、日期、模式、操作者和结果。

## 5. 仓库边界

| 代码库或 package | 责任 |
| --- | --- |
| `TF_ROS_CUsalm` | 坐标合同、来源适配、selector、外部位姿 gateway、YOPO 状态格式桥 |
| `YOPO_ROS2` | 网络推理、轨迹生成和目标输入 |
| `isaac_ros_yopo_bringup` | D435、深度、IMU、cuVSLAM 和系统启动编排 |
| `so3_control_ros2` | SO3、限幅、超时和可替换控制 backend |
| PX4 | EKF 状态融合、OFFBOARD failsafe 和底层飞行控制 |

桥接器作为 `TF_ROS_CUsalm` 内的独立 ROS package 开发，不新建 Git 仓库。

代码仓库归属不等于运行环境合并。部署必须遵循
[`DEPLOYMENT_ARCHITECTURE.md`](DEPLOYMENT_ARCHITECTURE.md)：

- localization-runtime 独占 D435I，并包含 Isaac ROS、NITROS 和 cuVSLAM；
- YOPO 使用宿主机 ROS 2 和 `/usr/bin/python3`，当前源码位于
  `/home/nvidia/catkin_ws/src/YOPO_ROS2`，并已完成构建、CUDA 模型加载和被动冒烟；
- MAVROS/PX4 链路和未来 control gateway 位于宿主或独立控制环境；
- 环境之间只通过 ROS 2 DDS 和固定消息合同交互；
- 禁止把 YOPO/PyTorch 依赖安装进 localization-runtime，也禁止让 YOPO runtime
  直接打开 D435I。

## 6. 首版明确排除

- 飞行中自动切换定位源；
- localization epoch 内动态修改世界对齐；
- 地理东/北标定；
- VRPN 光学采集延迟精密标定；
- 移动障碍和人员附近飞行；
- YOPO 网络重新训练；
- 高速飞行和窄缝穿越；
- 软件自动解锁；
- 直接电机控制。
