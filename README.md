# TF_ROS_CUsalm

YOPO 实机集成中的 ROS 2 定位合同、适配器、启动时定位源选择器和只读证据工具。

> 当前状态：`localization_source_selector` 已通过，`localization_output_gateway`
> 正在执行 TDD 红阶段，权威状态见
> [`TASKFLOW.md`](docs/yopo_integration/TASKFLOW.md)。本仓库当前不授予 PX4 external
> vision、OFFBOARD、解锁、控制或飞行权限。

## 1. 当前架构

本项目把来源适配、启动时选择和最终输出分成三层：

```text
Isaac ROS cuVSLAM                         VRPN / droneyee207
T[odom,camera_link]                      T[world,mocap_rigid_body]
          |                                          |
          v                                          v
cuvslam_localization_adapter             mocap_localization_adapter
          |                                          |
          | LocalizationSourceCandidate              |
          v                                          v
/localization/candidates/cuvslam/base_pose   /localization/candidates/mocap/base_pose
          \                                          /
           \-------- launch-time selected source ---/
                              |
                              v
                localization_source_selector
       one subscription, mode-specific map transform
                              |
                              v
                 /localization/selected/pose
                    SelectedPoseCandidate
                              |
                              v
             localization_output_gateway (YP-230 in progress)
             default: disabled, no privileged publisher
                              X
       /mavros/vision_pose/pose_cov (publisher count = 0)
```

两个定位源是同级候选。`cuvslam_primary` 和 `mocap_primary` 必须由互斥的完整 launch
选择；启动后不提供 mode service、动态切换或故障自动接管。主源 stale、reset、时间
回退、publisher 变化或合同不匹配时，selector 停止 selected output 并 fail closed。
恢复 authority 不能靠自动切换来源，也不能靠自动重启单个 selector。

完整双模式 launch 属于 `YP-250`，output gateway 属于当前 `YP-230`。默认 gateway
合同不创建 MAVROS publisher；当前仓库中的 module launch 只用于实现与测试，不代表
完整系统或飞行授权。

## 2. 已固定的坐标合同

变换记号 `T[A,B]` 表示 frame B 在 frame A 中的位姿，并把 B 中的点变换到 A。

```text
base_link = fcu_imu = mocap_rigid_body

T[base_link,camera_link]:
  translation_m = [0.05, 0.0, 0.0]
  rotation_xyzw = [0.0, 0.0, 0.0, 1.0]
```

相机位于飞控 IMU 前方 50 mm，轴向与机体平行。cuVSLAM adapter 必须先把
`T[odom,camera_link]` 转换为 `T[odom,base_link]`，不能只修改 frame 名称。动捕刚体
中心、飞控 IMU 和 `base_link` 重合，不得重复施加杠杆臂补偿。

`map` 的来源语义由启动模式固定。`cuvslam_primary` 在每个 localization epoch
以首帧建立局部右手 z-up 世界：初始位置为原点、初始机头为 `+x`、左侧为 `+y`。
`mocap_primary` 则令 `map == mocap_world`，使用固定 identity 变换并保留动捕定义的
绝对 position 和 yaw；不得用无人机启动位置重新建立动捕世界原点。

详细 frame 和时间语义见
[`INTERFACE_CONTRACTS.md`](docs/yopo_integration/INTERFACE_CONTRACTS.md)。

## 3. 数据与权限边界

| Topic | Type | 唯一生产者 | 当前权限 |
| --- | --- | --- | --- |
| `/visual_slam/tracking/odometry` | `nav_msgs/msg/Odometry` | cuVSLAM | 原始来源 |
| `/droneyee207/pose` | `geometry_msgs/msg/PoseStamped` | VRPN client | 原始来源 |
| `/localization/candidates/cuvslam/base_pose` | `LocalizationSourceCandidate` | cuVSLAM adapter | `source_pose_candidate_only` |
| `/localization/candidates/mocap/base_pose` | `LocalizationSourceCandidate` | mocap adapter | `source_pose_candidate_only` |
| `/localization/shadow/mocap/assumed_base_pose` | `ShadowPoseCandidate` | mocap adapter | 只读 shadow evidence |
| `/localization/selected/pose` | `SelectedPoseCandidate` | selector | `selected_pose_candidate_only` |
| `/mavros/vision_pose/pose_cov` | `PoseWithCovarianceStamped` | Gate G3 output gateway | 当前 publisher 必须为 0 |

adapter 只能生产 source-private candidate 或 shadow evidence。只有 selector 可以生产
selected candidate；selected candidate 仍不是 canonical localization、TF、MAVROS、
YOPO control 或 flight authority。非主来源可以继续留下 source-private/shadow evidence，
但不得获得 selected、gateway、PX4 external-vision 或控制权限。

任何 adapter 或 selector 都不得：

- 发布 `/mavros/*`、TF、YOPO command 或电机控制 topic；
- 调用 PX4 parameter、mode、OFFBOARD、arming、takeoff 或 landing 服务；
- 根据来源故障自动订阅或切换到另一定位源；
- 用最后一个有效 pose 掩盖 stale 或 reset；
- 从 pose 差分伪造定位速度。

## 4. ROS 2 packages

| Package | 责任 |
| --- | --- |
| `localization_contracts` | frame-aware 数学、校验和 fail-closed 合同 |
| `localization_adapter_interfaces` | `ShadowPoseCandidate`、`LocalizationSourceCandidate` 和 `SelectedPoseCandidate` |
| `cuvslam_localization_adapter` | cuVSLAM camera pose 到 `base_link` source candidate |
| `mocap_localization_adapter` | VRPN pose 校验、mocap source candidate 和 shadow evidence |
| `localization_source_selector` | 启动时单源订阅、模式固定的 map 变换和 selected pose seam |
| `localization_output_gateway` | 默认禁用的 pose-only MAVROS 输出门禁；YP-230 TDD 中 |
| `bag_contract_probe` | 对固定 rosbag 执行只读、确定性的合同审计 |

仓库不包含 YOPO 模型运行时、cuVSLAM SDK、RealSense 驱动、MAVROS 实现、PX4 固件或
SO3 控制器。localization runtime 与 YOPO runtime 保持独立，通过 ROS 2 DDS 接口通信。

## 5. 构建与测试

在 ROS 2 Humble workspace 中执行：

```bash
cd /workspaces/isaac_ros-dev
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --symlink-install \
  --packages-up-to \
    localization_contracts \
    localization_adapter_interfaces \
    cuvslam_localization_adapter \
    mocap_localization_adapter \
    localization_source_selector \
    localization_output_gateway \
    bag_contract_probe \
  --event-handlers console_direct+

source install/setup.bash

colcon test \
  --packages-select \
    localization_contracts \
    localization_adapter_interfaces \
    cuvslam_localization_adapter \
    mocap_localization_adapter \
    localization_source_selector \
    localization_output_gateway \
    bag_contract_probe \
  --event-handlers console_direct+

colcon test-result --verbose
```

测试通过只说明对应 module 满足当前代码合同，不会自动把任务改为 `PASSED`。任务状态
只能在证据产生并复核后更新 [`TASKFLOW.md`](docs/yopo_integration/TASKFLOW.md)。

## 6. 文档入口

集成合同位于 [`docs/yopo_integration`](docs/yopo_integration/README.md)：

- [`PROJECT_REQUIREMENTS.md`](docs/yopo_integration/PROJECT_REQUIREMENTS.md)：目标、范围和物理定义；
- [`INTERFACE_CONTRACTS.md`](docs/yopo_integration/INTERFACE_CONTRACTS.md)：frame、topic、时间和 authority；
- [`SAFETY_CONSTRAINTS.md`](docs/yopo_integration/SAFETY_CONSTRAINTS.md)：禁止项和安全门禁；
- [`ACCEPTANCE_CRITERIA.md`](docs/yopo_integration/ACCEPTANCE_CRITERIA.md)：Gate 验收条件；
- [`TASKFLOW.md`](docs/yopo_integration/TASKFLOW.md)：唯一任务状态来源；
- [`YP_200_MAVROS_PX4_EXTERNAL_VISION_AUDIT.md`](docs/yopo_integration/YP_200_MAVROS_PX4_EXTERNAL_VISION_AUDIT.md)：pose-only MAVROS/PX4 审计；
- [`YP_210_EXTRINSIC_CONTRACT_20260723.md`](docs/yopo_integration/YP_210_EXTRINSIC_CONTRACT_20260723.md)：50 mm 外参合同；
- [`YP_220_SOURCE_SELECTOR_CONTRACT.md`](docs/yopo_integration/YP_220_SOURCE_SELECTOR_CONTRACT.md)：selector 详细合同与验收范围；
- [`YP_230_OUTPUT_GATEWAY_CONTRACT.md`](docs/yopo_integration/YP_230_OUTPUT_GATEWAY_CONTRACT.md)：默认禁用的 pose-only gateway 合同。

## 7. 当前阶段

Gate G2、YP-200、YP-210 和 YP-220 已通过并绑定证据；`YP-230` 正在执行默认
disabled gateway 的 TDD 红阶段。Gate G3、external vision、YOPO 主动输出、SO3、
OFFBOARD、解锁和实机飞行均未授权。
