# YOPO 实机集成接口合同

## 1. 变换记号

`T[A,B]` 表示 frame B 在 frame A 中的位姿，并把 B 中的点变换到 A：

```text
p_A = T[A,B] * p_B
T[A,C] = T[A,B] * T[B,C]
```

禁止仅根据 TF 箭头文字、变量名或 `frame_id` 猜测矩阵方向。

## 2. Frame 合同

| Frame | 定义 | 生命周期 |
| --- | --- | --- |
| `map` | 模式固定世界：cuVSLAM 为起飞局部世界，mocap 等同 `mocap_world` | cuVSLAM epoch 或动捕标定周期 |
| `odom` | cuVSLAM 连续局部世界，初始 yaw 可任意 | cuVSLAM epoch |
| `mocap_world` | 动捕固定实验室世界，右手、z-up | 动捕标定周期 |
| `base_link` | 项目机体参考点，FLU | 固定安装 |
| `fcu_imu` | 飞控 IMU 参考点，项目中与 `base_link` 重合 | 固定安装 |
| `mocap_rigid_body` | `droneyee207` 刚体虚拟中心，FLU | 固定安装 |
| `camera_link` | D435 机身参考点，FLU | 固定安装 |
| optical frames | x 右、y 下、z 前 | RealSense 驱动合同 |

`map` 是右手 z-up 世界；cuVSLAM 模式下为局部世界，mocap 模式下为固定实验室世界，
均不是本阶段的地理 ENU 声明。ROS 数据进入 MAVROS 时仍使用经审计的 ROS ENU/FLU
边界表达，最终 NED/FRD 转换只由 MAVROS 完成。

## 3. 固定安装变换

```text
T[base_link,fcu_imu] = I
T[base_link,mocap_rigid_body] = I

T[base_link,camera_link]:
  translation_m = [0.05, 0.0, 0.0]
  rotation_xyzw = [0.0, 0.0, 0.0, 1.0]
```

cuVSLAM Pose 转换必须使用：

```text
T[odom,base_link]
  = T[odom,camera_link]
  * inverse(T[base_link,camera_link])
```

不得只把 `child_frame_id` 从 `camera_link` 改成 `base_link`。

## 4. 模式固定的世界变换

`cuvslam_primary` 启动时将当前 `base_link` 位置设为 `map` 原点，并只使用水平 yaw
建立对齐，避免把起飞时的轻微 roll/pitch 倾斜固化到世界 z 轴：

```text
yaw_map_from_odom = -yaw(T[odom,base_link]_initial)
R[map,odom] = Rz(yaw_map_from_odom)
t[map,odom] = -R[map,odom] * p[odom,base_link]_initial
```

`mocap_primary` 不执行首帧归零或 yaw 对齐。项目把动捕世界直接定义为全局 `map`：

```text
T[map,mocap_world] = identity
T[map,base_link] = T[mocap_world,base_link]
```

选定模式的变换在当前 epoch 内必须保持常量。定位源重启、GID 改变、时间回退、
Pose reset 或模式改变都必须结束当前 epoch。

## 5. 定位模式与发布权

### 5.1 `cuvslam_primary`

- cuVSLAM Pose 是外部定位候选；
- 动捕继续发布命名清晰的 shadow candidate；
- 只有 cuVSLAM 路径可获得 PX4 外部定位 gateway 授权；
- 动捕路径不得发布 PX4 输入或 YOPO 主状态。

### 5.2 `mocap_primary`

- 动捕 Pose 是外部定位候选；
- cuVSLAM 继续发布 shadow candidate；
- 只有动捕路径可获得 PX4 外部定位 gateway 授权；
- cuVSLAM 路径不得发布 PX4 输入或 YOPO 主状态。

两个模式必须使用独立 launch。mode 参数启动后不可变，且不提供切换 service。

### 5.3 `localization_source_selector` seam

`YP-220` 采用方案 A。selector 只为启动时选定的一个来源创建 pose subscription，
不订阅另一个来源，也不在运行中创建或替换 subscription。选定来源必须先由其
source adapter 形成 `T[source_world,base_link]`，selector 再按第 4 节公式计算一次
模式固定的 `T[map,source_world]` 并在整个 localization epoch 内锁定。

selector 的唯一 pose 输出是：

```text
/localization/selected/pose
  localization_adapter_interfaces/msg/SelectedPoseCandidate
  T[map,base_link]
```

selector 输入精确使用 `LocalizationSourceCandidate`，字段为 `Header`、
`semantic_child_frame`、`Pose`、`source_id`、`source_contract_id` 和 `authorization`。
`SelectedPoseCandidate` 不含 twist 或 covariance，继承来源采样时间，并精确携带
`selector_contract_id`、`localization_epoch_id`、`mode`、`source_contract_id` 和
`authorization`。它只是 YP-230 的类型受限 pose candidate，不是 odometry、TF 或
MAVROS authority。详细合同见
[`YP_220_SOURCE_SELECTOR_CONTRACT.md`](YP_220_SOURCE_SELECTOR_CONTRACT.md)。

mode、source contract、publisher GID、source reset 或锁定变换发生变化时必须停止
selected output 并结束 epoch；禁止自动切换、自动重新对齐或用未选来源接管。

## 6. Topic 合同

下表区分当前观测输入与目标接口。目标接口只有在依赖门禁通过后才能发布。
这些 topic 同时是运行环境之间的 DDS 边界；任何消费者不得通过共享进程内存、
Python 环境或容器动态库绕过消息合同。

| Topic | Type | 生产者 | 语义 |
| --- | --- | --- | --- |
| `/visual_slam/tracking/odometry` | `nav_msgs/msg/Odometry` | cuVSLAM | 当前观测为 `odom -> camera_link` |
| `/droneyee207/pose` | `geometry_msgs/msg/PoseStamped` | VRPN | `world -> mocap_rigid_body` |
| `/localization/shadow/mocap/assumed_base_pose` | `ShadowPoseCandidate` | mocap adapter | `mocap_world -> base_link` 影子候选 |
| `/localization/candidates/cuvslam/base_pose` | `LocalizationSourceCandidate` | cuVSLAM adapter | `odom -> base_link`；已应用 50 mm 外参；仅 `source_pose_candidate_only` |
| `/localization/candidates/mocap/base_pose` | `LocalizationSourceCandidate` | mocap adapter | `mocap_world -> base_link`；仅 `source_pose_candidate_only` |
| `/localization/selected/pose` | `SelectedPoseCandidate` | `localization_source_selector` | 当前 epoch 的 `map -> base_link`；无 twist/covariance；只供 gateway 使用 |
| `/mavros/vision_pose/pose_cov` | `geometry_msgs/msg/PoseWithCovarianceStamped` | localization output gateway | 选定的 pose-only PX4 输入候选；当前仍禁止发布 |
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | MAVROS/PX4 | PX4 EKF 融合状态；实际 frame 必须启动时验证 |
| `/state/odom` | `nav_msgs/msg/Odometry` | `yopo_state_bridge` | YOPO legacy：Pose 在 `map`，线速度表达于 `map` |
| D435 depth topic | `sensor_msgs/msg/Image` | RealSense | 原生深度；实际名称由 bringup 合同固定 |
| YOPO position command | `quadrotor_msgs/msg/PositionCommand` | YOPO | 位置、速度、加速度、yaw 均属于 `map` |
| SO3 internal command | 独立 ROS 2 接口 | SO3 core | 期望姿态、角速度、归一化推力 |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/msg/AttitudeTarget` | control backend | 首版候选 PX4 控制边界 |

localization-runtime 是 D435I 和原生深度 topic 的唯一 publisher authority。YOPO
runtime 只订阅标准深度和状态接口。MAVROS 与 PX4 的接口审计在宿主/控制环境完成，
不要求 localization-runtime 安装 MAVROS 可执行包。

### 6.1 PX4 外部视觉接口选择

MAVROS 2.14/PX4 1.15.4 源码审计选择 `/mavros/vision_pose/pose_cov` 作为两个
pose-only 定位源的唯一 gateway 候选。gateway 必须把未知的 6x6 pose covariance
全部填写为 IEEE-754 `NaN`；这表示“未知”，不是测量零方差。MAVROS 负责
ENU/FLU 到 NED/FRD 的边界转换，ROS 侧禁止预转换。

以下接口不用于首版 gateway：

- `/mavros/odometry/out`：MAVROS 会无条件序列化 twist，ROS 默认零会伪造成
  速度测量；
- `/mavros/vision_pose/pose`：MAVROS 会为缺失 covariance 构造全零数组；
- `/mavros/mocap/pose`：不会作为 cuVSLAM 与动捕共用的规范 gateway 边界。

PX4 接收 `VISION_POSITION_ESTIMATE` 后只得到 pose，velocity 保持 `NaN/UNKNOWN`。
`EKF2_EV_CTRL` 的 velocity 位必须保持关闭。由于 `YP-210` 已把相机 Pose 转换到
`base_link`，PX4 的 `EKF2_EV_POS_X/Y/Z` 目标值为零，禁止再次补偿 50 mm。

实际 endpoint、publisher authority 和 PX4 参数只读值已由 `YP-200` 验证。全
`NaN` covariance 经当前 MAVROS binary 到 PX4 的透传，以及 PX4 实际采用参数噪声
回退，仍须 Gate G3 台架验证；完成前该 topic 必须保持 publisher count 为零，也不
构成 Gate G3 或飞行授权。

### 6.2 `/state/odom` 的非标准语义

`/state/odom.twist.linear` 表达于 `map`，不符合标准 `nav_msgs/Odometry` 通常约定。
这是受控的 YOPO legacy 接口，必须满足：

- 只有 `yopo_state_bridge` 发布；
- 只有 YOPO 订阅；
- 数据来源是 PX4 已估计的融合速度；
- bridge 只旋转/换基，不估计速度；
- 删除 legacy 接口前不得静默改变语义。

### 6.3 `/localization/odometry` 延后

首版不创建 `/localization/odometry` publisher。selected seam 只有 Pose，不携带 twist
或 covariance；零值会伪造有效测量，NaN Odometry 也会引入当前没有消费者、没有
验收依据的额外合同。若未来确有规范 Odometry 需求，必须从 PX4 EKF 融合状态定义
独立 module 和验收，不得从 pose-only selected 输入推导或填充速度。

## 7. 时间合同

- 转换输出继承来源采样时间，不使用转换回调时刻替换；
- 接收时刻只能用于 stale、gap 和 clock-domain 诊断；
- VRPN 当前 `use_server_time=false`，其 stamp 是 Jetson 回调时间；
- 该时间语义允许首版低速影子验证，不允许精密光学延迟结论；
- PX4 融合状态、YOPO 深度和控制命令必须各自实施 freshness gate。

## 8. 速度职责

- 动捕 adapter 不计算速度；
- `localization_source_selector` 不消费、计算或发布速度；
- localization output gateway 只输出 pose-only MAVROS 候选，不创建 Odometry 或速度；
- `yopo_state_bridge` 只转换 PX4 EKF 已估计速度；
- YOPO 不估计定位速度；
- SO3 使用 PX4 融合状态反馈；
- 任一缺失速度不得用常数零静默填充并声明为有效测量。

## 9. 控制职责

```text
YOPO reference
  -> SO3 translational/geometric outer controller
  -> replaceable MAVROS backend
  -> PX4 attitude/rate loops and mixer
```

PX4 EKF 是状态估计 authority，不是轨迹控制器。SO3 核心不得负责解锁、模式切换、
定位源切换或直接电机控制。
