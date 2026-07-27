# YP-200 MAVROS/PX4 外部视觉接口审计

日期：2026-07-23；只读运行审计完成于 2026-07-24

```text
source_audit=COMPLETE
status_source=TASKFLOW.md
selected_candidate=/mavros/vision_pose/pose_cov
selected_message=geometry_msgs/msg/PoseWithCovarianceStamped
velocity_measurement=ABSENT
covariance_measurement=UNKNOWN_ALL_NAN
runtime_endpoint_evidence=COMPLETE
runtime_nan_passthrough_evidence=DEFERRED_TO_GATE_G3
px4_parameter_evidence=COMPLETE_READ_ONLY
external_vision_output_authorization=DENIED
offboard_authorization=DENIED
arming_authorization=DENIED
flight_authorization=DENIED
```

本报告已经完成源码层接口选择、运行时 endpoint 和 PX4 参数只读取证；`YP-200` 的
权威任务状态只在 `TASKFLOW.md` 维护。本报告不构成 Gate G3、OFFBOARD、解锁或飞行
授权。NaN covariance 透传必须由实现后的 gateway 在 Gate G3 拆桨台架验证，不作为
尚未实现 gateway 时的循环前置条件。

## 1. 固定版本与一手来源

| 组件 | 本项目版本 | 本次审计的不可变源码 |
| --- | --- | --- |
| MAVROS | Debian `2.14.0-1jammy.20260608.191037` | 官方 tag `2.14.0` 指向 commit [`c655e6343ec81687d51e7185bcba7a651e361fcd`](https://github.com/mavlink/mavros/tree/c655e6343ec81687d51e7185bcba7a651e361fcd) |
| PX4 | `v1.15.4` | 官方 tag/实机 custom revision [`99c40407ffd7ac184e2d7b4b293f36f10fe561ef`](https://github.com/PX4/PX4-Autopilot/tree/99c40407ffd7ac184e2d7b4b293f36f10fe561ef) |

主要一手来源：

- MAVROS [`vision_pose_estimate.cpp`](https://github.com/mavlink/mavros/blob/c655e6343ec81687d51e7185bcba7a651e361fcd/mavros_extras/src/plugins/vision_pose_estimate.cpp#L52-L183)；
- MAVROS [`odom.cpp`](https://github.com/mavlink/mavros/blob/c655e6343ec81687d51e7185bcba7a651e361fcd/mavros_extras/src/plugins/odom.cpp#L59-L341)；
- MAVROS [`local_position.cpp`](https://github.com/mavlink/mavros/blob/c655e6343ec81687d51e7185bcba7a651e361fcd/mavros/src/plugins/local_position.cpp)；
- MAVROS [`plugin.hpp`](https://github.com/mavlink/mavros/blob/c655e6343ec81687d51e7185bcba7a651e361fcd/mavros/include/mavros/plugin.hpp) 与 [`px4_config.yaml`](https://github.com/mavlink/mavros/blob/c655e6343ec81687d51e7185bcba7a651e361fcd/mavros/launch/px4_config.yaml)；
- PX4 [`mavlink_receiver.cpp`](https://github.com/PX4/PX4-Autopilot/blob/99c40407ffd7ac184e2d7b4b293f36f10fe561ef/src/modules/mavlink/mavlink_receiver.cpp#L1237-L1479)；
- PX4 [`EKF2.cpp`](https://github.com/PX4/PX4-Autopilot/blob/99c40407ffd7ac184e2d7b4b293f36f10fe561ef/src/modules/ekf2/EKF2.cpp#L2175-L2314)；
- PX4 [`module.yaml`](https://github.com/PX4/PX4-Autopilot/blob/99c40407ffd7ac184e2d7b4b293f36f10fe561ef/src/modules/ekf2/module.yaml)；
- PX4 [`Timesync.cpp`](https://github.com/PX4/PX4-Autopilot/blob/99c40407ffd7ac184e2d7b4b293f36f10fe561ef/src/lib/timesync/Timesync.cpp#L124-L133)；
- MAVLink [`VISION_POSITION_ESTIMATE`](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE) 与 [`ODOMETRY`](https://mavlink.io/en/messages/common.html#ODOMETRY) 消息定义；
- PX4 v1.15 官方[外部位置估计文档](https://docs.px4.io/v1.15/en/ros/external_position_estimation.html)。

以下“已验证事实”均来自上述固定源码；“建议”是本项目的工程选择；“待补证据”
必须在 Jetson/PX4 拆桨台架完成。

## 2. MAVROS 2.14 的实际 ROS 2 接口

MAVROS plugin 用独立 ROS 2 subnode 运行。默认 UAS FQN 为 `/mavros` 时，源码中的
private topic `~/...` 展开如下。自定义 namespace/remap 会改变完整 topic，因此仍需
运行时复核。

| 完整 topic | 类型 | MAVROS endpoint | 方向 | MAVLink 作用 |
| --- | --- | --- | --- | --- |
| `/mavros/vision_pose/pose` | `geometry_msgs/msg/PoseStamped` | Node `vision_pose`，namespace `/mavros` | MAVROS 订阅 | 发送 `VISION_POSITION_ESTIMATE` |
| `/mavros/vision_pose/pose_cov` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Node `vision_pose`，namespace `/mavros` | MAVROS 订阅 | 发送 `VISION_POSITION_ESTIMATE` |
| `/mavros/odometry/out` | `nav_msgs/msg/Odometry` | Node `odometry`，namespace `/mavros` | MAVROS 订阅 | 发送 `ODOMETRY` |
| `/mavros/odometry/in` | `nav_msgs/msg/Odometry` | Node `odometry`，namespace `/mavros` | MAVROS 发布 | 接收 FCU `ODOMETRY` |
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | Node `local_position`，namespace `/mavros` | MAVROS 发布 | PX4 local position/attitude 的 ROS ENU/FLU 表示 |
| `/mavros/mocap/pose` | `geometry_msgs/msg/PoseStamped` | Node `mocap`，namespace `/mavros` | MAVROS 订阅 | 发送 `ATT_POS_MOCAP`；本项目不选用 |
| `/mavros/state` | `mavros_msgs/msg/State` | Node `sys`，namespace `/mavros` | MAVROS 发布 | FCU 连接、模式和 armed 状态 |
| `/mavros/timesync_status` | `mavros_msgs/msg/TimesyncStatus` | Node `time`，namespace `/mavros` | MAVROS 发布 | Jetson/PX4 时间同步状态 |

`vision_pose` 的 topic 和类型直接见源码
[`L68-L74`](https://github.com/mavlink/mavros/blob/c655e6343ec81687d51e7185bcba7a651e361fcd/mavros_extras/src/plugins/vision_pose_estimate.cpp#L68-L74)；
`odometry` 的 `~/in` publisher 与 `~/out` subscriber 见
[`L82-L89`](https://github.com/mavlink/mavros/blob/c655e6343ec81687d51e7185bcba7a651e361fcd/mavros_extras/src/plugins/odom.cpp#L82-L89)。

## 3. 已选 pose-only 路径

### 3.1 选择

本项目的 external-vision gateway 候选固定为：

```text
/localization/selected/pose 的 selected_pose_candidate_only Pose
  -> localization_output_gateway
  -> geometry_msgs/msg/PoseWithCovarianceStamped
  -> /mavros/vision_pose/pose_cov
  -> MAVLink VISION_POSITION_ESTIMATE
  -> PX4 vehicle_visual_odometry
  -> EKF2（只融合获批字段）
```

消息合同：

- Pose 必须是当前 localization epoch 的 `map -> base_link`；
- stamp 必须继承原始定位采样时间；
- 不携带、不估计且不填充速度；
- 现阶段 covariance 没有测量依据，36 个元素全部写入 IEEE `NaN`，明确表达
  `unknown`，不得填零或编造经验数值；
- YP-210 已把相机 pose 转成 `base_link` pose，MAVROS 之前不得再次应用 50 mm 外参。

该选择仍是“待台架验证的接口候选”，不是输出授权。

### 3.2 为什么拒绝 `/mavros/vision_pose/pose`

MAVROS 的 `PoseStamped` callback 明确执行：

```cpp
ftf::Covariance6d cov {};  // zero initialized
```

然后把这个全零矩阵发送到 `VISION_POSITION_ESTIMATE`。证据见
[`vision_cb()` L171-L177](https://github.com/mavlink/mavros/blob/c655e6343ec81687d51e7185bcba7a651e361fcd/mavros_extras/src/plugins/vision_pose_estimate.cpp#L171-L177)。
全零不是“未知”，会错误声明零测量方差。虽然 PX4 还会以 EKF 参数作为噪声下界，
本项目仍禁止在 gateway 边界产生这个错误语义，因此裸 `/pose` 被否决。

TF listener 路径同样创建全零 covariance，且会引入第二种数据入口，因此
`/mavros/vision_pose` 参数 `tf/listen` 必须保持 `false`。

### 3.3 为什么拒绝 `/mavros/odometry/out`

MAVROS odometry plugin 会无条件序列化 `nav_msgs/Odometry.twist` 和 twist covariance，
并发送 `ODOMETRY.vx/vy/vz`。默认初始化的 ROS twist 是有限的零；PX4 会把三个有限值
当作有效速度，若 `EKF2_EV_CTRL` velocity bit 开启即可参与融合。源码证据：

- MAVROS 设置 `LOCAL_FRD`、`BODY_FRD`、`VISION`：
  [`L276-L278`](https://github.com/mavlink/mavros/blob/c655e6343ec81687d51e7185bcba7a651e361fcd/mavros_extras/src/plugins/odom.cpp#L276-L278)；
- MAVROS 填入 velocity 和 covariance：
  [`L316-L339`](https://github.com/mavlink/mavros/blob/c655e6343ec81687d51e7185bcba7a651e361fcd/mavros_extras/src/plugins/odom.cpp#L316-L339)；
- PX4 只用 `isAllFinite()` 判断三轴速度是否存在：
  [`L1371-L1428`](https://github.com/PX4/PX4-Autopilot/blob/99c40407ffd7ac184e2d7b4b293f36f10fe561ef/src/modules/mavlink/mavlink_receiver.cpp#L1371-L1428)。

用 NaN twist 可以让该路径在源码上退化为 pose-only，但它仍增加 twist、velocity
covariance、两个 TF suffix frame 和 `ODOMETRY` frame enum 的合同面。已有专用 pose-only
消息时没有理由承担这些风险，因此首版不选 `/odometry/out`。

## 4. 坐标系转换事实

`vision_pose` 对输入执行：

- position：ROS ENU -> MAVLink/PX4 NED；
- orientation：ROS body FLU (`base_link`) -> aircraft FRD，再做 ENU -> NED；
- pose covariance：ENU -> NED；
- stamp：ROS header stamp 纳秒除以 1000，写入 MAVLink `usec`。

对应实现见
[`L127-L154`](https://github.com/mavlink/mavros/blob/c655e6343ec81687d51e7185bcba7a651e361fcd/mavros_extras/src/plugins/vision_pose_estimate.cpp#L127-L154)。

因此 gateway 必须输出 ROS 侧 `map`/`base_link`、右手 z-up/FLU 数据，禁止先手工转为
NED/FRD；否则会发生二次转换。`cuvslam_primary` 的 `map +x` 是 epoch 初始机头；
`mocap_primary` 的 `map` 等同动捕固定世界。两者都不构成地理 East/North 声明，
也不改变 ROS 边界的轴序合同。

重要限制：`vision_pose` callback 不检查 `header.frame_id`，`PoseWithCovarianceStamped`
也没有 `child_frame_id`。所以 frame 合法性只能由本项目 gateway 在发布前严格验证，
不能依赖 MAVROS 拒绝错误 frame。

## 5. 时间戳行为

### 5.1 已验证事实

- MAVROS 保留输入 header stamp，微秒化后写入 `VISION_POSITION_ESTIMATE.usec`；
- `vision_pose` 只丢弃与上一条完全相同的 stamp，不拒绝零值或时间回退；
- PX4 receiver 用 `_mavlink_timesync.sync_stamp(vpe.usec)` 生成
  `vehicle_visual_odometry.timestamp_sample`，见
  [`mavlink_receiver.cpp L1237-L1271`](https://github.com/PX4/PX4-Autopilot/blob/99c40407ffd7ac184e2d7b4b293f36f10fe561ef/src/modules/mavlink/mavlink_receiver.cpp#L1237-L1271)；
- PX4 timesync 收敛后使用 remote stamp 加估计 offset；未收敛时退化为当前
  `hrt_absolute_time()`，见
  [`Timesync.cpp L124-L132`](https://github.com/PX4/PX4-Autopilot/blob/99c40407ffd7ac184e2d7b4b293f36f10fe561ef/src/lib/timesync/Timesync.cpp#L124-L132)。

### 5.2 项目要求

gateway 必须在 MAVROS 前拒绝 zero、duplicate、nonmonotonic、future 和 stale stamp。
Gate G3 必须先证明 `/mavros/timesync_status` 稳定，再允许一次拆桨 external-vision
注入试验。`EKF2_EV_DELAY` 只用于固定延迟补偿，不能替代健康的源 stamp 或 timesync。

VRPN 当前使用 Jetson callback time，足以支持本项目首版低速定位验证，但不授权
精密光学采集延迟结论。该限制不妨碍选择 pose-only 消息。

## 6. PX4 对 pose-only 和 covariance 的处理

### 6.1 已验证事实

PX4 的空 `vehicle_odometry` 将 position、orientation、velocity 及其 variance 初始化为
NaN，并把 pose/velocity frame 初始化为 `UNKNOWN`，见
[`mavlink_receiver.cpp L93-L108`](https://github.com/PX4/PX4-Autopilot/blob/99c40407ffd7ac184e2d7b4b293f36f10fe561ef/src/modules/mavlink/mavlink_receiver.cpp#L93-L108)。

`VISION_POSITION_ESTIMATE` handler 只填 position、orientation、pose covariance 对角项
和 reset counter，然后发布 `vehicle_visual_odometry`；它没有创建 velocity。因此该
路径确实支持“有 Pose、无速度”，见
[`L1237-L1271`](https://github.com/PX4/PX4-Autopilot/blob/99c40407ffd7ac184e2d7b4b293f36f10fe561ef/src/modules/mavlink/mavlink_receiver.cpp#L1237-L1271)。

EKF2 仅在 velocity 三轴全部 finite 且 frame 可识别时创建速度观测。position 和
orientation 也各自经过 finite/frame/quaternion 检查，见
[`EKF2.cpp L2175-L2307`](https://github.com/PX4/PX4-Autopilot/blob/99c40407ffd7ac184e2d7b4b293f36f10fe561ef/src/modules/ekf2/EKF2.cpp#L2175-L2307)。

当 `EKF2_EV_NOISE_MD=0` 且消息 variance 全部 finite 时，EKF2 使用
`max(消息 variance, 参数 noise^2)`；当 variance 非 finite 或 mode 为 1 时，EKF2 使用
`EKF2_EVP_NOISE`/`EKF2_EVA_NOISE`。所以全 NaN covariance 不会被当成零 uncertainty，
而会明确触发参数噪声路径。

### 6.2 工程建议

- 首版使用 `/pose_cov` 并发送 36 个 NaN，明确声明 covariance 未知；
- Gate G3 候选配置采用参数噪声模式 `EKF2_EV_NOISE_MD=1`，但噪声具体值必须单独
  评审并从实机读回，不能直接把默认值当作已批准值；
- `EKF2_EV_CTRL` 的 velocity bit 2 必须为 0；
- 若水平位置、垂直位置和 yaw 都经 Gate G3 批准，候选 mask 才是 `1+2+8=11`；
  在此之前不批准任何具体写值；
- 因 gateway 已输出 `base_link` 且 `base_link=fcu_imu`，`EKF2_EV_POS_X/Y/Z` 候选
  必须为 `0/0/0`。把 X 写成 `0.05` 会对已经校正的相机杆臂重复补偿。

### 6.3 Gate G3 待补证据

实现 gateway 后，必须用当前 Jetson MAVROS binary 和 PX4 固件证明：ROS covariance
的 36 个 NaN 经 MAVROS ENU/NED covariance 变换后，PX4 收到的
position/orientation variance 仍为 non-finite，并实际采用参数噪声。源码结论不能
替代这条运行证据。

## 7. 最小 PX4 参数只读审计集

以下参数只允许读取和封存；本报告不授权 `param set`。

| 参数 | 必须确认的内容 |
| --- | --- |
| `EKF2_EV_CTRL` | 实际启用的 EV 字段；velocity bit 2 必须关闭 |
| `EKF2_EV_DELAY` | 当前固定延迟补偿值和单位 ms |
| `EKF2_EV_NOISE_MD` | 消息 variance 或参数噪声策略 |
| `EKF2_EVP_NOISE` | 未知/低估位置 covariance 时的 noise/lower bound |
| `EKF2_EVA_NOISE` | 未知/低估角度 covariance 时的 noise/lower bound |
| `EKF2_EVP_GATE` | EV position innovation gate |
| `EKF2_EV_QMIN` | VPE 路径没有有效 quality 指标时是否会阻断融合 |
| `EKF2_EV_POS_X/Y/Z` | 杆臂；本项目 base pose 候选为 `0/0/0` |
| `EKF2_HGT_REF` | 高度参考源及与 EV vertical 的关系 |
| `EKF2_GPS_CTRL` | 室内 GNSS 是否仍作为竞争 aiding source |
| `EKF2_BARO_CTRL` | baro height aiding 是否启用 |
| `EKF2_RNG_CTRL` | range height aiding 是否启用 |
| `EKF2_MAG_TYPE` | magnetometer yaw 与 EV yaw 的组合 |
| `EKF2_NOAID_TOUT` | 最后一次约束速度漂移的融合后，水平导航失效超时 |
| `EKF2_EVV_NOISE` | 记录速度噪声配置，证明即使存在也不代表速度获批 |
| `EKF2_EVV_GATE` | 同上；velocity fusion 必须由 `EKF2_EV_CTRL` 明确关闭 |

参数名称、bit 定义和噪声策略均来自 PX4 v1.15.4
[`module.yaml`](https://github.com/PX4/PX4-Autopilot/blob/99c40407ffd7ac184e2d7b4b293f36f10fe561ef/src/modules/ekf2/module.yaml#L105-L680)。
读取证据必须包含 `/mavros/param` 标准 ROS 2 parameter API 的每项成功结果、
整数/实数值、同一次 `/mavros/vehicle_info_get` 返回的 PX4 revision、时间和
SHA-256；缺项或读取失败不得用默认值补齐。

## 8. Gate G3 运行时待验证项

### 8.1 只读 endpoint 证据

默认 launch 下，`ros2 topic info -v` 必须验证：

- `/mavros/state` publisher 是 Node `sys`、namespace `/mavros`；
- `/mavros/timesync_status` publisher 是 Node `time`、namespace `/mavros`；
- `/mavros/local_position/odom` publisher 是 Node `local_position`、namespace `/mavros`；
- `/mavros/vision_pose/pose_cov` 恰有一个 MAVROS subscription，Node `vision_pose`、
  namespace `/mavros`、类型完全匹配，且 Gate G3 前 publisher count 为 0；
- `/mavros/odometry/out` 的 MAVROS subscriber 是 Node `odometry`，但 gateway publisher
  必须为 0；
- `/mavros/vision_pose/pose`、`/mavros/mocap/pose` 和其他 external-vision 候选入口的
  非授权 publisher 必须为 0；
- `/mavros/vision_pose` 必须对 `tf/listen` 明确返回 `false`；未设置、无法读取或
  返回 `true` 均不通过；
- `ROS_DOMAIN_ID=42` 且 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`。

校验必须在同一个 endpoint block 内同时匹配 Node name、namespace、方向和 type，
并独立核对 topic 的全局 publisher count；禁止只用输出中的独立字符串拼接判定通过。

在 Jetson 宿主机、MAVROS 已连接且 PX4 保持未解锁时执行：

```bash
cd /home/nvidia/workspaces/isaac_ros_3_2/src/TF_ROS_CUsalm

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash

set -o pipefail
bash tools/collect_mavros_px4_external_vision_audit.sh \
  | tee /home/nvidia/mavros_px4_external_vision_audit_20260724_v2.txt
AUDIT_RC=${PIPESTATUS[0]}

sha256sum /home/nvidia/mavros_px4_external_vision_audit_20260724_v2.txt \
  | tee /home/nvidia/mavros_px4_external_vision_audit_20260724_v2.SHA256

echo "audit_exit_code=$AUDIT_RC"
```

只有 `errors=0`、`analysis=READ_ONLY_AUDIT_COMPLETE` 且 `audit_exit_code=0` 才能
作为完整的只读运行时证据。`warnings` 必须逐项解释；选定的
`/mavros/vision_pose/pose_cov`、PX4 revision 或必要参数等关键缺口会计入 error 并
返回退出码 2。未选用插件 endpoint 不存在可以记为 warning，但其输入 topic 一旦
存在非授权 publisher，必须计入 error。

#### 2026-07-24 首次运行

| 字段 | 结果 |
| --- | --- |
| 时间 | `2026-07-24T00:51:40+00:00` |
| 输出 SHA-256 | `d520e92ad3d95e5b66f8991f053ca44a189b9165655779de8e77af2de6318cd9` |
| endpoint | 所有必需与候选 MAVROS endpoint 的 node、namespace、type 和方向均通过 |
| PX4 状态 | connected、disarmed、`STABILIZED` |
| timesync | 样本可读，round-trip `1.301682 ms` |
| 结果 | `INCOMPLETE`，`errors=2`，`audit_exit_code=2` |

两个 error 都属于采集脚本与 MAVROS 2.14 ROS 2 参数 API 的兼容性错误，不是
flight stack 故障：脚本使用了错误的 `tf.listen` 名称，而运行时实际名称是
`tf/listen`；脚本还查询了已废弃且未提供的 `/mavros/param/get` service，而安装的
接口定义明确要求 ROS 2 parameter API。修订版改为从 `/mavros/param` 只读参数。
该次输出只保留为修复前证据，不能用于关闭 `YP-200`。

#### 2026-07-24 修订版 clean rerun

| 字段 | 结果 |
| --- | --- |
| 文件 | `/home/nvidia/mavros_px4_external_vision_audit_20260724_v2.txt` |
| 输出 SHA-256 | `6778c559bb7203bc29ae18f653da287ecd55371fd25681a72fb96350f3f2c66d` |
| MAVROS/PX4 | MAVROS `2.14.0`；PX4 revision `99c40407ff000000` |
| PX4 状态 | connected、disarmed、`STABILIZED` |
| TF 输入 | `/mavros/vision_pose` 的 `tf/listen=false` |
| endpoint | node、namespace、type、方向和 publisher authority 全部通过 |
| timesync | 样本 round-trip `1.470744 ms` |
| 写操作 | parameter/mode/arming/OFFBOARD 写操作均为 0 |
| 结果 | `errors=0`、`warnings=0`、`READ_ONLY_AUDIT_COMPLETE`、exit `0` |

同一次只读审计取得的 PX4 参数如下。分类 `OBSERVED_ONLY` 表示实机现状已经固定，
不是参数写入建议或运行授权。

| 参数 | 实际值 | 审计结论 |
| --- | ---: | --- |
| `EKF2_EV_CTRL` | `11` | `0b1011`；position、height、yaw bit 已置位，velocity bit `0b0100` 未置位 |
| `EKF2_EV_DELAY` | `0.0` | 当前固定延迟；Gate G3 仍需结合 stamp/timesync 验证 |
| `EKF2_EV_NOISE_MD` | `0` | `PREEXISTING_OBSERVED_ONLY`；Gate G3 候选 `1` 仍待独立评审 |
| `EKF2_EV_QMIN` | `0` | `OBSERVED_ONLY`；Gate G3 验证 VPE quality 行为 |
| `EKF2_EV_POS_X/Y/Z` | `0.0/0.0/0.0` | 与已去除 50 mm 相机杆臂的 `base_link` pose 一致 |
| `EKF2_EVP_NOISE` | `0.10000000149` | `OBSERVED_ONLY` |
| `EKF2_EVP_GATE` | `5.0` | `OBSERVED_ONLY` |
| `EKF2_EVV_NOISE` | `0.10000000149` | 已记录但 velocity fusion 未启用 |
| `EKF2_EVV_GATE` | `3.0` | 已记录但 velocity fusion 未启用 |
| `EKF2_EVA_NOISE` | `0.10000000149` | `OBSERVED_ONLY` |
| `EKF2_HGT_REF` | `3` | Vision height reference；首次注入 reset/fallback 留给 Gate G3 |
| `EKF2_GPS_CTRL` | `7` | `OBSERVED_ONLY`；与其他 aiding source 的竞争留给 Gate G3 |
| `EKF2_BARO_CTRL` | `1` | `OBSERVED_ONLY` |
| `EKF2_RNG_CTRL` | `2` | `OBSERVED_ONLY` |
| `EKF2_MAG_TYPE` | `0` | `OBSERVED_ONLY` |
| `EKF2_NOAID_TOUT` | `5000000` | `OBSERVED_ONLY` |

`EKF2_EV_CTRL=11` 只证明当前 velocity bit 已关闭，不批准 external-vision 发布。
`EKF2_EV_NOISE_MD=0` 下，源码仍会在消息 variance 非 finite 时回退到参数噪声；
全 NaN 是否经过当前 MAVROS binary 保持 non-finite，必须由 Gate G3 实测，不能用
本次只读证据代替。所有 parameter approval 继续为 `NONE`。

### 8.2 拆桨、人工批准后的注入证据

后续单独批准的 Gate G3 试验必须保持 disarmed、非 OFFBOARD、拆桨，并证明：

- 全 NaN covariance 在 PX4 `vehicle_visual_odometry` 侧仍表达 unknown；
- velocity 为 NaN/UNKNOWN，且 EKF2 没有 EV velocity fusion；
- ENU/NED、FLU/FRD 的 x/y/z/yaw 符号与手动位移一致；
- source stamp、timesync、`EKF2_EV_DELAY` 和 innovation 时间关系可解释；
- 丢包、stale、frame 错误、重复 publisher、定位 reset 或 epoch 改变时 gateway
  立即停止新输出；
- PX4 innovation、reset、local position validity 和 `/mavros/local_position/odom`
  没有未解释跳变。

## 9. 权限和失败边界

- source selector 只能在独立 launch 中选择 `mocap_primary` 或 `cuvslam_primary`；
  运行中禁止自动切换；
- external-vision gateway 是 MAVROS 输入 topic 唯一允许的 publisher；
- `vision_pose` 不校验 frame、stale 或 publisher identity，这些检查必须由 gateway
  完成；
- MAVROS 的 `VISION_POSITION_ESTIMATE` 路径不能从 ROS 消息传递项目 epoch 语义，
  定位 reset/时间回退必须结束 epoch 并撤销 gateway authority；
- timesync 未收敛、PX4 参数未知、covariance NaN 透传未验证、source stale、frame
  不符或 publisher 冲突时一律 fail closed；
- 停止 external-vision 发布不等于可以自动切换定位源、自动进入其他 PX4 mode 或
  自动降落；后续动作仍由独立安全门禁和飞手决定；
- 本报告不发布任何 external vision 或控制消息，不修改 PX4 参数，不请求 OFFBOARD，
  不解锁，也不允许节点重启后自动恢复任何 authority。

## 10. 审计结论

源码和只读运行证据已经证明 MAVROS 2.14/PX4 v1.15.4 支持真正的 pose-only 外部
视觉输入。首版选择 `/mavros/vision_pose/pose_cov`，用全 NaN covariance 表达
“未知”；PX4 源码在 variance 非 finite 时定义参数噪声回退。实机参数只读值已经
归档，但没有获得配置批准；速度字段天然缺失，当前 `EKF2_EV_CTRL=11` 的 velocity
bit 已关闭。

裸 `/mavros/vision_pose/pose` 因 MAVROS 自动生成全零 covariance 被否决；
`/mavros/odometry/out` 因默认零 twist 会伪造速度且合同面更大而被否决。接口审计
任务已经关闭；完成 NaN 透传和其他 Gate G3 台架证据前，external-vision output
authority 继续保持 `DENIED`。
