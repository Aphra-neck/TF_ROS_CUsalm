# YP-230 pose-only localization output gateway 合同

状态：`IN_PROGRESS`

```text
external_vision_output_authorization=DENIED
offboard_authorization=DENIED
arming_authorization=DENIED
flight_authorization=DENIED
```

本合同固定 `localization_output_gateway` 的首版 interface 和 fail-closed 行为。
它不构成 Gate G3、OFFBOARD、解锁、控制或飞行授权。

## 1. 责任和非责任

Gateway 只负责：

- 接收 `/localization/selected/pose` 的 `SelectedPoseCandidate`；
- 验证 selector publisher、GID、QoS、mode、contract、epoch、frame、stamp 和 Pose；
- 在独立 Gate G3 合同明确获批后，逐条生成
  `/mavros/vision_pose/pose_cov` 的 `PoseWithCovarianceStamped`；
- 持续发布结构化 diagnostics，并在任何合同破坏时 fail closed。

Gateway 明确不负责：

- 自动选择或切换定位源；
- map 对齐、相机外参或动捕刚体变换；
- Pose 差分、速度估计、插值或最后样本重放；
- `/localization/odometry`、TF、`/state/odom` 或控制输出；
- PX4 参数写入、模式切换、OFFBOARD、解锁或降落。

## 2. 固定 ROS seam

| 方向 | Topic | Type | 权限 |
| --- | --- | --- | --- |
| 输入 | `/localization/selected/pose` | `localization_adapter_interfaces/msg/SelectedPoseCandidate` | `selected_pose_candidate_only` |
| 诊断输出 | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 始终允许，不表示飞行授权 |
| Gate G3 候选输出 | `/mavros/vision_pose/pose_cov` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 当前 `DENIED` |

输入必须精确表达 `map -> base_link`。允许的 mode 只有 `cuvslam_primary` 和
`mocap_primary`，并分别绑定版本化 selector/source contract ID。首个有效消息绑定
非空 `localization_epoch_id`；mode、任一 contract ID、epoch 或 publisher GID 变化
均结束当前 gateway epoch 并锁存故障。

## 3. 输出语义

未来 Gate G3 候选输出必须满足：

- header stamp 逐字段继承 selected 输入，不使用回调时间替换；
- `header.frame_id` 保持 `map`；
- Pose 数值和四元数保持 ROS z-up/FLU 语义；
- 不执行 ENU/NED、FLU/FRD 转换，该转换只由 MAVROS 完成；
- 不再次应用 50 mm 相机外参；
- 36 个 pose covariance 元素全部为 IEEE-754 quiet NaN；
- 每个获批输入最多产生一个输出，不限频、不重排、不补发。

`PoseWithCovarianceStamped` 不携带 epoch 字段，因此 gateway 必须在进程内锁定 epoch，
并把 epoch 和 contract ID 写入 diagnostics。进程重启不能在后台自动恢复 authority。

## 4. 默认 disabled profile

零参数默认 launch 使用版本化 `disabled` 合同：

- 创建 selected pose subscription 和 diagnostics publisher；
- 不创建 MAVROS、Odometry、TF、YOPO 或控制 publisher；
- 不提供 enable、reset、switch、publish service/action；
- `contract_file` 为只读参数；node、topic 和 namespace remap 全部禁止；
- diagnostics 明确报告 `external_vision_output_authorization=denied`。
- disabled 状态的固定 `reason_code` 是 `OUTPUT_AUTHORIZATION_DENIED`；

Gate G3 active 合同在阈值获批前只能存在于测试 fixtures，不能安装生产 launch。
active profile 不是一个布尔参数，也不能从 disabled profile 运行时切换。

## 5. 状态和故障

disabled profile 是稳定的 `disabled` 状态，只验证 selected seam 并发布 diagnostics，
没有通往 active profile 的运行时转换。以下状态机只适用于未来显式启动的 Gate G3
active profile：

```text
active_starting -> active_healthy -> latched_fault
       |                |
       +----------------+
```

- `active_starting`：等待唯一合法 input/MAVROS endpoint、PX4 未解锁状态和 timesync
  健康；
- `active_healthy`：仅在显式 Gate G3 合同下创建一次 MAVROS publisher 并逐条发布；
- `latched_fault`：立即停止新输出，销毁 privileged publisher，禁止自动恢复。

zero、duplicate、nonmonotonic、future、stale、frame/contract/epoch/GID 变化、重复
publisher、MAVROS endpoint 异常、timesync 不健康或 PX4 状态不允许，都必须停止
输出。锁存只能通过人工停止并重新启动完整 launch 清除。

## 6. 尚待数据固定的阈值

以下数值不得从 selector 或单个 timesync 样本推断：

- selected pose 最大 age、future tolerance 和 stale 阈值；
- MAVROS state/timesync freshness；
- timesync 最小样本数、窗口长度、RTT、offset jitter 和 drift；
- graph startup grace、四元数 norm 容差和 diagnostics 周期；
- 若需要 pose reset gate，其平移和旋转阈值。

在 MAVROS 已连接、PX4 未解锁且不发布 external vision 时，只读采集不少于 60 秒
state/timesync，再形成独立阈值证据和版本化 active 合同。

## 7. TDD seam 和验收顺序

测试只通过公开 ROS seam、版本化 config 和 diagnostics 观察行为：

1. packaged 默认 launch 存在，节点可发现；
2. 默认 graph 只有 selected subscription 和 diagnostics publisher；
3. 默认 graph 的 MAVROS、Odometry、TF、YOPO 和控制 publisher count 为零；
4. 严格 config parser 拒绝未知 key、重复 key、错误 profile 和 contract identity；
5. input FQN/type/QoS/GID 和字段 tuple 精确匹配；
6. stamp、Pose、epoch 和故障锁存测试通过；
7. test-only active fixture 证明 stamp/Pose 原样复制且 covariance 36/36 为 NaN；
8. 60 秒只读证据固定健康阈值；
9. 人工批准后才安装 Gate G3 bench contract 和 launch；
10. 拆桨、disarmed、非 OFFBOARD 条件下完成 Gate G3 运行证据。

当前 TDD slice 只交付第 1--3 项的红测试，不实现 active output。

## 8. YP-230 通过条件

只有下列条件全部满足后，`TASKFLOW.md` 才能把 `YP-230` 标记为 `PASSED`：

- 默认 disabled launch、全部单元/graph/authority/lifecycle/lint 测试通过；
- 60 秒 state/timesync 阈值证据和 SHA-256 已封存；
- Gate G3 active contract 获得单独人工批准并绑定 revision；
- MAVROS/PX4 拆桨台架证明 NaN covariance 保持 unknown、velocity 保持 absent；
- frame/stamp、唯一 publisher、停止输出和锁存故障行为全部实测通过；
- 没有 OFFBOARD、解锁、控制或飞行授权被隐式产生。
