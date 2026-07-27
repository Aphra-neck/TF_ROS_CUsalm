# YP-220 启动时定位源 selector 合同

日期：2026-07-24

```text
status_source=TASKFLOW.md
design=OPTION_A_SELECTED_POSE_SEAM
selection_time=LAUNCH_ONLY
automatic_source_switching=FORBIDDEN
selected_output=/localization/selected/pose
selected_type=localization_adapter_interfaces/msg/SelectedPoseCandidate
selected_pose_semantics=map_to_base_link
twist=ABSENT
covariance=ABSENT
canonical_odometry_authority=DENIED
mavros_authority=DENIED
tf_authority=DENIED
control_authority=DENIED
```

本合同固定 `YP-220` 的方案 A：`localization_source_selector` 是启动时数据选择
module。它只订阅当前 launch 明确选定的定位源，在 module 内应用模式固定的 `map`
语义：cuVSLAM 首帧建立局部世界，mocap 使用固定 identity 世界变换；随后通过一个
小而类型受限的 interface 发布 pose candidate。

selector 不是定位融合器、速度估计器或 PX4 gateway。本合同只允许开始实现和测试，
不构成 Gate G3、外部视觉、OFFBOARD、解锁、控制或飞行授权。

## 1. Module、interface 与 seam

下游唯一数据 seam 是：

```text
/localization/selected/pose
  localization_adapter_interfaces/msg/SelectedPoseCandidate
```

调用方只需要理解一个 `map -> base_link` pose candidate interface。来源特有的输入
topic、原始世界 frame、50 mm 相机外参、动捕刚体语义、输入 publisher identity、
时间和健康门禁都隐藏在 selector implementation 及其版本化 mode 配置中。

删除 selector 后，启动时互斥、来源合同校验、模式变换、epoch 锁定和 fail-closed
行为会重新散落到 gateway 与 launch，因此该 module 不是简单 topic pass-through。

## 2. 启动模式

只接受两个精确值：

| Mode | 唯一可创建的 pose subscription | 禁止创建的 pose subscription |
| --- | --- | --- |
| `cuvslam_primary` | cuVSLAM `LocalizationSourceCandidate` | 动捕 candidate 和原始 VRPN pose |
| `mocap_primary` | 动捕 `LocalizationSourceCandidate` | cuVSLAM candidate 和原始 cuVSLAM odometry |

每个 mode 的 source topic、type、expected publisher FQN、contract ID、frame、QoS 和
健康阈值必须由版本控制配置固定。`LocalizationSourceCandidate` 必须已经由对应
adapter 转换为 `T[source_world,base_link]`；selector 不得再次应用
`T[base_link,camera_link]` 或 `T[base_link,mocap_rigid_body]`。

输入 interface 精确固定为
`localization_adapter_interfaces/msg/LocalizationSourceCandidate`：

```text
std_msgs/Header header
string semantic_child_frame
geometry_msgs/Pose pose
string source_id
string source_contract_id
string authorization
```

`source_id` 只允许 mode 配置中精确绑定的 `cuvslam` 或 `mocap`；
`semantic_child_frame` 必须为 `base_link`。输入 `authorization` 必须精确为
`source_pose_candidate_only`；它只声明 source-private candidate 资格，不携带选择、
gateway、MAVROS 或飞行授权。selector 必须同时校验消息字段和实际 topic publisher
authority，不能信任消息的自声明字符串。任何其他值都必须拒绝且不得产生 selected
output。

mode 必须是 read-only 启动参数。禁止 application-level mode service、切换 action、
动态参数切换、自动 fallback 或“当前源不健康时订阅另一源”。完整的
`cuvslam_primary` 和 `mocap_primary` 系统 launch 仍属于 `YP-250`；`YP-220` 只交付
selector module、消息 interface、模式配置和隔离测试。

selector 必须显式 allowlist 当前两个 adapter 的 source contract ID。adapter 只能产出
authorization 为 `source_pose_candidate_only` 的 `LocalizationSourceCandidate`；只有
selector 才能产出 authorization 为 `selected_pose_candidate_only` 的
`SelectedPoseCandidate`。这两个值都不包含 canonical localization、PX4 external
vision 或飞行授权。未知、空或占位 source contract ID 必须使 selected output 保持
为零；external-vision allowlist 和 Gate G3 授权属于 `YP-230`。

## 3. `SelectedPoseCandidate` interface

目标消息 schema 固定为不包含 twist 和 covariance 的类型受限消息：

```text
std_msgs/Header header
  stamp: 继承选定来源的采样时间
  frame_id: map
string semantic_child_frame  # 必须为 base_link
geometry_msgs/Pose pose      # T[map,base_link]
string selector_contract_id
string localization_epoch_id
string mode
string source_contract_id
string authorization
```

interface 的完整不变量如下：

- topic 固定为 `/localization/selected/pose`，禁止 remap；
- 唯一 publisher 必须是 `/localization_source_selector`；
- 每个输入样本最多产生一个输出，不得定时重发最后 pose；
- `header.stamp` 原样继承输入采样时间，不替换为 callback 或 publish time；
- `header.frame_id` 必须为 `map`，`semantic_child_frame` 必须为 `base_link`；
- `selector_contract_id`、`source_contract_id`、`localization_epoch_id`、`mode` 和
  `authorization` 在一次进程生命周期内不可改变；
- 消息没有 twist、twist covariance 或 pose covariance 字段；
- 消息类型必须与 `nav_msgs/msg/Odometry`、MAVROS pose 输入和 TF 不兼容；
- `mode` 必须为当前 read-only 启动值；
- `authorization` 必须精确为 `selected_pose_candidate_only`；它只表示“经启动选择并
  应用当前模式固定世界语义的 pose candidate”，不携带 PX4、控制或飞行 authorization。

未知 mode、空 contract ID、空 epoch ID、错误 authorization、错误 frame、非有限
position、无效 quaternion 或时间合同失败时不得发布。首版 `selector_contract_id`
按启动模式精确绑定配置：

- `cuvslam_primary`：`yopo_cuvslam_primary_selector_20260724_v1`；
- `mocap_primary`：`yopo_mocap_primary_selector_20260727_v2`。

两个 ID 分别绑定各自的 input topic、publisher、source contract 和 parent frame，禁止
跨模式复用。

## 4. 模式固定的 `map` 语义

`cuvslam_primary` 的首个有效 `T[odom,base_link]` 样本建立局部 epoch：

```text
yaw_map_from_odom = -yaw(T[odom,base_link]_initial)
R[map,odom] = Rz(yaw_map_from_odom)
t[map,odom] = -R[map,odom] * p[odom,base_link]_initial

T[map,base_link] = T[map,odom] * T[odom,base_link]
```

只使用初始 yaw 建立世界旋转，不把初始 roll/pitch 固化到 `map` z 轴。初始位置变为
`[0,0,0]`，初始机头变为 `map +x`；后续 pose 仍保留来源测得的 roll/pitch。

`mocap_primary` 使用动捕定义的固定全局世界，不从首帧推导任何平移或 yaw：

```text
T[map,mocap_world] = identity
T[map,base_link] = T[mocap_world,base_link]
```

因此动捕报告 `[2,4,0]` 时，selected pose 也必须报告 `[2,4,0]`，并保留绝对 yaw。

`T[map,S]` 计算一次后必须锁定。selector 不得持续重新对齐、平滑修改原点或因短时
stale 改用另一来源。selector 进程存活期间，以下事件结束当前 epoch 并锁存停止
selected output：

- mode 值、source contract ID 或 source topic 改变；
- source publisher GID 改变、出现重复 publisher 或 identity/type/QoS 不匹配；
- source reset、时间回退或不能解释的 pose 跳变。

selector 进程终止即结束当前 epoch；进程本身不声明能够跨重启保留锁存状态。只有
人工停止并重新启动完整 localization launch 才能创建新 epoch 和执行新的一次对齐，
禁止由进程管理器自动重启单个 selector 来恢复 authority。进程内的短时恢复必须保持
同一 mode、同一 source epoch 和同一锁定变换，并满足配置的连续健康样本数；不能在
运行中自动创建第二个 epoch。

## 5. 发布权与禁止接口

selector 只允许发布：

```text
/localization/selected/pose
/diagnostics
```

selector 明确没有以下 publisher authority：

- `/localization/odometry` 或 `/state/odom`；
- `/mavros/vision_pose/pose_cov` 及任何 `/mavros/*` 输入；
- `/tf` 或 `/tf_static`；
- YOPO、SO3、姿态、推力、模式、解锁或其他控制 topic。

`YP-230` 后续只能把 `SelectedPoseCandidate` 作为 pose-only 输入，再独立执行 gateway
frame、时间、publisher、covariance 和输出授权门禁。`YP-220` 通过不能提前创建上述
publisher，也不能绕过 Gate G3。

## 6. Diagnostics

selector 必须持续发布结构化 diagnostics，至少包含：

- mode、source ID、selector/source contract ID 和 localization epoch ID；
- expected/actual input topic、type、publisher FQN、GID 和 QoS；
- input frame、semantic child frame、source stamp 和 receive age；
- alignment state，以及锁定的 yaw 和 translation；
- received、accepted、published、rejected、stale、duplicate、nonmonotonic、frame、
  nonfinite、quaternion、reset 和 publisher-authority counters；
- selected output publisher count、authorization 和最后故障 reason code。

diagnostics 可以说明 candidate 健康，但不得声明 MAVROS、OFFBOARD、解锁、控制或
飞行已获授权。

## 7. `YP-220` 验收范围

`TASKFLOW.md` 只有在下列条件同时满足后才能把 `YP-220` 标记为 `PASSED`：

- `LocalizationSourceCandidate` 和 `SelectedPoseCandidate` schema 与本合同逐字段一致，
  且自动化测试证明两者不存在 twist/covariance 字段；
- 两个 mode 分别只创建选定来源的 pose subscription；
- 未选来源持续健康时也不能产生交叉 subscription 或切换；
- 前进、左移、上升和 yaw 输入证明输出为正确的 `map -> base_link`；
- cuVSLAM 初始非零 position/yaw 被一次对齐，初始 roll/pitch 不会倾斜 `map` z 轴；
- mocap 的绝对 position/yaw 原样保留，且 diagnostics 报告 identity 世界变换；
- 后续样本不能修改锁定的 `T[map,S]`；
- mode 修改、source reset、GID 改变、重复 publisher、stale、frame 和时间错误均
  fail closed；
- graph 测试证明 selector 没有 odometry、MAVROS、TF、YOPO 或控制 publisher；
- diagnostics、build、单元测试、launch 测试和 lint 全部通过并绑定 commit hash。

本任务不要求启动完整 RealSense/cuVSLAM/VRPN/MAVROS/PX4 双模式系统，也不执行
external-vision 注入。两个完整 primary launch、跨 module publisher authority 和
非主来源 shadow 并行行为由 `YP-250` 验收。
