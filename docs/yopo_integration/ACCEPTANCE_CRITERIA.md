# YOPO 实机集成验收标准

## 1. 判定规则

每个 Gate 只能产生 `PASSED` 或 `FAILED`。条件不完整、证据缺失或仍需人工判断时，
保持 `BLOCKED` 或 `IN_PROGRESS`，不得标记通过。

大型 rosbag 不提交 Git。验收提交只保存摘要、统计、版本、路径和 SHA-256。

## 2. Gate G0：需求与版本基线

通过条件：

- 本目录全部文档完成复核；
- 新建 `feat/yopo-integration-v1`；
- 当前定位证据分支 revision 被固定；
- YOPO、bringup、MAVROS、PX4 revision/版本被记录；
- 未决接口在 `TASKFLOW.md` 有 owner 和阻塞条件。

## 3. Gate G1：构建、lint 与单元测试

通过条件：

- 所有新增 ROS 2 package 在目标 Jetson 上干净构建；
- 单元测试、cpplint、uncrustify、cppcheck、CMake lint 和 XML lint 无失败；
- frame 组合、外参方向、发布权冲突、stale 和 reset 有自动化测试；
- 两个 mode 的 launch 测试证明无交叉 authority；
- 测试结果绑定 commit hash。

`YP-220` selector module 还必须证明：

- `LocalizationSourceCandidate` 与 `SelectedPoseCandidate` schema 逐字段匹配 YP-220
  合同，且两者都没有 twist/covariance；
- `cuvslam_primary` 和 `mocap_primary` 各自只创建选定来源的 pose subscription；
- cuVSLAM 初始 position/yaw 只对齐一次，初始 roll/pitch 不会倾斜 `map` z 轴；
- mocap 的全局 position/yaw 必须原样保留，世界变换必须为 identity；
- mode、publisher GID、source reset 或时间回退不能触发重新对齐或切换来源；
- graph 中不存在来自 selector 的 odometry、TF、MAVROS、YOPO 或控制 publisher；
- diagnostics 在 starting、healthy、stale 和 latched fault 状态持续可用。

这些条件可以使用 synthetic `LocalizationSourceCandidate` 完成 module 验收。完整
`cuvslam_primary` 与 `mocap_primary` bringup、非主源 shadow 并行和跨 module authority
属于 `YP-250`，不属于 `YP-220` 的通过前置条件。

## 4. Gate G2：D435 深度与 cuVSLAM 联合运行

通过条件：

- 红外双目保持 `640x360@90 Hz`；
- cuVSLAM tracking odometry 持续不低于已验证运行下限；
- 原生深度 topic 类型、frame、encoding 和单位明确；
- 深度流持续可用，YOPO 能正确处理实际 encoding；
- 发射器关闭条件下记录深度有效率；
- 无 USB 重连、CUDA OOM 或持续热降频；
- 若执行 emitter A/B，两个配置分别留存完整证据。

## 5. Gate G3：定位 selector 与 PX4 外部视觉台架

通过条件：

- `cuvslam_primary` 和 `mocap_primary` 分别启动成功；
- mode 启动后不可改变；
- selector 只发布一个 `SelectedPoseCandidate` pose seam，且自身没有
  `/localization/odometry`、TF 或 MAVROS publisher；
- gateway 首版不创建 `/localization/odometry`、TF、YOPO 或控制 publisher；
- packaged 默认 disabled gateway 的 MAVROS publisher count 必须为零；
- active Gate G3 合同安装前，必须封存不少于 60 秒的 MAVROS state/timesync 证据，
  并据此批准 freshness、RTT 和 offset jitter 阈值；
- 非主定位源可以继续发布 source-private candidate 或 shadow evidence，但不得获得
  selected、gateway、PX4 external-vision 或控制 authority；
- PX4 外部视觉输入最多一个 publisher；
- MAVROS/PX4 frame 转换和可用字段经源码与消息实测固定；
- 动捕路径不伪造速度；
- PX4 EKF 创新、reset 和本地状态输出满足台架判据；
- 停止主定位源时 gateway 撤销输出且不会自动切换。

本 Gate 未通过前，不允许使用 PX4 融合状态驱动实机 YOPO 控制。

## 6. Gate G4：YOPO 被动规划

连续运行至少 10 分钟，且：

- `output_enabled=false`；
- 有效推理频率不低于 15 Hz；
- 深度和状态无持续 stale；
- 不出现 NaN、Inf、模型异常、CUDA OOM 或进程退出；
- 候选轨迹持续发布且数值位于配置边界；
- YOPO 不发布有效 PositionCommand、姿态目标或 PX4 控制；
- rosbag、diagnostics、CPU/GPU 温度、内存和频率统计完整；
- 使用录包或明确标记的 `shadow/test` 状态，不绕过未通过的 PX4 融合门禁。

## 7. Gate G5：拆桨 SO3 与 MAVROS 联调

拆除所有螺旋桨，连续运行至少 5 分钟，且：

- YOPO PositionCommand 目标频率为 50 Hz；
- SO3 和 MAVROS 控制流无未解释的长时间断流；
- 姿态、角速度、推力、速度和加速度限幅生效；
- 软件不能自动解锁；
- 深度断开、主定位停止和 YOPO 停止三种故障注入均触发预期行为；
- PX4 OFFBOARD loss 参数和实际行为一致；
- backend type mask、推力归一化和 frame 符号经过记录和复核。

## 8. Gate G6：`mocap_primary` 低速飞行

必须依次完成：

1. 人工解锁和模式切换；
2. 起飞至安全高度；
3. 稳定悬停；
4. 低速直线往返；
5. 安全降落；
6. 定位、控制和 PX4 日志复核。

速度、高度和目标距离不得超过首版包络。cuVSLAM 在整个过程中只做 shadow。

## 9. Gate G7：`cuvslam_primary` 空场飞行

必须完成：

- 起飞、悬停和降落；
- 前进、横移和受限偏航；
- 多个不超过 3 m 的目标点；
- 全程满足 0.5 m/s、1.0 m/s^2 和 2.5 m 上限；
- 动捕 shadow 对比没有发现未解释的轴向、尺度、跳变或持续漂移异常；
- 整个飞行期间没有定位源切换。

## 10. Gate G8：静态障碍避障

按顺序完成：

1. 单个宽度至少 0.5 m 的大型静态障碍物；
2. 两侧保留充足空间的多障碍或宽走廊场景。

最小规划安全间距为 0.5 m。首版不接受移动障碍、人员附近、窄缝或高速测试。
最终项目完成判定以 `cuvslam_primary` 通过本 Gate 为准。

## 11. Gate G9：证据封存

通过条件：

- 所有 Gate 的结果与 revision 一一对应；
- rosbag 和报告 hash 校验通过；
- PX4 参数、MAVROS 版本和硬件配置已归档；
- `TASKFLOW.md` 没有未解释的 `FAILED` 或与飞行相关的 `BLOCKED`；
- 文档明确保留首版排除项，不把阶段结果外推到更高速度或其他场地。
