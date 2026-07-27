# YOPO 实机集成决策记录

所有决策均在 2026-07-23 的需求访谈中确认。后续变更不得重写历史条目；应新增
带日期的新条目并明确替代关系。

## D-001：最终交付是实机闭环飞行

**决策：** 最终完成标准是 cuVSLAM 主定位下的室内低速 YOPO 避障飞行，不是
节点启动、模型推理或台架输出。

**影响：** 任务流必须覆盖定位、深度、YOPO、SO3、MAVROS、PX4 和实飞证据。

## D-002：两个互斥定位模式

**决策：** 提供 `cuvslam_primary` 和 `mocap_primary`，只允许启动时选择，明确禁止
飞行中自动切换。

**理由：** 自动切换会引入世界原点、yaw 和 estimator reset 的不可控跳变。

## D-003：每个 epoch 使用起飞局部 map

**决策：** 起飞位置归零，初始机头为 +x，左 +y，上 +z。对齐只计算一次并锁定。

**理由：** 首版不依赖地理定向，也不要求没有动捕时恢复固定实验室 map。

## D-004：安装参考点和相机外参固定

**决策：** `base_link`、`fcu_imu` 和 `mocap_rigid_body` 重合；相机在其前方
50 mm，旋转为 identity。

**影响：** cuVSLAM Pose 必须执行完整刚体变换，不再把外参标记为未知占位值。

## D-005：首版保留 YOPO legacy bridge

**决策：** 先用 C++ bridge 生成 YOPO 当前需要的世界系速度 `/state/odom`，不在
同一阶段修改 YOPO 网络和轨迹代码。该接口后续计划删除。

**理由：** 缩小首版改动面，同时保持标准 `/localization/odometry` 不被污染。

## D-006：所有新增运行代码使用 ROS 2 Humble

**决策：** 不引入 `ros1_bridge`。SO3 只移植生产必需部分到 ROS 2 C++。

## D-007：深度与 cuVSLAM 并行运行

**决策：** 红外双目保持 `640x360@90 Hz`，开启原生深度，YOPO 推理 15--30 Hz，
首轮关闭 emitter。

**影响：** 深度开启属于必须重新验证的传感器配置变更。

## D-008：PX4 EKF 提供最终完整状态

**决策：** 动捕和 cuVSLAM 提供外部 Pose；PX4 EKF 与 IMU 融合并提供飞行时的
位置、姿态和速度。坐标仓库不新增速度估计器。

## D-009：SO3 与 PX4 的职责分层

**决策：** YOPO 生成参考轨迹，SO3 是平移/几何外环，PX4 执行姿态、角速度和
更底层控制。首版使用可替换的 MAVROS `AttitudeTarget` backend。

## D-010：不新建桥接器 Git 仓库

**决策：** selector、output gateway 和 YOPO state bridge 作为独立 ROS package
放入 `TF_ROS_CUsalm`，共享坐标合同和测试。

## D-011：安全权限属于飞手

**决策：** 软件不得自动解锁。遥控器拥有最高权限，节点重启不得自动恢复飞行。

## D-012：先动捕飞行，再 cuVSLAM 飞行

**决策：** 先用 `mocap_primary` 验证 YOPO/SO3/PX4 基本控制链，再以
`cuvslam_primary` 完成最终避障验收，动捕转为 shadow。

## D-013：VRPN 精密时间标定延期

**决策：** 当前 Jetson 回调时间足够首版低速验证。精密光学采集延迟只在未来
高动态 ATE/RPE 或正式动捕性能研究中处理。

## D-014：首版飞行限制

**决策：** 最大倾角 15 deg、偏航角速度 30 deg/s、水平速度 0.5 m/s、垂直速度
0.3 m/s、加速度 1.0 m/s^2、高度 2.5 m、单次目标距离 3 m。

## D-015：飞行器物理参数

**决策：** X 布局；包含机圈的两条对角外轮廓均为 248 mm，交点是飞控和
`base_link` 中心；完整起飞重量 0.689 kg；当前 `MPC_THR_HOVER=0.60`。

## D-016：分支与证据策略

**决策：** 冻结 `feat/mocap-localization-shadow`，从其创建
`feat/yopo-integration-v1`。大型 rosbag 不进 Git，只提交摘要和 SHA-256。

## D-017：首版范围排除

**决策：** 排除自动定位切换、动态 map 对齐、地理定向、VRPN 精密延迟、移动
障碍、人员附近飞行、YOPO 重训练、高速穿越、窄缝、自动解锁和直接电机控制。

## D-018：localization、YOPO 与 flight stack 保持运行环境隔离

**决策：** localization-runtime 包含 RealSense、Isaac ROS、NITROS 和 cuVSLAM；
YOPO 保持宿主机独立运行时，源码位于
`/home/nvidia/catkin_ws/src/YOPO_ROS2`，实测执行入口为 `/usr/bin/python3`；
MAVROS/PX4 和控制 backend 位于宿主或独立控制环境。三者只通过 ROS 2 DDS 合同
通信，不合并 Python、PyTorch、CUDA wheel 或容器动态库依赖。
已创建的 Conda `yopo` 环境当前缺少 `torch`/`numpy`，且不在已安装
ROS 2 `yopo_node` 的执行路径中；不对其重复安装 Jetson PyTorch。

**理由：** 该拆分是为隔离已经识别的环境和依赖冲突，同时保证定位、规划与飞控
可以分别升级、测试和回滚。单仓库中的 ROS package 组织不改变部署隔离。

## D-019：首版 gateway 不发布 canonical Odometry

**决策：** `localization_output_gateway` 首版只把获批的
`SelectedPoseCandidate` 门禁到 MAVROS pose-only 输入，不创建
`/localization/odometry`。默认部署不得创建 MAVROS publisher；Gate G3 active 合同
在阈值获批前只允许作为测试 fixture，不安装生产 launch。

**理由：** selected seam 没有 twist 或 covariance。零值会伪造测量，NaN Odometry
仍会扩大未获批准且无人消费的合同。YOPO 最终状态 authority 是 PX4 EKF，首版没有
重复创建 canonical Odometry 的必要。

**验证要求：** 先只读采集不少于 60 秒的 MAVROS state/timesync，再固定 gateway
freshness、RTT、offset jitter 等阈值；禁止直接继承 selector 的健康阈值。

## D-020：动捕保留固定全局世界

**决策：** `mocap_primary` 令 `map == mocap_world`，固定使用 identity 世界变换，
原样保留动捕系统定义的绝对 position 和 yaw。只有 `cuvslam_primary` 使用首帧位置
归零和 yaw-only 局部对齐。本条在 mocap 模式下替代 D-003。

**理由：** 动捕是固定实验室世界中的全局定位源；用无人机上电位置重新定义原点会
把它错误降级为 VIO 式相对定位，并破坏跨启动一致的绝对坐标语义。
