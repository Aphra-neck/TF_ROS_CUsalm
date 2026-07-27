# YOPO 实机部署架构

## 1. 隔离原则

本项目采用多个运行环境，通过 ROS 2 DDS 交换标准消息。拆分原因是
Isaac ROS/NITROS/cuVSLAM 与 YOPO/PyTorch 的系统、CUDA、Python 和依赖组合存在
冲突风险。禁止为了减少启动终端而把这些环境重新合并。

    Jetson host
    ├── localization-runtime
    │   ├── RealSense ROS（D435I 唯一设备 owner）
    │   ├── Isaac ROS / NITROS
    │   ├── Visual SLAM / cuVSLAM SDK 12.6
    │   └── 定位、TF、深度和 diagnostics
    ├── YOPO runtime（宿主机 ROS 2 + system Python）
    │   ├── /home/nvidia/catkin_ws/src/YOPO_ROS2
    │   ├── /usr/bin/python3
    │   ├── Jetson CUDA 12.6 PyTorch（user site）
    │   └── 轨迹候选输出
    ├── future planner runtime（与 YOPO 互斥）
    │   ├── EGO-Planner
    │   └── Super Planner
    └── host flight stack / control runtime
        ├── MAVROS
        ├── SO3 backend / vehicle-control gateway
        └── PX4 serial link

YOPO、EGO-Planner 和 Super Planner 是互斥的规划器选择，不同时拥有控制输出权限。

## 2. 运行时责任

| 运行时 | 拥有 | 不得拥有 |
| --- | --- | --- |
| localization-runtime | D435I、Isaac ROS、NITROS、cuVSLAM、定位适配、标准深度和 selected pose 输出 | YOPO/PyTorch 环境、规划决策、SO3、解锁和模式切换 |
| YOPO runtime | 宿主机 ROS 2、system Python、Jetson PyTorch、YOPO 模型、目标输入、轨迹生成 | D435I 驱动、cuVSLAM、MAVROS、PX4 external vision、解锁 |
| host flight stack / control runtime | MAVROS、PX4 链路、经授权的 SO3 backend 和安全门禁 | RealSense 驱动、YOPO 神经网络、定位源自动切换 |
| future planner runtime | 对应规划器及其私有依赖 | D435I 驱动、MAVROS authority、与其他规划器并行控制 |

## 3. DDS 边界

所有参与运行时必须使用：

- ROS 2 Humble；
- ROS_DOMAIN_ID=42；
- 经实测互通的 RMW，当前固定为 rmw_fastrtps_cpp；
- host networking 或等价的、已验证的 DDS 网络配置；
- 完全一致的自定义消息定义和 type support；
- 明确的 QoS、frame、时间戳、单位和 publisher authority。

跨运行时只交换 ROS 2 消息。localization 容器不得挂载或复用 YOPO 的
host Python site-packages；YOPO runtime 也不得复用 localization 容器中的
CUDA wheel 或动态库。历史 Conda `yopo` 环境不在当前生产执行路径中。

## 4. 接口方向

    localization-runtime
      /localization/selected/pose
      /diagnostics
      /camera/depth/image
      /camera/depth/camera_info
              |
              | ROS 2 DDS
              v
    YOPO runtime
      /state/odom
      /depth_image
      -> shadow trajectory / PositionCommand
              |
              | ROS 2 DDS
              v
    control runtime
      -> localization_output_gateway
      -> /mavros/vision_pose/pose_cov
      -> PX4 EKF
      -> SO3
      -> replaceable MAVROS backend
      -> PX4

上述方向不表示当前已经授权 external vision、YOPO 或控制输出。首版不跨 DDS 发布
`/localization/odometry`；YOPO 状态由 PX4 EKF 融合输出经 `yopo_state_bridge` 生成。

## 5. 设备与发布权

- D435I 只能由 localization-runtime 中的一份 RealSense 节点打开；
- YOPO runtime 只能订阅深度，不得打开 D435I；
- 每个规范 topic 只能有一个 publisher；
- 规划器切换通过停止一个 runtime、验证 publisher 消失、再启动另一个 runtime 完成；
- 禁止运行时自动切换定位源或规划器；
- localization-runtime 不需要安装 MAVROS 可执行包；
- YOPO runtime 不需要安装 Isaac ROS、NITROS 或 cuVSLAM。

## 6. 版本证据

版本采集必须在各自真实运行环境执行，不能用一个容器的包版本代表另一个环境：

| Scope | 采集脚本 | 默认代码位置 |
| --- | --- | --- |
| localization-runtime | tools/collect_yopo_platform_baseline.sh | /workspaces/isaac_ros-dev |
| YOPO runtime | tools/collect_yopo_runtime_baseline.sh | 宿主机 `/home/nvidia/catkin_ws/src/YOPO_ROS2`，`/usr/bin/python3` |
| host flight stack | tools/collect_host_flight_stack_baseline.sh | Jetson host |
| MAVROS/PX4 external vision | tools/collect_mavros_px4_external_vision_audit.sh | Jetson host；MAVROS 已连接且 PX4 未解锁 |

三份输出分别生成 SHA-256，然后共同组成一次平台版本基线。
