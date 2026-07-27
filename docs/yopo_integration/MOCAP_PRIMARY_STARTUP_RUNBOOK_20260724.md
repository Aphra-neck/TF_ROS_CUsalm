# 动捕定位接入 PX4 启动手册（2026-07-24）

本手册用于当前临时方案：`VRPN 动捕 -> mocap adapter -> selector -> output gateway -> MAVROS -> PX4`。MAVROS 在 Jetson 宿主机运行，其余节点在 Isaac ROS 容器内运行。

当前不启动 cuVSLAM、SO3 控制器或 YOPO 控制输出。gateway 只发布外部视觉 pose，不解锁、不切换模式、不发送速度或控制指令。

## 0. 前提

- PX4 已上电，串口为 `/dev/ttyTHS2`，波特率为 `921600`。
- 动捕服务器已运行，刚体名称为 `droneyee207`；VRPN 配置对应 `192.168.151.168:3883`。
- Jetson 工作区已构建，容器内存在 `/workspaces/isaac_ros-dev/install/setup.bash`。
- 启动期间保持飞机未解锁；先完成本文末尾的链路诊断。

### 0.1 首次固化容器依赖（宿主机）

运行中的容器里手动安装的软件包会随容器删除。首次部署或
`docker/Dockerfile.yopo_mocap` 变更后，在飞机未解锁且旧容器任务均已停止时执行：

```bash
cd "$HOME/workspaces/isaac_ros_3_2/src/TF_ROS_CUsalm"
bash tools/configure_isaac_ros_image.sh

docker stop isaac_ros_dev-aarch64-container 2>/dev/null || true

cd "$HOME/workspaces/isaac_ros_3_2/src/isaac_ros_common"
unset SKIP_DOCKER_BUILD
./scripts/run_dev.sh -d "$HOME/workspaces/isaac_ros_3_2"
```

这一次不能加 `-b`，因为必须生成含 `ros-humble-mavros-msgs` 的新镜像。进入新容器后验证：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix mavros_msgs
test -f /opt/ros/humble/lib/libmavros_msgs__rosidl_typesupport_cpp.so \
  && echo "mavros_msgs_cpp_typesupport=PASS"
```

预期输出 `/opt/ros/humble`。镜像构建成功后退出容器，以后按第 1 节使用 `-b` 即可，
无需再次手动 `apt install`。

## 1. 进入 Isaac ROS 容器（宿主机终端）

每次需要新的容器终端时，在新的宿主机终端执行以下命令。已有容器运行时，脚本会连接到该容器。

```bash
conda deactivate 2>/dev/null || true
unset PYTHONHOME

export ISAAC_ROS_WS="$HOME/workspaces/isaac_ros_3_2"
export ROS_DOMAIN_ID=42

cd "$ISAAC_ROS_WS/src/isaac_ros_common"
./scripts/run_dev.sh -b -d "$ISAAC_ROS_WS"
```

成功后提示符应类似：

```text
admin@tegra-ubuntu:/workspaces/isaac_ros-dev$
```

## 2. 启动 MAVROS（宿主机终端 1）

这仍是当前正确的 MAVROS 启动命令。不要在容器内执行。

```bash
conda deactivate 2>/dev/null || true
unset PYTHONHOME
set +u
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

FCU_DEV=/dev/ttyTHS2
FCU_BAUD=921600

if [ ! -c "$FCU_DEV" ]; then
  echo "[STOP] $FCU_DEV does not exist"
  exit 1
fi

if ! id -nG | tr ' ' '\n' | grep -qx dialout; then
  echo "[STOP] current user is not in dialout"
  exit 1
fi

if fuser "$FCU_DEV" >/dev/null 2>&1; then
  echo "[STOP] $FCU_DEV is already in use"
  fuser -v "$FCU_DEV"
  exit 1
fi

ros2 launch mavros px4.launch \
  fcu_url:="${FCU_DEV}:${FCU_BAUD}"
```

保持该终端运行。

## 3. 可选：请求 HIGHRES_IMU 200 Hz（宿主机终端 2）

纯动捕定位不需要此步骤。只有同时运行 cuVSLAM、FCU IMU relay，或需要检查 `/mavros/imu/data_raw` 时才执行。

```bash
conda deactivate 2>/dev/null || true
unset PYTHONHOME
set +u
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 service call \
  /mavros/set_message_interval \
  mavros_msgs/srv/MessageInterval \
  "{message_id: 105, message_rate: 200.0}"
```

该调用只是请求 PX4 发送 MAVLink `HIGHRES_IMU`（ID 105），不代表实际频率一定为 200 Hz。需要时另行检查：

```bash
timeout --signal=INT 8s ros2 topic hz /mavros/imu/data_raw
```

## 4. 启动 VRPN（容器终端 1）

先按第 1 节进入容器，再执行：

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

ros2 launch vrpn_client_ros sample.launch.py
```

保持该终端运行。在新容器终端确认原始动捕有数据：

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

timeout --signal=INT 5s ros2 topic hz /droneyee207/pose
```

期望约 `120 Hz`。没有频率时先修复 VRPN 或动捕刚体，不启动下游链路。

## 5. 启动动捕 adapter（容器终端 2）

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

ros2 launch mocap_localization_adapter \
  mocap_adapter_shadow.launch.py
```

保持该终端运行。在新容器终端确认 candidate 有数据：

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

timeout --signal=INT 5s ros2 topic hz \
  /localization/candidates/mocap/base_pose
```

期望约 `120 Hz`。必须先看到 candidate 频率，再启动下一节组合 launch。

## 6. 启动 selector + gateway（容器终端 3）

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

for NODE in /localization_source_selector /localization_output_gateway; do
  if ros2 node list | grep -qx "$NODE"; then
    echo "[STOP] $NODE is already running"
    exit 1
  fi
done

ros2 launch localization_output_gateway \
  mocap_primary_output.launch.py
```

这个组合 launch 已同时启动：

```text
/localization_source_selector
/localization_output_gateway
```

不要再单独启动 selector 或 gateway，否则 publisher authority 检查会锁存故障。

## 7. 链路诊断（容器终端 4）

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

timeout --signal=INT 5s ros2 topic hz /droneyee207/pose
timeout --signal=INT 5s ros2 topic hz \
  /localization/candidates/mocap/base_pose
timeout --signal=INT 5s ros2 topic hz \
  /localization/selected/pose
timeout --signal=INT 5s ros2 topic hz \
  /mavros/vision_pose/pose_cov
```

四个话题都应约为 `120 Hz`。随后检查 PX4 输入端点：

```bash
ros2 topic info -v /mavros/vision_pose/pose_cov

ros2 topic echo /mavros/vision_pose/pose_cov \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  --once --no-arr
```

必须满足：

```text
Publisher count: 1
publisher node: /localization_output_gateway
Subscription count: 1
subscriber node: /mavros/vision_pose
```

检查 adapter、selector 和 gateway 诊断：

```bash
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray \
  --once \
  --filter "any('mocap_localization_adapter' in s.name for s in m.status)"

ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray \
  --once \
  --filter "any('localization_source_selector' in s.name for s in m.status)"

ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray \
  --once \
  --filter "any('localization_output_gateway' in s.name for s in m.status)"
```

adapter 应为 `health_state=healthy` 且 `latched=0`；selector 应为 `state=healthy`、`reason_code=SOURCE_HEALTHY` 且 `input_publisher_count=1`。gateway 成功标志：

```text
state=active_healthy
reason_code=EXTERNAL_VISION_PUBLISHED
input_publisher_count=1
mavros_state_publisher_count=1
mavros_timesync_publisher_count=1
output_publisher_count=1
mavros_output_subscription_count=1
authority_violations=0
contract_violations=0
timestamp_violations=0
pose_violations=0
```

最后检查 MAVROS 连接和 PX4 本地位置输出：

```bash
ros2 topic echo /mavros/state mavros_msgs/msg/State --once
ros2 topic echo /mavros/timesync_status mavros_msgs/msg/TimesyncStatus --once

timeout --signal=INT 8s ros2 topic hz /mavros/local_position/pose
ros2 topic echo /mavros/local_position/pose \
  geometry_msgs/msg/PoseStamped --once
```

`/mavros/state` 必须显示 `connected: true`。上述链路成功只证明外部视觉 pose 已持续送到 MAVROS；PX4 EKF 是否按预期融合仍需结合 PX4 estimator 状态确认。

## 8. 停止顺序

必须先确认 PX4 已解除武装，再停止定位链路。确认 `armed: false` 后，各运行终端按以下顺序按 `Ctrl-C`：

```bash
ros2 topic echo /mavros/state mavros_msgs/msg/State --once
```

1. selector + gateway；
2. mocap adapter；
3. VRPN；
4. MAVROS。
