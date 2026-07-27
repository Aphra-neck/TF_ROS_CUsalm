# YOPO 阶段任务流

本文件是 YOPO 实机集成阶段唯一任务状态来源。

## 1. 状态定义

| 状态 | 含义 |
| --- | --- |
| `BLOCKED` | 依赖、授权或证据不足，不允许开始 |
| `READY` | 依赖满足，可以开始 |
| `IN_PROGRESS` | 正在执行，尚未通过验收 |
| `PASSED` | 对应验收条件和证据全部满足 |
| `FAILED` | 已执行但未满足条件，必须记录原因和恢复任务 |

状态只能在证据产生后更新。代码合并、节点启动或测试“看起来正常”不能自动产生
`PASSED`。

## 2. 当前里程碑

基线日期：2026-07-23。

| ID | 任务 | 状态 | 依赖 | 完成证据 |
| --- | --- | --- | --- | --- |
| YP-000 | 复核并冻结本目录需求合同 | `PASSED` | 用户确认 | `5f03815` |
| YP-010 | 从定位证据分支创建 `feat/yopo-integration-v1` | `PASSED` | YP-000 | 起点 `5f03815`，远程分支已建立 |
| YP-020 | 固定跨运行环境和 PX4/MAVROS 版本清单 | `PASSED` | YP-010 | `VERSION_MANIFEST.md`、localization/YOPO/host 三份采集输出 |
| YP-100 | 在 bringup 中开启原生深度，保持 emitter 关闭 | `PASSED` | YP-020 | `2584606`；73/73 tests；Depth `640x360@90`/`16UC1`；emitter=0 |
| YP-110 | 验证红外 90 Hz、深度质量和 cuVSLAM 无回归 | `PASSED` | YP-100 | `GATE_G2_REPORT_20260723.md` |
| YP-120 | 深度不足时执行 emitter A/B | `PASSED` | YP-110 判定不足 | `N/A_DECISION`；emitter-off centre p05 `0.854368` |
| YP-200 | 审计 MAVROS 2.14/PX4 外部视觉接口和缺失速度表达 | `PASSED` | YP-020 | `f8fde79`；v2 SHA `6778c559...f2c66d`；errors/warnings `0/0`；NaN 透传归 Gate G3 |
| YP-210 | 固化 cuVSLAM 50 mm 外参合同与测试 | `PASSED` | YP-010 | `f300bec`；2 packages；417 tests，0 failures |
| YP-220 | 实现 `localization_source_selector` | `IN_PROGRESS` | YP-200、YP-210 | v2 全局 mocap 语义待 Jetson 测试和新包验证；旧 v1 证据仅保留为历史 |
| YP-230 | 实现 `localization_output_gateway` | `IN_PROGRESS` | YP-200、YP-220 | Gateway 合同与 Gate G3 报告待完成 |
| YP-240 | 实现 `yopo_state_bridge` | `BLOCKED` | YP-230 | frame/twist/authority 测试 |
| YP-250 | 实现两个互斥 primary launch | `BLOCKED` | YP-220、YP-230、YP-240 | launch authority 测试 |
| YP-290 | 确认 YOPO 实际 Python 推理环境并完成模型加载冒烟 | `PASSED` | YP-010 | system Python/CUDA 证据；`yopo_model_smoke_20260723.txt` |
| YP-300 | 固定 YOPO ROS 2 参数、模型、深度和状态话题 | `BLOCKED` | YP-110、YP-240、YP-290 | config hash、模型 hash |
| YP-310 | 完成 YOPO 10 分钟被动规划 | `BLOCKED` | YP-300 | Gate G4 报告和 rosbag |
| YP-400 | 移植最小 ROS 2 C++ SO3 core | `BLOCKED` | YP-310 | 单元测试、接口测试 |
| YP-410 | 实现可替换 MAVROS `AttitudeTarget` backend | `BLOCKED` | YP-400、YP-200 | type-mask/符号/限幅测试 |
| YP-420 | 配置并验证 PX4 OFFBOARD failsafe | `BLOCKED` | YP-410 | 参数导出、故障注入日志 |
| YP-430 | 完成 5 分钟拆桨控制联调 | `BLOCKED` | YP-410、YP-420 | Gate G5 报告 |
| YP-500 | 完成 `mocap_primary` 低速起飞、悬停、直线和降落 | `BLOCKED` | YP-430 | Gate G6 飞行证据 |
| YP-600 | 完成 `cuvslam_primary` 空场和多目标飞行 | `BLOCKED` | YP-500 | Gate G7 飞行证据 |
| YP-610 | 完成单一大型静态障碍避障 | `BLOCKED` | YP-600 | Gate G8-1 证据 |
| YP-620 | 完成多障碍或宽走廊避障 | `BLOCKED` | YP-610 | Gate G8-2 证据 |
| YP-700 | 封存版本、参数、rosbag、报告和 SHA-256 | `BLOCKED` | YP-620 | Gate G9 manifest |
| YP-710 | 评审并合并稳定版本 | `BLOCKED` | YP-700 | review 与 merge commit |

## 3. 当前已知输入证据

2026-07-23 的 dual-shadow 录包与 `analysis_v2` 已完成 hash 封存，但其
`runtime_contract=FAIL`。它证明了工具链能够读完固定证据并识别启动边界 orphan、
历史 clock counter 和 mocap 缺口；它不是 Gate G3 或飞行授权证据。

2026-07-27 在 D-020 之前录制的 mocap 包使用首帧归零语义，不能验收固定全局
`mocap_world`。必须用 v2 selector/gateway 重建、通过测试并重录证据后，才能恢复
`YP-220=PASSED` 并把该定位链作为后续 YOPO 状态桥输入。

YOPO 已在 Jetson 宿主机完成构建，源码位于
`/home/nvidia/catkin_ws/src/YOPO_ROS2`。已安装节点使用 `/usr/bin/python3`，
Jetson CUDA 12.6 PyTorch 模型加载和被动 warm-up 已通过。当前未解决项是 `YP-400/410` 的 ROS 2
SO3 core 与 MAVROS 控制 backend，不得把它误记为 YOPO 未部署。

## 4. 更新模板

每次改变任务状态时，在提交说明或关联证据中记录：

```text
task_id:
old_status:
new_status:
date:
owner:
revision:
evidence:
result:
remaining_risk:
```

`FAILED` 任务恢复前必须新增原因分析和修复任务，禁止直接改回 `READY` 掩盖失败。
