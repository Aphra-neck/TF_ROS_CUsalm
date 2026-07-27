# YOPO 实机集成合同

本目录是 YOPO、定位适配、SO3、MAVROS 与 PX4 实机集成阶段的主合同。
合同基线标识为 `yopo_integration_20260723_v1`。

当前状态是**需求和跨运行时版本基线已冻结，D435 Depth 与 cuVSLAM
Gate G2 已通过，定位输出和控制链仍未授权**。本目录不构成飞行授权。

## 文档导航

| 文档 | 责任 |
| --- | --- |
| [PROJECT_REQUIREMENTS.md](PROJECT_REQUIREMENTS.md) | 最终目标、系统范围、功能和非功能需求 |
| [INTERFACE_CONTRACTS.md](INTERFACE_CONTRACTS.md) | frame、变换、topic、语义和发布权 |
| [SAFETY_CONSTRAINTS.md](SAFETY_CONSTRAINTS.md) | 安全边界、飞行限制、故障处理和禁止事项 |
| [ACCEPTANCE_CRITERIA.md](ACCEPTANCE_CRITERIA.md) | 各阶段通过条件和必须保留的证据 |
| [TASKFLOW.md](TASKFLOW.md) | YOPO 阶段唯一任务状态来源 |
| [DECISION_LOG.md](DECISION_LOG.md) | 需求访谈中确认的架构决策与理由 |
| [VERSION_MANIFEST.md](VERSION_MANIFEST.md) | 仓库、模型、Jetson、相机、MAVROS 与 PX4 版本基线 |
| [DEPLOYMENT_ARCHITECTURE.md](DEPLOYMENT_ARCHITECTURE.md) | 多运行时隔离、DDS 边界和设备 ownership |
| [GATE_G2_REPORT_20260723.md](GATE_G2_REPORT_20260723.md) | D435 原生深度与 cuVSLAM 联合运行证据 |
| [YP_210_EXTRINSIC_CONTRACT_20260723.md](YP_210_EXTRINSIC_CONTRACT_20260723.md) | 50 mm 相机外参、变换方向和权限边界 |
| [YP_200_MAVROS_PX4_EXTERNAL_VISION_AUDIT.md](YP_200_MAVROS_PX4_EXTERNAL_VISION_AUDIT.md) | MAVROS/PX4 pose-only 接口源码与只读运行审计；注入证据归 Gate G3 |
| [YP_220_SOURCE_SELECTOR_CONTRACT.md](YP_220_SOURCE_SELECTOR_CONTRACT.md) | 启动时固定主源、一次 map 对齐和 pose-only selected seam |
| [YP_230_OUTPUT_GATEWAY_CONTRACT.md](YP_230_OUTPUT_GATEWAY_CONTRACT.md) | 默认禁用的 pose-only MAVROS gateway、publisher authority 和 Gate G3 边界 |

## 权威顺序

发生冲突时按以下顺序处理：

1. `SAFETY_CONSTRAINTS.md` 的禁止项和门禁；
2. `INTERFACE_CONTRACTS.md` 的运行语义；
3. `PROJECT_REQUIREMENTS.md` 的范围和目标；
4. `ACCEPTANCE_CRITERIA.md` 的验收方法；
5. `TASKFLOW.md` 的当前执行状态。

`DECISION_LOG.md` 记录为什么这样设计，但不能覆盖现行安全和接口合同。

## 变更规则

- `TASKFLOW.md` 是唯一任务状态来源，其他文档不得维护平行待办列表。
- 坐标、外参、主定位模式、飞行限制或控制权限的变更必须新增决策记录。
- 任何放宽安全门禁的变更必须单独评审，不能夹带在普通实现提交中。
- rosbag、模型权重和其他大型二进制证据不提交 Git；Git 只保存版本、摘要和 SHA-256。
- 目标接口和已验收事实必须分开陈述，禁止用设计文档代替运行证据。

## 当前执行动作

`YP-220` 的启动时定位源选择器已经通过，当前执行 `YP-230` 的 pose-only output
gateway 合同与实现。在 Gate G3 通过前，不向 PX4 发布定位或控制输出。
