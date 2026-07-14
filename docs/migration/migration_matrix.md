# 迁移矩阵

更新时间：2026-07-15。

状态按“代码迁移、离线验证、数据回放、实机验收”分级记录。当前离线回归为
`118 passed`；离线通过不代表定位精度、导航性能或实车安全验收通过。

| 模块 | Phase | 当前状态 | 已验证范围 | 下一步 |
| --- | --- | --- | --- | --- |
| `agt_interfaces` | 1 | 骨架完成 | package、目录和命名契约检查通过 | 按跨模块状态与任务需求补充 msg/srv/action，并增加序列化和兼容性测试 |
| `agt_description` | 2/8 | 离线完成 | 固定 TF、MK-mini/BUNKER profile、Xacro 展开、TF 单父节点和可配置 MID360 外参通过；BUNKER profile 已确立为 footprint 单一数据源 | 标定 `base_link -> lidar_link`，实测 BUNKER 基准高度和履带中心距，确认 `calibration_verified` |
| `agt_bringup` | 1/6 | baseline 完成 | 建图模式、地图保存、导航模式、Qt5、底盘、安全链和一键录包总控入口已建立 | 用真实传感器和地图验证异常退出、生命周期、重复启动防护与节点重启 |
| `agt_sensor_adapters` | 3 | baseline 完成 | Livox 驱动已迁入，MID360 PointCloud2 到 CustomMsg 转换、统一 topic 和短 bag 回放通过 | 实机验证网络、QoS、频率、时间戳、丢包和长时间运行稳定性 |
| `agt_mapping` | 3 | baseline 完成 | 指定 FAST-LIVO2 分支已 vendor 和编译；adapter、位姿/twist 外参换算、TF 与局部雷达帧点云回放通过 | 标定车辆外参，使用完整 bag 生成新旧轨迹、点云数量和数值精度对比报告 |
| `agt_map_processing` | 5 | baseline 可用 | OctoMap 动态射线原点、二维 OccupancyGrid 以及 PGM/YAML 保存已通过短回放 | 完整 bag 调整高度阈值并对比旧 `/projected_map`；后续增加 PCD 离线转换和几何地面分割后端 |
| `agt_localization` | 4 | 代码已落地 | ICP/NDT core、局部点云输入、base/lidar 初值修正和唯一 `map -> odom` 发布逻辑已编译 | 使用同一次建图导出的全局 PCD 验证收敛率、误差、恢复时间、TF 稳定性和错误初值拒绝 |
| `agt_localization_fusion` | 6 | 仅骨架 | package 和领域边界已建立 | 定义融合状态与诊断接口，接入 LIO、轮速和 IMU；后续扩展 RTK/UWB 与失效降级 |
| `agt_perception` | 6/8 | baseline 完成 | 已实现 base frame 高度/量程/车体裁剪的局部障碍点云，并接入 Nav2 costmap 和 Collision Monitor；裁剪边界已由契约测试约束到 BUNKER profile | 使用典型场景点云评估地面/障碍精度、误检漏检和频率，再增加可靠地面分割 |
| `agt_navigation` | 6/8 | TASK-07 离线完成 | 原 1 m 闭环继续通过；FilterInfo、global `Static -> Keepout -> Inflation`、跨禁行墙规划失败及 toggle 后恢复规划已验证 | 使用真实语义地图与重定位验证边界误差、切换时延、规划成功率和窄通道通过性 |
| `agt_coverage_planning` | 8 | TASK-00~09 完成，原始路径可生成 | polygon 外环/孔洞、annotated rows GML、profile 参数同步、双 server lifecycle、标准输出和失败诊断完成；真实规划返回 174/161 个有效 `map` 姿态且 polygon 路径不穿孔洞 | TASK-10 实现 costmap footprint、插值碰撞和曲率 Validator |
| `agt_safety` | 6 | baseline 完成 | BUNKER 履带仲裁、手动优先、限速、输入超时、急停锁存和复位保持禁用的合成消息回归通过 | 架空履带验证方向和急停，再完成低速制动距离、进程退出和通信中断验收 |
| `agt_chassis` | 6 | baseline 完成 | 官方 `bunker_ros2`、状态桥接、TF 隔离和双层命令 watchdog 已接入并离线构建 | CAN 实机验证协议版本、轮速里程计、错误码、方向、断连归零和长时间通讯稳定性 |
| `agt_ui_bridge` | 8 | TASK-07 接口接入完成 | enabled exclusion/keepout 与 field 外部生成对齐 mask；Nav2 接收、阻断规划、切换与禁用不污染底图 | 使用真实地图验证语义切换、服务器异常和 fail-open 操作门禁 |
| `agt_experiment_manager` | 7 | 仅骨架 | package、profile 和 runtime 目录边界已建立 | 实现配置合并、Git/参数快照、产物命名、失败恢复和一键复现实验 |
| `agt_evaluation` | 7 | 仅骨架 | package 和指标职责边界已建立 | 实现轨迹、重定位、导航、地图质量和资源占用指标，并生成可复现报告 |

## 阶段汇总

| 阶段 | 当前结论 | 进入下一验收级别的条件 |
| --- | --- | --- |
| Phase 0：旧系统基线 | 部分完成 | 固定旧仓库 tag/commit、参数快照和可复现报告 |
| Phase 1：仓库与接口 | 已完成 | 后续按实际需求扩充自定义接口，避免提前过度设计 |
| Phase 2：机器人描述 | 已离线完成 | 完成车辆外参和 BUNKER 几何尺寸实测 |
| Phase 3：传感器与建图 | baseline 完成 | 完成完整 bag 新旧输出对比和车辆外参验证 |
| Phase 4：重定位 | 代码已落地 | 使用同源全局 PCD 完成回放精度与失败恢复验收 |
| Phase 5：地图处理 | baseline 可用 | 完成完整 bag 地图质量对比并固定导航地图参数 |
| Phase 6：Nav2、底盘与安全 | 离线 baseline 完成 | 完成真实地图导航、CAN、硬件急停和低速制动验收 |
| Phase 7：实验与评测 | 尚未实现 | 完成配置快照、自动记录、指标计算和报告生成 |
| Phase 8：Qt5 与覆盖规划 | TASK-00~09 完成 | 进入 TASK-10，实现 Coverage Path Validator |
| Phase 9：扩展研究 | 未开始 | 基础导航闭环稳定后再接入 RTK/UWB、语义点云和其他雷达 |
