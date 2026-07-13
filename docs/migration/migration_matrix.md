# 迁移矩阵

| 模块 | Phase | 当前状态 | 下一步 |
| --- | --- | --- | --- |
| agt_interfaces | 1 | 已建骨架 | 补充 msg/srv/action 定义 |
| agt_description | 2 | 已落地固定 TF 与可配置 MID360 外参；实测值待标定 | 实车测量并确认 calibration_verified |
| agt_bringup | 1 | 已建骨架 | 增加最小系统 launch |
| agt_sensor_adapters | 3 | 已迁入 Livox 驱动并完成 MID360 统一 launch/topic | 实机验证网络、QoS、频率和时间戳 |
| agt_mapping | 3 | 离线迁移完成：adapter、统一 topic、位姿/twist 换算与 TF 边界 | 应用上游补丁并做同 bag/实机对比 |
| agt_map_processing | 1 | 已建骨架 | 先复现 OctoMap baseline |
| agt_localization | 1 | 已建骨架 | 迁移 ICP 重定位 |
| agt_localization_fusion | 1 | 已建骨架 | 定义融合接口与状态 |
| agt_perception | 1 | 已建骨架 | 先接几何障碍处理链 |
| agt_navigation | 1 | 已建骨架 | 后续接 Nav2 适配 |
| agt_coverage_planning | 1 | 已建骨架 | 预留 Fields2Cover adapter |
| agt_safety | 1 | 已建骨架 | 迁移限速与急停仲裁 |
| agt_chassis | 1 | 已建骨架 | 统一底盘控制接口 |
| agt_ui_bridge | 1 | 已建骨架 | 定义 Qt5 标准桥接接口 |
| agt_experiment_manager | 1 | 已建骨架 | 实现配置合并与快照 |
| agt_evaluation | 1 | 已建骨架 | 补充离线评测脚本 |
