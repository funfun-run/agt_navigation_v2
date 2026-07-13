# 迁移矩阵

| 模块 | Phase | 当前状态 | 下一步 |
| --- | --- | --- | --- |
| agt_interfaces | 1 | 已建骨架 | 补充 msg/srv/action 定义 |
| agt_description | 2 | 已落地固定 TF、MK-mini/BUNKER 尺寸 profile 与可配置 MID360 外参 | 实测车辆外参、BUNKER 基准高度和履带中心距并确认 calibration_verified |
| agt_bringup | 1 | 已建骨架 | 增加最小系统 launch |
| agt_sensor_adapters | 3 | 已迁入 Livox 驱动并完成 MID360 统一 launch/topic | 实机验证网络、QoS、频率和时间戳 |
| agt_mapping | 3 | 指定 FAST-LIVO2 已 vendor 并编译；adapter/TF/局部雷达帧点云回放通过 | 标定车辆外参并做完整 bag 新旧输出对比 |
| agt_map_processing | 5 | OctoMap 在线投影、动态射线原点和 OccupancyGrid 保存已通过短回放 | 完整 bag 调参与对比旧 `/projected_map`，再增加 PCD/地面分割后端 |
| agt_localization | 4 | ICP/NDT core 已迁入；修正 base/lidar 初值并实现唯一 `map -> odom` | 准备同源全局 PCD，用 bag 验证收敛率和 TF |
| agt_localization_fusion | 6 | 已建骨架 | 定义融合接口与状态 |
| agt_perception | 6 | 已建骨架 | 先接几何障碍处理链 |
| agt_navigation | 6 | 已建骨架 | 后续接 Nav2 适配 |
| agt_coverage_planning | 8 | 已建骨架 | 预留 Fields2Cover adapter |
| agt_safety | 6 | 已落地 BUNKER 履带安全仲裁、急停锁存、限速和超时，合成消息回归通过 | 架空履带后做低速实车制动距离与硬件急停验收 |
| agt_chassis | 6 | 已接入 bunker_ros2、状态桥接、TF 隔离和双层命令 watchdog，离线构建通过 | 需要 CAN 实机验证协议、轮速、错误状态和断连保护 |
| agt_ui_bridge | 8 | Ros_Qt5_Gui_App `b0825e3` 已 vendor 和编译，V2 topic、地图服务及退出测试通过 | 导航迁移后接 NavigateToPose action，并做实机手动控制验收 |
| agt_experiment_manager | 7 | 已建骨架 | 实现配置合并与快照 |
| agt_evaluation | 7 | 已建骨架 | 补充离线评测脚本 |
