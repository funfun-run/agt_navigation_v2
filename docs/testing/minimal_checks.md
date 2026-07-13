# 最小检查项

## 仓库级
- `colcon list` 能识别所有 package
- 占位测试能被发现
- package、frame、topic 和参数符合命名规范
- 所有 Python launch 文件可以完成语法解析

## Phase 2 预备检查
- TF 树无重复发布责任
- `colcon test --packages-select agt_description` 通过
- `base_footprint -> base_link -> lidar_link -> livox_frame` 连通
- 实车外参未测量前保持 `calibration_verified: false`
- `agt_description` 可以单独启动并输出机器人描述

## Phase 3 离线检查
- FAST-LIVO2 原生 TF 补丁可以应用到基线源码
- adapter 位姿和平移/旋转外参换算测试通过
- adapter twist 旋转和传感器杠杆臂换算测试通过
- 注册点云 remap 为 `/agt/mapping/registered_points`
- 当前帧雷达点云 `/agt/mapping/registered_points_lidar` 使用 `lidar_link`，并能通过动态
  `odom -> lidar_link` 生成二维栅格
- 标准里程计输出为 `/agt/mapping/odometry`，child frame 为 `base_footprint`

## Phase 3 待数据检查
- `mid360_mapping_20260603_195044` 已完成字段审计和完整 CustomMsg 转换
- 待使用派生 bag 对比旧链与 V2 的点云、里程计、时间戳和轨迹
- 检查 `odom -> base_footprint` 只有 adapter 一个发布源
- 检查长时间运行漂移、丢帧、延迟和资源占用
- 使用实测外参完成直线、转向和倾斜安装验证

## Phase 4 重定位检查
- 全局 PCD 与二维导航地图来自同一次建图且共用 `map` 原点
- `/initialpose` 按 `map -> base_link` 解释，并通过 TF 合成 `map -> lidar_link` 初值
- 只有 fitness 阈值通过后才发布 `map -> odom`
- `ros2 topic echo /agt/localization/status` 能区分地图缺失、点数不足、未收敛和质量拒绝
- `map -> odom` 只有 `agt_localization` 一个发布源

## Qt5 地图工具检查
- `third_party/ros_qt5_gui_app` 与记录的上游提交和 GPL-2.0 许可证一致
- `runtime/gui/ros_qt5_gui_app/config.json` 使用 V2 map、odom、manual cmd 和 base frame
- 主入口 `ros_qt5_gui.launch.py` 能启动 GUI；新构建不存在时仅允许显式旧工作区 fallback
- PGM/YAML 加载保存往返后尺寸、分辨率、原点和占据值不变
- 编辑结果发布到 `/agt/map/edited`，且不会被原始投影 topic 覆盖
- `/initialpose` 和 `/goal_pose` 的 frame 均为 `map`
- 无 GUI 环境可独立运行 `map_io_bridge.py`

## BUNKER 底盘与安全检查
- 官方 `/cmd_vel` 只能 remap 自 `/agt/chassis/cmd_vel`，不能直接连接 Nav2 或 GUI
- `publish_driver_odom_tf=false`，`odom -> base_footprint` 仍只有定位/融合链一个发布源
- 未调用运动使能服务时，导航和手动命令均输出零速
- 手动命令优先于导航命令，超时后 `/agt/safety/cmd_vel` 与 `/agt/chassis/cmd_vel` 均归零
- 左右履带投影速度不超过 `max_track_speed`，反向速度使用独立低速上限
- 急停为锁存状态；物理输入未释放时不能复位，复位后仍需重新使能
- 状态超时后 `/agt/chassis/connected=false` 且 `/agt/chassis/status` 为 ERROR
- 实机依次完成架空履带、`0.15 m/s` 空旷低速、通信中断和硬件急停制动距离测试

## Phase 6 Nav2 离线检查
- `offline_navigation.launch.py` 启动后八个 Nav2 lifecycle 节点全部为 `active`
- SmacPlanner2D、SimpleSmoother 和 MPPI DiffDrive 插件均能在 ROS 2 Humble 加载
- 测试地图发布到 `/agt/map/global_occupancy`，局部代价地图接收 `/agt/perception/obstacle_cloud`
- `NavigateToPose` 的 1 m 目标完成并返回 `SUCCEEDED`，里程计产生对应位移
- Qt5 `/goal_pose` 能触发 NavigateToPose，并由 `/agt/navigation/status` 报告桥接状态
- raw 命令必须经过 Collision Monitor 和 `agt_safety`，禁止控制器直连底盘
- 离线入口可自动使能运动；真实系统入口必须保持安全层默认禁用

## Phase 6 待实车检查
- 使用实测外参验证地面高度过滤、低矮障碍、反光点、斜坡和负障碍场景
- 调整 footprint、inflation、MPPI critic、速度/加速度和恢复行为
- 测试碰撞 stop/slowdown 区、传感器超时、定位跳变、LIO/重定位退出和 CAN 断连
- 统计窄通道通过率、路径跟踪误差、制动距离和 CPU/GPU 资源占用

## 总控模式检查
- `system.launch.py mode:=mapping` 与 `mode:=navigation` 均只启动一个 robot_state_publisher
- 建图模式强制 `save_pcd=true`，退出后同时生成原始和降采样 PCD
- 导航模式强制 `save_pcd=false`，不得覆盖已有地图
- 每个子 launch 显式绑定自己的 `params_file`，禁止 FAST-LIVO2/OctoMap/Nav2 参数串包
- `record_bag:=true` 记录传感器、TF、地图、里程计、控制链和诊断 topic
- 结束总控必须使用 `Ctrl+C`，并检查 bag 的 `metadata.yaml` 与地图文件完整性
