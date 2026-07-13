# agt_navigation_v2

`agt_navigation_v2` 是面向农业机器人导航实验的 ROS 2 模块化平台。

当前已完成仓库与接口骨架、机器人描述、MID360 驱动、FAST-LIVO2 建图适配、OctoMap
二维投影、ICP/NDT 重定位代码迁移、Qt5 地图界面、完整 Nav2 离线闭环，以及 BUNKER
底盘通讯和履带安全层。
现有 MID360 bag 已完成字段审计、CustomMsg 转换和短回放链路验证；全局 PCD 重定位联调、
完整 bag 地图调优、真实地图重定位联调和实机验收仍待执行。

## 项目进度概览

| 阶段 | 当前状态 | 已验证范围 | 主要剩余工作 |
| --- | --- | --- | --- |
| Phase 0：旧系统基线 | 部分完成 | 现有 bag 保留旧链注册点云、里程计、TF 和投影地图 | 固定旧仓库 tag/commit、参数快照和可复现报告 |
| Phase 1：仓库与接口 | 已完成 | 16 个 `agt_*` package 可被 colcon 识别，命名和目录契约已建立 | 按后续模块需要补充自定义 msg/srv/action |
| Phase 2：机器人描述 | 已离线完成 | TF 单父节点、MK-mini/BUNKER 尺寸配置和 Xacro 展开通过 | 标定 `base_link -> lidar_link`，实测 BUNKER 基准高度和履带中心距 |
| Phase 3：传感器与建图 | 已完成 baseline | MID360 PointCloud2 转 CustomMsg、FAST-LIVO2 编译回放、统一里程计/点云接口通过 | 完整 bag 新旧轨迹/点云数值对比和车辆外参验证 |
| Phase 4：重定位 | 代码已落地 | ICP/NDT core、初值外参修正和唯一 `map -> odom` 发布逻辑已编译 | 从同源建图结果导出全局 PCD，验证收敛率与错误初值拒绝 |
| Phase 5：地图处理 | baseline 可用 | OctoMap 动态射线原点、二维 OccupancyGrid 和 PGM/YAML 保存通过短回放 | 完整 bag 调整高度阈值，确认最终导航地图 |
| Phase 6：Nav2 与安全链 | 离线 baseline 完成 | Smac2D、MPPI、BT、costmap、Collision Monitor、Qt action、BUNKER 安全链完成闭环目标测试 | 用真实地图/定位调参；完成障碍、CAN 与制动验收 |
| Phase 7：实验与评测 | 尚未实现 | package 和 runtime 目录边界已建立 | 实现配置快照、数据记录、指标计算和报告生成 |
| Phase 8：Qt5 与覆盖规划 | 部分完成 | Qt5 GUI 编译、地图接口、Nav2 action 桥接、ROS 图和干净退出验证通过 | 实机手动控制与 Fields2Cover |

当前离线回归结果：`46 passed`；BUNKER 无 CAN 运行测试已验证默认禁用、手动优先、
履带速度投影、输入超时归零、急停锁存和复位后保持禁用。

## MID360 外参填写

外参按底盘 profile 分开保存，只修改实际使用平台对应的文件：

- MK-mini：[`src/agt_description/config/mk_mini_mid360.yaml`](src/agt_description/config/mk_mini_mid360.yaml)
- BUNKER：[`src/agt_description/config/bunker_mid360.yaml`](src/agt_description/config/bunker_mid360.yaml)

不要同时维护两份相同外参，也不要直接修改 Xacro 或 launch 文件中的同名数值。

```yaml
lidar_x: 0.0       # 米，向前为正
lidar_y: 0.0       # 米，向左为正
lidar_z: 0.50      # 米，向上为正
lidar_roll: 0.0    # 弧度，绕 X 轴
lidar_pitch: 0.0   # 弧度，绕 Y 轴
lidar_yaw: 0.0     # 弧度，绕 Z 轴
calibration_verified: false
```

以上参数表示所选底盘的 `base_link -> lidar_link`。标定并实机验证后，将
`calibration_verified` 改为 `true`。临时试验可使用 launch 参数覆盖，但不会改写标定文件：

```bash
ros2 launch agt_description description.launch.py \
  lidar_x:=0.12 lidar_z:=0.63 lidar_pitch:=-0.0872665
```

## 命名规范

- package、topic、参数和文件名统一使用小写 `snake_case`。
- ROS package 统一以 `agt_` 开头；节点名使用 `agt_<模块>_<功能>`。
- 标准 frame 不带前导 `/`：`map`、`odom`、`base_footprint`、`base_link`、`lidar_link`、`imu_link`。
- `livox_frame` 仅作为旧驱动兼容 frame；V2 模块接口统一使用 `lidar_link`。
- MID360 到 FAST-LIVO2 的后端输入使用 `/agt/sensors/lidar/custom`；跨模块点云统一使用
  PointCloud2。不要把 Livox `CustomMsg` 扩散到地图处理、感知和导航模块。
- V2 topic 放在 `/agt/<领域>/<名称>` 下，例如 `/agt/sensors/lidar/points`。
- launch 参数和 YAML key 使用相同名称；长度用米，角度用弧度。
- TF 发布责任固定：全局定位发布 `map -> odom`，连续里程计发布
  `odom -> base_footprint`，机器人描述发布 `base_footprint -> sensor`。

## 顶层目录
```text
agt_navigation_v2/
├── docs/
├── profiles/
├── runtime/
├── src/
├── tests/
├── third_party/
├── tools/
├── AGENTS.md
├── nav_dependencies.repos
└── README.md
```

## 核心功能包
- `agt_interfaces`
- `agt_description`
- `agt_bringup`
- `agt_sensor_adapters`
- `agt_mapping`
- `agt_map_processing`
- `agt_localization`
- `agt_localization_fusion`
- `agt_perception`
- `agt_navigation`
- `agt_coverage_planning`
- `agt_safety`
- `agt_chassis`
- `agt_ui_bridge`
- `agt_experiment_manager`
- `agt_evaluation`

## 构建与验证

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
colcon build --symlink-install --allow-overriding fast_livo relocalization_core
source install/setup.bash
colcon test
colcon test-result --verbose
python3 -m pytest -q \
  tests \
  src/agt_description/test \
  src/agt_mapping/test \
  src/agt_map_processing/test \
  src/agt_ui_bridge/test
```

当前离线测试覆盖 package 命名、launch 语法、TF 拓扑、外参配置唯一性、FAST-LIVO2
位姿与速度外参换算、地图保存、Qt5 配置、Nav2 接口，以及 BUNKER 履带限速、急停和
上游补丁契约。当前完整命令结果为 `46 passed`。离线测试通过不代表算法精度或实车安全验收通过。

Nav2 无车闭环测试：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agt_navigation offline_navigation.launch.py

# 另开终端发送 1 m 测试目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0}, orientation: {w: 1.0}}}}"
```

当前实测八个 lifecycle 节点全部 `active`，1 m 目标约 4.2 s 返回 `SUCCEEDED`。
该入口仅使用测试地图、空障碍点云和运动学模拟器，不评价真实定位或避障精度。

## 模块验收清单

| 模块 | 当前可离线完成/状态 | 后续需要补充的测试 |
| --- | --- | --- |
| `agt_interfaces` | package 骨架和命名检查已完成 | 定义接口后做消息生成、序列化和兼容性测试 |
| `agt_description` | Xacro 展开、URDF、TF 单父节点和 MK-mini/BUNKER profile 检查已完成 | 实测 BUNKER 基准高度、履带中心距与 MID360 外参；实机检查方向和 footprint |
| `agt_bringup` | 完整导航组合入口和离线启动入口已完成 | 用真实传感器/地图做异常退出、生命周期和重启测试 |
| `agt_sensor_adapters` | 已迁入 Livox 驱动，MID360 配置、统一 topic remap 和 launch 离线检查已完成 | 需要 MID360 实机或 bag，验证点云/IMU topic、frame、QoS、频率、时间戳和丢包 |
| `agt_mapping` | adapter、统一 topic、位姿/twist 外参换算及 TF 补丁离线测试已完成 | 需要应用补丁后的 FAST-LIVO2、同一 bag 对比轨迹/点云；实机检查漂移和 TF 唯一发布源 |
| `agt_map_processing` | 已迁移 OctoMap 在线投影与二维 OccupancyGrid 保存入口 | 用当前 bag 调整高度阈值，对比新旧栅格完整性与处理耗时；后续增加 PCD 和地面分割后端 |
| `agt_localization` | ICP/NDT core、局部点云输入、外参修正和 `map -> odom` 已落地并编译 | 需要与栅格图同源的全局 PCD，测试成功率、误差、恢复时间和错误初值拒绝 |
| `agt_localization_fusion` | package 边界已建立 | 需要 LIO、轮速、IMU，后续 RTK/UWB 数据；测试延迟、漂移、跳变和传感器失效降级 |
| `agt_perception` | base frame 高度/量程/车体裁剪障碍点云 baseline 已编译 | 需要标注或典型场景点云，测试地面/障碍精度、误检漏检和处理频率 |
| `agt_navigation` | Nav2 核心、局部/全局 costmap、碰撞监控、Qt action 和运动学闭环已通过 | 用真实地图/定位测试规划成功率、跟踪误差、恢复行为和窄通道通过性 |
| `agt_coverage_planning` | package 边界已建立 | 需要地块边界和障碍数据，测试覆盖率、重复率、转弯次数及路径可执行性 |
| `agt_safety` | BUNKER 履带仲裁、急停锁存、限速、超时和合成消息测试已完成 | 架空履带后做低速实车制动距离、急停和进程/通信中断测试 |
| `agt_chassis` | 官方 bunker_ros2、状态桥接、TF 隔离和双层命令 watchdog 已落地并编译 | 需要 BUNKER CAN 实机验证协议版本、轮速里程计、状态错误码和断连归零 |
| `agt_ui_bridge` | Qt5 上游版本已编译并完成地图接口、NavigateToPose 桥接和干净退出测试 | 用实机验证目标下发和手动速度链 |
| `agt_experiment_manager` | profile 骨架已建立 | 实现后测试配置合并、Git/参数快照、产物命名、失败恢复和复现实验 |
| `agt_evaluation` | package 边界已建立 | 指标实现后用合成轨迹单测；有 bag/真值后生成定位、导航、资源占用报告 |

## 后续数据与实机准备

当前已有可重复播放的 MID360 建图 bag。后续还需要补充静止、直线、原地转向、温室窄通道
四类验收片段，以及 BUNKER 的 CAN 状态、轮速、软件命令和急停记录。每次实验应记录传感器
安装尺寸、ROS 2 版本、提交版本和参数快照。bag 放入 `runtime/rosbag/`，地图/PCD 放入
`runtime/maps/`，实验结果放入 `runtime/results/`；这些 runtime 产物默认不提交 Git。

Phase 3 有数据后的最低验收项：新旧注册点云数量和时间戳一致，转换后的
`/agt/mapping/odometry` 连续，`odom -> base_footprint` 只有一个发布源，轨迹相对旧链无
非预期跳变，并保存对比报告到 `runtime/results/`。

## 当前测试数据

已检查 `runtime/rosbag/mid360_mapping_20260603_195044`：196.116885 秒，包含 1962 帧
MID360 PointCloud2、39201 帧 IMU，以及旧链注册点云、里程计、TF 和投影地图。原始
PointCloud2 完整保留 `timestamp/line/tag`，可重建 FAST-LIVO2 所需 CustomMsg。

派生输入位于 `runtime/rosbag/mid360_mapping_custom_full`，包含：

- `/agt/sensors/lidar/custom`：1962 帧 `livox_ros_driver2/msg/CustomMsg`。
- `/agt/sensors/imu/data`：39201 帧 `sensor_msgs/msg/Imu`。

两份 bag 起止时间一致。转换工具见
[`tools/bag_tools/convert_mid360_pointcloud2_to_custom.py`](tools/bag_tools/convert_mid360_pointcloud2_to_custom.py)。
当前已验证算法分支 `a713004` 加 TF/CMake 补丁可以编译并完成实际回放，注册点云 frame、
QoS 和 OctoMap 二维栅格输出链路均已通过。新旧轨迹与点云数值精度对比仍待生成正式报告。

详细 TF 约束见 [`src/agt_description/README.md`](src/agt_description/README.md)，
迁移进度见 [`docs/migration/migration_matrix.md`](docs/migration/migration_matrix.md)。

## 下一阶段优先级

1. 固定旧仓库 tag/commit、参数快照和当前 V2 Git 状态，补齐可复现基线记录。
2. 标定车辆 `base_link -> lidar_link` 外参，并实测 BUNKER 的 `base_link` 高度与履带中心距。
3. 使用完整 bag 对比新旧注册点云、轨迹和二维地图，固定 OctoMap 高度阈值并生成正式报告。
4. 从同一次建图导出全局 PCD，完成 ICP/NDT 重定位和 `map -> odom` 回放验收。
5. 架空履带完成 CAN、方向、轮速、双 watchdog 和硬件急停测试，再以不高于 `0.15 m/s`
   做空旷场地制动距离测试。
6. 使用离线 Nav2 baseline 先完成恢复行为、障碍注入和参数扫描，再集中进行真实地图、定位、
   窄通道和 Qt5 目标下发调试。

## 系统总控

BUNKER 平台统一使用 `agt_bringup/system.launch.py`。总控已包含 BUNKER 描述、传感器、
FAST-LIVO2、地图处理、导航、安全层、底盘和 Qt5 的条件启动。运行总控时不要再单独启动
`description.launch.py`、`bunker_description.launch.py` 或 `bunker.launch.py`，否则会重复
启动 robot_state_publisher、固定 TF 或同名节点。

每个终端先执行：

```bash
cd /home/yangxuan/agt_navigation_v2
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
source install/setup.bash
```

### 建图模式

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=mapping map_name:=greenhouse_01 record_bag:=true
```

建图模式会启动唯一一份 BUNKER TF、MID360、FAST-LIVO2、OctoMap 二维投影、底盘安全链
和 Qt5，并强制开启 FAST-LIVO2 PCD 保存。`record_bag:=true` 会同时记录传感器、TF、
里程计、地图、导航、安全和底盘诊断话题到 `runtime/rosbag/mapping_<时间>/`。

建图过程中在另一个终端保存当前二维地图，必须保持建图总控运行：

```bash
ros2 launch agt_bringup save_mapping_result.launch.py map_name:=greenhouse_01
```

随后在总控终端使用 `Ctrl+C` 正常退出，FAST-LIVO2 才会写出完整 PCD：

```text
runtime/maps/greenhouse_01/greenhouse_01.pgm
runtime/maps/greenhouse_01/greenhouse_01.yaml
runtime/maps/greenhouse_01/pcd/all_raw_points.pcd
runtime/maps/greenhouse_01/pcd/all_downsampled_points.pcd
```

重定位优先使用 `all_downsampled_points.pcd`。不要用 `kill -9` 结束建图，否则 PCD 和 bag
元数据可能来不及落盘。安全层仍默认禁止运动，现场检查完成后再显式调用
`/agt/safety/set_motion_enabled`。

### Bag 离线建图

```bash
# 终端 1：总控，不启动真实雷达、CAN 和 Qt5
ros2 launch agt_bringup system.launch.py \
  mode:=mapping map_name:=mid360_bag_test use_sim_time:=true \
  start_sensor:=false start_chassis:=false start_gui:=false

# 终端 2：回放转换后的 CustomMsg + IMU
ros2 bag play runtime/rosbag/mid360_mapping_custom_full --clock

# 终端 3：回放结束后、总控退出前保存二维地图
ros2 launch agt_bringup save_mapping_result.launch.py map_name:=mid360_bag_test
```

保存二维图后，再对终端 1 使用 `Ctrl+C` 生成两份 PCD。RViz2 的 `Fixed Frame` 使用 `odom`，
地图 topic 为 `/agt/map/global_occupancy`。

### 导航模式

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  map:=/home/yangxuan/agt_navigation_v2/runtime/maps/greenhouse_01/greenhouse_01.yaml \
  global_map_pcd:=/home/yangxuan/agt_navigation_v2/runtime/maps/greenhouse_01/pcd/all_downsampled_points.pcd \
  record_bag:=true
```

导航模式强制设置 `save_pcd:=false`：FAST-LIVO2 只提供稳定的
`/agt/mapping/odometry` 和当前帧点云，不积累或覆盖建图 PCD。ICP/NDT 发布 `map -> odom`，
Nav2、Collision Monitor、安全层与 BUNKER 底盘依次启动，Qt5 默认自动打开。

Qt5 可发布 `/initialpose` 和 `/goal_pose`，目标会转换为 NavigateToPose action。地图编辑结果
需保存为新的 PGM/YAML，再用新的 `map:=...` 重启导航；当前不会把正在编辑的地图热替换到
运行中的全局 costmap。无显示环境可设置 `start_gui:=false`，无 CAN 联调可设置
`start_chassis:=false`。详细接口见 [`src/agt_bringup/README.md`](src/agt_bringup/README.md)。
