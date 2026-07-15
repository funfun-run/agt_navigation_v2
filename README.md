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
| Phase 8：Qt5 与覆盖规划 | TASK-00~15 完成 | 语义/Keepout、覆盖 Action 及总控条件启动和 fail-closed 参数校验通过 | TASK-16 增加记录、覆盖质量指标和可复现报告 |

当前离线回归结果：`185 passed`；BUNKER 无 CAN 运行测试已验证默认禁用、手动优先、
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

### 车辆几何单一数据源

车辆外形与导航 footprint 统一以 `profiles/platforms/<platform>.yaml` 为真源。当前 BUNKER
导航 footprint 为车辆外形四周增加 80 mm 安全裕量；Nav2 局部/全局 costmap 和 perception
车体点云裁剪由 `tests/test_vehicle_geometry_contracts.py` 检查是否与 profile 一致。后续覆盖
路径 Validator 必须读取所选平台 profile，不得在 coverage 配置中复制 footprint 或再次叠加
另一套安全裕量。

### 语义地图合同

农业语义对象独立保存为 GeoJSON 与 `coverage.yaml`，统一使用 `map` frame、米制坐标和 ROS
右手坐标系，不写入基础 PGM。1.0 格式、Feature 类型、哈希规则和错误策略见
[`docs/interfaces/semantic_map_schema.md`](docs/interfaces/semantic_map_schema.md)，版本化合法/非法
样例位于 `docs/interfaces/examples/semantic_map/`。实际任务文件写入
`runtime/maps/<map_id>/semantic/`，默认不提交 Git。

TASK-02 已提供无 Qt/ROS 依赖的 `agt_ui_bridge` Python 基础库，统一处理 PGM Y 翻转、非零
origin 与 yaw、GeoJSON/YAML 重载、SHA256 只读降级、原子写入和 scene undo/redo。TASK-03
已新增独立 Qt5 语义编辑器，支持对象绘制、顶点编辑、图层、保存重载和未保存退出提示。
TASK-04 使用 Shapely 检查多边形自交、区域包含、地图范围、入口约束、边界净距及入口
navigation footprint 可行性；错误会关联对象 ID、高亮并阻止保存。TASK-05 已新增事务式
语义地图服务器、标准 markers/mask/status 和 load/reload/validate 服务。TASK-06 已将 enabled
exclusion/keepout 及默认 field 外部栅格化到严格对齐的 OccupancyGrid。TASK-07 已接入 Nav2
FilterInfo 与 global KeepoutFilter，并在 keepout 后执行 inflation。
TASK-08 已将 Humble `opennav_coverage humble-v2`、Fields2Cover `v2.0.0` 及其传递源码固定到
完整 commit，并在不 source 旧工作区的纯净工作区完成 rosdep、4 个目标包构建和 action 核验。
TASK-09 已实现 semantic/profile 到 `ComputeCoveragePath` 的 polygon 与 annotated rows 适配；
真实服务器分别生成 174/161 个 `map` frame 姿态，孔洞和 orientation 检查通过。TASK-10 已实现
基于全局 costmap、canonical footprint、距离/角度插值及曲率的 Validator；失败时清空验证路径，
原始路径永远禁止直接执行。TASK-11 已从锁定版 PathComponents 保留 SWATH/CONNECTION 语义，
提供稳定作业行编号、扁平路径重建、长度误差和 Path 指纹合同，Validator 可报告无效 swath ID。
TASK-12 已实现仅修复无效 CONNECTION：调用 profile 指定 Nav2 planner，直接检查 global costmap
与 keepout mask，锁定连接端点并保证所有 SWATH Pose 数值不变，最终再次通过 Validator。
TASK-13/14 已生成统一 Action 并实现可取消状态机；TASK-15 已把语义服务器、Keepout Filter、
覆盖规划和标注模式接入 `agt_bringup`，默认关闭时不改变原导航节点集合。
接口见
[`docs/interfaces/coverage_planning.md`](docs/interfaces/coverage_planning.md)。

已编辑完成的语义地图可先用纯离线入口查看 Fields2Cover 路线，不启动定位、控制器、安全链或
底盘，且固定禁止执行：

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
COVERAGE_WS=${COVERAGE_WS:-$HOME/agt_coverage_ws}
source "$COVERAGE_WS/install/setup.bash"
source install/setup.bash

# 三项都必须有输出；缺失时按覆盖依赖文档创建外部工作区。
ros2 pkg prefix opennav_coverage_msgs
ros2 pkg prefix opennav_coverage
ros2 pkg prefix opennav_row_coverage

ros2 launch agt_coverage_planning coverage_preview.launch.py \
  map:="$(realpath runtime/maps/mid360_map/mid360_map.yaml)" \
  semantic_map:="$(realpath runtime/maps/mid360_map/semantic/semantic_map.geojson)" \
  platform_profile:="$(realpath profiles/platforms/bunker.yaml)"
```

RViz 红线是 Coverage Server 的只读 `path_preview`，青线是通过 SWATH/CONNECTION 语义重建的
路线，黄色 Marker 是作业行，半透明区域是 keepout mask。`path_preview` 不进入 Validator 或
执行链；轻量预览不提供 global costmap，因此绿色 validated path 为空是预期。当前
`mid360_map` 已实测生成 `679` 个预览姿态，但 OpenNav 返回的 PathComponents 含零长度 SWATH，
所以语义重建仍会 fail-closed 并报告 `zero_length_swath`。外部工作区的固定版本导入与构建命令见
[`docs/development/coverage_dependencies.md`](docs/development/coverage_dependencies.md)。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agt_ui_bridge semantic_editor.launch.py \
  map:=runtime/maps/greenhouse_01/greenhouse_01.yaml \
  platform_profile:=profiles/platforms/bunker.yaml
```

语义导航默认不启用，以保持原导航兼容。TASK-15 后统一通过总控同时启用语义服务器、Nav2
Keepout Filter 与覆盖模块，不再分别启动：

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  map:=/absolute/path/greenhouse_01.yaml \
  global_map_pcd:=/absolute/path/all_downsampled_points.pcd \
  semantic_map:=/absolute/path/semantic_map.geojson \
  coverage_params:=/absolute/path/coverage.yaml \
  start_semantic_map_server:=true \
  start_coverage_planning:=true
```

Humble 在 mask 缺失时会 fail-open；运动前必须确认 `/agt/map/semantic_status` 为 `LOADED`。
完整启动、检查和运行时启停命令见 [`src/agt_navigation/README.md`](src/agt_navigation/README.md)。

语义结果建议保存到 `runtime/maps/<map_id>/semantic/`。详细操作和重载命令见
[`src/agt_ui_bridge/README.md`](src/agt_ui_bridge/README.md)。

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

覆盖规划外部依赖由 [`nav_dependencies.repos`](nav_dependencies.repos) 固定到 commit，必须在
独立工作区导入和构建。TASK-08 的系统依赖、`vcs import`、rosdep、最小构建及版本核验流程见
[`docs/development/coverage_dependencies.md`](docs/development/coverage_dependencies.md)。

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
source /path/to/agt_coverage_ws/install/setup.bash
colcon build --symlink-install --allow-overriding fast_livo relocalization_core
source install/setup.bash
colcon test
colcon test-result --verbose
python3 -m pytest -q \
  tests \
  src/agt_description/test \
  src/agt_coverage_planning/test \
  src/agt_interfaces/test \
  src/agt_mapping/test \
  src/agt_map_processing/test \
  src/agt_ui_bridge/test
```

当前离线测试覆盖 package 命名、launch 语法、TF 拓扑、外参配置唯一性、FAST-LIVO2
位姿与速度外参换算、地图保存、Qt5 配置、Nav2 接口，以及 BUNKER 履带限速、急停和
上游补丁契约、车辆几何单一数据源、语义地图基础库、Qt5 编辑器、覆盖请求适配和 Coverage
Path Validator、SWATH/CONNECTION 路径语义、无效 CONNECTION 事务修复，以及覆盖任务 Action 的
序列化、阶段反馈、安全门禁、取消传播和 TASK-15 总控前置契约。当前完整命令结果为
`185 passed`；C++ 生成头文件另由
`colcon test` 编译运行。
离线测试通过不代表算法精度或实车安全验收通过。

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
| `agt_interfaces` | TASK-13/14 完成：生成的 `ExecuteCoverageTask` 已由可取消服务端消费，Python/C++ 类型与序列化通过 | 后续字段变更做兼容性评审，并保持服务端与客户端同步 |
| `agt_description` | Xacro 展开、URDF、TF 单父节点和 MK-mini/BUNKER profile 检查已完成 | 实测 BUNKER 基准高度、履带中心距与 MID360 外参；实机检查方向和 footprint |
| `agt_bringup` | TASK-15 完成：语义、Keepout、覆盖与标注模式条件启动，路径前置校验和录包扩展通过 | 用真实地图验证 readiness 顺序、异常退出、Action 关闭和节点重启 |
| `agt_sensor_adapters` | 已迁入 Livox 驱动，MID360 配置、统一 topic remap 和 launch 离线检查已完成 | 需要 MID360 实机或 bag，验证点云/IMU topic、frame、QoS、频率、时间戳和丢包 |
| `agt_mapping` | adapter、统一 topic、位姿/twist 外参换算及 TF 补丁离线测试已完成 | 需要应用补丁后的 FAST-LIVO2、同一 bag 对比轨迹/点云；实机检查漂移和 TF 唯一发布源 |
| `agt_map_processing` | 已迁移 OctoMap 在线投影与二维 OccupancyGrid 保存入口 | 用当前 bag 调整高度阈值，对比新旧栅格完整性与处理耗时；后续增加 PCD 和地面分割后端 |
| `agt_localization` | ICP/NDT core、局部点云输入、外参修正和 `map -> odom` 已落地并编译 | 需要与栅格图同源的全局 PCD，测试成功率、误差、恢复时间和错误初值拒绝 |
| `agt_localization_fusion` | package 边界已建立 | 需要 LIO、轮速、IMU，后续 RTK/UWB 数据；测试延迟、漂移、跳变和传感器失效降级 |
| `agt_perception` | base frame 高度/量程/车体裁剪障碍点云 baseline 已编译 | 需要标注或典型场景点云，测试地面/障碍精度、误检漏检和处理频率 |
| `agt_navigation` | Nav2 核心、运动学闭环和 TASK-07 global KeepoutFilter 阻断/恢复规划已通过 | 用真实地图/定位测试语义边界、规划成功率、跟踪误差和窄通道通过性 |
| `agt_coverage_planning` | TASK-15 完成：可取消 Action 已由总控条件启动，普通导航和标注模式保持 fail-closed | TASK-16 增加覆盖率、重叠率和任务日志 |
| `agt_safety` | BUNKER 履带仲裁、急停锁存、限速、超时和合成消息测试已完成 | 架空履带后做低速实车制动距离、急停和进程/通信中断测试 |
| `agt_chassis` | 官方 bunker_ros2、状态桥接、TF 隔离和双层命令 watchdog 已落地并编译 | 需要 BUNKER CAN 实机验证协议版本、轮速里程计、状态错误码和断连归零 |
| `agt_ui_bridge` | TASK-07 完成；语义 mask 已由 Nav2 消费且切换/禁用不写回基础地图 | 用真实地图验证服务器异常、语义切换和操作门禁 |
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

### CAN 与 BUNKER 通讯测试

该测试只启动 CAN、BUNKER 官方驱动、状态桥接和安全层，不启动 MID360、FAST-LIVO2、
Nav2 或 Qt5。首次测试应架空两侧履带，准备好实体遥控器和硬件急停，并保持软件运动默认
禁用；确认通讯不需要调用 `/agt/safety/set_motion_enabled`。

> **当前硬件风险记录：** 现用 CAN 模块连接笔记本电脑时，车辆移动和线缆晃动可能造成
> 接头松动或瞬时断连，因此该组合仅用于静态和架空低速联调。有条件时应更换带可靠固定、
> 应力释放或锁紧接头的 CAN 模块，并改为与车载工控机固定连接后再进行连续移动测试。
> 若移动后出现 `candump` 断流、ROS 状态话题停止、`connected` 变为 `false`、CAN 错误计数
> 增长或接口进入 `BUS-OFF`，应先检查模块、USB/CAN 接头和线缆固定，不要直接归因于驱动。

以下命令均在仓库根目录执行。终端 1 配置 SocketCAN；`CAN_IFACE` 默认使用 `can0`，实际
接口不同可在第一行修改：

CAN 接口配置需要宿主机 root/`CAP_NET_ADMIN` 权限，不能在 Codex、浏览器沙箱或启用了
`no-new-privileges` 的容器终端中执行。先在系统原生终端检查 `NoNewPrivs`，结果必须为 `0`；
若为 `1`，请关闭该受限终端，通过桌面应用菜单或 `Ctrl+Alt+T` 打开新的宿主机终端：

```bash
grep NoNewPrivs /proc/$$/status
CAN_IFACE=${CAN_IFACE:-can0}

sudo modprobe gs_usb
sudo ip link set "$CAN_IFACE" down 2>/dev/null || true
sudo ip link set "$CAN_IFACE" up type can bitrate 500000
ip -details -statistics link show "$CAN_IFACE"
timeout 5 candump "$CAN_IFACE"
```

`ip` 输出应显示接口为 `UP`、CAN 状态为 `ERROR-ACTIVE`、bitrate 为 `500000`；BUNKER 上电后
`candump` 应持续出现 CAN 帧。若没有 `candump` 命令，先安装 `can-utils`。保持车辆上电，
然后在终端 1 启动底盘通讯节点：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
CAN_IFACE=${CAN_IFACE:-can0}

ros2 launch agt_chassis bunker.launch.py \
  can_interface:="$CAN_IFACE"
```

驱动日志应显示检测到 `AGX_V1` 或 `AGX_V2`，随后显示正在通过 CAN 与机器人通讯。终端 2
检查 ROS 接口；`timeout` 到时退出属于正常现象：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

timeout 5 ros2 topic echo /agt/chassis/connected --once
timeout 5 ros2 topic echo /agt/chassis/status --once
timeout 5 ros2 topic hz /agt/chassis/status/raw
timeout 5 ros2 topic hz /agt/chassis/odometry
timeout 5 ros2 topic echo /agt/chassis/rc_state --once
```

通讯正常的最低判据：

- `/agt/chassis/connected` 返回 `data: true`。
- `/agt/chassis/status` 的 `level` 为 `0`、`message` 为 `connected`。
- `/agt/chassis/status/raw` 和 `/agt/chassis/odometry` 持续更新。
- 操作实体遥控器时 `/agt/chassis/rc_state` 有响应；架空履带低速转动时 odometry 速度变化。
- 对实际 CAN 接口再次执行 `ip -details -statistics link show "$CAN_IFACE"` 时没有进入
  `BUS-OFF`，错误计数不持续增长。

这里只验证通讯和反馈，不测试 ROS 软件控车。检查完成后将遥控器切回停止位置，并在驱动终端
使用 `Ctrl+C` 正常退出。若 `candump` 无数据、`connected` 为 `false`、驱动无法识别协议或
`bunker_base_node` 退出，应停止测试并优先检查底盘供电、CAN-H/CAN-L、终端电阻、bitrate、
USB-CAN 驱动和接口名，不要使能运动。

### 外参标定 Bag 采集

这组数据用于联合分析 BUNKER 轮速里程计与 FAST-LIVO2 轨迹，优化并验证
`base_link -> lidar_link`。开始前先用卷尺、水平仪和铅垂线测量 `x/y/z/roll/pitch/yaw`
初值并记录；bag 优化用于修正和验证，不能替代机械测量。测试区域应空旷、地面较平、具有
墙面或立柱等稳定几何特征，硬件急停和遥控器必须随时可用。

启动总控前检查 CAN。若 `can0` 尚未处于 `UP`，先执行仓库提供的 500 kbit/s 启动脚本；
`candump` 能持续收到状态帧后再继续：

```bash
cd /home/yangxuan/agt_navigation_v2
ip -details link show can0
sudo bash third_party/ugv_sdk/scripts/bringup_can2usb_500k.bash
timeout 3 candump can0
```

终端 1 启动完整标定记录链：

```bash
cd /home/yangxuan/agt_navigation_v2
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
source install/setup.bash

ros2 launch agt_bringup system.launch.py \
  mode:=mapping \
  map_name:=bunker_extrinsic_calibration_01 \
  start_sensor:=true \
  start_chassis:=true \
  start_rviz:=true \
  record_bag:=true
```

启动后不要立即移动车辆。终端 2 检查数据链；三个 `topic hz` 命令会在 5 秒后自动结束：

```bash
cd /home/yangxuan/agt_navigation_v2
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
source install/setup.bash

timeout 5 ros2 topic echo /agt/chassis/connected --once
timeout 5 ros2 topic hz /agt/sensors/lidar/custom
timeout 5 ros2 topic hz /agt/sensors/imu/data
timeout 5 ros2 topic hz /agt/chassis/odometry
timeout 3 ros2 run tf2_ros tf2_echo base_link lidar_link
```

只有 `/agt/chassis/connected` 为 `true`、三类数据持续更新、RViz 点云正常且车辆已架空或处于
封闭空旷场地时，才允许开始运动。推荐标定时使用 BUNKER 实体遥控器，并让遥控器保持手动
接管模式；此时软件运动保持默认禁用，**不需要**调用 `set_motion_enabled`。传感器、
FAST-LIVO2、底盘里程计和 bag 录制不会因为软件运动禁用而停止。

以下服务只在使用 Qt/手柄节点或 ROS topic 发布 `/agt/cmd_vel_manual` 时调用，用于放行软件
速度链；它不负责启动车辆、传感器、建图或录包：

```bash
ros2 service call /agt/safety/set_motion_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

全程速度不高于 `0.15 m/s`，按顺序采集并在每段之间静止约 5 秒：静止 30 秒、直线前进、
直线后退、左大圆弧、右大圆弧、左原地转向、右原地转向、结束静止 30 秒。直线和圆弧建议
各重复 2～3 次；履带原地转向滑移较大，只用于提供旋转激励，不作为平移真值。

如果使用软件速度链，动作完成后先禁止软件运动；如果全程使用实体遥控器，则保持软件运动
禁用并将遥控器切回安全/停止位置。随后回到终端 1 使用 `Ctrl+C` 正常结束总控和录包：

```bash
ros2 service call /agt/safety/set_motion_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

bag 默认写入 `runtime/rosbag/mapping_<时间>/`，终端日志会显示准确目录。结束后用日志中的
实际目录检查内容：

```bash
ros2 bag info runtime/rosbag/mapping_YYYYMMDD_HHMMSS
```

标定 bag 至少应包含 `/agt/sensors/lidar/custom`、`/agt/sensors/imu/data`、
`/agt/mapping/odometry`、`/agt/chassis/odometry`、`/agt/chassis/status`、
`/agt/chassis/rc_state`、`/tf` 和 `/tf_static`，且持续时间应覆盖完整动作。若轮速里程计缺失
或时间戳不连续，该 bag 只能用于地面拟合和 FAST-LIVO2 检查，不能可靠求解完整车体外参。

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
FAST-LIVO2、地图处理、RViz、导航、安全层、底盘、Qt5、语义服务器和覆盖规划的条件启动。建图模式默认打开
RViz，导航模式默认打开 Qt5。运行总控时不要再单独启动
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
和专用 RViz，并强制开启 FAST-LIVO2 PCD 保存。RViz 的 `Fixed Frame` 已设置为 `odom`，
默认显示 `/agt/mapping/registered_points` 和 `/agt/map/mapping_occupancy`。Qt5 不在建图模式启动。
`record_bag:=true` 会同时记录传感器、TF、
里程计、地图、导航、安全和底盘诊断话题到 `runtime/rosbag/mapping_<时间>/`。

结束建图时，先保持总控运行，在另一个终端保存二维地图：

```bash
ros2 launch agt_bringup save_mapping_result.launch.py map_name:=greenhouse_01
```

看到 `Map saved` 后，再回到总控终端使用一次 `Ctrl+C` 正常退出。退出过程会让
FAST-LIVO2 写出完整 PCD，同时让 rosbag 写完元数据：

```text
runtime/maps/greenhouse_01/greenhouse_01.pgm
runtime/maps/greenhouse_01/greenhouse_01.yaml
runtime/maps/greenhouse_01/pcd/all_raw_points.pcd
runtime/maps/greenhouse_01/pcd/all_downsampled_points.pcd
```

必须先保存二维地图再退出总控；如果先按 `Ctrl+C`，OctoMap 发布者会关闭，随后无法可靠
保存 PGM/YAML。重定位优先使用 `all_downsampled_points.pcd`。不要用 `kill -9` 结束建图，
否则 PCD 和 bag 元数据可能来不及落盘。安全层仍默认禁止运动，现场检查完成后再显式调用
`/agt/safety/set_motion_enabled`。

### Bag 离线建图

```bash
# 终端 1：总控，不启动真实雷达、CAN 和 RViz
ros2 launch agt_bringup system.launch.py \
  mode:=mapping map_name:=mid360_bag_test use_sim_time:=true \
  start_sensor:=false start_chassis:=false start_rviz:=false

# 终端 2：回放转换后的 CustomMsg + IMU
ros2 bag play runtime/rosbag/mid360_mapping_custom_full --clock

# 终端 3：回放结束后、总控退出前保存二维地图
ros2 launch agt_bringup save_mapping_result.launch.py map_name:=mid360_bag_test
```

保存二维图后，再对终端 1 使用 `Ctrl+C` 生成两份 PCD。需要离线观察效果时不要设置
`start_rviz:=false`；建图 RViz 配置会自动使用 `odom` 和 `/agt/map/mapping_occupancy`。

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

完整覆盖作业模式增加以下参数；启动前还必须 source TASK-08 外部覆盖依赖工作区：

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  map:=/absolute/path/greenhouse_01.yaml \
  global_map_pcd:=/absolute/path/all_downsampled_points.pcd \
  semantic_map:=/absolute/path/semantic_map.geojson \
  coverage_params:=/absolute/path/coverage.yaml \
  start_semantic_map_server:=true \
  start_coverage_planning:=true \
  record_bag:=true
```

总控默认不启用语义和覆盖模块，因此原导航不受影响。`annotation_mode:=true` 会打开项目语义
编辑器而非普通 Qt5，并禁止覆盖路径执行。详细 readiness 与安全检查见
[`src/agt_bringup/README.md`](src/agt_bringup/README.md)。

Qt5 可发布 `/initialpose` 和 `/goal_pose`，目标会转换为 NavigateToPose action。地图编辑结果
需保存为新的 PGM/YAML，再用新的 `map:=...` 重启导航；当前不会把正在编辑的地图热替换到
运行中的全局 costmap。无显示环境可设置 `start_gui:=false`，无 CAN 联调可设置
`start_chassis:=false`。详细接口见 [`src/agt_bringup/README.md`](src/agt_bringup/README.md)。
