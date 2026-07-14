# agt_navigation

职责：承载 Nav2 规划、控制、行为树、碰撞监控、生命周期和标准导航接口。

当前 BUNKER baseline 使用 SmacPlanner2D、SimpleSmoother、MPPI DiffDrive、Nav2 BT、
WaypointFollower 和 Collision Monitor。ICP/NDT 负责 `map -> odom`，因此不启动 AMCL；
MPPI 与 `agt_safety` 已分别提供控制优化和加速度/超时约束，因此暂不叠加 velocity_smoother。
global costmap 按 `StaticLayer -> KeepoutFilter -> InflationLayer` 处理语义禁行区，local
costmap 障碍点云链保持不变。

速度链固定为：

```text
MPPI /agt/navigation/cmd_vel_raw
  -> Collision Monitor /agt/navigation/cmd_vel
  -> agt_safety /agt/safety/cmd_vel
  -> agt_chassis /agt/chassis/cmd_vel
```

## 无车离线闭环

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agt_navigation offline_navigation.launch.py
```

该入口使用随包测试地图、空障碍点云和差速履带运动学模拟器，且仅在离线入口中自动
使能安全层。另开终端发送目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0}, orientation: {w: 1.0}}}}"
```

验证 Collision Monitor stop 区时启动固定在车前 `0.7 m` 的合成障碍：

```bash
ros2 launch agt_navigation offline_navigation.launch.py \
  synthetic_obstacle_enabled:=true synthetic_obstacle_x:=0.7
```

此模式下向前目标应保持停车并最终由进度检查器中止；不要把该参数用于真实系统入口。

也可向 Qt5 使用的 `/goal_pose` 发布 `geometry_msgs/PoseStamped`；`goal_pose_bridge.py`
会转换为 NavigateToPose action，并在 `/agt/navigation/status` 发布桥接状态。

## 接真实地图

先启动机器人描述、LIO、障碍过滤和重定位，再执行：

```bash
ros2 launch agt_navigation navigation.launch.py \
  map:=/absolute/path/to/navigation_map.yaml use_sim_time:=false
```

局部代价地图与 Collision Monitor 都订阅 `/agt/perception/obstacle_cloud`。实车运动前
必须完成雷达外参、footprint、履带中心距、低矮障碍和制动距离验收；真实启动不会自动
使能 `agt_safety`。

## 启用语义禁行区

Filter Info Server 默认不启动，KeepoutFilter 仅等待数据，因此原导航和离线闭环不依赖
语义服务器。启用时先准备语义任务，然后分别启动语义服务器和 Nav2；语义服务器会等待
Nav2 基础地图到达：

```bash
SEMANTIC_MAP=runtime/maps/greenhouse_01/semantic/semantic_map.geojson
PLATFORM_PROFILE=profiles/platforms/bunker.yaml
NAV_MAP=runtime/maps/greenhouse_01/greenhouse_01.yaml

ros2 launch agt_ui_bridge semantic_map_server.launch.py \
  semantic_map:="$SEMANTIC_MAP" platform_profile:="$PLATFORM_PROFILE"

ros2 launch agt_navigation navigation.launch.py \
  map:="$NAV_MAP" use_keepout_filter:=true
```

运动前必须确认语义状态为 `LOADED`，并检查 FilterInfo 与 mask 均存在：

```bash
ros2 topic echo /agt/map/semantic_status --once
ros2 topic echo /agt/map/keepout_filter_info --once
ros2 topic echo /agt/map/keepout_mask --once
```

ROS 2 Humble 的 KeepoutFilter 在 FilterInfo 或 mask 缺失时会告警并继续使用基础地图，属于
fail-open；此时不得依赖语义禁行区保障安全。语义服务器异常退出后，costmap 保留最后收到的
有效 mask；加载新语义图成功时原子替换，加载失败时继续保留旧 mask。

运行时临时关闭或恢复语义过滤应使用插件服务，而不是直接改参数：

```bash
ros2 service call /global_costmap/keepout_filter/toggle_filter \
  std_srvs/srv/SetBool "{data: false}"
ros2 service call /global_costmap/keepout_filter/toggle_filter \
  std_srvs/srv/SetBool "{data: true}"
```

关闭后 global costmap 从只读基础地图重新合成，不会修改 `/agt/map/global_occupancy` 或源 PGM。
