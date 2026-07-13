# agt_navigation

职责：承载 Nav2 规划、控制、行为树、碰撞监控、生命周期和标准导航接口。

当前 BUNKER baseline 使用 SmacPlanner2D、SimpleSmoother、MPPI DiffDrive、Nav2 BT、
WaypointFollower 和 Collision Monitor。ICP/NDT 负责 `map -> odom`，因此不启动 AMCL；
MPPI 与 `agt_safety` 已分别提供控制优化和加速度/超时约束，因此暂不叠加 velocity_smoother。

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
