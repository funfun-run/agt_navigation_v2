# 核心接口草案

## Topics
- `/agt/sensors/lidar/points`
- `/agt/sensors/imu/data`
- `/agt/mapping/odometry`
- `/agt/mapping/registered_cloud`
- `/agt/mapping/status`
- `/agt/perception/ground_cloud`
- `/agt/perception/obstacle_cloud`
- `/agt/perception/semantic_cloud`
- `/agt/localization/status`
- `/agt/navigation/cmd_vel`
- `/agt/navigation/cmd_vel_raw`
- `/agt/navigation/status`
- `/agt/safety/cmd_vel`
- `/agt/safety/emergency_stop`
- `/agt/safety/status`
- `/agt/chassis/cmd_vel`
- `/agt/chassis/odometry`
- `/agt/chassis/status`
- `/agt/chassis/connected`
- `/agt/experiment/events`
- `/agt/map/global_occupancy`: 建图投影与导航 map_server 共用的二维地图接口

## 导航动作与速度链
- `/navigate_to_pose`: `nav2_msgs/action/NavigateToPose`
- `/navigate_through_poses`: `nav2_msgs/action/NavigateThroughPoses`
- `/goal_pose`: Qt5/RViz2 的 `geometry_msgs/PoseStamped` 兼容入口
- `/agt/navigation/cmd_vel_raw -> /agt/navigation/cmd_vel -> /agt/safety/cmd_vel -> /agt/chassis/cmd_vel`

## TF 责任
- `map -> odom`: 全局定位或全局融合模块唯一发布
- `odom -> base_footprint`: 连续里程计或融合模块唯一发布
- `base_footprint -> base_link`: 描述或姿态适配模块维护
- `base_link -> sensor`: `agt_description` 集中管理
