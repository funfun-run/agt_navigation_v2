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
- `/agt/safety/cmd_vel`
- `/agt/chassis/status`
- `/agt/experiment/events`

## TF 责任
- `map -> odom`: 全局定位或全局融合模块唯一发布
- `odom -> base_footprint`: 连续里程计或融合模块唯一发布
- `base_footprint -> base_link`: 描述或姿态适配模块维护
- `base_link -> sensor`: `agt_description` 集中管理
