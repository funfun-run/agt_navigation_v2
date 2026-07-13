# agt_map_processing

职责：把全局 PCD 或注册点云转成二维栅格、可通行性地图和对比输出。

## OctoMap 二维投影 baseline

当前已迁移旧仓库的在线 OctoMap 投影链：

- 输入：`/agt/mapping/registered_points_lidar` (`sensor_msgs/msg/PointCloud2`，`lidar_link` frame)。
- 建图输出：`/agt/map/mapping_occupancy` (`nav_msgs/msg/OccupancyGrid`)。
- 默认分辨率：`0.05 m`。
- 默认仅把 `0.10 m <= z <= 1.00 m` 的点作为投影障碍候选。

建图工作图与导航静态图分开：OctoMap 只发布 `/agt/map/mapping_occupancy`，导航模式的
`map_server` 才发布 `/agt/map/global_occupancy`。这样建图 RViz 不会误显示仍在运行的旧导航地图。

FAST-LIVO2 和投影节点都启动后，回放传感器 bag：

```bash
ros2 launch agt_map_processing octomap_projection.launch.py use_sim_time:=true
```

参数集中在 [`config/octomap_projection.yaml`](config/octomap_projection.yaml)。实测前重点根据
地面位置、机器人高度和作物冠层调整 `pointcloud_min_z`、`pointcloud_max_z`、
`occupancy_min_z` 和 `occupancy_max_z`。

生成地图后，在仓库根目录执行：

```bash
cd /home/yangxuan/agt_navigation_v2
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p /home/yangxuan/agt_navigation_v2/runtime/maps
ros2 launch agt_map_processing save_occupancy_map.launch.py \
  map_prefix:=/home/yangxuan/agt_navigation_v2/runtime/maps/mid360_map
```

会生成 `mid360_map.pgm` 和 `mid360_map.yaml`。保存节点使用 transient-local 订阅，投影节点
也保留最后一帧，因此可在回放结束后启动保存命令。该二维图是全局静态地图候选，不包含
Nav2 local costmap 的瞬时局部障碍。

OctoMap 使用当前帧 `lidar_link` 点云和 `odom -> lidar_link` TF，因此射线原点会随机器人
运动。车辆 `base_link -> lidar_link` 外参完成标定和高度阈值调优前，输出地图只用于链路
验证与后端对比，不作为最终导航地图。

## 后续后端

- PCD 离线投影入口。
- 几何地面分割与可通行性栅格。
- OctoMap baseline 与地面分割结果对比。
