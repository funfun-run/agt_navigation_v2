# agt_localization

使用全局 PCD 与当前 `lidar_link` 点云执行 ICP/NDT 重定位，质量通过后唯一发布
`map -> odom`。

## 接口

- 输入点云：`/agt/mapping/registered_points_lidar` (`sensor_msgs/PointCloud2`)
- 初值：`/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`，语义为 `map -> base_link`)
- 状态：`/agt/localization/status` (`std_msgs/String`)
- 对齐点云：`/agt/localization/aligned_points`
- TF：成功后持续发布 `map -> odom`

节点会从 TF 查询 `base_link -> lidar_link` 来修正配准初值，因此应先启动
`agt_description` 并填写车辆到雷达外参。MID360 雷达到内置 IMU 的内部外参仍由
FAST-LIVO2 参数管理，不应重复填入这里。

## 启动

先启动机器人描述与连续里程计，再提供与二维导航地图同源的全局 PCD：

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
source install/setup.bash
ros2 launch agt_localization relocalization.launch.py \
  global_map_pcd:=/absolute/path/to/global_map.pcd backend:=ndt use_sim_time:=true
```

在 RViz2 中使用 `2D Pose Estimate` 发布初值。调试时可将 `backend:=icp`；参数阈值见
`config/relocalization.yaml`。目前只有 PGM/YAML 栅格图，不能替代重定位所需的三维 PCD。

## 待验证

- 用同一建图数据导出的 PCD 检查 NDT/ICP 收敛率、fitness 和恢复时间。
- 标定 `base_link -> lidar_link` 后验证非零外参下的 `map -> odom`。
- 检查系统中没有第二个 `map -> odom` 发布者，并对错误初值执行拒绝测试。
