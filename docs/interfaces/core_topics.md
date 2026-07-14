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
- `/agt/map/mapping_occupancy`: OctoMap 建图过程中的实时二维工作图
- `/agt/map/global_occupancy`: 导航模式下由 Nav2 map server 发布的已保存静态地图
- `/agt/map/semantic_markers`: 语义服务器发布的 transient-local 标注可视化
- `/agt/map/keepout_mask`: 语义服务器发布、与基础地图严格对齐的 transient-local 语义 mask
- `/agt/map/keepout_filter_info`: Nav2 Costmap Filter Info Server 发布的 transient-local keepout 元数据
- `/agt/map/semantic_status`: 语义加载、校验和产品构建诊断
- `/agt/coverage/path_raw`: Fields2Cover 原始覆盖路径，TASK-10 前禁止直接执行
- `/agt/coverage/path_components`: swath 与连接段组件
- `/agt/coverage/swaths`: 覆盖 swath MarkerArray
- `/agt/coverage/headland`: field 与 planning field MarkerArray
- `/agt/coverage/status`: 覆盖请求、规划结果和稳定错误码诊断

## 语义地图服务
- `/agt/map/semantic/load`: `nav2_msgs/srv/LoadMap`，输入 GeoJSON 路径或 `file://` URL
- `/agt/map/semantic/reload`: `std_srvs/srv/Trigger`
- `/agt/map/semantic/validate`: `std_srvs/srv/Trigger`

完整 QoS、状态和事务规则见 [`semantic_map_server.md`](semantic_map_server.md)。

## 覆盖规划服务
- `/agt/coverage/plan`: `std_srvs/srv/Trigger`，按当前语义任务发起一次异步规划
- polygon action：`/agt/coverage/polygon/compute_coverage_path`
- annotated rows action：`/agt/coverage/rows/compute_coverage_path`

转换、GML、状态和安全边界见 [`coverage_planning.md`](coverage_planning.md)。

## Nav2 语义过滤
- global costmap 顺序：`StaticLayer -> KeepoutFilter -> InflationLayer`
- `/global_costmap/keepout_filter/toggle_filter`: `std_srvs/srv/SetBool`，运行时启停语义层
- FilterInfo/mask 缺失时 Humble 插件 fail-open 并告警；实车运动前必须确认语义状态 `LOADED`
- 语义成本只存在于 costmap 过滤层，不写回 `/agt/map/global_occupancy` 或基础 PGM

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
