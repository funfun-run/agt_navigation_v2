# agt_bringup

职责：组合启动、生命周期编排和系统级健康检查入口。

统一入口：

```bash
ros2 launch agt_bringup system.launch.py mode:=mapping map_name:=greenhouse_01
ros2 launch agt_bringup system.launch.py mode:=navigation \
  map:=/absolute/path/map.yaml global_map_pcd:=/absolute/path/map.pcd
```

`mapping` 启动 BUNKER TF、传感器、FAST-LIVO2 PCD 保存、二维投影、底盘和 RViz；
`navigation` 关闭 PCD 保存，启动 LIO 里程计、重定位、Nav2、安全层、底盘和 Qt5。
两个模式均可设置 `record_bag:=true`，输出到 `runtime/rosbag/`。

原导航默认保持语义与覆盖模块关闭。启动完整覆盖作业链前，先 source TASK-08 的外部依赖工作区：

```bash
source /opt/ros/humble/setup.bash
source /path/to/agt_coverage_ws/install/setup.bash
source install/setup.bash

ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  map:=/absolute/path/greenhouse_01.yaml \
  global_map_pcd:=/absolute/path/all_downsampled_points.pcd \
  semantic_map:=/absolute/path/semantic_map.geojson \
  coverage_params:=/absolute/path/coverage.yaml \
  start_semantic_map_server:=true \
  start_coverage_planning:=true
```

`coverage_params` 必须是 `semantic_map` 同目录的 `coverage.yaml`，因为语义服务器和请求适配器
以二者作为一个原子任务加载。顶层会在启动任何子系统前检查地图、PCD、GeoJSON、coverage 和
platform profile；覆盖规划不能脱离语义服务器启动。

进程按 Nav2、语义服务器、覆盖规划的所有者顺序加入同一 launch。运行时 readiness 由
`map_server active -> localization ready -> semantic LOADED -> keepout mask -> global costmap ->
coverage planner` 链共同决定；进程存在不等于可执行。只有下列检查通过后才允许手动使能安全层：

```bash
ros2 lifecycle get /map_server
ros2 topic echo /agt/localization/status --once
ros2 topic echo /agt/map/semantic_status --once
ros2 topic echo /agt/map/keepout_mask --once --field info
ros2 lifecycle get /planner_server
ros2 action info /agt/coverage/execute
```

`start_coverage_planning:=true` 且 `annotation_mode:=false` 时总控会允许 TASK-14 进入执行门禁，
但仍不会自动调用 `/agt/safety/set_motion_enabled`；上述 readiness、定位和现场检查必须先完成。

标注模式使用项目语义编辑器替代普通 Qt5 操作界面，并强制覆盖 Action 的执行开关为 false：

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=navigation map:=/absolute/path/greenhouse_01.yaml \
  global_map_pcd:=/absolute/path/all_downsampled_points.pcd \
  semantic_map:=/absolute/path/semantic_map.geojson \
  coverage_params:=/absolute/path/coverage.yaml \
  start_semantic_map_server:=true annotation_mode:=true
```

保存标注后调用 `/agt/map/semantic/reload` 或重新启动作业模式。正常 `Ctrl+C` 会关闭同一进程树
中的覆盖与 Nav2 Action Server；安全层和底盘 watchdog 会将残余速度归零。禁止使用 `kill -9`。

建图工作图使用 `/agt/map/mapping_occupancy`，导航 `map_server` 使用
`/agt/map/global_occupancy`，两者不会互相显示旧的 transient-local 地图。

二维地图必须在建图仍运行时保存：

```bash
ros2 launch agt_bringup save_mapping_result.launch.py map_name:=greenhouse_01
```

确认二维地图保存成功后，再对建图总控使用 `Ctrl+C`；PCD 将保存到
`runtime/maps/<map_name>/pcd/`，rosbag 也会完成元数据写入。不要先关闭总控再保存二维地图。

无雷达或 CAN 时可分别使用 `start_sensor:=false`、`start_chassis:=false`。建图无显示器时
使用 `start_rviz:=false`，导航无显示器时使用 `start_gui:=false`。
运行总控后禁止再单独启动 description/chassis launch；真实运动前仍需显式使能安全层。
