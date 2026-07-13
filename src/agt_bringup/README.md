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
