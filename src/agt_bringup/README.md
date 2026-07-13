# agt_bringup

职责：组合启动、生命周期编排和系统级健康检查入口。

统一入口：

```bash
ros2 launch agt_bringup system.launch.py mode:=mapping map_name:=greenhouse_01
ros2 launch agt_bringup system.launch.py mode:=navigation \
  map:=/absolute/path/map.yaml global_map_pcd:=/absolute/path/map.pcd
```

`mapping` 启动 BUNKER TF、传感器、FAST-LIVO2 PCD 保存、二维投影、底盘和 Qt5；
`navigation` 关闭 PCD 保存，启动 LIO 里程计、重定位、Nav2、安全层、底盘和 Qt5。
两个模式均可设置 `record_bag:=true`，输出到 `runtime/rosbag/`。

二维地图必须在建图仍运行时保存：

```bash
ros2 launch agt_bringup save_mapping_result.launch.py map_name:=greenhouse_01
```

PCD 在建图总控收到 `Ctrl+C` 后保存到 `runtime/maps/<map_name>/pcd/`。无雷达、CAN 或
显示器时可分别使用 `start_sensor:=false`、`start_chassis:=false`、`start_gui:=false`。
运行总控后禁止再单独启动 description/chassis launch；真实运动前仍需显式使能安全层。
