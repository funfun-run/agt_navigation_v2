# Bunker Qt5 FAST-LIO 基础导航闭环

本流程只覆盖 MID360、FAST-LIVO2、二维建图、Qt5、PGM/YAML/PCD 保存、NDT/ICP、
Nav2、安全层和 Bunker 底盘。语义地图、Keepout 和覆盖规划保留但默认关闭，不属于本次
实机验收。

## 系统数据链

建图链：

```text
MID360 -> FAST-LIVO2 -> /agt/mapping/odometry
                    -> /agt/mapping/registered_points_lidar
                    -> OctoMap -> /agt/map/mapping_occupancy -> Qt5
Qt5 /agt/cmd_vel_manual -> agt_safety -> chassis guard
                         -> /agt/chassis/cmd_vel -> Bunker driver
```

导航链：

```text
PGM/YAML -> map_server -> /agt/map/global_occupancy
同源 PCD + 实时点云 -> NDT/ICP -> map -> odom
FAST-LIVO2 -> odom -> base_footprint + /agt/mapping/odometry
Qt5 /goal_pose -> goal_pose_bridge -> NavigateToPose
Nav2 -> /agt/navigation/cmd_vel_raw -> Collision Monitor
     -> /agt/navigation/cmd_vel -> agt_safety -> chassis guard
     -> /agt/chassis/cmd_vel -> Bunker driver
```

FAST-LIVO2 adapter 是 `odom -> base_footprint` 的唯一发布者，NDT/ICP 是
`map -> odom` 的唯一发布者。Bunker driver 的 odom TF 和 FAST-LIVO2 上游 TF 默认关闭。

## 准备

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
tools/build_ros_qt5_gui_app.sh
```

设备名、地图名和路径必须通过 launch 参数提供；不要写进源码或配置模板。实车运动前确认
MID360 网络、CAN 接口、雷达外参、车辆 footprint、履带方向、硬件急停和制动距离均已验收。

## 建图与 Qt 手动控制

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=mapping map_name:=greenhouse_01 \
  start_gui:=true start_rviz:=true
```

Qt5 mapping profile 使用 `odom` 固定坐标系，显示 `/agt/map/mapping_occupancy` 和
`/agt/mapping/odometry`。手动速度发布到 `/agt/cmd_vel_manual`，不能直接发往底盘。

安全层启动后仍默认禁止运动。确认急停释放且车辆具备安全测试条件后，显式使能：

```bash
ros2 service call /agt/safety/reset_emergency_stop std_srvs/srv/Trigger '{}'
ros2 service call /agt/safety/set_motion_enabled std_srvs/srv/SetBool '{data: true}'
```

先架空履带或在隔离区域以低速确认前进、后退和转向符号。手动输入具有短超时；Qt 停止发送
后安全层应归零。

## 保存 PGM/YAML 和 PCD

地图稳定后保持建图进程运行，在另一个已 source 的终端执行：

```bash
tools/save_mapping_outputs.sh greenhouse_01
```

输出为：

```text
runtime/maps/greenhouse_01/greenhouse_01.pgm
runtime/maps/greenhouse_01/greenhouse_01.yaml
runtime/maps/greenhouse_01/pcd/                 # FAST-LIVO2 PCD 输出目录
```

保存脚本检查 `/agt/map/mapping_occupancy` 的存在和类型，以 transient-local 方式保存栅格，
不会终止 FAST-LIVO2。保存 PGM/YAML 后使用正常 `Ctrl+C` 结束建图，让 FAST-LIVO2 完成 PCD
落盘；确认 PCD 文件存在且非空。不要使用 `kill -9` 作为正常停止方式。

## 使用同源地图启动导航

将 `global_map_pcd` 指向本次建图正常结束后生成的对应 PCD：

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  map:="$(pwd)/runtime/maps/greenhouse_01/greenhouse_01.yaml" \
  global_map_pcd:="$(pwd)/runtime/maps/greenhouse_01/pcd/<generated-map>.pcd" \
  backend:=ndt start_gui:=true \
  start_semantic_map_server:=false \
  start_coverage_planning:=false
```

也可在已完成回放验收后使用 `backend:=icp`。不要修改已经验证的 NDT 参数来掩盖地图、
外参或初值错误。

Qt5 navigation profile 使用 `map` 固定坐标系。先用“设置初始位姿”发布 `/initialpose`，
等待 NDT/ICP 收敛并确认 `map -> odom -> base_footprint` 连续，再用单点目标工具发布
`/goal_pose`。`goal_pose_bridge.py` 会把目标转换成 Nav2 `NavigateToPose` action。

确认定位、costmap、Collision Monitor 和安全状态正常后，再显式使能运动：

```bash
ros2 service call /agt/safety/set_motion_enabled std_srvs/srv/SetBool '{data: true}'
```

## 必检 topic 与 TF

```bash
ros2 topic info /agt/map/mapping_occupancy -v
ros2 topic info /agt/map/global_occupancy -v
ros2 topic info /agt/mapping/odometry -v
ros2 topic info /agt/navigation/cmd_vel_raw -v
ros2 topic info /agt/navigation/cmd_vel -v
ros2 topic info /agt/cmd_vel_manual -v
ros2 topic info /agt/safety/cmd_vel -v
ros2 topic info /agt/chassis/cmd_vel -v
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

期望 `/agt/map/global_occupancy` 只有 map server 的静态地图；Nav2 原始速度只进入
Collision Monitor，手动和自动速度都由 `agt_safety` 仲裁，Bunker driver 只接收
`/agt/chassis/cmd_vel`。TF 检查应各自只有一个动态发布者。

## 急停与正常停止

需要停车时先禁用运动：

```bash
ros2 service call /agt/safety/set_motion_enabled std_srvs/srv/SetBool '{data: false}'
```

触发硬件或软件急停后，必须先排除原因、释放物理急停，再调用 reset；reset 不会自动重新
使能运动。系统正常退出使用启动终端中的 `Ctrl+C`，并确认安全层与底盘 watchdog 将命令归零。

## 当前限制

- 完整 bag 的地图质量、NDT/ICP 收敛率和真实温室导航尚需实机验收。
- 雷达外参、OctoMap 高度阈值、footprint、速度上限和制动距离必须使用已验证值；本流程不调参。
- 语义地图、Keepout、Fields2Cover 和覆盖规划属于后续阶段，当前启动必须保持默认关闭。
