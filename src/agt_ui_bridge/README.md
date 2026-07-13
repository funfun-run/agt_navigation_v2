# agt_ui_bridge

主界面采用 [`chengyangkj/Ros_Qt5_Gui_App`](https://github.com/chengyangkj/Ros_Qt5_Gui_App)，
固定源码位于 `third_party/ros_qt5_gui_app`。上游 GUI 负责地图与代价地图显示、机器人位姿、
速度控制、重定位、单点/多点导航、路径显示、栅格和拓扑地图编辑；本包只负责 V2 接口配置、
启动和地图 I/O，不在上游源码中硬编码项目 topic。

## 构建主界面

首次构建前安装上游依赖：

```bash
sudo apt-get install -y \
  qtbase5-dev qtbase5-private-dev libqt5svg5-dev \
  libsdl2-dev libsdl2-image-dev libeigen3-dev libgtest-dev
```

然后在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
./tools/build_ros_qt5_gui_app.sh
```

构建过程会按上游 CMake 配置下载 Advanced Docking System、yaml-cpp、nlohmann/json 和
topology_msgs，产物写入 `build/ros_qt5_gui_app`。源码固定在上游提交 `b0825e3`，构建产物
不会提交 Git。

当前机器旧工作区已有可运行版本；若新版本尚未构建，启动脚本会临时回退到
`/home/yangxuan/ros2_ws/src/Ros_Qt5_Gui_App/build`。

## 启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agt_ui_bridge ros_qt5_gui.launch.py
```

首次启动会将 V2 默认配置复制到
`runtime/gui/ros_qt5_gui_app/config.json`。GUI 中修改的界面和 topic 配置只保存在 runtime，
不会污染 vendor 源码。需要恢复仓库默认配置时执行：

```bash
ros2 run agt_ui_bridge start_ros_qt5_gui_app.sh --reset-config
```

## V2 默认映射

- 全局地图：`/agt/map/global_occupancy`
- 机器人里程计：`/agt/mapping/odometry`
- 重定位初值：`/initialpose`
- 导航目标：`/goal_pose`
- 手动速度：`/agt/cmd_vel_manual`
- 全局/局部路径：`/plan`、`/local_plan`
- 全局/局部代价地图：`/global_costmap/costmap`、`/local_costmap/costmap`
- 机器人 frame：`base_link`
- GUI 显示 frame：`odom`（建图阶段）
- 拓扑地图：`/agt/map/topology`、`/agt/map/topology/update`

目前 `/goal_pose` 仍是统一输出接口，导航模块完成后接入 Nav2 action；手动速度已经接入
`agt_safety -> agt_chassis`，但必须先显式使能安全层。GUI 可直接打开、编辑并保存 PGM/YAML；保存后的导航地图放在
`runtime/maps/`，下一次启动 map server 时使用对应 YAML。

默认 `FixedFrameId=odom` 与 FAST-LIVO2 建图链一致，避免尚未启动重定位时缺少
`map -> odom` 而持续产生 TF 警告。进入重定位/导航阶段并确认该 TF 已发布后，将
`runtime/gui/ros_qt5_gui_app/config.json` 中的 `FixedFrameId` 改为 `map`。

## 备用工具

轻量 Python 编辑器仍保留用于无上游 GUI 或地图 I/O 调试：

```bash
ros2 launch agt_ui_bridge map_editor.launch.py
ros2 run agt_ui_bridge map_io_bridge.py
```

它通过标准 `nav2_msgs/srv/LoadMap`、`SaveMap` 服务加载和保存地图，并将编辑结果发布到
`/agt/map/edited`。主界面优先使用 `ros_qt5_gui.launch.py`。
