# Third-party dependencies

## livox_ros_driver2

- 来源：旧工作区 `/home/yangxuan/ros2_ws/src/livox_ros_driver2`
- 旧仓库：`https://github.com/Aldoubt/ros2_3d-nav.git`
- 基线提交：`115c7beeaea02593957af46ccbecc263bc5cf12f`
- 上游许可证：MIT，见 `livox_ros_driver2/LICENSE.txt`
- 本地调整：旧仓库版本已支持相邻 Livox-SDK2 或 `/usr/local` 系统安装回退；为兼容
  当前系统 SDK，移除了本项目不使用的 MID360s 枚举分支，MID360 数据路径保持不变

第三方源码只做必要的构建兼容调整。设备 IP、topic remap 和 V2 frame 规则放在
`agt_sensor_adapters`，不写入 vendor 目录。

## FASTLIVO2_ROS2

- 来源：`https://github.com/Aldoubt/FASTLIVO2_ROS2.git`
- 固定提交：`a713004f0ba0624c8fb80d85c7047fe62523c6fb`
- 目录：`third_party/fast_livo2_ros2`
- 上游许可证：GPL-2.0，见 `fast_livo2_ros2/LICENSE`
- 本地调整：增加原生 TF 发布开关；使用 vikit 导出的 CMake target；增加
  `/cloud_registered_lidar` 当前帧雷达坐标点云，供 OctoMap 使用动态传感器原点；修正
  上游 `package.xml` 与实际 GPL-2.0 许可证不一致的声明

FAST-LIVO2 已 vendor 到主仓库，不再由 `nav_dependencies.repos` 下载，也不依赖 `/tmp`
安装空间。算法接口和 topic remap 仍由 `agt_mapping` 管理。

## relocalization_core / ndt_omp_ros2

- `relocalization_core` 来源：旧工作区 `relocalization_module/relocalization_core`，Apache-2.0，
  见 `relocalization_core/LICENSE`
- `ndt_omp_ros2` 来源：旧工作区同名包，基线提交
  `115c7beeaea02593957af46ccbecc263bc5cf12f`，BSD-2-Clause
- 目录：`third_party/relocalization_core`、`third_party/ndt_omp_ros2`
- 本地调整：移除与算法库无关的 NDT 示例程序和数据；移除 core 中多余的直接 libusb
  链接。ROS/TF 适配由 `agt_localization` 实现，不写入第三方核心。

## Ros_Qt5_Gui_App

- 来源：`https://github.com/chengyangkj/Ros_Qt5_Gui_App.git`
- 固定提交：`b0825e3cba3e7186cba8a6b83ff230be37c8b1fb`
- 目录：`third_party/ros_qt5_gui_app`
- 上游许可证：GPL-2.0，见 `ros_qt5_gui_app/LICENSE`
- 本地策略：V2 topic 和运行配置放在 `agt_ui_bridge`；vendor 源码仅保留 ROS2 集成所需的
  最小修复，包括可配置 `FixedFrameId`、通信线程安全退出和幂等 ROS shutdown。构建产物
  写入 `build/ros_qt5_gui_app`，不提交 Git。

## BUNKER ROS2

- 来源：`https://github.com/agilexrobotics/bunker_ros2.git`
- 分支：`humble`
- 固定提交：`c4737f249129e88c8e9e0bfeb3af81b498a0ebbe`
- 目录：`third_party/bunker_ros2`
- 上游许可证：Apache-2.0（仓库 LICENSE）；package 元数据标记 BSD
- 本地调整：增加 odom TF 开关和底层命令超时，项目默认关闭其 TF，避免与定位链冲突。

## UGV SDK

- 来源：`https://github.com/agilexrobotics/ugv_sdk.git`
- 分支：`main`（浅克隆后 vendor）
- 固定提交：`c3dfaf444f9bae10757e546acae055aaf4a13de7`
- 目录：`third_party/ugv_sdk`
- 上游许可证：BSD，见目录内 LICENSE
- 本地调整：将旧 catkin 元数据适配为 ament/colcon，默认不构建示例程序，并修复 BUNKER
  执行器状态数组从 3 项复制到 2 项目标数组时的越界写入；CAN 协议解析不改。未保留与
  编译无关、体积约 113 MB 的多车型 PDF/DOCX `docs/` 目录。
