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
