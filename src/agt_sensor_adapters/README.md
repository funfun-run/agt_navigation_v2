# agt_sensor_adapters

将传感器原生输出转换到统一 AGT 接口。当前已迁入旧仓库验证过的
`livox_ros_driver2`，并提供 MID360 启动入口：

```bash
ros2 launch agt_sensor_adapters mid360.launch.py
```

统一输出：

- `/agt/sensors/lidar/custom`：`livox_ros_driver2/msg/CustomMsg`，保留每点
  `offset_time/line/tag`，供 FAST-LIVO2 使用。
- `/agt/sensors/imu/data`：MID360 内置 IMU，frame 为 `livox_frame`。

这里有意使用 `xfer_format=1` 的 Livox 原生消息。选定的
`Aldoubt/FASTLIVO2_ROS2@a713004` 只有 Livox `CustomMsg` 路径会使用每点时间、线号和
回波标签；它没有 MID360 PointCloud2 handler。PointCloud2 仍作为 V2 注册点云等通用
输出格式，不作为该后端的原始输入。

网络配置填写在 `config/mid360_network.json`。当前沿用旧仓库实机配置：主机
`192.168.1.5`，MID360 `192.168.1.12`。更换网卡或雷达后只修改该文件，不修改
第三方驱动目录。设备网络配置中的 extrinsic 保持全零；机器人安装外参只填写在
`agt_description/config/mk_mini_mid360.yaml`，避免两处重复补偿。

离线只能验证构建、launch 和配置格式。实机后需检查 topic、QoS、点云/IMU 频率、
时间戳、丢包和 frame，再将结果记录到根 README 的模块验收清单。
