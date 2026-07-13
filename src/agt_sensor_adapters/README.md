# agt_sensor_adapters

将传感器原生输出转换到统一 AGT 接口。当前已迁入旧仓库验证过的
`livox_ros_driver2`，并提供 MID360 启动入口：

```bash
ros2 launch agt_sensor_adapters mid360.launch.py
```

统一输出：

- `/agt/sensors/lidar/points`：Livox PointCloud2，frame 为 `livox_frame`。
- `/agt/sensors/imu/data`：MID360 内置 IMU，frame 为 `livox_frame`。

网络配置填写在 `config/mid360_network.json`。当前沿用旧仓库实机配置：主机
`192.168.1.5`，MID360 `192.168.1.12`。更换网卡或雷达后只修改该文件，不修改
第三方驱动目录。设备网络配置中的 extrinsic 保持全零；机器人安装外参只填写在
`agt_description/config/mk_mini_mid360.yaml`，避免两处重复补偿。

离线只能验证构建、launch 和配置格式。实机后需检查 topic、QoS、点云/IMU 频率、
时间戳、丢包和 frame，再将结果记录到根 README 的模块验收清单。
