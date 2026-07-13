# agt_mapping

隔离 FAST-LIVO2 等后端并输出统一接口：

- `/agt/mapping/odometry`：`odom` 下的 `base_footprint` 里程计。
- `/agt/mapping/registered_points`：注册点云。
- `odom -> base_footprint`：由当前连续里程计唯一发布。

算法基线固定为 `Aldoubt/FASTLIVO2_ROS2@a713004`，MID360 使用 Livox
`CustomMsg` 输入。该版本无条件发布 `camera_init -> aft_mapped`。使用前必须应用
`patches/fast_livo2_publish_tf.patch`；启动文件会设置 `common.publish_tf=false`，adapter
结合 `agt_description` 外参转换并发布标准 TF。未应用补丁时禁止同时启动机器人描述。

```bash
ros2 launch agt_description description.launch.py
ros2 launch agt_mapping fast_livo2_mapping.launch.py
```

当前完成接口隔离和位姿换算；同 bag 输出对比与实机验收仍待执行。
