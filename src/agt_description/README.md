# agt_description

集中管理机器人几何、固定 TF 和传感器外参。本包只发布
`base_footprint -> base_link -> lidar_link -> livox_frame/imu_link`，不发布
`map -> odom` 或 `odom -> base_footprint`。

## 启动

```bash
ros2 launch agt_description description.launch.py
```

BUNKER 使用产品图中的 `1.023 x 0.778 x 0.400 m` 外廓尺寸：

```bash
ros2 launch agt_description bunker_description.launch.py
```

MID360 六自由度外参的唯一持久化填写位置是
`config/mk_mini_mid360.yaml`；BUNKER 对应 `config/bunker_mid360.yaml`。launch 会从配置读取
默认值，命令行参数仅用于临时覆盖：

```bash
ros2 launch agt_description description.launch.py \
  lidar_x:=0.12 lidar_z:=0.63 lidar_pitch:=-0.0872665
```

BUNKER 外廓尺寸来自已提供的产品图，但 `base_link` 基准高度和 MID360 外参仍未实测，配置中的
`calibration_verified: false` 不应在实车验收前改为 `true`。不需要同步修改 Xacro、launch
或 sensor profile。

## Frame 兼容策略

- `lidar_link` 是 V2 的传感器无关坐标系。
- `livox_frame` 保留旧 MID360/FAST-LIVO2 链的 frame 名称，与 `lidar_link` 重合。
- `imu_link` 表示 MID360 内置 IMU，目前与雷达原点重合。
- Phase 3 接入 FAST-LIVO2 时必须关闭或拦截其原生 `odom -> livox_frame` TF，转换为
  V2 约定的 `odom -> base_footprint`，否则会产生多父节点。

## 验收

```bash
colcon test --packages-select agt_description
colcon test-result --verbose
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_footprint livox_frame
```
