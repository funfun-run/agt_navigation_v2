# agt_navigation_v2

`agt_navigation_v2` 是面向农业机器人导航实验的 ROS 2 模块化平台。

当前已完成 Phase 1 仓库骨架、Phase 2 机器人描述，以及 Phase 3 可离线完成的
FAST-LIVO2 接口适配。由于暂无可用 bag，Phase 3 的算法输出对比和实机验收暂缓。

## MID360 外参填写

外参唯一填写位置：
[`src/agt_description/config/mk_mini_mid360.yaml`](src/agt_description/config/mk_mini_mid360.yaml)。
不要直接修改 Xacro 或 launch 文件中的同名数值。

```yaml
lidar_x: 0.0       # 米，向前为正
lidar_y: 0.0       # 米，向左为正
lidar_z: 0.50      # 米，向上为正
lidar_roll: 0.0    # 弧度，绕 X 轴
lidar_pitch: 0.0   # 弧度，绕 Y 轴
lidar_yaw: 0.0     # 弧度，绕 Z 轴
calibration_verified: false
```

以上参数表示 `base_link -> lidar_link`。标定并实机验证后，将
`calibration_verified` 改为 `true`。临时试验可使用 launch 参数覆盖，但不会改写标定文件：

```bash
ros2 launch agt_description description.launch.py \
  lidar_x:=0.12 lidar_z:=0.63 lidar_pitch:=-0.0872665
```

## 命名规范

- package、topic、参数和文件名统一使用小写 `snake_case`。
- ROS package 统一以 `agt_` 开头；节点名使用 `agt_<模块>_<功能>`。
- 标准 frame 不带前导 `/`：`map`、`odom`、`base_footprint`、`base_link`、`lidar_link`、`imu_link`。
- `livox_frame` 仅作为旧驱动兼容 frame；V2 模块接口统一使用 `lidar_link`。
- V2 topic 放在 `/agt/<领域>/<名称>` 下，例如 `/agt/sensors/lidar/points`。
- launch 参数和 YAML key 使用相同名称；长度用米，角度用弧度。
- TF 发布责任固定：全局定位发布 `map -> odom`，连续里程计发布
  `odom -> base_footprint`，机器人描述发布 `base_footprint -> sensor`。

## 顶层目录
```text
agt_navigation_v2/
├── docs/
├── profiles/
├── runtime/
├── src/
├── tests/
├── third_party/
├── tools/
├── AGENTS.md
├── nav_dependencies.repos
└── README.md
```

## 核心功能包
- `agt_interfaces`
- `agt_description`
- `agt_bringup`
- `agt_sensor_adapters`
- `agt_mapping`
- `agt_map_processing`
- `agt_localization`
- `agt_localization_fusion`
- `agt_perception`
- `agt_navigation`
- `agt_coverage_planning`
- `agt_safety`
- `agt_chassis`
- `agt_ui_bridge`
- `agt_experiment_manager`
- `agt_evaluation`

## 构建与验证

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
python3 -m pytest -q tests src/agt_description/test src/agt_mapping/test
```

当前离线测试覆盖 package 命名、launch 语法、TF 拓扑、外参配置唯一性、FAST-LIVO2
位姿与速度外参换算，以及上游 TF 开关补丁契约。离线测试通过不代表算法精度或实车安全验收通过。

## 模块验收清单

| 模块 | 当前可离线完成/状态 | 后续需要补充的测试 |
| --- | --- | --- |
| `agt_interfaces` | package 骨架和命名检查已完成 | 定义接口后做消息生成、序列化和兼容性测试 |
| `agt_description` | Xacro 展开、URDF、TF 单父节点和 launch 参数检查已完成 | 实测底盘尺寸与 MID360 外参；实机检查倾斜、方向和 footprint |
| `agt_bringup` | 最小 launch 语法检查已完成 | 各阶段实现后做全系统启动、生命周期、异常退出和重启测试 |
| `agt_sensor_adapters` | 已迁入 Livox 驱动，MID360 配置、统一 topic remap 和 launch 离线检查已完成 | 需要 MID360 实机或 bag，验证点云/IMU topic、frame、QoS、频率、时间戳和丢包 |
| `agt_mapping` | adapter、统一 topic、位姿/twist 外参换算及 TF 补丁离线测试已完成 | 需要应用补丁后的 FAST-LIVO2、同一 bag 对比轨迹/点云；实机检查漂移和 TF 唯一发布源 |
| `agt_map_processing` | package 边界已建立 | 需要 PCD/bag，比较 OctoMap、地面分割、栅格完整性与处理耗时 |
| `agt_localization` | package 边界已建立 | 需要地图和带真值/参考轨迹的 bag，测试 ICP/NDT 成功率、误差、恢复时间和 `map -> odom` |
| `agt_localization_fusion` | package 边界已建立 | 需要 LIO、轮速、IMU，后续 RTK/UWB 数据；测试延迟、漂移、跳变和传感器失效降级 |
| `agt_perception` | package 边界已建立 | 需要标注或典型场景点云，测试地面/障碍精度、误检漏检和处理频率 |
| `agt_navigation` | package 边界已建立 | 需要地图、定位和底盘/仿真，测试规划成功率、跟踪误差、恢复行为和窄通道通过性 |
| `agt_coverage_planning` | package 边界已建立 | 需要地块边界和障碍数据，测试覆盖率、重复率、转弯次数及路径可执行性 |
| `agt_safety` | package 边界已建立 | 实现后先做合成消息超时/限速/急停测试，再做低速实车制动距离与通信中断测试 |
| `agt_chassis` | package 边界已建立 | 需要 MK-mini/BUNKER 或协议模拟器，测试命令映射、里程计、超时、断连和零速保护 |
| `agt_ui_bridge` | package 边界已建立 | 接口确定后做无 GUI 服务测试；后续测试 Qt5 重连、状态同步和错误提示 |
| `agt_experiment_manager` | profile 骨架已建立 | 实现后测试配置合并、Git/参数快照、产物命名、失败恢复和复现实验 |
| `agt_evaluation` | package 边界已建立 | 指标实现后用合成轨迹单测；有 bag/真值后生成定位、导航、资源占用报告 |

## 后续数据准备

优先准备一份可重复播放的 MID360 数据集，至少包含原始点云、内置 IMU、FAST-LIVO2
原生里程计和 `/tf`、`/tf_static`。建议再准备静止、直线、原地转向、温室窄通道四类片段，
并记录传感器安装尺寸、ROS 2 版本、提交版本和参数文件。bag 放入 `runtime/rosbag/`，该目录
默认不提交 Git；地图/PCD 放入 `runtime/maps/`。

Phase 3 有数据后的最低验收项：新旧注册点云数量和时间戳一致，转换后的
`/agt/mapping/odometry` 连续，`odom -> base_footprint` 只有一个发布源，轨迹相对旧链无
非预期跳变，并保存对比报告到 `runtime/results/`。

详细 TF 约束见 [`src/agt_description/README.md`](src/agt_description/README.md)，
迁移进度见 [`docs/migration/migration_matrix.md`](docs/migration/migration_matrix.md)。
