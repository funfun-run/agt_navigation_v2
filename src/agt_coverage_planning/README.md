# agt_coverage_planning

职责：将已验证的农业语义任务适配为 OpenNav/Fields2Cover 覆盖规划请求。

TASK-09 已实现 polygon 与 annotated rows 两种模式。核心转换层无 ROS 依赖；ROS 节点在发送
`ComputeCoveragePath` goal 前，将 canonical platform profile 的 `robot_width`、
`min_turning_radius` 和任务 `operation_width` 原子同步到选中的 Coverage Server。

## 转换规则

| 输入 | OpenNav Humble 输出 |
| --- | --- |
| 唯一 enabled `field_boundary` | polygon `polygons[0]`；row GML 的外环 |
| enabled `exclusion_zone` | `polygons[1..N]`；row GML 的内环 |
| enabled `work_direction` | `SwathMode.SET_ANGLE`，角度归一到 `[0, pi)` |
| `planning_mode: annotated_rows` | 临时 GML `Row`，`ROWSARESWATHS` |
| `allow_reverse` | `REEDS_SHEPP`；否则 `DUBIN` |
| `headland_width` | polygon 模式 `CONSTANT` headland |

Humble 的 Row Coverage Server 只接受 GML 文件。适配器在进程私有临时目录生成 GML，退出时
删除，不改写 GeoJSON、`coverage.yaml` 或基础地图。第一版每次请求只支持一个 field；多 field
会明确返回 `unsupported_field_count`，不会静默丢弃。

## 启动与规划

先 source TASK-08 的外部覆盖工作区，再 source 本仓库：

```bash
source /opt/ros/humble/setup.bash
source /path/to/agt_coverage_ws/install/setup.bash
source install/setup.bash

ros2 launch agt_coverage_planning coverage_planning.launch.py \
  semantic_map:=runtime/maps/greenhouse_01/semantic/semantic_map.geojson \
  platform_profile:=profiles/platforms/bunker.yaml
```

服务器 lifecycle active 后触发一次规划：

```bash
ros2 service call /agt/coverage/plan std_srvs/srv/Trigger "{}"
ros2 topic echo /agt/coverage/status --once
```

`plan_on_start:=true` 可在服务器就绪后自动规划。第一版只生成原始覆盖结果，不执行车辆运动。

## ROS 接口

| 接口 | 类型 |
| --- | --- |
| `/agt/coverage/plan` | `std_srvs/srv/Trigger` |
| `/agt/coverage/path_raw` | `nav_msgs/msg/Path` |
| `/agt/coverage/path_components` | `opennav_coverage_msgs/msg/PathComponents` |
| `/agt/coverage/swaths` | `visualization_msgs/msg/MarkerArray` |
| `/agt/coverage/headland` | `visualization_msgs/msg/MarkerArray` |
| `/agt/coverage/status` | `diagnostic_msgs/msg/DiagnosticArray` |

输出均为 `map` frame。适配器会拒绝空路径、错误 frame 和无效 quaternion，并在 status 的
`error_code`、`object_id`、`detail` 中返回明确原因。TASK-10 才负责 costmap footprint 碰撞、
插值和曲率验证；`path_raw` 当前不能直接作为实车安全路径。
