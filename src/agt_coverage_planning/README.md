# agt_coverage_planning

职责：将已验证的农业语义任务适配为 OpenNav/Fields2Cover 覆盖规划请求，并验证规划路径。

TASK-09 已实现 polygon 与 annotated rows 两种模式。核心转换层无 ROS 依赖；ROS 节点在发送
`ComputeCoveragePath` goal 前，将 canonical platform profile 的 `robot_width`、
`min_turning_radius` 和任务 `operation_width` 原子同步到选中的 Coverage Server。
TASK-10 已新增独立 Coverage Path Validator，使用全局 costmap、完整 profile footprint、距离/角度
插值和曲率约束生成可复现验证报告。
TASK-11 以 OpenNav `PathComponents` 为语义真源，重建扁平路径并为每个原始区间标记 `SWATH` 或
`CONNECTION`；稳定 swath ID 可被规划和验证报告引用。
TASK-12 只将 Validator 标记无效的 CONNECTION 交给 Nav2 planner，候选与最终路径复用完整
footprint 验证，并证明全部 SWATH Pose 数值不变。
TASK-14 提供 `/agt/coverage/execute`，按语义加载、规划、验证、可选修复和 Nav2 `FollowPath`
执行顺序提供阶段反馈与取消传播；节点不发布速度，也不自动使能安全链。

## 转换规则

| 输入 | OpenNav Humble 输出 |
| --- | --- |
| 唯一 enabled `field_boundary` | polygon `polygons[0]`；row GML 的外环 |
| enabled `exclusion_zone` | `polygons[1..N]`；row GML 的内环 |
| enabled `work_direction` | `SwathMode.SET_ANGLE`，角度归一到 `[0, pi)` |
| `planning_mode: annotated_rows` | 临时 GML `Row`，`ROWSARESWATHS` |
| `allow_reverse` | `REEDS_SHEPP`；否则 `DUBIN` |
| `headland_width` | polygon 模式 `CONSTANT` headland |

`work_direction` 使用一条有方向的 LineString 表达角度：第一点指向第二点的方向决定平行作业行
的排列角度，线段长度不代表路线长度，也不会要求机器人沿这条线行驶。编辑器中的顶点手柄用于
修改几何控制点：作业区/障碍手柄调整边界，作物行手柄调整中心线，方向线两端手柄调整角度，
入口位姿手柄调整位置和朝向。拖动后仍须通过包含关系、footprint 和边界合法性检查。

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
  platform_profile:=profiles/platforms/bunker.yaml \
  execution_enabled:=false
```

服务器 lifecycle active 后触发一次规划：

```bash
ros2 service call /agt/coverage/plan std_srvs/srv/Trigger "{}"
ros2 topic echo /agt/coverage/status --once

# Validator 报告 invalid CONNECTION 后显式触发修复
ros2 service call /agt/coverage/repair std_srvs/srv/Trigger "{}"
ros2 topic echo /agt/coverage/repair_report --once
```

`plan_on_start:=true` 可在服务器就绪后自动规划。`execution_enabled` 默认 `false`，用于离线检查
完整任务链但禁止 Nav2 运动。实车执行必须显式改为 `true`，并由操作者在检查语义状态、定位、
障碍链和急停后单独使能 `agt_safety`；Action Server 不会解除安全门禁。

## 离线路线预览

该入口只启动基础地图、语义服务器、Coverage Server 和 RViz，不启动定位、Nav2 controller、
安全链或底盘。以当前 `mid360_map` 为例：

```bash
cd ~/agt_navigation_v2
source /opt/ros/humble/setup.bash
COVERAGE_WS=${COVERAGE_WS:-$HOME/agt_coverage_ws}
source "$COVERAGE_WS/install/setup.bash"
source install/setup.bash

ros2 launch agt_coverage_planning coverage_preview.launch.py \
  map:="$(realpath runtime/maps/mid360_map/mid360_map.yaml)" \
  semantic_map:="$(realpath runtime/maps/mid360_map/semantic/semantic_map.geojson)" \
  platform_profile:="$(realpath profiles/platforms/bunker.yaml)"
```

RViz 中红色为 `/agt/coverage/path_preview`，青色为通过 SWATH/CONNECTION 语义重建的路线，
黄色 Marker 为作业行，半透明层为 keepout mask。`path_preview` 只用于检查 Fields2Cover 原始
效果；即使组件语义校验失败也可显示，但永远不得送入 Validator、Nav2 或底盘。该轻量入口不启动 global costmap，因此绿色
`path_validated` 为空属于预期；需要检查 footprint 碰撞时使用完整 Nav2 离线系统。

当前 `mid360_map` 已实测发布 `679` 个 `path_preview` 姿态。锁定版 OpenNav 的 PathComponents
同时包含零长度 SWATH，因此状态会报告 `zero_length_swath`，青色重建路径和绿色验证路径为空；
这是执行链主动拒绝不完整语义的预期行为，不影响先查看红色服务器路线。

若自动规划尚未返回，可在服务器 active 后手动触发并查看诊断：

```bash
ros2 service call /agt/coverage/plan std_srvs/srv/Trigger "{}"
ros2 topic echo /agt/coverage/status --once
ros2 topic echo /agt/coverage/path_raw --once
```

发送统一覆盖任务：

```bash
ros2 action send_goal --feedback /agt/coverage/execute \
  agt_interfaces/action/ExecuteCoverageTask \
  "{semantic_map_uri: 'runtime/maps/greenhouse_01/semantic/semantic_map.geojson', \
    field_id: 'field_01', planning_mode: 'polygon', \
    controller_id: 'FollowPath', allow_repair: true}"
```

`field_id` 和 `planning_mode` 必须与语义任务完全一致。执行阶段 `Ctrl+C` 只会停止本地 CLI；要
验证 Action 取消，请使用支持 cancel 的客户端。TASK-15 已将该动作接入 `agt_bringup` 总控。

## ROS 接口

| 接口 | 类型 |
| --- | --- |
| `/agt/coverage/plan` | `std_srvs/srv/Trigger` |
| `/agt/coverage/path_raw` | `nav_msgs/msg/Path` |
| `/agt/coverage/path_preview` | `nav_msgs/msg/Path`，仅显示 Coverage Server 原始返回 |
| `/agt/coverage/path_components` | `opennav_coverage_msgs/msg/PathComponents` |
| `/agt/coverage/path_reconstructed` | `nav_msgs/msg/Path` |
| `/agt/coverage/path_semantics` | `std_msgs/msg/String`，schema 1.0 JSON |
| `/agt/coverage/swaths` | `visualization_msgs/msg/MarkerArray` |
| `/agt/coverage/headland` | `visualization_msgs/msg/MarkerArray` |
| `/agt/coverage/status` | `diagnostic_msgs/msg/DiagnosticArray` |
| `/agt/coverage/path_validated` | `nav_msgs/msg/Path` |
| `/agt/coverage/collision_poses` | `geometry_msgs/msg/PoseArray` |
| `/agt/coverage/footprint_markers` | `visualization_msgs/msg/MarkerArray` |
| `/agt/coverage/validation_report` | `std_msgs/msg/String`，稳定键序 JSON |
| `/agt/coverage/repair` | `std_srvs/srv/Trigger` |
| `/agt/coverage/path_repaired` | `nav_msgs/msg/Path` |
| `/agt/coverage/repair_report` | `std_msgs/msg/String`，稳定键序 JSON |
| `/agt/coverage/execute` | `agt_interfaces/action/ExecuteCoverageTask` |
| `/agt/coverage/task_status` | `diagnostic_msgs/msg/DiagnosticArray` |

输出均为 `map` frame。适配器会拒绝空路径、错误 frame 和无效 quaternion，并在 status 的
`error_code`、`object_id`、`detail` 中返回明确原因。Validator 默认把未知和地图外空间视为碰撞，
占用阈值为 65；检查失败时 `path_validated` 为空，碰撞姿态和 footprint markers 可在 RViz2
查看。参数见 `config/coverage_planning.yaml`。

同一规划结果中的 `swath_id` 按 canonical 端点几何稳定编号，`order_index` 表示本次路线顺序。
默认重建长度容差为 `max(0.05 m, 0.5%)`，超限、组件未排序、区间未完全分类或 Path 指纹不匹配
都会拒绝结果。

修复要求 semantic status 为 `LOADED`，并直接检查 global costmap 和 keepout mask。BUNKER 当前
使用 `GridBased`；MK-mini 因运动学 profile 与 Ackermann 任务合同冲突而明确禁用，标定和配置
完成前启动会失败，不会复用 BUNKER 参数。

`path_raw` 永远不能直接执行。TASK-10~12 通过也只代表静态安全、语义一致性和连接修复通过；
TASK-14 还要求语义 `LOADED`、近期 `agt_safety` 可运动诊断和 Nav2 ready。最终路径只发送到标准
Nav2 `FollowPath`，`path_validated`、`path_repaired` 仍不得直连底盘。
