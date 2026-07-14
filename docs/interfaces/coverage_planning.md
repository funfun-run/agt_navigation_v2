# 覆盖规划请求适配接口

TASK-09 由 `agt_coverage_planning` 将通过完整校验的 GeoJSON、`coverage.yaml` 和 canonical
platform profile 转换为 OpenNav Humble `ComputeCoveragePath` 请求。适配器不发布 TF，不修改
语义文件或基础地图，也不执行返回路径。

## 请求流程

1. 读取语义任务并重新执行 schema、SHA256、Shapely、地图范围及 footprint 校验。
2. 校验 `robot_profile`、`robot_width` 和 `min_turning_radius` 快照与所选 profile 一致。
3. 将车辆/机具参数原子写入目标 Coverage Server；拒绝更新时不发送 action goal。
4. polygon 模式直接发送外环与 exclusion 孔洞；annotated rows 模式生成进程私有临时 GML。
5. 校验结果为 `map` frame、非空路径和有效 orientation 后，原子发布本次产品。

Humble Row Coverage Server 只接受 GML 文件。临时文件位于系统临时目录，并随适配器退出删除；
它不是运行时地图产物。第一版每次请求只支持一个 enabled field，row 模式至少需要两条 enabled
`row_centerline`，并使用 `ROWSARESWATHS`。

## ROS 接口

所有输出采用 `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)`：

| 名称 | 类型 | 含义 |
| --- | --- | --- |
| `/agt/coverage/plan` | `std_srvs/srv/Trigger` | 使用参数指定的语义任务发起一次异步规划 |
| `/agt/coverage/path_raw` | `nav_msgs/msg/Path` | 未经过 TASK-10 安全验证的原始覆盖路径 |
| `/agt/coverage/path_components` | `opennav_coverage_msgs/msg/PathComponents` | swath、连接段、方向和速度组件 |
| `/agt/coverage/swaths` | `visualization_msgs/msg/MarkerArray` | action 结果 swath 可视化 |
| `/agt/coverage/headland` | `visualization_msgs/msg/MarkerArray` | 算法 field/planning field 或语义回退可视化 |
| `/agt/coverage/status` | `diagnostic_msgs/msg/DiagnosticArray` | 状态、稳定错误码、对象 ID、模式和耗时 |

两个 lifecycle server 分别位于 `/agt/coverage/polygon` 和 `/agt/coverage/rows` namespace，避免
同名 `compute_coverage_path` action 冲突。每个 server 使用同 namespace 的 lifecycle manager，
确保 bond 与自动激活正常。

## 状态与失败

正常状态依次为 `IDLE -> PLANNING -> SUCCEEDED`。服务器或参数服务未就绪时为
`WAITING_FOR_SERVER`；输入在 action 前被拒绝为 `REJECTED`；服务器或结果校验失败为 `FAILED`。
诊断至少包含 `detail`、`error_code`、`object_id`、`planning_mode` 和 `frame_id`。

稳定输入错误包括 `robot_width_profile_mismatch`、`min_turning_radius_profile_mismatch`、
`unsupported_field_count`、`insufficient_annotated_rows` 及 TASK-04 的原始验证 code。上游 Humble
将 `INVALID_REQUEST` 和 `INVALID_COORDS` 都定义为数值 `803`，因此运行时统一报告
`invalid_request_or_coordinates`，不能伪造更细分类别。

## 安全边界

`/agt/coverage/path_raw` 只证明 Fields2Cover 能生成几何路径。TASK-10 尚未执行 costmap、完整
footprint、插值碰撞和曲率检查，因此禁止直接把该 topic 接到实车控制器。Humble 第一版不依赖
Coverage Navigator、BT plugins 或 demo。
