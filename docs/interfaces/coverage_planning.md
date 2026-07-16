# 覆盖规划与路径验证接口

TASK-09 由 `agt_coverage_planning` 将通过完整校验的 GeoJSON、`coverage.yaml` 和 canonical
platform profile 转换为 OpenNav Humble `ComputeCoveragePath` 请求。TASK-10 再使用 Nav2 全局
costmap 和同一 profile 的完整 footprint 对原始路径做离线安全验证。两者都不发布 TF、不修改
语义文件或基础地图，也不控制底盘。

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
| `/agt/coverage/path_preview` | `nav_msgs/msg/Path` | Coverage Server 基础结果，仅供 RViz 显示 |
| `/agt/coverage/path_components` | `opennav_coverage_msgs/msg/PathComponents` | swath、连接段、方向和速度组件 |
| `/agt/coverage/path_reconstructed` | `nav_msgs/msg/Path` | TASK-11 从有序组件重建的扁平 Path |
| `/agt/coverage/path_semantics` | `std_msgs/msg/String` | 每个原始 Path 区间的 SWATH/CONNECTION 稳定键序 JSON |
| `/agt/coverage/swaths` | `visualization_msgs/msg/MarkerArray` | action 结果 swath 可视化 |
| `/agt/coverage/headland` | `visualization_msgs/msg/MarkerArray` | 算法 field/planning field 或语义回退可视化 |
| `/agt/coverage/status` | `diagnostic_msgs/msg/DiagnosticArray` | 状态、稳定错误码、对象 ID、模式和耗时 |
| `/agt/coverage/path_validated` | `nav_msgs/msg/Path` | 全部 TASK-10 检查通过时的原始路径；失败时为空 Path |
| `/agt/coverage/collision_poses` | `geometry_msgs/msg/PoseArray` | 插值后与占用、未知或地图外区域碰撞的姿态 |
| `/agt/coverage/footprint_markers` | `visualization_msgs/msg/MarkerArray` | 无效采样点的完整 footprint 轮廓 |
| `/agt/coverage/validation_report` | `std_msgs/msg/String` | 稳定键序 JSON 验证报告 |
| `/agt/coverage/repair` | `std_srvs/srv/Trigger` | 显式启动一次无效连接段修复 |
| `/agt/coverage/path_repaired` | `nav_msgs/msg/Path` | 所有替换和最终复验成功后的完整路径 |
| `/agt/coverage/repair_report` | `std_msgs/msg/String` | 修复数量、耗时、ID、长度和最终验证 JSON |
| `/agt/coverage/execute` | `agt_interfaces/action/ExecuteCoverageTask` | 可取消的覆盖任务总动作，只通过 Nav2 执行最终路径 |
| `/agt/coverage/task_status` | `diagnostic_msgs/msg/DiagnosticArray` | 当前任务阶段、swath 进度和剩余距离 |
| `/agt/coverage/simulation_report` | `std_msgs/msg/String` | 离线路径运行时间估算，稳定键序 JSON |

两个 lifecycle server 分别位于 `/agt/coverage/polygon` 和 `/agt/coverage/rows` namespace，避免
同名 `compute_coverage_path` action 冲突。每个 server 使用同 namespace 的 lifecycle manager，
确保 bond 与自动激活正常。

## 路径组件语义

TASK-11 以锁定版 OpenNav `PathComponents.swaths[]` 与 `turns[]` 为权威输入，不根据航向或位置
重新猜测作业行。第一版只定义 `SWATH` 和 `CONNECTION`；上游未提供独立语义时，不把首尾 turn
擅自标记为 `APPROACH` 或 `EXIT`。

适配器按照组件端点连续性恢复交替顺序。`swath_NNNN` 由不受行驶方向影响的 canonical 端点
几何排序产生，因此路线反转或 swath 数组换序不会改变同一作业行编号；`order_index` 单独表示
本次执行顺序。连接段按执行顺序编号为 `connection_NNNN`。

`/agt/coverage/path_reconstructed` 对 swath 直线按默认 `0.10 m` 插值，并原样保留 turn Path 姿态；
组件交界区间归入相邻 CONNECTION，避免 TASK-12 修复连接段时改动作业行内部坐标。重建前后几何
长度误差不得超过 `max(0.05 m, 原始长度 * 0.005)`，阈值可显式配置，超限时整次规划结果拒绝
发布。每个原始 Path 区间必须且只能有一种类型。

`path_semantics` schema 1.0 至少包含原始 Path SHA256 指纹、长度误差、稳定 swath IDs、组件执行
顺序和覆盖全部原始区间的 `raw_segments`。Validator 必须验证指纹，拒绝旧语义与新 Path 混配，
并在报告中发布 `swath_ids`、`invalid_component_ids` 和 `invalid_swath_ids`。

## 连接段修复

TASK-12 的输入是 `path_reconstructed`、`path_semantics`、`validation_report`、global costmap、
published footprint、`/agt/map/keepout_mask` 和 semantic status。修复只能由
`/agt/coverage/repair` 显式触发，不自动执行车辆运动。

修复器先校验 validation report 的原始 Path 指纹和 semantics 的 reconstructed Path 指纹，再选择
同时满足“报告为 invalid”和“类型为 CONNECTION”的组件。任何 `invalid_swath_ids` 都立即返回
`swath_repair_forbidden`；有效 SWATH、语义 field/exclusion 及原有组件顺序不会修改。

每个目标依次向 Nav2 `/compute_path_to_pose` 发送原连接段起终点和 profile 指定的 planner ID。
候选路径必须通过 global costmap 与 keepout mask 的完整 footprint 检查；直接检查 mask 可避免运行时
KeepoutFilter 被关闭时越出允许区域。候选端点允许默认最多 `0.25 m` planner 偏差，但拼接前会强制
恢复为原始端点的精确坐标和姿态。

全部替换完成后，修复器证明每个 SWATH 原始 Pose 序列仍以完全相同的数值连续存在，再对完整路径
执行 TASK-10 同一验证内核。报告至少包含 `success`、`error_code`、`planner_id`、
`repaired_segment_count`、`repaired_component_ids`、`preserved_swath_ids`、`duration`、修复前后长度、
`swath_coordinates_unchanged` 和 `final_validation`。任一步失败都会清空旧 `path_repaired`，源 Path
保持不变。

BUNKER profile 当前启用 `GridBased`，履带差速允许原地旋转。MK-mini profile 仍声明
`differential`，而任务规范要求 Ackermann Hybrid-A*/State Lattice 且需要正最小转弯半径，因此
当前明确禁用 coverage repair；完成底盘运动学确认和参数标定前不得回退使用 BUNKER 配置。

## 状态与失败

正常状态依次为 `IDLE -> PLANNING -> SUCCEEDED`。服务器或参数服务未就绪时为
`WAITING_FOR_SERVER`；输入在 action 前被拒绝为 `REJECTED`；服务器或结果校验失败为 `FAILED`。
诊断至少包含 `detail`、`error_code`、`object_id`、`planning_mode` 和 `frame_id`。

稳定输入错误包括 `robot_width_profile_mismatch`、`min_turning_radius_profile_mismatch`、
`unsupported_field_count`、`insufficient_annotated_rows` 及 TASK-04 的原始验证 code。上游 Humble
将 `INVALID_REQUEST` 和 `INVALID_COORDS` 都定义为数值 `803`，因此运行时统一报告
`invalid_request_or_coordinates`，不能伪造更细分类别。

## Coverage Path Validator

Validator 同时等待 `/agt/coverage/path_raw`、`/global_costmap/costmap` 和
`/global_costmap/published_footprint`。路径、costmap 和 footprint frame 均须为 `map`。节点按
costmap resolution 与 profile footprint 最大半径确定距离步长，并限制角度步长，使稀疏直线段
和原地旋转的角点扫掠都能被检查；不得只检查原始路径点、中心点或四角。

每个插值姿态使用 Shapely 将完整 footprint 多边形与候选 costmap cell 相交。OccupancyGrid
代价保持 `-1/0..100` 语义，默认 `occupied_cost_threshold=65`；未知空间默认视为碰撞，也可显式
配置为 `free`。地图外默认视为碰撞。`minimum_clearance` 计算 footprint 到占用/未知栅格和地图
边界的最小距离；曲率由相邻姿态的 yaw 变化与平移距离计算，并与 profile 的
`min_turning_radius` 对照。

实际碰撞检查始终使用 `profiles/platforms/<platform>.yaml` 的 `navigation_footprint`。Nav2 发布的
footprint 只用于运行时形状一致性检查，比较对平移和旋转不敏感，默认容许 `0.03 m` 的显式
padding 差异；不在 coverage 配置中复制 footprint，也不叠加第二套隐式裕量。

报告至少包含 `valid`、`collision_pose_count`、`invalid_segment_indices`、`maximum_cost`、
`minimum_clearance`、`maximum_curvature` 和 `required_min_turning_radius`，并附带稳定错误码、采样
数量和未知空间策略。原始路径、costmap 或 footprint 更新后以默认 2 Hz 重新验证。任何输入错误
或检查失败都会发布空 `path_validated`，避免下游继续使用旧结果。

## 安全边界

`/agt/coverage/path_raw` 永远不是执行接口。`path_validated` 只表示 TASK-10 的当前静态 costmap、
footprint、曲率和 TASK-11 语义一致性检查通过；`path_repaired` 表示 TASK-12 也成功，但后续任务
Action 仍必须检查语义 `LOADED` 与 `agt_safety` readiness。两者都不得直接接到底盘控制器，MPPI
执行期检查也不能替代本验证器。

TASK-14 的 `/agt/coverage/execute` 按 `LOADING -> VALIDATING_MAP -> PLANNING ->
VALIDATING_PATH -> [REPAIRING] -> READY -> EXECUTING` 串联已有模块。规划失败、指纹不匹配、
非法 SWATH 或禁止 repair 时不会发送 Nav2 goal。执行前要求 `execution_enabled=true`、最新安全诊断
中 `motion_enabled=true` 且 `estop_latched=false`，但任务服务自身绝不调用安全使能服务。

最终路径仅通过标准 Nav2 `/follow_path` action 执行；速度仍经过既有 Nav2 remap 与 `agt_safety`
命令链。父 Goal 在执行阶段取消时会向 Nav2 子 Goal 转发取消，安全状态过期或失效也会先取消
子 Goal 再失败。swath 反馈基于 TASK-11 的 SWATH 区间和实际累计路径距离，索引从 0 开始；
`PAUSED` 暂为接口保留值。覆盖率和重叠率留待 TASK-16 计算，TASK-14 结果固定为 0。

## 总控接入

TASK-15 仅在 `agt_bringup` navigation 模式中条件组合现有节点。语义和覆盖开关默认 false；开启
覆盖必须同时开启语义服务器，并在任何子 launch 启动前确认基础地图、全局 PCD、GeoJSON、其
同目录 `coverage.yaml` 和 canonical platform profile 均存在。语义开关同时传给 Nav2 已有的
global Keepout Filter Info Server，不新增第二套 Nav2、TF、底盘或安全节点。

普通模式启动现有 Qt5 操作界面；`annotation_mode=true` 改为项目语义编辑器，并向 TASK-14 传入
`execution_enabled=false`。进程启动顺序不作为 readiness：覆盖 Goal 仍必须等待 semantic
`LOADED`、新 keepout mask、匹配验证产品、Nav2 ready 和 `agt_safety` 可运动状态。全部组件位于
同一 launch 进程树，正常 `Ctrl+C` 关闭 Action Server，安全与底盘 watchdog 负责残余命令归零。

## 离线预览边界

`coverage_preview.launch.py` 组合只读基础 map server、语义服务器、Coverage Server 和专用 RViz，
并固定自动规划与 `execution_enabled=false`。它不得启动 localization、Nav2 controller、
`agt_safety` 或底盘节点，也不得发布 TF 和速度命令。该入口用于查看 `path_preview`、重建路径、
SWATH、headland 和 keepout。`path_preview` 可在 PathComponents 语义重建失败时保留服务器路径，
但不得输入 Validator、repair、Action、Nav2 或底盘；每次新请求前必须先清空，防止显示旧结果。
没有 global costmap 时不产生 `path_validated`，不能把预览成功解释为
路径已通过完整 footprint 安全验收。

## 离线时间仿真

`coverage_time_simulator.py` 默认消费 `/agt/coverage/path_preview`，从 canonical platform profile
读取前进/倒车速度、线速度加减速、角速度和角加减速上限。模型对每个 Path 区间计算距离、航向
变化、前进/倒车和曲率，以 `min(线速度上限, 最大角速度/曲率)` 限速；起终点、纯旋转和前后换向
点速度固定为零，再执行前向加速和后向减速约束，使用梯形/三角速度曲线累计预计运行时间。

报告至少包含总/前进/倒车路径长度、纯旋转角、换向次数、估计转弯数和运动时间。只有
`path_semantics` 指纹与输入 Path 完全匹配时才计算 SWATH 作业长度/时间和 CONNECTION 非作业
长度/时间；否则 `classification_source=geometric_fallback`，这些字段为 null。该模型不包含履带
打滑、土壤阻力、电机动态、控制跟踪误差和停车作业时间，因此属于确定性运动学估算，不是实车
耗时承诺或 Gazebo 动力学结果。

## 离线多候选比较

`coverage_comparison.launch.py` 顺序复用一个 polygon Coverage Server，对
`coverage_variants.yaml` 中的路线排序、连接模型和作业方向候选逐一规划。默认比较相邻行
`BOUSTROPHEDON`、`SNAKE`、`SPIRAL`、仅前进 Dubins、允许倒车 Reeds-Shepp，以及作业方向
正负 15 度。它只发布 `/agt/coverage/comparison/markers`、status 和 JSON report，不发布任何
候选 `nav_msgs/Path`，也不启动 Validator、repair、Nav2 control、安全层或底盘。

几何排名依次使用预计运动时间、总路径长度和稳定 candidate ID，只用于离线筛选，永远不代表
可执行。报告中的所有 candidate 固定 `eligible_for_execution=false`。只有 PathComponents 完整
通过 TASK-11 SWATH/CONNECTION 重建后，才根据 authoritative SWATH 中心线和
`coverage.yaml.operation_width` 计算面积：`coverage_rate=SWATH 扫掠并集面积/可作业面积`，
`overlap_rate=(各 SWATH 扫掠面积之和-并集面积)/可作业面积`，`missed_area` 是可作业面积减去
扫掠并集。零长度 SWATH、重建长度不匹配或语义缺失时，所有面积字段为 null，不能用整条
Path buffer 伪造作业覆盖率。
