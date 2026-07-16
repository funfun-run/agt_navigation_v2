# launch

`coverage_planning.launch.py` 启动 polygon/row 两个 namespaced lifecycle server、各自的
lifecycle manager、语义请求适配器、Coverage Path Validator、CONNECTION Repair 和 TASK-14
Coverage Task Server。Validator 还需要 Nav2 发布
`/global_costmap/costmap` 与 `/global_costmap/published_footprint`；启动前必须 source TASK-08
外部依赖工作区。Repair 还要求 Nav2 `/compute_path_to_pose`、semantic status 和 keepout mask；
只有显式调用 `/agt/coverage/repair` 才会启动修复。

`execution_enabled:=false` 是默认值，Action 最多走到 `READY` 后以 fail-closed 结果结束；设为
`true` 也不会自动调用安全使能服务，只有近期 `agt_safety` 状态允许运动时才向 `/follow_path`
发送最终路径。

`coverage_preview.launch.py` 是纯离线可视化入口，组合基础 map server、语义服务器、覆盖规划和
专用 RViz，并固定 `plan_on_start:=true`、`execution_enabled:=false`。它只显示规划结果，不启动
定位、Nav2 controller、安全链或底盘；`path_validated` 仍需另行提供 global costmap 才会产生。
该入口同时启动 metrics-only 时间估算节点，默认消费 `path_preview`；可用
`simulation_report_path:=...` 将最新报告原子写入指定 JSON 文件。

`coverage_comparison.launch.py` 是独立的纯离线多候选入口，只组合 map server、语义服务器、
一个 polygon Coverage Server、候选比较器和 RViz。它不会启动 adapter 执行输出、Validator、
repair、Coverage Task Server、Nav2 control、安全层或底盘。`variants_file` 默认读取
`config/coverage_variants.yaml`，`report_path:=...` 可原子保存稳定键序 JSON。
