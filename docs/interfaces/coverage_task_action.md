# 覆盖任务 Action 接口

TASK-13 生成 `agt_interfaces/action/ExecuteCoverageTask.action` 的 C/C++/Python typesupport；
TASK-14 在 `/agt/coverage/execute` 提供可取消 Action Server，串联语义加载、覆盖规划、路径验证、
可选 CONNECTION 修复和标准 Nav2 `FollowPath`。源码 `.action` 不是可独立使用的运行时接口。

## Goal

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `semantic_map_uri` | `string` | GeoJSON 普通路径或 `file://` URI |
| `field_id` | `string` | 必须与唯一 enabled field 的 ID 完全一致 |
| `planning_mode` | `string` | 必须与 `coverage.yaml` 的模式一致 |
| `controller_id` | `string` | 透传给 Nav2 `FollowPath` 的控制器 ID |
| `allow_repair` | `bool` | 验证失败时是否允许 TASK-12 修复 CONNECTION |

同一时刻只接受一个 Goal。空 URI、field、mode 或 controller 在进入语义加载前拒绝；新 Goal
不会复用旧语义、mask、验证报告或修复报告。

## Result

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `success` | `bool` | 仅 Nav2 完整执行成功时为 true |
| `error_code` | `uint16` | 下表稳定错误码 |
| `message` | `string` | 人类可读结果说明 |
| `coverage_rate` | `float64` | TASK-16 前固定为 0 |
| `overlap_rate` | `float64` | TASK-16 前固定为 0 |
| `executed_length` | `float64` | 根据 Nav2 剩余距离估计的执行长度，单位 m |
| `repaired_segment_count` | `uint32` | 本任务已采用的修复 CONNECTION 数量 |

| 错误码 | 含义 |
| --- | --- |
| `0` | 成功 |
| `100` | Goal 字段非法 |
| `110` | 语义地图或 mask 未就绪 |
| `120` | 请求适配或覆盖规划失败 |
| `130` | 路径验证产品非法或不匹配 |
| `140` | 路径非法且 `allow_repair=false` |
| `141` | CONNECTION 修复失败 |
| `150` | 安全状态未就绪、过期或执行中失效 |
| `160` | Nav2 不可用、拒绝、超时或执行失败 |
| `170` | 用户取消 |
| `199` | 未分类内部错误 |

## Feedback 与阶段

正常阶段为 `LOADING -> VALIDATING_MAP -> PLANNING -> VALIDATING_PATH -> [REPAIRING] ->
READY -> EXECUTING -> COMPLETED`；失败进入 `FAILED`，取消进入 `CANCELED`。`PAUSED` 是接口保留
值，TASK-14 不提供暂停/恢复操作，也不会发布该阶段。

`current_swath_index` 从 0 开始，`total_swaths` 来自 TASK-11 的 SWATH 组件。任务服务按 Nav2
`distance_to_goal` 和路径实际累计距离定位当前 SWATH，CONNECTION 只影响剩余距离，不增加作业行。

## 取消与安全边界

规划、验证和修复阶段取消会终止父任务并忽略晚到产品，不进入执行。TASK-15 已将任务服务与 Nav2
放入同一总控进程树，正常关闭会终止全部 Action Server。执行阶段取消必须先向活动的 Nav2
`FollowPath` 子 Goal 转发，并确认 Nav2 接受后才返回
`CANCELED`。安全诊断失效同样取消 Nav2，但父任务以错误码 150 失败。

任务服务不发布 `cmd_vel`、不调用 `/agt/safety/set_motion_enabled`、不直接控制底盘。默认
`execution_enabled=false`；启用后仍要求语义状态为 `LOADED`、Nav2 action ready，并在
`safety_status_timeout` 内收到指定诊断，且 `motion_enabled=true`、`estop_latched=false`。
`safety_status_name` 默认 `agt_safety/tracked_controller`，其他平台必须显式配置自己的诊断名。

`stage_timeout` 约束加载、规划、验证和修复等待；`execution_timeout=0` 表示农业作业执行期不设
总时限，正值才启用 Nav2 执行超时。

## 生成与验证

Python 通过 `agt_interfaces.action.ExecuteCoverageTask` 导入，C++ 通过
`agt_interfaces/action/execute_coverage_task.hpp` 导入。测试覆盖 ROSIDL 序列化、阶段反馈、规划
失败不执行、禁止修复立即失败、安全门禁、修复路径执行、执行中安全失效和父子 Goal 取消传播。
