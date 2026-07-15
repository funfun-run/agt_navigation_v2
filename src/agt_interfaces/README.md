# agt_interfaces

职责：定义统一的消息、服务、动作和状态契约。

TASK-13 已启用 ROSIDL 代码生成，当前接口：

- `action/ExecuteCoverageTask.action`：覆盖任务 Goal、Result 和 Feedback 数据结构。

构建后可通过 Python 的 `agt_interfaces.action.ExecuteCoverageTask` 或 C++ 的
`agt_interfaces/action/execute_coverage_task.hpp` 使用。包内测试会验证两种语言的生成产物，并对
Goal、Result、Feedback 执行 Python 序列化往返。

本包只定义数据接口，不实现 Action Server。取消、状态机、安全门和 Nav2 执行链属于 TASK-14
及后续任务。
