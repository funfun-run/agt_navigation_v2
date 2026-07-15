# action

统一任务动作接口目录。

- `ExecuteCoverageTask.action`：覆盖任务请求、结果与阶段反馈。TASK-13 只负责生成和序列化；
  Action Server 行为在 TASK-14 实现。
