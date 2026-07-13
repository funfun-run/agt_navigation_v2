# agt_safety

本包独立完成速度仲裁、急停、超时和履带底盘运动约束，不依赖 Nav2 是否正常运行。

## 仲裁规则

- 新鲜的 `/agt/cmd_vel_manual` 优先于 `/agt/navigation/cmd_vel`。
- 手动命令超过 0.35 秒、导航命令超过 0.50 秒即失效。
- 启动默认禁止运动，必须调用 `/agt/safety/set_motion_enabled` 明确使能。
- `/agt/safety/emergency_stop=true` 会锁存急停并立即输出零速；输入恢复后仍需调用
  `/agt/safety/reset_emergency_stop`，再重新使能运动。
- 非有限数命令被拒绝；横移、升降、滚转和俯仰速度不会传给履带底盘。
- 对 `linear.x`、`angular.z` 做速度和加速度限制，并根据差速履带模型约束左右履带速度。

默认参数在 `config/bunker_safety.yaml`。当前值是低速联调起点，不是最终实车认证值；尤其
`effective_track_width=0.62 m` 是根据外廓宽度做的保守估计，需要测量左右履带中心距离并
通过原地转向测试校准。

急停示例：

```bash
ros2 topic pub --once /agt/safety/emergency_stop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /agt/safety/emergency_stop std_msgs/msg/Bool "{data: false}"
ros2 service call /agt/safety/reset_emergency_stop std_srvs/srv/Trigger "{}"
```

软件急停不能替代硬件急停。第一次实车测试应架空履带，随后在空旷区域以不高于
`0.15 m/s` 测量通信中断和急停制动距离，再逐级放开配置上限。
