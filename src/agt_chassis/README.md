# agt_chassis

本包封装 AgileX BUNKER 的 CAN 通讯边界。官方 `bunker_base` 与 `bunker_msgs` 固定在
`third_party/bunker_ros2`，底层 SDK 固定在 `third_party/ugv_sdk`；上层模块不直接使用
厂商 `/cmd_vel` 或厂商状态 topic。

## 话题对齐

| 方向 | V2 接口 | 说明 |
| --- | --- | --- |
| 输入 | `/agt/safety/cmd_vel` | 安全层最终速度 |
| 内部 | `/agt/chassis/cmd_vel` | 独立 watchdog 输出，remap 到官方 `/cmd_vel` |
| 输出 | `/agt/chassis/odometry` | BUNKER 轮速积分里程计，仅作为融合输入 |
| 输出 | `/agt/chassis/status` | 标准 `DiagnosticArray` |
| 输出 | `/agt/chassis/connected` | 状态帧是否持续到达 |
| 输出 | `/battery` | 标准 `BatteryState`，百分比未知，仅提供电压 |
| 厂商 | `/agt/chassis/status/raw` | 原始 `bunker_msgs/BunkerStatus` |
| 厂商 | `/agt/chassis/rc_state` | 原始遥控器状态 |

官方 odom TF 默认关闭，避免与 FAST-LIVO2/融合模块争用 `odom -> base_footprint`。其里程计
frame 使用隔离名称 `bunker_odom -> base_footprint`，但只发布 Odometry 消息，不发布 TF。

## 启动

系统依赖：

```bash
sudo apt-get install -y libasio-dev can-utils
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agt_chassis bunker.launch.py can_interface:=can0
```

启动后默认禁止运动。确认车辆架空或周围安全、遥控器可随时接管后再使能：

```bash
ros2 service call /agt/safety/set_motion_enabled std_srvs/srv/SetBool "{data: true}"
```

若只验证安全层和 topic，不连接 CAN：

```bash
ros2 launch agt_chassis bunker.launch.py start_driver:=false
```

CAN 初始化按照上游脚本执行；首次配置用 `third_party/ugv_sdk/scripts/setup_can2usb.bash`，
以后每次上电用 `bringup_can2usb_500k.bash`。实机前先检查 `ip -details link show can0` 和
`candump can0`。

## 安全边界

`agt_chassis_command_guard` 在 0.20 秒内收不到安全命令即持续发布零速；官方驱动还有
0.25 秒的第二级 watchdog。两层都只保留 `linear.x` 和 `angular.z`，其他自由度强制为零。
这不能替代硬件急停、遥控器接管和实车制动距离验收。
