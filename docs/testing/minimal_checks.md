# 最小检查项

## 仓库级
- `colcon list` 能识别所有 package
- 占位测试能被发现
- package、frame、topic 和参数符合命名规范
- 所有 Python launch 文件可以完成语法解析

## Phase 2 预备检查
- TF 树无重复发布责任
- `colcon test --packages-select agt_description` 通过
- `base_footprint -> base_link -> lidar_link -> livox_frame` 连通
- 实车外参未测量前保持 `calibration_verified: false`
- `agt_description` 可以单独启动并输出机器人描述

## Phase 3 离线检查
- FAST-LIVO2 原生 TF 补丁可以应用到基线源码
- adapter 位姿和平移/旋转外参换算测试通过
- adapter twist 旋转和传感器杠杆臂换算测试通过
- 注册点云 remap 为 `/agt/mapping/registered_points`
- 标准里程计输出为 `/agt/mapping/odometry`，child frame 为 `base_footprint`

## Phase 3 待数据检查
- 使用同一 bag 对比旧链与 V2 的点云、里程计、时间戳和轨迹
- 检查 `odom -> base_footprint` 只有 adapter 一个发布源
- 检查长时间运行漂移、丢帧、延迟和资源占用
- 使用实测外参完成直线、转向和倾斜安装验证
