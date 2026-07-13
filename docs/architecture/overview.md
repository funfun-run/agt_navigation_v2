# 架构总览

## 分层
- 适配与描述层：`agt_description`、`agt_sensor_adapters`
- 建图定位感知层：`agt_mapping`、`agt_localization`、`agt_localization_fusion`、`agt_perception`
- 地图与规划层：`agt_map_processing`、`agt_navigation`、`agt_coverage_planning`
- 执行与系统边界层：`agt_safety`、`agt_chassis`、`agt_ui_bridge`、`agt_experiment_manager`
- 评测层：`agt_evaluation`

## 当前目标
- 建立统一 TF、topic、状态接口和配置组织方式
- 为后续模块迁移提供稳定包边界

## 本阶段不做
- 第三方算法源码迁移
- 具体导航参数调优
- GUI 和覆盖规划业务实现
