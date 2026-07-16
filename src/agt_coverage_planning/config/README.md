# config

`coverage_planning.yaml` 配置 TASK-09 适配器、polygon/row server 和 TASK-10 Validator。车辆
宽度、作业宽度和最小转弯半径以每次已验证任务/profile 为准；Validator 的 footprint 也只读取
同一 profile。未知空间默认碰撞，地图外默认碰撞，占用阈值默认 65；修改这些策略必须显式记录。
TASK-11 的 swath 重建步长默认 `0.10 m`，长度容差为绝对 `0.05 m` 或相对 `0.005` 中较大者；
Validator 通过 `semantics_topic` 校验语义指纹并引用 component/swath ID。

离线时间估算不复制车辆限速参数，始终读取所选 canonical platform profile 的 `limits`。默认
预览输入为 `/agt/coverage/path_preview`；语义指纹匹配时才计算 SWATH/CONNECTION 分项，否则仅
报告几何时间估算并标记 `geometric_fallback`。

`coverage_variants.yaml` 定义离线候选 ID、显示名称、OpenNav route/path mode、作业方向偏移和
SPIRAL 的 `spiral_n`。首版支持 `BOUSTROPHEDON/SNAKE/SPIRAL` 与
`DUBIN/REEDS_SHEPP`；配置只影响离线比较器，不覆盖语义任务或执行链参数。
TASK-12 repair 参数定义 Nav2 action、global costmap、keepout mask、semantic status、端点容差和
同一套 Validator 策略。planner ID 不在此文件复制，由所选 platform profile 的
`coverage_repair` 合同提供。
TASK-14 task server 默认 `execution_enabled: false`。`stage_timeout` 用于加载、规划和验证，
`execution_timeout: 0` 表示执行不限总时长；`safety_status_name` 必须匹配所选平台安全节点发布的
诊断项，诊断超过 `safety_status_timeout` 即 fail-closed。
