# 语义地图数据合同 1.0

本文定义 `agt_ui_bridge` 语义编辑器/服务器和 `agt_coverage_planning` 共同使用的数据合同。
机器可读约束位于 `src/agt_ui_bridge/config/semantic_schema.yaml`。编辑、服务加载、Keepout
栅格化、Nav2 KeepoutFilter 和 TASK-09 Fields2Cover 请求适配已实现；路径验证和任务执行
尚未实现。

## 坐标合同

- 所有几何使用 `frame_id: map`、米和 ROS 右手坐标系，角度使用弧度。
- GeoJSON 坐标直接表示 `[x, y]` 地图坐标，不是经纬度、栅格索引、图像像素或 Qt scene 坐标。
- 文件不得包含 GeoJSON `crs` 成员；坐标系由顶层和每个 Feature 的 `frame_id` 明确声明。
- Polygon 外环必须闭合，最后一点必须等于第一点，且至少包含 3 个不同顶点。
- Feature ID 在一个文件内唯一，使用小写 `snake_case`，保存后不得因显示顺序变化而重编号。

## 文件布局

实际地图任务写入运行目录，不提交 Git：

```text
runtime/maps/<map_id>/
├── <map_id>.pgm
├── <map_id>.yaml
└── semantic/
    ├── semantic_map.geojson
    ├── coverage.yaml
    ├── keepout_mask.pgm
    ├── keepout_mask.yaml
    └── validation_report.json
```

仓库中的 `docs/interfaces/examples/semantic_map/` 是可版本化合同样例，不是运行地图。

## GeoJSON 顶层

语义文件必须是 GeoJSON `FeatureCollection`，并包含以下外部成员：

| 成员 | 类型 | 约束 |
| --- | --- | --- |
| `type` | string | 必须为 `FeatureCollection` |
| `schema_version` | string | 第一版必须为 `1.0` |
| `map_id` | string | 必须与 `coverage.yaml` 和地图目录一致 |
| `frame_id` | string | 必须为 `map` |
| `features` | array | 所有农业语义对象 |

每个 Feature 必须包含 `id`、`feature_type`、`name`、`enabled` 和 `frame_id` 属性；`enabled`
必须为布尔值，`frame_id` 必须为 `map`。禁用对象可以保存和显示，但不进入规划输入。

## Feature 类型

| `feature_type` | Geometry | 数量 | 附加约束 |
| --- | --- | --- | --- |
| `field_boundary` | Polygon | 至少 1 个 | 作业区外边界，不允许自交 |
| `exclusion_zone` | Polygon | 至少 1 个 | 必须位于目标 field 内，不允许自交 |
| `entry_pose` | Point | 至少 1 个 | 属性必须包含有限数值 `yaw`，位置在 field 内且不在 exclusion 内 |
| `work_direction` | LineString | 至少 1 个 | 至少 2 点，首尾距离不得接近 0 |
| `row_centerline` | LineString | 可选 | 至少 2 点，表示已有作物行中心线 |
| `headland_zone` | Polygon | 可选 | 调头区域，不直接等同于禁行区 |
| `keepout_zone` | Polygon | 可选 | 仅用于 Nav2 语义禁行层 |

`speed_zone` 不属于 1.0，加载器必须将未知类型报告为错误，不得静默忽略。

## coverage.yaml

`coverage.yaml` 保存任务参数与基础地图绑定关系。最低字段如下：

```yaml
schema_version: "1.0"
map_id: greenhouse_01
frame_id: map
base_map: ../greenhouse_01.yaml
base_map_sha256: "<64 lowercase hexadecimal characters>"
robot_profile: bunker
planning_mode: polygon
robot_width: 0.938
operation_width: 0.60
min_turning_radius: 0.0
headland_width: 1.50
allow_reverse: true
preferred_swath_angle: 0.0
```

`planning_mode` 只允许 `polygon` 和 `annotated_rows`。`robot_width` 与
`min_turning_radius` 是从 `profiles/platforms/<robot_profile>.yaml` 生成或校验的快照，不是新的
车辆几何真源；任何不一致都必须阻止规划。`base_map_sha256` 定义为 `base_map` 指向的 Nav2
地图 YAML 文件原始字节的 SHA256，小写十六进制表示。

## 加载与一致性

加载时按以下顺序检查：

1. 两个文件可解析且 schema version 可识别；
2. `map_id` 和 `frame_id` 一致；
3. `base_map` 存在，SHA256 与记录值一致，其 image 文件存在；
4. Feature 必填属性、ID 唯一性和 Geometry 类型正确；
5. 坐标位于基础 OccupancyGrid 范围；
6. 多边形拓扑、包含关系、入口位姿和方向长度合法。

哈希不一致时只允许只读显示，禁止覆盖已有文件和启动覆盖规划。加载失败不得替换上一份已生效
语义地图，也不得自动修改基础 PGM/YAML 或用户语义文件。

## 错误级别

- `ERROR`：未知 schema、错误 frame、哈希不匹配、重复 ID、缺少必需对象、Geometry/拓扑非法；
  阻止保存为有效任务和覆盖规划。
- `WARNING`：可识别但不影响几何正确性的可选信息缺失；允许保存，但必须向用户显示。
- 所有错误必须包含稳定错误代码和对象 ID；顶层错误使用 `object_id: <document>`。

合法样例与非法样例见 `docs/interfaces/examples/semantic_map/README.md`。

## 无 GUI 基础库

`agt_ui_bridge` 提供以下不依赖 ROS graph 或 PyQt5 的 Python 模块：

| 模块 | 输入 | 输出/职责 |
| --- | --- | --- |
| `map_transform` | Nav2 YAML、grid/image/scene/world 坐标 | 处理 PGM Y 翻转、resolution、非零 origin、origin yaw 和边界检查 |
| `semantic_model` | GeoJSON/YAML 字典 | `SemanticMap`、`SemanticFeature`、`CoverageParameters` 可序列化模型 |
| `semantic_io` | 语义文件路径 | 原子读取/写入、基础地图 SHA256 检查、哈希异常只读降级 |
| `semantic_validation` | 数据模型 | 稳定错误代码、对象 ID 和阻断级结构检查 |
| `semantic_scene` | `SemanticMap` | 对象增删改、选择状态和有界 undo/redo |
| `semantic_rasterizer` | `SemanticMap`、`MapGeometry` | 生成与基础地图严格对齐的底向上 Keepout mask 数据 |

这些模块的坐标 frame 固定为 `map`，没有 ROS topic、QoS、TF 或节点参数。它们不负责 Qt
绘图、覆盖规划、导航执行或车辆控制。多边形自交、包含关系和地图范围等完整几何验证由
TASK-04 的 Shapely 验证层实现，TASK-06 栅格器只处理已验证几何。

## Qt5 语义编辑器 MVP

TASK-03 的 `agt_ui_bridge/semantic_editor_qt5.py` 是本合同的本地文件编辑入口。它以 Nav2
地图 YAML 为只读底图，以所选 `profiles/platforms/<platform>.yaml` 为 footprint 和
`robot_width` 真源，输出相互配套的 `semantic_map.geojson` 与 `coverage.yaml`。

MVP 支持语义对象绘制、顶点拖动、ID/名称编辑、对象删除、图层显隐、footprint 预览、
undo/redo、保存重载及未保存退出提示。所有 scene 点击和拖动必须经 `MapTransform` 转换为
`map` frame 米制坐标，不允许直接把像素坐标写入 GeoJSON。

编辑器不发布 ROS topic 或 TF，不修改基础 PGM/YAML，也不修改第三方 Qt 主界面。基础地图
SHA256 不一致时只读显示，结构或几何校验失败时高亮错误对象并阻止有效任务保存。编辑器不
包含 Keepout 栅格化、Fields2Cover 规划或 Nav2 执行。

## Shapely 几何合法性

TASK-04 使用 Shapely/GEOS，不自行实现多边形布尔算法。完整上下文由基础地图几何、所选平台
`navigation_footprint`、基础地图路径和显式 `minimum_boundary_clearance` 组成。净距默认
`0.0 m`，不得作为第二套隐式安全裕量。

| 错误 code | 对象 | 阻断条件 |
| --- | --- | --- |
| `polygon_self_intersection` | Polygon Feature | 多边形自交 |
| `invalid_polygon_topology` | Polygon Feature | 除自交外的 GEOS 非法拓扑 |
| `exclusion_outside_field` | exclusion ID | 未完整位于任一 enabled field 内 |
| `entry_outside_field` | entry ID | 入口不在任一 enabled field 内 |
| `entry_inside_exclusion` | entry ID | 入口点位于 exclusion 内或边界上 |
| `coordinate_outside_map` | Feature ID | 几何超出含 origin yaw 的基础地图范围 |
| `entry_footprint_outside_field` | entry ID | 入口位姿下 navigation footprint 无法完整放入 field |
| `entry_footprint_intersects_exclusion` | entry ID | 入口 footprint 与 exclusion 相交或接触 |
| `insufficient_boundary_clearance` | entry ID | footprint 到 field/exclusion 的最小净距低于显式阈值 |
| `base_map_hash_mismatch` | document | 基础地图 YAML SHA256 与 coverage 不一致 |

禁用 Feature 不参与空间关系和 footprint 检查，但仍接受 schema/属性检查。存在任一 `ERROR`
时禁止保存有效任务或进入后续覆盖规划；UI 合法性列表必须显示 code、对象 ID 和具体消息。

## Keepout 栅格合同

TASK-06 对所有 enabled `exclusion_zone` 和 `keepout_zone` 取并集；默认将 enabled
`field_boundary` 并集之外设为禁行。输出按 OccupancyGrid 的底向上 row-major 顺序排列，默认
可通行值为 `0`、禁行值为 `100`。禁用对象、row、entry、direction 和 headland 不参与 mask。

world 坐标先按基础地图 origin yaw 变换到本地地图坐标，再投影到原始 resolution 网格；输出
不得改变 width、height、resolution、origin、frame，也不得读取后重写源 PGM。Polygon 离散
边界允许最多一个栅格误差。

TASK-07 仅由 Nav2 costmap 过滤层消费上述 mask，不生成或改写 PGM。FilterInfo 使用 keepout
类型 `0`、`base=0.0`、`multiplier=1.0`；global costmap 在 KeepoutFilter 后执行 inflation。
