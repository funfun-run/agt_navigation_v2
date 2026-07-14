# AGT Navigation V2：Qt5 语义标注、Fields2Cover 覆盖规划与路径修复实施规范

> 文档用途：供 Codex 或其他代码代理直接读取并按阶段实施。  
> 仓库：`https://github.com/Aldoubt/agt_navigation_v2.git`  
> 基线分支：`main`  
> 基线提交：`0fe28df792bf399e2c53f2ba133ffefbe266c079`  
> 目标平台：ROS 2 Humble，Ubuntu 22.04  
> 文档状态：实施约束文档，不代表所有模块已经实现。

---

## 1. 项目目标

在现有 `agt_navigation_v2` 仓库中，建立一条可复用的农业机器人覆盖作业链：

```text
OccupancyGrid 基础地图
    ↓
Qt5 语义标注
    ↓
GeoJSON / YAML 语义任务文件
    ↓
Nav2 KeepoutFilter 语义禁行层
    ↓
Fields2Cover / opennav_coverage 生成覆盖路径
    ↓
基于 Nav2 Costmap 和车辆 footprint 的路径验证
    ↓
仅修复无效连接段
    ↓
Nav2 FollowPath + MPPI 执行
    ↓
Collision Monitor 与 agt_safety 兜底
```

第一目标不是开发新的覆盖规划算法，而是把现有成熟组件正确连接起来，并保证：

1. 语义标注结果可保存、可重载、可验证；
2. 语义地图与基础 OccupancyGrid 严格对齐；
3. 覆盖路径满足作业区域、禁行区域和车辆几何约束；
4. 路径修复不破坏 Fields2Cover 生成的作业行；
5. 所有阶段可独立测试、回滚和验收。

---

## 2. 当前仓库状态

### 2.1 已有能力

- `agt_ui_bridge` 已有 Qt5 主界面适配和轻量 Python 地图编辑器；
- 当前地图编辑器可以修改 `OccupancyGrid` 中的障碍、自由空间和未知区域；
- 当前界面可以发布 `/initialpose` 和 `/goal_pose`；
- `agt_navigation` 已有 Nav2、SmacPlanner2D、MPPI、完整 footprint 代价检查和 Collision Monitor；
- `agt_coverage_planning` 已建立包边界；
- `nav_dependencies.repos` 已预留 Fields2Cover；
- BUNKER 平台 profile 已包含实际尺寸和导航 footprint。

### 2.2 当前缺口

现有“地图编辑”不等于“农业语义标注”。当前缺少：

- 作业区 Polygon；
- 内部障碍或禁行区 Polygon；
- 作物行 LineString；
- 入口位姿；
- 作业方向；
- 地头区域；
- 独立语义文件格式；
- 语义地图加载、保存和合法性检查；
- KeepoutFilter mask；
- Fields2Cover ROS 2 适配节点；
- 覆盖路径碰撞验证；
- 连接段局部修复；
- 覆盖任务 Action；
- 覆盖率、重复率和修复结果评测。

---

## 3. 全局任务边界

以下约束适用于所有阶段。

### 3.1 必须遵守

1. 一次只实施一个阶段或一个明确任务。
2. 每次变更必须可独立构建、测试和回滚。
3. 所有语义几何坐标统一使用 `frame_id: map`、米制坐标和 ROS 右手坐标系。
4. 禁止将语义对象保存为 Qt 窗口像素坐标。
5. 禁止将语义禁行区永久写入原始 PGM 地图。
6. 禁止在多个 YAML 中手工维护不同版本的车辆 footprint。
7. 所有车辆几何必须来自平台 profile，或由平台 profile 自动生成。
8. 不修改已验证的定位、建图、底盘、安全链参数，除非当前任务明确要求。
9. 任何架构、接口或启动方式变化必须同步更新：
   - `README.md`；
   - 对应 package README；
   - `docs/migration/migration_matrix.md`；
   - 必要的接口文档。
10. 新增节点必须写明输入、输出、frame、QoS、参数和非职责范围。
11. 所有地图类持久话题使用 `RELIABLE + TRANSIENT_LOCAL`。
12. 所有生成文件必须写入 `runtime/`，不得把实验产物提交到 Git。

### 3.2 禁止事项

除非任务明确解锁，Codex 不得：

- 重写整个仓库；
- 修改 `third_party/ros_qt5_gui_app`；
- 重构 FAST-LIVO2；
- 修改 ICP/NDT 定位逻辑；
- 修改 BUNKER CAN 协议；
- 修改 `agt_safety` 仲裁逻辑；
- 用自研算法替代 Fields2Cover、Nav2 Costmap 或 FootprintCollisionChecker；
- 将 Fields2Cover 直接耦合进 Qt UI；
- 在 UI 线程中运行覆盖规划；
- 让多个节点发布同一条 TF；
- 自动修改真实地图、标定文件或用户数据；
- 在未完成合法性检查前启动覆盖规划；
- 在路径修复时改变有效作业行的几何形状。

---

## 4. 目标模块边界

### 4.1 `agt_ui_bridge`

负责：

- OccupancyGrid 显示；
- 语义对象编辑；
- 语义文件加载和保存；
- UI 合法性提示；
- 将语义对象发布为标准可视化消息。

不负责：

- Fields2Cover 算法；
- 路径碰撞判断；
- 路径修复；
- 导航执行；
- 车辆控制。

### 4.2 `agt_coverage_planning`

负责：

- 读取语义任务；
- 转换为 Fields2Cover / opennav_coverage 输入；
- 生成原始覆盖路径；
- 区分作业行和连接段；
- 路径验证；
- 连接段修复；
- 发布覆盖规划状态。

不负责：

- Qt 绘图；
- 地图建图；
- 定位；
- 底盘控制；
- 紧急停车。

### 4.3 `agt_navigation`

负责：

- Nav2 Costmap；
- KeepoutFilter；
- 点到点规划；
- 路径跟踪；
- MPPI；
- Collision Monitor。

不负责：

- 语义文件解析；
- Fields2Cover 场景建模；
- 覆盖率统计。

### 4.4 `agt_interfaces`

负责跨包使用的 msg、srv、action，以及 ROS 2 接口生成和导出。

### 4.5 `agt_evaluation`

后续负责：

- 覆盖率；
- 重复覆盖率；
- 路径长度；
- 非作业行程；
- 转弯次数；
- 碰撞数量；
- 修复数量；
- 规划耗时；
- 执行成功率。

---

## 5. 数据模型

### 5.1 目录结构

每张地图使用以下结构：

```text
runtime/maps/<map_id>/
├── <map_id>.pgm
├── <map_id>.yaml
├── pcd/
│   ├── all_raw_points.pcd
│   └── all_downsampled_points.pcd
└── semantic/
    ├── semantic_map.geojson
    ├── coverage.yaml
    ├── keepout_mask.pgm
    ├── keepout_mask.yaml
    └── validation_report.json
```

### 5.2 第一版语义类型

| `feature_type` | GeoJSON 类型 | 必需 | 说明 |
|---|---|---:|---|
| `field_boundary` | Polygon | 是 | 覆盖作业外边界 |
| `exclusion_zone` | Polygon | 是 | 内部障碍、支柱、水沟等 |
| `entry_pose` | Point | 是 | 属性中保存 yaw |
| `work_direction` | LineString | 是 | 两点定义作业方向 |
| `row_centerline` | LineString | 否 | 温室或果园已有作物行 |
| `headland_zone` | Polygon | 否 | 调头区域 |
| `keepout_zone` | Polygon | 否 | 仅用于 Nav2 禁行 |
| `speed_zone` | Polygon | 后续 | 速度限制区，第一版不实现 |

### 5.3 GeoJSON 属性最低要求

```json
{
  "id": "field_01",
  "feature_type": "field_boundary",
  "name": "温室一区",
  "enabled": true,
  "frame_id": "map"
}
```

`entry_pose` 额外包含：

```json
{
  "yaw": 1.57079632679
}
```

### 5.4 `coverage.yaml` 最低字段

```yaml
schema_version: "1.0"
map_id: greenhouse_01
frame_id: map
base_map: ../greenhouse_01.yaml
base_map_sha256: ""
robot_profile: bunker

planning_mode: polygon
robot_width: 0.938
operation_width: 0.60
min_turning_radius: 0.0
headland_width: 1.50
allow_reverse: true
preferred_swath_angle: 0.0
```

`planning_mode` 允许值：

```text
polygon
annotated_rows
```

### 5.5 文件一致性要求

语义文件必须记录基础地图哈希。加载时必须检查：

- 基础地图文件存在；
- map ID 一致；
- SHA256 一致；
- `frame_id == map`；
- 所有坐标位于地图范围内；
- schema version 可识别。

哈希不一致时：

- 允许只读打开；
- 禁止生成覆盖路径；
- UI 显示明确警告；
- 不自动修改文件。

---

## 6. 实施阶段

## Phase 0：统一车辆几何与语义规范

### TASK-00：统一车辆 footprint 数据源

#### 目标

消除 BUNKER footprint 在多个模块中重复维护的问题。

#### 当前已知基线

当前平台 profile 使用：

```yaml
navigation_footprint:
  - [0.5915, 0.4690]
  - [0.5915, -0.4690]
  - [-0.5915, -0.4690]
  - [-0.5915, 0.4690]
```

该尺寸对应车辆外形加 80 mm 安全裕量。

#### 允许修改

- `profiles/platforms/bunker.yaml`
- 新增 footprint 配置生成脚本；
- 新增一致性测试；
- 必要时修改：
  - `src/agt_navigation/config/nav2_bunker.yaml`
  - `src/agt_perception/config/local_obstacle_filter.yaml`

#### 禁止修改

- BUNKER 实际外形尺寸；
- CAN 参数；
- 速度和加速度参数；
- 外参文件；
- 安全仲裁逻辑。

#### 实施要求

建立单一数据源：

```text
profiles/platforms/bunker.yaml
        ↓
生成或测试约束
        ↓
Nav2 footprint
local_obstacle_filter 车体裁剪
coverage validator footprint
```

第一阶段允许先用测试保证一致，不强制立即建立自动生成器。

#### 验收标准

- 三处 footprint 数值完全一致；
- 修改 profile 后，一致性测试能发现其他配置未同步；
- 不存在 80 mm 与 100 mm 两套导航 footprint 同时生效；
- `pytest` 中新增对应契约测试。

---

### TASK-01：定义语义地图规范

#### 目标

建立 UI、规划器和测试共同遵守的数据合同。

#### 允许新增

```text
docs/interfaces/semantic_map_schema.md
src/agt_ui_bridge/config/semantic_schema.yaml
tests/test_semantic_map_contracts.py
runtime/maps/example_semantic/
```

#### 禁止实施

- Qt 绘图；
- Fields2Cover；
- KeepoutFilter；
- 自定义 ROS Action。

#### 验收标准

- 文档完整定义所有第一版 Feature；
- 提供至少一个合法示例；
- 提供至少三个非法示例；
- 测试可验证必填字段、frame、ID 唯一性和 schema version；
- 不依赖 ROS 节点即可运行 schema 测试。

---

## Phase 1：Qt5 语义标注 MVP

> 本阶段是第一轮实际开发范围。  
> 本阶段完成前，不接入 Fields2Cover。

### TASK-02：拆分语义基础库

#### 目标

将坐标变换、数据模型和文件读写从 UI 代码中拆出。

#### 建议结构

```text
src/agt_ui_bridge/
├── agt_ui_bridge/
│   ├── __init__.py
│   ├── map_transform.py
│   ├── semantic_model.py
│   ├── semantic_io.py
│   ├── semantic_validation.py
│   └── semantic_scene.py
├── scripts/
│   └── semantic_editor_qt5.py
└── test/
    ├── test_map_transform.py
    ├── test_semantic_io.py
    └── test_semantic_validation.py
```

#### 允许修改

- `src/agt_ui_bridge/CMakeLists.txt`
- `src/agt_ui_bridge/package.xml`
- `src/agt_ui_bridge/README.md`
- 上述新增文件。

#### 禁止修改

- `third_party/ros_qt5_gui_app/**`
- 原有 `map_io_bridge.py` 行为；
- 原有 PGM/YAML 保存格式；
- Nav2 配置。

#### 实施要求

`map_transform.py` 必须支持：

```text
grid cell ↔ map world coordinate
image pixel ↔ grid cell
Qt scene coordinate ↔ map world coordinate
```

必须正确处理：

- PGM Y 轴翻转；
- 非零 origin；
- origin yaw；
- 任意 resolution；
- 地图边界检查。

#### 验收标准

- 坐标往返误差小于 `0.5 * resolution`；
- 带 yaw 的地图原点测试通过；
- 所有业务逻辑可在无 GUI 环境下单测；
- 原有地图编辑器测试继续通过。

---

### TASK-03：实现语义编辑器 MVP

#### 目标

在只读 OccupancyGrid 底图上编辑农业语义对象。

#### 必须实现

- 加载基础 Nav2 地图；
- 平移；
- 滚轮缩放；
- 绘制作业区；
- 绘制内部障碍；
- 绘制作物行；
- 设置入口位置和朝向；
- 设置作业方向；
- 顶点选择；
- 顶点拖动；
- 对象删除；
- 撤销；
- 重做；
- 图层显示和隐藏；
- 对象 ID 和名称；
- 保存；
- 另存为；
- 重新加载；
- 未保存退出提示；
- 状态栏显示地图坐标。

#### 建议 Qt 架构

```text
QGraphicsView
└── QGraphicsScene
    ├── OccupancyGrid 底图
    ├── field_boundary
    ├── exclusion_zone
    ├── row_centerline
    ├── entry_pose
    ├── work_direction
    └── footprint preview
```

#### 允许修改

- `src/agt_ui_bridge/**`
- 新增 launch 和 config；
- 必要的 package 依赖。

#### 禁止修改

- vendored C++ Qt 主界面；
- Fields2Cover；
- Nav2；
- OccupancyGrid 原始数据；
- 真实地图文件，除非用户主动选择保存。

#### 验收标准

完成以下闭环：

```text
加载地图
→ 绘制四类必需对象
→ 保存 GeoJSON 和 coverage.yaml
→ 关闭编辑器
→ 重新启动
→ 重新加载
→ 所有对象坐标、ID、属性和显示结果一致
```

额外要求：

- 保存前必须运行合法性检查；
- 非法对象不得静默保存；
- UI 崩溃不得破坏已存在文件；
- 保存采用临时文件加原子替换。

---

### TASK-04：语义合法性检查

#### 必须检查

- `field_boundary` 数量至少为 1；
- Polygon 至少 3 个不同顶点；
- Polygon 不自交；
- `exclusion_zone` 位于目标 field 内；
- Feature ID 不重复；
- `entry_pose` 位于 field 内；
- `entry_pose` 不在 exclusion zone 内；
- `work_direction` 两端点距离不接近 0；
- `row_centerline` 至少包含 2 个点；
- 所有坐标位于基础地图覆盖范围；
- frame 为 `map`；
- 基础地图哈希一致。

#### 推荐依赖

优先使用成熟几何库，例如 GEOS/Shapely。不得自行实现完整多边形布尔运算。

#### 验收标准

- 非法对象在 UI 中高亮；
- UI 显示具体错误；
- 错误中包含对象 ID；
- 存在阻断级错误时禁止生成覆盖任务；
- 单元测试覆盖每一种错误类型。

---

### Phase 1 完成定义

Phase 1 只有满足以下全部条件才算完成：

- Qt5 可以稳定标注并保存语义地图；
- 保存文件可重载；
- 坐标无翻转、缩放或 origin 偏移；
- 基础地图只读；
- 未修改 `third_party`；
- 未接入 F2C；
- 单元测试和现有回归全部通过；
- 文档和迁移矩阵已更新。

---

## Phase 2：语义地图服务器与 Nav2 KeepoutFilter

### TASK-05：实现语义地图服务器

#### 节点名

```text
agt_semantic_map_server
```

#### 输入

- `semantic_map.geojson`
- `coverage.yaml`
- 基础地图 OccupancyGrid。

#### 发布

```text
/agt/map/semantic_markers
    visualization_msgs/msg/MarkerArray

/agt/map/keepout_mask
    nav_msgs/msg/OccupancyGrid

/agt/map/semantic_status
    diagnostic_msgs/msg/DiagnosticArray
```

#### 服务

第一版优先使用标准服务或简单 Trigger/SetBool 组合；确实无法表达时才新增自定义 srv。

建议服务语义：

```text
/agt/map/semantic/load
/agt/map/semantic/reload
/agt/map/semantic/validate
```

#### QoS

地图、mask、marker：

```text
RELIABLE
TRANSIENT_LOCAL
depth = 1
```

#### 验收标准

- 节点晚启动仍能拿到基础地图；
- 订阅节点晚启动仍能收到语义 mask；
- 加载失败不覆盖上一份有效语义地图；
- 状态中能区分未加载、加载成功、哈希不匹配、几何非法和栅格化失败。

---

### TASK-06：生成 KeepoutFilter mask

#### 语义来源

以下对象参与栅格化：

- `exclusion_zone`
- `keepout_zone`

`field_boundary` 外部是否设为禁行由参数决定，第一版默认设为禁行。

#### 对齐约束

生成 mask 必须与基础 OccupancyGrid 完全一致：

```text
resolution
width
height
origin
origin yaw
frame_id
```

#### 禁止事项

- 不得重采样成另一分辨率；
- 不得覆盖 `/agt/map/global_occupancy`；
- 不得直接修改原 PGM。

#### 验收标准

- 合成地图上 Polygon 边界与 mask 对齐误差不超过 1 个栅格；
- 带 origin yaw 的地图测试通过；
- exclusion zone 内部为禁行；
- field 外部按配置处理；
- mask 可由 RViz 正确显示。

---

### TASK-07：接入 Nav2 KeepoutFilter

#### 允许修改

- `src/agt_navigation/config/nav2_bunker.yaml`
- `src/agt_navigation/launch/navigation.launch.py`
- 对应 README 和测试。

#### 全局代价地图目标层级

```text
StaticLayer
KeepoutFilter
InflationLayer
```

实际插件配置按 ROS 2 Humble Nav2 支持方式实现。

#### 禁止修改

- MPPI 核心参数；
- Collision Monitor 区域；
- local costmap 障碍点云链；
- 地图服务器 topic；
- 定位 TF。

#### 验收标准

- 未启动语义服务器时有明确行为；
- 启动后禁行区进入 global costmap；
- 全局规划器不能穿越禁行区；
- 关闭或切换语义地图不会污染基础地图；
- 原有 1 m 离线导航闭环继续通过。

---

## Phase 3：Fields2Cover / opennav_coverage 接入

### TASK-08：固定依赖版本

#### 目标版本

```text
ROS 2 Humble
opennav_coverage: humble-v2
Fields2Cover: v2.0.0
```

在实施时再次验证依赖兼容性，然后固定 commit 或 tag。

#### 允许修改

- `nav_dependencies.repos`
- `third_party/README.md`
- 依赖安装文档。

#### 禁止事项

- 使用 `<pin-commit-or-tag>` 占位符；
- 直接复制外部算法源码到 `agt_coverage_planning`；
- 修改外部依赖源码以适配本项目，除非独立补丁且有文档。

#### 验收标准

- 新工作区执行 `vcs import` 可复现依赖；
- `rosdep` 和 `colcon build` 可完成；
- 依赖来源、版本和许可证有记录；
- 无隐式依赖本机旧工作区。

---

### TASK-09：实现语义到覆盖规划请求的适配

#### 输入

- 已通过验证的 GeoJSON；
- `coverage.yaml`；
- 机器人 profile。

#### 转换规则

```text
field_boundary
    → polygons[0]

exclusion_zone
    → polygons[1...N]

row_centerline
    → opennav_row_coverage 行输入

work_direction
    → swath angle

robot profile
    → robot_width
      min_turning_radius

coverage.yaml
    → operation_width
      headland_width
      planning_mode
      allow_reverse
```

#### 输出

```text
/agt/coverage/path_raw
    nav_msgs/msg/Path

/agt/coverage/path_components
    opennav_coverage_msgs 对应消息

/agt/coverage/swaths
    visualization_msgs/msg/MarkerArray

/agt/coverage/headland
    visualization_msgs/msg/MarkerArray

/agt/coverage/status
    diagnostic_msgs/msg/DiagnosticArray
```

#### Humble 边界

第一版只依赖 Coverage Server 和 `ComputeCoveragePath`。不得把 Iron 以上才支持的 Navigator 插件作为第一版必需条件。

#### 验收标准

- 矩形空场地能生成覆盖路径；
- 带内部障碍场地不会穿越障碍孔洞；
- annotated rows 模式能按已有行生成路径；
- 输出 frame 为 `map`；
- 输出路径 orientation 有效；
- 规划失败能返回明确错误原因。

---

## Phase 4：路径验证与连接段修复

### TASK-10：实现 Coverage Path Validator

#### 输入

```text
/agt/coverage/path_raw
/global_costmap/costmap
/global_costmap/published_footprint
```

#### 核心步骤

```text
检查 Path frame
→ 路径距离插值
→ 路径角度插值
→ 每个姿态执行 footprint collision check
→ 记录 Costmap 最大代价
→ 检查曲率
→ 检查最小转弯半径
→ 标记无效路径区间
```

#### 插值约束

应以 costmap resolution 和 footprint 最大半径计算采样步长。不得只检查原始 Path 点。

#### 输出

```text
/agt/coverage/path_validated
/agt/coverage/collision_poses
/agt/coverage/footprint_markers
/agt/coverage/validation_report
```

#### Validation Report 最低字段

```json
{
  "valid": false,
  "collision_pose_count": 3,
  "invalid_segment_indices": [4, 5],
  "maximum_cost": 254,
  "minimum_clearance": 0.12,
  "maximum_curvature": 1.8,
  "required_min_turning_radius": 0.55
}
```

#### 禁止事项

- 不得只检查中心点；
- 不得只检查 footprint 四个角；
- 不得把 MPPI 的执行期检查当成规划期验证；
- 不得在 Validator 中直接控制底盘。

#### 验收标准

- 稀疏路径中间碰撞能被发现；
- 原地旋转时角点碰撞能被发现；
- 未知空间策略可配置；
- footprint 来源与平台 profile 一致；
- 报告可稳定复现。

---

### TASK-11：区分作业行与连接段

#### 目标

保留 Fields2Cover 路径语义，避免修复器破坏覆盖效果。

#### 路径分类

```text
SWATH
CONNECTION
APPROACH
EXIT
```

第一版至少支持：

```text
SWATH
CONNECTION
```

#### 验收标准

- 每个 Path 区间都有类型；
- 能从 `path_components` 重建扁平 Path；
- 重建前后路径长度误差在允许范围；
- 作业行编号稳定；
- 规划和验证报告可引用 swath ID。

---

### TASK-12：仅修复无效连接段

#### 允许修复

- 无效 `CONNECTION`；
- 从入口到第一条作业行的接近段；
- 最后一条作业行到出口的离开段。

#### 禁止修复

- 有效 `SWATH`；
- 被标为必须保持的作物行；
- field boundary；
- exclusion zone；
- 用户手工指定的作业顺序，除非参数明确允许。

#### 修复策略

```text
发现无效连接段
→ 取连接段起点和终点
→ 调用 Nav2 规划器
→ 限制在允许区域内
→ 验证新连接段
→ 替换原连接段
→ 重新拼接完整路径
```

#### 底盘差异

- BUNKER：履带差速，可允许原地旋转；
- MK-mini：Ackermann，需使用 Hybrid-A* 或 State Lattice；
- 不得共用同一运动模型参数。

#### 输出

```text
/agt/coverage/path_repaired
/agt/coverage/repair_report
```

#### 验收标准

- 修复前后所有 SWATH 点坐标完全不变；
- 无效连接段被替换后再次通过 Validator；
- 修复失败时保留原始路径并返回失败；
- 不允许静默删除作业行；
- 修复数量和耗时进入报告。

---

## Phase 5：任务 Action、总控和评测

### TASK-13：修复 `agt_interfaces`

#### 当前问题

`agt_interfaces` 目前只安装 `msg/srv/action` 目录，没有调用 `rosidl_generate_interfaces()`。

#### 必须实现

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteCoverageTask.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
```

并更新 `package.xml`。

#### 验收标准

- `colcon build` 生成接口；
- Python 和 C++ 均可导入；
- 接口序列化测试通过；
- 不使用未生成的纯文本 action 文件冒充接口。

---

### TASK-14：定义覆盖任务 Action

建议接口：

```text
ExecuteCoverageTask.action
```

#### Goal

```text
string semantic_map_uri
string field_id
string planning_mode
string controller_id
bool allow_repair
---
```

#### Result

```text
bool success
uint16 error_code
string message
float64 coverage_rate
float64 overlap_rate
float64 executed_length
uint32 repaired_segment_count
---
```

#### Feedback

```text
string current_stage
uint32 current_swath_index
uint32 total_swaths
float64 distance_remaining
```

#### `current_stage` 允许值

```text
LOADING
VALIDATING_MAP
PLANNING
VALIDATING_PATH
REPAIRING
READY
EXECUTING
PAUSED
COMPLETED
FAILED
CANCELED
```

#### 验收标准

- Goal 可取消；
- 规划失败不进入执行；
- 路径非法且不允许 repair 时立即失败；
- 执行中反馈 swath 进度；
- Action Server 不直接绕过 Nav2 和 `agt_safety`。

---

### TASK-15：接入总控

#### 允许修改

- `src/agt_bringup/launch/system.launch.py`
- `src/agt_bringup/launch/navigation_system.launch.py`
- bag 录制列表；
- README。

#### 新增参数

```text
start_semantic_map_server
start_coverage_planning
semantic_map
coverage_params
annotation_mode
```

#### 推荐启动方式

```bash
ros2 launch agt_bringup system.launch.py \
  mode:=navigation \
  map:=/absolute/path/greenhouse_01.yaml \
  global_map_pcd:=/absolute/path/all_downsampled_points.pcd \
  semantic_map:=/absolute/path/semantic_map.geojson \
  coverage_params:=/absolute/path/coverage.yaml \
  start_coverage_planning:=true
```

#### 启动顺序

```text
map_server active
→ localization ready
→ semantic map loaded
→ keepout mask available
→ global costmap active
→ coverage planner ready
→ coverage task allowed
```

#### 验收标准

- 任一前置条件失败时禁止运动；
- 启动失败有明确日志；
- 不重复启动 Nav2、TF 或底盘节点；
- `start_coverage_planning:=false` 时原有导航不受影响；
- 关闭总控时所有 Action 正确取消。

---

### TASK-16：记录与评测

#### bag 新增话题

```text
/agt/map/semantic_markers
/agt/map/keepout_mask
/agt/coverage/path_raw
/agt/coverage/path_validated
/agt/coverage/path_repaired
/agt/coverage/collision_poses
/agt/coverage/status
/agt/coverage/validation_report
/global_costmap/costmap
/local_costmap/costmap
```

#### 最低评测指标

```text
coverage_rate
overlap_rate
missed_area
total_path_length
work_path_length
non_work_path_length
turn_count
planning_time
validation_time
repair_time
collision_pose_count
repaired_segment_count
execution_success
```

#### 验收标准

- 同一输入多次运行指标可重复；
- 评测使用固定地图、固定参数和固定 commit；
- 报告记录 Git commit、ROS 版本、参数和地图哈希；
- 实验产物写入 `runtime/results/`。

---

## 7. 第一轮修改范围

第一轮只允许实现 Phase 0 和 Phase 1。

### 7.1 允许修改或新增

```text
docs/interfaces/semantic_map_schema.md

src/agt_ui_bridge/
├── CMakeLists.txt
├── package.xml
├── README.md
├── agt_ui_bridge/
│   ├── __init__.py
│   ├── map_transform.py
│   ├── semantic_model.py
│   ├── semantic_io.py
│   ├── semantic_validation.py
│   └── semantic_scene.py
├── scripts/
│   └── semantic_editor_qt5.py
├── launch/
│   └── semantic_editor.launch.py
├── config/
│   └── semantic_editor.yaml
└── test/
    ├── test_map_transform.py
    ├── test_semantic_io.py
    └── test_semantic_validation.py

tests/
├── test_semantic_map_contracts.py
└── test_vehicle_geometry_contracts.py

README.md
docs/migration/migration_matrix.md
```

### 7.2 第一轮禁止修改

```text
third_party/**
src/agt_mapping/**
src/agt_localization/**
src/agt_chassis/**
src/agt_safety/**
src/agt_navigation/config/nav2_bunker.yaml
src/agt_coverage_planning/**
src/agt_interfaces/**
nav_dependencies.repos
```

例外：

- TASK-00 若仅需修复 footprint 一致性，可修改明确列出的 Nav2 和 perception 配置；
- 不得顺带调参。

### 7.3 第一轮完成标准

必须完成以下演示：

```text
1. 启动语义编辑器；
2. 加载一张带非零 origin 的 Nav2 地图；
3. 绘制 field_boundary；
4. 绘制 exclusion_zone；
5. 设置 entry_pose；
6. 设置 work_direction；
7. 保存；
8. 退出；
9. 重新启动；
10. 重新加载；
11. 所有坐标和属性一致；
12. 非法 Polygon 被识别并禁止用于规划。
```

第一轮不得声称：

- 已接入 Fields2Cover；
- 已实现覆盖规划；
- 已实现 KeepoutFilter；
- 已实现路径修复；
- 已完成实车验收。

---

## 8. 测试矩阵

### 8.1 单元测试

| 测试 | 最低覆盖内容 |
|---|---|
| `test_map_transform.py` | Y 翻转、resolution、origin、origin yaw、边界 |
| `test_semantic_io.py` | GeoJSON 保存重载、原子保存、schema version |
| `test_semantic_validation.py` | 自交、多边形包含、ID 重复、入口位置 |
| `test_mask_rasterization.py` | Polygon 到 OccupancyGrid 对齐 |
| `test_f2c_request_conversion.cpp` | Polygon、hole、row、参数转换 |
| `test_coverage_path_resampling.cpp` | 距离与角度插值 |
| `test_footprint_collision.cpp` | 中间碰撞、角点碰撞、未知空间 |
| `test_path_segment_split.cpp` | SWATH 与 CONNECTION 分段 |
| `test_path_repair.cpp` | 仅连接段替换、swath 不变 |
| `test_interfaces.py` | Action 生成、导入和序列化 |

### 8.2 离线集成场景

必须固定四个合成场景：

```text
scene_01_empty_rectangle
scene_02_internal_obstacle
scene_03_narrow_headland
scene_04_greenhouse_rows
```

每个场景保存：

```text
map.yaml
map.pgm
semantic_map.geojson
coverage.yaml
expected_metrics.yaml
```

### 8.3 回归要求

每个阶段必须执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

colcon test
colcon test-result --verbose

python3 -m pytest -q \
  tests \
  src/agt_ui_bridge/test \
  src/agt_coverage_planning/test
```

不存在的测试目录可以在对应阶段未创建时从命令中移除，但不得用忽略失败的方式绕过测试。

---

## 9. Codex 执行协议

Codex 每次开始工作前必须：

1. 读取根目录 `AGENTS.md`；
2. 读取本文件；
3. 检查当前 Git branch、commit 和工作区状态；
4. 检查是否存在未提交修改；
5. 明确当前只实施哪个 TASK；
6. 列出本次允许修改文件；
7. 列出本次禁止修改文件。

每个 TASK 完成后必须输出：

```text
1. 修改摘要
2. 修改文件列表
3. 新增接口
4. 未修改的边界
5. 构建结果
6. 测试结果
7. 已知限制
8. 下一任务
```

### 9.1 失败处理

遇到以下情况时必须停止当前任务并报告，不得自行扩大范围：

- 依赖版本不兼容；
- 地图格式与规范冲突；
- 需要修改第三方源码；
- 需要修改安全链；
- 需要修改定位 TF；
- 需要改变已验证车辆尺寸；
- 测试无法在当前环境运行；
- 工作区存在来源不明的未提交修改。

允许给出最小修复建议，但不得越过当前任务边界直接实施。

### 9.2 提交粒度

推荐一项任务一个提交，提交示例：

```text
feat(ui): add semantic map data model and GeoJSON I/O
feat(ui): add Qt5 semantic annotation MVP
feat(map): publish semantic keepout mask
feat(coverage): adapt semantic fields to opennav coverage
feat(coverage): validate paths with vehicle footprint
feat(coverage): repair invalid connection segments
feat(interfaces): add coverage task action
```

禁止使用：

```text
update
fix stuff
complete project
misc changes
```

---

## 10. 各阶段 Definition of Done

### Phase 0 DoD

- footprint 单一来源明确；
- 语义 schema 文档完成；
- 示例文件和契约测试完成。

### Phase 1 DoD

- Qt5 标注可用；
- GeoJSON/YAML 可可靠保存和加载；
- 坐标转换测试通过；
- 合法性检查可阻断错误任务；
- 未修改第三方 Qt 项目。

### Phase 2 DoD

- 语义服务器稳定发布；
- Keepout mask 与地图严格对齐；
- Nav2 不规划穿越禁行区；
- 原点到点导航回归通过。

### Phase 3 DoD

- Polygon 和 annotated rows 两种模式至少各有一个成功案例；
- 能输出原始 Path 和 PathComponents；
- 依赖版本固定且可复现。

### Phase 4 DoD

- footprint 验证不会漏掉中间碰撞；
- 路径区分作业行和连接段；
- 修复只改变无效连接段；
- 修复后重新验证通过。

### Phase 5 DoD

- Action 可启动、反馈、取消和返回结果；
- 总控按依赖顺序启动；
- 任一前置失败时不允许运动；
- bag 和评测报告完整；
- 离线合成场景全部通过；
- 实车验收另行执行，不得用离线通过替代。

---

## 11. 最终系统验收标准

### 11.1 功能验收

- 可加载基础 OccupancyGrid；
- 可完成语义标注；
- 可保存并重载语义任务；
- 可生成 Keepout mask；
- 可生成 Fields2Cover 覆盖路径；
- 可检测 footprint 碰撞；
- 可识别无效连接段；
- 可局部修复连接段；
- 可通过 Nav2 执行；
- 可记录完整任务过程。

### 11.2 数据正确性验收

- 语义坐标与地图对齐误差不超过一个栅格；
- 所有 Path 使用 `map` frame；
- 基础地图哈希验证有效；
- footprint 在所有模块中一致；
- 修复前后作业行坐标不变；
- 任务结果可由相同输入重复生成。

### 11.3 安全验收

- 未定位时禁止启动覆盖任务；
- 语义地图非法时禁止启动；
- 路径验证失败且 repair 失败时禁止执行；
- Collision Monitor 保持启用；
- `agt_safety` 保持在速度链中；
- 软件异常退出时速度归零；
- 实车测试必须保留硬件急停和手动接管。

### 11.4 工程验收

- 新节点有 README；
- 参数有 YAML；
- launch 可独立启动；
- 无硬编码用户名和绝对工作区路径；
- 无重复 TF；
- 无未固定外部依赖；
- 测试可运行；
- 迁移矩阵已更新；
- runtime 产物未提交 Git。

---

## 12. 推荐实施顺序

严格按以下顺序执行：

```text
TASK-00
→ TASK-01
→ TASK-02
→ TASK-03
→ TASK-04
→ Phase 1 验收
→ TASK-05
→ TASK-06
→ TASK-07
→ Phase 2 验收
→ TASK-08
→ TASK-09
→ Phase 3 验收
→ TASK-10
→ TASK-11
→ TASK-12
→ Phase 4 验收
→ TASK-13
→ TASK-14
→ TASK-15
→ TASK-16
→ 最终离线验收
→ 实车低速验收
```

禁止并行实施以下组合：

```text
Qt UI + Fields2Cover
Fields2Cover + 路径修复
路径修复 + 实车执行
接口重构 + 总控重构
车辆 footprint 修改 + Nav2 调参
```

---

## 13. 第一条 Codex 执行指令

Codex 首次读取本文档后，只执行：

```text
TASK-00：统一车辆 footprint 数据源
TASK-01：定义语义地图规范
```

不得开始 Qt 绘图、Fields2Cover、KeepoutFilter 或路径修复。

完成后提交：

```text
- footprint 一致性检查
- semantic_map_schema.md
- 合法与非法示例
- 契约测试
- README 和 migration_matrix 更新
```

并等待下一条明确任务指令。
