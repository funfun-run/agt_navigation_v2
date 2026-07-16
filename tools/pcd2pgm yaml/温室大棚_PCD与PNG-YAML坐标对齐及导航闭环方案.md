# 温室大棚 PCD 与 PNG/YAML 坐标对齐及导航闭环方案

**版本：** V1.0  
**当前目标：** 优先打通“完整 PCD 重定位 + 二维 PNG/YAML 全局规划”闭环；高程信息只保留，不在本阶段接入规划。  
**适用环境：** Ubuntu 22.04、ROS 2 Humble、Nav2、CloudCompare、现有地图标注工具。

---

## 1. 方案目标与边界

当前实验场所路面整体较平齐，短期不需要把砖路、行道之间的小幅高差直接加入全局规划代价。本阶段采用以下结构：

```text
完整且坐标已校正的 PCD
        │
        ├── 三维重定位地图
        │
        └── 裁剪得到二维地图源点云
                    │
                    └── CloudCompare Rasterize
                              │
                              ├── 三值 PNG + YAML：Nav2 全局规划
                              └── GeoTIFF：保留高程信息，暂不接入
```

本阶段最重要的工程要求是：

> 三维 PCD 重定位输出的机器人位姿，与二维 PNG/YAML 地图使用同一个 `map` 坐标系。

高程层作为独立数据保存，待二维导航闭环稳定后，再接入 `grid_map`、坡度、局部高差或自定义 Nav2 代价层。

---

## 2. 当前已经确定的点云处理方法

### 2.1 原始处理流程

1. 从完整原始 PCD 中按 Z 轴裁剪一段高度，暴露可用于校正的地面。
2. 使用 CloudCompare `Level` 工具，将地面校正为水平。
3. 找到并保存 `Level` 产生的刚体变换。
4. 将同一个变换应用到完整 PCD，得到完整校正点云。
5. 从完整校正点云中裁剪，只保留田垄、行道和固定障碍等导航相关区域。
6. 根据实际点密度判断是否去噪。
7. 当前点云较稀疏，为避免误删有效结构，不使用 SOR。
8. 使用 CloudCompare `Rasterize` 生成二维栅格。

### 2.2 当前 Rasterize 参数

```yaml
grid_step: 0.05
projection_direction: Z
cell_height: Maximum
empty_cells: Leave empty
sor_filter: disabled
```

参数含义：

- `Grid step = 0.05`：每个像素对应实际环境中的 `0.05 m × 0.05 m`。
- `Projection direction = Z`：把 XY 平面投影为二维地图。
- `Cell height = Maximum`：每个栅格使用该格内最高点，适合保留田垄、墙体、立柱等障碍结构。
- `Empty cells = Leave empty`：没有点的区域不自动插值，避免把未观测区域错误生成成自由空间或连续障碍。
- 不使用 SOR：当前点云较稀疏，SOR 可能删除真实的边缘、砖块、细杆和稀疏田垄点。

---

## 3. 最终应形成的数据产品

建议使用固定命名，避免后续把不同坐标系或不同处理阶段的文件混用。

```text
greenhouse_map/
├── 00_greenhouse_raw.pcd
├── 01_T_raw_to_aligned.txt
├── 02_greenhouse_aligned_full.pcd
├── 03_greenhouse_nav_source.pcd
├── 04_greenhouse_observed.png
├── 05_greenhouse_navigation.png
├── 05_greenhouse_navigation.yaml
├── 06_greenhouse_elevation.tif
├── 07_processing_record.yaml
├── 08_landmark_check.csv
└── 09_rviz_alignment.rviz
```

各文件用途：

| 文件 | 用途 |
|---|---|
| `00_greenhouse_raw.pcd` | 原始数据归档，禁止覆盖 |
| `01_T_raw_to_aligned.txt` | Level 变换矩阵 |
| `02_greenhouse_aligned_full.pcd` | 三维重定位使用的完整地图 |
| `03_greenhouse_nav_source.pcd` | Rasterize 使用的导航源点云 |
| `04_greenhouse_observed.png` | CloudCompare 导出的观测参考图 |
| `05_greenhouse_navigation.png` | 标注后的三值导航地图 |
| `05_greenhouse_navigation.yaml` | Nav2 地图元数据 |
| `06_greenhouse_elevation.tif` | 保留高程数值，暂不参与规划 |
| `07_processing_record.yaml` | 完整参数和版本记录 |
| `08_landmark_check.csv` | PCD 与 PNG 对齐验证点 |
| `09_rviz_alignment.rviz` | 对齐检查用 RViz 配置 |

---

# 4. 坐标对齐的核心约束

## 4.1 统一坐标系来源

必须满足：

```text
02_greenhouse_aligned_full.pcd
            │
            └── 只删除点，不再旋转和平移
                        ↓
03_greenhouse_nav_source.pcd
                        ↓
PNG/YAML
```

允许的后续处理：

- 按 Z 范围裁剪；
- 按 ROI 裁剪；
- 删除无关点；
- 可选去噪；
- 栅格化。

不允许在生成 `03_greenhouse_nav_source.pcd` 后继续做：

- Level；
- Apply Transformation；
- 手动旋转；
- 平移到原点；
- 缩放；
- 交换 X/Y 轴。

只要后续操作仅删除点，剩余点仍然保留完整 PCD 的原始 XY 坐标，二维地图就可以与三维重定位地图共用同一个 `map` 坐标系。

---

## 4.2 不需要把 PNG 转为 PGM

Nav2 的地图 YAML 可以直接引用 PNG：

```yaml
image: greenhouse_navigation.png
mode: trinary
resolution: 0.05
origin: [ORIGIN_X, ORIGIN_Y, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

因此当前建议：

> 直接使用无损、单通道或标准灰度的三值 PNG，不必额外转成 PGM。

PNG 更便于现有标注工具处理，也不会因为 JPEG 压缩产生边缘杂色。只有旧工具明确要求 PGM 时，才额外导出一份 PGM。

---

# 5. CloudCompare 栅格参数与 YAML 原点

## 5.1 必须记录的 Rasterize 信息

每次生成地图时，记录：

```yaml
grid_step: 0.05
grid_min_center_x: ...
grid_min_center_y: ...
grid_width_cells: ...
grid_height_cells: ...
projection_direction: Z
cell_height: Maximum
```

CloudCompare 的 Rasterize 网格采用 `pixel-is-area` 约定：

- `Min corner X/Y` 是左下角边界栅格的**中心坐标**；
- `Max corner X/Y` 是右上角边界栅格的**中心坐标**。

ROS 地图 YAML 的 `origin` 应表示地图栅格 `(0,0)` 左下角的实际坐标。因此：

```text
origin_x = grid_min_center_x - resolution / 2
origin_y = grid_min_center_y - resolution / 2
```

当前分辨率为 `0.05 m`：

```text
origin_x = grid_min_center_x - 0.025
origin_y = grid_min_center_y - 0.025
```

示例：

```text
CloudCompare Min corner X = -2.450
CloudCompare Min corner Y = -30.450
resolution = 0.05
```

则：

```yaml
origin: [-2.475, -30.475, 0.0]
```

### 重要规则

- 不要为了让数值好看而把原点改成整数。
- 不要用点云包围盒最小值直接代替 Rasterize 的 `Min corner`。
- 不要在标注后重新裁剪图片，否则原点不再成立。
- 如果图片被旋转，不能继续使用原来的 `origin` 和 `yaw=0`。

---

## 5.2 图像像素与 PCD 坐标的换算

定义：

```text
r  = resolution
x0 = YAML origin_x
y0 = YAML origin_y
W  = 图像宽度
H  = 图像高度
```

ROS 栅格坐标中，列 `u` 从左向右增加，行 `v` 从下向上增加：

```text
x = x0 + (u + 0.5) × r
y = y0 + (v + 0.5) × r
```

世界坐标转换为 ROS 栅格索引：

```text
u = floor((x - x0) / r)
v = floor((y - y0) / r)
```

常规 PNG 图像数组从左上角开始，图像行号 `i` 向下增加，因此：

```text
i = H - 1 - v
j = u
```

世界坐标直接转换到图像像素：

```text
j = floor((x - x0) / r)
i = H - 1 - floor((y - y0) / r)
```

这个公式可用于把 CloudCompare 中选取的 PCD 特征点投影到 PNG 上，验证原点、分辨率和上下方向。

---

# 6. PNG 标注阶段必须遵守的规则

你已经有地图标注方法，可以继续使用，但标注工具必须满足以下约束。

## 6.1 允许修改

只允许修改像素类别：

- 黑色：固定障碍；
- 白色：确认可通行；
- 灰色：未知或暂未确认。

推荐像素值：

```text
occupied = 0
unknown  = 205
free     = 254 或 255
```

## 6.2 禁止修改

禁止进行：

- 缩放图像；
- 改变画布尺寸；
- 裁剪边缘；
- 旋转图像；
- 水平或垂直翻转；
- 非整数像素平移；
- 使用透视变换；
- 导出为 JPEG；
- 自动添加透明边缘；
- 自动“适配内容”裁剪。

最终标注图必须与 CloudCompare 原始导出图保持：

```text
相同宽度 W
相同高度 H
相同像素网格
相同方向
```

## 6.3 推荐非破坏式标注

建议保留：

```text
底层：CloudCompare 参考图
上层：三值占据标注层
```

最终导出时只输出三值标注层，但工程文件保留参考图层，方便后续修订。

---

# 7. PNG 三值地图生成建议

CloudCompare 导出的彩色高度图不能直接灰度化作为导航地图，因为颜色是可视化色带，不是占据概率。

正确流程：

```text
CloudCompare 彩色 PNG
        ↓ 仅作为参考底图
现有标注工具
        ↓ 人工/半自动分类
三值灰度 PNG
        ↓
Nav2 map_server
```

三值分类原则：

- 确认存在固定结构：黑色障碍；
- 现场确认可通行或机器人实际走过：白色自由；
- 点云没有覆盖、遮挡严重、证据不足：灰色未知。

不要使用：

```text
没有点 = 自由
```

因为没有点可能表示遮挡、未采集或点云缺失。

---

# 8. YAML 推荐模板

```yaml
image: greenhouse_navigation.png
mode: trinary
resolution: 0.05
origin: [-2.475, -30.475, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

字段说明：

| 字段 | 当前建议 |
|---|---|
| `image` | 与 YAML 同目录的三值 PNG |
| `mode` | `trinary` |
| `resolution` | `0.05` |
| `origin` | 由 Rasterize Min corner 计算 |
| `yaw` | 图像未旋转时设 `0.0` |
| `negate` | 黑障碍、白自由时设 `0` |
| `occupied_thresh` | `0.65` |
| `free_thresh` | `0.25` |

图片不必与 YAML 使用绝对路径，建议放在同一目录并使用相对文件名。

---

# 9. 自动生成 YAML，避免手算错误

配套脚本：

```text
generate_nav2_map_yaml.py
```

使用示例：

```bash
python3 generate_nav2_map_yaml.py \
  --image greenhouse_navigation.png \
  --resolution 0.05 \
  --min-center-x -2.45 \
  --min-center-y -30.45 \
  --output greenhouse_navigation.yaml
```

脚本会：

1. 读取 PNG 宽度和高度；
2. 根据 CloudCompare Min corner 自动减去半个栅格；
3. 输出 Nav2 YAML；
4. 检查图片颜色通道；
5. 打印分辨率、实际地图尺寸和原点。

---

# 10. 三维重定位与 Nav2 的 TF 关系

推荐统一使用：

```text
map
 └── odom
      └── base_link
```

要求：

1. `02_greenhouse_aligned_full.pcd` 的数值坐标属于 `map`。
2. 点云重定位算法输出机器人在该 PCD 中的位姿。
3. 重定位模块最终提供 `map → odom`，或提供可被转换为该 TF 的定位结果。
4. Nav2 的二维地图 `/map` 也属于同一个 `map`。
5. 全局路径 `nav_msgs/Path` 中的位姿使用 `frame_id = map`。

需要避免：

- 一个节点发布 `map → odom`，另一个节点也发布同名 TF；
- PCD 话题写成 `camera_init`，二维地图写成 `map`，但没有固定变换；
- 为了显示方便，仅修改 PointCloud2 的 `header.frame_id`，却没有实际转换点坐标。

如果当前重定位系统固定使用 `camera_init`，可以二选一：

### 方案 A：推荐

将重定位地图和定位输出统一改为 `map`。

### 方案 B：兼容旧系统

保留：

```text
map → camera_init
```

但该静态变换必须与 Level 后 PCD 和 PNG/YAML 的真实关系一致，并且只能有一个明确的坐标变换来源。

---

# 11. 对齐验证流程

## 11.1 离线特征点检查

在 CloudCompare 中使用 Point Picking，选择至少 5 个稳定特征：

- 大棚左下角；
- 大棚右上角；
- 一根独立立柱；
- 一条田垄起点；
- 门口或横向道路边缘。

记录：

```csv
name,x,y,z
corner_1,...
corner_2,...
column_1,...
ridge_start,...
entrance,...
```

根据第 5.2 节公式把 `(x,y)` 转换为 PNG 像素，并检查是否落在相应结构上。

### 判定逻辑

- 所有点同方向偏移：YAML 原点错误；
- 偏差随距离增加：分辨率错误；
- 上下镜像：图像 Y 方向错误；
- 左右镜像：图像 X 方向错误；
- 发生旋转：图片被旋转，或 PCD 与 Rasterize 源点云不在同一坐标系；
- 固定约半个像素偏差：把 cell center 与 cell corner 混淆。

---

## 11.2 RViz 联合显示

同时发布：

```text
/map                         二维 OccupancyGrid
/aligned_map_cloud           完整校正 PCD
/TF                          map → odom → base_link
```

RViz 设置：

```text
Fixed Frame: map
```

添加：

- Map；
- PointCloud2；
- TF；
- RobotModel；
- Path。

优先检查：

1. 外墙与二维边界是否重合；
2. 田垄起点、末端是否重合；
3. 立柱是否位于二维障碍中心；
4. 大棚入口是否一致；
5. 机器人重定位位姿是否落在正确行道；
6. 全局路径是否在白色自由区内。

---

## 11.3 导航闭环测试顺序

不要一开始就测试长距离自动导航，按以下顺序推进：

### 阶段 1：地图加载

- PNG/YAML 可被 map_server 正常加载；
- `/map` 分辨率为 0.05；
- 地图宽高与 PNG 一致；
- 原点与记录一致。

### 阶段 2：静态对齐

- PCD 和 OccupancyGrid 在 RViz 中重合；
- 5 个以上固定特征无明显系统偏差。

### 阶段 3：重定位

- 机器人在不同起点重定位；
- 位姿落在真实行道内；
- 朝向与现场一致。

### 阶段 4：只规划不运动

- 发送目标点；
- 查看全局路径；
- 路径位于白色区域；
- 不穿越田垄或外墙。

### 阶段 5：低速短距离

- 直线路段 2～3 m；
- 横向通道；
- 田垄入口；
- 停止与恢复。

### 阶段 6：完整路线

- 多行道连续规划；
- 中途重定位；
- 局部障碍绕行；
- 规划与控制闭环。

---

# 12. 验收标准

## 12.1 地图坐标验收

建议第一阶段采用：

```yaml
landmark_count: 5
mean_alignment_error_m: <= 0.10
max_alignment_error_m: <= 0.15
systematic_rotation: none
axis_mirror: none
```

在 `0.05 m/cell` 下：

- 平均误差 0.10 m 相当于约 2 个像素；
- 最大误差 0.15 m 相当于约 3 个像素。

如果点云本身重影明显，应分别记录：

- 栅格坐标误差；
- 原始点云局部建图误差。

不要把 SLAM 重影误认为 YAML 原点错误。

## 12.2 导航验收

```yaml
map_load_success: true
localization_frame: map
global_path_frame: map
path_crosses_known_obstacle: false
short_route_success_rate: ">= 90%"
manual_emergency_stop_available: true
```

---

# 13. 常见错误诊断表

| 现象 | 更可能原因 | 修改方向 |
|---|---|---|
| 整张 PNG 相对 PCD 平移固定距离 | YAML origin 错误 | 重新检查 Min corner 和半栅格 |
| 越远偏差越大 | resolution 错误或图像被缩放 | 核对 0.05 和 W×H |
| 地图上下颠倒 | 标注工具或导出过程垂直翻转 | 恢复原方向，不先改 origin |
| 地图左右颠倒 | 水平翻转 | 恢复原方向 |
| 地图旋转固定角度 | 图片被旋转或 PCD 又做了变换 | 禁止后处理旋转 |
| PCD 与 PNG 对齐，但机器人位姿不对 | 重定位 TF 链错误 | 检查 map→odom |
| 局部对齐、局部不对齐 | PCD 建图重影或非刚性误差 | 先处理 SLAM 地图质量 |
| 路径穿过灰区 | planner 的 unknown 配置 | 检查 `track_unknown_space` 等参数 |
| 路径贴近田垄 | footprint、inflation 配置不足 | 调整 footprint 和 inflation |

---

# 14. 高程信息的保留与后续升级

本阶段只保留：

```text
06_greenhouse_elevation.tif
```

不直接加入全局路径规划。

后续升级顺序：

```text
GeoTIFF / elevation matrix
        ↓
grid_map elevation layer
        ↓
局部坡度 slope
        ↓
相邻栅格高差 step
        ↓
traversability
        ↓
Nav2 自定义 costmap layer 或速度限制
```

适合在以下条件满足后启动：

1. PNG/YAML 与 PCD 对齐稳定；
2. 重定位闭环稳定；
3. 二维 Nav2 导航成功率稳定；
4. 现场确实存在影响底盘通过性的坡度或台阶；
5. 有底盘离地间隙、最大爬坡角和最大可跨越高度数据。

当前路面相对平齐，不应为了保留高程而提前增加系统复杂度。

---

# 15. 处理记录模板

保存为 `07_processing_record.yaml`：

```yaml
project: greenhouse_pcd_nav_map
version: 1.0
date: YYYY-MM-DD

coordinate_frame:
  target_frame: map
  raw_cloud: 00_greenhouse_raw.pcd
  transform_matrix: 01_T_raw_to_aligned.txt
  aligned_full_cloud: 02_greenhouse_aligned_full.pcd
  localization_cloud: 02_greenhouse_aligned_full.pcd

navigation_cloud:
  source: 02_greenhouse_aligned_full.pcd
  output: 03_greenhouse_nav_source.pcd
  z_crop:
    enabled: true
    min: null
    max: null
  roi_crop: greenhouse_ridges_and_aisles
  sor:
    enabled: false
    reason: sparse_cloud_preserve_valid_points

rasterize:
  software: CloudCompare
  grid_step: 0.05
  projection_direction: Z
  cell_height: Maximum
  empty_cells: Leave empty
  min_center_x: null
  min_center_y: null
  width_cells: null
  height_cells: null

navigation_map:
  image: 05_greenhouse_navigation.png
  yaml: 05_greenhouse_navigation.yaml
  mode: trinary
  resolution: 0.05
  origin_x: null
  origin_y: null
  origin_yaw: 0.0
  negate: 0
  occupied_thresh: 0.65
  free_thresh: 0.25

annotation:
  tool: null
  image_resized: false
  canvas_cropped: false
  image_rotated: false
  image_flipped: false
  output_width: null
  output_height: null

elevation:
  retained: true
  file: 06_greenhouse_elevation.tif
  used_for_planning: false

validation:
  landmark_count: 0
  mean_error_m: null
  max_error_m: null
  rviz_overlay_passed: false
  localization_passed: false
  planning_only_passed: false
  short_motion_passed: false
```

---

# 16. 当前阶段的执行清单

```text
[ ] 确认完整重定位 PCD 已应用同一 Level 变换
[ ] 从完整校正 PCD 生成导航源点云
[ ] 记录 Rasterize Min corner、W、H、step
[ ] 导出 CloudCompare 参考 PNG
[ ] 使用现有工具生成三值 PNG
[ ] 确认三值 PNG 未缩放、裁剪、旋转和翻转
[ ] 用脚本自动生成 YAML
[ ] 选择至少 5 个 PCD 特征点进行像素投影检查
[ ] 在 RViz 同时显示 PCD 和 /map
[ ] 修正 origin 或图像方向
[ ] 验证重定位输出属于 map 坐标系
[ ] 只运行全局规划，不启动车辆
[ ] 低速短距离导航
[ ] 保存 GeoTIFF，暂不接入高程规划
```

---

# 17. 参考资料

1. CloudCompare Rasterize 官方文档：  
   https://www.cloudcompare.org/doc/wiki/index.php/Rasterize

2. ROS `nav_msgs/MapMetaData`：地图分辨率、宽高及 `(0,0)` 栅格的现实坐标。  
   https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/MapMetaData.html

3. ROS 2 Humble `nav_msgs/OccupancyGrid`：二维栅格数据的行主序和坐标方向。  
   https://docs.ros.org/en/humble/p/nav_msgs/msg/OccupancyGrid.html

4. Nav2 官方示例：PNG 与 YAML 可以直接作为二维占据地图。  
   https://docs.nav2.org/tutorials/docs/using_isaac_perceptor.html
