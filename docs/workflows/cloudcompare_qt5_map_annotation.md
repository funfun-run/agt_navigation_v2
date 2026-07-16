# CloudCompare 栅格到 Qt5 可标注导航地图

更新时间：2026-07-16。

## 目标

把 `runtime/maps/from cloudcompare/` 中的 CloudCompare 导出图整理成：

- 可被 Nav2 `map_server` 加载的 `PNG/YAML`；
- 可被 `agt_ui_bridge` Qt5 地图工具继续编辑；
- 可继续进入语义编辑器的 `runtime/maps/<map_id>/semantic/` 目录。

当前阶段先完成“可装载、可编辑、可留痕”的底图封装，不把 CloudCompare 观测图直接当作最终导航图。

## 输入与输出

输入示例：

- `runtime/maps/from cloudcompare/greenhouse_obstacle_height.png`
- `runtime/maps/from cloudcompare/greenhouse_aligned_full.pcd`

输出示例：

- `runtime/maps/greenhouse_cloudcompare/greenhouse_cloudcompare.png`
- `runtime/maps/greenhouse_cloudcompare/greenhouse_cloudcompare.yaml`
- `runtime/maps/greenhouse_cloudcompare/processing_record.yaml`
- `runtime/maps/greenhouse_cloudcompare/semantic/coverage.yaml`

## 一次性封装

先把 CloudCompare 栅格封装成运行时地图包：

```bash
cd ~/agt_navigation_v2

python3 tools/map_tools/create_cloudcompare_runtime_map.py \
  --source-image "runtime/maps/from cloudcompare/greenhouse_obstacle_height.png" \
  --source-pcd "runtime/maps/from cloudcompare/greenhouse_aligned_full.pcd" \
  --map-id greenhouse_cloudcompare \
  --resolution 0.05 \
  --origin-x 0.0 \
  --origin-y 0.0
```

说明：

- 这一步会把源图转成标准灰度 PNG，并生成可被 Nav2/Qt5 读取的 YAML。
- 当前仓库里还没有该图对应的 CloudCompare `min center x/y` 记录，因此默认先用占位原点生成底图。
- `processing_record.yaml` 会显式标记“缺少 Rasterize 原点元数据”，提醒后续补齐，不把占位值误当闭环真值。

如果已经从 CloudCompare 记录了 Rasterize 最小格中心，可直接写入：

```bash
python3 tools/map_tools/create_cloudcompare_runtime_map.py \
  --source-image "runtime/maps/from cloudcompare/greenhouse_obstacle_height.png" \
  --map-id greenhouse_cloudcompare \
  --resolution 0.05 \
  --origin-x -2.475 \
  --origin-y -30.475 \
  --min-center-x -2.45 \
  --min-center-y -30.45 \
  --overwrite
```

或继续使用现有脚本按 `min center` 计算 YAML：

```bash
python3 "tools/pcd2pgm yaml/generate_nav2_map_yaml.py" \
  --image "runtime/maps/greenhouse_cloudcompare/greenhouse_cloudcompare.png" \
  --resolution 0.05 \
  --min-center-x -2.45 \
  --min-center-y -30.45 \
  --output "runtime/maps/greenhouse_cloudcompare/greenhouse_cloudcompare.yaml"
```

## Qt5 二维底图编辑

对于“点云围成外框，框内空白可通行”的地面栅格，先按拓扑关系生成三值底图：

```bash
python3 tools/map_tools/prepare_trinary_nav_map.py \
  --input runtime/maps/greenhouse_ground/greenhouse_ground_observed.png \
  --output runtime/maps/greenhouse_ground/greenhouse_ground.png \
  --unknown-value 105 \
  --unknown-margin 4 \
  --classification point-topology \
  --closure-size 13
```

其中有点云像素为障碍，闭合外框内部的无点区域为自由，图像边缘连通区域为未知。`closure-size` 只修补外框的栅格断口，不会加粗输出障碍。

用轻量编辑器把 CloudCompare 观测图刷成真正的导航底图：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agt_ui_bridge map_editor.launch.py
```

打开后执行：

1. 点击“加载地图”。
2. 选择 `runtime/maps/greenhouse_cloudcompare/greenhouse_cloudcompare.yaml`。
3. 使用“障碍 / 自由 / 未知”画笔，把观测栅格整理成真正的通行地图。
4. 点击“保存地图”，覆盖保存回 `runtime/maps/greenhouse_cloudcompare/greenhouse_cloudcompare.yaml`。

建议：

- 把确定通行的行间区域刷成 `free`。
- 把墙体、立柱、围挡、温室结构刷成 `occupied`。
- 把未观测或暂不可信区域保留为 `unknown`。

## Qt5 语义标注

底图可用后，再进入项目自有语义编辑器：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agt_ui_bridge semantic_editor.launch.py \
  map:="runtime/maps/greenhouse_cloudcompare/greenhouse_cloudcompare.yaml" \
  platform_profile:="profiles/platforms/bunker.yaml"
```

语义结果保存到：

- `runtime/maps/greenhouse_cloudcompare/semantic/semantic_map.geojson`
- `runtime/maps/greenhouse_cloudcompare/semantic/coverage.yaml`

## 工程约束

- CloudCompare 观测图不是最终导航图，必须经过人工检查和 Qt5 编辑。
- 未记录 `Rasterize min center x/y` 前，只能把该地图当作“可编辑底图”，不能宣称已与三维重定位地图严格共原点。
- 正式闭环前，`processing_record.yaml` 中的 `pending_action` 必须清空。
- 语义标注文件必须与基础 PNG/YAML 分离，仍保持 `runtime/maps/<map_id>/semantic/` 结构。
