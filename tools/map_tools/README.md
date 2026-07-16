# map_tools

PCD 清洗、地图格式转换和地图元数据检查工具预留目录。

当前可用工具：

- `create_cloudcompare_runtime_map.py`
  - 把 CloudCompare 导出的 `png` 封装成 `runtime/maps/<map_id>/` 运行时地图包；
  - 自动生成 `PNG/YAML`、`processing_record.yaml` 和 `semantic/coverage.yaml`；
  - 适合先做 Qt5 二维底图编辑，再进入项目语义标注。
- `prepare_trinary_nav_map.py`
  - 把灰度观测图转换为仅含障碍、未知、自由的 Nav2 三值图；
  - `point-topology` 模式将有点云像素标为障碍、外框内部空白标为自由、外框外部标为未知；
  - 外框闭合只用于判断内外，不会加粗最终障碍。
