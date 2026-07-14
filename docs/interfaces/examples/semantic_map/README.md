# 语义地图合同样例

`semantic/semantic_map.geojson` 与 `semantic/coverage.yaml` 构成合法的 polygon 1.0 示例；
`annotated_rows/` 包含两条已标注作物行的合法任务。`invalid/` 包含三类必须拒绝的输入：
重复 Feature ID、错误 frame 和不支持的 schema version。

这些文件只用于文档和无 ROS 契约测试。实际使用时应复制数据结构而不是这些占位几何，并由
编辑器写入 `runtime/maps/<map_id>/semantic/`。
