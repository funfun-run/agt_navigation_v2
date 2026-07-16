# tools

这里放离线和诊断工具，不放长期运行的 ROS 2 节点。

当前预留子目录：
- `time_sync`
- `calibration`
- `bag_tools`
- `map_tools`
- `diagnostics`
- `dataset_tools`

其中 `map_tools/create_cloudcompare_runtime_map.py` 用于把 CloudCompare 栅格整理成
`runtime/maps/<map_id>/` 结构，便于 Qt5 地图编辑器和语义标注器继续处理。
