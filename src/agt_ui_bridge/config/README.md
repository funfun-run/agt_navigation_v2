# config

UI bridge 的版本化默认配置：

- `map_io.yaml`：基础 OccupancyGrid 加载与保存接口；
- `ros_qt5_gui_app.json`：上游 Qt5 主界面的 V2 topic 映射；
- `semantic_editor.yaml`：独立语义编辑器的显示、作业参数和显式边界净距，不保存车辆几何；
- `semantic_map_server.yaml`：语义服务器输入、基础地图 topic、field 外部策略和 mask 数值；
- `semantic_schema.yaml`：农业语义地图与覆盖任务文件的 1.0 机器可读合同。

运行中产生的 GUI 配置和语义任务文件写入 `runtime/`，不得覆盖这里的默认合同。
