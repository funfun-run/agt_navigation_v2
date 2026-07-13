# bag_tools

bag 裁剪、合并、重命名和标准测试集准备工具。

旧 MID360 bag 若记录为带 `timestamp/line/tag` 字段的 PointCloud2，可转换为新
FAST-LIVO2 所需的 CustomMsg：

```bash
source install/setup.bash
python3 tools/bag_tools/convert_mid360_pointcloud2_to_custom.py \
  runtime/rosbag/mid360_mapping_20260603_195044 \
  runtime/rosbag/mid360_mapping_custom_smoke \
  --max-lidar-messages 10
```

输出仅包含 `/agt/sensors/lidar/custom` 和 `/agt/sensors/imu/data`，不修改原 bag。
