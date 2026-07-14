# 语义地图服务器接口

`agt_semantic_map_server` 属于 `agt_ui_bridge`，负责把经过合同与 Shapely 校验的本地语义任务
转换为 ROS 2 标准消息。它不发布 TF，不修改基础地图，也不负责覆盖规划或车辆控制。

## 输入与参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `semantic_map` | 空 | `semantic_map.geojson` 路径；同目录必须有 `coverage.yaml` |
| `platform_profile` | 空 | canonical `profiles/platforms/<platform>.yaml` 路径 |
| `base_map_topic` | `/agt/map/global_occupancy` | transient-local 基础 OccupancyGrid |
| `auto_load` | `true` | 基础地图到达后自动加载参数指定任务 |
| `minimum_boundary_clearance` | `0.0` | navigation footprint 之外的显式额外净距 |
| `outside_field_is_keepout` | `true` | 是否将所有 enabled field 外部设为禁行 |
| `mask_free_value` | `0` | mask 可通行值，范围 0~100 |
| `mask_occupied_value` | `100` | mask 禁行值，范围 0~100 |

服务器要求基础 OccupancyGrid 的 frame、resolution、width、height、origin 和 origin yaw 与
`coverage.yaml` 指向的 Nav2 地图完全一致。文件哈希、语义几何、平台 profile 或地图元数据不一致
时拒绝候选任务。

## 发布接口

以下 topic 全部使用 `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)`：

| Topic | 类型 | TASK-07 行为 |
| --- | --- | --- |
| `/agt/map/semantic_markers` | `visualization_msgs/msg/MarkerArray` | 发布 enabled 语义对象，首项为 `DELETEALL` |
| `/agt/map/keepout_mask` | `nav_msgs/msg/OccupancyGrid` | 栅格化 enabled exclusion/keepout 和可配置 field 外部 |
| `/agt/map/semantic_status` | `diagnostic_msgs/msg/DiagnosticArray` | 发布加载状态、路径、active map ID 和 mask mode |

mask 使用基础 OccupancyGrid 的 frame、resolution、width、height、origin 和 origin yaw，不读取
或修改源 PGM 像素。状态中的 `mask_mode` 为 `semantic_keepout_task06`。Polygon 边界离散误差
不超过一个栅格。Nav2 通过独立 FilterInfo topic 消费 mask，基础 OccupancyGrid 保持只读。

## 服务接口

| 服务 | 类型 | 语义 |
| --- | --- | --- |
| `/agt/map/semantic/load` | `nav2_msgs/srv/LoadMap` | `map_url` 接收普通路径或 `file://` GeoJSON URL；成功时 response map 为基础地图 |
| `/agt/map/semantic/reload` | `std_srvs/srv/Trigger` | 重载当前 active/requested 路径 |
| `/agt/map/semantic/validate` | `std_srvs/srv/Trigger` | 验证当前路径但不切换 active 产品 |

## 状态与事务

| 状态 | 含义 |
| --- | --- |
| `UNLOADED` | 未配置任务或仍在等待基础 OccupancyGrid |
| `LOADED` | 文件、几何、profile 与底图一致，候选已原子切换 |
| `HASH_MISMATCH` | 基础 Nav2 YAML SHA256 与 coverage 不一致 |
| `GEOMETRY_INVALID` | schema、Shapely、footprint、profile 或地图元数据非法 |
| `RASTERIZATION_FAILED` | marker/mask 产品构建失败 |
| `LOAD_FAILED` | 路径、文件读取或解析失败 |

加载流程先在候选对象中完成读取、完整验证和消息构建，全部成功后才替换 active task 并发布。
失败只更新状态，上一份有效 markers/mask 保持 latched，不发布清空消息。

## Nav2 Keepout 消费规则

`agt_navigation` 的 Costmap Filter Info Server 发布 `/agt/map/keepout_filter_info`，其类型为
keepout `0`、mask topic 为 `/agt/map/keepout_mask`、`base=0.0`、`multiplier=1.0`。global
costmap 使用 `StaticLayer -> KeepoutFilter -> InflationLayer`，使 footprint 周边也避开禁行边界。

Humble 插件在尚未收到 FilterInfo 或 mask 时会告警并 fail-open，继续按基础地图规划；因此
实车运动前必须确认本服务器状态为 `LOADED` 且两个 keepout topic 均可读取。服务器退出后
Nav2 保留最后收到的 mask。显式关闭语义层使用
`/global_costmap/keepout_filter/toggle_filter`，不会修改基础地图；成功加载新任务会原子替换
mask，失败加载继续使用上一份有效 mask。
