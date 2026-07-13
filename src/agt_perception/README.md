# agt_perception

职责：提供地面、障碍、聚类、语义和可通行性相关感知输出。

当前已落地局部几何障碍基线：

```bash
ros2 launch agt_perception local_obstacles.launch.py
```

节点把 `/agt/mapping/registered_points_lidar` 变换到 `base_footprint`，过滤地面、
高点、车体自身和量程外点，发布 `/agt/perception/obstacle_cloud`，供 Nav2 局部
代价地图与 Collision Monitor 共用。高度过滤依赖准确的 `base_link -> lidar_link`
外参；实车阶段仍需用斜坡、低矮障碍和负障碍数据评估，并决定是否升级为地面分割算法。
