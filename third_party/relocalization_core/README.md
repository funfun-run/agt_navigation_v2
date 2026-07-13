# relocalization_core

`relocalization_core` 是一个纯 C++ 的重定位核心库，负责把“全局点云地图 + 当前扫描 + 初值”变成一次统一的配准结果。

它的设计目标不是直接服务某个固定 ROS2 节点，而是把重定位逻辑从现有工程里拆出来，后续迁移到别的系统时只需要重写适配层，不需要改核心配准流程。

## 当前定位

当前库已经实现：

- 统一的重定位请求/结果类型
- 地图加载与体素下采样
- 扫描预处理
- `ICP` 与 `NDT` 双后端封装
- 统一的 `Relocalizer` 编排入口

当前库不负责：

- ROS topic 订阅/发布
- TF 查询与发布
- `/initialpose` 触发逻辑
- 节点生命周期管理

这些能力都放在上层 `relocalization_ros2` 里。

## 主要接口

核心入口位于：

- [include/relocalization_core/relocalizer.hpp](/home/yangxuan/ros2_ws/src/relocalization_module/relocalization_core/include/relocalization_core/relocalizer.hpp:1)

当前对外主接口包括：

- `setGlobalMap(...)`
- `setGlobalMapFromPcd(...)`
- `setConfig(...)`
- `relocalize(...)`
- `latestDebugInfo()`

返回结果统一通过：

- `RelocalizationResult`
- `RelocalizationDebugInfo`

公共类型定义位于：

- [include/relocalization_core/types.hpp](/home/yangxuan/ros2_ws/src/relocalization_module/relocalization_core/include/relocalization_core/types.hpp:1)
- [include/relocalization_core/config.hpp](/home/yangxuan/ros2_ws/src/relocalization_module/relocalization_core/include/relocalization_core/config.hpp:1)

## 当前后端

首版已接入两个后端：

- `ICP`
- `NDT`

对应实现位于：

- [src/backends/icp_backend.cpp](/home/yangxuan/ros2_ws/src/relocalization_module/relocalization_core/src/backends/icp_backend.cpp:1)
- [src/backends/ndt_backend.cpp](/home/yangxuan/ros2_ws/src/relocalization_module/relocalization_core/src/backends/ndt_backend.cpp:1)

其中：

- `ICP` 当前在合成回归测试中结果稳定
- `NDT` 对点云分布和参数更敏感，尤其在规则、重复、近对称点云上更容易掉到局部最优

这不是当前接口语义错误，而是 `NDT` 本身的优化特性。为避免把“测试数据对称性”误判成“实现 bug”，当前回归测试已经改成使用非对称点云。

## 当前测试状态

核心测试位于：

- [test/test_relocalizer.cpp](/home/yangxuan/ros2_ws/src/relocalization_module/relocalization_core/test/test_relocalizer.cpp:1)

当前已验证：

- 地图未加载时返回 `map_not_ready`
- 扫描点数不足时返回 `scan_too_small`
- `ICP` 后端可在回归测试中恢复期望位姿
- `NDT` 后端在非对称测试点云上可通过回归测试

当前还观察到：

- `ndt_omp` 的 `KDTREE` 搜索方式在很小 `resolution` 下可能出现异常，不适合首版默认参数
- `DIRECT7` 速度更好，但更依赖点云结构和初值质量

## 编译与测试

编译：

```bash
cd /home/yangxuan/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select relocalization_core
```

单测：

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
cd /home/yangxuan/ros2_ws/build/relocalization_core
ctest --output-on-failure -R test_relocalizer
```

## 当前边界说明

这个包目前已经完成“核心能力实现 + 单测回归”这一层，但还没有接入 `mid360_nav_demo` 的现有导航主链。

也就是说，现在它是：

- 已实现
- 已能单独编译和测试
- 尚未替换当前 `icp_relocalizer_node`

后续如果要正式接入，再单独开一条线处理 launch、参数兼容、TF 语义对接和回归联调。
