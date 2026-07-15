# 覆盖规划依赖复现流程

本文固定 TASK-08 的外部覆盖规划依赖。外部源码必须通过 `nav_dependencies.repos` 导入独立
工作区，不复制到 `agt_coverage_planning`，也不得依赖本机旧 ROS 工作区的 overlay。

## 版本与许可证

| 依赖 | 上游版本 | 固定提交 | 许可证 | Humble 用途 |
| --- | --- | --- | --- | --- |
| `opennav_coverage` | `humble-v2` | `f413d0da7b4e52249b9abdb9d1fec7cef0238449` | Apache-2.0 | Coverage/Row Coverage Server 与消息 |
| `Fields2Cover` | `v2.0.0` | `3613525c241538fa9fd9df3e1209ae8184627958` | BSD-3-Clause | 覆盖几何与路径算法 |
| `steering_functions` | F2C 固定提交 | `13e3f5658144b3832fb1eb31a0e2f5a3cbf57db9` | BSD-3-Clause | F2C 转向曲线依赖 |
| `matplotplusplus` | F2C 固定提交 | `5d01eb3695b07634a2b6642fd423740dea9b026c` | MIT | F2C 可视化编译依赖 |
| `nlohmann/json` | F2C 固定提交 | `4424a0fcc1c7fa640b5c87d26776d99150dacd10` | MIT | F2C JSON 依赖 |

`opennav_coverage humble-v2` 的 README 和 `.github/deps.repos` 均指定 Fields2Cover
`v2.0.0`，其 CI 使用 Ubuntu 22.04 和 ROS 2 Humble。普通 `humble` 分支对应 F2C 1.2.1，
不可与本合同混用。

后三项是 Fields2Cover `v2.0.0` 在 `cmake/F2CUtils.cmake` 中声明的精确提交。清单显式导入
这些传递源码，避免 `FetchContent` 在构建过程中临时访问 GitHub。

## 系统依赖

首次安装使用 ROS 2 Humble 已配置的软件源：

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-vcstool python3-rosdep \
  lcov libgeos++-dev swig ros-humble-ortools-vendor
```

其余依赖由 rosdep 根据锁定源码中的 `package.xml` 安装。不要手工安装另一份 OR-Tools，
也不要 source 旧工作区来掩盖缺失依赖。

## 全新工作区导入

在本仓库根目录设置一个明确的外部工作区路径：

```bash
source /opt/ros/humble/setup.bash

COVERAGE_WS=/path/to/agt_coverage_ws
mkdir -p "$COVERAGE_WS/src"
vcs import "$COVERAGE_WS/src" < nav_dependencies.repos
vcs export --exact "$COVERAGE_WS/src"
```

`vcs export --exact` 输出必须全部为 40 位提交 SHA，且覆盖依赖分别等于本文表格中的提交。
Jammy 自带 vcstool 的 `vcs validate` 对完整 commit SHA 存在 `version_type` 异常，因此不把
该命令作为验收项；实际 `vcs import` 成功和逐仓库 `vcs export --exact` 是权威检查。

清单还包含项目其他已固定源码依赖；已 vendor 到本仓库的算法不重复列入。TASK-08 的
rosdep/build 只检查覆盖规划两个源码目录，避免无关第三方包的既有 manifest 问题污染本任务。

## rosdep 与最小构建

只 source `/opt/ros/humble`，从而验证没有隐式旧 overlay：

```bash
source /opt/ros/humble/setup.bash
cd "$COVERAGE_WS"

rosdep install --from-paths \
    src/fields2cover \
    src/opennav_coverage \
  --ignore-src \
  --rosdistro humble -r -y

colcon build --symlink-install \
  --packages-up-to \
    fields2cover \
    opennav_coverage_msgs \
    opennav_coverage \
    opennav_row_coverage \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DBUILD_PYTHON=OFF \
    -DBUILD_TUTORIALS=OFF \
    -DUSE_ORTOOLS_VENDOR=ON \
    -DFETCHCONTENT_SOURCE_DIR_STEERING_FUNCTIONS="$COVERAGE_WS/src/fields2cover_steering_functions" \
    -DFETCHCONTENT_SOURCE_DIR_MATPLOT="$COVERAGE_WS/src/fields2cover_matplotplusplus" \
    -DFETCHCONTENT_SOURCE_DIR_JSON="$COVERAGE_WS/src/fields2cover_json"
```

`USE_ORTOOLS_VENDOR=ON` 强制使用 ROS vendor 包，避免 F2C 的 fallback 在 CMake 阶段从网络
下载 OR-Tools；三个 `FETCHCONTENT_SOURCE_DIR_*` 参数强制使用清单导入的锁定源码。完成
`vcs import` 和 `rosdep install` 后，`colcon build` 不再需要网络。Python 绑定和 tutorials
不属于 TASK-09 的最小运行链。

## 安装核验

```bash
source "$COVERAGE_WS/install/setup.bash"
test -f "$COVERAGE_WS/install/fields2cover/lib/cmake/Fields2Cover/Fields2CoverConfig.cmake"
ros2 pkg prefix opennav_coverage_msgs
ros2 pkg prefix opennav_coverage
ros2 pkg prefix opennav_row_coverage
ros2 pkg executables opennav_coverage
ros2 pkg executables opennav_row_coverage
ros2 interface show opennav_coverage_msgs/action/ComputeCoveragePath
```

离线预览至少要求上述三个 `ros2 pkg prefix` 命令全部成功。仅能找到
`opennav_coverage_msgs` 表示工作区只生成了接口，不能运行 Coverage Server；此时
`coverage_preview.launch.py` 尚不能生成路线，必须继续完成本节的 `--packages-up-to` 构建。

Fields2Cover 是普通 CMake/colcon 包，不在 ament resource index 中，因此不能用
`ros2 pkg prefix fields2cover` 判断安装结果。构建时 colcon 会把 Fields2Cover 专用参数也传给
其他包；其他包报告 `Manually-specified variables were not used` 属于可忽略警告。

TASK-09 只使用 Coverage Server 和 `ComputeCoveragePath`。Humble 第一版不要求 Coverage
Navigator、BT plugin 或 demo；它们不能成为语义适配器的隐式依赖。
