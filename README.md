# moke-nav

基于 ROS 2 Jazzy + Gazebo Harmonic 的差速机器人自主导航工作区。
目标：走通 **URDF 建模 → Gazebo 仿真 → SLAM 建图 → Nav2 导航 → 调优评测** 的完整闭环。

> 当前状态：**框架已就绪，实现待填**。本工作区只包含目录结构、包元数据与文档骨架，
> 尚未写入 URDF / launch / 参数等实现内容。每个目录的 `README.md` 说明了它要放什么。

完整技术方案、阶段任务、踩坑清单见 [`docs/nav2_项目规划.md`](docs/nav2_项目规划.md)。

## 环境

| 组件   | 版本                                                    |
| ------ | ------------------------------------------------------- |
| ROS 2  | Jazzy（`/opt/ros/jazzy`）                             |
| Nav2   | 34 包，默认 MPPI + NavFn                                |
| Gazebo | Harmonic（gz-sim 8.15，经 `ros-jazzy-gz-sim-vendor`） |
| 桥接   | ros_gz 1.0.24                                           |
| SLAM   | slam_toolbox（主）/ cartographer_ros（对比）            |

> Jazzy 不支持 Gazebo Classic，所有 `gazebo_ros` 教程不适用。

## 目录结构

```text
moke-nav_ws/
├── src/
│   ├── nav_bot_description/   # 模型层：URDF/Xacro、meshes、RViz      → 阶段 1
│   ├── nav_bot_gazebo/        # 仿真层：world、模型、bridge、spawn    → 阶段 2
│   ├── nav_bot_navigation/    # 导航层：Nav2 参数、地图、行为树        → 阶段 3-5
│   ├── nav_bot_bringup/       # 装配层：一键启动全链路                 → 阶段 4
│   └── nav_bot_tools/         # 工具层：评测脚本、自研插件             → 阶段 5
├── docs/                      # 规划文档、架构说明、实验日志、验收清单
├── experiments/               # 实验原始数据：评测 CSV、图表（阶段 5）
├── bags/                      # ros2 bag 录制（不入库）
└── outputs/                   # 导出的图 / 报告（不入库）
```

依赖方向严格单向：`bringup → gazebo → description`，`bringup → navigation`。
上游包不得反向依赖下游，`bringup` 不承载任何参数与模型。

## 常用命令

```bash
# 首次编译
source /opt/ros/jazzy/setup.bash
export COLCON_DEFAULTS_FILE=$PWD/.colcon/defaults.yaml   # 见下方"构建环境"
colcon build
source install/setup.bash

# 只编一个包
colcon build --packages-select nav_bot_description

# 清理后重编
rm -rf build install log && colcon build

# 依赖自检
rosdep check --from-paths src --ignore-src
```

### 构建环境（必读）

本机 PATH 首位是受管 Python 3.13，其中没有 `catkin_pkg`，直接 `colcon build` 会在
`package_xml_2_cmake.py` 处报 `ModuleNotFoundError: No module named 'catkin_pkg'`。

`.colcon/defaults.yaml` 已经存了修正项（`-DPython3_EXECUTABLE=/usr/bin/python3` + `--symlink-install`），
启用方式二选一：

```bash
# 方式 A：每个新终端导出一次（可写进 ~/.bashrc）
export COLCON_DEFAULTS_FILE=/home/moke/moke-nav_ws/.colcon/defaults.yaml

# 方式 B：不用 defaults 文件，直接传参
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

验证框架完好：`rosdep check` 应输出 *All system dependencies have been satisfied*，
`ros2 pkg list | grep nav_bot` 应列出 5 个包。

## 阶段进度

- [X] **阶段 1** 建模：`check_urdf` 通过、TF 树完整、轮子转向正确
- [X] **阶段 2** 仿真：键盘遥控可行驶，`/scan` `/odom` `/tf` `/clock` 正常
- [ ] **阶段 3** 建图：10×10 m 地图闭合、无重影
- [ ] **阶段 4** 导航：AMCL 收敛 < 3 s，10 目标 ≥ 8 成功
- [ ] **阶段 5** 评测：批量脚本 + CSV + 参数敏感性曲线
- [ ] **阶段 6** 真机（可选）：`ros2_control` 替换 gz 插件

## 工作约定

1. **一次只改一个参数**，改动记录进 `docs/experiments.md`。
2. **所有 launch 暴露** `use_sim_time` / `world` / `map` / `params_file`，不写死路径。
3. **参数全部外置 yaml**，launch 只做引用与传参。
4. 仿真链路任何节点都必须统一 `use_sim_time:=true`，这是 TF 时间戳错乱的唯一根源。
5. 出问题按 **TF → 话题频率 → costmap → 参数** 的顺序查，反了会浪费几天。
6. 关键实验前录 bag，便于回放复现。
