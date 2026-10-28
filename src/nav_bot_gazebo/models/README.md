# models/ —— AWS RoboMaker Small House 家具模型

本目录内容**不是自研资产**，来源与许可见下，改动前请先读这一页。

## 来源

| 项 | 值 |
|---|---|
| 原始作者 | Amazon.com, Inc.（AWS RoboMaker） |
| 原始仓库 | <https://github.com/aws-robotics/aws-robomaker-small-house-world>（**已归档**，RoboMaker 于 2025-09-10 停止支持） |
| 实际取用 | fork `IntelligentRoboticsLabs/aws-robomaker-small-house-world` 的 **`ros2-jazzy` 分支**（SDF 1.11 + `gz-sim-*` 插件，Harmonic 原生） |
| 许可 | **MIT-0**（见 `AWS_LICENSE` / `LICENSE.aws-robomaker-small-house-world`），可商用、可修改、无需署名 |
| 取用日期 | 2026-10-28 |

## 本目录做了什么改动

1. **只保留 world 实际引用的模型**：上游 68 个 → 本地 **64 个**（去掉 `Pillow_01`、`Quilt_01`、`Sofa_01`、`SofaB_01`）。
2. **`worlds/small_house.world` 清理**：删除 99 处 `<pose frame=''>` 里的 `frame` 属性
   （SDF 1.11 规范中 `pose` 无此属性，gz 会刷一屏 `XML Attribute[frame] not defined` 警告）。
   几何、位姿数值、物理参数**未改动**。
3. 体积：约 75 MB（mesh 占绝大部分）。

## 已知无害告警

加载时仍会出现 1 条惯量告警：

```
Warning [Inertia.cpp:233] [Inertia::verifyMoment] Invalid entry for (1,1): 0.
```

来自某个静态家具模型的上游惯性张量为 0；静态体不参与动力学求解，可忽略，
不要在它上面做接触力相关的实验结论。

## 使用方式

```bash
export GZ_SIM_RESOURCE_PATH=<workspace>/src/nav_bot_gazebo/models
ros2 launch nav_bot_gazebo view_world.launch.py
```

`view_world.launch.py` 已自动把 `models` 与 `worlds` 拼进 `GZ_SIM_RESOURCE_PATH`，
一般情况下不需要手工导出。

## 尺寸速查（对 0.52 × 0.50 m 小车的影响）

- 房子主体约 **19 × 11 m**，配套地图 `nav_bot_navigation/maps/small_house.{pgm,yaml}` 为 25 × 25 m / 0.05 m 栅格。
- 门洞实测最窄约 **0.9 m**（地图 p5 通道 0.90 m），车宽 0.5 m 可通过但余量小 → `inflation_radius` 建议 0.25。
- 家具多为静态体、桌面高度 0.7 m 以上，雷达 0.28 m 扫描面主要打到家具体与桌腿：
  **建图时桌下空洞属正常现象**，不要误判为"漏扫"。
