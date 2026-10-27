# nav_bot_bringup — 装配层

只做"把上游包按顺序拉起来"这一件事，不放任何参数和模型。判断标准：删掉这个包，
其他包依然能各自单独启动；有了它，一条命令走完全链路。

## 目录职责

| 路径 | 放什么 | 状态 |
|---|---|---|
| `launch/bringup.launch.py` | 仿真 + 定位 + 导航 + RViz 一键启动 | 待建 |
| `launch/sim_only.launch.py` | 只起仿真（调试阶段用） | 待建 |
| `config/nav2.rviz` | 全链路 RViz 布局（TF、map、costmap、path、scan） | 待建 |

## 必须暴露的 launch 参数

所有 launch 用 `DeclareLaunchArgument` 暴露，默认值面向仿真，切换真机时只改参数：

- `use_sim_time`（默认 `true`）
- `world`（默认 `room_10x10.sdf`）
- `map`（默认 `maps/room.yaml`）
- `params_file`（默认 `nav2_params.yaml`）
- `autostart`（默认 `true`）

## 依赖方向（不可反转）

```text
nav_bot_bringup  →  nav_bot_gazebo  →  nav_bot_description
       ↓
nav_bot_navigation
```

## 验收

```bash
ros2 launch nav_bot_bringup bringup.launch.py
```
一条命令后 RViz 中机器人可见、地图加载、TF 完整、可直接点目标点。
